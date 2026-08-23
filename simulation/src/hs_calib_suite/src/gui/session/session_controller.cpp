#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QRgb>
#include <QFileInfo>
#include <QMetaObject>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/aruco.hpp>

#include "hs_calib_suite/core/detectors/board_type_identifier.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"
#include "hs_calib_suite/core/detectors/circle_grid_detect_impl.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_capture_filter.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/detectors/aruco_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/aprilgrid_detector.hpp"
#include "hs_calib_suite/core/detectors/aruco_marker_detector.hpp"
#include "hs_calib_suite/core/targets/aruco_grid_target.hpp"
#include "hs_calib_suite/core/targets/aprilgrid_target.hpp"
#include "hs_calib_suite/core/detectors/charuco_detector.hpp"
#include "hs_calib_suite/core/targets/charuco_target.hpp"
#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"
#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/detectors/circle_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/asymmetric_circle_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/symmetric_circle_grid_detector.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/util/camera_models.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"
#include "hs_calib_suite/core/detectors/trihedral_charuco_detector.hpp"
#include "hs_calib_suite/core/detectors/trihedral_chess_detector.hpp"
#include "hs_calib_suite/core/targets/trihedral_target.hpp"
#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"

namespace hs_calib {
namespace gui {

namespace {

/// \brief BGR Mat 转 QImage（RGB888 拷贝）
QImage mat_bgr_to_qimage(const cv::Mat &bgr) {
  if (bgr.empty()) {
    return {};
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(
             rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
             QImage::Format_RGB888)
      .copy();
}

/// \brief YAML 双引号转义
std::string yaml_quote(const std::string &s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '\\' || c == '"') {
      o.push_back('\\');
    }
    if (c == '\n') {
      o += "\\n";
      continue;
    }
    o.push_back(c);
  }
  o.push_back('"');
  return o;
}

/// \brief QString 转 YAML 引号串
std::string yaml_quote_q(const QString &s) {
  return yaml_quote(s.toStdString());
}

struct DetectFp {
  double area_ratio = 0.0;
  double cx = 0.5;
  double cy = 0.5;
  double tilt_deg = 0.0;
};

struct DetectJobIn {
  cv::Mat bgr;
  bool trihedral = false;
  bool ros_mode = false;
  bool fast = false;
  int squares_x = 9;
  int squares_y = 6;
  double square_length_m = 0.025;
  std::map<std::string, std::string> solve_options;
  bool viz_corners = true;
  bool viz_hull = true;
  bool viz_conf = true;
  bool viz_aruco = true;
  int viz_marker_radius = 4;
  cv::Mat camera_matrix;  ///< 3x3 CV_64F，空则 guess_K
  cv::Mat dist_coeffs;    ///< 畸变，可空
  std::string camera_model = "brown_conrady";
  double camera_xi = 0.0;
  cv::Rect chess_roi;
  int chess_lost_frames = 0;
  bool has_chess_roi = false;
};

struct DetectJobOut {
  bool ok = false;
  QString error;
  QImage preview;
  core::Correspondence corr;
  double confidence = 0.0;
  int faces = 0;
  int width = 0;
  int height = 0;
  DetectFp fp;
  double aruco_reproj_px = -1.0;  ///< 单码/ArUco 轴位姿平均重投影误差；无效为 -1
  cv::Rect chess_roi;
  int chess_lost_frames = 0;
  bool has_chess_roi = false;
};

/// \brief 由角点包围盒生成视角指纹（面积/中心/倾角）
DetectFp fingerprint_from_corners_job(
    const std::vector<cv::Point2f> &corners, int width, int height) {
  DetectFp fp;
  if (corners.empty() || width <= 0 || height <= 0) {
    return fp;
  }
  cv::Rect box = cv::boundingRect(corners);
  fp.area_ratio =
      static_cast<double>(box.area()) / (static_cast<double>(width) * height);
  fp.cx = (box.x + box.width * 0.5) / static_cast<double>(width);
  fp.cy = (box.y + box.height * 0.5) / static_cast<double>(height);
  fp.tilt_deg = std::atan2(
                    static_cast<double>(box.height),
                    static_cast<double>(std::max(1, box.width))) *
                180.0 / CV_PI;
  return fp;
}

/// \brief 按角点数量与几何启发式估计检测置信度
double confidence_from_corners_job(
    const std::vector<cv::Point2f> &corners, int width, int height, int squares_x,
    int squares_y, bool trihedral, const std::map<std::string, std::string> &opts) {
  const int expect_face = squares_x * squares_y;
  if (corners.size() < 4 || expect_face <= 0 || width <= 0 || height <= 0) {
    return 0.0;
  }
  const DetectFp fp = fingerprint_from_corners_job(corners, width, height);
  double score = 0.0;
  const std::string target_type =
      opts.count("target") ? opts.at("target") : "chessboard";
  const std::string completeness =
      opts.count("detect_completeness") ? opts.at("detect_completeness") : "";
  const bool require_full = (completeness == "full" || completeness == "complete");
  const bool force_partial = (completeness == "partial");
  const bool aruco_loose =
      target_type == "aruco" || target_type == "aruco_single";
  const bool is_charuco =
      target_type == "charuco" || target_type == "trihedral_charuco";
  const bool is_aruco_grid = target_type == "aruco_grid";
  const bool is_aprilgrid = target_type == "aprilgrid";
  int expect_pts = expect_face;
  if (is_charuco) {
    expect_pts = std::max(4, (squares_x - 1) * (squares_y - 1));
  } else if (is_aruco_grid || is_aprilgrid) {
    expect_pts = std::max(4, expect_face * 4);
  }
  // 未指定时：三面/码类允许局部；棋盘/圆点仍要求整面（兼容标定会话）
  const bool partial_ok =
      !require_full &&
      (trihedral || is_charuco || is_aruco_grid || is_aprilgrid || aruco_loose ||
       (force_partial && target_type == "chessboard"));
  if (partial_ok) {
    if (corners.size() < (aruco_loose ? 4u : 6u)) {
      return 0.0;  // 单码至少 4 角；标定用靶至少 6 点
    }
    if (aruco_loose) {
      const int n_markers = static_cast<int>(corners.size()) / 4;
      score = std::min(0.95, 0.55 + 0.08 * static_cast<double>(n_markers));
    } else {
      const double fill = std::min(
          1.0, static_cast<double>(corners.size()) /
                   static_cast<double>(std::max(6, expect_pts)));
      score = 0.15 + 0.50 * fill;
    }
  } else {
    // 完整模式：点数须达到整面期望（允许极少漏点：≥95%）
    if (aruco_loose) {
      if (corners.size() < 4) {
        return 0.0;
      }
      score = 0.55;
    } else {
      const int need = std::max(4, static_cast<int>(0.95 * expect_pts + 0.5));
      if (static_cast<int>(corners.size()) < need) {
        return 0.0;
      }
      score = 0.45;
    }
  }
  if (fp.area_ratio >= 0.04 && fp.area_ratio <= 0.45) {
    score += 0.25;
    if (fp.area_ratio >= 0.08 && fp.area_ratio <= 0.30) {
      score += 0.10;
    }
  }
  cv::Rect box = cv::boundingRect(corners);
  const double margin = 0.04 * std::min(width, height);
  if (box.x >= margin && box.y >= margin &&
      box.x + box.width <= width - margin &&
      box.y + box.height <= height - margin) {
    score += 0.15;
  }
  const double expect_aspect =
      static_cast<double>(squares_x) / static_cast<double>(std::max(1, squares_y));
  const double aspect =
      static_cast<double>(std::max(1, box.width)) / std::max(1, box.height);
  const double aspect_err =
      std::abs(aspect - expect_aspect) / std::max(expect_aspect, 1e-6);
  if (aspect_err < 0.35) {
    score += 0.15;
  }
  return std::min(1.0, score);
}

/// \brief 统一为连续 8 位三通道 BGR
cv::Mat ensure_bgr8(cv::Mat bgr) {
  if (bgr.empty()) {
    return {};
  }
  if (bgr.type() != CV_8UC3) {
    cv::Mat converted;
    if (bgr.channels() == 1) {
      cv::cvtColor(bgr, converted, cv::COLOR_GRAY2BGR);
    } else if (bgr.channels() == 4) {
      cv::cvtColor(bgr, converted, cv::COLOR_BGRA2BGR);
    } else if (bgr.depth() != CV_8U) {
      bgr.convertTo(converted, CV_8U);
      if (converted.channels() == 1) {
        cv::Mat tmp;
        cv::cvtColor(converted, tmp, cv::COLOR_GRAY2BGR);
        converted = tmp;
      }
    } else {
      converted = bgr;
    }
    bgr = converted;
  }
  if (!bgr.isContinuous()) {
    bgr = bgr.clone();
  }
  return bgr;
}

/// \brief 后台检测任务：靶标检测 + 叠加可视化
DetectJobOut run_detect_job(DetectJobIn in) {
  DetectJobOut out;
  try {
    // —— 预处理 ——
    in.bgr = ensure_bgr8(std::move(in.bgr));
    if (in.bgr.empty()) {
      out.error = in.ros_mode ? QStringLiteral("尚无 ROS 图像帧，请确认话题有数据")
                              : QStringLiteral("无当前图片");
      return out;
    }
    out.width = in.bgr.cols;
    out.height = in.bgr.rows;

    core::Correspondence detected;
    core::DetectedMarkers aruco;
    int faces_found = 0;
    const core::ImageFrame frame = core::mat_as_image_frame(in.bgr, "bgr8");

    // 三面分色：XY / XZ / YZ（由几何聚类写入 aruco.face_ids，与码 ID 无关）
    const cv::Scalar kFaceColors[3] = {
        cv::Scalar(40, 220, 40),    // XY 绿
        cv::Scalar(40, 180, 255),   // XZ 橙
        cv::Scalar(255, 140, 40),   // YZ 蓝
    };
    // 单码 / 未分面：高对比色循环，便于检测台肉眼核对
    const cv::Scalar kArucoPalette[] = {
        cv::Scalar(0, 255, 255),    // 黄
        cv::Scalar(0, 200, 0),      // 绿
        cv::Scalar(255, 128, 0),    // 蓝
        cv::Scalar(0, 128, 255),    // 橙
        cv::Scalar(255, 0, 255),    // 品红
        cv::Scalar(255, 255, 0),    // 青
    };
    constexpr int kArucoPaletteN =
        static_cast<int>(sizeof(kArucoPalette) / sizeof(kArucoPalette[0]));

    auto draw_aruco_overlay = [&]() {
      if (!in.viz_aruco || aruco.empty() ||
          aruco.corners.size() != aruco.ids.size()) {
        return;
      }
      if (aruco.face_ids.size() != aruco.ids.size()) {
        aruco.face_ids.assign(aruco.ids.size(), -1);
      }
      for (size_t i = 0; i < aruco.ids.size(); ++i) {
        const int id = aruco.ids[i];
        const int fi = aruco.face_ids[i];
        const cv::Scalar &col = (fi >= 0 && fi < 3)
            ? kFaceColors[fi]
            : kArucoPalette[static_cast<int>(i) % kArucoPaletteN];
        const auto &corners = aruco.corners[i];
        if (corners.size() < 4) {
          continue;
        }
        std::vector<cv::Point> poly;
        poly.reserve(corners.size());
        cv::Point2f center(0.f, 0.f);
        for (const auto &p : corners) {
          poly.emplace_back(cvRound(p.x), cvRound(p.y));
          center += p;
        }
        center *= 1.f / static_cast<float>(corners.size());
        const cv::Point *pts = poly.data();
        const int npts = static_cast<int>(poly.size());
        // 细边框略加粗，高分辨率下仍可见
        cv::polylines(in.bgr, &pts, &npts, 1, true, col, 2, cv::LINE_AA);
        const std::string label = std::to_string(id);
        int baseline = 0;
        const double font_scale = 0.45;
        const cv::Size tsz = cv::getTextSize(
            label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
        const cv::Point org(
            cvRound(center.x) - tsz.width / 2,
            cvRound(center.y) + tsz.height / 2);
        cv::putText(
            in.bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
            cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(
            in.bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale, col, 1,
            cv::LINE_AA);
      }
    };

    auto resolve_K_D = [&](cv::Mat *K_out, cv::Mat *D_out) {
      if (!in.camera_matrix.empty() && in.camera_matrix.rows == 3 &&
          in.camera_matrix.cols == 3) {
        in.camera_matrix.convertTo(*K_out, CV_64F);
        if (in.dist_coeffs.empty()) {
          *D_out = cv::Mat::zeros(1, 5, CV_64F);
        } else {
          in.dist_coeffs.convertTo(*D_out, CV_64F);
        }
        return true;
      }
      *K_out = core::guess_K(in.bgr.cols, in.bgr.rows);
      *D_out = cv::Mat::zeros(1, 5, CV_64F);
      return false;
    };

    /// \brief 估计 ArUco 位姿、写重投影误差，并按需画坐标系
    auto draw_aruco_axes = [&]() {
      if (aruco.empty() || aruco.corners.size() != aruco.ids.size()) {
        return;
      }
      auto opt_double = [&](const char *key, double def) -> double {
        const auto it = in.solve_options.find(key);
        if (it == in.solve_options.end()) {
          return def;
        }
        try {
          return std::stod(it->second);
        } catch (...) {
          return def;
        }
      };
      const double marker_len = opt_double("marker_length", 0.1);
      if (marker_len <= 1e-6) {
        return;
      }
      cv::Mat K, D;
      const bool real_K = resolve_K_D(&K, &D);
      const core::CameraModelId mid = core::parse_camera_model(in.camera_model);
      const double xi = in.camera_xi;
      const float h = static_cast<float>(marker_len * 0.5);
      // TL,TR,BR,BL — OpenCV ArUco / calib_sim_isaac
      const std::vector<cv::Point3f> obj = {
          {-h, h, 0.f}, {h, h, 0.f}, {h, -h, 0.f}, {-h, -h, 0.f}};
      const float axis_len = static_cast<float>(marker_len * 0.5);

      int drawn = 0;
      double sum_err = 0.0;
      int n_err = 0;
      for (size_t i = 0; i < aruco.corners.size(); ++i) {
        if (aruco.corners[i].size() < 4) {
          continue;
        }
        cv::Mat rvec, tvec;
        bool ok = false;
        // Brown 可用 OpenCV ArUco 官方位姿；其它模型走统一 PnP
        if (mid == core::CameraModelId::BrownConrady) {
          try {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                std::vector<std::vector<cv::Point2f>>{aruco.corners[i]}, marker_len, K, D,
                rvecs, tvecs);
            if (!rvecs.empty() && !tvecs.empty()) {
              rvec = cv::Mat(rvecs[0]);
              tvec = cv::Mat(tvecs[0]);
              ok = true;
            }
          } catch (const cv::Exception &) {
            ok = false;
          }
        }
        if (!ok) {
          ok = core::solve_pnp_model(
              mid, obj, aruco.corners[i], K, D, xi, &rvec, &tvec, true);
        }
        if (!ok) {
          continue;
        }

        std::vector<cv::Point2f> reproj;
        if (!core::project_points_model(mid, obj, rvec, tvec, K, D, xi, &reproj)) {
          continue;
        }
        double err = 0.0;
        if (reproj.size() == 4) {
          for (int k = 0; k < 4; ++k) {
            err += cv::norm(reproj[static_cast<size_t>(k)] -
                            aruco.corners[i][static_cast<size_t>(k)]);
          }
          err *= 0.25;
        }
        sum_err += err;
        ++n_err;
        if (!in.viz_aruco || err > 40.0) {
          continue;
        }
        core::draw_frame_axes_model(in.bgr, mid, K, D, xi, rvec, tvec, axis_len, 2);
        ++drawn;
      }
      const double last_err = n_err > 0 ? (sum_err / static_cast<double>(n_err)) : -1.0;
      out.aruco_reproj_px = last_err;
      if (in.viz_conf) {
        std::string msg;
        const std::string model_tag = core::camera_model_to_string(mid);
        if (n_err > 0 && last_err >= 0.0) {
          msg = cv::format("reproj=%.1fpx (%s)", last_err, model_tag.c_str());
          if (!real_K) {
            msg += " guess_K";
          } else if (drawn > 0) {
            msg += " axes";
          }
        } else {
          msg = "reproj: skip (PnP failed)";
        }
        cv::putText(
            in.bgr, msg, cv::Point(12, in.bgr.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45,
            n_err > 0 ? (real_K ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255))
                      : cv::Scalar(0, 80, 255),
            1, cv::LINE_AA);
      }
    };

    // —— 靶标检测 ——
    if (in.trihedral) {
      auto opt_str = [&](const char *key, const char *def) -> std::string {
        const auto it = in.solve_options.find(key);
        return it == in.solve_options.end() ? std::string(def) : it->second;
      };
      auto opt_double = [&](const char *key, double def) -> double {
        const auto it = in.solve_options.find(key);
        if (it == in.solve_options.end()) {
          return def;
        }
        try {
          return std::stod(it->second);
        } catch (...) {
          return def;
        }
      };
      const std::string target_type = opt_str("target", "trihedral_charuco");
      core::TrihedralTarget geom(in.squares_x, in.squares_y, in.square_length_m);
      if (target_type == "trihedral_charuco" || target_type == "charuco") {
        const std::string dictionary = opt_str("dictionary", "DICT_4X4_250");
        const double marker_len = opt_double("marker_length", 0.018);
        core::TrihedralCharucoDetector detector(geom, marker_len, dictionary);
        detected = detector.detect_merged(frame, &faces_found, &aruco, in.fast);
        if (detected.image_points.rows() < 4) {
          draw_aruco_overlay();
          if (in.viz_conf) {
            cv::putText(
                in.bgr, cv::format("aruco=%d (no charuco)", static_cast<int>(aruco.ids.size())),
                cv::Point(12, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1,
                cv::LINE_AA);
          }
          out.preview = mat_bgr_to_qimage(in.bgr);
          out.error = aruco.empty()
              ? QStringLiteral("未检测到三面靶 ChArUco（请确认字典/方格与 Isaac 一致；分面靠几何聚类）")
              : QStringLiteral("检测到 %1 个 ArUco，但共面分面/棋盘角点不足")
                    .arg(static_cast<int>(aruco.ids.size()));
          return out;
        }
      } else {
        // trihedral_chess / 默认：仅角点棋盘，掩膜剥面后贴 XY/XZ/YZ
        core::TrihedralChessDetector detector(geom);
        const std::string completeness = opt_str("detect_completeness", "");
        const bool require_full =
            completeness == "full" || completeness == "complete";
        core::TrihedralChessDetectSpeed speed =
            require_full ? core::TrihedralChessDetectSpeed::Complete
                         : (in.fast ? core::TrihedralChessDetectSpeed::Fast
                                    : core::TrihedralChessDetectSpeed::Thorough);
        detected = detector.detect_merged(frame, &faces_found, speed);
        if (detected.image_points.rows() < 4) {
          out.preview = mat_bgr_to_qimage(in.bgr);
          out.error = (target_type == "trihedral_aruco")
              ? QStringLiteral("三面 ArUco 网格请改用 trihedral_charuco；当前按棋盘角点路径未检出")
              : (require_full
                     ? QStringLiteral(
                           "未检测到完整三面棋盘网格（完整模式不接受局部；确认 squares_x/y）")
                     : QStringLiteral(
                           "未检测到三面棋盘角点（允许局部网格；确认 squares_x/y=内角点数 与 "
                           "Isaac 一致）"));
          return out;
        }
      }
    } else {
      auto opt_str = [&](const char *key, const char *def) -> std::string {
        const auto it = in.solve_options.find(key);
        return it == in.solve_options.end() ? std::string(def) : it->second;
      };
      auto opt_double = [&](const char *key, double def) -> double {
        const auto it = in.solve_options.find(key);
        if (it == in.solve_options.end()) {
          return def;
        }
        try {
          return std::stod(it->second);
        } catch (...) {
          return def;
        }
      };
      const std::string target_type = opt_str("target", "chessboard");
      const std::string dictionary = opt_str(
          "dictionary",
          (target_type == "charuco" || target_type == "trihedral_charuco")
              ? "DICT_4X4_50"
              : "DICT_6X6_1000");
      double marker_len = opt_double(
          "marker_length",
          (target_type == "charuco" || target_type == "trihedral_charuco")
              ? 0.03
              : (target_type == "aprilgrid" ? 0.088 : 0.1));
      double marker_sep = opt_double("marker_separation", -1.0);
      if (marker_sep <= 0.0) {
        marker_sep = (in.square_length_m > marker_len)
            ? (in.square_length_m - marker_len)
            : 0.006;
      }

      std::vector<core::Correspondence> corrs;
      if (target_type == "charuco") {
        const auto ch_params = core::charuco_detector_from_config(in.solve_options);
        if (ch_params.marker_length_m > 1e-6) {
          marker_len = ch_params.marker_length_m;
        }
        core::CharucoTarget target(
            in.squares_x, in.squares_y, in.square_length_m, marker_len, dictionary);
        corrs = core::CharucoDetector(target, ch_params).detect(frame, &aruco);
        if (corrs.empty()) {
          draw_aruco_overlay();
          out.preview = mat_bgr_to_qimage(in.bgr);
          if (!aruco.empty()) {
            out.error = QStringLiteral(
                            "检测到 %1 个 ArUco，但 ChArUco 角点不足。"
                            "请核对：squares_x/y 是方格数(不是内角点)、dictionary、"
                            "marker_length < square_length")
                            .arg(static_cast<int>(aruco.ids.size()));
          } else {
            out.error = QStringLiteral(
                "未检测到 ChArUco 标记。请核对 dictionary"
                "（常见 DICT_4X4_50 / DICT_5X5_100，默认 DICT_6X6_1000 常不匹配）与光照");
          }
          return out;
        }
      } else if (target_type == "aruco" || target_type == "aruco_single") {
        corrs = core::ArucoMarkerDetector(dictionary, marker_len)
                    .detect(frame, &aruco, in.fast);
      } else if (target_type == "aruco_grid") {
        core::ArucoGridTarget target(
            in.squares_x, in.squares_y, marker_len, marker_sep, dictionary);
        corrs = core::ArucoGridDetector(target).detect(frame, &aruco);
      } else if (target_type == "aprilgrid") {
        // UI 把 tagSpacing 放在 square_length；若未单独写 tag_spacing 则回退
        double tag_spacing = opt_double("tag_spacing", -1.0);
        if (tag_spacing < 0.0) {
          tag_spacing = in.square_length_m;
        }
        // 棋盘残留 0.025 等不合理值钳回 Kalibr 默认 0.3
        if (tag_spacing < 0.05 || tag_spacing > 2.0) {
          tag_spacing = 0.3;
        }
        std::string april_dict = dictionary;
        if (april_dict.find("APRILTAG") == std::string::npos) {
          april_dict = "DICT_APRILTAG_36h11";
        }
        core::AprilgridTarget target(
            in.squares_x, in.squares_y, marker_len, tag_spacing);
        const auto april_params =
            core::aprilgrid_detector_from_config(in.solve_options);
        corrs = core::AprilgridDetector(target, april_dict, april_params)
                    .detect(frame, &aruco);
        if (corrs.empty()) {
          draw_aruco_overlay();
          if (aruco.empty()) {
            cv::putText(
                in.bgr,
                "AprilGrid: 0 tags — print on paper (screen moire kills detection)",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 80, 255), 2, cv::LINE_AA);
          } else {
            cv::putText(
                in.bgr,
                cv::format(
                    "AprilGrid: %d tags outside %dx%d grid (dict=%s)",
                    static_cast<int>(aruco.ids.size()), in.squares_x, in.squares_y,
                    aruco.dictionary_name.empty() ? "?" : aruco.dictionary_name.c_str()),
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
          }
          out.preview = mat_bgr_to_qimage(in.bgr);
          if (!aruco.empty()) {
            out.error = QStringLiteral(
                            "检测到 %1 个 AprilTag（%2），但未落入当前网格"
                            "（tagCols×tagRows=%3×%4，ID 须为 0..%5）")
                            .arg(static_cast<int>(aruco.ids.size()))
                            .arg(QString::fromStdString(
                                aruco.dictionary_name.empty() ? april_dict
                                                              : aruco.dictionary_name))
                            .arg(in.squares_x)
                            .arg(in.squares_y)
                            .arg(in.squares_x * in.squares_y - 1);
          } else {
            out.error = QStringLiteral(
                "未检测到 AprilTag。请优先用纸质板（屏显摩尔纹极易全灭）。"
                "核对：Kalibr AprilGrid / tag36h11；列×行；边长与间距；"
                "字典 DICT_APRILTAG_36h11；对焦清晰");
          }
          return out;
        }
      } else if (
          target_type == "circles_symmetric" || target_type == "circle_grid" ||
          target_type == "circles" || target_type == "Circles" ||
          target_type == "circles_asymmetric" ||
          target_type == "asymmetric_circles" ||
          target_type == "Asymmetric Circles") {
        const bool asymmetric =
            target_type == "circles_asymmetric" ||
            target_type == "asymmetric_circles" ||
            target_type == "Asymmetric Circles";
        const auto pattern = asymmetric ? core::CircleGridPattern::Asymmetric
                                        : core::CircleGridPattern::Symmetric;
        const double circle_diam = opt_double("circle_diameter", 0.0);
        core::CircleGridTarget target(
            in.squares_x, in.squares_y, in.square_length_m, pattern, circle_diam);
        const auto dot_params = core::dot_detector_from_config(in.solve_options);
        corrs = core::detect_circle_grid_impl(frame, target, dot_params);
        if (corrs.empty()) {
          cv::putText(
              in.bgr,
              asymmetric
                  ? cv::format(
                        "Asym circles fail: size=(per-row x rows)=%dx%d",
                        in.squares_x, in.squares_y)
                  : cv::format(
                        "Sym circles fail: size=%dx%d", in.squares_x,
                        in.squares_y),
              cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(0, 80, 255), 2, cv::LINE_AA);
          out.preview = mat_bgr_to_qimage(in.bgr);
          out.error = asymmetric
              ? QStringLiteral(
                    "未检测到非对称圆点阵（当前 %1×%2）。"
                    "请核对：每行×行数；对角间距（calib.io Diagonal Spacing）；圆直径")
                    .arg(in.squares_x)
                    .arg(in.squares_y)
              : QStringLiteral(
                    "未检测到对称圆点阵（当前 %1×%2）。请核对列×行、圆心距与圆直径")
                    .arg(in.squares_x)
                    .arg(in.squares_y);
          return out;
        }
      } else {
        core::ChessboardTarget target(
            in.squares_x, in.squares_y, in.square_length_m);
        const core::ChessDetectorParams cb_params =
            core::chess_detector_from_config(in.solve_options);
        core::ChessboardDetectOptions dopts;
        auto flag_on = [&](const char *key, bool def) {
          const auto it = in.solve_options.find(key);
          if (it == in.solve_options.end()) {
            return def;
          }
          return it->second == "1" || it->second == "true";
        };
        const std::string completeness = opt_str("detect_completeness", "");
        const bool require_full =
            completeness == "full" || completeness == "complete";
        const bool force_partial = completeness == "partial";
        dopts.adaptive_thresh = cb_params.adaptive_thresh;
        dopts.normalize_image = cb_params.normalize_image;
        dopts.filter_quads = flag_on("cb_filter_quads", false);
        dopts.fast_check = cb_params.fast_check;
        dopts.resized_detection = cb_params.resized_detection;
        dopts.resized_max_resolution = cb_params.resized_max_resolution;
        dopts.padding = cb_params.padding;
        if (in.has_chess_roi &&
            in.chess_lost_frames <= cb_params.max_lost_frames) {
          dopts.search_roi = in.chess_roi;
        }
        dopts.allow_partial = force_partial && !require_full;
        dopts.thorough = !in.fast;
        if (cb_params.sub_pixel_refinement) {
          const auto it = in.solve_options.find("subpix_win");
          if (it != in.solve_options.end()) {
            try {
              dopts.subpix_win = std::stoi(it->second);
            } catch (...) {
              dopts.subpix_win = 11;
            }
          } else {
            dopts.subpix_win = 11;
          }
        } else {
          dopts.subpix_win = 0;
        }
        core::ChessboardDetector detector(target, dopts);
        corrs = detector.detect(frame);
        if (!corrs.empty()) {
          out.has_chess_roi = true;
          out.chess_roi = detector.last_search_roi();
          out.chess_lost_frames = 0;
        } else if (in.has_chess_roi) {
          out.chess_lost_frames = in.chess_lost_frames + 1;
          if (out.chess_lost_frames <= cb_params.max_lost_frames) {
            out.has_chess_roi = true;
            out.chess_roi = in.chess_roi;
          }
        }
      }
      if (corrs.empty()) {
        draw_aruco_overlay();
        if (in.viz_conf && !aruco.empty()) {
          cv::putText(
              in.bgr, cv::format("aruco=%d (no board)", static_cast<int>(aruco.ids.size())),
              cv::Point(12, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1,
              cv::LINE_AA);
        }
        if (target_type == "aruco" || target_type == "aruco_single") {
          cv::putText(
              in.bgr,
              "ArUco: 0 markers (check dictionary / lighting)",
              cv::Point(12, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 80, 255), 2,
              cv::LINE_AA);
          out.preview = mat_bgr_to_qimage(in.bgr);
          out.error = QStringLiteral(
              "未检测到 ArUco 码。calib_sim_isaac 默认字典为 DICT_6X6_1000；"
              "请确认 dictionary / 光照，或到检测台调试");
          return out;
        }
        out.preview = mat_bgr_to_qimage(in.bgr);
        out.error = aruco.empty()
            ? QStringLiteral("未检测到靶标角点（%1）").arg(QString::fromStdString(target_type))
            : QStringLiteral("检测到 %1 个 ArUco，但棋盘/阵列角点不足")
                  .arg(static_cast<int>(aruco.ids.size()));
        return out;
      }
      detected = corrs.front();
      faces_found = 1;
    }

    // —— 置信度与视角指纹 ——
    std::vector<cv::Point2f> corners;
    corners.reserve(static_cast<size_t>(detected.image_points.rows()));
    for (int i = 0; i < detected.image_points.rows(); ++i) {
      corners.emplace_back(
          static_cast<float>(detected.image_points(i, 0)),
          static_cast<float>(detected.image_points(i, 1)));
    }
    out.fp = fingerprint_from_corners_job(corners, in.bgr.cols, in.bgr.rows);
    out.confidence = confidence_from_corners_job(
        corners, in.bgr.cols, in.bgr.rows, in.squares_x, in.squares_y, in.trihedral,
        in.solve_options);
    if (in.trihedral) {
      out.confidence =
          std::min(1.0, out.confidence + 0.08 * (faces_found - 1));
    }
    out.faces = faces_found;
    out.corr = std::move(detected);

    // —— 叠加可视化 ——
    draw_aruco_overlay();
    {
      const std::string target_type =
          in.solve_options.count("target") ? in.solve_options.at("target")
                                           : "chessboard";
      if (target_type == "aruco" || target_type == "aruco_single" ||
          target_type == "aruco_grid" || target_type == "aprilgrid" ||
          target_type == "charuco" || in.trihedral) {
        draw_aruco_axes();
      }
    }
    if (in.viz_corners) {
      const std::string target_type =
          in.solve_options.count("target") ? in.solve_options.at("target")
                                           : "chessboard";
      const bool aruco_only =
          target_type == "aruco" || target_type == "aruco_single";
      // 单码路径已由 draw_aruco_overlay 画边框/ID/角序，避免再叠实心点
      if (!aruco_only) {
      const bool draw_as_markers =
          in.trihedral || in.viz_marker_radius > 4 || target_type != "chessboard" ||
          static_cast<int>(corners.size()) != in.squares_x * in.squares_y;
      if (draw_as_markers) {
        std::vector<cv::Point2f> per_face[3];
        for (int i = 0; i < out.corr.image_points.rows(); ++i) {
          const int face = (i < static_cast<int>(out.corr.ids.size()))
              ? out.corr.ids[static_cast<size_t>(i)] / 1000
              : 0;
          const int fi = std::max(0, std::min(2, face));
          const cv::Scalar &col = in.trihedral ? kFaceColors[fi] : kFaceColors[0];
          const cv::Point2f pt(
              static_cast<float>(out.corr.image_points(i, 0)),
              static_cast<float>(out.corr.image_points(i, 1)));
          cv::circle(in.bgr, pt, in.viz_marker_radius, col, -1, cv::LINE_AA);
          if (in.trihedral) {
            per_face[fi].push_back(pt);
          }
        }
        if (in.viz_hull && in.trihedral) {
          for (int f = 0; f < 3; ++f) {
            if (per_face[f].size() < 3) {
              continue;
            }
            std::vector<cv::Point2f> hull_f;
            cv::convexHull(per_face[f], hull_f);
            std::vector<cv::Point> hull;
            hull.reserve(hull_f.size());
            for (const auto &p : hull_f) {
              hull.emplace_back(cvRound(p.x), cvRound(p.y));
            }
            if (hull.size() >= 3) {
              const cv::Point *pts = hull.data();
              const int npts = static_cast<int>(hull.size());
              cv::polylines(in.bgr, &pts, &npts, 1, true, kFaceColors[f], 2, cv::LINE_AA);
            }
          }
        }
      } else {
        cv::drawChessboardCorners(
            in.bgr, cv::Size(in.squares_x, in.squares_y), corners, true);
      }
      }
    }
    if (in.viz_hull && corners.size() >= 3 && !(in.trihedral && in.viz_corners)) {
      std::vector<cv::Point2f> hull_f;
      cv::convexHull(corners, hull_f);
      std::vector<cv::Point> hull;
      hull.reserve(hull_f.size());
      for (const auto &p : hull_f) {
        hull.emplace_back(cvRound(p.x), cvRound(p.y));
      }
      if (hull.size() >= 3) {
        const cv::Point *pts = hull.data();
        const int npts = static_cast<int>(hull.size());
        cv::polylines(
            in.bgr, &pts, &npts, 1, true, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      }
    }
    if (in.viz_conf) {
      std::string label;
      if (in.trihedral) {
        label = cv::format(
            "conf=%.2f faces=%d aruco=%d", out.confidence, faces_found,
            static_cast<int>(aruco.ids.size()));
      } else if (!aruco.empty()) {
        if (!aruco.dictionary_name.empty()) {
          label = cv::format(
              "conf=%.2f aruco=%d %s", out.confidence,
              static_cast<int>(aruco.ids.size()), aruco.dictionary_name.c_str());
        } else {
          label = cv::format(
              "conf=%.2f aruco=%d", out.confidence,
              static_cast<int>(aruco.ids.size()));
        }
      } else {
        label = cv::format("conf=%.2f", out.confidence);
      }
      cv::putText(
          in.bgr, label, cv::Point(12, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      if (in.trihedral) {
        // 图例：XY / XZ / YZ
        const char *names[3] = {"XY", "XZ", "YZ"};
        for (int f = 0; f < 3; ++f) {
          const int x0 = 12 + f * 54;
          cv::rectangle(
              in.bgr, cv::Point(x0, 28), cv::Point(x0 + 12, 40), kFaceColors[f], -1,
              cv::LINE_AA);
          cv::putText(
              in.bgr, names[f], cv::Point(x0 + 16, 39), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              kFaceColors[f], 1, cv::LINE_AA);
        }
      }
    }
    out.preview = mat_bgr_to_qimage(in.bgr);
    {
      const std::string completeness =
          in.solve_options.count("detect_completeness")
              ? in.solve_options.at("detect_completeness")
              : "";
      const bool require_full =
          completeness == "full" || completeness == "complete";
      if (require_full && out.confidence <= 1e-9) {
        out.ok = false;
        out.error = QStringLiteral(
            "完整模式：标定板不完整或点数不足，已忽略（请换整面清晰帧）");
        return out;
      }
    }
    out.ok = true;
    return out;
  } catch (const cv::Exception &e) {
    out.ok = false;
    out.error = QStringLiteral("OpenCV 检测异常：%1").arg(QString::fromUtf8(e.what()));
    return out;
  }
}

}  // namespace

/// \brief 构造会话控制器
SessionController::SessionController(QObject *parent)
    : QObject(parent) {}

/// \brief 析构：取消后台检测并 join 线程
SessionController::~SessionController() {
  detect_epoch_.fetch_add(1);
  if (detect_thread_.joinable()) {
    detect_thread_.join();
  }
  if (solve_thread_.joinable()) {
    solve_thread_.join();
  }
}

/// \brief 设置标定器 ID，并按类型调整位姿源/最少视角
void SessionController::set_calibrator_id(const QString &id) {
  if (calibrator_id_ == id) {
    return;
  }
  calibrator_id_ = id;
  const DetectLabMode from_id = detect_lab_mode_from_task_id(id);
  if (from_id != DetectLabMode::None) {
    detect_lab_mode_ = from_id;
  }
  if (is_handeye() && pose_source_ == PoseSource::None) {
    pose_source_ = PoseSource::Csv;
  }
  if (is_intrinsics() || is_trihedral() || is_stereo_extrinsics()) {
    pose_source_ = PoseSource::None;
  }
  if (is_intrinsics()) {
    configure_intrinsics_engine();
  }
  if (is_trihedral() && min_views_ > 1) {
    // Default oneshot-friendly; user can raise for multi-view
    min_views_ = 1;
  }
}

void SessionController::set_detect_lab_mode(DetectLabMode mode) {
  detect_lab_mode_ = mode;
}

void SessionController::sync_detect_lab_mode_from_task_id(const QString &task_id) {
  detect_lab_mode_ = detect_lab_mode_from_task_id(task_id);
}

/// \brief 是否为手眼标定器
bool SessionController::is_handeye() const {
  return calibrator_id_ == QStringLiteral("eye_in_hand") ||
         calibrator_id_ == QStringLiteral("eye_to_hand");
}

/// \brief 是否为单目内参标定器
bool SessionController::is_intrinsics() const {
  return calibrator_id_ == QStringLiteral("cam_intrinsics") ||
         calibrator_id_ == QStringLiteral("stereo_intrinsics");
}

bool SessionController::is_stereo_intrinsics() const {
  return calibrator_id_ == QStringLiteral("stereo_intrinsics");
}

bool SessionController::is_stereo_extrinsics() const {
  return calibrator_id_ == QStringLiteral("stereo_extrinsics");
}

bool SessionController::is_stereo_side_tagged() const {
  return is_stereo_intrinsics() || is_stereo_extrinsics();
}

/// \brief 是否为直角三面标定器
bool SessionController::is_trihedral() const {
  return calibrator_id_ == QStringLiteral("trihedral_oneshot");
}

/// \brief 切换离线/ROS 图像源并清除当前检测
void SessionController::set_source_mode(SourceMode mode) {
  source_mode_ = mode;
  has_detection_ = false;
  if (is_intrinsics()) {
    intrinsics_state_.set_offline_source(
        mode == SourceMode::Offline || mode == SourceMode::RosBag);
    configure_intrinsics_engine();
  }
  emit current_changed();
}

/// \brief 设置手眼位姿源（CSV/TF/无）
void SessionController::set_pose_source(PoseSource src) {
  pose_source_ = src;
}

/// \brief 设置靶标网格参数
void SessionController::set_board_params(
    int squares_x, int squares_y, double square_length_m) {
  squares_x_ = squares_x;
  squares_y_ = squares_y;
  square_length_m_ = square_length_m;
  has_detection_ = false;
}

/// \brief 设置自动采集阈值与冷却
void SessionController::set_capture_options(
    int min_views, double min_confidence, double min_diversity, int cooldown_ms) {
  min_views_ = std::max(1, min_views);
  min_confidence_ = min_confidence;
  min_diversity_ = min_diversity;
  auto_cooldown_ms_ = std::max(100, cooldown_ms);
}

/// \brief 设置求解器键值选项
void SessionController::set_solve_options(const std::map<std::string, std::string> &opts) {
  if (solve_options_ == opts) {
    return;
  }
  solve_options_ = opts;
  configure_intrinsics_engine();
}

/// \brief 设置检测预览叠加开关
void SessionController::set_viz_options(
    bool corners, bool hull, bool conf, int marker_radius, bool aruco) {
  viz_corners_ = corners;
  viz_hull_ = hull;
  viz_conf_ = conf;
  viz_aruco_ = aruco;
  viz_marker_radius_ = std::max(1, marker_radius);
}

/// \brief 设置手眼 TF 坐标系并同步到桥
void SessionController::set_handeye_frames(const QString &base, const QString &gripper) {
  base_frame_ = base;
  gripper_frame_ = gripper;
  if (tf_bridge_) {
    tf_bridge_->set_frames(base_frame_, gripper_frame_);
  }
}

/// \brief 设置手眼求解方法名
void SessionController::set_handeye_method(const QString &method) {
  handeye_method_ = method;
}

/// \brief 设置手眼用相机内参 YAML 路径
void SessionController::set_camera_yaml(const QString &path) {
  if (camera_yaml_ == path && !detect_K_.empty()) {
    return;
  }
  camera_yaml_ = path;
  if (path.isEmpty()) {
    return;
  }
  core::CameraIntrinsics intr;
  if (core::load_camera_yaml(path.toStdString(), &intr) && intr.valid && !intr.K.empty()) {
    detect_K_ = intr.K.clone();
    detect_D_ = intr.D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : intr.D.clone();
    detect_model_ = intr.model.empty() ? "brown_conrady" : intr.model;
    detect_xi_ = intr.xi;
  }
}

/// \brief 设置检测叠加用内参（CameraInfo 优先）
void SessionController::set_detect_intrinsics(
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    const std::string &model,
    double xi) {
  if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
    return;
  }
  camera_matrix.convertTo(detect_K_, CV_64F);
  if (dist_coeffs.empty()) {
    detect_D_ = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    dist_coeffs.convertTo(detect_D_, CV_64F);
  }
  detect_model_ = core::camera_model_to_string(core::parse_camera_model(model));
  detect_xi_ = xi;
}

void SessionController::clear_detect_intrinsics() {
  detect_K_ = cv::Mat();
  detect_D_ = cv::Mat();
  detect_model_ = "brown_conrady";
  detect_xi_ = 0.0;
}

/// \brief 绑定 TF 位姿桥
void SessionController::set_tf_bridge(TfPoseBridge *bridge) {
  tf_bridge_ = bridge;
  if (tf_bridge_) {
    tf_bridge_->set_frames(base_frame_, gripper_frame_);
  }
}

/// \brief 记录当前 ROS 图像话题名（展示用）
void SessionController::set_ros_topic_name(const QString &topic) {
  ros_topic_name_ = topic;
}

/// \brief 写入在线模式最新 BGR 帧
void SessionController::set_live_bgr(const cv::Mat &bgr) {
  if (bgr.empty()) {
    live_bgr_.release();
  } else {
    live_bgr_ = bgr.clone();
  }
}

void SessionController::set_live_bgr(cv::Mat &&bgr) {
  if (bgr.empty()) {
    live_bgr_.release();
  } else {
    live_bgr_ = std::move(bgr);
  }
}

QImage SessionController::cached_live_preview_qimage() const {
  std::lock_guard<std::mutex> lock(live_preview_mutex_);
  return cached_live_preview_qimage_;
}

QImage SessionController::cached_stereo_live_preview_left() const {
  std::lock_guard<std::mutex> lock(stereo_live_preview_mutex_);
  return cached_stereo_live_left_;
}

QImage SessionController::cached_stereo_live_preview_right() const {
  std::lock_guard<std::mutex> lock(stereo_live_preview_mutex_);
  return cached_stereo_live_right_;
}

QImage SessionController::cached_offline_preview_qimage() const {
  std::lock_guard<std::mutex> lock(offline_preview_mutex_);
  return cached_offline_preview_qimage_;
}

void SessionController::schedule_live_preview_update() {
  if (source_mode_ != SourceMode::RosTopic || live_bgr_.empty()) {
    return;
  }
  if (live_preview_busy_.exchange(true)) {
    return;
  }
  cv::Mat bgr = live_bgr_.clone();
  std::thread([this, bgr = std::move(bgr)]() mutable {
    QImage img = mat_bgr_to_qimage(bgr);
    QMetaObject::invokeMethod(
        this,
        [this, img = std::move(img)]() {
          {
            std::lock_guard<std::mutex> lock(live_preview_mutex_);
            cached_live_preview_qimage_ = img;
          }
          live_preview_busy_.store(false);
          emit live_preview_updated();
        },
        Qt::QueuedConnection);
  }).detach();
}

void SessionController::schedule_stereo_live_preview_update() {
  if (source_mode_ != SourceMode::RosTopic || !uses_stereo_dual_session()) {
    return;
  }
  if (live_left_bgr_.empty() && live_right_bgr_.empty()) {
    return;
  }
  if (stereo_live_preview_busy_.exchange(true)) {
    return;
  }
  cv::Mat left = live_left_bgr_.clone();
  cv::Mat right = live_right_bgr_.clone();
  if (left.empty() && right.empty()) {
    stereo_live_preview_busy_.store(false);
    return;
  }
  std::thread([this, left = std::move(left), right = std::move(right)]() mutable {
    QImage l = mat_bgr_to_qimage(left);
    QImage r = mat_bgr_to_qimage(right);
    QMetaObject::invokeMethod(
        this,
        [this, l = std::move(l), r = std::move(r)]() {
          {
            std::lock_guard<std::mutex> lock(stereo_live_preview_mutex_);
            cached_stereo_live_left_ = l;
            cached_stereo_live_right_ = r;
          }
          stereo_live_preview_busy_.store(false);
          emit stereo_live_preview_updated();
        },
        Qt::QueuedConnection);
  }).detach();
}

void SessionController::schedule_offline_preview_update() {
  if (source_mode_ == SourceMode::RosTopic) {
    return;
  }
  if (offline_preview_busy_.exchange(true)) {
    return;
  }
  const bool stereo = uses_stereo_dual_session();
  const int request_index = stereo ? stereo_pair_index_ : current_index_;
  QString left_path;
  QString mono_path;
  if (stereo && stereo_pair_index_ >= 0 &&
      stereo_pair_index_ < stereo_left_paths_.size()) {
    left_path = stereo_left_paths_.at(stereo_pair_index_);
  } else if (current_index_ >= 0 && current_index_ < image_paths_.size()) {
    mono_path = image_paths_.at(current_index_);
  } else if (source_mode_ == SourceMode::RosBag && current_index_ >= 0 &&
             current_index_ < bag_reader_.size()) {
    // Bag 帧已在内存，主线程转图即可
    offline_preview_busy_.store(false);
    cv::Mat bgr = bag_reader_.frame(current_index_);
    QImage img = mat_bgr_to_qimage(bgr);
    {
      std::lock_guard<std::mutex> lock(offline_preview_mutex_);
      cached_offline_preview_qimage_ = img;
      offline_preview_index_ = current_index_;
    }
    emit offline_preview_updated();
    return;
  } else {
    offline_preview_busy_.store(false);
    return;
  }
  std::thread([this, request_index, stereo, left_path, mono_path]() {
    QImage img;
    if (stereo && !left_path.isEmpty()) {
      cv::Mat bgr = cv::imread(left_path.toStdString(), cv::IMREAD_COLOR);
      img = mat_bgr_to_qimage(bgr);
    } else if (!mono_path.isEmpty()) {
      cv::Mat bgr = cv::imread(mono_path.toStdString(), cv::IMREAD_COLOR);
      img = mat_bgr_to_qimage(bgr);
    }
    QMetaObject::invokeMethod(
        this,
        [this, request_index, stereo, img = std::move(img)]() {
          offline_preview_busy_.store(false);
          if (stereo) {
            if (stereo_pair_index_ != request_index) {
              return;
            }
          } else if (current_index_ != request_index) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(offline_preview_mutex_);
            cached_offline_preview_qimage_ = img;
            offline_preview_index_ = request_index;
          }
          emit offline_preview_updated();
        },
        Qt::QueuedConnection);
  }).detach();
}

/// \brief 加载离线位姿 CSV
bool SessionController::load_pose_csv(const QString &path, QString *error_out) {
  return pose_csv_.load(path, error_out);
}

/// \brief 清空离线/Bag 载入前的会话图像与标定缓存
void SessionController::clear_loaded_source_data() {
  image_paths_.clear();
  current_index_ = -1;
  has_detection_ = false;
  last_confidence_ = 0.0;
  last_aruco_reproj_px_ = -1.0;
  last_preview_ = {};
  captured_fps_.clear();
  captured_views_.clear();
  clear_capture_cache();
  batch_ = {};
  last_result_ = {};
  bag_reader_.clear();
  stereo_bag_reader_.clear();
  stereo_left_paths_.clear();
  stereo_right_paths_.clear();
  stereo_pair_index_ = -1;
  stereo_pairs_.clear();
  live_left_bgr_.release();
  live_right_bgr_.release();
  last_stereo_sync_delta_ms_ = -1;
  stereo_left_detect_ = {};
  stereo_right_detect_ = {};
  has_stereo_rectify_maps_ = false;
  stereo_map1_x_.release();
  stereo_map1_y_.release();
  stereo_map2_x_.release();
  stereo_map2_y_.release();
  if (is_intrinsics()) {
    intrinsics_state_.reset();
    if (is_stereo_intrinsics()) {
      intrinsics_left_state_.reset();
      intrinsics_right_state_.reset();
    }
  }
}

/// \brief 扫描目录加载离线图片列表并重置会话数据
int SessionController::load_image_dir(const QString &dir_path) {
  clear_loaded_source_data();

  QDir dir(dir_path);
  if (!dir.exists()) {
    emit images_changed();
    emit observations_changed();
    emit result_changed();
    return 0;
  }

  // —— 扫描常见图片后缀 ——
  const QStringList filters = {
      QStringLiteral("*.png"),  QStringLiteral("*.jpg"),
      QStringLiteral("*.jpeg"), QStringLiteral("*.bmp"),
      QStringLiteral("*.PNG"),  QStringLiteral("*.JPG"),
      QStringLiteral("*.JPEG"), QStringLiteral("*.BMP"),
  };
  const QFileInfoList files =
      dir.entryInfoList(filters, QDir::Files, QDir::Name);
  QStringList paths;
  paths.reserve(files.size());
  for (const QFileInfo &fi : files) {
    paths.push_back(fi.absoluteFilePath());
  }
  return apply_image_paths(dir_path, paths);
}

int SessionController::apply_image_paths(
    const QString &dir_path, const QStringList &paths) {
  clear_loaded_source_data();
  (void)dir_path;
  image_paths_ = paths;
  current_index_ = paths.isEmpty() ? -1 : 0;
  emit images_changed();
  emit current_changed();
  emit observations_changed();
  emit result_changed();
  return image_paths_.size();
}

int SessionController::load_rosbag(
    const QString &bag_uri,
    const QString &topic,
    int max_frames,
    QString *error_out) {
  RosBagFrameReader reader;
  const int n = reader.open(bag_uri, topic, max_frames, error_out);
  if (n <= 0) {
    clear_loaded_source_data();
    emit images_changed();
    emit observations_changed();
    emit result_changed();
    return 0;
  }
  return apply_loaded_bag(std::move(reader), topic);
}

int SessionController::apply_loaded_bag(RosBagFrameReader reader, const QString &topic) {
  clear_loaded_source_data();
  bag_reader_ = std::move(reader);
  const int n = bag_reader_.size();
  if (n <= 0) {
    emit images_changed();
    emit observations_changed();
    emit result_changed();
    return 0;
  }
  image_paths_.reserve(n);
  for (int i = 0; i < n; ++i) {
    image_paths_.push_back(bag_reader_.frame_label(i));
  }
  current_index_ = 0;
  set_ros_topic_name(topic);
  set_source_mode(SourceMode::RosBag);
  emit images_changed();
  emit current_changed();
  emit observations_changed();
  emit result_changed();
  return n;
}

/// \brief 切换离线当前图片索引
void SessionController::set_current_index(int index) {
  if (source_mode_ == SourceMode::RosTopic) {
    return;
  }
  if (uses_stereo_dual_session() && !stereo_left_paths_.isEmpty()) {
    set_stereo_pair_index(index);
    return;
  }
  if (image_paths_.isEmpty()) {
    current_index_ = -1;
    has_detection_ = false;
    emit current_changed();
    return;
  }
  if (index < 0) {
    index = 0;
  }
  if (index >= image_paths_.size()) {
    index = image_paths_.size() - 1;
  }
  if (current_index_ == index) {
    return;
  }
  current_index_ = index;
  has_detection_ = false;
  emit current_changed();
}

/// \brief 当前帧路径或 ros:// 话题标识
QString SessionController::current_path() const {
  if (source_mode_ == SourceMode::RosTopic) {
    if (ros_topic_name_.isEmpty()) {
      return QStringLiteral("ros://(未选择话题)");
    }
    return QStringLiteral("ros://%1").arg(ros_topic_name_);
  }
  if (uses_stereo_dual_session() && stereo_pair_index_ >= 0 &&
      stereo_pair_index_ < stereo_left_paths_.size()) {
    return stereo_left_paths_.at(stereo_pair_index_);
  }
  if (current_index_ < 0 || current_index_ >= image_paths_.size()) {
    return {};
  }
  return image_paths_.at(current_index_);
}

/// \brief 取当前帧 BGR（在线 live / 离线 imread / bag 内存）
cv::Mat SessionController::current_bgr() const {
  if (uses_stereo_dual_session()) {
    const QString mode = stereo_capture_mode();
    if (mode == QStringLiteral("right") && !live_right_bgr_.empty()) {
      return live_right_bgr_;
    }
    if (!live_left_bgr_.empty()) {
      return live_left_bgr_;
    }
    if (stereo_pair_index_ >= 0) {
      if (source_mode_ == SourceMode::RosBag && !stereo_bag_reader_.empty() &&
          stereo_pair_index_ < stereo_bag_reader_.size()) {
        const auto &pair = stereo_bag_reader_.pair(stereo_pair_index_);
        if (mode == QStringLiteral("right")) {
          return pair.right_bgr.empty() ? cv::Mat() : pair.right_bgr.clone();
        }
        return pair.left_bgr.empty() ? cv::Mat() : pair.left_bgr.clone();
      }
      if (stereo_pair_index_ < stereo_left_paths_.size()) {
        const QString path =
            (mode == QStringLiteral("right") &&
             stereo_pair_index_ < stereo_right_paths_.size())
                ? stereo_right_paths_.at(stereo_pair_index_)
                : stereo_left_paths_.at(stereo_pair_index_);
        if (!path.startsWith(QStringLiteral("bag://"))) {
          return cv::imread(path.toStdString(), cv::IMREAD_COLOR);
        }
      }
    }
  }
  if (source_mode_ == SourceMode::RosTopic) {
    return live_bgr_;
  }
  if (source_mode_ == SourceMode::RosBag && !bag_reader_.empty()) {
    if (current_index_ < 0 || current_index_ >= bag_reader_.size()) {
      return {};
    }
    return bag_reader_.frame(current_index_);
  }
  const QString path = current_path();
  if (path.isEmpty()) {
    return {};
  }
  return cv::imread(path.toStdString(), cv::IMREAD_COLOR);
}

/// \brief 当前帧转为 QImage
QImage SessionController::load_current_qimage() const {
  return mat_bgr_to_qimage(current_bgr());
}

/// \brief 组装标定求解配置字典
std::map<std::string, std::string> SessionController::solve_config_map() const {
  std::map<std::string, std::string> m = solve_options_;
  m["squares_x"] = std::to_string(squares_x_);
  m["squares_y"] = std::to_string(squares_y_);
  m["square_length"] = std::to_string(square_length_m_);
  if (!m.count("target")) {
    m["target"] = is_trihedral() ? "trihedral_charuco" : "chessboard";
  }
  if (!m.count("model")) {
    m["model"] = "brown_conrady";
  }
  m["method"] = handeye_method_.toStdString();
  if (!camera_yaml_.isEmpty()) {
    m["camera_yaml"] = camera_yaml_.toStdString();
    // 双目外参：若未单独指定左 YAML，则复用 camera_yaml
    if (is_stereo_extrinsics() && !m.count("left_camera_yaml")) {
      m["left_camera_yaml"] = camera_yaml_.toStdString();
    }
  }
  if (calibrator_id_ == QStringLiteral("eye_in_hand")) {
    m["parent_frame"] = "gripper";
    m["child_frame"] = "camera";
  } else if (calibrator_id_ == QStringLiteral("eye_to_hand")) {
    m["parent_frame"] = "base";
    m["child_frame"] = "camera";
  } else if (is_stereo_extrinsics()) {
    if (!m.count("parent_frame") || m["parent_frame"].empty()) {
      m["parent_frame"] = "left";
    }
    if (!m.count("child_frame") || m["child_frame"].empty()) {
      m["child_frame"] = "right";
    }
  }
  return m;
}

/// \brief 结果父坐标系名
QString SessionController::result_parent_frame() const {
  if (last_result_.intrinsics_meta.count("parent_frame")) {
    return QString::fromStdString(last_result_.intrinsics_meta.at("parent_frame"));
  }
  if (is_stereo_extrinsics()) {
    return QStringLiteral("left");
  }
  return is_handeye()
      ? (calibrator_id_ == QStringLiteral("eye_to_hand") ? QStringLiteral("base")
                                                         : QStringLiteral("gripper"))
      : QString();
}

/// \brief 结果子坐标系名
QString SessionController::result_child_frame() const {
  if (last_result_.intrinsics_meta.count("child_frame")) {
    return QString::fromStdString(last_result_.intrinsics_meta.at("child_frame"));
  }
  if (is_stereo_extrinsics()) {
    return QStringLiteral("right");
  }
  return is_handeye() ? QStringLiteral("camera") : QString();
}

/// \brief 最近一次有效检测的图像点数
int SessionController::last_point_count() const {
  if (!has_detection_) {
    return 0;
  }
  return static_cast<int>(current_corr_.image_points.rows());
}

/// \brief 同步检测当前帧并更新预览/置信度
bool SessionController::detect_current(QImage *preview_out, QString *error_out, bool fast) {
  DetectJobIn in;
  in.bgr = current_bgr().clone();
  in.trihedral = is_trihedral();
  in.ros_mode = (source_mode_ == SourceMode::RosTopic);
  in.fast = fast;
  in.squares_x = squares_x_;
  in.squares_y = squares_y_;
  in.square_length_m = square_length_m_;
  in.solve_options = solve_config_map();
  in.viz_corners = viz_corners_;
  in.viz_hull = viz_hull_;
  in.viz_conf = viz_conf_;
  in.viz_aruco = viz_aruco_;
  in.viz_marker_radius = viz_marker_radius_;
  in.camera_matrix = detect_K_.empty() ? cv::Mat() : detect_K_.clone();
  in.dist_coeffs = detect_D_.empty() ? cv::Mat() : detect_D_.clone();
  in.camera_model = detect_model_;
  in.camera_xi = detect_xi_;
  in.has_chess_roi = has_chess_track_roi_;
  in.chess_roi = chess_track_roi_;
  in.chess_lost_frames = chess_lost_frames_;
  const DetectJobOut out = run_detect_job(std::move(in));
  last_preview_ = out.preview;
  last_confidence_ = out.confidence;
  last_aruco_reproj_px_ = out.aruco_reproj_px;
  last_faces_found_ = out.faces;
  detect_width_ = out.width;
  detect_height_ = out.height;
  if (out.ok) {
    current_corr_ = out.corr;
    has_detection_ = true;
    last_fp_.area_ratio = out.fp.area_ratio;
    last_fp_.cx = out.fp.cx;
    last_fp_.cy = out.fp.cy;
    last_fp_.tilt_deg = out.fp.tilt_deg;
    has_chess_track_roi_ = out.has_chess_roi;
    chess_track_roi_ = out.chess_roi;
    chess_lost_frames_ = out.chess_lost_frames;
  } else {
    has_detection_ = false;
    has_chess_track_roi_ = out.has_chess_roi;
    chess_track_roi_ = out.chess_roi;
    chess_lost_frames_ = out.chess_lost_frames;
  }
  update_board_metrics_after_detect();
  last_preview_ = decorate_intrinsics_preview(last_preview_, fast);
  if (preview_out) {
    *preview_out = last_preview_;
  }
  if (error_out) {
    *error_out = out.error;
  }
  return out.ok;
}

int SessionController::live_detect_interval_ms() const {
  // 实时预览不限频；忙时由 detect_busy / stereo_detect_busy 自然背压
  return 0;
}

/// \brief 请求后台检测（忙则挂起一次）
void SessionController::request_detect(bool fast) {
  if (detect_busy_.load()) {
    pending_detect_ = true;
    pending_fast_ = fast;
    return;
  }
  start_detect_job(fast);
}

/// \brief 冻结/离开工作台时：丢掉排队任务并作废进行中的结果
void SessionController::cancel_pending_detect() {
  pending_detect_ = false;
  detect_epoch_.fetch_add(1);
  stereo_detect_epoch_.fetch_add(1);
  // 进行中的线程回主线程后会因 epoch 不匹配直接丢弃；此处先解除 busy，避免 UI 卡在「检测中」
  detect_busy_.store(false);
  stereo_detect_busy_.store(false);
}

void SessionController::clear_live_ros_frames() {
  live_bgr_.release();
  live_left_bgr_.release();
  live_right_bgr_.release();
  last_stereo_sync_delta_ms_ = -1;
  has_detection_ = false;
  last_preview_ = {};
  stereo_left_detect_ = {};
  stereo_right_detect_ = {};
  {
    std::lock_guard<std::mutex> lock(live_preview_mutex_);
    cached_live_preview_qimage_ = {};
  }
  {
    std::lock_guard<std::mutex> lock(stereo_live_preview_mutex_);
    cached_stereo_live_left_ = {};
    cached_stereo_live_right_ = {};
  }
  live_preview_busy_.store(false);
  stereo_live_preview_busy_.store(false);
}

/// \brief 请求后台类型识别（忙则忽略，避免与检测队列搅在一起）
void SessionController::request_identify(
    const core::BoardTypeIdentifyOptions &options) {
  if (detect_busy_.load()) {
    return;
  }
  pending_detect_ = false;
  detect_lab_mode_ = DetectLabMode::Identify;
  start_identify_job(options);
}

/// \brief 启动后台类型识别线程
void SessionController::start_identify_job(
    const core::BoardTypeIdentifyOptions &options) {
  cv::Mat bgr = current_bgr().clone();
  const bool ros_mode = (source_mode_ == SourceMode::RosTopic);
  if (bgr.empty()) {
    last_identify_ranked_.clear();
    last_identify_message_ = ros_mode ? QStringLiteral("尚无 ROS 图像帧")
                                      : QStringLiteral("无当前图片");
    last_preview_ = {};
    emit identify_started();
    emit identify_finished(false, last_identify_message_);
    return;
  }
  if (detect_thread_.joinable()) {
    detect_thread_.join();
  }
  detect_busy_.store(true);
  const uint64_t epoch = detect_epoch_.load();
  emit identify_started();
  detect_thread_ = std::thread([this, bgr = std::move(bgr), options, epoch]() mutable {
    const core::BoardTypeIdentifyResult result =
        core::BoardTypeIdentifier().identify(bgr, options);
    QImage preview;
    if (!result.overlay_bgr.empty()) {
      preview = mat_bgr_to_qimage(result.overlay_bgr);
    }
    QMetaObject::invokeMethod(
        this,
        [this, result, preview, epoch]() {
          if (epoch != detect_epoch_.load()) {
            detect_busy_.store(false);
            return;
          }
          last_identify_ranked_ = result.ranked;
          last_identify_message_ = QString::fromStdString(result.message);
          last_preview_ = preview;
          has_detection_ = false;
          detect_busy_.store(false);
          emit identify_finished(result.ok(), last_identify_message_);
        },
        Qt::QueuedConnection);
  });
}

/// \brief 导出最近一次识别结果 JSON
bool SessionController::export_identify_json(
    const QString &path, QString *error_out) const {
  if (path.trimmed().isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("路径为空");
    }
    return false;
  }
  if (last_identify_ranked_.empty()) {
    if (error_out) {
      *error_out = QStringLiteral("尚无识别结果可导出");
    }
    return false;
  }
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    if (error_out) {
      *error_out = QStringLiteral("无法写入：%1").arg(path);
    }
    return false;
  }
  const QDateTime now = QDateTime::currentDateTimeUtc();
  QString json;
  json += QStringLiteral("{\n");
  json += QStringLiteral("  \"timestamp_utc\": \"%1\",\n")
              .arg(now.toString(Qt::ISODate));
  json += QStringLiteral("  \"source\": \"%1\",\n")
              .arg(current_path().isEmpty() ? QStringLiteral("live")
                                            : current_path());
  json += QStringLiteral("  \"message\": \"%1\",\n")
              .arg(QString(last_identify_message_)
                       .replace(QLatin1Char('\\'), QStringLiteral("\\\\"))
                       .replace(QLatin1Char('"'), QStringLiteral("\\\"")));
  json += QStringLiteral("  \"hypotheses\": [\n");
  for (size_t i = 0; i < last_identify_ranked_.size(); ++i) {
    const auto &h = last_identify_ranked_[i];
    json += QStringLiteral("    {\n");
    json += QStringLiteral("      \"type_id\": \"%1\",\n")
                .arg(QString::fromStdString(h.type_id));
    json += QStringLiteral("      \"score\": %1,\n")
                .arg(h.score, 0, 'f', 4);
    json += QStringLiteral("      \"feature_count\": %1,\n")
                .arg(h.feature_count);
    json += QStringLiteral("      \"dict_hint\": \"%1\",\n")
                .arg(QString::fromStdString(h.dict_hint));
    json += QStringLiteral("      \"note\": \"%1\"\n")
                .arg(QString::fromStdString(h.note)
                         .replace(QLatin1Char('\\'), QStringLiteral("\\\\"))
                         .replace(QLatin1Char('"'), QStringLiteral("\\\"")));
    json += QStringLiteral("    }%1\n")
                .arg(i + 1 < last_identify_ranked_.size() ? QStringLiteral(",")
                                                          : QString());
  }
  json += QStringLiteral("  ]\n}\n");
  f.write(json.toUtf8());
  f.close();
  return true;
}

/// \brief 启动后台检测线程，结果经信号回主线程
void SessionController::start_detect_job(bool fast) {
  DetectJobIn in;
  in.bgr = current_bgr().clone();
  in.trihedral = is_trihedral();
  in.ros_mode = (source_mode_ == SourceMode::RosTopic);
  in.fast = fast;
  in.squares_x = squares_x_;
  in.squares_y = squares_y_;
  in.square_length_m = square_length_m_;
  in.solve_options = solve_config_map();
  in.viz_corners = viz_corners_;
  in.viz_hull = viz_hull_;
  in.viz_conf = viz_conf_;
  in.viz_aruco = viz_aruco_;
  in.viz_marker_radius = viz_marker_radius_;
  in.camera_matrix = detect_K_.empty() ? cv::Mat() : detect_K_.clone();
  in.dist_coeffs = detect_D_.empty() ? cv::Mat() : detect_D_.clone();
  in.camera_model = detect_model_;
  in.camera_xi = detect_xi_;
  in.has_chess_roi = has_chess_track_roi_;
  in.chess_roi = chess_track_roi_;
  in.chess_lost_frames = chess_lost_frames_;
  if (in.bgr.empty()) {
    has_detection_ = false;
    last_preview_ = {};
    emit detect_started();
    emit detect_finished(
        false, in.ros_mode ? QStringLiteral("尚无 ROS 图像帧，请确认话题有数据")
                           : QStringLiteral("无当前图片"));
    return;
  }
  if (detect_thread_.joinable()) {
    detect_thread_.join();
  }
  detect_busy_.store(true);
  const uint64_t epoch = detect_epoch_.load();
  emit detect_started();
  // —— 后台线程跑检测，结果排队回主线程 ——
  detect_thread_ = std::thread([this, in = std::move(in), epoch]() mutable {
    const bool fast = in.fast;
    const DetectJobOut out = run_detect_job(std::move(in));
    QMetaObject::invokeMethod(
        this,
        [this, out, epoch, fast]() {
          if (epoch != detect_epoch_.load()) {
            detect_busy_.store(false);
            pending_detect_ = false;
            return;
          }
          last_preview_ = out.preview;
          last_confidence_ = out.confidence;
          last_aruco_reproj_px_ = out.aruco_reproj_px;
          last_faces_found_ = out.faces;
          detect_width_ = out.width;
          detect_height_ = out.height;
          if (out.ok) {
            current_corr_ = out.corr;
            has_detection_ = true;
            last_fp_.area_ratio = out.fp.area_ratio;
            last_fp_.cx = out.fp.cx;
            last_fp_.cy = out.fp.cy;
            last_fp_.tilt_deg = out.fp.tilt_deg;
            has_chess_track_roi_ = out.has_chess_roi;
            chess_track_roi_ = out.chess_roi;
            chess_lost_frames_ = out.chess_lost_frames;
          } else {
            has_detection_ = false;
            has_chess_track_roi_ = out.has_chess_roi;
            chess_track_roi_ = out.chess_roi;
            chess_lost_frames_ = out.chess_lost_frames;
          }
          update_board_metrics_after_detect();
          last_preview_ = decorate_intrinsics_preview(last_preview_, fast);
          detect_busy_.store(false);
          emit detect_finished(out.ok, out.error);
          // 仅在仍排队且 epoch 未被 cancel 作废时续跑
          if (pending_detect_ && epoch == detect_epoch_.load()) {
            pending_detect_ = false;
            start_detect_job(pending_fast_);
          } else {
            pending_detect_ = false;
          }
        },
        Qt::QueuedConnection);
  });
}

/// \brief 由角点生成视角指纹
SessionController::ViewFingerprint SessionController::fingerprint_from_corners(
    const std::vector<cv::Point2f> &corners, int width, int height) const {
  ViewFingerprint fp;
  if (corners.empty() || width <= 0 || height <= 0) {
    return fp;
  }
  cv::Rect box = cv::boundingRect(corners);
  fp.area_ratio =
      static_cast<double>(box.area()) / (static_cast<double>(width) * height);
  fp.cx = (box.x + box.width * 0.5) / static_cast<double>(width);
  fp.cy = (box.y + box.height * 0.5) / static_cast<double>(height);
  fp.tilt_deg = std::atan2(static_cast<double>(box.height),
                           static_cast<double>(std::max(1, box.width))) *
                180.0 / CV_PI;
  return fp;
}

/// \brief 由角点估计检测置信度
double SessionController::confidence_from_corners(
    const std::vector<cv::Point2f> &corners, int width, int height) const {
  const int expect_face = squares_x_ * squares_y_;
  if (corners.size() < 4 || expect_face <= 0 || width <= 0 || height <= 0) {
    return 0.0;
  }
  const ViewFingerprint fp = fingerprint_from_corners(corners, width, height);

  // —— 按靶标类型计分 ——
  // detect_completeness=full：只认整面；partial：三面/ChArUco/ArUco/棋盘局部均可
  double score = 0.0;
  const std::string target_type =
      solve_options_.count("target") ? solve_options_.at("target") : "chessboard";
  const std::string completeness = solve_options_.count("detect_completeness")
      ? solve_options_.at("detect_completeness")
      : "";
  const bool require_full =
      completeness == "full" || completeness == "complete";
  const bool force_partial = completeness == "partial";
  const bool aruco_loose =
      target_type == "aruco" || target_type == "aruco_single";
  const bool is_charuco =
      target_type == "charuco" || target_type == "trihedral_charuco";
  const bool is_aruco_grid = target_type == "aruco_grid";
  const bool is_aprilgrid = target_type == "aprilgrid";
  int expect_pts = expect_face;
  if (is_charuco) {
    expect_pts = std::max(4, (squares_x_ - 1) * (squares_y_ - 1));
  } else if (is_aruco_grid || is_aprilgrid) {
    expect_pts = std::max(4, expect_face * 4);
  }
  const bool partial_ok =
      !require_full &&
      (is_trihedral() || is_charuco || is_aruco_grid || is_aprilgrid || aruco_loose ||
       (force_partial && target_type == "chessboard"));
  if (partial_ok) {
    if (corners.size() < (aruco_loose ? 4u : 6u)) {
      return 0.0;
    }
    if (aruco_loose) {
      const int n_markers = static_cast<int>(corners.size()) / 4;
      score = std::min(0.95, 0.55 + 0.08 * static_cast<double>(n_markers));
    } else {
      const double fill =
          std::min(1.0, static_cast<double>(corners.size()) /
                            static_cast<double>(std::max(6, expect_pts)));
      score = 0.15 + 0.50 * fill;
    }
  } else {
    if (aruco_loose) {
      if (corners.size() < 4) {
        return 0.0;
      }
      score = 0.55;
    } else {
      const int need = std::max(4, static_cast<int>(0.95 * expect_pts + 0.5));
      if (static_cast<int>(corners.size()) < need) {
        return 0.0;
      }
      score = 0.45;
    }
  }

  // Prefer mid-size board (not too far / too close)
  if (fp.area_ratio >= 0.04 && fp.area_ratio <= 0.45) {
    score += 0.25;
    if (fp.area_ratio >= 0.08 && fp.area_ratio <= 0.30) {
      score += 0.10;
    }
  }

  // Keep away from image border
  cv::Rect box = cv::boundingRect(corners);
  const double margin = 0.04 * std::min(width, height);
  if (box.x >= margin && box.y >= margin &&
      box.x + box.width <= width - margin &&
      box.y + box.height <= height - margin) {
    score += 0.15;
  }

  // Aspect roughly matches board
  const double expect_aspect =
      static_cast<double>(squares_x_) / static_cast<double>(std::max(1, squares_y_));
  const double aspect =
      static_cast<double>(std::max(1, box.width)) / std::max(1, box.height);
  const double aspect_err =
      std::abs(aspect - expect_aspect) / std::max(expect_aspect, 1e-6);
  if (aspect_err < 0.35) {
    score += 0.15;
  }

  return std::min(1.0, score);
}

/// \brief 判断视角相对已采集是否足够多样
bool SessionController::is_diverse_enough(
    const ViewFingerprint &fp, double min_diversity) const {
  for (const auto &prev : captured_fps_) {
    const double d =
        std::abs(fp.area_ratio - prev.area_ratio) * 2.5 +
        std::abs(fp.cx - prev.cx) +
        std::abs(fp.cy - prev.cy) +
        std::abs(fp.tilt_deg - prev.tilt_deg) / 90.0;
    if (d < min_diversity) {
      return false;
    }
  }
  return true;
}

/// \brief 是否对内参任务启用 Tier4 采集过滤
bool SessionController::uses_intrinsics_capture_filter() const {
  return uses_tier4_intrinsics();
}

bool SessionController::uses_tier4_intrinsics() const {
  if (!is_intrinsics()) {
    return false;
  }
  return core::tier4_intrinsics_enabled(solve_options_);
}

void SessionController::set_intrinsics_profile_id(const std::string &profile_id) {
  solve_options_["intrinsics_profile"] = profile_id;
  configure_intrinsics_engine();
  emit intrinsics_state_changed();
}

std::string SessionController::intrinsics_profile_id() const {
  const auto it = solve_options_.find("intrinsics_profile");
  if (it != solve_options_.end()) {
    return it->second;
  }
  return "classic";
}

/// \brief 用已采帧刷新临时内参（供采集过滤）
void SessionController::refresh_provisional_intrinsics() {
  request_provisional_intrinsics_refresh(true);
}

void SessionController::request_provisional_intrinsics_refresh(bool force) {
  if (!uses_intrinsics_capture_filter() || detect_width_ <= 0 ||
      detect_height_ <= 0) {
    provisional_intrinsics_.valid = false;
    return;
  }
  const int count = observation_count();
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  constexpr qint64 kMinIntervalMs = 2500;
  if (!force && count < 6) {
    return;
  }
  if (!force && now - last_provisional_refresh_ms_ < kMinIntervalMs) {
    provisional_refresh_dirty_.store(true);
    return;
  }
  if (provisional_refresh_busy_.exchange(true)) {
    provisional_refresh_dirty_.store(true);
    return;
  }
  last_provisional_sample_count_ = count;
  last_provisional_refresh_ms_ = now;

  const int w = detect_width_;
  const int h = detect_height_;
  core::ObservationBatch batch =
      is_intrinsics() && uses_tier4_intrinsics()
          ? intrinsics_state_.training_batch()
          : batch_;
  const core::IntrinsicsProfile profile =
      core::profile_from_config_map(solve_options_);
  const auto params = core::collector_params_from_config(solve_options_, profile);
  const int max_fast = params.max_fast_calibration_samples;

  std::thread([this, batch = std::move(batch), w, h, profile, max_fast]() {
    core::ProvisionalIntrinsics prov;
    core::update_provisional_intrinsics(
        batch, w, h, profile, &prov, max_fast);
    QMetaObject::invokeMethod(
        this,
        [this, prov]() {
          provisional_intrinsics_ = prov;
          if (is_intrinsics()) {
            intrinsics_state_.set_provisional_model(prov);
          }
          provisional_refresh_busy_.store(false);
          if (provisional_refresh_dirty_.exchange(false)) {
            request_provisional_intrinsics_refresh(true);
            return;
          }
          emit intrinsics_state_changed();
        },
        Qt::QueuedConnection);
  }).detach();
}

core::IntrinsicsSessionState &SessionController::intrinsics_state() {
  if (uses_stereo_dual_session() && uses_tier4_intrinsics()) {
    const QString mode = stereo_capture_mode();
    if (mode == QStringLiteral("right")) {
      return intrinsics_right_state_;
    }
    return intrinsics_left_state_;
  }
  return intrinsics_state_;
}

const core::IntrinsicsSessionState &SessionController::intrinsics_state() const {
  if (uses_stereo_dual_session() && uses_tier4_intrinsics()) {
    const QString mode = stereo_capture_mode();
    if (mode == QStringLiteral("right")) {
      return intrinsics_right_state_;
    }
    return intrinsics_left_state_;
  }
  return intrinsics_state_;
}

core::ObservationBatch SessionController::evaluation_batch() const {
  if (!is_intrinsics()) {
    return {};
  }
  return intrinsics_state_.evaluation_batch();
}

int SessionController::observation_count() const {
  if (uses_stereo_dual_session() && uses_tier4_intrinsics()) {
    return stereo_left_sample_count() + stereo_right_sample_count();
  }
  if (is_intrinsics() && uses_tier4_intrinsics()) {
    return intrinsics_state_.collector().training_count();
  }
  return static_cast<int>(batch_.items.size());
}

bool SessionController::can_solve() const {
  if (uses_stereo_dual_session()) {
    if (uses_tier4_intrinsics()) {
      return stereo_left_sample_count() >= 3 && stereo_right_sample_count() >= 3;
    }
    return stereo_pair_count() >= 3;
  }
  return observation_count() >= 3;
}

void SessionController::request_offline_batch_ingest() {
  if (source_mode_ != SourceMode::Offline || offline_ingest_busy_.load()) {
    return;
  }
  if (observation_count() > 0) {
    return;
  }
  const bool stereo = uses_stereo_dual_session();
  const int n = stereo ? std::min(stereo_left_paths_.size(), stereo_right_paths_.size())
                       : static_cast<int>(image_paths_.size());
  if (n <= 0) {
    return;
  }

  offline_ingest_busy_.store(true);
  emit offline_ingest_started(n);
  DetectJobIn tmpl;
  tmpl.trihedral = is_trihedral();
  tmpl.ros_mode = false;
  tmpl.fast = false;
  tmpl.squares_x = squares_x_;
  tmpl.squares_y = squares_y_;
  tmpl.square_length_m = square_length_m_;
  tmpl.solve_options = solve_config_map();
  tmpl.viz_corners = viz_corners_;
  tmpl.viz_hull = viz_hull_;
  tmpl.viz_conf = viz_conf_;
  tmpl.viz_aruco = viz_aruco_;
  tmpl.viz_marker_radius = viz_marker_radius_;
  tmpl.camera_matrix = detect_K_.empty() ? cv::Mat() : detect_K_.clone();
  tmpl.dist_coeffs = detect_D_.empty() ? cv::Mat() : detect_D_.clone();
  tmpl.camera_model = detect_model_;
  tmpl.camera_xi = detect_xi_;
  tmpl.has_chess_roi = has_chess_track_roi_;
  tmpl.chess_roi = chess_track_roi_;
  tmpl.chess_lost_frames = chess_lost_frames_;

  std::thread([this, stereo, n, tmpl]() {
    struct MonoItem {
      QString path;
      DetectJobOut out;
    };
    struct StereoItem {
      QString left_path;
      QString right_path;
      DetectJobOut left;
      DetectJobOut right;
    };
    std::vector<MonoItem> mono_items;
    std::vector<StereoItem> stereo_items;
    mono_items.reserve(static_cast<size_t>(n));
    stereo_items.reserve(static_cast<size_t>(n));

    if (stereo) {
      for (int i = 0; i < n; ++i) {
        StereoItem item;
        item.left_path = stereo_left_paths_.at(i);
        item.right_path = stereo_right_paths_.at(i);
        cv::Mat left = cv::imread(item.left_path.toStdString(), cv::IMREAD_COLOR);
        cv::Mat right = cv::imread(item.right_path.toStdString(), cv::IMREAD_COLOR);
        if (left.empty() || right.empty()) {
          continue;
        }
        DetectJobIn lin = tmpl;
        lin.bgr = std::move(left);
        item.left = run_detect_job(std::move(lin));
        DetectJobIn rin = tmpl;
        rin.bgr = std::move(right);
        item.right = run_detect_job(std::move(rin));
        if (item.left.ok && item.right.ok) {
          stereo_items.push_back(std::move(item));
        }
      }
    } else {
      for (int i = 0; i < n; ++i) {
        MonoItem item;
        item.path = image_paths_.at(i);
        cv::Mat bgr = cv::imread(item.path.toStdString(), cv::IMREAD_COLOR);
        if (bgr.empty()) {
          continue;
        }
        DetectJobIn in = tmpl;
        in.bgr = std::move(bgr);
        item.out = run_detect_job(std::move(in));
        if (item.out.ok) {
          mono_items.push_back(std::move(item));
        }
      }
    }

    QMetaObject::invokeMethod(
        this,
        [this, stereo, mono_items = std::move(mono_items),
         stereo_items = std::move(stereo_items), n]() {
          int added = 0;
          if (stereo) {
            for (const auto &item : stereo_items) {
              QString err;
              if (!add_side_observation(
                      "left", item.left.corr, item.left.width, item.left.height,
                      item.left_path, &err)) {
                continue;
              }
              if (!add_side_observation(
                      "right", item.right.corr, item.right.width, item.right.height,
                      item.right_path, &err)) {
                continue;
              }
              StereoPairRecord rec;
              rec.pair_id = next_stereo_pair_id_++;
              rec.left_source_path =
                  std::string("left:") + item.left_path.toStdString();
              rec.right_source_path =
                  std::string("right:") + item.right_path.toStdString();
              if (!item.left_path.isEmpty() &&
                  !item.left_path.startsWith(QStringLiteral("ros://"))) {
                rec.left_image_path = item.left_path.toStdString();
              }
              if (!item.right_path.isEmpty() &&
                  !item.right_path.startsWith(QStringLiteral("ros://"))) {
                rec.right_image_path = item.right_path.toStdString();
              }
              rec.timestamp_delta_ms = 0;
              stereo_pairs_.push_back(rec);
              ++added;
            }
            if (uses_tier4_intrinsics()) {
              batch_ = intrinsics_left_state_.training_batch();
              for (const auto &obs : intrinsics_right_state_.training_batch().items) {
                batch_.items.push_back(obs);
              }
              emit intrinsics_state_changed();
            }
          } else {
            for (const auto &item : mono_items) {
              QString err;
              if (add_observation_from_detect(
                      item.path, item.out.corr, item.out.width, item.out.height,
                      item.out.preview, &err)) {
                ++added;
              }
            }
          }
          offline_ingest_busy_.store(false);
          emit offline_ingest_finished(added, n - added);
          if (added > 0) {
            emit observations_changed();
          }
        },
        Qt::QueuedConnection);
  }).detach();
}

int SessionController::training_sample_count() const {
  return is_intrinsics() && uses_tier4_intrinsics()
             ? intrinsics_state_.collector().training_count()
             : static_cast<int>(batch_.items.size());
}

int SessionController::evaluation_sample_count() const {
  return is_intrinsics() && uses_tier4_intrinsics()
             ? intrinsics_state_.collector().evaluation_count()
             : 0;
}

void SessionController::set_intrinsics_solver_kind(const std::string &solver) {
  solve_options_["intrinsics_solver"] = solver;
  configure_intrinsics_engine();
}

std::string SessionController::intrinsics_solver_kind() const {
  const auto it = solve_options_.find("intrinsics_solver");
  if (it != solve_options_.end()) {
    return it->second;
  }
  const core::IntrinsicsProfile profile =
      core::profile_from_config_map(solve_options_);
  return profile.solver == core::IntrinsicsSolverKind::Ceres ? "ceres" : "opencv";
}

void SessionController::configure_intrinsics_engine() {
  if (!is_intrinsics()) {
    return;
  }
  if (!uses_tier4_intrinsics()) {
    return;
  }
  if (uses_stereo_dual_session()) {
    configure_stereo_intrinsics_states();
    return;
  }
  core::merge_tier4_intrinsics_defaults(&solve_options_);
  if (source_mode_ == SourceMode::RosTopic) {
    solve_options_["collector_filter_speed"] = "false";
  }
  core::IntrinsicsProfile profile =
      core::profile_from_config_map(solve_options_);
  const auto it_solver = solve_options_.find("intrinsics_solver");
  if (it_solver != solve_options_.end()) {
    const std::string s = it_solver->second;
    if (s == "ceres" || s == "Ceres") {
      profile.solver = core::IntrinsicsSolverKind::Ceres;
    } else {
      profile.solver = core::IntrinsicsSolverKind::OpenCV;
    }
  }
  const auto collector_params =
      core::collector_params_from_config(solve_options_, profile);
  intrinsics_state_.configure(
      profile, collector_params, solve_config_map());
  intrinsics_state_.set_offline_source(
      source_mode_ == SourceMode::Offline || source_mode_ == SourceMode::RosBag);
}

void SessionController::sync_batch_from_intrinsics() {
  if (!is_intrinsics()) {
    return;
  }
  batch_ = intrinsics_state_.training_batch();
}

double SessionController::compute_pixel_speed() const {
  if (!has_last_frame_centroid_ || detect_width_ <= 0 || detect_height_ <= 0) {
    return 0.0;
  }
  const double cx = last_fp_.cx * detect_width_;
  const double cy = last_fp_.cy * detect_height_;
  const double dx = cx - last_frame_centroid_.x;
  const double dy = cy - last_frame_centroid_.y;
  return std::hypot(dx, dy);
}

void SessionController::update_board_metrics_after_detect() {
  if (!uses_tier4_intrinsics()) {
    return;
  }
  if (!has_detection_) {
    last_board_metrics_ = {};
    emit intrinsics_state_changed();
    return;
  }
  intrinsics_state_.update_frame_metrics(
      current_corr_, detect_width_, detect_height_, square_length_m_);
  last_board_metrics_ = intrinsics_state_.last_metrics();
  const int cells = std::max(
      4, intrinsics_state_.collector().params().heatmap_cells);
  accumulate_linearity_heatmap(
      current_corr_, detect_width_, detect_height_, cells,
      &intrinsics_linearity_grid_);
  emit intrinsics_state_changed();
}

bool SessionController::tier4_preview_needs_overlay() const {
  if (!uses_tier4_intrinsics()) {
    return false;
  }
  const auto &v = intrinsics_viz_options_;
  return v.draw_training_points || v.draw_evaluation_points ||
         v.draw_training_occupancy || v.draw_evaluation_occupancy ||
         v.draw_linearity_error || v.draw_indicators ||
         intrinsics_view_mode_ != IntrinsicsImageViewMode::Source;
}

bool SessionController::evaluate_calibration(QString *error_out) {
  if (!is_intrinsics()) {
    if (error_out) {
      *error_out = QStringLiteral("仅内参任务支持评估");
    }
    return false;
  }
  std::string err;
  if (!intrinsics_state_.evaluate(&err)) {
    if (error_out) {
      *error_out = QString::fromStdString(err);
    }
    return false;
  }
  last_result_ = intrinsics_state_.last_result();
  emit intrinsics_state_changed();
  emit result_changed();
  return true;
}

/// \brief 高置信且多样时自动入库当前观测
bool SessionController::try_auto_capture(
    double min_confidence, double min_diversity, QString *error_out) {
  if (!has_detection_ || last_confidence_ < min_confidence) {
    if (error_out) {
      *error_out = QStringLiteral("置信度不足");
    }
    return false;
  }
  if (uses_tier4_intrinsics()) {
    return add_current_observation(error_out);
  }
  if (!is_diverse_enough(last_fp_, min_diversity)) {
    if (error_out) {
      *error_out = QStringLiteral("与已采集姿态过于相似");
    }
    return false;
  }
  return add_current_observation(error_out);
}

/// \brief 手眼：从 CSV/TF 附加 T_base_gripper
bool SessionController::attach_pose_to_observation(
    core::Observation *obs, QString *error_out) {
  if (!is_handeye()) {
    return true;
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  // —— 按位姿源取 T_base_gripper ——
  if (pose_source_ == PoseSource::Csv) {
    const QString key = (source_mode_ == SourceMode::RosTopic)
        ? QStringLiteral("live_%1").arg(live_seq_ + 1)
        : current_path();
    if (source_mode_ == SourceMode::RosTopic) {
      // 在线+CSV：允许用序号键 live_N；更常见是 TF
      if (!pose_csv_.get(key, &T) && !pose_csv_.get(current_path(), &T)) {
        if (error_out) {
          *error_out = QStringLiteral("CSV 中无当前帧位姿（在线建议改用 TF）");
        }
        return false;
      }
    } else if (!pose_csv_.get(current_path(), &T)) {
      if (error_out) {
        *error_out = QStringLiteral("CSV 中无图片 %1 的位姿")
                         .arg(QFileInfo(current_path()).fileName());
      }
      return false;
    }
  } else if (pose_source_ == PoseSource::Tf) {
    if (tf_bridge_ == nullptr || !tf_bridge_->lookup(&T, error_out)) {
      if (error_out && error_out->isEmpty()) {
        *error_out = QStringLiteral("TF 查询失败");
      }
      return false;
    }
  } else {
    if (error_out) {
      *error_out = QStringLiteral("手眼模式请选择位姿源（CSV 或 TF）");
    }
    return false;
  }
  obs->has_base_gripper = true;
  obs->T_base_gripper = T;
  return true;
}

/// \brief 将指定检测结果加入观测批次（离线批量入库用）
bool SessionController::add_observation_from_detect(
    const QString &path,
    const core::Correspondence &corr,
    int width,
    int height,
    const QImage &overlay,
    QString *error_out) {
  if (corr.image_points.rows() < 6) {
    if (error_out) {
      *error_out = QStringLiteral("有效角点不足 6 个，无法用于 calibrateCamera");
    }
    return false;
  }
  const std::string stored_path = path.toStdString();
  for (const auto &obs : batch_.items) {
    if (obs.source_path == stored_path) {
      if (error_out) {
        *error_out = QStringLiteral("该图片已在观测列表中");
      }
      return false;
    }
  }

  core::Observation obs;
  obs.source_path = stored_path;
  obs.frame_id = is_stereo_side_tagged() ? "left" : "camera";
  obs.image_width = width;
  obs.image_height = height;
  obs.correspondences = {corr};

  if (uses_intrinsics_capture_filter()) {
    const core::IntrinsicsProfile profile =
        core::profile_from_config_map(solve_options_);
    cv::Mat K = provisional_intrinsics_.valid
        ? provisional_intrinsics_.camera_matrix
        : core::make_initial_camera_matrix(width, height);
    cv::Mat D = provisional_intrinsics_.valid
        ? provisional_intrinsics_.dist_coeffs
        : core::make_initial_dist_coeffs(profile.rational_coeffs);
    core::IntrinsicsView view;
    view.object_points.reserve(static_cast<size_t>(corr.image_points.rows()));
    view.image_points.reserve(static_cast<size_t>(corr.image_points.rows()));
    for (int r = 0; r < corr.image_points.rows(); ++r) {
      view.object_points.emplace_back(
          static_cast<float>(corr.object_points(r, 0)),
          static_cast<float>(corr.object_points(r, 1)),
          static_cast<float>(corr.object_points(r, 2)));
      view.image_points.emplace_back(
          static_cast<float>(corr.image_points(r, 0)),
          static_cast<float>(corr.image_points(r, 1)));
    }
    core::fill_view_fingerprint(&view, width, height);
    cv::Mat rv, tv;
    if (core::solve_board_pose(view, K, D, &rv, &tv)) {
      const auto stats = core::compute_reprojection_stats(view, K, D, rv, tv);
      obs.has_board_pose = true;
      for (int i = 0; i < 3; ++i) {
        obs.board_rvec(i) = rv.at<double>(i, 0);
        obs.board_tvec(i) = tv.at<double>(i, 0);
      }
      obs.board_reproj_rms = stats.rms;
      obs.board_reproj_max = stats.max;
      obs.board_center_x_norm = view.centroid_x;
      obs.board_center_y_norm = view.centroid_y;
      obs.board_tilt_deg = view.tilt_deg;
    }
  }
  if (!attach_pose_to_observation(&obs, error_out)) {
    return false;
  }

  if (is_intrinsics() && uses_tier4_intrinsics()) {
    core::BoardFrameFingerprint fp =
        core::fingerprint_from_correspondence(corr, width, height);
    if (obs.has_board_pose) {
      fp.has_pose = true;
      fp.position_m = cv::Vec3d(
          obs.board_tvec.x(), obs.board_tvec.y(), obs.board_tvec.z());
      fp.tilt_deg = obs.board_tilt_deg;
      fp.rough_angle_x_deg = obs.board_rvec.x() * 180.0 / CV_PI;
      fp.rough_angle_y_deg = obs.board_rvec.y() * 180.0 / CV_PI;
    }
    core::IntrinsicsSampleSplit split = core::IntrinsicsSampleSplit::Training;
    const auto reason =
        intrinsics_state_.try_capture(std::move(obs), fp, 0.0, &split);
    if (reason != core::CollectorRejectReason::Accepted) {
      if (error_out) {
        *error_out = QString::fromStdString(core::collector_reject_reason_text(reason));
      }
      return false;
    }
    sync_batch_from_intrinsics();
    request_provisional_intrinsics_refresh();
    CapturedView view;
    view.overlay = overlay;
    view.image_path = path;
    captured_views_.push_back(std::move(view));
    return true;
  }

  batch_.items.push_back(std::move(obs));
  request_provisional_intrinsics_refresh();
  CapturedView view;
  view.overlay = overlay;
  view.image_path = path;
  captured_views_.push_back(std::move(view));
  return true;
}

/// \brief 将当前检测结果加入观测批次
bool SessionController::add_current_observation(QString *error_out) {
  if (!has_detection_) {
    if (error_out) {
      *error_out = QStringLiteral("请先成功检测当前帧");
    }
    return false;
  }
  if (current_corr_.image_points.rows() < 6) {
    if (error_out) {
      *error_out = QStringLiteral("有效角点不足 6 个，无法用于 calibrateCamera");
    }
    return false;
  }

  QString path = current_path();
  std::string side = "left";
  if (is_stereo_side_tagged()) {
    const auto it = solve_options_.find("stereo_side");
    if (it != solve_options_.end() &&
        (it->second == "right" || it->second == "RIGHT" || it->second == "R")) {
      side = "right";
    }
  }
  if (source_mode_ == SourceMode::RosTopic) {
    path = QStringLiteral("ros://%1#%2")
               .arg(ros_topic_name_.isEmpty() ? QStringLiteral("image")
                                              : ros_topic_name_)
               .arg(++live_seq_);
  }
  std::string stored_path = path.toStdString();
  if (is_stereo_side_tagged()) {
    stored_path = side + ":" + stored_path;
  }
  if (source_mode_ != SourceMode::RosTopic) {
    for (const auto &obs : batch_.items) {
      if (obs.source_path == stored_path) {
        if (error_out) {
          *error_out = QStringLiteral("该图片已在观测列表中（同侧）");
        }
        return false;
      }
    }
  }

  // —— 组装观测并缓存原图/叠加 ——
  core::Observation obs;
  obs.source_path = stored_path;
  obs.frame_id = is_stereo_side_tagged() ? side : "camera";
  obs.image_width = detect_width_;
  obs.image_height = detect_height_;
  obs.correspondences = {current_corr_};
  if (uses_intrinsics_capture_filter()) {
    const core::IntrinsicsProfile profile =
        core::profile_from_config_map(solve_options_);
    cv::Mat K = provisional_intrinsics_.valid
        ? provisional_intrinsics_.camera_matrix
        : core::make_initial_camera_matrix(detect_width_, detect_height_);
    cv::Mat D = provisional_intrinsics_.valid
        ? provisional_intrinsics_.dist_coeffs
        : core::make_initial_dist_coeffs(profile.rational_coeffs);
    core::IntrinsicsView view;
    view.object_points.reserve(static_cast<size_t>(current_corr_.image_points.rows()));
    view.image_points.reserve(static_cast<size_t>(current_corr_.image_points.rows()));
    for (int r = 0; r < current_corr_.image_points.rows(); ++r) {
      view.object_points.emplace_back(
          static_cast<float>(current_corr_.object_points(r, 0)),
          static_cast<float>(current_corr_.object_points(r, 1)),
          static_cast<float>(current_corr_.object_points(r, 2)));
      view.image_points.emplace_back(
          static_cast<float>(current_corr_.image_points(r, 0)),
          static_cast<float>(current_corr_.image_points(r, 1)));
    }
    core::fill_view_fingerprint(&view, detect_width_, detect_height_);
    cv::Mat rv, tv;
    if (core::solve_board_pose(view, K, D, &rv, &tv)) {
      const auto stats = core::compute_reprojection_stats(view, K, D, rv, tv);
      obs.has_board_pose = true;
      for (int i = 0; i < 3; ++i) {
        obs.board_rvec(i) = rv.at<double>(i, 0);
        obs.board_tvec(i) = tv.at<double>(i, 0);
      }
      obs.board_reproj_rms = stats.rms;
      obs.board_reproj_max = stats.max;
      obs.board_center_x_norm = view.centroid_x;
      obs.board_center_y_norm = view.centroid_y;
      obs.board_tilt_deg = view.tilt_deg;
    }
  }
  if (!attach_pose_to_observation(&obs, error_out)) {
    return false;
  }

  if (is_intrinsics() && uses_tier4_intrinsics()) {
    core::BoardFrameFingerprint fp =
        core::fingerprint_from_correspondence(
            current_corr_, detect_width_, detect_height_);
    if (obs.has_board_pose) {
      fp.has_pose = true;
      fp.position_m = cv::Vec3d(
          obs.board_tvec.x(), obs.board_tvec.y(), obs.board_tvec.z());
      fp.tilt_deg = obs.board_tilt_deg;
      fp.rough_angle_x_deg = obs.board_rvec.x() * 180.0 / CV_PI;
      fp.rough_angle_y_deg = obs.board_rvec.y() * 180.0 / CV_PI;
    }
    core::IntrinsicsSampleSplit split = core::IntrinsicsSampleSplit::Training;
    const auto reason = intrinsics_state_.try_capture(
        std::move(obs), fp, compute_pixel_speed(), &split);
    if (reason != core::CollectorRejectReason::Accepted) {
      if (error_out) {
        *error_out = QString::fromStdString(core::collector_reject_reason_text(reason));
      }
      return false;
    }
    sync_batch_from_intrinsics();
    request_provisional_intrinsics_refresh();
    CapturedView view;
    fill_capture_view(&view);
    captured_views_.push_back(std::move(view));
    captured_fps_.push_back(last_fp_);
    has_last_frame_centroid_ = true;
    last_frame_centroid_ = cv::Point2f(
        static_cast<float>(last_fp_.cx * detect_width_),
        static_cast<float>(last_fp_.cy * detect_height_));
    if (source_mode_ == SourceMode::RosTopic) {
      has_detection_ = false;
    }
    emit observations_changed();
    emit intrinsics_state_changed();
    return true;
  }

  batch_.items.push_back(std::move(obs));
  if (has_detection_) {
    captured_fps_.push_back(last_fp_);
  }
  request_provisional_intrinsics_refresh();
  CapturedView view;
  fill_capture_view(&view);
  captured_views_.push_back(std::move(view));
  if (source_mode_ == SourceMode::RosTopic) {
    has_detection_ = false;
  }
  emit observations_changed();
  return true;
}

/// \brief 删除指定行观测及相关缓存
void SessionController::remove_observation(int row) {
  if (is_intrinsics() && uses_tier4_intrinsics()) {
    remove_intrinsics_sample(core::IntrinsicsSampleSplit::Training, row);
    return;
  }
  if (row < 0 || row >= static_cast<int>(batch_.items.size())) {
    return;
  }
  batch_.items.erase(batch_.items.begin() + row);
  if (row >= 0 && row < static_cast<int>(captured_fps_.size())) {
    captured_fps_.erase(captured_fps_.begin() + row);
  } else if (captured_fps_.size() > batch_.items.size()) {
    captured_fps_.resize(batch_.items.size());
  }
  if (row >= 0 && row < static_cast<int>(captured_views_.size())) {
    captured_views_.erase(captured_views_.begin() + row);
  } else if (captured_views_.size() > batch_.items.size()) {
    captured_views_.resize(batch_.items.size());
  }
  refresh_provisional_intrinsics();
  emit observations_changed();
}

void SessionController::remove_intrinsics_sample(
    core::IntrinsicsSampleSplit split, int row) {
  if (!is_intrinsics()) {
    return;
  }
  if (!intrinsics_state_.collector().remove(split, row)) {
    return;
  }
  sync_batch_from_intrinsics();
  if (split == core::IntrinsicsSampleSplit::Training && row >= 0 &&
      row < static_cast<int>(captured_views_.size())) {
    captured_views_.erase(captured_views_.begin() + row);
    if (row < static_cast<int>(captured_fps_.size())) {
      captured_fps_.erase(captured_fps_.begin() + row);
    }
  }
  refresh_provisional_intrinsics();
  emit observations_changed();
  emit intrinsics_state_changed();
}

/// \brief 清空全部观测与采集缓存
void SessionController::clear_observations() {
  batch_.items.clear();
  captured_fps_.clear();
  captured_views_.clear();
  clear_capture_cache();
  provisional_intrinsics_.valid = false;
  if (is_intrinsics()) {
    intrinsics_state_.reset();
    if (uses_stereo_dual_session()) {
      intrinsics_left_state_.reset();
      intrinsics_right_state_.reset();
      stereo_pairs_.clear();
      next_stereo_pair_id_ = 1;
      stereo_left_detect_ = {};
      stereo_right_detect_ = {};
    }
    has_last_frame_centroid_ = false;
    intrinsics_linearity_grid_.clear();
    emit intrinsics_state_changed();
  }
  emit observations_changed();
}

bool SessionController::has_observations() const {
  if (is_intrinsics() && uses_tier4_intrinsics()) {
    const auto &collector = intrinsics_state_.collector();
    if (collector.training_count() > 0 || collector.evaluation_count() > 0) {
      return true;
    }
    return !batch_.items.empty();
  }
  return !batch_.items.empty();
}

core::ObservationBatch SessionController::training_observations() const {
  if (uses_stereo_dual_session() && uses_tier4_intrinsics()) {
    core::ObservationBatch merged;
    const auto left = intrinsics_left_state_.training_batch();
    const auto right = intrinsics_right_state_.training_batch();
    merged.items.reserve(left.items.size() + right.items.size());
    for (const auto &obs : left.items) {
      merged.items.push_back(obs);
    }
    for (const auto &obs : right.items) {
      merged.items.push_back(obs);
    }
    return merged;
  }
  if (is_intrinsics() && uses_tier4_intrinsics()) {
    return intrinsics_state_.training_batch();
  }
  return batch_;
}

QString SessionController::ensure_capture_cache_dir() {
  if (!capture_cache_dir_.isEmpty()) {
    QDir dir(capture_cache_dir_);
    if (dir.exists()) {
      return capture_cache_dir_;
    }
    capture_cache_dir_.clear();
  }
  const QString dir_path = QDir::temp().filePath(
      QStringLiteral("hs_calib_capture_%1").arg(QDateTime::currentMSecsSinceEpoch()));
  if (!QDir().mkpath(dir_path)) {
    return {};
  }
  capture_cache_dir_ = dir_path;
  return capture_cache_dir_;
}

void SessionController::clear_capture_cache() {
  if (capture_cache_dir_.isEmpty()) {
    return;
  }
  QDir dir(capture_cache_dir_);
  dir.removeRecursively();
  capture_cache_dir_.clear();
}

QString SessionController::save_capture_original(const cv::Mat &bgr, int index) {
  if (bgr.empty()) {
    return {};
  }
  const QString dir = ensure_capture_cache_dir();
  if (dir.isEmpty()) {
    return {};
  }
  const QString path =
      QDir(dir).filePath(QStringLiteral("%1.png").arg(index, 4, 10, QChar('0')));
  if (cv::imwrite(path.toStdString(), bgr)) {
    return path;
  }
  return {};
}

void SessionController::fill_capture_view(CapturedView *view) {
  if (view == nullptr) {
    return;
  }
  view->overlay = last_preview_;
  const cv::Mat bgr = current_bgr();
  if (source_mode_ == SourceMode::RosTopic) {
    view->bgr = cv::Mat();
    view->image_path = save_capture_original(bgr, static_cast<int>(captured_views_.size()));
    return;
  }
  view->image_path.clear();
  view->bgr = bgr.clone();
}

QString SessionController::resolve_obs_source_path(const QString &source) {
  QString path = source;
  if (path.startsWith(QStringLiteral("left:"))) {
    path = path.mid(5);
  } else if (path.startsWith(QStringLiteral("right:"))) {
    path = path.mid(6);
  }
  if (path.startsWith(QStringLiteral("ros://"))) {
    return {};
  }
  return path;
}

bool SessionController::validate_solve_preconditions(QString *error_out) const {
  if (is_handeye() && camera_yaml_.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("手眼标定需要先指定相机内参 YAML");
    }
    return false;
  }
  if (is_stereo_extrinsics()) {
    const auto cfg = solve_config_map();
    const bool has_left =
        (cfg.count("left_camera_yaml") && !cfg.at("left_camera_yaml").empty()) ||
        (cfg.count("camera_yaml") && !cfg.at("camera_yaml").empty());
    const bool has_right =
        cfg.count("right_camera_yaml") && !cfg.at("right_camera_yaml").empty();
    if (!has_left || !has_right) {
      if (error_out) {
        *error_out = QStringLiteral(
            "双目外参需要左/右内参 YAML（left_camera_yaml 与 right_camera_yaml）");
      }
      return false;
    }
  }
  return true;
}

void SessionController::request_solve() {
  if (solve_busy_.load()) {
    return;
  }
  QString err;
  if (!validate_solve_preconditions(&err)) {
    emit solve_finished(false, err);
    return;
  }
  start_solve_job();
}

void SessionController::start_solve_job() {
  if (solve_thread_.joinable()) {
    solve_thread_.join();
  }
  solve_busy_.store(true);
  emit solve_started();
  emit solve_progress(5, QStringLiteral("准备观测数据…"));
  solve_thread_ = std::thread([this]() {
    auto report = [this](int percent, const QString &message) {
      QMetaObject::invokeMethod(
          this,
          [this, percent, message]() { emit solve_progress(percent, message); },
          Qt::QueuedConnection);
    };
    QString err;
    bool ok = false;
    try {
      report(20, QStringLiteral("校验参数…"));
      report(35, QStringLiteral("标定求解中，请稍候…"));
      ok = solve(&err);
      report(90, QStringLiteral("整理结果…"));
    } catch (const std::exception &ex) {
      err = QString::fromStdString(ex.what());
      ok = false;
    } catch (...) {
      err = QStringLiteral("标定过程发生未知错误");
      ok = false;
    }
    QMetaObject::invokeMethod(
        this,
        [this, ok, err]() {
          solve_busy_.store(false);
          emit solve_progress(
              100, ok ? QStringLiteral("完成") : QStringLiteral("失败"));
          emit solve_finished(ok, err);
        },
        Qt::QueuedConnection);
  });
}

/// \brief 调用注册表标定器求解
bool SessionController::solve(QString *error_out) {
  if (uses_stereo_dual_session()) {
    return solve_stereo_intrinsics(error_out);
  }
  if (!validate_solve_preconditions(error_out)) {
    return false;
  }
  try {
    if (is_intrinsics() && uses_tier4_intrinsics()) {
      std::string err;
      if (!intrinsics_state_.calibrate(&err)) {
        last_result_ = intrinsics_state_.last_result();
        if (error_out) {
          *error_out = err.empty() ? QStringLiteral("标定失败")
                                   : QString::fromStdString(err);
        }
        emit result_changed();
        emit intrinsics_state_changed();
        return false;
      }
      last_result_ = intrinsics_state_.last_result();
      sync_batch_from_intrinsics();
    } else {
      auto calibrator =
          core::CalibratorRegistry::instance().create(calibrator_id_.toStdString());
      last_result_ = calibrator->calibrate(batch_, solve_config_map());
    }
  } catch (const std::exception &ex) {
    last_result_ = {};
    last_result_.success = false;
    last_result_.message = ex.what();
    if (error_out) {
      *error_out = QString::fromStdString(ex.what());
    }
    emit result_changed();
    return false;
  }

  emit result_changed();
  if (!last_result_.success) {
    if (error_out) {
      *error_out = QString::fromStdString(last_result_.message);
    }
    return false;
  }
  return true;
}

/// \brief 导出内参或手眼外参 YAML
bool SessionController::export_yaml(const QString &path, QString *error_out) const {
  if (!last_result_.success) {
    if (error_out) {
      *error_out = QStringLiteral("尚无成功标定结果");
    }
    return false;
  }
  if (is_handeye() || is_stereo_extrinsics()) {
    if (!core::export_extrinsics_yaml(
            last_result_, result_parent_frame().toStdString(),
            result_child_frame().toStdString(), path.toStdString())) {
      if (error_out) {
        *error_out = QStringLiteral("写出外参失败：%1").arg(path);
      }
      return false;
    }
    return true;
  }
  if (!core::export_camera_yaml(last_result_, path.toStdString())) {
    if (error_out) {
      *error_out = QStringLiteral("写入失败：%1").arg(path);
    }
    return false;
  }
  return true;
}

/// \brief 导出结果目录：YAML、标定设置、原图与叠加图
bool SessionController::export_bundle(const QString &dir_path, QString *error_out) const {
  if (!last_result_.success) {
    if (error_out) {
      *error_out = QStringLiteral("尚无成功标定结果");
    }
    return false;
  }
  if (dir_path.trimmed().isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("导出目录为空");
    }
    return false;
  }

  QDir out(dir_path);
  if (!out.mkpath(QStringLiteral("."))) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建目录：%1").arg(dir_path);
    }
    return false;
  }
  if (!out.mkpath(QStringLiteral("images"))) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建 images 子目录");
    }
    return false;
  }
  if (!out.mkpath(QStringLiteral("images/original"))) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建 images/original 子目录");
    }
    return false;
  }
  if (!out.mkpath(QStringLiteral("images/overlay"))) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建 images/overlay 子目录");
    }
    return false;
  }

  // —— 标定结果 YAML ——
  QString result_name = is_handeye() ? QStringLiteral("handeye_extrinsics.yaml")
                                     : QStringLiteral("camera_intrinsics.yaml");
  if (is_stereo_extrinsics()) {
    result_name = QStringLiteral("stereo_extrinsics.yaml");
  }
  if (is_stereo_intrinsics()) {
    result_name = QStringLiteral("camera_left.yaml+camera_right.yaml");
    const QString left_path = out.filePath(QStringLiteral("camera_left.yaml"));
    const QString right_path = out.filePath(QStringLiteral("camera_right.yaml"));
    bool any = false;
    if (core::export_camera_yaml_prefixed(
            last_result_, "left_", left_path.toStdString())) {
      any = true;
    }
    if (core::export_camera_yaml_prefixed(
            last_result_, "right_", right_path.toStdString())) {
      any = true;
    }
    // 另存一份默认名（优先左）便于兼容旧流程
    const QString fallback = out.filePath(QStringLiteral("camera_intrinsics.yaml"));
    if (!export_yaml(fallback, nullptr)) {
      // ignore
    }
    if (!any) {
      if (error_out) {
        *error_out = QStringLiteral("未能写出左右内参 YAML");
      }
      return false;
    }
    if (has_stereo_rectified()) {
      const QString rect_path = out.filePath(QStringLiteral("stereo_rectified.yaml"));
      if (!core::export_stereo_rectified_yaml(
              last_result_, rect_path.toStdString())) {
        if (error_out) {
          *error_out = QStringLiteral("立体校正 YAML 写入失败：%1").arg(rect_path);
        }
        return false;
      }
    }
  } else {
    const QString result_path = out.filePath(result_name);
    if (!export_yaml(result_path, error_out)) {
      return false;
    }
  }

  // —— 标定设置 ——
  const QString cfg_path = out.filePath(QStringLiteral("session_config.yaml"));
  std::ofstream cfg(cfg_path.toStdString());
  if (!cfg) {
    if (error_out) {
      *error_out = QStringLiteral("无法写入标定设置：%1").arg(cfg_path);
    }
    return false;
  }

  auto source_name = [](SourceMode m) {
    switch (m) {
      case SourceMode::RosTopic:
        return "ros_topic";
      case SourceMode::RosBag:
        return "ros_bag";
      case SourceMode::Offline:
      default:
        return "offline";
    }
  };
  auto pose_name = [](PoseSource p) {
    if (p == PoseSource::Csv) {
      return "csv";
    }
    if (p == PoseSource::Tf) {
      return "tf";
    }
    return "none";
  };

  cfg << "# hs_calib_suite session export\n";
  cfg << "calibrator_id: " << yaml_quote_q(calibrator_id_) << "\n";
  cfg << "source_mode: " << source_name(source_mode_) << "\n";
  cfg << "pose_source: " << pose_name(pose_source_) << "\n";
  if (!ros_topic_name_.isEmpty()) {
    cfg << "ros_topic: " << yaml_quote_q(ros_topic_name_) << "\n";
  }
  cfg << "squares_x: " << squares_x_ << "\n";
  cfg << "squares_y: " << squares_y_ << "\n";
  cfg << "square_length_m: " << square_length_m_ << "\n";
  cfg << "min_views: " << min_views_ << "\n";
  cfg << "min_confidence: " << min_confidence_ << "\n";
  cfg << "min_diversity: " << min_diversity_ << "\n";
  cfg << "auto_cooldown_ms: " << auto_cooldown_ms_ << "\n";
  cfg << "viz_corners: " << (viz_corners_ ? 1 : 0) << "\n";
  cfg << "viz_hull: " << (viz_hull_ ? 1 : 0) << "\n";
  cfg << "viz_conf: " << (viz_conf_ ? 1 : 0) << "\n";
  cfg << "viz_aruco: " << (viz_aruco_ ? 1 : 0) << "\n";
  cfg << "viz_marker_radius: " << viz_marker_radius_ << "\n";
  if (is_handeye()) {
    cfg << "handeye:\n";
    cfg << "  method: " << yaml_quote_q(handeye_method_) << "\n";
    cfg << "  base_frame: " << yaml_quote_q(base_frame_) << "\n";
    cfg << "  gripper_frame: " << yaml_quote_q(gripper_frame_) << "\n";
    if (!camera_yaml_.isEmpty()) {
      cfg << "  camera_yaml: " << yaml_quote_q(camera_yaml_) << "\n";
    }
  }

  cfg << "solve:\n";
  const auto solve_cfg = solve_config_map();
  for (const auto &kv : solve_cfg) {
    cfg << "  " << kv.first << ": " << yaml_quote(kv.second) << "\n";
  }

  cfg << "result:\n";
  cfg << "  success: " << (last_result_.success ? "true" : "false") << "\n";
  cfg << "  message: " << yaml_quote(last_result_.message) << "\n";
  cfg << "  yaml: " << yaml_quote_q(result_name) << "\n";
  for (const auto &kv : last_result_.metrics) {
    cfg << "  " << kv.first << ": " << kv.second << "\n";
  }
  if (!last_result_.intrinsics_meta.empty()) {
    cfg << "  intrinsics_meta:\n";
    for (const auto &kv : last_result_.intrinsics_meta) {
      cfg << "    " << kv.first << ": " << yaml_quote(kv.second) << "\n";
    }
  }

  cfg << "observations:\n";
  int missing_images = 0;
  // —— 逐条写出观测原图与检测叠加 ——
  for (size_t i = 0; i < batch_.items.size(); ++i) {
    const auto &obs = batch_.items[i];
    std::ostringstream stem;
    stem << std::setw(2) << std::setfill('0') << static_cast<int>(i);
    const QString rel_img =
        QStringLiteral("images/original/%1.png").arg(QString::fromStdString(stem.str()));
    const QString rel_det =
        QStringLiteral("images/overlay/%1.png").arg(QString::fromStdString(stem.str()));
    const QString abs_img = out.filePath(rel_img);
    const QString abs_det = out.filePath(rel_det);

    bool wrote_img = false;
    bool wrote_det = false;
    if (i < captured_views_.size() && !captured_views_[i].bgr.empty()) {
      wrote_img = cv::imwrite(abs_img.toStdString(), captured_views_[i].bgr);
    }
    if (!wrote_img && i < captured_views_.size() &&
        !captured_views_[i].image_path.isEmpty()) {
      const QString &cache = captured_views_[i].image_path;
      if (QFileInfo::exists(cache) && QFile::copy(cache, abs_img)) {
        wrote_img = true;
      }
    }
    if (!wrote_img) {
      const QString src = resolve_obs_source_path(QString::fromStdString(obs.source_path));
      if (QFileInfo::exists(src) && QFile::copy(src, abs_img)) {
        wrote_img = true;
      }
    }
    if (!wrote_img) {
      ++missing_images;
    }
    if (i < captured_views_.size() && !captured_views_[i].overlay.isNull()) {
      wrote_det = captured_views_[i].overlay.save(abs_det, "PNG");
    }

    int npts = 0;
    for (const auto &c : obs.correspondences) {
      npts += static_cast<int>(c.image_points.rows());
    }
    cfg << "  - index: " << i << "\n";
    cfg << "    source: " << yaml_quote(obs.source_path) << "\n";
    cfg << "    image_width: " << obs.image_width << "\n";
    cfg << "    image_height: " << obs.image_height << "\n";
    cfg << "    num_points: " << npts << "\n";
    if (wrote_img) {
      cfg << "    image: " << yaml_quote_q(rel_img) << "\n";
    }
    if (wrote_det) {
      cfg << "    detect: " << yaml_quote_q(rel_det) << "\n";
    }
    if (obs.has_base_gripper) {
      cfg << "    T_base_gripper: [";
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          if (r || c) {
            cfg << ", ";
          }
          cfg.precision(12);
          cfg << obs.T_base_gripper(r, c);
        }
      }
      cfg << "]\n";
    }
  }

  if (!cfg) {
    if (error_out) {
      *error_out = QStringLiteral("写入标定设置失败");
    }
    return false;
  }
  if (missing_images > 0 && error_out) {
    *error_out = QStringLiteral("已导出，但有 %1 张原图缺失（在线采集请重新拍一次后再导出）")
                     .arg(missing_images);
  }
  return true;
}

void SessionController::set_intrinsics_image_view_mode(IntrinsicsImageViewMode mode) {
  intrinsics_view_mode_ = mode;
}

IntrinsicsImageViewMode SessionController::intrinsics_image_view_mode() const {
  return intrinsics_view_mode_;
}

void SessionController::set_intrinsics_viz_options(const IntrinsicsVizOptions &options) {
  intrinsics_viz_options_ = options;
}

IntrinsicsVizOptions SessionController::intrinsics_viz_options() const {
  return intrinsics_viz_options_;
}

void SessionController::clear_intrinsics_linearity_heatmap() {
  intrinsics_linearity_grid_.clear();
}

void SessionController::set_intrinsics_browse_sample(int index, bool evaluation_set) {
  intrinsics_browse_index_ = index;
  intrinsics_browse_eval_ = evaluation_set;
}

int SessionController::intrinsics_browse_sample_index() const {
  return intrinsics_browse_index_;
}

bool SessionController::intrinsics_browse_is_evaluation() const {
  return intrinsics_browse_eval_;
}

void SessionController::clear_intrinsics_browse() {
  intrinsics_browse_index_ = -1;
}

QImage SessionController::decorate_intrinsics_preview(
    const QImage &preview, bool lightweight) const {
  if (!is_intrinsics() || preview.isNull() || !uses_tier4_intrinsics()) {
    return preview;
  }
  const bool need_overlay = tier4_preview_needs_overlay();
  if (lightweight && !need_overlay && intrinsics_viz_options_.draw_detection) {
    return preview;
  }

  QImage base = preview;
  if (!intrinsics_viz_options_.draw_detection) {
    const cv::Mat raw = current_bgr();
    if (!raw.empty()) {
      const QImage raw_img = cv_bgr_to_qimage(raw);
      if (!raw_img.isNull()) {
        base = raw_img;
      }
    }
  }

  cv::Mat bgr = qimage_to_cv_bgr(base);
  if (bgr.empty()) {
    return preview;
  }
  const auto extras = core::calibration_extras_from_config(solve_options_);
  apply_intrinsics_preview_overlay(
      &bgr, intrinsics_view_mode_, intrinsics_viz_options_,
      intrinsics_state_.collector(), last_board_metrics_,
      intrinsics_linearity_grid_, extras.viz_pixel_cells);
  QImage img = cv_bgr_to_qimage(bgr);
  if (img.isNull()) {
    return preview;
  }
  const cv::Mat K = intrinsics_state_.has_calibrated_model()
      ? intrinsics_state_.calibrated_K()
      : intrinsics_state_.provisional_model().camera_matrix;
  const cv::Mat D = intrinsics_state_.has_calibrated_model()
      ? intrinsics_state_.calibrated_D()
      : intrinsics_state_.provisional_model().dist_coeffs;
  if (intrinsics_view_mode_ == IntrinsicsImageViewMode::Source) {
    return img;
  }
  return apply_intrinsics_view_mode(img, intrinsics_view_mode_, K, D, 0.0);
}

bool SessionController::intrinsics_browse_preview(QImage *out) const {
  if (out == nullptr || !uses_tier4_intrinsics() || intrinsics_browse_index_ < 0) {
    return false;
  }
  const auto &pool = intrinsics_browse_eval_
                         ? intrinsics_state_.collector().evaluation()
                         : intrinsics_state_.collector().training();
  if (intrinsics_browse_index_ >= static_cast<int>(pool.size())) {
    return false;
  }
  const auto &obs = pool[static_cast<size_t>(intrinsics_browse_index_)].observation;
  if (obs.source_path.empty()) {
    return false;
  }
  const cv::Mat bgr = cv::imread(obs.source_path, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    return false;
  }
  *out = cv_bgr_to_qimage(bgr);
  return !out->isNull();
}

}  // namespace gui
}  // namespace hs_calib
