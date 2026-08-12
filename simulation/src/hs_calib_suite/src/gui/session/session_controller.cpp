#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMetaObject>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/aruco.hpp>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/detectors/aruco_grid_detector.hpp"
#include "hs_calib_suite/core/targets/aruco_grid_target.hpp"
#include "hs_calib_suite/core/detectors/charuco_detector.hpp"
#include "hs_calib_suite/core/targets/charuco_target.hpp"
#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"
#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/detectors/circle_grid_detector.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
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
  const bool partial_ok =
      trihedral || target_type == "charuco" || target_type == "aruco_grid";
  if (partial_ok) {
    if (corners.size() < 6) {
      return 0.0;  // OpenCV calibrateCamera / DLT 每帧至少 6 点
    }
    const double fill = std::min(
        1.0, static_cast<double>(corners.size()) /
                 static_cast<double>(std::max(6, expect_face)));
    score = 0.15 + 0.50 * fill;
  } else {
    if (static_cast<int>(corners.size()) != expect_face) {
      return 0.0;
    }
    score = 0.35;
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
    const cv::Scalar kArucoDefault(160, 160, 160);

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
        const cv::Scalar &col =
            (fi >= 0 && fi < 3) ? kFaceColors[fi] : kArucoDefault;
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
        cv::polylines(in.bgr, &pts, &npts, 1, true, col, 1, cv::LINE_AA);
        for (const auto &p : corners) {
          cv::circle(in.bgr, p, 2, col, -1, cv::LINE_AA);
        }
        const std::string label = std::to_string(id);
        const cv::Point org(cvRound(center.x) + 3, cvRound(center.y) - 3);
        cv::putText(
            in.bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 2,
            cv::LINE_AA);
        cv::putText(
            in.bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv::LINE_AA);
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
        detected = detector.detect_merged(
            frame, &faces_found,
            in.fast ? core::TrihedralChessDetectSpeed::Fast
                    : core::TrihedralChessDetectSpeed::Thorough);
        if (detected.image_points.rows() < 4) {
          out.preview = mat_bgr_to_qimage(in.bgr);
          out.error = (target_type == "trihedral_aruco")
              ? QStringLiteral("三面 ArUco 网格请改用 trihedral_charuco；当前按棋盘角点路径未检出")
              : QStringLiteral(
                    "未检测到三面棋盘角点（允许局部网格；确认 squares_x/y=内角点数 与 Isaac 一致）");
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
      const std::string dictionary = opt_str("dictionary", "DICT_4X4_50");
      const double marker_len = opt_double("marker_length", 0.018);
      double marker_sep = opt_double("marker_separation", -1.0);
      if (marker_sep <= 0.0) {
        marker_sep = (in.square_length_m > marker_len)
            ? (in.square_length_m - marker_len)
            : 0.006;
      }

      std::vector<core::Correspondence> corrs;
      if (target_type == "charuco") {
        core::CharucoTarget target(
            in.squares_x, in.squares_y, in.square_length_m, marker_len, dictionary);
        corrs = core::CharucoDetector(target).detect(frame, &aruco);
      } else if (target_type == "aruco_grid") {
        core::ArucoGridTarget target(
            in.squares_x, in.squares_y, marker_len, marker_sep, dictionary);
        corrs = core::ArucoGridDetector(target).detect(frame, &aruco);
      } else if (
          target_type == "circles_symmetric" || target_type == "circle_grid" ||
          target_type == "circles_asymmetric") {
        const auto pattern = (target_type == "circles_asymmetric")
            ? core::CircleGridPattern::Asymmetric
            : core::CircleGridPattern::Symmetric;
        core::CircleGridTarget target(
            in.squares_x, in.squares_y, in.square_length_m, pattern);
        corrs = core::CircleGridDetector(target).detect(frame);
      } else {
        core::ChessboardTarget target(
            in.squares_x, in.squares_y, in.square_length_m);
        core::ChessboardDetectOptions dopts;
        auto flag_on = [&](const char *key, bool def) {
          const auto it = in.solve_options.find(key);
          if (it == in.solve_options.end()) {
            return def;
          }
          return it->second == "1" || it->second == "true";
        };
        dopts.adaptive_thresh = flag_on("cb_adaptive", true);
        dopts.normalize_image = flag_on("cb_normalize", true);
        dopts.filter_quads = flag_on("cb_filter_quads", false);
        dopts.fast_check = flag_on("cb_fast_check", true);
        {
          const auto it = in.solve_options.find("subpix_win");
          if (it != in.solve_options.end()) {
            try {
              dopts.subpix_win = std::stoi(it->second);
            } catch (...) {
            }
          }
        }
        corrs = core::ChessboardDetector(target, dopts).detect(frame);
      }
      if (corrs.empty()) {
        draw_aruco_overlay();
        if (in.viz_conf && !aruco.empty()) {
          cv::putText(
              in.bgr, cv::format("aruco=%d (no board)", static_cast<int>(aruco.ids.size())),
              cv::Point(12, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1,
              cv::LINE_AA);
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
    if (in.viz_corners) {
      const std::string target_type =
          in.solve_options.count("target") ? in.solve_options.at("target")
                                           : "chessboard";
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
      const std::string label = in.trihedral
          ? cv::format(
                "conf=%.2f faces=%d aruco=%d", out.confidence, faces_found,
                static_cast<int>(aruco.ids.size()))
          : (aruco.empty()
                 ? cv::format("conf=%.2f", out.confidence)
                 : cv::format(
                       "conf=%.2f aruco=%d", out.confidence,
                       static_cast<int>(aruco.ids.size())));
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
}

/// \brief 设置标定器 ID，并按类型调整位姿源/最少视角
void SessionController::set_calibrator_id(const QString &id) {
  calibrator_id_ = id;
  if (is_handeye() && pose_source_ == PoseSource::None) {
    pose_source_ = PoseSource::Csv;
  }
  if (is_intrinsics() || is_trihedral()) {
    pose_source_ = PoseSource::None;
  }
  if (is_trihedral() && min_views_ > 1) {
    // Default oneshot-friendly; user can raise for multi-view
    min_views_ = 1;
  }
}

/// \brief 是否为手眼标定器
bool SessionController::is_handeye() const {
  return calibrator_id_ == QStringLiteral("eye_in_hand") ||
         calibrator_id_ == QStringLiteral("eye_to_hand");
}

/// \brief 是否为单目内参标定器
bool SessionController::is_intrinsics() const {
  return calibrator_id_ == QStringLiteral("cam_intrinsics");
}

/// \brief 是否为直角三面标定器
bool SessionController::is_trihedral() const {
  return calibrator_id_ == QStringLiteral("trihedral_oneshot");
}

/// \brief 切换离线/ROS 图像源并清除当前检测
void SessionController::set_source_mode(SourceMode mode) {
  source_mode_ = mode;
  has_detection_ = false;
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
  solve_options_ = opts;
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
  camera_yaml_ = path;
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

/// \brief 加载离线位姿 CSV
bool SessionController::load_pose_csv(const QString &path, QString *error_out) {
  return pose_csv_.load(path, error_out);
}

/// \brief 扫描目录加载离线图片列表并重置会话数据
int SessionController::load_image_dir(const QString &dir_path) {
  // —— 重置会话状态 ——
  image_paths_.clear();
  current_index_ = -1;
  has_detection_ = false;
  last_confidence_ = 0.0;
  last_preview_ = {};
  captured_fps_.clear();
  captured_views_.clear();
  batch_ = {};
  last_result_ = {};

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
  for (const QFileInfo &fi : files) {
    image_paths_.push_back(fi.absoluteFilePath());
  }
  if (!image_paths_.isEmpty()) {
    current_index_ = 0;
  }
  emit images_changed();
  emit current_changed();
  emit observations_changed();
  emit result_changed();
  return image_paths_.size();
}

/// \brief 切换离线当前图片索引
void SessionController::set_current_index(int index) {
  if (source_mode_ == SourceMode::RosTopic) {
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
  if (current_index_ < 0 || current_index_ >= image_paths_.size()) {
    return {};
  }
  return image_paths_.at(current_index_);
}

/// \brief 取当前帧 BGR（在线 live / 离线 imread）
cv::Mat SessionController::current_bgr() const {
  if (source_mode_ == SourceMode::RosTopic) {
    return live_bgr_;
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
    m["model"] = "pinhole";
  }
  m["method"] = handeye_method_.toStdString();
  if (!camera_yaml_.isEmpty()) {
    m["camera_yaml"] = camera_yaml_.toStdString();
  }
  if (calibrator_id_ == QStringLiteral("eye_in_hand")) {
    m["parent_frame"] = "gripper";
    m["child_frame"] = "camera";
  } else if (calibrator_id_ == QStringLiteral("eye_to_hand")) {
    m["parent_frame"] = "base";
    m["child_frame"] = "camera";
  }
  return m;
}

/// \brief 结果父坐标系名
QString SessionController::result_parent_frame() const {
  if (last_result_.intrinsics_meta.count("parent_frame")) {
    return QString::fromStdString(last_result_.intrinsics_meta.at("parent_frame"));
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
  return is_handeye() ? QStringLiteral("camera") : QString();
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
  const DetectJobOut out = run_detect_job(std::move(in));
  last_preview_ = out.preview;
  last_confidence_ = out.confidence;
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
  } else {
    has_detection_ = false;
  }
  if (preview_out) {
    *preview_out = last_preview_;
  }
  if (error_out) {
    *error_out = out.error;
  }
  return out.ok;
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
    const DetectJobOut out = run_detect_job(std::move(in));
    QMetaObject::invokeMethod(
        this,
        [this, out, epoch]() {
          if (epoch != detect_epoch_.load()) {
            detect_busy_.store(false);
            return;
          }
          last_preview_ = out.preview;
          last_confidence_ = out.confidence;
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
          } else {
            has_detection_ = false;
          }
          detect_busy_.store(false);
          emit detect_finished(out.ok, out.error);
          if (pending_detect_) {
            pending_detect_ = false;
            start_detect_job(pending_fast_);
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
  // 三面 / ChArUco / ArUco：按填充比例计分（允许部分角点）
  // 棋盘 / 圆点：通常要求整面点数
  double score = 0.0;
  const std::string target_type =
      solve_options_.count("target") ? solve_options_.at("target") : "chessboard";
  const bool partial_ok =
      is_trihedral() || target_type == "charuco" || target_type == "aruco_grid";
  if (partial_ok) {
    if (corners.size() < 6) {
      return 0.0;
    }
    const double fill =
        std::min(1.0, static_cast<double>(corners.size()) /
                          static_cast<double>(std::max(6, expect_face)));
    score = 0.15 + 0.50 * fill;
  } else {
    if (static_cast<int>(corners.size()) != expect_face) {
      return 0.0;
    }
    score = 0.35;
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

/// \brief 高置信且多样时自动入库当前观测
bool SessionController::try_auto_capture(
    double min_confidence, double min_diversity, QString *error_out) {
  if (!has_detection_ || last_confidence_ < min_confidence) {
    if (error_out) {
      *error_out = QStringLiteral("置信度不足");
    }
    return false;
  }
  if (!is_diverse_enough(last_fp_, min_diversity)) {
    if (error_out) {
      *error_out = QStringLiteral("与已采集姿态过于相似");
    }
    return false;
  }
  if (!add_current_observation(error_out)) {
    return false;
  }
  return true;
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
  if (source_mode_ == SourceMode::RosTopic) {
    path = QStringLiteral("ros://%1#%2")
               .arg(ros_topic_name_.isEmpty() ? QStringLiteral("image")
                                              : ros_topic_name_)
               .arg(++live_seq_);
  } else {
    for (const auto &obs : batch_.items) {
      if (obs.source_path == path.toStdString()) {
        if (error_out) {
          *error_out = QStringLiteral("该图片已在观测列表中");
        }
        return false;
      }
    }
  }

  // —— 组装观测并缓存原图/叠加 ——
  core::Observation obs;
  obs.source_path = path.toStdString();
  obs.frame_id = "camera";
  obs.image_width = detect_width_;
  obs.image_height = detect_height_;
  obs.correspondences = {current_corr_};
  if (!attach_pose_to_observation(&obs, error_out)) {
    return false;
  }
  batch_.items.push_back(std::move(obs));
  if (has_detection_) {
    captured_fps_.push_back(last_fp_);
  }
  CapturedView view;
  view.bgr = current_bgr().clone();
  view.overlay = last_preview_;
  captured_views_.push_back(std::move(view));
  if (source_mode_ == SourceMode::RosTopic) {
    has_detection_ = false;
  }
  emit observations_changed();
  return true;
}

/// \brief 删除指定行观测及相关缓存
void SessionController::remove_observation(int row) {
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
  emit observations_changed();
}

/// \brief 清空全部观测与采集缓存
void SessionController::clear_observations() {
  batch_.items.clear();
  captured_fps_.clear();
  captured_views_.clear();
  emit observations_changed();
}

/// \brief 调用注册表标定器求解
bool SessionController::solve(QString *error_out) {
  if (is_handeye() && camera_yaml_.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("手眼标定需要先指定相机内参 YAML");
    }
    return false;
  }
  try {
    auto calibrator =
        core::CalibratorRegistry::instance().create(calibrator_id_.toStdString());
    last_result_ = calibrator->calibrate(batch_, solve_config_map());
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
  if (is_handeye()) {
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

/// \brief 导出结果目录：YAML、会话配置、原图与叠加图
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

  // —— 标定结果 YAML ——
  const QString result_name = is_handeye()
      ? QStringLiteral("handeye_extrinsics.yaml")
      : QStringLiteral("camera_intrinsics.yaml");
  const QString result_path = out.filePath(result_name);
  if (!export_yaml(result_path, error_out)) {
    return false;
  }

  // —— 会话配置 ——
  const QString cfg_path = out.filePath(QStringLiteral("session_config.yaml"));
  std::ofstream cfg(cfg_path.toStdString());
  if (!cfg) {
    if (error_out) {
      *error_out = QStringLiteral("无法写入会话配置：%1").arg(cfg_path);
    }
    return false;
  }

  auto source_name = [](SourceMode m) {
    return m == SourceMode::RosTopic ? "ros_topic" : "offline";
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
    const QString rel_img = QStringLiteral("images/%1.png").arg(QString::fromStdString(stem.str()));
    const QString rel_det =
        QStringLiteral("images/%1_detect.png").arg(QString::fromStdString(stem.str()));
    const QString abs_img = out.filePath(rel_img);
    const QString abs_det = out.filePath(rel_det);

    bool wrote_img = false;
    bool wrote_det = false;
    if (i < captured_views_.size() && !captured_views_[i].bgr.empty()) {
      wrote_img = cv::imwrite(abs_img.toStdString(), captured_views_[i].bgr);
    }
    if (!wrote_img) {
      const QString src = QString::fromStdString(obs.source_path);
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
      *error_out = QStringLiteral("写入会话配置失败");
    }
    return false;
  }
  if (missing_images > 0 && error_out) {
    *error_out = QStringLiteral("已导出，但有 %1 张原图缺失（在线采集请重新拍一次后再导出）")
                     .arg(missing_images);
  }
  return true;
}

}  // namespace gui
}  // namespace hs_calib
