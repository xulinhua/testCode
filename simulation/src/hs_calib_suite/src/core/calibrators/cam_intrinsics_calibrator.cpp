#include "hs_calib_suite/core/calibrators/cam_intrinsics_calibrator.hpp"

#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>

#if defined(__has_include)
#  if __has_include(<opencv2/ccalib/omnidir.hpp>)
#    include <opencv2/ccalib/omnidir.hpp>
#    define HS_CALIB_HAS_OMNIDIR 1
#  endif
#endif
#ifndef HS_CALIB_HAS_OMNIDIR
#  define HS_CALIB_HAS_OMNIDIR 0
#endif

#include "hs_calib_suite/core/registry/registry.hpp"
#include "hs_calib_suite/core/util/camera_models.hpp"

namespace hs_calib {
namespace core {

namespace {

/// \brief 从 config 解析整型参数，失败时返回默认值
int parse_int(const std::map<std::string, std::string> &config, const char *key, int def) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return def;
  }
  try {
    return std::stoi(it->second);
  } catch (...) {
    return def;
  }
}

/// \brief 从 config 解析浮点参数，失败时返回默认值
double parse_double(
    const std::map<std::string, std::string> &config, const char *key, double def) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return def;
  }
  try {
    return std::stod(it->second);
  } catch (...) {
    return def;
  }
}

/// \brief 将 double 格式化为高精度字符串
std::string to_str(double v) {
  std::ostringstream oss;
  oss.precision(12);
  oss << v;
  return oss.str();
}

double d_at(const cv::Mat &D, int i) {
  if (D.empty() || i < 0) {
    return 0.0;
  }
  const int n = D.rows * D.cols;
  if (i >= n) {
    return 0.0;
  }
  return D.at<double>(i);
}

}  // namespace

/// \brief 返回标定器元信息（ID、显示名、支持靶标）
CalibratorInfo CamIntrinsicsCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "cam_intrinsics";
  info.display_name = "相机内参";
  info.category = "intrinsics";
  info.supported_targets = {
      "chessboard", "charuco", "aruco_grid", "circles_symmetric", "circles_asymmetric"};
  return info;
}

/// \brief 多帧观测 → Brown / Kannala–Brandt / CMei 内参标定
CalibrationResult CamIntrinsicsCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  (void)parse_int(config, "squares_x", 9);
  (void)parse_int(config, "squares_y", 6);
  (void)parse_double(config, "square_length", 0.025);

  // —— 收集有效帧的 3D-2D 对应 ——
  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_pts;
  int image_width = 0;
  int image_height = 0;

  for (const auto &obs : observations.items) {
    if (obs.correspondences.empty()) {
      continue;
    }
    const auto &c = obs.correspondences.front();
    if (c.image_points.rows() < 6 || c.object_points.rows() != c.image_points.rows()) {
      continue;
    }
    std::vector<cv::Point3f> op;
    std::vector<cv::Point2f> ip;
    op.reserve(static_cast<size_t>(c.image_points.rows()));
    ip.reserve(static_cast<size_t>(c.image_points.rows()));
    for (int i = 0; i < c.image_points.rows(); ++i) {
      op.emplace_back(
          static_cast<float>(c.object_points(i, 0)),
          static_cast<float>(c.object_points(i, 1)),
          static_cast<float>(c.object_points(i, 2)));
      ip.emplace_back(
          static_cast<float>(c.image_points(i, 0)),
          static_cast<float>(c.image_points(i, 1)));
    }
    obj_pts.push_back(std::move(op));
    img_pts.push_back(std::move(ip));
    if (obs.image_width > 0 && obs.image_height > 0) {
      image_width = obs.image_width;
      image_height = obs.image_height;
    }
  }

  if (obj_pts.size() < 3) {
    result.success = false;
    result.message = "有效观测不足（至少需要 3 帧检测到靶标）";
    return result;
  }
  // —— 校验各帧图像尺寸一致 ——
  for (const auto &obs : observations.items) {
    if (obs.correspondences.empty()) {
      continue;
    }
    if (obs.image_width > 0 && obs.image_height > 0) {
      if (image_width <= 0) {
        image_width = obs.image_width;
        image_height = obs.image_height;
      } else if (obs.image_width != image_width || obs.image_height != image_height) {
        result.success = false;
        result.message = "观测图像尺寸不一致，请使用同一相机分辨率";
        return result;
      }
    }
  }
  if (image_width <= 0 || image_height <= 0) {
    result.success = false;
    result.message = "缺少图像尺寸（Observation.image_width/height）";
    return result;
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat D = cv::Mat::zeros(8, 1, CV_64F);
  double xi = 0.0;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  // —— 解析标定标志位（主要用于 Brown） ——
  auto flag_on = [&](const char *key) {
    const auto it = config.find(key);
    return it != config.end() && (it->second == "1" || it->second == "true");
  };
  int flags = 0;
  if (flag_on("fix_principal")) {
    flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
  }
  if (flag_on("fix_aspect")) {
    flags |= cv::CALIB_FIX_ASPECT_RATIO;
  }
  if (flag_on("zero_tangent")) {
    flags |= cv::CALIB_ZERO_TANGENT_DIST;
  }
  if (flag_on("fix_k1")) {
    flags |= cv::CALIB_FIX_K1;
  }
  if (flag_on("fix_k2")) {
    flags |= cv::CALIB_FIX_K2;
  }
  if (flag_on("fix_k3") || config.find("fix_k3") == config.end()) {
    flags |= cv::CALIB_FIX_K3;
  }
  if (flag_on("rational_model")) {
    flags |= cv::CALIB_RATIONAL_MODEL;
  }
  if (flag_on("thin_prism")) {
    flags |= cv::CALIB_THIN_PRISM_MODEL;
  }
  if (flag_on("use_intrinsic_guess")) {
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  }

  const std::string model_raw =
      config.count("model") ? config.at("model") : std::string("brown_conrady");
  const CameraModelId model_id = parse_camera_model(model_raw);
  const std::string model = camera_model_to_string(model_id);
  double rms = 0.0;

  try {
    if (model_id == CameraModelId::KannalaBrandt) {
      cv::Mat Kf = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat Df = cv::Mat::zeros(4, 1, CV_64F);
      rms = cv::fisheye::calibrate(
          obj_pts, img_pts, cv::Size(image_width, image_height), Kf, Df, rvecs, tvecs,
          cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND |
              cv::fisheye::CALIB_FIX_SKEW);
      K = Kf;
      D = Df.clone();
      result.message = "fisheye::calibrate (Kannala-Brandt) ok";
    } else if (model_id == CameraModelId::CMei) {
#if HS_CALIB_HAS_OMNIDIR
      cv::Mat Ko = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat Do = cv::Mat::zeros(4, 1, CV_64F);
      cv::Mat xi_m = cv::Mat::zeros(1, 1, CV_64F);
      int omni_flags = 0;
      if (flag_on("use_intrinsic_guess")) {
        omni_flags |= cv::omnidir::CALIB_USE_GUESS;
      }
      rms = cv::omnidir::calibrate(
          obj_pts, img_pts, cv::Size(image_width, image_height), Ko, xi_m, Do, rvecs, tvecs,
          omni_flags,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-8));
      K = Ko;
      D = Do.clone();
      xi = xi_m.at<double>(0, 0);
      result.message = "omnidir::calibrate (CMei) ok";
#else
      result.success = false;
      result.message = "本机构建未包含 OpenCV omnidir（ccalib），无法标定 CMei";
      return result;
#endif
    } else {
      rms = cv::calibrateCamera(
          obj_pts, img_pts, cv::Size(image_width, image_height), K, D, rvecs, tvecs, flags);
      result.message = "calibrateCamera (Brown-Conrady) ok";
    }
  } catch (const cv::Exception &ex) {
    result.success = false;
    result.message = std::string("标定失败: ") + ex.what();
    return result;
  }

  result.success = true;
  result.score = static_cast<float>(1.0 / (1.0 + rms));
  result.metrics["reprojection_rmse"] = rms;
  result.metrics["num_views"] = static_cast<double>(obj_pts.size());

  // —— 写入内参元数据 ——
  result.intrinsics_meta["fx"] = to_str(K.at<double>(0, 0));
  result.intrinsics_meta["fy"] = to_str(K.at<double>(1, 1));
  result.intrinsics_meta["cx"] = to_str(K.at<double>(0, 2));
  result.intrinsics_meta["cy"] = to_str(K.at<double>(1, 2));
  result.intrinsics_meta["xi"] = to_str(xi);
  result.intrinsics_meta["model"] = model;
  result.intrinsics_meta["image_width"] = std::to_string(image_width);
  result.intrinsics_meta["image_height"] = std::to_string(image_height);
  result.intrinsics_meta["rms"] = to_str(rms);

  if (model_id == CameraModelId::KannalaBrandt) {
    // fisheye: k1..k4
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["k3"] = to_str(d_at(D, 2));
    result.intrinsics_meta["k4"] = to_str(d_at(D, 3));
    result.intrinsics_meta["p1"] = "0";
    result.intrinsics_meta["p2"] = "0";
    result.intrinsics_meta["dist_n"] = "4";
  } else if (model_id == CameraModelId::CMei) {
    // omnidir: k1,k2,p1,p2
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["p1"] = to_str(d_at(D, 2));
    result.intrinsics_meta["p2"] = to_str(d_at(D, 3));
    result.intrinsics_meta["k3"] = "0";
    result.intrinsics_meta["k4"] = "0";
    result.intrinsics_meta["dist_n"] = "4";
  } else {
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["p1"] = to_str(d_at(D, 2));
    result.intrinsics_meta["p2"] = to_str(d_at(D, 3));
    result.intrinsics_meta["k3"] = to_str(d_at(D, 4));
    result.intrinsics_meta["k4"] = "0";
    result.intrinsics_meta["dist_n"] = "5";
  }
  return result;
}

HS_CALIB_REGISTER("cam_intrinsics", CamIntrinsicsCalibrator);

}  // namespace core
}  // namespace hs_calib
