#include "hs_calib_suite/core/calibrators/cam_intrinsics_calibrator.hpp"

#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/registry/registry.hpp"

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

/// \brief 多帧观测 → OpenCV calibrateCamera / fisheye 标定内参
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
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  // —— 解析标定标志位 ——
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
    // default FIX_K3 when unset (legacy behavior)
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

  const std::string model =
      config.count("model") ? config.at("model") : std::string("pinhole");
  double rms = 0.0;
  // —— 执行 pinhole 或 fisheye 标定 ——
  if (model == "fisheye") {
    cv::Mat Kf = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat Df = cv::Mat::zeros(4, 1, CV_64F);
    rms = cv::fisheye::calibrate(
        obj_pts, img_pts, cv::Size(image_width, image_height), Kf, Df, rvecs, tvecs,
        cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND |
            cv::fisheye::CALIB_FIX_SKEW);
    K = Kf;
    D = cv::Mat::zeros(8, 1, CV_64F);
    for (int i = 0; i < 4; ++i) {
      D.at<double>(i, 0) = Df.at<double>(i, 0);
    }
  } else {
    rms = cv::calibrateCamera(
        obj_pts, img_pts, cv::Size(image_width, image_height), K, D, rvecs, tvecs,
        flags);
  }

  result.success = true;
  result.score = static_cast<float>(1.0 / (1.0 + rms));
  result.message = "calibrateCamera ok";
  result.metrics["reprojection_rmse"] = rms;
  result.metrics["num_views"] = static_cast<double>(obj_pts.size());

  // —— 写入内参元数据 ——
  result.intrinsics_meta["fx"] = to_str(K.at<double>(0, 0));
  result.intrinsics_meta["fy"] = to_str(K.at<double>(1, 1));
  result.intrinsics_meta["cx"] = to_str(K.at<double>(0, 2));
  result.intrinsics_meta["cy"] = to_str(K.at<double>(1, 2));
  result.intrinsics_meta["k1"] = to_str(D.at<double>(0, 0));
  result.intrinsics_meta["k2"] = to_str(D.at<double>(1, 0));
  result.intrinsics_meta["p1"] = to_str(D.at<double>(2, 0));
  result.intrinsics_meta["p2"] = to_str(D.at<double>(3, 0));
  result.intrinsics_meta["k3"] = to_str(D.rows > 4 ? D.at<double>(4, 0) : 0.0);
  result.intrinsics_meta["image_width"] = std::to_string(image_width);
  result.intrinsics_meta["image_height"] = std::to_string(image_height);
  result.intrinsics_meta["rms"] = to_str(rms);
  result.intrinsics_meta["model"] = model;
  return result;
}

HS_CALIB_REGISTER("cam_intrinsics", CamIntrinsicsCalibrator);

}  // namespace core
}  // namespace hs_calib
