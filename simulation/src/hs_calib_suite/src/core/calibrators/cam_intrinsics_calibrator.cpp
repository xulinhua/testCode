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

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_pipeline.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"
#include "hs_calib_suite/core/util/camera_models.hpp"

namespace hs_calib {
namespace core {

namespace {

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

CalibrationResult result_from_brown_pipeline(
    const IntrinsicsPipelineResult &pipe,
    const IntrinsicsProfile &profile,
    int image_width,
    int image_height) {
  CalibrationResult result;
  if (!pipe.ok) {
    result.success = false;
    result.message = pipe.message.empty() ? "内参流水线失败" : pipe.message;
    return result;
  }
  const cv::Mat &K = pipe.camera_matrix;
  const cv::Mat &D = pipe.dist_coeffs;
  result.success = true;
  result.score = static_cast<float>(1.0 / (1.0 + pipe.rms));
  result.message = pipe.message;
  result.metrics["reprojection_rmse"] = pipe.rms;
  result.metrics["num_views"] = static_cast<double>(pipe.num_after_post);
  result.metrics["num_views_input"] = static_cast<double>(pipe.num_input_views);
  result.metrics["num_views_ransac"] = static_cast<double>(pipe.num_after_ransac);
  result.metrics["num_views_train"] = static_cast<double>(pipe.num_after_subsample);
  result.intrinsics_meta["fx"] = to_str(K.at<double>(0, 0));
  result.intrinsics_meta["fy"] = to_str(K.at<double>(1, 1));
  result.intrinsics_meta["cx"] = to_str(K.at<double>(0, 2));
  result.intrinsics_meta["cy"] = to_str(K.at<double>(1, 2));
  result.intrinsics_meta["xi"] = "0";
  result.intrinsics_meta["model"] = "brown_conrady";
  result.intrinsics_meta["intrinsics_profile"] = profile.id;
  result.intrinsics_meta["image_width"] = std::to_string(image_width);
  result.intrinsics_meta["image_height"] = std::to_string(image_height);
  result.intrinsics_meta["rms"] = to_str(pipe.rms);
  result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
  result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
  result.intrinsics_meta["p1"] = to_str(d_at(D, 2));
  result.intrinsics_meta["p2"] = to_str(d_at(D, 3));
  result.intrinsics_meta["k3"] = to_str(d_at(D, 4));
  result.intrinsics_meta["k4"] = to_str(d_at(D, 5));
  result.intrinsics_meta["k5"] = to_str(d_at(D, 6));
  result.intrinsics_meta["k6"] = to_str(d_at(D, 7));
  result.intrinsics_meta["dist_n"] =
      std::to_string(D.rows * D.cols > 0 ? D.rows * D.cols : 5);
  return result;
}

}  // namespace

CalibratorInfo CamIntrinsicsCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "cam_intrinsics";
  info.display_name = "相机内参";
  info.category = "intrinsics";
  info.supported_targets = {
      "chessboard", "charuco", "aruco_grid", "aprilgrid", "circles_symmetric",
      "circles_asymmetric"};
  return info;
}

CalibrationResult CamIntrinsicsCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  (void)parse_int(config, "squares_x", 9);
  (void)parse_int(config, "squares_y", 6);
  (void)parse_double(config, "square_length", 0.025);

  const std::string model_raw =
      config.count("model") ? config.at("model") : std::string("brown_conrady");
  const CameraModelId model_id = parse_camera_model(model_raw);
  const std::string model = camera_model_to_string(model_id);

  int image_width = 0;
  int image_height = 0;
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

  if (model_id == CameraModelId::BrownConrady && tier4_intrinsics_enabled(config)) {
    const IntrinsicsProfile profile = intrinsics_profile_from_config(config);
    const auto views = build_intrinsics_views(observations);
    const auto pipe = run_intrinsics_pipeline(
        views, image_width, image_height, profile, config);
    return result_from_brown_pipeline(pipe, profile, image_width, image_height);
  }

  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_pts;
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
  }
  if (obj_pts.size() < 3) {
    result.success = false;
    result.message = "有效观测不足（至少需要 3 帧检测到靶标）";
    return result;
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat D = cv::Mat::zeros(8, 1, CV_64F);
  double xi = 0.0;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  auto flag_on = [&](const char *key) {
    const auto it = config.find(key);
    return it != config.end() && (it->second == "1" || it->second == "true");
  };
  int flags = 0;
  if (core::config_flag_on(config, "fix_principal_point", "fix_principal")) {
    flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
  }
  if (core::config_flag_on(config, "fix_aspect_ratio", "fix_aspect")) {
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
  if (core::config_flag_on(config, "enable_prism_model", "thin_prism")) {
    flags |= cv::CALIB_THIN_PRISM_MODEL;
  }
  if (flag_on("use_intrinsic_guess")) {
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  }

  double rms = 0.0;
  try {
    if (model_id == CameraModelId::BrownConrady) {
      rms = cv::calibrateCamera(
          obj_pts, img_pts, cv::Size(image_width, image_height), K, D, rvecs, tvecs,
          flags,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-8));
      result.message = "calibrateCamera (classic Brown) ok";
    } else if (model_id == CameraModelId::KannalaBrandt) {
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
  result.intrinsics_meta["fx"] = to_str(K.at<double>(0, 0));
  result.intrinsics_meta["fy"] = to_str(K.at<double>(1, 1));
  result.intrinsics_meta["cx"] = to_str(K.at<double>(0, 2));
  result.intrinsics_meta["cy"] = to_str(K.at<double>(1, 2));
  result.intrinsics_meta["xi"] = to_str(xi);
  result.intrinsics_meta["model"] = model;
  result.intrinsics_meta["intrinsics_profile"] =
      config.count("intrinsics_profile") ? config.at("intrinsics_profile") : "classic";
  result.intrinsics_meta["image_width"] = std::to_string(image_width);
  result.intrinsics_meta["image_height"] = std::to_string(image_height);
  result.intrinsics_meta["rms"] = to_str(rms);
  if (model_id == CameraModelId::BrownConrady) {
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["p1"] = to_str(d_at(D, 2));
    result.intrinsics_meta["p2"] = to_str(d_at(D, 3));
    result.intrinsics_meta["k3"] = to_str(d_at(D, 4));
    result.intrinsics_meta["k4"] = to_str(d_at(D, 5));
    result.intrinsics_meta["k5"] = to_str(d_at(D, 6));
    result.intrinsics_meta["k6"] = to_str(d_at(D, 7));
    result.intrinsics_meta["dist_n"] =
        std::to_string(D.rows * D.cols > 0 ? D.rows * D.cols : 5);
  } else if (model_id == CameraModelId::KannalaBrandt) {
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["k3"] = to_str(d_at(D, 2));
    result.intrinsics_meta["k4"] = to_str(d_at(D, 3));
    result.intrinsics_meta["p1"] = "0";
    result.intrinsics_meta["p2"] = "0";
    result.intrinsics_meta["dist_n"] = "4";
  } else if (model_id == CameraModelId::CMei) {
    result.intrinsics_meta["k1"] = to_str(d_at(D, 0));
    result.intrinsics_meta["k2"] = to_str(d_at(D, 1));
    result.intrinsics_meta["p1"] = to_str(d_at(D, 2));
    result.intrinsics_meta["p2"] = to_str(d_at(D, 3));
    result.intrinsics_meta["k3"] = "0";
    result.intrinsics_meta["k4"] = "0";
    result.intrinsics_meta["dist_n"] = "4";
  }
  return result;
}

HS_CALIB_REGISTER("cam_intrinsics", CamIntrinsicsCalibrator);

}  // namespace core
}  // namespace hs_calib
