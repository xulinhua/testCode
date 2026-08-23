// Copyright 2024 Tier IV, Inc. (Apache-2.0) — C++ API extracted for hs_calib_suite.
#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace tier4_ceres_intrinsic {

/// \brief Ceres 联合优化内参（对齐 Tier4 intrinsic_camera_calibrator）
struct CeresCalibrateOptions {
  int num_radial_coeffs = 3;
  int num_rational_coeffs = 0;
  bool use_tangential_distortion = true;
  double coeffs_regularization_weight = 0.0;
  double fov_regularization_weight = 0.0;
  int width = 0;
  int height = 0;
  bool verbose = false;
};

struct CeresCalibrateResult {
  bool ok = false;
  double rms = 0.0;
  cv::Mat camera_matrix;   ///< 3x3 CV_64F
  cv::Mat dist_coeffs;     ///< 5 or 8 CV_64F
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
};

/// \brief 多视图 Ceres 标定（需有效初值，通常先 OpenCV 粗标定）
CeresCalibrateResult calibrate(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const cv::Mat &initial_camera_matrix,
    const cv::Mat &initial_dist_coeffs,
    const CeresCalibrateOptions &opt);

}  // namespace tier4_ceres_intrinsic
