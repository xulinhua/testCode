// Copyright 2024 Tier IV, Inc. (Apache-2.0) — logic from ceres_intrinsic_camera_calibrator_py.cpp
#include "ceres_intrinsic_camera_calibrator/ceres_intrinsic_calibrate.hpp"

#include <ceres_intrinsic_camera_calibrator/ceres_camera_intrinsics_optimizer.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

namespace tier4_ceres_intrinsic {
namespace {

bool valid_inputs(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const cv::Mat &initial_camera_matrix,
    const cv::Mat &initial_dist_coeffs,
    const CeresCalibrateOptions &opt) {
  if (object_points.empty() || object_points.size() != image_points.size()) {
    return false;
  }
  if (initial_camera_matrix.rows != 3 || initial_camera_matrix.cols != 3) {
    return false;
  }
  const int expect_d = opt.num_rational_coeffs > 0 ? 8 : 5;
  if (initial_dist_coeffs.rows * initial_dist_coeffs.cols != expect_d) {
    return false;
  }
  if (opt.num_radial_coeffs < 0 || opt.num_radial_coeffs > 3) {
    return false;
  }
  if (opt.num_rational_coeffs < 0 || opt.num_rational_coeffs > 3) {
    return false;
  }
  if (opt.width <= 0 || opt.height <= 0) {
    return false;
  }
  return true;
}

}  // namespace

CeresCalibrateResult calibrate(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const cv::Mat &initial_camera_matrix,
    const cv::Mat &initial_dist_coeffs,
    const CeresCalibrateOptions &opt) {
  CeresCalibrateResult out;
  if (!valid_inputs(object_points, image_points, initial_camera_matrix,
                    initial_dist_coeffs, opt)) {
    return out;
  }

  cv::Mat_<double> K0, D0, K, D;
  initial_camera_matrix.convertTo(K0, CV_64F);
  initial_dist_coeffs.convertTo(D0, CV_64F);

  std::vector<cv::Mat> rvecs0, tvecs0, rvecs, tvecs;
  for (size_t i = 0; i < object_points.size(); ++i) {
    cv::Mat rv, tv;
    cv::solvePnP(object_points[i], image_points[i], K0, D0, rv, tv);
    rvecs0.push_back(rv);
    tvecs0.push_back(tv);
  }

  CeresCameraIntrinsicsOptimizer optimizer;
  optimizer.setRadialDistortionCoefficients(opt.num_radial_coeffs);
  optimizer.setTangentialDistortion(opt.use_tangential_distortion);
  optimizer.setRationalDistortionCoefficients(opt.num_rational_coeffs);
  optimizer.setCoeffsRegularizationWeight(opt.coeffs_regularization_weight);
  optimizer.setFovRegularizationWeight(opt.fov_regularization_weight);
  optimizer.setSourceDimensions(opt.width, opt.height);
  optimizer.setVerbose(opt.verbose);
  optimizer.setData(K0, D0, object_points, image_points, rvecs0, tvecs0);
  optimizer.dataToPlaceholders();
  optimizer.evaluate();
  optimizer.solve(false);
  optimizer.placeholdersToData();

  if (opt.fov_regularization_weight > 0.0) {
    const double init_avg = optimizer.getAvgCeresError();
    double best_fov = optimizer.evaluateFov();
    if (best_fov > CeresCameraIntrinsicsOptimizer::FOV_THR) {
      int attempt = 0;
      while (attempt < CeresCameraIntrinsicsOptimizer::SOLVE_MAX_ATTEMPTS) {
        if (attempt > 0) {
          optimizer.solve(false);
        }
        ++attempt;
        optimizer.solve(true);
        const double avg = optimizer.getAvgCeresError();
        const double adj = init_avg - avg;
        const double fov = optimizer.evaluateFov();
        if (fov < best_fov &&
            adj >= -CeresCameraIntrinsicsOptimizer::REPR_THR) {
          best_fov = fov;
          optimizer.placeholdersToData();
          if (best_fov <= CeresCameraIntrinsicsOptimizer::FOV_THR) {
            break;
          }
        }
      }
    }
  }

  out.rms = optimizer.getSolution(K, D, rvecs, tvecs);
  out.camera_matrix = K;
  out.dist_coeffs = D;
  out.rvecs = std::move(rvecs);
  out.tvecs = std::move(tvecs);
  out.ok = std::isfinite(out.rms);
  return out;
}

}  // namespace tier4_ceres_intrinsic
