#pragma once

#include <map>
#include <string>

namespace hs_calib {
namespace core {

enum class IntrinsicsSolverKind { OpenCV, Ceres };

/// \brief 内参求解预设：classic（经典 OpenCV）或 Tier4（general / c1 / ceres / c2）
struct IntrinsicsProfile {
  std::string id = "classic";
  std::string display_name = "Classic";

  IntrinsicsSolverKind solver = IntrinsicsSolverKind::OpenCV;
  int radial_coeffs = 2;
  int rational_coeffs = 0;
  bool use_tangential = true;
  double coeffs_regularization_weight = 0.0;
  double fov_regularization_weight = 0.0;

  bool use_ransac_pre_rejection = true;
  int pre_rejection_iterations = 100;
  int pre_rejection_min_hypotheses = 6;
  double pre_rejection_max_rms_error = 0.5;

  int max_calibration_samples = 80;
  bool use_entropy_subsampling = true;
  int subsampling_pixel_cells = 16;
  double subsampling_tilt_resolution_deg = 15.0;
  double subsampling_max_tilt_deg = 45.0;

  bool use_post_rejection = true;
  double post_rejection_max_rms_error = 0.5;

  bool filter_capture_by_reproj = true;
  double capture_max_reproj_error = 2.0;
  double capture_max_rms_reproj_error = 0.5;
};

/// \brief 解析 profile id；未知 id 返回 classic
IntrinsicsProfile intrinsics_profile_from_id(const std::string &id);

/// \brief 是否为 Tier4 流水线预设（非 classic）
bool is_tier4_intrinsics_profile(const std::string &profile_id);

/// \brief 从 config 读取 intrinsics_profile（默认 classic）
IntrinsicsProfile intrinsics_profile_from_config(
    const std::map<std::string, std::string> &config);

/// \brief config 是否启用 Tier4 内参流水线（训练/评估分流、RANSAC 等）
bool tier4_intrinsics_enabled(
    const std::map<std::string, std::string> &config);

}  // namespace core
}  // namespace hs_calib
