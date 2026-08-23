#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"

namespace hs_calib {
namespace core {

/// \brief Tier4 Data collection parameters（§10）
struct IntrinsicsCollectorParams {
  int max_samples = 500;
  int decorrelate_eval_samples = 5;

  double max_allowed_tilt_deg = 45.0;
  bool filter_by_speed = true;
  double max_allowed_pixel_speed = 25.0;
  double max_allowed_speed = 0.1;

  bool filter_by_reprojection_error = true;
  double max_allowed_max_reprojection_error = 2.0;
  double max_allowed_rms_reprojection_error = 0.5;

  bool filter_by_2d_redundancy = true;
  double min_normalized_2d_center_difference = 0.05;
  double min_normalized_skew_difference = 0.05;
  double min_normalized_2d_size_difference = 0.05;

  bool filter_by_3d_redundancy = false;
  double min_3d_center_difference_m = 1.0;
  double min_tilt_difference_deg = 15.0;

  int heatmap_cells = 16;
  int rotation_heatmap_angle_res = 10;
  int point_2d_hist_bins = 20;
  int point_3d_hist_bins = 20;
  bool skip_frames_when_not_detection = true;

  int max_fast_calibration_samples = 20;
};

/// \brief 由 profile 填充采集重投影阈值等
IntrinsicsCollectorParams collector_params_from_profile(const IntrinsicsProfile &profile);

/// \brief 从 config map 解析（可选键覆盖默认）
IntrinsicsCollectorParams collector_params_from_config(
    const std::map<std::string, std::string> &config,
    const IntrinsicsProfile &profile);

}  // namespace core
}  // namespace hs_calib
