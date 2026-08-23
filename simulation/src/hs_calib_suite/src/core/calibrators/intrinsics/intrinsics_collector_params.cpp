#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_collector_params.hpp"

#include <cstdlib>

namespace hs_calib {
namespace core {
namespace {

double parse_double(
    const std::map<std::string, std::string> &config,
    const char *key,
    double fallback) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return fallback;
  }
  return std::strtod(it->second.c_str(), nullptr);
}

int parse_int(
    const std::map<std::string, std::string> &config,
    const char *key,
    int fallback) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return fallback;
  }
  return std::atoi(it->second.c_str());
}

bool parse_bool(
    const std::map<std::string, std::string> &config,
    const char *key,
    bool fallback) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return fallback;
  }
  const std::string v = it->second;
  return v == "1" || v == "true" || v == "True" || v == "yes";
}

}  // namespace

IntrinsicsCollectorParams collector_params_from_profile(
    const IntrinsicsProfile &profile) {
  IntrinsicsCollectorParams p;
  p.filter_by_reprojection_error = profile.filter_capture_by_reproj;
  p.max_allowed_max_reprojection_error = profile.capture_max_reproj_error;
  p.max_allowed_rms_reprojection_error = profile.capture_max_rms_reproj_error;
  return p;
}

IntrinsicsCollectorParams collector_params_from_config(
    const std::map<std::string, std::string> &config,
    const IntrinsicsProfile &profile) {
  IntrinsicsCollectorParams p = collector_params_from_profile(profile);
  p.max_samples = parse_int(config, "collector_max_samples", p.max_samples);
  p.decorrelate_eval_samples = parse_int(
      config, "decorrelate_eval_samples",
      parse_int(config, "collector_decorrelate_eval", p.decorrelate_eval_samples));
  p.max_allowed_tilt_deg =
      parse_double(config, "collector_max_tilt", p.max_allowed_tilt_deg);
  p.filter_by_speed = parse_bool(config, "collector_filter_speed", p.filter_by_speed);
  p.max_allowed_pixel_speed = parse_double(
      config, "collector_max_pixel_speed", p.max_allowed_pixel_speed);
  p.max_allowed_speed =
      parse_double(config, "collector_max_speed", p.max_allowed_speed);
  p.filter_by_reprojection_error =
      parse_bool(config, "collector_filter_reproj", p.filter_by_reprojection_error);
  p.max_allowed_max_reprojection_error = parse_double(
      config, "max_allowed_max_reprojection_error",
      p.max_allowed_max_reprojection_error);
  p.max_allowed_rms_reprojection_error = parse_double(
      config, "max_allowed_rms_reprojection_error",
      p.max_allowed_rms_reprojection_error);
  p.filter_by_2d_redundancy =
      parse_bool(config, "collector_filter_2d", p.filter_by_2d_redundancy);
  p.min_normalized_2d_center_difference = parse_double(
      config, "collector_min_center_diff", p.min_normalized_2d_center_difference);
  p.min_normalized_skew_difference = parse_double(
      config, "collector_min_skew_diff", p.min_normalized_skew_difference);
  p.min_normalized_2d_size_difference = parse_double(
      config, "collector_min_size_diff", p.min_normalized_2d_size_difference);
  p.filter_by_3d_redundancy =
      parse_bool(config, "collector_filter_3d", p.filter_by_3d_redundancy);
  p.min_3d_center_difference_m = parse_double(
      config, "collector_min_3d_center_m", p.min_3d_center_difference_m);
  p.min_tilt_difference_deg = parse_double(
      config, "collector_min_tilt_diff_deg", p.min_tilt_difference_deg);
  p.heatmap_cells = parse_int(config, "collector_heatmap_cells", p.heatmap_cells);
  p.rotation_heatmap_angle_res = parse_int(
      config, "rotation_heatmap_angle_res", p.rotation_heatmap_angle_res);
  p.point_2d_hist_bins =
      parse_int(config, "collector_point_2d_hist_bins", p.point_2d_hist_bins);
  p.point_3d_hist_bins =
      parse_int(config, "collector_point_3d_hist_bins", p.point_3d_hist_bins);
  p.skip_frames_when_not_detection = parse_bool(
      config, "skip_frames_when_not_detection", p.skip_frames_when_not_detection);
  p.max_fast_calibration_samples = parse_int(
      config, "max_fast_calibration_samples", p.max_fast_calibration_samples);
  return p;
}

}  // namespace core
}  // namespace hs_calib
