#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"

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
  const std::string &v = it->second;
  return v == "1" || v == "true" || v == "True" || v == "yes";
}

void set_bool(std::map<std::string, std::string> *m, const char *k, bool v) {
  (*m)[k] = v ? "true" : "false";
}

void set_int(std::map<std::string, std::string> *m, const char *k, int v) {
  (*m)[k] = std::to_string(v);
}

void set_double(std::map<std::string, std::string> *m, const char *k, double v) {
  (*m)[k] = std::to_string(v);
}

}  // namespace

IntrinsicsProfile profile_from_config_map(
    const std::map<std::string, std::string> &config) {
  IntrinsicsProfile p = intrinsics_profile_from_config(config);

  p.use_ransac_pre_rejection =
      parse_bool(config, "use_ransac_pre_rejection", p.use_ransac_pre_rejection);
  p.pre_rejection_iterations =
      parse_int(config, "pre_rejection_iterations", p.pre_rejection_iterations);
  p.pre_rejection_min_hypotheses = parse_int(
      config, "pre_rejection_min_hypotheses", p.pre_rejection_min_hypotheses);
  p.pre_rejection_max_rms_error = parse_double(
      config, "pre_rejection_max_rms_error", p.pre_rejection_max_rms_error);

  p.max_calibration_samples =
      parse_int(config, "max_calibration_samples", p.max_calibration_samples);
  p.use_entropy_subsampling = parse_bool(
      config, "use_entropy_maximization_subsampling", p.use_entropy_subsampling);
  p.subsampling_pixel_cells =
      parse_int(config, "subsampling_pixel_cells", p.subsampling_pixel_cells);
  p.subsampling_tilt_resolution_deg = parse_double(
      config, "subsampling_tilt_resolution", p.subsampling_tilt_resolution_deg);
  p.subsampling_max_tilt_deg =
      parse_double(config, "subsampling_max_tilt_deg", p.subsampling_max_tilt_deg);

  p.use_post_rejection =
      parse_bool(config, "use_post_rejection", p.use_post_rejection);
  p.post_rejection_max_rms_error = parse_double(
      config, "post_rejection_max_rms_error", p.post_rejection_max_rms_error);

  p.radial_coeffs =
      parse_int(config, "radial_distortion_coefficients", p.radial_coeffs);
  p.rational_coeffs = parse_int(
      config, "rational_distortion_coefficients", p.rational_coeffs);
  p.use_tangential =
      parse_bool(config, "use_tangential_distortion", p.use_tangential);

  p.coeffs_regularization_weight = parse_double(
      config, "coeffs_regularization_weight", p.coeffs_regularization_weight);
  p.fov_regularization_weight = parse_double(
      config, "fov_regularization_weight", p.fov_regularization_weight);

  p.filter_capture_by_reproj = parse_bool(
      config, "filter_by_reprojection_error", p.filter_capture_by_reproj);
  p.capture_max_reproj_error = parse_double(
      config, "max_allowed_max_reprojection_error", p.capture_max_reproj_error);
  p.capture_max_rms_reproj_error = parse_double(
      config, "max_allowed_rms_reprojection_error",
      p.capture_max_rms_reproj_error);

  const auto solver_it = config.find("intrinsics_solver");
  if (solver_it != config.end()) {
    if (solver_it->second == "ceres" || solver_it->second == "Ceres") {
      p.solver = IntrinsicsSolverKind::Ceres;
    } else {
      p.solver = IntrinsicsSolverKind::OpenCV;
    }
  }
  return p;
}

IntrinsicsCalibrationExtras calibration_extras_from_config(
    const std::map<std::string, std::string> &config,
    const IntrinsicsCalibrationExtras &defaults) {
  IntrinsicsCalibrationExtras e = defaults;
  e.pre_calibration_num_samples = parse_int(
      config, "pre_calibration_num_samples", e.pre_calibration_num_samples);
  e.plot_calibration_data_statistics = parse_bool(
      config, "plot_calibration_data_statistics",
      e.plot_calibration_data_statistics);
  e.plot_calibration_results_statistics = parse_bool(
      config, "plot_calibration_results_statistics",
      e.plot_calibration_results_statistics);
  e.viz_pixel_cells = parse_int(config, "viz_pixel_cells", e.viz_pixel_cells);
  e.viz_tilt_resolution_deg =
      parse_double(config, "viz_tilt_resolution", e.viz_tilt_resolution_deg);
  e.viz_max_tilt_deg =
      parse_double(config, "viz_max_tilt_deg", e.viz_max_tilt_deg);
  e.viz_z_cells = parse_int(config, "viz_z_cells", e.viz_z_cells);
  if (config.count("enable_prism_model") > 0 || config.count("thin_prism") > 0) {
    e.enable_prism_model =
        config_flag_on(config, "enable_prism_model", "thin_prism");
  }
  if (config.count("fix_principal_point") > 0 || config.count("fix_principal") > 0) {
    e.fix_principal_point =
        config_flag_on(config, "fix_principal_point", "fix_principal");
  }
  if (config.count("fix_aspect_ratio") > 0 || config.count("fix_aspect") > 0) {
    e.fix_aspect_ratio =
        config_flag_on(config, "fix_aspect_ratio", "fix_aspect");
  }
  e.use_lu_decomposition =
      parse_bool(config, "use_lu_decomposition", e.use_lu_decomposition);
  e.use_qr_decomposition =
      parse_bool(config, "use_qr_decomposition", e.use_qr_decomposition);
  return e;
}

void apply_calibration_to_config(
    const IntrinsicsProfile &profile,
    const IntrinsicsCalibrationExtras &extras,
    std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  auto &m = *config;
  m["intrinsics_profile"] = profile.id;
  m["intrinsics_solver"] =
      profile.solver == IntrinsicsSolverKind::Ceres ? "ceres" : "opencv";

  set_bool(&m, "use_ransac_pre_rejection", profile.use_ransac_pre_rejection);
  set_int(&m, "pre_rejection_iterations", profile.pre_rejection_iterations);
  set_int(&m, "pre_rejection_min_hypotheses", profile.pre_rejection_min_hypotheses);
  set_double(&m, "pre_rejection_max_rms_error", profile.pre_rejection_max_rms_error);

  set_int(&m, "max_calibration_samples", profile.max_calibration_samples);
  set_int(&m, "max_fast_calibration_samples", 20);
  set_bool(&m, "use_entropy_maximization_subsampling", profile.use_entropy_subsampling);
  set_int(&m, "subsampling_pixel_cells", profile.subsampling_pixel_cells);
  set_double(&m, "subsampling_tilt_resolution", profile.subsampling_tilt_resolution_deg);
  set_double(&m, "subsampling_max_tilt_deg", profile.subsampling_max_tilt_deg);

  set_bool(&m, "use_post_rejection", profile.use_post_rejection);
  set_double(&m, "post_rejection_max_rms_error", profile.post_rejection_max_rms_error);

  set_bool(&m, "plot_calibration_data_statistics", extras.plot_calibration_data_statistics);
  set_bool(&m, "plot_calibration_results_statistics",
          extras.plot_calibration_results_statistics);
  set_int(&m, "viz_pixel_cells", extras.viz_pixel_cells);
  set_double(&m, "viz_tilt_resolution", extras.viz_tilt_resolution_deg);
  set_double(&m, "viz_max_tilt_deg", extras.viz_max_tilt_deg);
  set_int(&m, "viz_z_cells", extras.viz_z_cells);

  set_int(&m, "radial_distortion_coefficients", profile.radial_coeffs);
  set_int(&m, "rational_distortion_coefficients", profile.rational_coeffs);
  set_bool(&m, "use_tangential_distortion", profile.use_tangential);

  set_int(&m, "pre_calibration_num_samples", extras.pre_calibration_num_samples);
  set_double(&m, "coeffs_regularization_weight", profile.coeffs_regularization_weight);
  set_double(&m, "fov_regularization_weight", profile.fov_regularization_weight);

  set_bool(&m, "enable_prism_model", extras.enable_prism_model);
  set_bool(&m, "fix_principal_point", extras.fix_principal_point);
  set_bool(&m, "fix_aspect_ratio", extras.fix_aspect_ratio);
  set_bool(&m, "thin_prism", extras.enable_prism_model);
  set_bool(&m, "fix_principal", extras.fix_principal_point);
  set_bool(&m, "fix_aspect", extras.fix_aspect_ratio);
  set_bool(&m, "rational_model", profile.rational_coeffs > 0);
  set_bool(&m, "use_lu_decomposition", extras.use_lu_decomposition);
  set_bool(&m, "use_qr_decomposition", extras.use_qr_decomposition);
}

void apply_collector_to_config(
    const IntrinsicsCollectorParams &p, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  auto &m = *config;
  set_int(&m, "collector_max_samples", p.max_samples);
  set_int(&m, "decorrelate_eval_samples", p.decorrelate_eval_samples);

  set_double(&m, "collector_max_tilt", p.max_allowed_tilt_deg);
  set_bool(&m, "collector_filter_speed", p.filter_by_speed);
  set_double(&m, "collector_max_pixel_speed", p.max_allowed_pixel_speed);
  set_double(&m, "collector_max_speed", p.max_allowed_speed);

  set_bool(&m, "collector_filter_reproj", p.filter_by_reprojection_error);
  set_double(&m, "max_allowed_max_reprojection_error",
             p.max_allowed_max_reprojection_error);
  set_double(&m, "max_allowed_rms_reprojection_error",
             p.max_allowed_rms_reprojection_error);

  set_bool(&m, "collector_filter_2d", p.filter_by_2d_redundancy);
  set_double(&m, "collector_min_center_diff", p.min_normalized_2d_center_difference);
  set_double(&m, "collector_min_skew_diff", p.min_normalized_skew_difference);
  set_double(&m, "collector_min_size_diff", p.min_normalized_2d_size_difference);

  set_bool(&m, "collector_filter_3d", p.filter_by_3d_redundancy);
  set_double(&m, "collector_min_3d_center_m", p.min_3d_center_difference_m);
  set_double(&m, "collector_min_tilt_diff_deg", p.min_tilt_difference_deg);

  set_int(&m, "collector_heatmap_cells", p.heatmap_cells);
  set_int(&m, "rotation_heatmap_angle_res", p.rotation_heatmap_angle_res);
  set_int(&m, "collector_point_2d_hist_bins", p.point_2d_hist_bins);
  set_int(&m, "collector_point_3d_hist_bins", p.point_3d_hist_bins);
  set_bool(&m, "skip_frames_when_not_detection", p.skip_frames_when_not_detection);
  set_int(&m, "max_fast_calibration_samples", p.max_fast_calibration_samples);
}

ChessDetectorParams chess_detector_from_config(
    const std::map<std::string, std::string> &config) {
  ChessDetectorParams p;
  p.adaptive_thresh = parse_bool(config, "cb_adaptive", p.adaptive_thresh);
  p.normalize_image = parse_bool(config, "cb_normalize", p.normalize_image);
  p.fast_check = parse_bool(config, "cb_fast_check", p.fast_check);
  p.resized_detection =
      parse_bool(config, "cb_resized_detection", p.resized_detection);
  p.resized_max_resolution =
      parse_int(config, "cb_resized_max_resolution", p.resized_max_resolution);
  p.sub_pixel_refinement =
      parse_bool(config, "cb_sub_pixel_refinement", p.sub_pixel_refinement);
  p.max_lost_frames =
      parse_int(config, "cb_max_lost_frames", p.max_lost_frames);
  p.padding = parse_int(config, "cb_padding", p.padding);
  return p;
}

void apply_chess_detector_to_config(
    const ChessDetectorParams &p, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  set_bool(config, "cb_adaptive", p.adaptive_thresh);
  set_bool(config, "cb_normalize", p.normalize_image);
  set_bool(config, "cb_fast_check", p.fast_check);
  set_bool(config, "cb_resized_detection", p.resized_detection);
  set_int(config, "cb_resized_max_resolution", p.resized_max_resolution);
  set_bool(config, "cb_sub_pixel_refinement", p.sub_pixel_refinement);
  set_int(config, "cb_max_lost_frames", p.max_lost_frames);
  set_int(config, "cb_padding", p.padding);
}

DotDetectorParams dot_detector_from_config(
    const std::map<std::string, std::string> &config) {
  DotDetectorParams p;
  p.symmetric_grid = parse_bool(config, "dot_symmetric_grid", p.symmetric_grid);
  p.clustering = parse_bool(config, "dot_clustering", p.clustering);
  p.filter_by_area = parse_bool(config, "dot_filter_by_area", p.filter_by_area);
  p.min_area_percentage =
      parse_double(config, "dot_min_area_percentage", p.min_area_percentage);
  p.max_area_percentage =
      parse_double(config, "dot_max_area_percentage", p.max_area_percentage);
  p.min_dist_between_blobs_percentage = parse_double(
      config, "dot_min_dist_between_blobs_percentage",
      p.min_dist_between_blobs_percentage);
  p.resized_detection =
      parse_bool(config, "dot_resized_detection", p.resized_detection);
  p.resized_max_resolution =
      parse_int(config, "dot_resized_max_resolution", p.resized_max_resolution);
  return p;
}

void apply_dot_detector_to_config(
    const DotDetectorParams &p, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  set_bool(config, "dot_symmetric_grid", p.symmetric_grid);
  set_bool(config, "dot_clustering", p.clustering);
  set_bool(config, "dot_filter_by_area", p.filter_by_area);
  set_double(config, "dot_min_area_percentage", p.min_area_percentage);
  set_double(config, "dot_max_area_percentage", p.max_area_percentage);
  set_double(config, "dot_min_dist_between_blobs_percentage",
           p.min_dist_between_blobs_percentage);
  set_bool(config, "dot_resized_detection", p.resized_detection);
  set_int(config, "dot_resized_max_resolution", p.resized_max_resolution);
}

AprilgridDetectorParams aprilgrid_detector_from_config(
    const std::map<std::string, std::string> &config) {
  AprilgridDetectorParams p;
  p.nthreads = parse_int(config, "april_nthreads", p.nthreads);
  p.quad_decimate = parse_int(config, "april_quad_decimate", p.quad_decimate);
  p.quad_sigma = parse_double(config, "april_quad_sigma", p.quad_sigma);
  p.refine_edges = parse_bool(config, "april_refine_edges", p.refine_edges);
  p.decode_sharpening =
      parse_double(config, "april_decode_sharpening", p.decode_sharpening);
  p.debug = parse_bool(config, "april_debug", p.debug);
  p.max_hamming_error =
      parse_int(config, "april_max_hamming_error", p.max_hamming_error);
  p.min_margin = parse_double(config, "april_min_margin", p.min_margin);
  p.min_detection_ratio =
      parse_double(config, "april_min_detection_ratio", p.min_detection_ratio);
  return p;
}

void apply_aprilgrid_detector_to_config(
    const AprilgridDetectorParams &p, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  set_int(config, "april_nthreads", p.nthreads);
  set_int(config, "april_quad_decimate", p.quad_decimate);
  set_double(config, "april_quad_sigma", p.quad_sigma);
  set_bool(config, "april_refine_edges", p.refine_edges);
  set_double(config, "april_decode_sharpening", p.decode_sharpening);
  set_bool(config, "april_debug", p.debug);
  set_int(config, "april_max_hamming_error", p.max_hamming_error);
  set_double(config, "april_min_margin", p.min_margin);
  set_double(config, "april_min_detection_ratio", p.min_detection_ratio);
}

CharucoDetectorParams charuco_detector_from_config(
    const std::map<std::string, std::string> &config) {
  CharucoDetectorParams p;
  p.adaptive_thresh_win_size_min = parse_int(
      config, "charuco_adaptive_win_min", p.adaptive_thresh_win_size_min);
  p.adaptive_thresh_win_size_max = parse_int(
      config, "charuco_adaptive_win_max", p.adaptive_thresh_win_size_max);
  p.marker_length_m =
      parse_double(config, "charuco_marker_length", p.marker_length_m);
  return p;
}

void apply_charuco_detector_to_config(
    const CharucoDetectorParams &p, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  set_int(config, "charuco_adaptive_win_min", p.adaptive_thresh_win_size_min);
  set_int(config, "charuco_adaptive_win_max", p.adaptive_thresh_win_size_max);
  set_double(config, "charuco_marker_length", p.marker_length_m);
}

std::string detector_target_from_config(
    const std::map<std::string, std::string> &config) {
  const auto it = config.find("target");
  if (it != config.end()) {
    return it->second;
  }
  return "chessboard";
}

void merge_tier4_intrinsics_defaults(std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  auto &m = *config;
  auto set_if_missing = [&](const char *k, const std::string &v) {
    if (m.find(k) == m.end()) {
      m[k] = v;
    }
  };

  set_if_missing("intrinsics_profile", "classic");
  set_if_missing("gui.stats_backend", "qt");
  set_if_missing("min_confidence", "0.40");
  set_if_missing("min_diversity", "0.08");
  if (!tier4_intrinsics_enabled(m)) {
    return;
  }

  const IntrinsicsProfile active_profile = intrinsics_profile_from_config(m);
  IntrinsicsCalibrationExtras extras;
  const IntrinsicsCollectorParams collector =
      collector_params_from_profile(active_profile);
  const ChessDetectorParams chess;
  const DotDetectorParams dot;
  const AprilgridDetectorParams april;
  const CharucoDetectorParams ch;

  std::map<std::string, std::string> tier4;
  apply_calibration_to_config(active_profile, extras, &tier4);
  apply_collector_to_config(collector, &tier4);
  apply_chess_detector_to_config(chess, &tier4);
  apply_dot_detector_to_config(dot, &tier4);
  apply_aprilgrid_detector_to_config(april, &tier4);
  apply_charuco_detector_to_config(ch, &tier4);
  for (const auto &kv : tier4) {
    set_if_missing(kv.first.c_str(), kv.second);
  }
  set_if_missing("intrinsics_solver", "opencv");
}

bool config_flag_on(
    const std::map<std::string, std::string> &config,
    const char *primary_key,
    const char *alias_key) {
  if (parse_bool(config, primary_key, false)) {
    return true;
  }
  if (alias_key != nullptr && parse_bool(config, alias_key, false)) {
    return true;
  }
  return false;
}

void apply_tier4_profile_bundle(
    const std::string &profile_id, std::map<std::string, std::string> *config) {
  if (config == nullptr) {
    return;
  }
  static const char *kPreserve[] = {
      "target", "squares_x", "squares_y", "square_length", "marker_length",
      "dictionary", "model", "tag_spacing", "circle_diameter", "detect_completeness",
      "min_views", "min_confidence", "min_diversity", "auto_cooldown_ms",
      "min_board_area", "max_board_area", "max_tag_distance",
      "camera_yaml", "left_camera_yaml", "right_camera_yaml",
      "image_frame", "camera_link_frame", "camera_info_topic", "intrinsics_source",
      "parent_frame", "child_frame", "method", "stereo_side",
      "gui.stats_backend", "viz_corners", "viz_hull", "viz_conf", "viz_aruco",
      "viz_marker_radius", "cb_filter_quads", "subpix_win",
      "fix_k1", "fix_k2", "fix_k3", "zero_tangent", "use_intrinsic_guess",
      "use_rectified_image",
  };
  std::map<std::string, std::string> preserved;
  for (const char *k : kPreserve) {
    const auto it = config->find(k);
    if (it != config->end()) {
      preserved[k] = it->second;
    }
  }

  const IntrinsicsProfile profile = intrinsics_profile_from_id(profile_id);
  IntrinsicsCalibrationExtras extras;
  const IntrinsicsCollectorParams collector =
      collector_params_from_profile(profile);
  const ChessDetectorParams chess;
  const DotDetectorParams dot;
  const AprilgridDetectorParams april;
  const CharucoDetectorParams ch;

  std::map<std::string, std::string> fresh;
  apply_calibration_to_config(profile, extras, &fresh);
  apply_collector_to_config(collector, &fresh);
  apply_chess_detector_to_config(chess, &fresh);
  apply_dot_detector_to_config(dot, &fresh);
  apply_aprilgrid_detector_to_config(april, &fresh);
  apply_charuco_detector_to_config(ch, &fresh);

  *config = std::move(fresh);
  for (const auto &kv : preserved) {
    (*config)[kv.first] = kv.second;
  }
}

}  // namespace core
}  // namespace hs_calib
