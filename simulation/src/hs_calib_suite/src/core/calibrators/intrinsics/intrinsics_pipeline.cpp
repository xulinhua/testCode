#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/calib3d.hpp>

#if defined(HS_CALIB_HAS_CERES) && HS_CALIB_HAS_CERES
#  include <ceres_intrinsic_camera_calibrator/ceres_intrinsic_calibrate.hpp>
#endif

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

namespace hs_calib {
namespace core {
namespace {

bool flag_on(const std::map<std::string, std::string> &config, const char *key) {
  const auto it = config.find(key);
  return it != config.end() && (it->second == "1" || it->second == "true");
}

int parse_int_local(
    const std::map<std::string, std::string> &config, const char *key, int fb) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return fb;
  }
  return std::atoi(it->second.c_str());
}

int build_opencv_flags(
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &config) {
  int flags = 0;
  if (config_flag_on(config, "fix_principal_point", "fix_principal")) {
    flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
  }
  if (config_flag_on(config, "fix_aspect_ratio", "fix_aspect")) {
    flags |= cv::CALIB_FIX_ASPECT_RATIO;
  }
  if (flag_on(config, "zero_tangent") || !profile.use_tangential) {
    flags |= cv::CALIB_ZERO_TANGENT_DIST;
  }
  if (flag_on(config, "fix_k1")) {
    flags |= cv::CALIB_FIX_K1;
  }
  if (flag_on(config, "fix_k2")) {
    flags |= cv::CALIB_FIX_K2;
  }
  if (profile.radial_coeffs < 3 || flag_on(config, "fix_k3")) {
    flags |= cv::CALIB_FIX_K3;
  }
  if (profile.rational_coeffs > 0 || flag_on(config, "rational_model")) {
    flags |= cv::CALIB_RATIONAL_MODEL;
  }
  if (config_flag_on(config, "enable_prism_model", "thin_prism")) {
    flags |= cv::CALIB_THIN_PRISM_MODEL;
  }
  if (flag_on(config, "use_intrinsic_guess")) {
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  }
  return flags;
}

struct SolveState {
  bool ok = false;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double rms = 0.0;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
};

std::vector<std::vector<cv::Point3f>> object_points_of(
    const std::vector<IntrinsicsView> &views) {
  std::vector<std::vector<cv::Point3f>> out;
  out.reserve(views.size());
  for (const auto &v : views) {
    out.push_back(v.object_points);
  }
  return out;
}

std::vector<std::vector<cv::Point2f>> image_points_of(
    const std::vector<IntrinsicsView> &views) {
  std::vector<std::vector<cv::Point2f>> out;
  out.reserve(views.size());
  for (const auto &v : views) {
    out.push_back(v.image_points);
  }
  return out;
}

SolveState solve_opencv(
    const std::vector<IntrinsicsView> &views,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &config,
    const cv::Mat &guess_k = cv::Mat(),
    const cv::Mat &guess_d = cv::Mat()) {
  SolveState out;
  if (views.size() < 3) {
    return out;
  }
  const auto obj = object_points_of(views);
  const auto img = image_points_of(views);
  out.camera_matrix = guess_k.empty()
      ? make_initial_camera_matrix(image_width, image_height)
      : guess_k.clone();
  out.dist_coeffs = guess_d.empty()
      ? make_initial_dist_coeffs(profile.rational_coeffs)
      : guess_d.clone();
  const int flags = build_opencv_flags(profile, config);
  try {
    out.rms = cv::calibrateCamera(
        obj, img, cv::Size(image_width, image_height), out.camera_matrix,
        out.dist_coeffs, out.rvecs, out.tvecs, flags);
    out.ok = std::isfinite(out.rms);
  } catch (const cv::Exception &) {
    out.ok = false;
  }
  return out;
}

SolveState solve_with_profile(
    const std::vector<IntrinsicsView> &views,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &config) {
  std::vector<IntrinsicsView> pre_views = views;
#if defined(HS_CALIB_HAS_CERES) && HS_CALIB_HAS_CERES
  if (profile.solver == IntrinsicsSolverKind::Ceres) {
    const int pre_n = parse_int_local(config, "pre_calibration_num_samples", 40);
    if (pre_n > 0 && static_cast<int>(views.size()) > pre_n) {
      pre_views.assign(views.begin(), views.begin() + static_cast<size_t>(pre_n));
    }
  }
#endif
  SolveState init =
      solve_opencv(pre_views, image_width, image_height, profile, config);
  if (!init.ok) {
    return init;
  }
#if defined(HS_CALIB_HAS_CERES) && HS_CALIB_HAS_CERES
  if (profile.solver == IntrinsicsSolverKind::Ceres) {
    tier4_ceres_intrinsic::CeresCalibrateOptions opt;
    opt.num_radial_coeffs = profile.radial_coeffs;
    opt.num_rational_coeffs = profile.rational_coeffs;
    opt.use_tangential_distortion = profile.use_tangential;
    opt.coeffs_regularization_weight = profile.coeffs_regularization_weight;
    opt.fov_regularization_weight = profile.fov_regularization_weight;
    opt.width = image_width;
    opt.height = image_height;
    const auto ceres = tier4_ceres_intrinsic::calibrate(
        object_points_of(views), image_points_of(views), init.camera_matrix,
        init.dist_coeffs, opt);
    if (ceres.ok) {
      init.camera_matrix = ceres.camera_matrix;
      init.dist_coeffs = ceres.dist_coeffs;
      init.rvecs = ceres.rvecs;
      init.tvecs = ceres.tvecs;
      init.rms = ceres.rms;
      init.ok = true;
    }
  }
#endif
  return init;
}

std::vector<IntrinsicsView> filter_by_rms(
    const std::vector<IntrinsicsView> &views,
    const SolveState &model,
    double max_rms) {
  std::vector<IntrinsicsView> out;
  auto working = views;
  update_view_poses_and_errors(
      &working, model.camera_matrix, model.dist_coeffs);
  for (const auto &v : working) {
    if (v.reproj_rms >= 0.0 && v.reproj_rms <= max_rms) {
      out.push_back(v);
    }
  }
  return out;
}

std::vector<IntrinsicsView> ransac_pre_rejection(
    const std::vector<IntrinsicsView> &views,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &config,
    SolveState *best_model) {
  if (!profile.use_ransac_pre_rejection || views.size() <=
      static_cast<size_t>(profile.pre_rejection_min_hypotheses)) {
    if (best_model != nullptr) {
      *best_model = solve_with_profile(
          views, image_width, image_height, profile, config);
    }
    return views;
  }

  std::mt19937 rng(42);
  std::vector<size_t> indices(views.size());
  for (size_t i = 0; i < views.size(); ++i) {
    indices[i] = i;
  }

  size_t best_inliers = 0;
  std::vector<IntrinsicsView> best_views;
  SolveState best_solve;

  for (int iter = 0; iter < profile.pre_rejection_iterations; ++iter) {
    std::shuffle(indices.begin(), indices.end(), rng);
    std::vector<IntrinsicsView> sample;
    sample.reserve(static_cast<size_t>(profile.pre_rejection_min_hypotheses));
    for (int i = 0; i < profile.pre_rejection_min_hypotheses; ++i) {
      sample.push_back(views[indices[static_cast<size_t>(i)]]);
    }
    const SolveState hypo = solve_opencv(
        sample, image_width, image_height, profile, config);
    if (!hypo.ok) {
      continue;
    }
    const auto inliers = filter_by_rms(
        views, hypo, profile.pre_rejection_max_rms_error);
    if (inliers.size() > best_inliers) {
      best_inliers = inliers.size();
      best_views = inliers;
      best_solve = hypo;
    }
  }

  if (best_views.empty()) {
    if (best_model != nullptr) {
      *best_model = solve_with_profile(
          views, image_width, image_height, profile, config);
    }
    return views;
  }
  if (best_model != nullptr) {
    *best_model = solve_with_profile(
        best_views, image_width, image_height, profile, config);
  }
  return best_views;
}

using BinKey = std::tuple<int, int, int>;

BinKey view_bin(const IntrinsicsView &view, const IntrinsicsProfile &profile) {
  const int cells = std::max(2, profile.subsampling_pixel_cells);
  const int cx = std::min(
      cells - 1,
      std::max(0, static_cast<int>(view.centroid_x * cells)));
  const int cy = std::min(
      cells - 1,
      std::max(0, static_cast<int>(view.centroid_y * cells)));
  const double tilt_res = std::max(1.0, profile.subsampling_tilt_resolution_deg);
  const int max_tilt_bins = std::max(
      1, static_cast<int>(profile.subsampling_max_tilt_deg / tilt_res + 0.5));
  const int tilt_bin = std::min(
      max_tilt_bins - 1,
      std::max(0, static_cast<int>(view.tilt_deg / tilt_res)));
  return {cx, cy, tilt_bin};
}

double histogram_entropy(const std::map<BinKey, int> &hist) {
  if (hist.empty()) {
    return 0.0;
  }
  int total = 0;
  for (const auto &kv : hist) {
    total += kv.second;
  }
  if (total <= 0) {
    return 0.0;
  }
  double ent = 0.0;
  for (const auto &kv : hist) {
    const double p = static_cast<double>(kv.second) / total;
    ent -= p * std::log(p);
  }
  return ent;
}

std::vector<IntrinsicsView> entropy_subsample(
    const std::vector<IntrinsicsView> &views,
    const IntrinsicsProfile &profile) {
  if (!profile.use_entropy_subsampling ||
      static_cast<int>(views.size()) <= profile.max_calibration_samples) {
    return views;
  }
  std::map<BinKey, int> hist;
  std::vector<IntrinsicsView> selected;
  std::vector<bool> used(views.size(), false);
  while (static_cast<int>(selected.size()) < profile.max_calibration_samples) {
    double best_gain = -1.0;
    int best_idx = -1;
    for (size_t i = 0; i < views.size(); ++i) {
      if (used[i]) {
        continue;
      }
      auto trial = hist;
      ++trial[view_bin(views[i], profile)];
      const double gain = histogram_entropy(trial) - histogram_entropy(hist);
      if (gain > best_gain) {
        best_gain = gain;
        best_idx = static_cast<int>(i);
      }
    }
    if (best_idx < 0) {
      break;
    }
    used[static_cast<size_t>(best_idx)] = true;
    selected.push_back(views[static_cast<size_t>(best_idx)]);
    ++hist[view_bin(views[static_cast<size_t>(best_idx)], profile)];
  }
  return selected.empty() ? views : selected;
}

}  // namespace

std::vector<IntrinsicsView> build_intrinsics_views(const ObservationBatch &batch) {
  std::vector<IntrinsicsView> views;
  for (size_t i = 0; i < batch.items.size(); ++i) {
    const auto &obs = batch.items[i];
    if (obs.correspondences.empty()) {
      continue;
    }
    const auto &c = obs.correspondences.front();
    if (c.image_points.rows() < 6 ||
        c.object_points.rows() != c.image_points.rows()) {
      continue;
    }
    IntrinsicsView view;
    view.source_index = i;
    view.object_points.reserve(static_cast<size_t>(c.image_points.rows()));
    view.image_points.reserve(static_cast<size_t>(c.image_points.rows()));
    for (int r = 0; r < c.image_points.rows(); ++r) {
      view.object_points.emplace_back(
          static_cast<float>(c.object_points(r, 0)),
          static_cast<float>(c.object_points(r, 1)),
          static_cast<float>(c.object_points(r, 2)));
      view.image_points.emplace_back(
          static_cast<float>(c.image_points(r, 0)),
          static_cast<float>(c.image_points(r, 1)));
    }
    fill_view_fingerprint(&view, obs.image_width, obs.image_height);
    views.push_back(std::move(view));
  }
  return views;
}

IntrinsicsPipelineResult run_intrinsics_pipeline(
    const std::vector<IntrinsicsView> &views,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &opencv_flags) {
  IntrinsicsPipelineResult result;
  result.num_input_views = views.size();
  if (views.size() < 3) {
    result.message = "有效观测不足（至少需要 3 帧）";
    return result;
  }
  if (image_width <= 0 || image_height <= 0) {
    result.message = "缺少图像尺寸";
    return result;
  }

  SolveState scratch;
  const auto after_ransac = ransac_pre_rejection(
      views, image_width, image_height, profile, opencv_flags, &scratch);
  result.num_after_ransac = after_ransac.size();
  if (after_ransac.size() < 3) {
    result.message = "RANSAC 预剔除后有效帧不足";
    return result;
  }

  const auto train = entropy_subsample(after_ransac, profile);
  result.num_after_subsample = train.size();

  SolveState solved =
      solve_with_profile(train, image_width, image_height, profile, opencv_flags);
  if (!solved.ok) {
    result.message = "标定求解失败";
    return result;
  }

  std::vector<IntrinsicsView> final_views = train;
  if (profile.use_post_rejection) {
    final_views = filter_by_rms(
        train, solved, profile.post_rejection_max_rms_error);
    result.num_after_post = final_views.size();
    if (final_views.size() >= 3 && final_views.size() < train.size()) {
      solved = solve_with_profile(
          final_views, image_width, image_height, profile, opencv_flags);
    }
  } else {
    result.num_after_post = train.size();
  }

  if (!solved.ok) {
    result.message = "后剔除重标定失败";
    return result;
  }

  result.ok = true;
  result.camera_matrix = solved.camera_matrix;
  result.dist_coeffs = solved.dist_coeffs;
  result.rms = solved.rms;
  result.rvecs = solved.rvecs;
  result.tvecs = solved.tvecs;
  std::ostringstream oss;
  oss << profile.display_name << " pipeline ok"
      << " (in=" << result.num_input_views << " ransac=" << result.num_after_ransac
      << " train=" << result.num_after_subsample
      << " post=" << result.num_after_post << ")";
  result.message = oss.str();
  return result;
}

bool compute_intrinsics_pipeline_stage_views(
    const ObservationBatch &training_batch,
    const ObservationBatch &evaluation_batch,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &opencv_flags,
    IntrinsicsPipelineStageViews *out,
    std::string *error_out) {
  if (out == nullptr) {
    if (error_out) {
      *error_out = "输出为空";
    }
    return false;
  }
  *out = {};
  if (image_width <= 0 || image_height <= 0) {
    if (error_out) {
      *error_out = "缺少图像尺寸";
    }
    return false;
  }
  out->training = build_intrinsics_views(training_batch);
  out->evaluation = build_intrinsics_views(evaluation_batch);
  if (out->training.size() < 3) {
    if (error_out) {
      *error_out = "训练样本不足";
    }
    return false;
  }

  SolveState scratch;
  out->pre_rejection_inliers = ransac_pre_rejection(
      out->training, image_width, image_height, profile, opencv_flags, &scratch);
  if (out->pre_rejection_inliers.size() < 3) {
    if (error_out) {
      *error_out = "RANSAC 后有效帧不足";
    }
    return false;
  }

  out->subsampled = entropy_subsample(out->pre_rejection_inliers, profile);
  if (out->subsampled.size() < 3) {
    if (error_out) {
      *error_out = "子采样后有效帧不足";
    }
    return false;
  }

  SolveState solved = solve_with_profile(
      out->subsampled, image_width, image_height, profile, opencv_flags);
  if (!solved.ok) {
    if (error_out) {
      *error_out = "阶段求解失败";
    }
    return false;
  }

  if (profile.use_post_rejection) {
    out->post_rejection_inliers = filter_by_rms(
        out->subsampled, solved, profile.post_rejection_max_rms_error);
    if (out->post_rejection_inliers.size() < 3) {
      out->post_rejection_inliers = out->subsampled;
    }
  } else {
    out->post_rejection_inliers = out->subsampled;
  }
  return true;
}

double compute_single_shot_view_rms(
    const IntrinsicsView &view,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &config) {
  if (image_width <= 0 || image_height <= 0 ||
      view.object_points.size() < 6 ||
      view.image_points.size() != view.object_points.size()) {
    return -1.0;
  }
  const std::vector<std::vector<cv::Point3f>> obj{{view.object_points}};
  const std::vector<std::vector<cv::Point2f>> img{{view.image_points}};
  cv::Mat K = make_initial_camera_matrix(image_width, image_height);
  cv::Mat D = make_initial_dist_coeffs(profile.rational_coeffs);
  const int flags = build_opencv_flags(profile, config);
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    const double rms = cv::calibrateCamera(
        obj, img, cv::Size(image_width, image_height), K, D, rvecs, tvecs,
        flags);
    if (!std::isfinite(rms) || rvecs.empty()) {
      return -1.0;
    }
    return rms;
  } catch (const cv::Exception &) {
    return -1.0;
  }
}

}  // namespace core
}  // namespace hs_calib
