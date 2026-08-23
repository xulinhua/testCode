#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_session_state.hpp"

#include <chrono>
#include <string>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_pipeline.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"

namespace hs_calib {
namespace core {
namespace {

struct BatchRmsStats {
  double rms_all = -1.0;
  double rms_inlier = -1.0;
  int inlier_count = 0;
  int valid_count = 0;
};

double view_rms_px(
    const Observation &obs, const cv::Mat &K, const cv::Mat &D) {
  if (obs.correspondences.empty() || K.empty()) {
    return -1.0;
  }
  IntrinsicsView view;
  const auto &corr = obs.correspondences.front();
  view.image_points.reserve(static_cast<size_t>(corr.image_points.rows()));
  view.object_points.reserve(static_cast<size_t>(corr.object_points.rows()));
  for (int r = 0; r < corr.image_points.rows(); ++r) {
    view.image_points.emplace_back(
        static_cast<float>(corr.image_points(r, 0)),
        static_cast<float>(corr.image_points(r, 1)));
    view.object_points.emplace_back(
        static_cast<float>(corr.object_points(r, 0)),
        static_cast<float>(corr.object_points(r, 1)),
        static_cast<float>(corr.object_points(r, 2)));
  }
  cv::Mat rv, tv;
  if (!solve_board_pose(view, K, D, &rv, &tv)) {
    return -1.0;
  }
  return compute_reprojection_stats(view, K, D, rv, tv).rms;
}

BatchRmsStats batch_rms_stats(
    const ObservationBatch &batch,
    const cv::Mat &K,
    const cv::Mat &D,
    double max_rms,
    bool use_post_rejection) {
  BatchRmsStats out;
  double sum_sq_all = 0.0;
  double sum_sq_inlier = 0.0;
  for (const auto &obs : batch.items) {
    const double rms = view_rms_px(obs, K, D);
    if (rms < 0.0 || !std::isfinite(rms)) {
      continue;
    }
    sum_sq_all += rms * rms;
    ++out.valid_count;
    const bool inlier = !use_post_rejection || rms <= max_rms;
    if (inlier) {
      sum_sq_inlier += rms * rms;
      ++out.inlier_count;
    }
  }
  if (out.valid_count > 0) {
    out.rms_all = std::sqrt(sum_sq_all / static_cast<double>(out.valid_count));
  }
  if (out.inlier_count > 0) {
    out.rms_inlier =
        std::sqrt(sum_sq_inlier / static_cast<double>(out.inlier_count));
  } else if (!use_post_rejection) {
    out.rms_inlier = out.rms_all;
    out.inlier_count = out.valid_count;
  }
  return out;
}

}  // namespace

void IntrinsicsSessionState::configure(
    const IntrinsicsProfile &profile,
    const IntrinsicsCollectorParams &collector_params,
    const std::map<std::string, std::string> &solve_config) {
  profile_ = profile;
  collector_params_ = collector_params;
  solve_config_ = solve_config;
  collector_.set_profile(profile);
  collector_.set_params(collector_params);
}

void IntrinsicsSessionState::set_offline_source(bool offline) {
  collector_.set_offline_source(offline);
}

void IntrinsicsSessionState::reset() {
  collector_.reset();
  provisional_.valid = false;
  singleshot_.valid = false;
  has_singleshot_model_ = false;
  last_metrics_ = {};
  status_ = IntrinsicsOperationStatus::Idle;
  stats_ = {};
  last_result_ = {};
  has_calibrated_model_ = false;
  calibrated_K_.release();
  calibrated_D_.release();
  has_last_capture_fp_ = false;
  has_last_centroid_ = false;
}

ObservationBatch IntrinsicsSessionState::training_batch() const {
  return collector_.training_batch();
}

ObservationBatch IntrinsicsSessionState::evaluation_batch() const {
  return collector_.evaluation_batch();
}

void IntrinsicsSessionState::refresh_provisional(
    int image_width, int image_height) {
  image_width_ = image_width;
  image_height_ = image_height;
  update_provisional_intrinsics(
      collector_.training_batch(), image_width, image_height, profile_,
      &provisional_, collector_params_.max_fast_calibration_samples);
}

void IntrinsicsSessionState::set_provisional_model(
    const ProvisionalIntrinsics &model) {
  provisional_ = model;
}

cv::Mat IntrinsicsSessionState::model_K_for_metrics() const {
  if (has_calibrated_model_ && !calibrated_K_.empty()) {
    return calibrated_K_;
  }
  if (provisional_.valid && !provisional_.camera_matrix.empty()) {
    return provisional_.camera_matrix;
  }
  if (image_width_ > 0 && image_height_ > 0) {
    return make_initial_camera_matrix(image_width_, image_height_);
  }
  return cv::Mat();
}

cv::Mat IntrinsicsSessionState::model_D_for_metrics() const {
  if (has_calibrated_model_ && !calibrated_D_.empty()) {
    return calibrated_D_;
  }
  if (provisional_.valid && !provisional_.dist_coeffs.empty()) {
    return provisional_.dist_coeffs;
  }
  return make_initial_dist_coeffs(profile_.rational_coeffs);
}

void IntrinsicsSessionState::update_frame_metrics(
    const Correspondence &corr,
    int image_width,
    int image_height,
    double cell_size_m) {
  image_width_ = image_width;
  image_height_ = image_height;
  last_metrics_ = compute_board_frame_metrics(
      corr, image_width, image_height, model_K_for_metrics(),
      model_D_for_metrics(), cell_size_m);
}

CollectorRejectReason IntrinsicsSessionState::try_capture(
    Observation obs,
    const BoardFrameFingerprint &fp,
    double pixel_speed,
    IntrinsicsSampleSplit *out_split) {
  return collector_.try_add(
      std::move(obs), fp, provisional_, pixel_speed, out_split);
}

void IntrinsicsSessionState::sync_calibrated_from_result() {
  (void)has_calibrated_model_;
}

bool IntrinsicsSessionState::calibrate(std::string *error_out) {
  const auto train = collector_.training_batch();
  if (train.items.size() < 6) {
    if (error_out) {
      *error_out = "训练集至少需要 6 帧";
    }
    return false;
  }
  status_ = IntrinsicsOperationStatus::Calibrating;
  const auto t0 = std::chrono::steady_clock::now();
  try {
    auto calibrator = CalibratorRegistry::instance().create("cam_intrinsics");
    last_result_ = calibrator->calibrate(train, solve_config_);
  } catch (const std::exception &ex) {
    last_result_ = {};
    last_result_.success = false;
    last_result_.message = ex.what();
    status_ = IntrinsicsOperationStatus::Idle;
    if (error_out) {
      *error_out = ex.what();
    }
    return false;
  }
  const auto t1 = std::chrono::steady_clock::now();
  stats_.calibration_time_sec =
      std::chrono::duration<double>(t1 - t0).count();
  stats_.training_samples = collector_.training_count();
  stats_.evaluation_samples = collector_.evaluation_count();
  if (!last_result_.success) {
    status_ = IntrinsicsOperationStatus::Idle;
    if (error_out) {
      *error_out = last_result_.message;
    }
    return false;
  }

  const auto it_pre = last_result_.metrics.find("num_views_ransac");
  const auto it_post = last_result_.metrics.find("num_views");
  if (it_pre != last_result_.metrics.end()) {
    stats_.pre_rejection_inliers = static_cast<int>(it_pre->second);
  }
  if (it_post != last_result_.metrics.end()) {
    stats_.post_rejection_inliers = static_cast<int>(it_post->second);
  }
  stats_.training_rms_all = last_result_.metrics.count("reprojection_rmse")
      ? last_result_.metrics.at("reprojection_rmse")
      : -1.0;
  stats_.training_rms_inlier = stats_.training_rms_all;

  const auto &m = last_result_.intrinsics_meta;
  if (m.count("fx") && m.count("fy")) {
    calibrated_K_ = (cv::Mat_<double>(3, 3) << std::stod(m.at("fx")), 0.0,
                     m.count("cx") ? std::stod(m.at("cx")) : 0.0, 0.0,
                     std::stod(m.at("fy")),
                     m.count("cy") ? std::stod(m.at("cy")) : 0.0, 0.0, 0.0, 1.0);
    const int dist_n = m.count("dist_n") ? std::stoi(m.at("dist_n")) : 5;
    calibrated_D_ = cv::Mat::zeros(dist_n, 1, CV_64F);
    const char *keys[] = {"k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"};
    for (int i = 0; i < dist_n && i < 8; ++i) {
      if (m.count(keys[i])) {
        calibrated_D_.at<double>(i, 0) = std::stod(m.at(keys[i]));
      }
    }
    has_calibrated_model_ = true;
  }

  if (provisional_.valid && !provisional_.camera_matrix.empty()) {
    singleshot_ = provisional_;
    has_singleshot_model_ = true;
  }

  provisional_.camera_matrix = calibrated_K_;
  provisional_.dist_coeffs = calibrated_D_;
  provisional_.valid = has_calibrated_model_;
  status_ = IntrinsicsOperationStatus::Idle;
  return true;
}

bool IntrinsicsSessionState::evaluate(std::string *error_out) {
  if (!has_calibrated_model_ && !provisional_.valid) {
    if (error_out) {
      *error_out = "请先执行标定";
    }
    return false;
  }
  status_ = IntrinsicsOperationStatus::Evaluating;
  const auto t0 = std::chrono::steady_clock::now();
  const cv::Mat K = model_K_for_metrics();
  const cv::Mat D = model_D_for_metrics();
  const bool use_post = profile_.use_post_rejection;
  const double max_rms = profile_.post_rejection_max_rms_error;
  const auto train_stats =
      batch_rms_stats(collector_.training_batch(), K, D, max_rms, use_post);
  const auto eval_stats =
      batch_rms_stats(collector_.evaluation_batch(), K, D, max_rms, use_post);
  stats_.training_rms_all = train_stats.rms_all;
  stats_.training_rms_inlier = train_stats.rms_inlier;
  stats_.evaluation_rms_all = eval_stats.rms_all;
  stats_.evaluation_rms_inlier = eval_stats.rms_inlier;
  stats_.training_samples = collector_.training_count();
  stats_.evaluation_samples = collector_.evaluation_count();
  stats_.post_rejection_inliers = train_stats.inlier_count;
  stats_.eval_post_rejection_inliers = eval_stats.inlier_count;
  const auto t1 = std::chrono::steady_clock::now();
  stats_.calibration_time_sec =
      std::chrono::duration<double>(t1 - t0).count();
  status_ = IntrinsicsOperationStatus::Idle;
  return true;
}

}  // namespace core
}  // namespace hs_calib
