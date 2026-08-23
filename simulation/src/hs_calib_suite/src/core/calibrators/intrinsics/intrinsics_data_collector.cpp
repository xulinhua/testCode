#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"

#include <algorithm>
#include <cmath>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

namespace hs_calib {
namespace core {
namespace {

bool differs_enough(double a, double b, double min_diff) {
  return std::fabs(a - b) >= min_diff;
}

}  // namespace

void IntrinsicsDataCollector::reset() {
  training_.clear();
  evaluation_.clear();
  training_grid_.clear();
  evaluation_grid_.clear();
  has_last_fp_ = false;
}

ObservationBatch IntrinsicsDataCollector::training_batch() const {
  ObservationBatch batch;
  batch.items.reserve(training_.size());
  for (const auto &s : training_) {
    batch.items.push_back(s.observation);
  }
  return batch;
}

ObservationBatch IntrinsicsDataCollector::evaluation_batch() const {
  ObservationBatch batch;
  batch.items.reserve(evaluation_.size());
  for (const auto &s : evaluation_) {
    batch.items.push_back(s.observation);
  }
  return batch;
}

ObservationBatch IntrinsicsDataCollector::all_batch() const {
  ObservationBatch batch = training_batch();
  const auto eval = evaluation_batch();
  batch.items.insert(
      batch.items.end(), eval.items.begin(), eval.items.end());
  return batch;
}

double IntrinsicsDataCollector::training_occupancy_percent() const {
  return occupancy_percent(training_grid_);
}

double IntrinsicsDataCollector::evaluation_occupancy_percent() const {
  return occupancy_percent(evaluation_grid_);
}

bool IntrinsicsDataCollector::passes_tilt(const BoardFrameFingerprint &fp) const {
  return fp.tilt_deg <= params_.max_allowed_tilt_deg;
}

bool IntrinsicsDataCollector::passes_speed(double pixel_speed) const {
  if (offline_source_ || !params_.filter_by_speed) {
    return true;
  }
  return pixel_speed <= params_.max_allowed_pixel_speed;
}

bool IntrinsicsDataCollector::passes_reprojection(
    const Observation &obs,
    const ProvisionalIntrinsics &model,
    double *rms_out,
    double *max_out) const {
  if (!params_.filter_by_reprojection_error) {
    return true;
  }
  if (obs.correspondences.empty()) {
    return false;
  }
  const auto &corr = obs.correspondences.front();
  if (model.valid) {
    return capture_passes_reprojection_filter(
        corr, obs.image_width, obs.image_height, model, profile_, rms_out,
        max_out);
  }
  // partial calib 未就绪时不做重投影门槛（避免用初始 K 误拒）
  return true;
}

bool IntrinsicsDataCollector::is_redundant_2d(
    const BoardFrameFingerprint &fp,
    const std::vector<CollectedIntrinsicsSample> &pool,
    int compare_last_n) const {
  if (!params_.filter_by_2d_redundancy || pool.empty()) {
    return false;
  }
  const int n = static_cast<int>(pool.size());
  const int start = std::max(0, n - std::max(1, compare_last_n));
  for (int i = start; i < n; ++i) {
    const auto &other = pool[static_cast<size_t>(i)].fingerprint;
    const bool center_ok = differs_enough(
        fp.centroid_x, other.centroid_x,
        params_.min_normalized_2d_center_difference);
    const bool skew_ok = differs_enough(
        fp.normalized_skew, other.normalized_skew,
        params_.min_normalized_skew_difference);
    const bool size_ok = differs_enough(
        fp.normalized_size, other.normalized_size,
        params_.min_normalized_2d_size_difference);
    if (!center_ok && !skew_ok && !size_ok) {
      return true;
    }
  }
  return false;
}

bool IntrinsicsDataCollector::is_redundant_3d(
    const BoardFrameFingerprint &fp,
    const std::vector<CollectedIntrinsicsSample> &pool,
    int compare_last_n) const {
  if (!params_.filter_by_3d_redundancy || !fp.has_pose || pool.empty()) {
    return false;
  }
  const int n = static_cast<int>(pool.size());
  const int start = std::max(0, n - std::max(1, compare_last_n));
  for (int i = start; i < n; ++i) {
    const auto &other = pool[static_cast<size_t>(i)].fingerprint;
    if (!other.has_pose) {
      continue;
    }
    const cv::Vec3d d = fp.position_m - other.position_m;
    const double dist = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    const bool center_ok = dist >= params_.min_3d_center_difference_m;
    const bool tilt_x_ok = differs_enough(
        fp.rough_angle_x_deg, other.rough_angle_x_deg,
        params_.min_tilt_difference_deg);
    const bool tilt_y_ok = differs_enough(
        fp.rough_angle_y_deg, other.rough_angle_y_deg,
        params_.min_tilt_difference_deg);
    if (!center_ok && !tilt_x_ok && !tilt_y_ok) {
      return true;
    }
  }
  return false;
}

void IntrinsicsDataCollector::update_occupancy(
    const BoardFrameFingerprint &fp, std::vector<int> *grid) const {
  if (grid == nullptr) {
    return;
  }
  const int cells = std::max(4, params_.heatmap_cells);
  const size_t n = static_cast<size_t>(cells * cells);
  if (grid->size() != n) {
    grid->assign(n, 0);
  }
  const int cx = std::clamp(
      static_cast<int>(fp.centroid_x * cells), 0, cells - 1);
  const int cy = std::clamp(
      static_cast<int>(fp.centroid_y * cells), 0, cells - 1);
  (*grid)[static_cast<size_t>(cy * cells + cx)] += 1;
}

double IntrinsicsDataCollector::occupancy_percent(
    const std::vector<int> &grid) const {
  if (grid.empty()) {
    return 0.0;
  }
  int used = 0;
  for (int v : grid) {
    if (v > 0) {
      ++used;
    }
  }
  return 100.0 * static_cast<double>(used) / static_cast<double>(grid.size());
}

CollectorRejectReason IntrinsicsDataCollector::try_add(
    Observation obs,
    const BoardFrameFingerprint &fp,
    const ProvisionalIntrinsics &model,
    double pixel_speed,
    IntrinsicsSampleSplit *out_split) {
  if (!passes_tilt(fp)) {
    return CollectorRejectReason::Tilt;
  }
  if (!passes_speed(pixel_speed)) {
    return CollectorRejectReason::Speed;
  }
  double rms = 0.0;
  double max_err = 0.0;
  if (!passes_reprojection(obs, model, &rms, &max_err)) {
    return CollectorRejectReason::Reprojection;
  }

  const bool train_redundant =
      is_redundant_2d(fp, training_, static_cast<int>(training_.size())) ||
      is_redundant_3d(fp, training_, static_cast<int>(training_.size()));
  const bool eval_redundant =
      is_redundant_2d(
          fp, evaluation_, params_.decorrelate_eval_samples) ||
      is_redundant_3d(fp, evaluation_, params_.decorrelate_eval_samples);

  if (!train_redundant) {
    if (training_.size() >= static_cast<size_t>(params_.max_samples)) {
      return CollectorRejectReason::MaxSamples;
    }
    CollectedIntrinsicsSample sample;
    sample.observation = std::move(obs);
    sample.fingerprint = fp;
    sample.split = IntrinsicsSampleSplit::Training;
    update_occupancy(fp, &training_grid_);
    training_.push_back(std::move(sample));
    if (out_split != nullptr) {
      *out_split = IntrinsicsSampleSplit::Training;
    }
    return CollectorRejectReason::Accepted;
  }

  if (!eval_redundant) {
    if (evaluation_.size() >= static_cast<size_t>(params_.max_samples)) {
      return CollectorRejectReason::MaxSamples;
    }
    CollectedIntrinsicsSample sample;
    sample.observation = std::move(obs);
    sample.fingerprint = fp;
    sample.split = IntrinsicsSampleSplit::Evaluation;
    update_occupancy(fp, &evaluation_grid_);
    evaluation_.push_back(std::move(sample));
    if (out_split != nullptr) {
      *out_split = IntrinsicsSampleSplit::Evaluation;
    }
    return CollectorRejectReason::Accepted;
  }

  return CollectorRejectReason::Redundant;
}

bool IntrinsicsDataCollector::remove(IntrinsicsSampleSplit split, int index) {
  auto *pool = (split == IntrinsicsSampleSplit::Training) ? &training_ : &evaluation_;
  auto *grid = (split == IntrinsicsSampleSplit::Training) ? &training_grid_ : &evaluation_grid_;
  if (index < 0 || index >= static_cast<int>(pool->size())) {
    return false;
  }
  pool->erase(pool->begin() + index);
  grid->clear();
  for (const auto &s : *pool) {
    update_occupancy(s.fingerprint, grid);
  }
  return true;
}

void IntrinsicsDataCollector::clear() {
  reset();
}

const char *collector_reject_reason_text(CollectorRejectReason reason) {
  switch (reason) {
    case CollectorRejectReason::Accepted:
      return "accepted";
    case CollectorRejectReason::MaxSamples:
      return "max_samples";
    case CollectorRejectReason::Tilt:
      return "tilt";
    case CollectorRejectReason::Speed:
      return "speed";
    case CollectorRejectReason::Reprojection:
      return "reprojection";
    case CollectorRejectReason::Redundant:
      return "redundant";
    case CollectorRejectReason::PoseFailed:
      return "pose_failed";
    default:
      return "unknown";
  }
}

}  // namespace core
}  // namespace hs_calib
