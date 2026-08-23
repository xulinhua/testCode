#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_collector_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_capture_filter.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

enum class IntrinsicsSampleSplit { Training, Evaluation };

enum class CollectorRejectReason {
  Accepted,
  MaxSamples,
  Tilt,
  Speed,
  Reprojection,
  Redundant,
  PoseFailed,
};

/// \brief 单条已采样本（含指纹）
struct CollectedIntrinsicsSample {
  Observation observation;
  BoardFrameFingerprint fingerprint;
  IntrinsicsSampleSplit split = IntrinsicsSampleSplit::Training;
};

/// \brief Tier4 训练/评估双库与采集过滤（§10）
class IntrinsicsDataCollector {
public:
  void set_params(const IntrinsicsCollectorParams &params) { params_ = params; }
  const IntrinsicsCollectorParams &params() const { return params_; }

  void set_profile(const IntrinsicsProfile &profile) { profile_ = profile; }
  const IntrinsicsProfile &profile() const { return profile_; }

  void set_offline_source(bool offline) { offline_source_ = offline; }
  void reset();

  const std::vector<CollectedIntrinsicsSample> &training() const { return training_; }
  const std::vector<CollectedIntrinsicsSample> &evaluation() const { return evaluation_; }

  int training_count() const { return static_cast<int>(training_.size()); }
  int evaluation_count() const { return static_cast<int>(evaluation_.size()); }

  double training_occupancy_percent() const;
  double evaluation_occupancy_percent() const;
  const std::vector<int> &training_grid() const { return training_grid_; }
  const std::vector<int> &evaluation_grid() const { return evaluation_grid_; }
  int heatmap_cell_count() const { return params_.heatmap_cells; }

  ObservationBatch training_batch() const;
  ObservationBatch evaluation_batch() const;
  ObservationBatch all_batch() const;

  /// \brief 尝试入库；成功时写入 out_split
  CollectorRejectReason try_add(
      Observation obs,
      const BoardFrameFingerprint &fp,
      const ProvisionalIntrinsics &model,
      double pixel_speed,
      IntrinsicsSampleSplit *out_split = nullptr);

  /// \brief 删除训练/评估集中指定索引
  bool remove(IntrinsicsSampleSplit split, int index);
  void clear();

private:
  bool passes_tilt(const BoardFrameFingerprint &fp) const;
  bool passes_speed(double pixel_speed) const;
  bool passes_reprojection(
      const Observation &obs,
      const ProvisionalIntrinsics &model,
      double *rms_out,
      double *max_out) const;
  bool is_redundant_2d(
      const BoardFrameFingerprint &fp,
      const std::vector<CollectedIntrinsicsSample> &pool,
      int compare_last_n) const;
  bool is_redundant_3d(
      const BoardFrameFingerprint &fp,
      const std::vector<CollectedIntrinsicsSample> &pool,
      int compare_last_n) const;
  void update_occupancy(
      const BoardFrameFingerprint &fp, std::vector<int> *grid) const;
  double occupancy_percent(const std::vector<int> &grid) const;

  IntrinsicsCollectorParams params_;
  IntrinsicsProfile profile_;
  bool offline_source_ = false;
  std::vector<CollectedIntrinsicsSample> training_;
  std::vector<CollectedIntrinsicsSample> evaluation_;
  std::vector<int> training_grid_;
  std::vector<int> evaluation_grid_;
  BoardFrameFingerprint last_fp_{};
  bool has_last_fp_ = false;
};

const char *collector_reject_reason_text(CollectorRejectReason reason);

}  // namespace core
}  // namespace hs_calib
