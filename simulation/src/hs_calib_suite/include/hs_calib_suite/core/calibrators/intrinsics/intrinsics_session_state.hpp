#pragma once

#include <chrono>
#include <map>
#include <string>

#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

enum class IntrinsicsOperationStatus {
  Idle,
  Calibrating,
  Evaluating,
};

/// \brief 标定/评估统计（Tier4 Calibration control §4.2）
struct IntrinsicsCalibrationStats {
  int training_samples = 0;
  int evaluation_samples = 0;
  int pre_rejection_inliers = 0;
  int post_rejection_inliers = 0;
  int eval_post_rejection_inliers = 0;
  double training_rms_all = -1.0;
  double training_rms_inlier = -1.0;
  double evaluation_rms_all = -1.0;
  double evaluation_rms_inlier = -1.0;
  double calibration_time_sec = 0.0;
};

/// \brief 内参会话编排：双库、partial 模型、指标（core 层，无 Qt）
class IntrinsicsSessionState {
public:
  void configure(
      const IntrinsicsProfile &profile,
      const IntrinsicsCollectorParams &collector_params,
      const std::map<std::string, std::string> &solve_config);

  void set_offline_source(bool offline);
  void reset();

  IntrinsicsDataCollector &collector() { return collector_; }
  const IntrinsicsDataCollector &collector() const { return collector_; }

  const BoardFrameMetrics &last_metrics() const { return last_metrics_; }
  IntrinsicsOperationStatus status() const { return status_; }
  const IntrinsicsCalibrationStats &stats() const { return stats_; }
  const ProvisionalIntrinsics &provisional_model() const { return provisional_; }
  bool has_singleshot_model() const { return has_singleshot_model_; }
  const ProvisionalIntrinsics &singleshot_model() const { return singleshot_; }
  const CalibrationResult &last_result() const { return last_result_; }
  bool has_calibrated_model() const { return has_calibrated_model_; }
  const cv::Mat &calibrated_K() const { return calibrated_K_; }
  const cv::Mat &calibrated_D() const { return calibrated_D_; }

  void refresh_provisional(int image_width, int image_height);
  void set_provisional_model(const ProvisionalIntrinsics &model);
  void update_frame_metrics(
      const Correspondence &corr,
      int image_width,
      int image_height,
      double cell_size_m);

  CollectorRejectReason try_capture(
      Observation obs,
      const BoardFrameFingerprint &fp,
      double pixel_speed,
      IntrinsicsSampleSplit *out_split = nullptr);

  bool calibrate(std::string *error_out = nullptr);
  bool evaluate(std::string *error_out = nullptr);

  ObservationBatch training_batch() const;
  ObservationBatch evaluation_batch() const;

private:
  void sync_calibrated_from_result();
  cv::Mat model_K_for_metrics() const;
  cv::Mat model_D_for_metrics() const;

  IntrinsicsProfile profile_;
  IntrinsicsCollectorParams collector_params_;
  std::map<std::string, std::string> solve_config_;
  IntrinsicsDataCollector collector_;
  ProvisionalIntrinsics provisional_;
  ProvisionalIntrinsics singleshot_;
  bool has_singleshot_model_ = false;
  BoardFrameMetrics last_metrics_;
  IntrinsicsOperationStatus status_ = IntrinsicsOperationStatus::Idle;
  IntrinsicsCalibrationStats stats_;
  CalibrationResult last_result_;
  bool has_calibrated_model_ = false;
  cv::Mat calibrated_K_;
  cv::Mat calibrated_D_;
  int image_width_ = 0;
  int image_height_ = 0;
  BoardFrameFingerprint last_capture_fp_{};
  bool has_last_capture_fp_ = false;
  cv::Point2f last_centroid_px_{0.5f, 0.5f};
  bool has_last_centroid_ = false;
};

}  // namespace core
}  // namespace hs_calib
