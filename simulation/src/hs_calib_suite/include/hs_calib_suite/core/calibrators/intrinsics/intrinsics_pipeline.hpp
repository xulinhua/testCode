#pragma once

#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_view.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

struct IntrinsicsPipelineResult {
  bool ok = false;
  std::string message;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double rms = 0.0;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  size_t num_input_views = 0;
  size_t num_after_ransac = 0;
  size_t num_after_subsample = 0;
  size_t num_after_post = 0;
};

/// \brief Tier4 采集统计五阶段视图（Training / Pre-RANSAC / Subsampled / Post / Eval）
struct IntrinsicsPipelineStageViews {
  std::vector<IntrinsicsView> training;
  std::vector<IntrinsicsView> pre_rejection_inliers;
  std::vector<IntrinsicsView> subsampled;
  std::vector<IntrinsicsView> post_rejection_inliers;
  std::vector<IntrinsicsView> evaluation;
};

/// \brief 观测 → 内参视图列表
std::vector<IntrinsicsView> build_intrinsics_views(const ObservationBatch &batch);

/// \brief 按 Tier4 流水线划分训练/评估各阶段视图（用于采集统计图）
bool compute_intrinsics_pipeline_stage_views(
    const ObservationBatch &training_batch,
    const ObservationBatch &evaluation_batch,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &opencv_flags,
    IntrinsicsPipelineStageViews *out,
    std::string *error_out = nullptr);

/// \brief Tier4 风格流水线：RANSAC → 熵子采样 → 求解 → 后剔除
IntrinsicsPipelineResult run_intrinsics_pipeline(
    const std::vector<IntrinsicsView> &views,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &opencv_flags = {});

/// \brief Tier4 柱状图 orange：单帧 calibrateCamera 重投影 RMS（下界）
double compute_single_shot_view_rms(
    const IntrinsicsView &view,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &opencv_flags = {});

}  // namespace core
}  // namespace hs_calib
