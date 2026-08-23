#pragma once

#include <map>
#include <string>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_pipeline.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief Tier4 统计图输入（采集 + 标定结果）
struct IntrinsicsPlotInput {
  const IntrinsicsDataCollector *collector = nullptr;
  IntrinsicsProfile profile;
  IntrinsicsCollectorParams collector_params;
  IntrinsicsCalibrationExtras extras;
  std::map<std::string, std::string> solve_config;
  int image_width = 0;
  int image_height = 0;
  cv::Mat calibrated_K;
  cv::Mat calibrated_D;
  cv::Mat singleshot_K;
  cv::Mat singleshot_D;
  bool has_calibrated = false;
  bool has_singleshot = false;
  bool has_owned_batches = false;
  ObservationBatch owned_training_batch;
  ObservationBatch owned_evaluation_batch;

  ObservationBatch training_batch() const {
    if (has_owned_batches) {
      return owned_training_batch;
    }
    if (collector != nullptr) {
      return collector->training_batch();
    }
    return {};
  }

  ObservationBatch evaluation_batch() const {
    if (has_owned_batches) {
      return owned_evaluation_batch;
    }
    if (collector != nullptr) {
      return collector->evaluation_batch();
    }
    return {};
  }
};

/// \brief 导出 Tier4「Calibration data statistics」JSON（5×3 子图）
bool export_collection_statistics_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out = nullptr);

/// \brief 导出 Tier4「Calibration result statistics vs single-shot」JSON
bool export_calibration_bars_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out = nullptr);

/// \brief 导出 Tier4「Calibration result statistics」RMS 热力图 JSON
bool export_calibration_rms_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out = nullptr);

/// \brief 计算流水线阶段视图（封装 collector + profile）
bool build_plot_pipeline_stages(
    const IntrinsicsPlotInput &input,
    IntrinsicsPipelineStageViews *stages,
    std::string *error_out = nullptr);

}  // namespace core
}  // namespace hs_calib
