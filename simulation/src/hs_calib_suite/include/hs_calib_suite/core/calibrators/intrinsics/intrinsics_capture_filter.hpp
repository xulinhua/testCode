#pragma once

#include <map>
#include <string>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 采集阶段临时内参（随已采帧更新）
struct ProvisionalIntrinsics {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  bool valid = false;
};

/// \brief 用已采观测刷新临时模型（帧数不足时保持无效）
void update_provisional_intrinsics(
    const ObservationBatch &batch,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    ProvisionalIntrinsics *out,
    int max_fast_samples = 20);

/// \brief 采集时重投影过滤（Tier4 data_collector）
bool capture_passes_reprojection_filter(
    const Correspondence &corr,
    int image_width,
    int image_height,
    const ProvisionalIntrinsics &model,
    const IntrinsicsProfile &profile,
    double *rms_out = nullptr,
    double *max_out = nullptr);

}  // namespace core
}  // namespace hs_calib
