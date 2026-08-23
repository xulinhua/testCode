#pragma once

#include <vector>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 对称/非对称圆点阵共用检测实现
std::vector<Correspondence> detect_circle_grid_impl(
    const ImageFrame &frame, const CircleGridTarget &target,
    const DotDetectorParams &dot_params = DotDetectorParams{});

}  // namespace core
}  // namespace hs_calib
