#pragma once

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 棋盘对应点 + 内参 → T_cam_target（成功返回 true）
bool estimate_board_pose(
    const Correspondence &corr,
    const cv::Mat &K,
    const cv::Mat &D,
    Eigen::Matrix4d *T_cam_target,
    std::string *error_out = nullptr);

}  // namespace core
}  // namespace hs_calib
