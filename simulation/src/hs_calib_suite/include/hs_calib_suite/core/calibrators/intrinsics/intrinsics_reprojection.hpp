#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_view.hpp"

namespace hs_calib {
namespace core {

struct ReprojectionStats {
  double rms = 0.0;
  double max = 0.0;
};

/// \brief 由像点估计板在图像中的覆盖指纹
void fill_view_fingerprint(IntrinsicsView *view, int image_width, int image_height);

/// \brief PnP 估计板位姿
bool solve_board_pose(
    const IntrinsicsView &view,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    cv::Mat *rvec,
    cv::Mat *tvec);

/// \brief 单帧重投影误差
ReprojectionStats compute_reprojection_stats(
    const IntrinsicsView &view,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    const cv::Mat &rvec,
    const cv::Mat &tvec);

/// \brief 用当前模型更新各帧位姿与 RMS
void update_view_poses_and_errors(
    std::vector<IntrinsicsView> *views,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs);

/// \brief 粗初值相机矩阵
cv::Mat make_initial_camera_matrix(int width, int height);

/// \brief 与 profile 畸变维数一致的零初值
cv::Mat make_initial_dist_coeffs(int rational_coeffs);

}  // namespace core
}  // namespace hs_calib
