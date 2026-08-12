#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace core {

/// \brief 与「解码 / 靶标几何」无关的 OpenCV 图像小工具
///
/// 灰度转换、对比度增强、内参初值、亚像素细化等可在各检测器间复用；
/// 请勿把 ArUco / ChArUco / 棋盘专用逻辑放进本头文件。

/// \brief 转为单通道灰度（已是灰度则返回原图浅拷贝视图）
/// \param bgr_or_gray 1 / 3 / 4 通道 8-bit 图
/// \return CV_8UC1；空输入返回空 Mat
cv::Mat to_gray(const cv::Mat &bgr_or_gray);

/// \brief 保证输出为 BGR 三通道（灰度 / BGRA → BGR）
cv::Mat to_bgr(const cv::Mat &src);

/// \brief CLAHE 对比度增强（输入须为灰度）
/// \param gray 单通道图
/// \param clip_limit CLAHE clipLimit
/// \param tile_grid CLAHE 分块
cv::Mat enhance_clahe(
    const cv::Mat &gray, double clip_limit = 3.0,
    cv::Size tile_grid = cv::Size(8, 8));

/// \brief 由图像尺寸与水平视场角估计针孔内参 K（主点在图像中心，fx=fy）
/// \param width 图像宽（像素）
/// \param height 图像高（像素）
/// \param hfov_deg 水平视场角（度），仿真相机常用 ~70°
cv::Mat guess_K(int width, int height, double hfov_deg = 70.0);

/// \brief \see guess_K(int,int,double)
cv::Mat guess_K(cv::Size size, double hfov_deg = 70.0);

/// \brief cornerSubPix 亚像素细化
/// \param gray 灰度图
/// \param corners 角点（就地更新）
/// \param win 窗口边长（奇数；偶数会自动 +1）
void refine_corners_subpix(
    const cv::Mat &gray, std::vector<cv::Point2f> *corners, int win = 11);

}  // namespace core
}  // namespace hs_calib
