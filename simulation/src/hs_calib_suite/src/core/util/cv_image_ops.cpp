#include "hs_calib_suite/core/util/cv_image_ops.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace hs_calib {
namespace core {

/// \brief 转为单通道灰度（已是灰度则返回视图）
cv::Mat to_gray(const cv::Mat &bgr_or_gray) {
  // 空图直接返回，避免下游误用
  if (bgr_or_gray.empty()) {
    return {};
  }
  // 已是单通道：不拷贝数据，只建视图
  if (bgr_or_gray.channels() == 1) {
    return bgr_or_gray;
  }
  cv::Mat gray;
  // 4 通道按 BGRA；其余按 BGR
  const int code =
      (bgr_or_gray.channels() == 4) ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY;
  cv::cvtColor(bgr_or_gray, gray, code);
  return gray;
}

/// \brief 保证输出为 BGR 三通道
cv::Mat to_bgr(const cv::Mat &src) {
  if (src.empty()) {
    return {};
  }
  if (src.channels() == 3) {
    return src;
  }
  cv::Mat bgr;
  if (src.channels() == 1) {
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
  } else if (src.channels() == 4) {
    cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
  } else {
    return {};
  }
  return bgr;
}

/// \brief CLAHE 对比度增强（输入须为灰度）
cv::Mat enhance_clahe(
    const cv::Mat &gray, double clip_limit, cv::Size tile_grid) {
  if (gray.empty() || gray.channels() != 1) {
    return {};
  }
  cv::Mat out;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip_limit, tile_grid);
  clahe->apply(gray, out);
  return out;
}

/// \brief 由宽高与水平视场角估计针孔内参 K
cv::Mat guess_K(int width, int height, double hfov_deg) {
  if (width <= 0 || height <= 0) {
    return {};
  }
  // fx = (W/2) / tan(HFOV/2)；主点取图像中心，假设方像素
  const double fx =
      0.5 * static_cast<double>(width) /
      std::tan(hfov_deg * 0.5 * CV_PI / 180.0);
  return (cv::Mat_<double>(3, 3) << fx, 0.0, width * 0.5, 0.0, fx, height * 0.5, 0.0,
          0.0, 1.0);
}

/// \brief 由图像尺寸估计针孔内参 K
cv::Mat guess_K(cv::Size size, double hfov_deg) {
  return guess_K(size.width, size.height, hfov_deg);
}

/// \brief cornerSubPix 亚像素细化
void refine_corners_subpix(
    const cv::Mat &gray, std::vector<cv::Point2f> *corners, int win) {
  if (gray.empty() || corners == nullptr || corners->empty()) {
    return;
  }
  // OpenCV 要求奇数窗口
  const int odd = std::max(3, win | 1);
  cv::cornerSubPix(
      gray, *corners, cv::Size(odd, odd), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.01));
}

}  // namespace core
}  // namespace hs_calib
