#pragma once

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief ImageFrame ↔ cv::Mat 视图桥接（不拷贝像素；调用方保证缓冲存活）
///
/// 与 \ref cv_image_ops.hpp 分工：本文件只做「帧包装」，不做颜色/增强。

/// \brief 将连续内存 ImageFrame 包装为 cv::Mat（不拷贝）
/// \note encoding 为 mono8 或 channels==1 → CV_8UC1；否则按 BGR 视为 CV_8UC3
inline cv::Mat image_frame_as_mat(const ImageFrame &frame) {
  if (frame.data == nullptr || frame.width <= 0 || frame.height <= 0) {
    return {};
  }
  const bool mono = (frame.encoding == "mono8" || frame.channels == 1);
  const int type = mono ? CV_8UC1 : CV_8UC3;
  const int elem = mono ? 1 : 3;
  const size_t step =
      frame.step > 0 ? frame.step
                     : static_cast<size_t>(frame.width) * static_cast<size_t>(elem);
  return cv::Mat(
      frame.height, frame.width, type, const_cast<uint8_t *>(frame.data), step);
}

/// \brief 从拥有数据的 cv::Mat 填充 ImageFrame 视图（不拥有；mat 须保活）
inline ImageFrame mat_as_image_frame(
    const cv::Mat &mat, const std::string &encoding = "bgr8") {
  ImageFrame f;
  f.width = mat.cols;
  f.height = mat.rows;
  f.channels = mat.channels();
  f.step = mat.step[0];
  f.data = mat.data;
  f.encoding = encoding;
  return f;
}

}  // namespace core
}  // namespace hs_calib
