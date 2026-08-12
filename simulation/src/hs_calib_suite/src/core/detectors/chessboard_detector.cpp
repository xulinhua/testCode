#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"

#include <algorithm>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {

/// \brief 仅用靶标构造，检测选项取默认值
ChessboardDetector::ChessboardDetector(ChessboardTarget target)
    : target_(std::move(target)) {}

/// \brief 靶标 + OpenCV 标志 / 亚像素窗口
ChessboardDetector::ChessboardDetector(
    ChessboardTarget target, ChessboardDetectOptions options)
    : target_(std::move(target)), options_(options) {}

/// \brief 将选项折叠为 findChessboardCorners 的 flags 位掩码
int ChessboardDetector::opencv_flags() const {
  int flags = 0;
  if (options_.adaptive_thresh) {
    flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
  }
  if (options_.normalize_image) {
    flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
  }
  if (options_.filter_quads) {
    flags |= cv::CALIB_CB_FILTER_QUADS;
  }
  if (options_.fast_check) {
    flags |= cv::CALIB_CB_FAST_CHECK;
  }
  return flags;
}

/// \brief 使用构造时绑定的棋盘几何检测
std::vector<Correspondence> ChessboardDetector::detect(const ImageFrame &frame) const {
  return detect(frame, target_);
}

/// \brief 平面棋盘：灰度 → findChessboardCorners → 亚像素 → 对应点
std::vector<Correspondence> ChessboardDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }

  // 与解码无关：统一灰度
  const cv::Mat gray = to_gray(mat);

  // 内角点网格尺寸（与靶标 squares_x/y 一致）
  const cv::Size pattern(target_.squares_x(), target_.squares_y());
  std::vector<cv::Point2f> corners;
  const int flags = opencv_flags();
  bool ok = cv::findChessboardCorners(gray, pattern, corners, flags);
  if (!ok) {
    // 去掉 FAST_CHECK 再试（多姿态 / 弱对比时常见回退）
    corners.clear();
    int retry = flags & ~cv::CALIB_CB_FAST_CHECK;
    if (retry == 0) {
      retry = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    }
    ok = cv::findChessboardCorners(gray, pattern, corners, retry);
    if (!ok || corners.empty()) {
      return out;
    }
  } else if (corners.empty()) {
    return out;
  }

  // 亚像素精化（窗口来自选项）
  refine_corners_subpix(gray, &corners, options_.subpix_win);

  Correspondence c;
  const int n = static_cast<int>(corners.size());
  c.image_points.resize(n, 2);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = corners[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = corners[static_cast<size_t>(i)].y;
    c.ids[static_cast<size_t>(i)] = i;
  }
  c.object_points = target_.all_object_points();
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
