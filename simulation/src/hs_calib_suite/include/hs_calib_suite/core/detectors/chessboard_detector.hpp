#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 棋盘检测选项（对应 OpenCV findChessboardCorners 标志 + 亚像素窗口）
struct ChessboardDetectOptions {
  bool adaptive_thresh = true;   ///< 自适应阈值
  bool normalize_image = true;   ///< 归一化图像
  bool filter_quads = false;     ///< 过滤四边形
  bool fast_check = true;        ///< 快速预检
  int subpix_win = 11;           ///< cornerSubPix 窗口边长（像素）
  bool allow_partial = false;    ///< 允许子网格 / SB LARGER（非完整板）
  bool thorough = true;          ///< 局部模式下更慢更全；完整模式可关
  bool resized_detection = false; ///< 大图降采样检测
  int resized_max_resolution = 1000;
  int padding = 120;             ///< ROI 外扩（像素）
  cv::Rect search_roi;           ///< 空矩形 = 全图
};

/// \brief 棋盘格角点检测器（OpenCV findChessboardCorners + cornerSubPix）
class ChessboardDetector : public DetectorBase {
public:
  /// \brief 构造检测器并绑定靶标参数
  explicit ChessboardDetector(ChessboardTarget target);
  /// \brief 构造检测器并指定检测选项
  ChessboardDetector(ChessboardTarget target, ChessboardDetectOptions options);

  /// \brief 检测接口（需传入 TargetModelBase）
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 使用构造时绑定的靶标检测
  std::vector<Correspondence> detect(const ImageFrame &frame) const;

  /// \brief 更新检测选项
  void set_options(const ChessboardDetectOptions &options) { options_ = options; }

  /// \brief 上次成功检测的 ROI（全图坐标，供跟踪）
  cv::Rect last_search_roi() const { return last_search_roi_; }

private:
  int opencv_flags() const;
  cv::Rect expand_roi(const cv::Rect &roi, int pad, int w, int h) const;
  void offset_corners(std::vector<cv::Point2f> *corners, const cv::Point &origin) const;

  ChessboardTarget target_;
  ChessboardDetectOptions options_;
  mutable cv::Rect last_search_roi_;
};

}  // namespace core
}  // namespace hs_calib
