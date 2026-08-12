#pragma once

#include <vector>

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

private:
  int opencv_flags() const;

  ChessboardTarget target_;
  ChessboardDetectOptions options_;
};

}  // namespace core
}  // namespace hs_calib
