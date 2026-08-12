#pragma once

#include <vector>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/targets/trihedral_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 三面检测预算：实时预览用 Fast，避免卡死；手动检测用 Thorough
enum class TrihedralChessDetectSpeed {
  Fast,     ///< 限时、降分辨率；仍会做掩膜再挖，尽量出 ≥2 面
  Thorough  ///< 更完整 ROI / 子网格 / SB 部分角点
};

/// \brief 直角三面**棋盘格**检测器（只认角点，无 ArUco）
///
/// # 设计要点（与单板标定的差别）
/// - **每面网格允许不完整**：斜视/遮挡时 4×4、5×5 等子网格即可贴面参与求解；
///   三面几何约束弥补单面信息不足——这正是三面靶相对「拍全一张平面板」的意义。
/// - OpenCV 一次通常只吐一个网格 → 找一面 → 掩膜 → 再找，最多三面；
///   再枚举摆放到模型 XY/XZ/YZ（PnP 残差）。
/// - 白边帮助检测，**不作分面标签**。
///
/// 约定：squares_x/y = 每面内角点数（正方形面取 max）。
class TrihedralChessDetector : public DetectorBase {
public:
  explicit TrihedralChessDetector(TrihedralTarget target);

  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 使用构造时靶标参数检测
  std::vector<Correspondence> detect(const ImageFrame &frame) const;

  /// \brief 合并为单条对应（各面角点拼在一起，供单帧/少帧求解）
  /// \param faces_found 若非空，写出成功贴到模型上的面数
  Correspondence detect_merged(
      const ImageFrame &frame, int *faces_found = nullptr,
      TrihedralChessDetectSpeed speed = TrihedralChessDetectSpeed::Thorough) const;

private:
  TrihedralTarget target_;
};

}  // namespace core
}  // namespace hs_calib
