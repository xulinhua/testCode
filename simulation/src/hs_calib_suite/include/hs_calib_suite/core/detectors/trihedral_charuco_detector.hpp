#pragma once

#include <string>
#include <vector>

#include <opencv2/aruco/charuco.hpp>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/targets/trihedral_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 直角三面 ChArUco 检测器（三面 = 三张可相同的标定板）
///
/// # 问题本质
/// 人眼看「三块带白边的板」很直观；程序里却是：
/// 1. 先检出一堆 ArUco（同 ID 可在三面各出现一次）；
/// 2. 再把它们**几何聚类成最多 3 个共面子集**（分面）；
/// 3. 每子集当一张 CharucoBoard 做角点插值 + PnP。
/// 白边只帮助检测，**不参与分面标签**。分面唯一依据是「板平面单应 / 法向」。
///
/// # detect_merged 完整流程（见 .cpp 逐步注释）
/// ```
/// 图像
///   │
///   ├─① detect_markers_robust   多尺度 + CLAHE；侧脸镜像时翻转补检
///   │
///   ├─② 建 MarkerObs 池         每个码：图像中心 + 板局部中心 (obj)
///   │
///   ├─③ 逐面剥离 (≤3 次)
///   │     fit_board_in_indices  紧凑种子 → 板→图单应 H → 内点
///   │     try_build_hyp         Charuco 角点 + PnP 法向
///   │     共面 → absorb；正交 → 新面；近平行碎块 → 合并
///   │
///   ├─④ assign_faces_by_normals 把 hyp 法向贴到模型 XY/XZ/YZ
///   │
///   └─⑤ 输出 Correspondence     各面角点并入统一点集；face_ids 着色
/// ```
///
/// # 为何比「识别三张独立板」难
/// - 同 ID 三面：无法用 ID 分面，只能靠几何；
/// - 折缝附近码同时接近两面，H 残差若松会跨面吞点；
/// - Isaac 侧脸曾镜像 → 原图解不出码，分面之前就缺观测；
/// - 斜视面码少 / 透视强 → 单应与 PnP 更脆。
///
/// 约定：squares_x/y = 内角点数；CharucoBoard 方格 = n+1；DICT_4X4_250。
class TrihedralCharucoDetector : public DetectorBase {
public:
  /// \brief 绑定三面几何、码边长与字典名
  TrihedralCharucoDetector(
      TrihedralTarget target, double marker_length_m,
      const std::string &dictionary = "DICT_4X4_250");

  /// \brief DetectorBase：Thorough 合并检测
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 使用构造时靶标检测
  std::vector<Correspondence> detect(const ImageFrame &frame) const;

  /// \brief 三面角点合并；markers->face_ids 为几何分面着色
  /// \param faces_found 成功贴到模型的面数
  /// \param markers 可选：原始码 + face_ids
  /// \param fast 实时预览：少尺度；仍会做镜像侧脸补检
  Correspondence detect_merged(
      const ImageFrame &frame, int *faces_found = nullptr,
      DetectedMarkers *markers = nullptr, bool fast = false) const;

private:
  TrihedralTarget target_;
  double marker_length_m_ = 0.018;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::CharucoBoard> local_board_;
};

}  // namespace core
}  // namespace hs_calib
