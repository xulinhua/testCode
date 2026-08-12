#pragma once

#include <vector>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/targets/aruco_grid_target.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief ArUco / AprilTag 阵列检测器（detectMarkers + GridBoard 物点匹配）
class ArucoGridDetector : public DetectorBase {
public:
  /// \brief 构造检测器并绑定 ArUco 阵列靶标
  explicit ArucoGridDetector(ArucoGridTarget target);

  /// \brief 检测接口（需传入 TargetModelBase）
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 检测并可选输出原始标记检测结果
  std::vector<Correspondence> detect(
      const ImageFrame &frame, DetectedMarkers *markers = nullptr) const;

private:
  ArucoGridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
