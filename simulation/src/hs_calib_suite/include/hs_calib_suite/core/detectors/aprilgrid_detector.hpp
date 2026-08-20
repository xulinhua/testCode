#pragma once

#include <vector>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/targets/aprilgrid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief Kalibr Aprilgrid 检测器（AprilTag 36h11 + Kalibr 角点索引）
class AprilgridDetector : public DetectorBase {
public:
  /// \brief 构造并绑定 Aprilgrid 几何
  explicit AprilgridDetector(AprilgridTarget target);

  /// \brief DetectorBase 入口
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 检测并可选输出原始 Tag 结果
  std::vector<Correspondence> detect(
      const ImageFrame &frame, DetectedMarkers *markers = nullptr) const;

private:
  AprilgridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
