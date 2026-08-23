#pragma once

#include <vector>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/targets/charuco_target.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief ChArUco 检测器（OpenCV CharucoBoard + detectMarkers / interpolate）
class CharucoDetector : public DetectorBase {
public:
  /// \brief 绑定平面 ChArUco 靶标
  explicit CharucoDetector(CharucoTarget target, CharucoDetectorParams params = {});

  /// \brief DetectorBase 接口（忽略外部 target，用成员板）
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 检测并可选写出原始 ArUco 码
  /// \param markers 非空时填充 corners/ids
  std::vector<Correspondence> detect(
      const ImageFrame &frame, DetectedMarkers *markers = nullptr) const;

private:
  CharucoTarget target_;
  CharucoDetectorParams params_;
};

}  // namespace core
}  // namespace hs_calib
