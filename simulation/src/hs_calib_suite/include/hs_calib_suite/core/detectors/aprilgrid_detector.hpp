#pragma once

#include <string>
#include <vector>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/targets/aprilgrid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief Kalibr 风格 Aprilgrid 检测（官方 AprilTag 库单次检测 + cornerSubPix）
class AprilgridDetector : public DetectorBase {
public:
  /// \param dictionary_name 映射到 AprilTag family，默认 DICT_APRILTAG_36h11
  explicit AprilgridDetector(
      AprilgridTarget target,
      std::string dictionary_name = "DICT_APRILTAG_36h11",
      AprilgridDetectorParams params = {});

  const AprilgridDetectorParams &params() const { return params_; }

  /// \brief DetectorBase 入口
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 检测并可选输出原始 Tag 结果
  std::vector<Correspondence> detect(
      const ImageFrame &frame, DetectedMarkers *markers = nullptr) const;

private:
  AprilgridTarget target_;
  std::string dictionary_name_;
  AprilgridDetectorParams params_;
};

}  // namespace core
}  // namespace hs_calib
