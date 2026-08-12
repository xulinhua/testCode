#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 眼在手上：求解 T（camera→gripper），即 OpenCV cam2gripper
class EyeInHandCalibrator : public CalibratorBase {
public:
  /// \brief 返回标定器元信息
  CalibratorInfo calibrator_info() const override;

  /// \brief 执行眼在手上手眼标定
  CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const override;
};

}  // namespace core
}  // namespace hs_calib
