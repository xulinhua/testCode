#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 双目相对外参：固定左右内参，成对观测做 stereoCalibrate + 立体校正
class StereoExtrinsicsCalibrator : public CalibratorBase {
public:
  CalibratorInfo calibrator_info() const override;

  CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const override;
};

}  // namespace core
}  // namespace hs_calib
