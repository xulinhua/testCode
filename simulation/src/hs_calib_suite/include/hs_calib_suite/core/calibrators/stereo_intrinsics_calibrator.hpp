#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 双目各自内参：左右目观测分别做多姿态内参标定
class StereoIntrinsicsCalibrator : public CalibratorBase {
public:
  CalibratorInfo calibrator_info() const override;

  CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const override;
};

}  // namespace core
}  // namespace hs_calib
