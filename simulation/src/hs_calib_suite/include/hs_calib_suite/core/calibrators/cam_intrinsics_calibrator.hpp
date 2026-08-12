#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 单目相机内参标定（棋盘等多姿态观测 → calibrateCamera）
class CamIntrinsicsCalibrator : public CalibratorBase {
public:
  /// \brief 返回标定器元信息
  CalibratorInfo calibrator_info() const override;

  /// \brief 执行多帧相机内参标定
  CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const override;
};

}  // namespace core
}  // namespace hs_calib
