#pragma once

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 直角三面单帧 / 多帧内参标定器
///
/// - 单帧：≥2 面非共面点；固定主点 / fy=fx / 无畸变，一维搜索焦距 + PnP（不用 calibrateCamera）
/// - 多帧：联合 `calibrateCamera`；每帧至少 6 个有效角点（OpenCV DLT）
class TrihedralOneshotCalibrator : public CalibratorBase {
public:
  /// \brief 返回标定器元信息
  CalibratorInfo calibrator_info() const override;

  /// \brief 执行三面靶单帧或多帧内参标定
  CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const override;
};

}  // namespace core
}  // namespace hs_calib
