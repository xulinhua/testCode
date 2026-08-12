#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 标定器抽象基类（算法层，无 ROS 依赖）
/// GUI / ROS 节点只依赖本基类；具体求解器在独立 Calibrator 中实现。
class CalibratorBase {
public:
  virtual ~CalibratorBase() = default;

  /// \brief 返回标定器元信息
  virtual CalibratorInfo calibrator_info() const = 0;

  /// \brief 执行标定求解
  /// \param observations 观测批
  /// \param config 键值配置（通常来自 YAML）
  /// \return 标定结果（变换 / 内参元数据 / 指标）
  virtual CalibrationResult calibrate(
      const ObservationBatch &observations,
      const std::map<std::string, std::string> &config) const = 0;
};

}  // namespace core
}  // namespace hs_calib
