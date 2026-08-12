#pragma once

#include <vector>

#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 圆点阵列检测器（findCirclesGrid）
class CircleGridDetector : public DetectorBase {
public:
  /// \brief 构造检测器并绑定圆点阵列靶标
  explicit CircleGridDetector(CircleGridTarget target);

  /// \brief 检测接口（需传入 TargetModelBase）
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 使用构造时绑定的靶标检测
  std::vector<Correspondence> detect(const ImageFrame &frame) const;

private:
  CircleGridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
