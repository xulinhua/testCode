#pragma once

#include <vector>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 非对称圆点阵检测器（CALIB_CB_ASYMMETRIC_GRID）
class AsymmetricCircleGridDetector : public DetectorBase {
public:
  explicit AsymmetricCircleGridDetector(CircleGridTarget target);

  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  std::vector<Correspondence> detect(const ImageFrame &frame) const;

private:
  CircleGridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
