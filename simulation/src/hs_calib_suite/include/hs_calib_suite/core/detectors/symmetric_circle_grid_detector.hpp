#pragma once

#include <vector>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 对称圆点阵检测器（CALIB_CB_SYMMETRIC_GRID）
class SymmetricCircleGridDetector : public DetectorBase {
public:
  explicit SymmetricCircleGridDetector(CircleGridTarget target);

  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  std::vector<Correspondence> detect(const ImageFrame &frame) const;

private:
  CircleGridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
