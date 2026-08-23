#pragma once

#include <vector>

#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/detectors/asymmetric_circle_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/symmetric_circle_grid_detector.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 兼容旧名：按 target.pattern() 分派到对称/非对称检测器
class CircleGridDetector : public DetectorBase {
public:
  explicit CircleGridDetector(CircleGridTarget target) : target_(std::move(target)) {}

  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override {
    (void)target;
    return detect(frame);
  }

  std::vector<Correspondence> detect(const ImageFrame &frame) const {
    if (target_.pattern() == CircleGridPattern::Asymmetric) {
      return AsymmetricCircleGridDetector(target_).detect(frame);
    }
    return SymmetricCircleGridDetector(target_).detect(frame);
  }

private:
  CircleGridTarget target_;
};

}  // namespace core
}  // namespace hs_calib
