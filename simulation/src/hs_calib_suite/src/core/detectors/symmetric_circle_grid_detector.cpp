#include "hs_calib_suite/core/detectors/symmetric_circle_grid_detector.hpp"

#include "hs_calib_suite/core/detectors/circle_grid_detect_impl.hpp"

namespace hs_calib {
namespace core {

SymmetricCircleGridDetector::SymmetricCircleGridDetector(CircleGridTarget target)
    : target_(std::move(target)) {
  // 强制对称几何
  target_ = CircleGridTarget(
      target_.circles_x(), target_.circles_y(), target_.center_distance_m(),
      CircleGridPattern::Symmetric, target_.circle_diameter_m());
}

std::vector<Correspondence> SymmetricCircleGridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame);
}

std::vector<Correspondence> SymmetricCircleGridDetector::detect(
    const ImageFrame &frame) const {
  return detect_circle_grid_impl(frame, target_);
}

}  // namespace core
}  // namespace hs_calib
