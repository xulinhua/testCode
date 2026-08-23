#include "hs_calib_suite/core/detectors/asymmetric_circle_grid_detector.hpp"

#include "hs_calib_suite/core/detectors/circle_grid_detect_impl.hpp"

namespace hs_calib {
namespace core {

AsymmetricCircleGridDetector::AsymmetricCircleGridDetector(CircleGridTarget target)
    : target_(std::move(target)) {
  target_ = CircleGridTarget(
      target_.circles_x(), target_.circles_y(), target_.center_distance_m(),
      CircleGridPattern::Asymmetric, target_.circle_diameter_m());
}

std::vector<Correspondence> AsymmetricCircleGridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame);
}

std::vector<Correspondence> AsymmetricCircleGridDetector::detect(
    const ImageFrame &frame) const {
  return detect_circle_grid_impl(frame, target_);
}

}  // namespace core
}  // namespace hs_calib
