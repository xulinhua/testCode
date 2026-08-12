#include "hs_calib_suite/core/detectors/circle_grid_detector.hpp"

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {

/// \brief 绑定圆点阵列几何
CircleGridDetector::CircleGridDetector(CircleGridTarget target)
    : target_(std::move(target)) {}

/// \brief 使用构造时靶标检测
std::vector<Correspondence> CircleGridDetector::detect(const ImageFrame &frame) const {
  return detect(frame, target_);
}

/// \brief 对称 / 非对称圆点格：灰度 → findCirclesGrid → 对应点
std::vector<Correspondence> CircleGridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }

  const cv::Mat gray = to_gray(mat);

  const cv::Size pattern(target_.circles_x(), target_.circles_y());
  // 默认带 CLUSTERING；失败后再去掉 CLUSTERING 重试
  int flags = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  if (target_.pattern() == CircleGridPattern::Asymmetric) {
    flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  }

  std::vector<cv::Point2f> centers;
  if (!cv::findCirclesGrid(gray, pattern, centers, flags) ||
      static_cast<int>(centers.size()) != pattern.area()) {
    flags &= ~cv::CALIB_CB_CLUSTERING;
    centers.clear();
    if (!cv::findCirclesGrid(gray, pattern, centers, flags) ||
        static_cast<int>(centers.size()) != pattern.area()) {
      return out;
    }
  }

  Correspondence c;
  const int n = static_cast<int>(centers.size());
  c.image_points.resize(n, 2);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = centers[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = centers[static_cast<size_t>(i)].y;
    c.ids[static_cast<size_t>(i)] = i;
  }
  c.object_points = target_.all_object_points();
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
