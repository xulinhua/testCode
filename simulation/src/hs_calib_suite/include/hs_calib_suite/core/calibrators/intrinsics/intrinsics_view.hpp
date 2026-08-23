#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace core {

/// \brief 单帧内参标定视图（物点/像点 + 覆盖度指纹）
struct IntrinsicsView {
  size_t source_index = 0;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  double centroid_x = 0.5;
  double centroid_y = 0.5;
  double tilt_deg = 0.0;
  double area_ratio = 0.0;
  cv::Mat rvec;
  cv::Mat tvec;
  double reproj_rms = -1.0;
  double reproj_max = -1.0;
};

}  // namespace core
}  // namespace hs_calib
