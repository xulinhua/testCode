#include "comm_alg/marker_detect_base.hpp"
#include <cmath>

namespace comm_alg
{

void MarkerDetectorBase::setCameraIntrinsics(
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs)
{
  camera_intrinsics_.camera_matrix = camera_matrix;
  camera_intrinsics_.dist_coeffs = dist_coeffs;
}

void MarkerDetectorBase::setTransMarkerMM(bool trans_marker_mm)
{
  trans_marker_mm_ = trans_marker_mm;
}

cv::Vec3f MarkerDetectorBase::rotationMatrixToEulerAngles(const cv::Matx33f & rotation_matrix) const
{
  float sy = sqrt(rotation_matrix(0, 0) * rotation_matrix(0, 0) + 
                  rotation_matrix(1, 0) * rotation_matrix(1, 0));
  bool singular = sy < 1e-6f;
  
  float rx, ry, rz;
  if (!singular) {
    rx = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    ry = atan2(-rotation_matrix(2, 0), sy);
    rz = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  } else {
    rx = atan2(-rotation_matrix(1, 2), rotation_matrix(1, 1));
    ry = atan2(-rotation_matrix(2, 0), sy);
    rz = 0.0f;
  }
  
  return cv::Vec3f(rx * 180.0f / CV_PI, ry * 180.0f / CV_PI, rz * 180.0f / CV_PI);
}

}  // namespace comm_alg
