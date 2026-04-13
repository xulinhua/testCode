#ifndef COMM_ALG__MARKER_DETECT_BASE_HPP_
#define COMM_ALG__MARKER_DETECT_BASE_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>

#include "comm_alg/marker_structs.hpp"

namespace comm_alg
{

class MarkerDetectorBase
{
public:
  MarkerDetectorBase() = default;
  virtual ~MarkerDetectorBase() = default;
public:
  virtual DetectionResult detectAndProcessMarkers(
    const cv::Mat & frame,
    void * depth_frame = nullptr,
    bool draw_results = true,
    bool print_results = true) = 0;

  /**
   * @brief Set camera intrinsics
   * 
   * @param camera_matrix Camera intrinsic matrix
   * @param dist_coeffs Distortion coefficients
   */
  virtual void setCameraIntrinsics(
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs);

  /**
   * @brief setTransArucoMM
   * 
   * @param trans_aruco_mm Whether to convert ArUco world coordinates to mm
   */
  virtual void setTransMarkerMM(bool trans_marker_mm);///< 是否将marker世界坐标转换为mm

  virtual cv::Vec3f rotationMatrixToEulerAngles(const cv::Matx33f & rotation_matrix) const;

protected:
  MarkerType marker_type_ = MarkerType::Unknown;
  CameraIntrinsics camera_intrinsics_;
  bool trans_marker_mm_;
};

}  // namespace comm_alg

#endif  // COMM_ALG__MARKER_DETECT_BASE_HPP_
