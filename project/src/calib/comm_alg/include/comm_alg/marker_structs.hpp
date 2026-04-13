#ifndef COMM_ALG__MARKER_STRUCTS_HPP_
#define COMM_ALG__MARKER_STRUCTS_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace comm_alg
{

  enum class MarkerType
  {
    Unknown = -1,
    Aruco,
    Chessboard
  };

struct MarkerInfo
{
  cv::Point2f center_2d;
  cv::Point3f position;
  cv::Vec3f rotation;  // Euler angles in degrees
  cv::Matx33f rotation_matrix;
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  std::vector<cv::Point2f> corners; // 角点
  std::vector<cv::Point3f> world_corners; // 世界坐标系下角点
  float distance;
  int marker_id = -1; // 标记ID，对于棋盘格可以设为-1
};

struct CameraIntrinsics
{
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

struct DetectionResult
{
  std::vector<MarkerInfo> markers_info;
  bool found;
  cv::Mat processed_frame;
  MarkerType marker_type = MarkerType::Unknown;
  
  // Default constructor
  DetectionResult() : found(false) {}
  
  // Copy constructor with deep copy for processed_frame
  DetectionResult(const DetectionResult& other)
    : markers_info(other.markers_info)
    , found(other.found)
  {
    if (!other.processed_frame.empty()) {
      other.processed_frame.copyTo(processed_frame);
    }
  }
  
  // Assignment operator with deep copy for processed_frame
  DetectionResult& operator=(const DetectionResult& other) {
    if (this != &other) {
      markers_info = other.markers_info;
      found = other.found;
      if (!other.processed_frame.empty()) {
        other.processed_frame.copyTo(processed_frame);
      } else {
        processed_frame.release();
      }
    }
    return *this;
  }
};

}  // namespace comm_alg

#endif  // COMM_ALG__MARKER_STRUCTS_HPP_
