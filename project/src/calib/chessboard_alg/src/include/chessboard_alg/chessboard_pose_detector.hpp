#ifndef CHESSBOARD_ALG__CHESSBOARD_POSE_DETECTOR_HPP_
#define CHESSBOARD_ALG__CHESSBOARD_POSE_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>

// 包含共用数据结构和基类
#include "comm_alg/marker_detect_base.hpp"

namespace chessboard_alg
{

// 使用共用的数据结构
typedef comm_alg::MarkerInfo MarkerInfo;
typedef comm_alg::CameraIntrinsics CameraIntrinsics;
typedef comm_alg::DetectionResult DetectionResult;

class ChessboardPoseDetector : public comm_alg::MarkerDetectorBase
{
public:
  /**
   * @brief Constructor for ChessboardPoseDetector
   * 
   * @param board_size Size of the chessboard (number of inner corners in width and height)
   * @param square_size Size of each square in meters
   */
  explicit ChessboardPoseDetector(
    cv::Size board_size = cv::Size(8, 6),
    float square_size = 0.025);

  /**
   * @brief Destructor
   */
  ~ChessboardPoseDetector() = default;

  /**
   * @brief Detect chessboard corners in the given frame
   * 
   * @param frame Input image frame
   * @return bool True if chessboard is detected, false otherwise
   */
  bool detectChessboard(const cv::Mat& frame);

  /**
   * @brief Estimate pose of the chessboard and return the result
   * 
   * @param frame Input image frame
   * @param marker_info Output pose information of the chessboard
   * @return bool True if pose is estimated, false otherwise  
   */
  bool estimatePose(const cv::Mat& frame, MarkerInfo& marker_info);

  /**
   * @brief Get the 2D center point of the detected chessboard
   * 
   * @param corners Detected chessboard corners
   * @return cv::Point2f Center point in pixel coordinates
   */
  cv::Point2f getChessboardCenterPixel(
    const std::vector<cv::Point2f> & corners) const;

  /**
   * @brief Calculate the world coordinates of the chessboard corners
   * 
   * @param square_size Size of each square in meters
   * @param rotation_matrix Rotation matrix of the chessboard
   * @param tvec Translation vector of the chessboard
   * @return std::vector<cv::Point3f> World coordinates of the chessboard corners
   */
  std::vector<cv::Point3f> calculateCornerWorldCoordinates(
    float square_size, const cv::Mat& rotation_matrix, const cv::Vec3d& tvec) const;

  /**
   * @brief Draw detection results on the frame
   * 
   * @param frame Input image frame
   * @param pose_info Chessboard pose information
   * @param corners Detected chessboard corners
   * @param draw_axes Whether to draw coordinate axes
   * @return cv::Mat Frame with drawn results
   */
  cv::Mat drawChessboardResults(
    const cv::Mat & frame,
    const MarkerInfo & pose_info,
    const std::vector<cv::Point2f> & corners,
    bool draw_axes = true) const;

  /**
   * @brief Print chessboard pose information to console
   * 
   * @param pose_info Chessboard pose information
   */
  void printChessboardResults(const MarkerInfo & pose_info) const;

  /**
   * @brief Process chessboard detection and pose estimation (complete pipeline)
   * 
   * @param frame Input image frame
   * @param draw_results Whether to draw results on frame (default: true)
   * @param print_results Whether to print results to console (default: true)
   * @return MarkerInfo Complete detection result
   */
  bool detectAndEstimatePose(
    const cv::Mat & frame,
    MarkerInfo& result,
    bool print_results = true);

  /**
   * @brief Detect and process ArUco markers (complete pipeline)
   * 
   * @param frame Input image frame
   * @param draw_results Whether to draw results on frame (default: true)
   * @param print_results Whether to print results to console (default: true)
   * @return DetectionResult Complete detection result
   */
  DetectionResult detectAndProcessMarkers(
    const cv::Mat & frame,
    void * depth_frame = nullptr,
    bool draw_results = true,
    bool print_results = true) override;

  // Setter methods for configuration
  void setBoardSize(cv::Size board_size);
  void setSquareSize(float square_size);

  // Getter methods
  cv::Size getBoardSize() const;
  float getSquareSize() const;

public:
  /**
   * @brief 标定相机内参
   * @param images 输入棋盘格图像集
   * @return true-标定成功，false-失败
   */
  bool calibrateCameraIntrinsics(const std::vector<cv::Mat>& images, 
    cv::Mat & camera_matrix,
    cv::Mat & dist_coeffs);

private:
  cv::Size board_size_;
  float square_size_;
  cv::TermCriteria criteria_;
};

}  // namespace chessboard_alg

#endif  // CHESSBOARD_ALG__CHESSBOARD_POSE_DETECTOR_HPP_