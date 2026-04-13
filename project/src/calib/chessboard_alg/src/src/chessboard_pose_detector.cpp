#include "../include/chessboard_alg/chessboard_pose_detector.hpp"
#include <iostream>
#include <cmath>
#include <opencv2/calib3d.hpp>

namespace chessboard_alg
{

ChessboardPoseDetector::ChessboardPoseDetector(
  cv::Size board_size,
  float square_size)
: board_size_(board_size),
  square_size_(square_size),
  criteria_(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001)
{
  marker_type_ = comm_alg::MarkerType::Chessboard;
}

bool ChessboardPoseDetector::detectChessboard(const cv::Mat& frame)
{
  if (frame.empty()) {
    std::cerr << "检测失败：输入图像为空" << std::endl;
    return false;
  }

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(gray, board_size_, corners,
                                        cv::CALIB_CB_ADAPTIVE_THRESH | 
                                        cv::CALIB_CB_NORMALIZE_IMAGE |
                                        cv::CALIB_CB_FAST_CHECK);

  return found;
}

bool ChessboardPoseDetector::estimatePose(const cv::Mat& frame, MarkerInfo& marker_info)
{
  MarkerInfo info;
  bool found = false;
  info.distance = -1.0f;

  if (frame.empty()) {
    std::cerr << "估计位姿失败：输入图像为空" << std::endl;
    return found;
  }

  if (camera_intrinsics_.camera_matrix.empty() || camera_intrinsics_.dist_coeffs.empty()) {
    std::cerr << "相机内参未设置" << std::endl;
    return found;
  }

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;
  bool foundCorners = cv::findChessboardCorners(gray, board_size_, corners,
                                        cv::CALIB_CB_ADAPTIVE_THRESH | 
                                        cv::CALIB_CB_NORMALIZE_IMAGE);

  if (foundCorners) {
    // 提高角点检测精度
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria_);
    
    // 准备世界坐标系中的3D点
    std::vector<cv::Point3f> obj_points;
    for (int i = 0; i < board_size_.height; ++i) {
      for (int j = 0; j < board_size_.width; ++j) {
        obj_points.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
      }
    }

    // 估计位姿
    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(obj_points, corners, 
                                camera_intrinsics_.camera_matrix, 
                                camera_intrinsics_.dist_coeffs, 
                                rvec, tvec);

    if (success) {
      // 计算旋转矩阵
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);

      // 计算距离
      float distance = static_cast<float>(cv::norm(tvec));

      // 转换为欧拉角
      double sy = sqrt(rotation_matrix.at<double>(0,0) * rotation_matrix.at<double>(0,0) + 
                       rotation_matrix.at<double>(1,0) * rotation_matrix.at<double>(1,0));

      bool singular = sy < 1e-6;
      double x, y, z;
      if (!singular) {
        x = atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
        y = atan2(-rotation_matrix.at<double>(2,0), sy);
        z = atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));
      } else {
        x = atan2(-rotation_matrix.at<double>(1,2), rotation_matrix.at<double>(1,1));
        y = atan2(-rotation_matrix.at<double>(2,0), sy);
        z = 0;
      }

      cv::Vec3f euler_angles(
        static_cast<float>(x * 180.0 / CV_PI),
        static_cast<float>(y * 180.0 / CV_PI),
        static_cast<float>(z * 180.0 / CV_PI)
      );

      // 获取中心点
      cv::Point2f center_2d = getChessboardCenterPixel(corners);

      // 计算角点的世界坐标
      std::vector<cv::Point3f> corner_world_coords = calculateCornerWorldCoordinates(
        square_size_, rotation_matrix, tvec);

      // 填充信息
      info.center_2d = center_2d;
      info.position = cv::Point3f(static_cast<float>(tvec[0]), 
                                  static_cast<float>(tvec[1]), 
                                  static_cast<float>(tvec[2]));
      info.rotation = euler_angles;
      info.rotation_matrix = cv::Matx33f(rotation_matrix);
      info.rvec = rvec;
      info.tvec = tvec;
      info.corners = corners;
      info.world_corners = corner_world_coords;
      info.distance = distance;
      marker_info = info;
      found = true;
    }
  }
  return found;
}

cv::Point2f ChessboardPoseDetector::getChessboardCenterPixel(
  const std::vector<cv::Point2f> & corners) const
{
  if (corners.empty()) {
    return cv::Point2f(-1.0f, -1.0f);
  }

  // 计算所有角点的平均值作为中心点
  cv::Point2f sum(0.0f, 0.0f);
  for (const auto & corner : corners) {
    sum += corner;
  }
  
  return cv::Point2f(sum.x / corners.size(), sum.y / corners.size());
}

std::vector<cv::Point3f> ChessboardPoseDetector::calculateCornerWorldCoordinates(
  float square_size, const cv::Mat& rotation_matrix, const cv::Vec3d& tvec) const
{
  std::vector<cv::Point3f> world_coords;
  
  // 在棋盘格坐标系中定义角点的3D坐标
  for (int i = 0; i < board_size_.height; ++i) {
    for (int j = 0; j < board_size_.width; ++j) {
      cv::Point3f local_point(j * square_size, i * square_size, 0);
      
      cv::Mat point_local = (cv::Mat_<double>(3,1) << 
          local_point.x, local_point.y, local_point.z);
      
      // 世界坐标 = R * 局部坐标 + t
      cv::Mat point_world = rotation_matrix * point_local + cv::Mat(tvec);
      
      world_coords.push_back(cv::Point3f(
          static_cast<float>(point_world.at<double>(0)),
          static_cast<float>(point_world.at<double>(1)),
          static_cast<float>(point_world.at<double>(2))
      ));
    }
  }
  
  return world_coords;
}

cv::Mat ChessboardPoseDetector::drawChessboardResults(
  const cv::Mat & frame,
  const MarkerInfo & pose_info,
  const std::vector<cv::Point2f> & corners,
  bool draw_axes) const
{
  cv::Mat result = frame.clone();

  if (pose_info.corners.empty()) {
    cv::putText(result, "No chessboard detected", cv::Point(20, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    return result;
  }

  // 绘制检测到的棋盘格角点
  cv::drawChessboardCorners(result, board_size_, corners, true);

  if (draw_axes) {
    // 绘制坐标轴
    cv::drawFrameAxes(result, camera_intrinsics_.camera_matrix, 
                      camera_intrinsics_.dist_coeffs, 
                      pose_info.rvec, pose_info.tvec, 
                      static_cast<float>(square_size_ * 2));  // 轴长度为两个方块大小
  }

  // 显示信息
  cv::putText(result, "Chessboard Detected", cv::Point(20, 40), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

  std::string pos_text = "Position (" + (trans_marker_mm_? std::string("mm") : std::string("m")) + "):" +
                        " X:" + std::to_string(pose_info.position.x).substr(0, 5) +
                        " Y:" + std::to_string(pose_info.position.y).substr(0, 5) +
                        " Z:" + std::to_string(pose_info.position.z).substr(0, 5);
  cv::putText(result, pos_text, cv::Point(20, 65), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

  std::string rot_text = "Rotation (deg): X:" + std::to_string(pose_info.rotation[0]).substr(0, 5) +
                        " Y:" + std::to_string(pose_info.rotation[1]).substr(0, 5) +
                        " Z:" + std::to_string(pose_info.rotation[2]).substr(0, 5);
  cv::putText(result, rot_text, cv::Point(20, 90), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

  if (pose_info.distance >= 0) {
    std::string dist_text = "Distance: " + std::to_string(pose_info.distance).substr(0, 5) + "m";
    cv::putText(result, dist_text, cv::Point(20, 115), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
  }

  if (pose_info.center_2d.x >= 0 && pose_info.center_2d.y >= 0) {
    std::string center_text = "Center (px): X:" + std::to_string(pose_info.center_2d.x).substr(0, 4) +
                             " Y:" + std::to_string(pose_info.center_2d.y).substr(0, 4);
    cv::putText(result, center_text, cv::Point(20, 140), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
  }

  return result;
}

bool ChessboardPoseDetector::calibrateCameraIntrinsics(const std::vector<cv::Mat>& images,
    cv::Mat & camera_matrix,
    cv::Mat & dist_coeffs)
{
  if (images.empty()) {
    std::cerr << "标定失败：输入图像集为空" << std::endl;
    return false;
  }
  std::vector<std::vector<cv::Point2f>> all_corners;
  std::vector<std::vector<cv::Point3f>> all_obj_points;
  cv::Size img_size;
  for (const auto& img : images) {
    if (img.empty()) continue;
    img_size = img.size();
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_size_, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (found) {
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria_);
      all_corners.push_back(corners);
      // 构造世界坐标
      std::vector<cv::Point3f> obj_pts;
      for (int i = 0; i < board_size_.height; ++i)
      {
        for (int j = 0; j < board_size_.width; ++j)
        {
          obj_pts.emplace_back(j * square_size_, i * square_size_, 0);
        }
      }    
      all_obj_points.push_back(obj_pts);
    }
  }
  if (all_corners.size() < 3) {
    std::cerr << "有效棋盘格图像数量不足，至少需要3张" << std::endl;
    return false;
  }
  std::vector<cv::Mat> rvecs, tvecs;
  double rms = cv::calibrateCamera(all_obj_points, all_corners, img_size, camera_matrix, dist_coeffs, rvecs, tvecs);
  std::cout << "标定完成，重投影误差: " << rms << std::endl;

  return true;
}

void ChessboardPoseDetector::printChessboardResults(const MarkerInfo & pose_info) const
{
  std::cout << "=== Chessboard Pose Info ===" << std::endl;
  std::cout << "Position (m) :"
            << " X:" << pose_info.position.x 
            << " Y:" << pose_info.position.y 
            << " Z:" << pose_info.position.z << std::endl;

  std::cout << "Rotation (deg): X:" << pose_info.rotation[0] 
            << " Y:" << pose_info.rotation[1] 
            << " Z:" << pose_info.rotation[2] << std::endl;

  if (pose_info.distance >= 0) {
    std::cout << "Distance: " << pose_info.distance << "m" << std::endl;
  }

  if (pose_info.center_2d.x >= 0 && pose_info.center_2d.y >= 0) {
    std::cout << "Center (px): X:" << pose_info.center_2d.x 
              << " Y:" << pose_info.center_2d.y << std::endl;
  }

  std::cout << "==========================" << std::endl;
}

bool ChessboardPoseDetector::detectAndEstimatePose(
  const cv::Mat & frame,
  MarkerInfo& result,
  bool print_results)
{
  bool found = estimatePose(frame, result);

  static bool printRes = false;
  if (print_results && printRes) 
  {
    if (found)
      printChessboardResults(result);
    else
      std::cout << "未检测到棋盘格" << std::endl;
  }

  return found;
}

/**
* @brief Detect and process ArUco markers (complete pipeline)
* 
* @param frame Input image frame
* @param draw_results Whether to draw results on frame (default: true)
* @param print_results Whether to print results to console (default: true)
* @return DetectionResult Complete detection result
*/
DetectionResult ChessboardPoseDetector::detectAndProcessMarkers(
    const cv::Mat & frame,
    void * depth_frame/* = nullptr*/,
    bool draw_results/* = true*/,
    bool print_results/* = true*/)
{
  DetectionResult result;
  result.marker_type = comm_alg::MarkerType::Chessboard;
  MarkerInfo marker_info;
  bool found = detectAndEstimatePose(frame, marker_info, print_results);
  if (found) {
    if (trans_marker_mm_)
    {
      marker_info.position.x *= 1000.0;
      marker_info.position.y *= 1000.0;
      marker_info.position.z *= 1000.0;
    }
    result.markers_info.push_back(marker_info);
    if (draw_results)
      result.processed_frame = drawChessboardResults(frame, marker_info, marker_info.corners, true);
    result.found = true;
  } else {
    result.found = false;
  }

  return result;
}

void ChessboardPoseDetector::setBoardSize(cv::Size board_size)
{
  board_size_ = board_size;
}

void ChessboardPoseDetector::setSquareSize(float square_size)
{
  square_size_ = square_size;
}

cv::Size ChessboardPoseDetector::getBoardSize() const
{
  return board_size_;
}

float ChessboardPoseDetector::getSquareSize() const
{
  return square_size_;
}


}  // namespace chessboard_alg