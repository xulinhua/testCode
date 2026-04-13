#include "../include/aruco_alg/aruco_detector.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <unistd.h>
#include <dlfcn.h>
#include "log_system/log_macros.hpp"

namespace aruco_alg
{

ArucoDetector::ArucoDetector(
  std::vector<double> marker_length,
  std::vector<int> aruco_dict_type)
: enable_scaling_(false),
  scale_factor_(0.5),
  force_pnp_(true),
  print_debug_info_(false)
{
  marker_type_ = comm_alg::MarkerType::Aruco;
  if (marker_length.size() != aruco_dict_type.size()) {
    LOG_ERROR("marker_length和aruco_dict_type的大小不一致");
    marker_length = {0.1};
    aruco_dict_type = {cv::aruco::DICT_5X5_100};
  }

  for (size_t i = 0; i < marker_length.size(); ++i) 
  {
    ArucoDetectorInfo info(marker_length[i], static_cast<cv::aruco::PredefinedDictionaryType>(aruco_dict_type[i]));
    detector_info_list_.push_back(info);

    LOG_INFO("  第%d个Aruco检测器初始化:", i);
    LOG_INFO("  marker_length: %f", info.marker_length_);
    LOG_INFO("  aruco_dict_type: %d", info.aruco_dict_type_);
    LOG_INFO("  minMarkerPerimeterRate: %f", info.parameters_->minMarkerPerimeterRate);
    LOG_INFO("  maxMarkerPerimeterRate: %f", info.parameters_->maxMarkerPerimeterRate);
    LOG_INFO("  errorCorrectionRate: %f", info.parameters_->errorCorrectionRate);
  }
}

bool ArucoDetector::detectMarkers(const cv::Ptr<cv::aruco::ArucoDetector> detector,
                                  const cv::Mat& image, 
                                 std::vector<std::vector<cv::Point2f>>& corners, 
                                 std::vector<int>& ids,
                                 std::vector<std::vector<cv::Point2f>>& rejected)
{
  if (image.empty()) {
        LOG_ERROR("检测失败：输入图像为空");
        return false;
    }

  cv::Mat scaled_image;
  double scale_x = 1.0, scale_y = 1.0;
  
  // Apply image scaling if enabled
  if (enable_scaling_ && scale_factor_ != 1.0) {
    int width = static_cast<int>(image.cols * scale_factor_);
    int height = static_cast<int>(image.rows * scale_factor_);
    cv::resize(image, scaled_image, cv::Size(width, height));
    scale_x = static_cast<double>(image.cols) / width;
    scale_y = static_cast<double>(image.rows) / height;
  } else {
    scaled_image = image.clone();
  }
  
  cv::Mat gray;
  cv::cvtColor(scaled_image, gray, cv::COLOR_BGR2GRAY);
  
  if (print_debug_info_) {
    LOG_DEBUG("图像尺寸: %dx%d", image.cols, image.rows);
    LOG_DEBUG("灰度图像尺寸: %dx%d", gray.cols, gray.rows);
  }
  
  detector->detectMarkers(gray, corners, ids, rejected);
  
  if (print_debug_info_) {
    LOG_DEBUG("detectMarkers结果 - corners: %zu, ids: %zu", corners.size(), ids.size());
  }
  
  // Scale corners back to original image size
  if (enable_scaling_ && scale_factor_ != 1.0 && !corners.empty()) {
    for (auto & corner : corners) {
      for (auto & point : corner) {
        point.x *= scale_x;
        point.y *= scale_y;
      }
    }
  }
  
  return true;
}

cv::Point2f ArucoDetector::getMarkerCenterPixel(
  const std::vector<std::vector<cv::Point2f>> & corners,
  size_t marker_index) const
{
  if (corners.empty() || marker_index >= corners.size()) {
    return cv::Point2f(-1.0f, -1.0f);
  }
  
  const auto & corner = corners[marker_index];
  // Calculate center using diagonal points for better precision
  float x = (corner[0].x + corner[2].x) / 2.0f;
  float y = (corner[0].y + corner[2].y) / 2.0f;
  
  return cv::Point2f(x, y);
}

std::vector<cv::Point3f> ArucoDetector::calculateCornerWorldCoordinates(
    double marker_length, const cv::Mat& rotation_matrix, const cv::Vec3d& tvec) const
{
    std::vector<cv::Point3f> world_coords;
    
    // 在标记坐标系中定义四个角点的3D坐标
    // 假设标记中心在原点，标记在XY平面上
    float half_len = static_cast<float>(marker_length) / 2.0f;
    
    std::vector<cv::Point3f> marker_corners_local = {
        cv::Point3f(-half_len, -half_len, 0),  // 左下角 (corner 0)
        cv::Point3f( half_len, -half_len, 0),  // 右下角 (corner 1)  
        cv::Point3f( half_len,  half_len, 0),  // 右上角 (corner 2)
        cv::Point3f(-half_len,  half_len, 0)   // 左上角 (corner 3)
    };
    
    // 将局部坐标转换到世界坐标系
    for (const auto& local_point : marker_corners_local) {
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
    
    return world_coords;
}

#if 0
MarkerInfo ArucoDetector::getMarkerResultWithDepth(
  const std::vector<std::vector<cv::Point2f>> & corners,
  const rs2::depth_frame & depth_frame,
  const rs2::intrinsics & intrinsics,
  float marker_length,
  size_t marker_index) const
{
  MarkerInfo info;
  info.marker_id = -1;
  info.distance = -1.0f;
  
  if (corners.empty() || marker_index >= corners.size()) {
    return info;
  }
  
  try {
    // Get marker corners as integer points
    std::vector<cv::Point> corner_points;
    for (const auto & point : corners[marker_index]) {
      corner_points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }
    
    // Sample points within marker region
    std::vector<cv::Point3f> point_cloud;
    cv::Rect rect = cv::boundingRect(corner_points);
    
    // Dense sampling within marker region
    for (int i = 0; i < rect.width; i += 3) {
      for (int j = 0; j < rect.height; j += 3) {
        int u = rect.x + i;
        int v = rect.y + j;
        
        // Check if point is inside marker
        if (cv::pointPolygonTest(corner_points, cv::Point2f(u, v), false) >= 0) {
          float depth = depth_frame.get_distance(u, v);
          // Strict depth filtering (0.1-10.0 meters)
          if (0.1f < depth && depth < 10.0f) {
            rs2::float2 pixel = {static_cast<float>(u), static_cast<float>(v)};
            rs2::float3 point_3d = rs2::rs2_deproject_pixel_to_point(intrinsics, pixel, depth);
            point_cloud.emplace_back(point_3d.x, point_3d.y, point_3d.z);
          }
        }
      }
    }
    
    // Need enough points for PCA analysis (at least 10 points)
    if (point_cloud.size() < 10) {
      return info;
    }
    
    // Perform PCA analysis to get normal vector
    cv::Vec3f normal_vector = performPCA(point_cloud);
    
    // Adjust normal vector direction (pointing towards camera)
    if (normal_vector[2] > 0) {
      normal_vector = -normal_vector;
    }
    
    // Normalize normal vector
    float norm = cv::norm(normal_vector);
    normal_vector /= norm;
    
    // Construct rotation matrix
    cv::Vec3f temp_up(0.0f, 0.0f, -1.0f);
    cv::Vec3f x_axis = temp_up.cross(normal_vector);
    if (cv::norm(x_axis) < 1e-6f) {
      temp_up = cv::Vec3f(0.0f, 1.0f, 0.0f);
      x_axis = temp_up.cross(normal_vector);
    }
    
    x_axis /= cv::norm(x_axis);
    cv::Vec3f y_axis = normal_vector.cross(x_axis);
    y_axis /= cv::norm(y_axis);
    
    // Build rotation matrix
    cv::Matx33f rotation_matrix(
      x_axis[0], y_axis[0], normal_vector[0],
      x_axis[1], y_axis[1], normal_vector[1],
      x_axis[2], y_axis[2], normal_vector[2]
    );
    
    // Convert to Euler angles
    cv::Vec3f euler_angles = rotationMatrixToEulerAngles(rotation_matrix);
    
    // Get marker center 3D position
    cv::Point2f center_2d = getMarkerCenterPixel(corners, marker_index);
    if (center_2d.x < 0 || center_2d.y < 0) {
      return info;
    }
    
    float center_depth = depth_frame.get_distance(static_cast<int>(center_2d.x), static_cast<int>(center_2d.y));
    if (center_depth <= 0) {
      return info;
    }
    
    rs2::float2 center_pixel = {center_2d.x, center_2d.y};
    rs2::float3 center_3d = rs2::rs2_deproject_pixel_to_point(intrinsics, center_pixel, center_depth);
    // cv::drawFrameAxes(result, camera_matrix, dist_coeffs, rvec_mat, tvec_mat, static_cast<float>(marker_length_));
    cv::drawFrameAxes(result, camera_matrix, dist_coeffs, rvec_mat, tvec_mat, static_cast<float>(0.1));

    cv::Point3f position(center_3d.x, center_3d.y, center_3d.z);
    float distance = cv::norm(position);
    
    // Convert rotation matrix to rvec
    cv::Mat rvec_mat;
    cv::Rodrigues(cv::Mat(rotation_matrix), rvec_mat);
    cv::Vec3d rvec(rvec_mat.at<double>(0), rvec_mat.at<double>(1), rvec_mat.at<double>(2));
    cv::Vec3d tvec(center_3d.x, center_3d.y, center_3d.z);
    
    // Calculate corner world coordinates
    std::vector<cv::Point3f> corner_world_coords = calculateCornerWorldCoordinates(
      marker_length, rotation_matrix, tvec);

    // Fill marker info
    info.center_2d = center_2d;
    info.position = position;
    info.rotation = euler_angles;
    info.rotation_matrix = rotation_matrix;
    info.rvec = rvec;
    info.tvec = tvec;
    info.world_corners = corner_world_coords;
    info.corners = corners[target_idx];
    info.distance = distance;
    
    return info;
    
  } catch (const std::exception & e) {
    std::cout << "点云法向量计算错误: " << e.what() << std::endl;
    return info;
  }
}
#endif

MarkerInfo ArucoDetector::getMarkerResultPca(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const cv::Mat & depth_frame,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    size_t marker_index) const
{
  MarkerInfo info;
  info.marker_id = -1;
  info.distance = -1.0f;
  
  if (corners.empty() || marker_index >= corners.size()) {
    return info;
  }
  LOG_INFO("获取标记结果PCA");
  try {
    // Get marker corners as integer points
    std::vector<cv::Point> corner_points;
    for (const auto & point : corners[marker_index]) {
      corner_points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }
    LOG_INFO("获取标记结果%d, 深度图size: %d x %d", marker_index, depth_frame.cols, depth_frame.rows);
    // Sample points within marker region
    std::vector<cv::Point3f> point_cloud;
    cv::Rect rect = cv::boundingRect(corner_points);
    LOG_INFO("获取标记结果%d, rect: %d, %d, %d, %d", marker_index, rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    
    float cx = camera_matrix.at<double>(0, 2);
    float cy = camera_matrix.at<double>(1, 2);
    float fx = camera_matrix.at<double>(0, 0);
    float fy = camera_matrix.at<double>(1, 1);
    LOG_INFO("相机内参: cx: %f, cy: %f, fx: %f, fy: %f", cx, cy, fx, fy);
    // Dense sampling within marker region
    for (int i = 10; i < rect.width - 10; i += 3) {
      for (int j = 10; j < rect.height - 10; j += 3) {
        int u = rect.x + i;
        int v = rect.y + j;
        
        // Check if point is inside marker
        if (cv::pointPolygonTest(corner_points, cv::Point2f(u, v), false) >= 0) 
        {
          if (depth_frame.rows < u || depth_frame.cols < v)
            continue;
          float depth = depth_frame.at<float>(u, v) / 1000.0; // mm -> m
          // LOG_INFO("获取深度: %f", depth);
          // Strict depth filtering (0.1-10.0 meters)
          if (0.1f < depth && depth < 10.0f) {
            cv::Point3f point_3d;
            point_3d.z = depth;
            point_3d.x = (u - cx) * depth / fx;
            point_3d.y = (v - cy) * depth / fy;
            // float X = (det.bbox[0] - intrinsics.cx) * Z / intrinsics.fx;
            // float Y = (det.bbox[1] - intrinsics.cy) * Z / intrinsics.fy;
            point_cloud.emplace_back(point_3d);
            // LOG_INFO("世界坐标: %f, %f, %f", point_3d.x, point_3d.y, point_3d.z);
          }
        }
      }
    }
    
    LOG_INFO("获取点云标记数量: %d", point_cloud.size());
    // Need enough points for PCA analysis (at least 10 points)
    if (point_cloud.size() < 10) {
      return info;
    }
    
    // Perform PCA analysis to get normal vector
    cv::Vec3f normal_vector = performPCA(point_cloud);
    
    // Adjust normal vector direction (pointing towards camera)
    if (normal_vector[2] > 0) {
      normal_vector = -normal_vector;
    }
    
    // Normalize normal vector
    float norm = cv::norm(normal_vector);
    normal_vector /= norm;
    
    // Construct rotation matrix
    cv::Vec3f temp_up(0.0f, 0.0f, -1.0f);
    cv::Vec3f x_axis = temp_up.cross(normal_vector);
    if (cv::norm(x_axis) < 1e-6f) {
      temp_up = cv::Vec3f(0.0f, 1.0f, 0.0f);
      x_axis = temp_up.cross(normal_vector);
    }
    
    x_axis /= cv::norm(x_axis);
    cv::Vec3f y_axis = normal_vector.cross(x_axis);
    y_axis /= cv::norm(y_axis);
    
    // Build rotation matrix
    cv::Matx33f rotation_matrix(
      x_axis[0], y_axis[0], normal_vector[0],
      x_axis[1], y_axis[1], normal_vector[1],
      x_axis[2], y_axis[2], normal_vector[2]
    );
    
    // Convert to Euler angles
    cv::Vec3f euler_angles = rotationMatrixToEulerAngles(rotation_matrix);
    LOG_INFO("欧拉角: %f, %f, %f", euler_angles[0], euler_angles[1], euler_angles[2]);
    
    // Get marker center 3D position
    cv::Point2f center_2d = getMarkerCenterPixel(corners, marker_index);
    if (center_2d.x < 0 || center_2d.y < 0) {
      return info;
    }
    
    float center_depth = -1.0;
    cv::Point center_pix;
    center_pix.x = static_cast<int>(center_2d.x);
    center_pix.y = static_cast<int>(center_2d.y);
    if (center_2d.y >= 0 && depth_frame.rows > center_2d.y && 
      center_2d.x >=0 && depth_frame.cols > center_2d.x)
    {
      center_depth = depth_frame.at<float>(center_pix);
      center_depth /= 1000.0;
    }
    LOG_INFO("获取中心点(%d, %d) 深度: %f", center_pix.x, center_pix.y, center_depth);
    if (center_depth <= 0) {
      return info;
    }
    
    cv::Point3f position;
    position.x = (center_2d.x - cx) * center_depth / fx;
    position.y = (center_2d.y - cy) * center_depth / fy;
    position.z = center_depth;
    LOG_INFO("世界坐标: %f, %f, %f", position.x, position.y, position.z);

    float distance = cv::norm(position);
    
    // Convert rotation matrix to rvec
    cv::Mat rvec_mat;
    cv::Rodrigues(cv::Mat(rotation_matrix), rvec_mat);
    cv::Vec3d rvec(rvec_mat.at<double>(0), rvec_mat.at<double>(1), rvec_mat.at<double>(2));
    cv::Vec3d tvec(position.x, position.y, position.z);
    LOG_INFO("rvec: %f, %f, %f", rvec[0], rvec[1], rvec[2]);

    // Calculate corner world coordinates
    // std::vector<cv::Point3f> corner_world_coords = calculateCornerWorldCoordinates(
    //  marker_length_, cv::Mat(rotation_matrix), tvec);

    // Fill marker info
    info.center_2d = center_2d;
    info.position = position;
    info.rotation = euler_angles;
    info.rotation_matrix = rotation_matrix;
    info.rvec = rvec;
    info.tvec = tvec;
    // info.world_corners = corner_world_coords;
    info.corners = corners[marker_index];
    info.distance = distance;
    info.marker_id = marker_index;
    
    return info;
    
  } catch (const std::exception & e) {
    std::cout << "点云法向量计算错误: " << e.what() << std::endl;
    return info;
  }
}
      
    

MarkerInfo ArucoDetector::getMarkerResultPnP(
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs,
  float marker_length,
  int target_id) const
{
  MarkerInfo info;
  info.marker_id = -1;
  info.distance = -1.0f;
  
  if (ids.empty()) {
    return info;
  }
  
  // Find target marker
  size_t target_idx = 0;
  if (target_id != -1) {
    auto it = std::find(ids.begin(), ids.end(), target_id);
    if (it == ids.end()) {
      return info;
    }
    target_idx = std::distance(ids.begin(), it);
  } else {
    target_id = ids[target_idx];
  }
  
  // Estimate pose for each marker
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, static_cast<float>(marker_length), 
    camera_matrix, dist_coeffs, rvecs, tvecs);
  
  if (target_idx >= rvecs.size() || target_idx >= tvecs.size()) {
    return info;
  }
  
  // Get position and rotation information
  cv::Vec3d rvec = rvecs[target_idx];
  cv::Vec3d tvec = tvecs[target_idx];
  cv::Point3f position(static_cast<float>(tvec[0]), static_cast<float>(tvec[1]), static_cast<float>(tvec[2]));
  
  // Calculate rotation matrix
  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);
  
  // Calculate distance
  float distance = static_cast<float>(cv::norm(tvec));
  
  // Convert rotation angles to degrees
  // cv::Vec3f rotation_degrees(
  //   static_cast<float>(cv::norm(rvec[0]) * 180.0 / CV_PI),
  //   static_cast<float>(cv::norm(rvec[1]) * 180.0 / CV_PI),
  //   static_cast<float>(cv::norm(rvec[2]) * 180.0 / CV_PI)
  // );

  // 转欧拉角，而不是旋转角度
  // 1. 旋转向量转旋转矩阵
  // 2. 从旋转矩阵提取欧拉角（ZYX顺序）
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

  // 弧度转角度
  cv::Vec3f euler_angles(
    static_cast<float>(x * 180.0 / CV_PI),
    static_cast<float>(y * 180.0 / CV_PI),
    static_cast<float>(z * 180.0 / CV_PI)
  );
  
  // Get 2D center point
  cv::Point2f center_2d = getMarkerCenterPixel(corners, target_idx);
  
  // Calculate corner world coordinates
  std::vector<cv::Point3f> corner_world_coords = calculateCornerWorldCoordinates(
    marker_length, rotation_matrix, tvec);
 
  // Fill marker info
  info.center_2d = center_2d;
  info.position = position;
  info.rotation = euler_angles;
  info.rotation_matrix = cv::Matx33f(rotation_matrix);
  info.rvec = rvec;
  info.tvec = tvec;
  info.corners = corners[target_idx];
  info.world_corners = corner_world_coords;
  info.distance = distance;
  info.marker_id = target_id;
  
  return info;
}

std::vector<MarkerInfo> ArucoDetector::getArucoResults(
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs,
  float marker_length, 
  void * depth_frame,
  void * intrinsics
) const
{
  if (print_debug_info_) {
    LOG_DEBUG("DEBUG: get_aruco_results输入参数 - corners: %zu, ids: %zu", corners.size(), ids.size());
  }
  
  std::vector<MarkerInfo> results;
  
  if (ids.empty()) {
    LOG_DEBUG("DEBUG: ids为空，直接返回空结果");
    return results;
  }
  
  // Use PnP method if forced or no depth information available
  if (force_pnp_) 
  {  // Fallback to PnP
    if (print_debug_info_) {
      LOG_DEBUG("DEBUG: 使用PnP方法计算位姿");
    }
    
    for (size_t i = 0; i < ids.size(); ++i) {
      try {
        MarkerInfo pose_info = getMarkerResultPnP(corners, ids, camera_matrix, dist_coeffs, marker_length, ids[i]);
        if (pose_info.marker_id != -1) {
          results.push_back(pose_info);
          if (print_debug_info_) {
            LOG_DEBUG("DEBUG: 成功添加标记 %d 的位姿信息", ids[i]);
          }
        } else {
          LOG_DEBUG("DEBUG: 标记 %d 的位姿信息为无效", ids[i]);
        }
      } catch (const std::exception & e) {
        LOG_ERROR("标记 %d 计算失败: %s", ids[i], e.what());
        continue;
      }
    }
  } 
  else if (force_pnp_ == false && depth_frame != nullptr)
  {  // Fallback to PnP
    if (print_debug_info_) {
      LOG_DEBUG("DEBUG: 使用PCA方法计算位姿");
    }
    LOG_INFO("DEBUG: 使用PCA方法计算位姿");
    for (size_t i = 0; i < ids.size(); ++i) {
      try {
        MarkerInfo pose_info = getMarkerResultPca(corners, *(cv::Mat*)depth_frame, camera_matrix, dist_coeffs, ids[i]);
        if (pose_info.marker_id != -1) {
          results.push_back(pose_info);
          if (print_debug_info_) {
            LOG_INFO("DEBUG: 成功添加标记 %d 的位姿信息", ids[i]);
          }
        } else {
          LOG_INFO("DEBUG: 标记 %d 的位姿信息为无效", ids[i]);
        }
      } catch (const std::exception & e) {
        LOG_ERROR("标记 %d 计算失败: %s", ids[i], e.what());
        continue;
      }
    }
  } 
  else
    LOG_ERROR("没有深度信息可用，无法使用深度方法计算位姿");
  
  if (print_debug_info_) {
    LOG_DEBUG("DEBUG: get_aruco_results返回结果 - results: %zu", results.size());
  }
  
  return results;
}

cv::Mat ArucoDetector::drawArucoResults(
  const cv::Mat & frame,
  const std::vector<MarkerInfo> & markers_info,
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs) const
{
  cv::Mat result = frame.clone();
  
  // Check if markers were found
  if (ids.empty()) {
    cv::putText(result, "No markers detected", cv::Point(20, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    return result;
  }
  
  // Draw marker borders
  cv::aruco::drawDetectedMarkers(result, corners);
  
  // Draw coordinate system and display info for each marker
  for (size_t i = 0; i < markers_info.size(); ++i) {
    const auto & marker_info = markers_info[i];
    
    // Get marker information
    int marker_id = marker_info.marker_id;
    cv::Point3f position = marker_info.position;
    cv::Vec3f rotation = marker_info.rotation;
    cv::Point2f center_2d = marker_info.center_2d;
    cv::Vec3d rvec = marker_info.rvec;
    cv::Vec3d tvec = marker_info.tvec;
    
    // Use provided rvec and tvec, or calculate from rotation matrix
    cv::Mat rvec_mat, tvec_mat;
    if (rvec[0] != 0 || rvec[1] != 0 || rvec[2] != 0) {
      rvec_mat = cv::Mat(rvec);
      tvec_mat = cv::Mat(tvec);
    } else if (cv::countNonZero(marker_info.rotation_matrix) > 0) {
      cv::Rodrigues(cv::Mat(marker_info.rotation_matrix), rvec_mat);
      tvec_mat = (cv::Mat_<double>(3, 1) << position.x, position.y, position.z);
    } else {
      rvec_mat = (cv::Mat_<double>(3, 1) << 0, 0, 0);
      tvec_mat = (cv::Mat_<double>(3, 1) << position.x, position.y, position.z);
    }
    
    //cv::rectangle(result, cv::Rect(corners[i][0].x - 10, corners[i][0].y - 10, 20, 20), cv::Scalar(128, 128, 128), 2);

    // Draw marker coordinate system
    cv::drawFrameAxes(result, camera_matrix, dist_coeffs, rvec_mat, tvec_mat, static_cast<float>(0.1));
    
    // Display information for each marker
    int text_y = 40 + static_cast<int>(i) * 120;
    cv::putText(result, "ID: " + std::to_string(marker_id), 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(0, 255, 255), 2);
    std::string unit = trans_marker_mm_ ? "mm" : "m";
    std::string pos_text = "Position (" + unit + "): X:" + std::to_string(position.x).substr(0, 5) +
                          " Y:" + std::to_string(position.y).substr(0, 5) +
                          " Z:" + std::to_string(position.z).substr(0, 5);
    cv::putText(result, pos_text, cv::Point(20, text_y + 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
    
    std::string rot_text = "Rotation (deg): X:" + std::to_string(rotation[0]).substr(0, 5) +
                          " Y:" + std::to_string(rotation[1]).substr(0, 5) +
                          " Z:" + std::to_string(rotation[2]).substr(0, 5);
    cv::putText(result, rot_text, cv::Point(20, text_y + 50), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
    
    // Display distance information
    if (marker_info.distance >= 0) {
      std::string dist_text = "Distance: " + std::to_string(marker_info.distance).substr(0, 5) + "m";
      cv::putText(result, dist_text, cv::Point(20, text_y + 75), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    }
    
    // Display 2D center point coordinates
    if (center_2d.x >= 0 && center_2d.y >= 0) {
      std::string center_text = "Center (px): X:" + std::to_string(center_2d.x).substr(0, 4) +
                               " Y:" + std::to_string(center_2d.y).substr(0, 4);
      cv::putText(result, center_text, cv::Point(20, text_y + 100), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
      
      // Display ID next to marker
      cv::putText(result, std::to_string(marker_id), 
                  cv::Point(static_cast<int>(center_2d.x), static_cast<int>(center_2d.y)), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    } else if (i < corners.size()) {
      cv::Point center = (corners[i][0] + corners[i][2]) / 2;
      center.x = static_cast<int>(center.x);
      center.y = static_cast<int>(center.y);
      cv::putText(result, std::to_string(marker_id), center, 
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
  }
  
  return result;
}

void ArucoDetector::printArucoResults(const std::vector<MarkerInfo> & markers_info) const
{
  static bool printRes = false;
  if (!printRes) return;
  for (size_t i = 0; i < markers_info.size(); ++i) 
  {
    const auto & marker_info = markers_info[i];
    
    LOG_INFO("=== Marker %d ===", marker_info.marker_id);
    LOG_INFO("Position (%s): X:%.3f Y:%.3f Z:%.3f", trans_marker_mm_ ? "mm" : "m", 
              marker_info.position.x, marker_info.position.y, marker_info.position.z);
    
    LOG_INFO("Rotation (deg): X:%.3f Y:%.3f Z:%.3f", 
              marker_info.rotation[0], marker_info.rotation[1], marker_info.rotation[2]);
    
    if (marker_info.distance >= 0) {
      LOG_INFO("Distance: %.3fm", marker_info.distance);
    }
    
    if (marker_info.center_2d.x >= 0 && marker_info.center_2d.y >= 0) {
      LOG_INFO("Center (px): X:%.1f Y:%.1f", 
                marker_info.center_2d.x, marker_info.center_2d.y);
    }
    
    if (cv::countNonZero(marker_info.rotation_matrix) > 0) {
      cv::Vec3f euler_angles = rotationMatrixToEulerAngles(marker_info.rotation_matrix);
      LOG_INFO("Rotation (deg): X:%.1f Y:%.1f Z:%.1f", 
                euler_angles[0], euler_angles[1], euler_angles[2]);
    }
    
    if (i < markers_info.size() - 1) {
      LOG_INFO("------------------------------");
    }
  }
}

DetectionResult ArucoDetector::detectAndProcessMarkers(
  const cv::Mat & frame,
  void * depth_frame,
  bool draw_results,
  bool print_results)
{
  DetectionResult result;
  result.marker_type = comm_alg::MarkerType::Aruco;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> rejected;

  std::vector<std::vector<cv::Point2f>> corners_all;
  std::vector<int> ids_all;
  std::vector<MarkerInfo> markers_info_all;

  // Check if camera intrinsics are set
  if (camera_intrinsics_.camera_matrix.empty() || camera_intrinsics_.dist_coeffs.empty()) {
    throw std::runtime_error("相机内参未设置，请先调用setCameraIntrinsics方法");
  }
  
  // Get camera intrinsics
  const cv::Mat & mtx = camera_intrinsics_.camera_matrix;
  const cv::Mat & dist = camera_intrinsics_.dist_coeffs;

  for (size_t i = 0; i < detector_info_list_.size(); ++i) 
  {
    // Detect ArUco markers
    bool bRet = detectMarkers(detector_info_list_[i].detector_, frame, corners, ids, rejected);
    if (print_debug_info_) {
      LOG_DEBUG("DEBUG: detect_markers返回结果 - corners类型: vector, ids类型: vector");
      LOG_DEBUG("DEBUG: detect_markers返回结果 - corners: %zu, ids: %zu", corners.size(), ids.size());
    }
  
    // Get complete pose information for markers
    std::vector<MarkerInfo> markers_info;
  
    markers_info = getArucoResults(corners, ids, mtx, dist, detector_info_list_[i].marker_length_, depth_frame, nullptr);

    corners_all.insert(corners_all.end(), corners.begin(), corners.end());
    ids_all.insert(ids_all.end(), ids.begin(), ids.end());
    markers_info_all.insert(markers_info_all.end(), markers_info.begin(), markers_info.end());
  }
  
  
  if (print_debug_info_) {
    LOG_DEBUG("DEBUG: get_aruco_results返回结果 - markers_info: %zu", markers_info_all.size());
  }
  
  // Check if markers were detected
  bool found_markers = !ids_all.empty();
  
  if (trans_marker_mm_)
  {
    for (size_t i = 0; i < markers_info_all.size(); ++i) {
      markers_info_all[i].position.x *= 1000.0;
      markers_info_all[i].position.y *= 1000.0;
      markers_info_all[i].position.z *= 1000.0;
    }
  }
  result.markers_info = markers_info_all;
  result.found = found_markers;
  
  // Draw results if needed
  if (draw_results && found_markers && !markers_info_all.empty()) {
    result.processed_frame = drawArucoResults(frame, markers_info_all, corners_all, ids_all, mtx, dist);
  } else if (draw_results) {
    cv::putText(frame, "No markers detected", cv::Point(20, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    result.processed_frame = frame.clone();
  }
  
  // Print results if needed
  if (print_results && found_markers && !markers_info_all.empty()) {
    printArucoResults(markers_info_all);
  } else if (print_results) {
    LOG_DEBUG("No markers detected");
  }
  
  return result;
}

// Setter methods
void ArucoDetector::setEnableScaling(bool enable)
{
  enable_scaling_ = enable;
}

void ArucoDetector::setScaleFactor(double factor)
{
  scale_factor_ = factor;
}

void ArucoDetector::setForcePnP(bool force_pnp)
{
  force_pnp_ = force_pnp;
}

void ArucoDetector::setPrintDebugInfo(bool print_debug)
{
  print_debug_info_ = print_debug;
}

// Getter methods
double ArucoDetector::getMarkerLength() const
{
  return 0.1;
}

bool ArucoDetector::isEnableScaling() const
{
  return enable_scaling_;
}

double ArucoDetector::getScaleFactor() const
{
  return scale_factor_;
}

bool ArucoDetector::isForcePnP() const
{
  return force_pnp_;
}

bool ArucoDetector::isPrintDebugInfo() const
{
  return print_debug_info_;
}


cv::Vec3f ArucoDetector::performPCA(const std::vector<cv::Point3f> & point_cloud) const
{
  if (point_cloud.empty()) {
    return cv::Vec3f(0, 0, 1);
  }
  
  // Calculate mean
  cv::Point3f mean(0, 0, 0);
  for (const auto & point : point_cloud) {
    mean += point;
  }
  mean *= (1.0f / static_cast<float>(point_cloud.size()));
  
  // Calculate covariance matrix
  cv::Matx33f covariance(0, 0, 0, 0, 0, 0, 0, 0, 0);
  for (const auto & point : point_cloud) {
    cv::Point3f diff = point - mean;
    covariance(0, 0) += diff.x * diff.x;
    covariance(0, 1) += diff.x * diff.y;
    covariance(0, 2) += diff.x * diff.z;
    covariance(1, 0) += diff.y * diff.x;
    covariance(1, 1) += diff.y * diff.y;
    covariance(1, 2) += diff.y * diff.z;
    covariance(2, 0) += diff.z * diff.x;
    covariance(2, 1) += diff.z * diff.y;
    covariance(2, 2) += diff.z * diff.z;
  }
  
  covariance *= (1.0f / static_cast<float>(point_cloud.size()));
  
  // Compute eigenvectors and eigenvalues
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(covariance, eigenvalues, eigenvectors);
  
  // Return the eigenvector corresponding to the smallest eigenvalue (normal vector)
  return cv::Vec3f(eigenvectors.at<double>(2, 0), 
                   eigenvectors.at<double>(2, 1), 
                   eigenvectors.at<double>(2, 2));
}

}  // namespace aruco_alg