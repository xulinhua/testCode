#pragma once

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 从内参 YAML（本工程 export 格式）加载 K/D 与分辨率
struct CameraIntrinsics {
  bool valid = false;
  int image_width = 0;
  int image_height = 0;
  cv::Mat K;   ///< 3x3 CV_64F
  cv::Mat D;   ///< Nx1 或 1xN CV_64F（Brown=5，fisheye/CMei=4）
  double xi = 0.0;  ///< CMei 参数；其它模型为 0
  std::string model = "brown_conrady";  ///< brown_conrady | kannala_brandt | cmei
  std::string message;
};

/// \brief 从 YAML 文件加载相机内参
bool load_camera_yaml(const std::string &path, CameraIntrinsics *out);

/// \brief 将 CalibrationResult.intrinsics_meta 填入 CameraIntrinsics
bool camera_intrinsics_from_result(const CalibrationResult &result, CameraIntrinsics *out);

/// \brief 导出相机内参 YAML
bool export_camera_yaml(const CalibrationResult &result, const std::string &path);

/// \brief 从 result.intrinsics_meta 的 prefix 字段导出（如 left_ / right_）
bool export_camera_yaml_prefixed(
    const CalibrationResult &result, const std::string &prefix, const std::string &path);

/// \brief 格式化内参结果为文本
std::string format_intrinsics_text(const CalibrationResult &result);

/// \brief 写出手眼 / 外参 4x4 YAML
bool export_extrinsics_yaml(
    const CalibrationResult &result,
    const std::string &parent_frame,
    const std::string &child_frame,
    const std::string &path);

/// \brief 格式化外参结果为文本
std::string format_extrinsics_text(
    const CalibrationResult &result,
    const std::string &parent_frame,
    const std::string &child_frame);

/// \brief OpenCV rvec/tvec → 4×4 齐次矩阵
Eigen::Matrix4d cv_rt_to_matrix4d(const cv::Mat &rvec, const cv::Mat &tvec);
/// \brief 4×4 齐次矩阵 → OpenCV R/t
void matrix4d_to_cv_rt(const Eigen::Matrix4d &T, cv::Mat *R, cv::Mat *t);

}  // namespace core
}  // namespace hs_calib
