#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace core {

/// \brief 内参畸变模型（前三种：Brown / Kannala–Brandt / CMei）
enum class CameraModelId {
  BrownConrady = 0,   ///< OpenCV pinhole / ROS plumb_bob
  KannalaBrandt = 1,  ///< OpenCV fisheye / ROS equidistant
  CMei = 2,           ///< OpenCV omnidir (Mei)
};

/// \brief 解析模型名（兼容 pinhole / fisheye / plumb_bob / equidistant 等别名）
CameraModelId parse_camera_model(const std::string &name);

/// \brief 规范导出名：brown_conrady | kannala_brandt | cmei
std::string camera_model_to_string(CameraModelId id);

/// \brief 人类可读短名
std::string camera_model_display_name(CameraModelId id);

/// \brief 按模型投影 3D 点到像素
bool project_points_model(
    CameraModelId model,
    const std::vector<cv::Point3f> &object_points,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    std::vector<cv::Point2f> *image_points);

/// \brief 按模型求解 PnP（通用点集）
bool solve_pnp_model(
    CameraModelId model,
    const std::vector<cv::Point3f> &object_points,
    const std::vector<cv::Point2f> &image_points,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    cv::Mat *rvec,
    cv::Mat *tvec,
    bool use_square = false);

/// \brief 模型感知画坐标轴（替代 cv::drawFrameAxes，避免 fisheye/CMei 误用 plumb_bob）
void draw_frame_axes_model(
    cv::Mat &image,
    CameraModelId model,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    float length,
    int thickness = 2);

}  // namespace core
}  // namespace hs_calib
