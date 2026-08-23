#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"

#include <algorithm>
#include <cstring>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace hs_calib {
namespace gui {

namespace {

/// \brief 类型列表是否含 sensor_msgs/Image
bool is_image_type(const std::vector<std::string> &types) {
  return std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end();
}

/// \brief 类型列表是否含 sensor_msgs/CameraInfo
bool is_camera_info_type(const std::vector<std::string> &types) {
  return std::find(types.begin(), types.end(), "sensor_msgs/msg/CameraInfo") !=
         types.end();
}

/// \brief 话题名是否像深度/视差
bool looks_like_depth_topic(const std::string &name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  return lower.find("depth") != std::string::npos ||
         lower.find("disparity") != std::string::npos;
}

}  // namespace

/// \brief 构造：创建图像桥节点（需 rclcpp 就绪）
RosImageBridge::RosImageBridge(QObject *parent)
    : QObject(parent) {
  if (!rclcpp::ok()) {
    RCLCPP_WARN(
        rclcpp::get_logger("hs_calib_gui"),
        "ROS image bridge: rclcpp not ok, offline-only");
    return;
  }
  node_ = std::make_shared<rclcpp::Node>("hs_calib_gui_image_bridge");
  RCLCPP_INFO(node_->get_logger(), "ROS image bridge node ready");
}

/// \brief 析构：退订并释放节点
RosImageBridge::~RosImageBridge() {
  unsubscribe();
  unsubscribe_camera_info();
  node_.reset();
}

/// \brief 枚举可用 Image 话题
QStringList RosImageBridge::list_image_topics() const {
  QStringList out;
  if (!is_ready()) {
    return out;
  }
  const auto topics = node_->get_topic_names_and_types();
  for (const auto &kv : topics) {
    if (!is_image_type(kv.second) || looks_like_depth_topic(kv.first)) {
      continue;
    }
    out.push_back(QString::fromStdString(kv.first));
  }
  out.sort();
  return out;
}

/// \brief 枚举可用 CameraInfo 话题
QStringList RosImageBridge::list_camera_info_topics() const {
  QStringList out;
  if (!is_ready()) {
    return out;
  }
  const auto topics = node_->get_topic_names_and_types();
  for (const auto &kv : topics) {
    if (!is_camera_info_type(kv.second)) {
      continue;
    }
    out.push_back(QString::fromStdString(kv.first));
  }
  out.sort();
  return out;
}

/// \brief 订阅图像话题（keep_last 1）
void RosImageBridge::subscribe(const QString &topic) {
  unsubscribe();
  subscribed_topic_ = topic;
  if (!is_ready()) {
    RCLCPP_WARN(
        rclcpp::get_logger("hs_calib_gui"),
        "subscribe skipped: ROS image bridge not ready (topic=%s)",
        topic.toStdString().c_str());
    return;
  }
  if (topic.isEmpty()) {
    RCLCPP_WARN(node_->get_logger(), "subscribe skipped: empty image topic");
    return;
  }
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic.toStdString(), qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });
  logged_first_frame_ = false;
  RCLCPP_INFO(node_->get_logger(), "subscribed image topic: %s", topic.toStdString().c_str());
}

/// \brief 订阅 CameraInfo
void RosImageBridge::subscribe_camera_info(const QString &topic) {
  unsubscribe_camera_info();
  camera_info_topic_ = topic;
  if (!is_ready()) {
    return;
  }
  if (topic.isEmpty()) {
    return;
  }
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  sub_info_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic.toStdString(), qos,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        on_camera_info(msg);
      });
  logged_first_info_ = false;
  RCLCPP_INFO(
      node_->get_logger(), "subscribed camera_info topic: %s",
      topic.toStdString().c_str());
}

/// \brief 取消图像订阅并清空缓存帧
void RosImageBridge::unsubscribe() {
  if (sub_ && node_) {
    RCLCPP_INFO(
        node_->get_logger(), "unsubscribe image topic: %s",
        subscribed_topic_.toStdString().c_str());
  }
  sub_.reset();
  subscribed_topic_.clear();
  logged_first_frame_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  latest_bgr_.release();
  has_frame_ = false;
}

/// \brief 取消 CameraInfo 订阅（保留最近一次 K/D）
void RosImageBridge::unsubscribe_camera_info() {
  if (sub_info_ && node_) {
    RCLCPP_INFO(
        node_->get_logger(), "unsubscribe camera_info topic: %s",
        camera_info_topic_.toStdString().c_str());
  }
  sub_info_.reset();
  camera_info_topic_.clear();
  logged_first_info_ = false;
}

/// \brief 返回最新 BGR 帧拷贝
cv::Mat RosImageBridge::latest_bgr() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_bgr_.empty() ? cv::Mat() : latest_bgr_.clone();
}

cv::Mat RosImageBridge::take_latest_bgr() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_frame_ || latest_bgr_.empty()) {
    return {};
  }
  cv::Mat out = std::move(latest_bgr_);
  latest_bgr_.release();
  has_frame_ = false;
  return out;
}

/// \brief 是否已有缓存帧
bool RosImageBridge::has_frame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_frame_ && !latest_bgr_.empty();
}

/// \brief 是否已有 CameraInfo
bool RosImageBridge::has_camera_info() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_camera_info_ && !camera_matrix_.empty();
}

/// \brief 返回 K 拷贝
cv::Mat RosImageBridge::camera_matrix() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return camera_matrix_.empty() ? cv::Mat() : camera_matrix_.clone();
}

/// \brief 返回 D 拷贝
cv::Mat RosImageBridge::dist_coeffs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dist_coeffs_.empty() ? cv::Mat() : dist_coeffs_.clone();
}

QString RosImageBridge::distortion_model() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return distortion_model_;
}

/// \brief 泵一次 ROS 回调
void RosImageBridge::spin_some() {
  if (node_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }
}

/// \brief 图像回调：解码为 BGR 并通知
void RosImageBridge::on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &) {
      cv_ptr = cv_bridge::toCvShare(msg);
    }
    cv::Mat bgr;
    if (cv_ptr->image.channels() == 1) {
      cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_GRAY2BGR);
    } else if (cv_ptr->encoding == "rgb8") {
      cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = cv_ptr->image.clone();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_bgr_ = std::move(bgr);
      has_frame_ = true;
    }
    if (!logged_first_frame_ && node_) {
      logged_first_frame_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "first image frame: %ux%u encoding=%s topic=%s",
          msg->width, msg->height, msg->encoding.c_str(),
          subscribed_topic_.toStdString().c_str());
    }
    emit frame_received();
  } catch (const std::exception &ex) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "image decode failed: %s", ex.what());
    }
  }
}

/// \brief CameraInfo 回调：缓存 K/D
void RosImageBridge::on_camera_info(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (!msg) {
    return;
  }
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      K.at<double>(r, c) = msg->k[static_cast<size_t>(r * 3 + c)];
    }
  }
  cv::Mat D;
  if (!msg->d.empty()) {
    D = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      D.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
  } else {
    D = cv::Mat::zeros(1, 5, CV_64F);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_matrix_ = K;
    dist_coeffs_ = D;
    distortion_model_ = QString::fromStdString(msg->distortion_model);
    has_camera_info_ = true;
  }
  if (!logged_first_info_ && node_) {
    logged_first_info_ = true;
    RCLCPP_INFO(
        node_->get_logger(),
        "first camera_info: %ux%u fx=%.1f fy=%.1f topic=%s",
        msg->width, msg->height, K.at<double>(0, 0), K.at<double>(1, 1),
        camera_info_topic_.toStdString().c_str());
  }
  emit camera_info_received();
}

}  // namespace gui
}  // namespace hs_calib
