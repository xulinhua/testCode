#include "ros_robot_assist_tools/module/image_viewer_module.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace ros_robot_assist_tools::ui
{

std::vector<QString> ListOnlineImageTopicsForViewer()
{
  std::vector<QString> topics;
  try {
    auto node = rclcpp::Node::make_shared("ros_robot_assist_tools_image_viewer_topics");
    const auto names_and_types = node->get_topic_names_and_types();
    for (const auto & item : names_and_types) {
      const auto & types = item.second;
      if (std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end()) {
        topics.emplace_back(QString::fromStdString(item.first));
      }
    }
    std::sort(topics.begin(), topics.end());
  } catch (...) {
  }
  return topics;
}

bool ConvertViewerRosImageToQImage(const sensor_msgs::msg::Image & msg, QImage * out_image)
{
  if (out_image == nullptr || msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return false;
  }
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  const QString encoding = QString::fromStdString(msg.encoding).toLower();

  if (encoding == "rgb8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_RGB888);
    *out_image = img.copy();
    return true;
  }
  if (encoding == "bgr8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_BGR888);
    *out_image = img.copy();
    return true;
  }
  if (encoding == "mono8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_Grayscale8);
    *out_image = img.copy();
    return true;
  }
  if (encoding == "bgra8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_ARGB32);
    *out_image = img.rgbSwapped().copy();
    return true;
  }
  if (encoding == "rgba8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_RGBA8888);
    *out_image = img.copy();
    return true;
  }
  if (encoding == "mono16" || encoding == "16uc1") {
    QImage gray(width, height, QImage::Format_Grayscale8);
    const auto * src = reinterpret_cast<const std::uint16_t *>(msg.data.data());
    const int stride = static_cast<int>(msg.step / sizeof(std::uint16_t));
    std::uint16_t min_v = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_v = 0;
    for (int y = 0; y < height; ++y) {
      const auto * row = src + y * stride;
      for (int x = 0; x < width; ++x) {
        const std::uint16_t v = row[x];
        if (v == 0) continue;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
      }
    }
    if (max_v <= min_v) {
      min_v = 0;
      max_v = 1;
    }
    const double inv = 255.0 / static_cast<double>(max_v - min_v);
    for (int y = 0; y < height; ++y) {
      auto * dst = gray.scanLine(y);
      const auto * row = src + y * stride;
      for (int x = 0; x < width; ++x) {
        const std::uint16_t v = row[x];
        if (v == 0) {
          dst[x] = 0;
        } else {
          const int mapped = static_cast<int>(std::round((v - min_v) * inv));
          dst[x] = static_cast<uchar>(std::clamp(mapped, 0, 255));
        }
      }
    }
    *out_image = gray;
    return true;
  }
  if (encoding == "32fc1") {
    QImage gray(width, height, QImage::Format_Grayscale8);
    const auto * src = reinterpret_cast<const float *>(msg.data.data());
    const int stride = static_cast<int>(msg.step / sizeof(float));
    float min_v = std::numeric_limits<float>::max();
    float max_v = 0.0f;
    for (int y = 0; y < height; ++y) {
      const auto * row = src + y * stride;
      for (int x = 0; x < width; ++x) {
        const float v = row[x];
        if (!std::isfinite(v) || v <= 0.0f) continue;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
      }
    }
    if (!(max_v > min_v)) {
      min_v = 0.0f;
      max_v = 1.0f;
    }
    const float inv = 255.0f / (max_v - min_v);
    for (int y = 0; y < height; ++y) {
      auto * dst = gray.scanLine(y);
      const auto * row = src + y * stride;
      for (int x = 0; x < width; ++x) {
        const float v = row[x];
        if (!std::isfinite(v) || v <= 0.0f) {
          dst[x] = 0;
        } else {
          const int mapped = static_cast<int>(std::round((v - min_v) * inv));
          dst[x] = static_cast<uchar>(std::clamp(mapped, 0, 255));
        }
      }
    }
    *out_image = gray;
    return true;
  }
  return false;
}

}  // namespace ros_robot_assist_tools::ui
