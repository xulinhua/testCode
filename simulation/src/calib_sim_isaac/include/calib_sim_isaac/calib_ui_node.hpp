#ifndef CALIB_SIM__CALIB_UI_NODE_HPP_
#define CALIB_SIM__CALIB_UI_NODE_HPP_

// 轻量 OpenCV 高窗 UI：订阅标定状态与原始/结果图，定时刷新显示（无 Qt 依赖）。

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace calib_sim_isaac
{

/// OpenCV 窗口 UI 节点：参数指定 status/raw/result 话题与窗口名。
class CalibSimUiNode : public rclcpp::Node
{
public:
  explicit CalibSimUiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  static cv::Mat toBgr(const sensor_msgs::msg::Image & msg);
  void render();

private:
  std::string status_topic_;
  std::string raw_image_topic_;
  std::string result_image_topic_;
  std::string window_name_;
  std::string status_{"waiting"};
  cv::Mat raw_;
  cv::Mat result_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_UI_NODE_HPP_
