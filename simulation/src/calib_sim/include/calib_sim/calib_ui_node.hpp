#ifndef CALIB_SIM__CALIB_UI_NODE_HPP_
#define CALIB_SIM__CALIB_UI_NODE_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace calib_sim
{

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

}  // namespace calib_sim

#endif  // CALIB_SIM__CALIB_UI_NODE_HPP_
