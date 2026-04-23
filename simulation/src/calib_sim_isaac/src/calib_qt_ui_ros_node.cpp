#include "calib_sim_isaac/calib_qt_ui.hpp"

namespace calib_sim_isaac
{

CalibQtUiRosNode::CalibQtUiRosNode(const rclcpp::NodeOptions & options)
: Node("calib_qt_ui_node", options)
{
  status_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim_isaac/status", 20, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      status_ = msg->data;
    });
  log_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim_isaac/log", 50, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      logs_.push_back(msg->data);
      if (logs_.size() > 200) {
        logs_.erase(logs_.begin());
      }
    });
  reach_error_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim_isaac/reach_error", 20, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      reach_error_ = msg->data;
    });
  result_text_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim_isaac/result_text", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      result_text_ = msg->data;
    });
  raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/calib_sim_isaac/raw_image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      raw_ = *msg;
    });
  result_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/calib_sim_isaac/result_image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      result_ = *msg;
    });
  ctrl_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim_isaac/control", 10);
}

void CalibQtUiRosNode::sendCmd(const std::string & cmd)
{
  std_msgs::msg::String m;
  m.data = cmd;
  ctrl_pub_->publish(m);
}

void CalibQtUiRosNode::clearLogsAndResult()
{
  std::lock_guard<std::mutex> lk(mu_);
  logs_.clear();
  result_text_ = "等待标定结果...";
  raw_ = sensor_msgs::msg::Image{};
  result_ = sensor_msgs::msg::Image{};
}

std::string CalibQtUiRosNode::status()
{
  std::lock_guard<std::mutex> lk(mu_);
  return status_;
}

std::string CalibQtUiRosNode::reachError()
{
  std::lock_guard<std::mutex> lk(mu_);
  return reach_error_;
}

std::string CalibQtUiRosNode::resultText()
{
  std::lock_guard<std::mutex> lk(mu_);
  return result_text_;
}

std::vector<std::string> CalibQtUiRosNode::logs()
{
  std::lock_guard<std::mutex> lk(mu_);
  return logs_;
}

sensor_msgs::msg::Image CalibQtUiRosNode::raw()
{
  std::lock_guard<std::mutex> lk(mu_);
  return raw_;
}

sensor_msgs::msg::Image CalibQtUiRosNode::result()
{
  std::lock_guard<std::mutex> lk(mu_);
  return result_;
}

}  // namespace calib_sim_isaac
