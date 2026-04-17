#ifndef CALIB_SIM__CALIB_QT_UI_HPP_
#define CALIB_SIM__CALIB_QT_UI_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace calib_sim
{

class CalibQtUiRosNode : public rclcpp::Node
{
public:
  explicit CalibQtUiRosNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void sendCmd(const std::string & cmd);
  std::string status();
  std::string reachError();
  std::string resultText();
  std::vector<std::string> logs();
  sensor_msgs::msg::Image raw();
  sensor_msgs::msg::Image result();

private:
  std::mutex mu_;
  std::string status_{"waiting"};
  std::string reach_error_{"reach_err pos_mm=0 ang_deg=0"};
  std::string result_text_{"等待标定结果..."};
  std::vector<std::string> logs_;
  sensor_msgs::msg::Image raw_;
  sensor_msgs::msg::Image result_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reach_error_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_text_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctrl_pub_;
};

int RunCalibQtUiApp(const std::shared_ptr<CalibQtUiRosNode> & ros_node, int argc, char ** argv);

}  // namespace calib_sim

#endif  // CALIB_SIM__CALIB_QT_UI_HPP_
