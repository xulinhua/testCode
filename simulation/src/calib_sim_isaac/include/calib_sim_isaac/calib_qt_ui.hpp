#ifndef CALIB_SIM__CALIB_QT_UI_HPP_
#define CALIB_SIM__CALIB_QT_UI_HPP_

// Qt 标定界面：订阅图像与日志，向标定节点发送控制字符串；与 CalibQtUiRosNode 配合。

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace calib_sim_isaac
{

/// 供 Qt 线程安全读写的 ROS 侧：订阅标定状态/图像，发布控制字符串。
class CalibQtUiRosNode : public rclcpp::Node
{
public:
  explicit CalibQtUiRosNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void sendCmd(const std::string & cmd);
  /// 清空订阅累积的日志与标定结果文本，并清空最近图像缓存（供 UI 重置显示）
  void clearLogsAndResult();
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

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_QT_UI_HPP_
