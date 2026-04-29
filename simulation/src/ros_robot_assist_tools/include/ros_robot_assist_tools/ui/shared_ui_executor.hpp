#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__SHARED_UI_EXECUTOR_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__UI__SHARED_UI_EXECUTOR_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace ros_robot_assist_tools::ui
{

class SharedUiExecutor
{
public:
  static SharedUiExecutor & instance();

  void add_node(rclcpp::Node::SharedPtr node);
  void remove_node(rclcpp::Node::SharedPtr node);
  void shutdown();

private:
  SharedUiExecutor();
  ~SharedUiExecutor();

  std::mutex mu_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace ros_robot_assist_tools::ui

#endif
