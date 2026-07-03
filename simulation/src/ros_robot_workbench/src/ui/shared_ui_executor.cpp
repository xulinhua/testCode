#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

#include <chrono>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace ros_robot_workbench::ui
{

SharedUiExecutor & SharedUiExecutor::instance()
{
  static SharedUiExecutor inst;
  return inst;
}

SharedUiExecutor::SharedUiExecutor() = default;
SharedUiExecutor::~SharedUiExecutor() = default;

void SharedUiExecutor::add_node(rclcpp::Node::SharedPtr node)
{
  if (!node) {
    return;
  }
  bool start_thread = false;
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (!exec_) {
      exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    exec_->add_node(node);
    if (!spin_thread_ || !spin_thread_->joinable()) {
      stop_.store(false);
      start_thread = true;
    }
  }
  if (start_thread) {
    spin_thread_ = std::make_unique<std::thread>([this]() {
      while (!stop_.load()) {
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ex;
        {
          std::lock_guard<std::mutex> lock(mu_);
          ex = exec_;
        }
        if (ex) {
          ex->spin_some(std::chrono::milliseconds(50));
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    });
  }
}

void SharedUiExecutor::remove_node(rclcpp::Node::SharedPtr node)
{
  if (!node) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (exec_) {
    exec_->remove_node(node);
  }
}

void SharedUiExecutor::shutdown()
{
  stop_.store(true);
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
  spin_thread_.reset();
  std::lock_guard<std::mutex> lock(mu_);
  exec_.reset();
}

}  // namespace ros_robot_workbench::ui
