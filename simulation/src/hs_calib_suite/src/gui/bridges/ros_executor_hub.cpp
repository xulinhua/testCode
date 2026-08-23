#include "hs_calib_suite/gui/bridges/ros_executor_hub.hpp"

#include <algorithm>
#include <chrono>

namespace hs_calib {
namespace gui {

RosExecutorHub::RosExecutorHub() = default;

RosExecutorHub::~RosExecutorHub() {
  stop();
}

void RosExecutorHub::add_node(const rclcpp::Node::SharedPtr &node) {
  if (!node) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (executor_) {
    executor_->add_node(node);
    return;
  }
  pending_nodes_.push_back(node);
}

void RosExecutorHub::start(int thread_count) {
  if (running_.load() || !rclcpp::ok()) {
    return;
  }
  const int threads = std::max(2, thread_count);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), static_cast<size_t>(threads));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &node : pending_nodes_) {
      if (node) {
        executor_->add_node(node);
      }
    }
    pending_nodes_.clear();
  }
  stop_requested_.store(false);
  running_.store(true);
  spin_thread_ = std::thread([this]() { spin_loop(); });
}

void RosExecutorHub::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  stop_requested_.store(true);
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();
}

void RosExecutorHub::spin_loop() {
  while (!stop_requested_.load() && rclcpp::ok() && executor_) {
    executor_->spin_some(std::chrono::milliseconds(10));
  }
}

}  // namespace gui
}  // namespace hs_calib
