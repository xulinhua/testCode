#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace hs_calib {
namespace gui {

/// \brief 后台多线程 ROS 执行器：图像/TF 回调不占用 Qt 主线程
class RosExecutorHub {
public:
  RosExecutorHub();
  ~RosExecutorHub();

  RosExecutorHub(const RosExecutorHub &) = delete;
  RosExecutorHub &operator=(const RosExecutorHub &) = delete;

  /// \brief 注册节点（可在 start 前后调用）
  void add_node(const rclcpp::Node::SharedPtr &node);

  /// \brief 启动后台 spin 线程
  void start(int thread_count = 2);

  /// \brief 停止并 join
  void stop();

  bool is_running() const { return running_.load(); }

private:
  void spin_loop();

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::mutex mutex_;
  std::vector<rclcpp::Node::SharedPtr> pending_nodes_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace gui
}  // namespace hs_calib
