#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "nova_grasp_moveit/grasp_ros_node.hpp"

namespace nova_grasp_moveit
{

/// Qt 退出前停止 ROS executor 的回调。
using ShutdownHook = std::function<void()>;

/// 在调用线程运行 Qt 事件循环；ros_node 应由独立 ROS spin 线程驱动。
/// 返回 QApplication::exec() 的退出码。
int RunGraspQtUiApp(
  const std::shared_ptr<GraspRosNode> & ros_node,
  int argc, char ** argv,
  ShutdownHook on_shutdown = nullptr);

}  // namespace nova_grasp_moveit
