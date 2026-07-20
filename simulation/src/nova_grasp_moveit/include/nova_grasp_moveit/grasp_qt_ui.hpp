#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "nova_grasp_moveit/grasp_ros_node.hpp"

namespace nova_grasp_moveit
{

using ShutdownHook = std::function<void()>;

int RunGraspQtUiApp(
  const std::shared_ptr<GraspRosNode> & ros_node,
  int argc, char ** argv,
  ShutdownHook on_shutdown = nullptr);

}  // namespace nova_grasp_moveit
