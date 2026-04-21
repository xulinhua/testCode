#ifndef NOVA_ROBOT_TOOLS__NOVA_ROBOT_TOOLS_UI_HPP_
#define NOVA_ROBOT_TOOLS__NOVA_ROBOT_TOOLS_UI_HPP_

// 参考 calib_sim 架构：所有 Qt UI 在 .cpp 中实现，不使用 Q_OBJECT

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace nova_robot_tools
{

/// ROS2 节点：提供工具功能
class NovaRobotToolsNode : public rclcpp::Node
{
public:
  explicit NovaRobotToolsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 这里可以添加 ROS2 服务/话题接口
};

/// Qt UI 应用入口
int RunNovaRobotToolsUiApp(int argc, char ** argv);

}  // namespace nova_robot_tools

#endif  // NOVA_ROBOT_TOOLS__NOVA_ROBOT_TOOLS_UI_HPP_
