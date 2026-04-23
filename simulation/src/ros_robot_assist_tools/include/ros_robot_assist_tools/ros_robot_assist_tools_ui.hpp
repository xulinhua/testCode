#ifndef ROS_ROBOT_ASSIST_TOOLS__ROS_ROBOT_ASSIST_TOOLS_UI_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__ROS_ROBOT_ASSIST_TOOLS_UI_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ros_robot_assist_tools
{

// ROS2 节点入口
class RosRobotAssistToolsNode : public rclcpp::Node
{
public:
  explicit RosRobotAssistToolsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

int RunRosRobotAssistToolsUiApp(int argc, char ** argv);

}  // namespace ros_robot_assist_tools

#endif  // ROS_ROBOT_ASSIST_TOOLS__ROS_ROBOT_ASSIST_TOOLS_UI_HPP_
