#ifndef ROS_ROBOT_WORKBENCH__ROS_ROBOT_WORKBENCH_UI_HPP_
#define ROS_ROBOT_WORKBENCH__ROS_ROBOT_WORKBENCH_UI_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ros_robot_workbench
{

// ROS2 节点入口
class RosRobotWorkbenchNode : public rclcpp::Node
{
public:
  explicit RosRobotWorkbenchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

int RunRosRobotWorkbenchUiApp(int argc, char ** argv);

}  // namespace ros_robot_workbench

#endif  // ROS_ROBOT_WORKBENCH__ROS_ROBOT_WORKBENCH_UI_HPP_
