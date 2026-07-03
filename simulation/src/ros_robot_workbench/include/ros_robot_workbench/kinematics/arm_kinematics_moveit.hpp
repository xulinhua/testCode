#ifndef ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_MOVEIT_HPP_
#define ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_MOVEIT_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ros_robot_workbench::kinematics
{

struct MoveitIkRequest
{
  std::string service{"/compute_ik"};
  std::string group_name;
  std::string ik_link_name;
  /// Pose 的 frame
  std::string frame_id{"base_link"};
  geometry_msgs::msg::Pose desired_pose;
  /// 全模型关节初值：关节名 -> 角（rad）
  std::map<std::string, double> seed;
  int timeout_ms{2000};
  bool avoid_collisions{false};
};

/// 通过 MoveIt2 的 GetPositionIK 服务（/compute_ik 等）请求逆解，与机械臂上 MoveIt/IKFast/插件 一致。
bool CallMoveitIk(
  const rclcpp::Node::SharedPtr & node, const MoveitIkRequest & req,
  std::vector<std::string> * joint_names_out, std::vector<double> * positions_out, std::string * err);

/// 查询 move_group 的 robot_description_kinematics.<group>.kinematics_solver 参数。
/// 返回值示例: "kdl_kinematics_plugin/KDLKinematicsPlugin" 或 ikfast 插件类名。
bool QueryMoveitKinematicsSolverPlugin(
  const rclcpp::Node::SharedPtr & node, const std::string & move_group_node_name,
  const std::string & group_name, std::string * plugin_class, std::string * err);

}  // namespace ros_robot_workbench::kinematics

#endif  // ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_MOVEIT_HPP_
