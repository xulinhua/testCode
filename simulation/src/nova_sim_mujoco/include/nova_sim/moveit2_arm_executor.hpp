#pragma once

// MoveIt2 IK 执行器：订阅目标位姿与夹爪命令，调用 /compute_ik，向 /arm_controller/commands 发布关节指令。

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "calib_sim_mujoco/msg/arm_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

/// 多臂 nova：arm_id 选择 planning group，IK 种子来自当前 /joint_states。
class MoveIt2ArmExecutorCpp : public rclcpp::Node
{
public:
  MoveIt2ArmExecutorCpp();

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_arm_id(const std_msgs::msg::Int32::SharedPtr msg);
  void on_target_arm_pose(const calib_sim_mujoco::msg::ArmPose::SharedPtr msg);
  void process_pose_goal(int arm_id, const geometry_msgs::msg::PoseStamped & pose_in);
  void on_gripper_goal(const std_msgs::msg::String::SharedPtr msg);
  void publish_command(const std::unordered_map<std::string, double> & command_map);
  double resolve_command_value(
    const std::string & joint, const std::unordered_map<std::string, double> & command_map) const;

  int arm_id_;
  std::unordered_map<int, std::string> arm_groups_;
  std::unordered_map<int, std::string> ee_links_;
  std::unordered_map<int, std::string> arm_prefix_;
  std::vector<std::string> control_joint_order_;
  std::unordered_map<int, std::pair<std::string, std::string>> gripper_joints_;
  std::unordered_map<int, std::pair<double, double>> gripper_open_;
  std::unordered_map<int, std::pair<double, double>> gripper_close_;
  std::unordered_map<std::string, double> current_joint_map_;
  std::unordered_map<std::string, double> last_published_command_map_;
  bool has_last_command_{false};

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_log_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_id_sub_;
  rclcpp::Subscription<calib_sim_mujoco::msg::ArmPose>::SharedPtr target_arm_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_sub_;
};
