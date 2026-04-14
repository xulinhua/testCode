#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class MoveIt2ArmExecutorCpp : public rclcpp::Node
{
public:
  MoveIt2ArmExecutorCpp();

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_arm_id(const std_msgs::msg::Int32::SharedPtr msg);
  void on_pose_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void on_gripper_goal(const std_msgs::msg::String::SharedPtr msg);
  void publish_command(const std::unordered_map<std::string, double> & command_map);

  int arm_id_;
  std::unordered_map<int, std::string> arm_groups_;
  std::unordered_map<int, std::string> ee_links_;
  std::unordered_map<int, std::string> arm_prefix_;
  std::vector<std::string> control_joint_order_;
  std::unordered_map<int, std::pair<std::string, std::string>> gripper_joints_;
  std::unordered_map<int, std::pair<double, double>> gripper_open_;
  std::unordered_map<int, std::pair<double, double>> gripper_close_;
  std::unordered_map<std::string, double> current_joint_map_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_log_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_id_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_sub_;
};
