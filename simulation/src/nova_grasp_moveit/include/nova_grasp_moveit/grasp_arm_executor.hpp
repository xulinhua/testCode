#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "nova_grasp_moveit/msg/arm_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace nova_grasp_moveit
{

/// 订阅 /nova_target_arm_pose，调用 /compute_ik，发布 /joint_command（Isaac）与 /arm_controller/commands。
class GraspArmExecutor : public rclcpp::Node
{
public:
  GraspArmExecutor();

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_arm_id(const std_msgs::msg::Int32::SharedPtr msg);
  void on_target_arm_pose(const nova_grasp_moveit::msg::ArmPose::SharedPtr msg);
  void process_pose_goal(int arm_id, const geometry_msgs::msg::PoseStamped & pose_in);
  void handle_ik_response(
    const rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture & future,
    int request_arm_id, const std::string & request_prefix);
  void publish_pose_log(const std::string & line);
  void on_gripper_goal(const std_msgs::msg::String::SharedPtr msg);
  /// command_map 内有哪些关节就只发哪些（Isaac 严格单臂）；MuJoCo 数组仍拼全量
  void publish_command(
    const std::unordered_map<std::string, double> & command_map, bool force = false);
  void publish_pose_joint_debug_lines(
    const std::unordered_map<std::string, double> & command_map, int request_arm_id);
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
  int joint_command_burst_count_{5};
  mutable std::mutex joint_mu_;

  struct IkRequestContext
  {
    std::atomic<bool> active{true};
    rclcpp::TimerBase::SharedPtr timeout_timer;
    int arm_id{0};
    std::string prefix;
  };
  std::mutex ik_ctx_mu_;
  std::shared_ptr<IkRequestContext> ik_ctx_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_log_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_id_sub_;
  rclcpp::Subscription<nova_grasp_moveit::msg::ArmPose>::SharedPtr target_arm_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_sub_;
};

}  // namespace nova_grasp_moveit
