#pragma once

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nova_grasp_moveit/grasp_executor.hpp"
#include "nova_grasp_moveit/grasp_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nova_grasp_moveit
{

struct ArmJointSnapshot
{
  std::vector<std::string> names;
  std::vector<double> positions_rad;
};

struct EePoseSnapshot
{
  bool ok{false};
  std::string ref_frame;
  std::string child_frame;
  double x{0}, y{0}, z{0};
  double roll_deg{0}, pitch_deg{0}, yaw_deg{0};
  /// TF 原始四元数（xyzw）；位姿下发时优先用此值，避免 RPY≈±180° 往返导致低头
  double qx{0}, qy{0}, qz{0}, qw{1};
};

struct GraspRosSnapshot
{
  std::string grasp_status;
  std::vector<std::string> logs;
  std::vector<std::string> pose_logs;
  geometry_msgs::msg::PoseStamped box_pose;
  bool has_box_pose{false};
  geometry_msgs::msg::PoseStamped computed_grasp_pose;
  bool has_computed_grasp{false};
  GraspPlan last_plan;
  bool has_plan{false};
  ArmJointSnapshot arm1_joints;
  ArmJointSnapshot arm2_joints;
  EePoseSnapshot arm1_ee;
  EePoseSnapshot arm2_ee;
  bool ik_service_ready{false};
  bool executor_busy{false};
  // 机械臂通信状态
  bool arm_comm_ok{false};
  bool arm_comm_stale{false};
  double joint_states_age_sec{999.0};
  int joint_states_count{0};
  bool tf_comm_ok{false};
  std::string arm_comm_summary;
};

struct GraspComputeResult
{
  bool ok{false};
  std::string error_title;
  std::string error_message;
};

struct GraspExecuteResult
{
  bool ok{false};
  bool finished{false};
  int step_index{0};
  int step_count{0};
  std::string step_name;
  std::string error_title;
  std::string error_message;
};

/// ROS 后端：订阅关节/盒子位姿，执行抓取序列，供 Qt 线程安全读取。
class GraspRosNode : public rclcpp::Node
{
public:
  explicit GraspRosNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void init_tf_listener();
  void prepare_shutdown();
  GraspRosSnapshot snapshot();
  void append_ui_log(const std::string & line);
  void clear_logs();

  bool compute_grasp_from_box();
  GraspComputeResult compute_grasp_from_box_detailed();
  bool execute_last_plan();
  GraspExecuteResult execute_last_plan_detailed();
  /// 单步：计算后点一次走一步
  GraspExecuteResult execute_step_once_detailed();
  bool execute_from_computed_grasp();
  void send_gripper(const std::string & cmd);
  void send_gripper_for_arm(int arm_id, const std::string & cmd);
  void publish_arm_joints(int arm_id, const std::vector<double> & positions_rad);
  bool publish_arm_joints_detailed(int arm_id, const std::vector<double> & positions_rad, std::string * error_out = nullptr);
  bool publish_gripper_opening_detailed(int arm_id, double opening_m, std::string * error_out = nullptr);
  /// 只改 J7/J8，不覆盖该臂 J1–J6 的上次指令（避免全开/全闭带动臂）
  bool publish_gripper_joint_pair_detailed(
    int arm_id, double j7, double j8, std::string * error_out = nullptr);
  bool publish_gripper_joints_detailed(int arm_id, bool open_gripper, std::string * error_out = nullptr);
  void set_settle_timing(double step_settle_sec, double gripper_settle_sec);
  double step_settle_sec() const { return step_settle_sec_; }
  double gripper_settle_sec() const { return gripper_settle_sec_; }
  void send_arm_pose_goal(
    int arm_id, double x, double y, double z,
    double roll_deg, double pitch_deg, double yaw_deg,
    const std::string & frame_id = "base_link");
  /// 直接用四元数下发（推荐：与 TF 当前姿态一致时只改位置）
  void send_arm_pose_goal_quat(
    int arm_id, double x, double y, double z,
    double qx, double qy, double qz, double qw,
    const std::string & frame_id = "base_link");

  std::map<std::string, double> joint_positions() const;

  GraspPlannerConfig planner_config() const;
  void set_planner_config(const GraspPlannerConfig & cfg);

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_box_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_grasp_status(const std_msgs::msg::String::SharedPtr msg);
  void on_pose_log(const std_msgs::msg::String::SharedPtr msg);
  void push_log(const std::string & line);
  void push_log_nolock(const std::string & line);
  void refresh_ik_ready();
  void refresh_ee_poses();
  void reset_pose_step_wait();
  bool wait_pose_step(double timeout_sec, std::string * fail_reason = nullptr);
  ArmJointSnapshot extract_arm_joints(const std::map<std::string, double> & joints, const std::string & prefix) const;
  EePoseSnapshot lookup_ee_pose(const std::string & child_frame, const std::string & ref_frame) const;

  mutable std::mutex mu_;
  GraspRosSnapshot data_;
  GraspPlannerConfig planner_cfg_;
  std::map<std::string, double> joint_map_;
  /// 各关节上次指令缓存（仅用于可选 MuJoCo 全量数组）；Isaac /joint_command 始终按单臂子集发布
  std::map<std::string, double> last_commanded_;
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<GraspExecutor> executor_;

  EePoseSnapshot cached_arm1_ee_;
  EePoseSnapshot cached_arm2_ee_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_log_sub_;
  rclcpp::Publisher<nova_grasp_moveit::msg::ArmPose>::SharedPtr arm_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grasp_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_id_pub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::TimerBase::SharedPtr ik_watch_timer_;
  rclcpp::TimerBase::SharedPtr ee_refresh_timer_;

  double step_settle_sec_{2.0};
  double gripper_settle_sec_{0.8};

  enum class PoseStepResult { Pending, Ok, Fail };
  std::mutex pose_step_mu_;
  std::condition_variable pose_step_cv_;
  PoseStepResult pose_step_result_{PoseStepResult::Pending};
  std::string pose_step_fail_reason_;
};

}  // namespace nova_grasp_moveit
