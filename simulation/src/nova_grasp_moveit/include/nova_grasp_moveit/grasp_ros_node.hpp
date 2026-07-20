#pragma once

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
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

/// UI 一次刷新所需的单臂关节名称和弧度值。
struct ArmJointSnapshot
{
  std::vector<std::string> names;
  std::vector<double> positions_rad;
};

/// 从 TF 读取的腕部位姿；ok=false 表示本次查询失败。
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

/// ROS 后端的线程安全只读快照；Qt 定时器只读取该结构，不直接访问回调状态。
struct GraspRosSnapshot
{
  std::string grasp_status;
  std::vector<std::string> logs;
  std::vector<std::string> pose_logs;
  geometry_msgs::msg::PoseStamped box_pose;
  bool has_box_pose{false};
  /// 最近收到的一整帧 GraspNet 候选；按钮点击时会复制并冻结该帧。
  geometry_msgs::msg::PoseArray graspnet_candidates;
  bool has_graspnet_candidates{false};
  /// 切换话题后重新计数，用于判断新话题是否持续出流。
  int graspnet_message_count{0};
  /// 最近一次选择时满足顶部倾角阈值的候选数量。
  int graspnet_top_count{0};
  std::string graspnet_topic{"/yolo_graspnet/collision_free_grasps"};
  /// 第三方 PoseArray 发布状态：ok<1s，stale<3s；空数组表示在线但暂无位姿。
  bool graspnet_comm_ok{false};
  bool graspnet_comm_stale{false};
  double graspnet_age_sec{999.0};
  std::string graspnet_comm_summary;
  /// 最近选中的机器人 TCP 位姿，已经转换到 base_link。
  geometry_msgs::msg::PoseStamped selected_graspnet_pose;
  bool has_selected_graspnet_pose{false};
  int selected_graspnet_index{-1};
  int selected_graspnet_arm{-1};
  double selected_graspnet_approach_deg{180.0};
  std::string graspnet_selection_summary;
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

/// 计算/选择操作结果；失败字段直接用于 QMessageBox。
struct GraspComputeResult
{
  bool ok{false};
  std::string error_title;
  std::string error_message;
};

/// 连续执行或单步执行结果。
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

  /// 构造后再创建 TransformListener；必须在 shared_ptr 管理节点后调用。
  void init_tf_listener();
  /// 停止后台执行器和定时器，供 Qt 退出流程调用。
  void prepare_shutdown();
  /// 复制当前 ROS 状态并计算通信新鲜度。
  GraspRosSnapshot snapshot();
  /// 将 UI 生成的信息追加到统一日志。
  void append_ui_log(const std::string & line);
  void clear_logs();

  /// 使用最近 /box_pose 生成固定顶抓计划。
  GraspComputeResult compute_grasp_from_box_detailed();
  /// 冻结最新一帧 GraspNet PoseArray，转换到 base_link 后优先选顶抓，并用双臂 IK 选臂。
  GraspComputeResult compute_grasp_from_graspnet_detailed(double top_max_angle_deg = 30.0);
  /// 异步启动最近一次成功计算的计划。
  bool execute_last_plan();
  GraspExecuteResult execute_last_plan_detailed();
  /// 单步：计算后点一次走一步
  GraspExecuteResult execute_step_once_detailed();
  /// 发送 open/close 兼容字符串；精确开口优先使用 publish_gripper_opening_detailed。
  void send_gripper(const std::string & cmd);
  void send_gripper_for_arm(int arm_id, const std::string & cmd);
  /// 发布单臂 J1–J8 目标；另一臂不会被带入消息。
  void publish_arm_joints(int arm_id, const std::vector<double> & positions_rad);
  bool publish_arm_joints_detailed(int arm_id, const std::vector<double> & positions_rad, std::string * error_out = nullptr);
  bool publish_gripper_opening_detailed(int arm_id, double opening_m, std::string * error_out = nullptr);
  /// 只改 J7/J8，不覆盖该臂 J1–J6 的上次指令（避免全开/全闭带动臂）
  bool publish_gripper_joint_pair_detailed(
    int arm_id, double j7, double j8, std::string * error_out = nullptr);
  bool publish_gripper_joints_detailed(int arm_id, bool open_gripper, std::string * error_out = nullptr);
  /// 更新位姿和夹爪步骤等待时间，并同步到 GraspExecutor。
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
  /// 运行时重建 PoseArray 订阅，并清空旧话题候选和选择结果。
  bool set_graspnet_topic(const std::string & topic, std::string * error_out = nullptr);

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_box_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_graspnet_candidates(const geometry_msgs::msg::PoseArray::SharedPtr msg);
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
  /// 只检查 IK，不发送关节命令；返回相对当前关节的平方移动代价。
  bool check_candidate_ik(
    int arm_id, const geometry_msgs::msg::PoseStamped & wrist_pose,
    double & joint_motion_cost, std::string & reason);

  mutable std::mutex mu_;
  GraspRosSnapshot data_;
  GraspPlannerConfig planner_cfg_;
  std::map<std::string, double> joint_map_;
  /// 各关节上次指令缓存（仅用于可选 MuJoCo 全量数组）；Isaac /joint_command 始终按单臂子集发布
  std::map<std::string, double> last_commanded_;
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_graspnet_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<GraspExecutor> executor_;

  EePoseSnapshot cached_arm1_ee_;
  EePoseSnapshot cached_arm2_ee_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr graspnet_sub_;
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
  std::string graspnet_topic_{"/yolo_graspnet/collision_free_grasps"};

  /// UI 节点通过 /nova_pose_log 与独立 IK executor 完成一步请求/响应握手。
  enum class PoseStepResult { Pending, Ok, Fail };
  std::mutex pose_step_mu_;
  std::condition_variable pose_step_cv_;
  PoseStepResult pose_step_result_{PoseStepResult::Pending};
  std::string pose_step_fail_reason_;
};

}  // namespace nova_grasp_moveit
