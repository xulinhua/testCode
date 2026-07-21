#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nova_grasp_moveit/msg/arm_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nova_grasp_moveit/grasp_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nova_grasp_moveit
{

/// 单步状态机一次调用的结果。
struct GraspStepResult
{
  bool ok{false};
  bool finished{false};  // 全部步骤已走完
  int step_index{0};     // 刚完成的步骤编号（1-based），0=尚未执行
  int step_count{0};
  std::string step_name;
  std::string error_title;
  std::string error_message;
};

/// 将 GraspPlan 编排为夹爪和腕部命令，支持连续线程和同步单步两种模式。
///
/// 本类不直接求 IK；腕部 Pose 通过 ArmPose 发布给独立的 GraspArmExecutor。
/// current_ee 回调提供实时 TF，pose-step 回调用 /nova_pose_log 判断 IK 是否完成。
class GraspExecutor
{
public:
  using StatusCallback = std::function<void(const std::string &)>;
  using LogCallback = std::function<void(const std::string &)>;
  using GripperApplyCallback = std::function<void(int arm_id, double opening_m)>;
  using CurrentEeCallback = std::function<bool(int arm_id, geometry_msgs::msg::Pose & out)>;
  using PoseStepResetCallback =
    std::function<void(const geometry_msgs::msg::Pose & expected_pose)>;
  using PoseStepWaitCallback = std::function<bool(double timeout_sec, std::string * fail_reason)>;

  GraspExecutor(
    rclcpp::Publisher<nova_grasp_moveit::msg::ArmPose>::SharedPtr arm_pose_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub,
    GraspPlannerConfig cfg,
    double step_settle_sec = 2.0,
    double gripper_settle_sec = 0.8);

  /// 更新后续规划/执行使用的参数；不会修改正在运行计划的副本。
  void set_config(const GraspPlannerConfig & cfg);
  void set_timing(double step_settle_sec, double gripper_settle_sec);
  bool is_busy() const;
  /// 后台线程连续执行完整计划；busy 时返回 false。
  bool start_sequence(const GraspPlan & plan);
  /// 兼容入口：从单个 Pose 创建基础计划后连续执行。
  bool start_from_grasp_pose(const geometry_msgs::msg::PoseStamped & grasp_pose);

  /// 计算后准备单步：刷新当前 EE、填路点、重置步进游标
  bool prepare_step_mode(const GraspPlan & plan, std::string * error_out = nullptr);
  /// 点一次走一步（同步，含等 IK）；全部走完 finished=true
  GraspStepResult step_once();
  int step_index() const;
  int step_count() const;
  void reset_step_mode();

  /// 注入 UI 状态、日志和硬件访问回调，避免执行器依赖 Qt/TF 实现。
  void set_callbacks(StatusCallback status_cb, LogCallback log_cb);
  void set_gripper_apply_callback(GripperApplyCallback cb);
  void set_current_ee_callback(CurrentEeCallback cb);
  void set_pose_step_callbacks(PoseStepResetCallback reset_cb, PoseStepWaitCallback wait_cb);
  /// 发布 open/close 兼容命令；正常抓取使用精确 opening 回调。
  void send_gripper(const std::string & cmd);
  /// 请求正在运行的序列在步骤边界退出。
  void request_shutdown();
  /// 复位/手动干预：立刻中止序列并清空 busy（避免一直报「执行器忙」）。
  void force_idle();

private:
  /// 固定的执行步骤类型；是否包含 Raise/Reorient 由计划标志决定。
  enum class StepKind
  {
    GripperOpen,
    Raise,
    MoveXy,
    Reorient,
    Descend,
    GripperClose,
    Lift
  };

  /// 单步状态机中的扁平步骤；夹爪步骤没有 pose。
  struct StepItem
  {
    StepKind kind;
    std::string name;
    geometry_msgs::msg::Pose pose;
    bool has_pose{false};
  };

  /// 连续模式线程函数，执行顺序必须与 build_step_list 保持一致。
  void run_sequence(GraspPlan plan);
  void publish_status(const std::string & text);
  void publish_log(const std::string & text);
  void send_arm_pose(int arm_id, const geometry_msgs::msg::PoseStamped & pose);
  bool send_arm_pose_and_wait(
    int arm_id, const geometry_msgs::msg::PoseStamped & pose, const char * step,
    bool do_settle = true);
  void apply_gripper_opening(int arm_id, double opening_m);
  void sleep_sec(double sec);
  /// 执行前刷新 EE；盒子计划重算路点，preserve_waypoints 计划只更新安全 Raise。
  void prepare_transit_poses(GraspPlan & plan);
  /// 将计划转换为 open→raise?→move→reorient?→descend→close→lift。
  std::vector<StepItem> build_step_list(const GraspPlan & plan) const;

  GraspPlannerConfig cfg_;
  double step_settle_sec_{2.0};
  double gripper_settle_sec_{0.8};
  rclcpp::Publisher<nova_grasp_moveit::msg::ArmPose>::SharedPtr arm_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  StatusCallback status_cb_;
  LogCallback log_cb_;
  GripperApplyCallback gripper_apply_cb_;
  CurrentEeCallback current_ee_cb_;
  PoseStepResetCallback pose_step_reset_cb_;
  PoseStepWaitCallback pose_step_wait_cb_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> shutdown_{false};

  mutable std::mutex step_mu_;
  GraspPlan step_plan_;
  std::vector<StepItem> step_items_;
  int step_cursor_{0};  // next index to run
  bool step_ready_{false};
};

}  // namespace nova_grasp_moveit
