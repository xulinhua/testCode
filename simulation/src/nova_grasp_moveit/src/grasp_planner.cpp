#include "nova_grasp_moveit/grasp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nova_grasp_moveit
{

namespace
{

geometry_msgs::msg::Pose copy_pose(const geometry_msgs::msg::Pose & pose)
{
  return pose;
}

void quat_to_rpy_deg(
  const geometry_msgs::msg::Quaternion & q, double & roll_deg, double & pitch_deg, double & yaw_deg)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  if (tq.length2() < 1e-12) {
    roll_deg = pitch_deg = yaw_deg = 0.0;
    return;
  }
  tq.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  roll_deg = roll * 180.0 / M_PI;
  pitch_deg = pitch * 180.0 / M_PI;
  yaw_deg = yaw * 180.0 / M_PI;
}

}  // namespace

int choose_arm_id(double target_x, double split_x)
{
  return target_x >= split_x ? 1 : 0;
}

geometry_msgs::msg::Pose box_pose_to_grasp_pose(
  const geometry_msgs::msg::Pose & box,
  const GraspPlannerConfig & cfg)
{
  // 只用盒子位置；姿态不跟盒子，执行时整段保持当前 EE 姿态。
  geometry_msgs::msg::Pose grasp;
  grasp.position = box.position;
  grasp.position.z += cfg.box_grasp_z_offset;
  grasp.orientation.x = 0.0;
  grasp.orientation.y = 0.0;
  grasp.orientation.z = 0.0;
  grasp.orientation.w = 1.0;
  return grasp;
}

GraspPlan plan_grasp_from_pose(
  const geometry_msgs::msg::Pose & grasp_pose,
  const std::string & frame_id,
  const GraspPlannerConfig & cfg)
{
  GraspPlan plan;
  plan.frame_id = frame_id.empty() ? "base_link" : frame_id;
  plan.grasp = copy_pose(grasp_pose);
  plan.finger_grasp_target = plan.grasp.position;
  plan.has_finger_grasp_target = true;
  plan.arm_id = choose_arm_id(plan.grasp.position.x, cfg.arm_split_x);
  return plan;
}

void apply_tcp_to_wrist(geometry_msgs::msg::Pose & pose, double tcp_along_ee_z)
{
  // MoveIt 跟踪腕部 J*_6；指尖 TCP ≈ 腕部原点沿 link +Z 偏移 tcp_along_ee_z。
  // 目标位姿按「指尖到点」规划后，须换算成腕部指令：wrist = tcp - R*[0,0,tcp_z]
  if (tcp_along_ee_z <= 1e-6) {
    return;
  }
  tf2::Quaternion q(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (q.length2() < 1e-12) {
    return;
  }
  q.normalize();
  const tf2::Vector3 tip_in_world = tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, tcp_along_ee_z);
  pose.position.x -= tip_in_world.x();
  pose.position.y -= tip_in_world.y();
  pose.position.z -= tip_in_world.z();
}

/// 固定顶抓姿态：Rx180（+Z 朝下）+ 指定绝对 yaw。不继承当前 EE 的歪姿态。
geometry_msgs::msg::Quaternion canonical_top_down_quat(double yaw_deg)
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw_deg * M_PI / 180.0);
  return tf2::toMsg(q.normalized());
}

void fill_plan_waypoints(
  GraspPlan & plan,
  const GraspPlannerConfig & cfg,
  const geometry_msgs::msg::Pose * current_ee)
{
  if (!plan.has_finger_grasp_target) {
    plan.finger_grasp_target = plan.grasp.position;
    plan.has_finger_grasp_target = true;
  }
  const geometry_msgs::msg::Point & finger = plan.finger_grasp_target;

  const double pre_dz = std::clamp(
    std::max(cfg.pregrasp_z_offset, cfg.min_approach_clearance), 0.10, 0.30);
  const double lift_dz = std::clamp(std::max(cfg.lift_z_offset, 0.10), 0.10, 0.30);
  const double tcp_z = std::clamp(cfg.ee_tcp_z_offset, 0.0, 0.30);
  plan.path_tcp_z_offset = tcp_z;

  // MoveIt/MTC 常见做法：笛卡尔路点用固定顶抓姿态，不用当前歪 EE。
  // approach: yaw=0；grasp: yaw=grasp_yaw_offset_deg（默认 90，下降前再转）。
  const auto approach_q = canonical_top_down_quat(0.0);
  const auto grasp_q = canonical_top_down_quat(cfg.grasp_yaw_offset_deg);
  plan.has_reorient = std::abs(cfg.grasp_yaw_offset_deg) > 5.0;

  const double tcp_grasp_z = finger.z;
  const double tcp_pre_z = tcp_grasp_z + pre_dz;
  const double tcp_lift_z = tcp_grasp_z + lift_dz;

  // 预位 / 平移：盒心 XY + 预位高度 + 顶抓 yaw=0（与当前姿态无关）
  plan.pregrasp.position.x = finger.x;
  plan.pregrasp.position.y = finger.y;
  plan.pregrasp.position.z = tcp_pre_z;
  plan.pregrasp.orientation = approach_q;

  // 原地转 yaw：指尖 TCP 保持在盒心 XY（须按 grasp_q 重算腕部，不能复用 pregrasp 腕位）
  plan.reorient.position.x = finger.x;
  plan.reorient.position.y = finger.y;
  plan.reorient.position.z = tcp_pre_z;
  plan.reorient.orientation = grasp_q;

  plan.grasp.position.x = finger.x;
  plan.grasp.position.y = finger.y;
  plan.grasp.position.z = tcp_grasp_z;
  plan.grasp.orientation = grasp_q;

  plan.lift.position.x = finger.x;
  plan.lift.position.y = finger.y;
  plan.lift.position.z = tcp_lift_z;
  plan.lift.orientation = grasp_q;

  apply_tcp_to_wrist(plan.pregrasp, tcp_z);
  apply_tcp_to_wrist(plan.reorient, tcp_z);
  apply_tcp_to_wrist(plan.grasp, tcp_z);
  apply_tcp_to_wrist(plan.lift, tcp_z);

  plan.has_raise = false;
  plan.need_vertical_raise = false;
  if (current_ee != nullptr) {
    const double wrist_pre_z = plan.pregrasp.position.z;
    plan.need_vertical_raise = current_ee->position.z < wrist_pre_z - 0.015;
    // 抬高：只改当前 XY 的 Z，姿态立刻切到顶抓 approach（避免带着歪姿态抬高）
    plan.raise.position.x = current_ee->position.x;
    plan.raise.position.y = current_ee->position.y;
    plan.raise.position.z = wrist_pre_z;
    plan.raise.orientation = approach_q;
    plan.has_raise = true;
  }
}

std::string format_pose_xyz_q(const geometry_msgs::msg::Pose & pose)
{
  double rd = 0.0;
  double pd = 0.0;
  double yd = 0.0;
  quat_to_rpy_deg(pose.orientation, rd, pd, yd);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4)
      << "pos=(" << pose.position.x << "," << pose.position.y << "," << pose.position.z << ") "
      << "quat=(" << pose.orientation.x << "," << pose.orientation.y << ","
      << pose.orientation.z << "," << pose.orientation.w << ") "
      << "rpy_deg=(" << rd << "," << pd << "," << yd << ")";
  return oss.str();
}

std::vector<std::string> format_plan_path_lines(const GraspPlan & plan)
{
  std::vector<std::string> lines;
  if (plan.preserve_waypoints) {
    lines.push_back(
      "[PATH] mode=GraspNet arm_id=" + std::to_string(plan.arm_id) +
      " frame=" + plan.frame_id +
      "  open -> raise?(Z) -> move_xy(pregrasp) -> descend(grasp) -> close -> lift");
    lines.push_back(
      "[PATH] note: poses below are WRIST J*_6 (MoveIt IK input), "
      "NOT GraspNet/TCP finger-center. "
      "finger_center was converted by: wrist = tcp - R*[0,0,tcp_offset]");
  } else {
    lines.push_back(
      "[PATH] mode=box arm_id=" + std::to_string(plan.arm_id) +
      " frame=" + plan.frame_id +
      "  open -> raise(Z) -> move_xy -> reorient(yaw) -> descend(Z) -> close -> lift");
    lines.push_back(
      "[PATH] note: reorient wrist != pregrasp wrist (same finger XY, grasp yaw); "
      "do NOT lock wrist XYZ during reorient");
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[PATH] note: finger_tcp_z=" << plan.finger_grasp_target.z
        << " tcp_offset=" << plan.path_tcp_z_offset
        << " | wrist_z pre=" << plan.pregrasp.position.z
        << " grasp=" << plan.grasp.position.z
        << " lift=" << plan.lift.position.z;
    // Recover intended finger Z at grasp/lift from wrist + offset along TCP +Z
    // (log-only; approach may be slightly tilted).
    oss << " | (expect finger≈wrist+offset along TCP +Z when top-down)";
    lines.push_back(oss.str());
  }
  if (plan.has_raise) {
    lines.push_back(
      "[PATH] 1.raise    " + format_pose_xyz_q(plan.raise) +
      (plan.need_vertical_raise ? "  (will run)" : "  (skip, already high)"));
  } else {
    lines.push_back("[PATH] 1.raise    (no current EE, skipped)");
  }
  lines.push_back("[PATH] 2.move_xy  " + format_pose_xyz_q(plan.pregrasp));
  if (plan.has_reorient) {
    lines.push_back("[PATH] 3.reorient " + format_pose_xyz_q(plan.reorient));
    lines.push_back("[PATH] 4.descend  " + format_pose_xyz_q(plan.grasp));
    lines.push_back("[PATH] 5.lift     " + format_pose_xyz_q(plan.lift));
  } else {
    lines.push_back("[PATH] 3.descend  " + format_pose_xyz_q(plan.grasp));
    lines.push_back("[PATH] 4.lift     " + format_pose_xyz_q(plan.lift));
  }
  return lines;
}

std::string format_plan_summary(const GraspPlan & plan)
{
  std::ostringstream oss;
  const auto lines = format_plan_path_lines(plan);
  for (size_t i = 0; i < lines.size(); ++i) {
    if (i > 0) {
      oss << "\n";
    }
    oss << lines[i];
  }
  return oss.str();
}

}  // namespace nova_grasp_moveit
