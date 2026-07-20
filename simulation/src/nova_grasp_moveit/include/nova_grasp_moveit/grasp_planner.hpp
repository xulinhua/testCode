#pragma once

#include <string>
#include <vector>

#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nova_grasp_moveit
{

struct GraspPlan
{
  int arm_id{0};
  std::string frame_id{"base_link"};
  /// 可选：先在当前 XY 抬高（执行时由当前 EE 填充）
  geometry_msgs::msg::Pose raise;
  bool has_raise{false};
  /// 当前高度已够则跳过竖直抬高，只做水平移动
  bool need_vertical_raise{false};
  /// 当前过高时跳过「原位下降」，改在 pregrasp 一步到盒子上方
  bool need_descend{false};
  geometry_msgs::msg::Pose descend;
  geometry_msgs::msg::Pose pregrasp;
  /// 盒子上方旋转后再下降（默认绕竖直轴 +90°）
  geometry_msgs::msg::Pose reorient;
  bool has_reorient{true};
  geometry_msgs::msg::Pose grasp;
  geometry_msgs::msg::Pose lift;
  /// 指尖目标（盒心+偏移），全程不变；路点 Z 由此 + TCP 换算成腕部
  geometry_msgs::msg::Point finger_grasp_target;
  bool has_finger_grasp_target{false};
  double path_tcp_z_offset{0.20};
};

struct GraspPlannerConfig
{
  double arm_split_x{0.40};
  /// 相对抓取高度的预抓取抬高；执行时还会与最低通过高度取 max
  double pregrasp_z_offset{0.15};
  double lift_z_offset{0.15};
  double box_grasp_z_offset{0.0};
  /// 腕部 J*_6 → 指尖中点沿 EE +Z 的距离 (m)。夹爪约 20cm。
  double ee_tcp_z_offset{0.20};
  /// 执行开头：相对当前 EE 再抬高的最小量 (m)
  double approach_raise_clearance{0.08};
  /// 水平移动阶段相对抓取点的最低净空 (m)
  double min_approach_clearance{0.15};
  /// 下降前在盒子上方绕竖直轴的绝对偏航 (deg)；approach 固定 yaw=0，再转到此角
  double grasp_yaw_offset_deg{90.0};
  bool use_top_down_if_identity{true};
  bool use_box_yaw_only{false};
  /// 夹爪开口量 (m)：0=完全闭合，约 0.08=完全张开（对指对称命令）
  double gripper_open_m{0.08};
  double gripper_close_m{0.02};
};

/// 将开口量映射为对指 prismatic 关节目标 (j7, j8)，单位 m。
///
/// Isaac/PhysX 导入后两指常为同向 +Z；若 J7/J8 同号增减会整侧平移。
/// 因此张开时两指走**相反**极限：J7→lower、J8→upper（对中开合）。
///   J*_7: [-0.04, 0.02]，J*_8: [-0.02, 0.04]
inline void gripper_opening_to_joint_pair(double opening_m, double & j7, double & j8)
{
  constexpr double kMaxOpening = 0.08;
  // 开：两指远离中心；闭：两指靠近中心（关节符号相反）
  constexpr double kCloseJ7 = -0.04;
  constexpr double kOpenJ7 = 0.02;
  constexpr double kCloseJ8 = 0.04;
  constexpr double kOpenJ8 = -0.02;
  const double t = std::clamp(opening_m, 0.0, kMaxOpening) / kMaxOpening;
  j7 = kCloseJ7 + t * (kOpenJ7 - kCloseJ7);
  j8 = kCloseJ8 + t * (kOpenJ8 - kCloseJ8);
}

/// 由对指关节位置反推开口量 (m)。
inline double gripper_joint_pair_to_opening(double j7, double j8)
{
  constexpr double kMaxOpening = 0.08;
  constexpr double kCloseJ7 = -0.04;
  constexpr double kOpenJ7 = 0.02;
  constexpr double kCloseJ8 = 0.04;
  constexpr double kOpenJ8 = -0.02;
  const double t7 = (j7 - kCloseJ7) / (kOpenJ7 - kCloseJ7);
  const double t8 = (j8 - kCloseJ8) / (kOpenJ8 - kCloseJ8);
  const double t = std::clamp(0.5 * (t7 + t8), 0.0, 1.0);
  return t * kMaxOpening;
}

int choose_arm_id(double target_x, double split_x = 0.53);

geometry_msgs::msg::Pose box_pose_to_grasp_pose(
  const geometry_msgs::msg::Pose & box,
  const GraspPlannerConfig & cfg);

GraspPlan plan_grasp_from_pose(
  const geometry_msgs::msg::Pose & grasp_pose,
  const std::string & frame_id,
  const GraspPlannerConfig & cfg);

/// 用当前 EE 姿态填充 raise / pregrasp / grasp / lift（只改位置，姿态锁定为 keep_q）
void fill_plan_waypoints(
  GraspPlan & plan,
  const GraspPlannerConfig & cfg,
  const geometry_msgs::msg::Pose * current_ee);

std::string format_pose_xyz_q(const geometry_msgs::msg::Pose & pose);
std::string format_plan_summary(const GraspPlan & plan);
/// 每行一个关键点，前缀 [PATH]，供 UI 蓝色显示
std::vector<std::string> format_plan_path_lines(const GraspPlan & plan);

}  // namespace nova_grasp_moveit
