#pragma once

#include <string>
#include <vector>

#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nova_grasp_moveit
{

/// 一次抓取的完整腕部路点和执行策略。
///
/// 所有 Pose 均位于 frame_id；除 finger_grasp_target 外，Pose 已从指尖 TCP
/// 换算为 MoveIt 跟踪的 J*_6 腕部目标。
struct GraspPlan
{
  /// 0=左侧 J1/l_arm，1=右侧 J2/r_arm。
  int arm_id{0};
  /// 路点公共参考系，执行前通常已统一为 base_link。
  std::string frame_id{"base_link"};
  /// GraspNet 等外部位姿已生成完整路点；executor 不得再用盒子顶抓规则覆盖。
  bool preserve_waypoints{false};
  /// 可选：先在当前 XY 抬高（执行时由当前 EE 填充）
  geometry_msgs::msg::Pose raise;
  bool has_raise{false};
  /// 当前高度已够则跳过竖直抬高，只做水平移动
  bool need_vertical_raise{false};
  /// 接近前安全路点。
  geometry_msgs::msg::Pose pregrasp;
  /// 盒子上方旋转后再下降（默认绕竖直轴 +90°）
  geometry_msgs::msg::Pose reorient;
  bool has_reorient{true};
  /// 夹爪闭合前的最终腕部目标。
  geometry_msgs::msg::Pose grasp;
  /// 闭合后的抬升腕部目标。
  geometry_msgs::msg::Pose lift;
  /// 指尖目标（盒心+偏移），全程不变；路点 Z 由此 + TCP 换算成腕部
  geometry_msgs::msg::Point finger_grasp_target;
  bool has_finger_grasp_target{false};
  /// 本计划采用的 TCP 偏移，写入日志便于核对高度。
  double path_tcp_z_offset{0.20};
};

/// 路点几何、夹爪开口和选臂参数；由 YAML 和 UI 共同更新。
struct GraspPlannerConfig
{
  /// 盒子路径按目标 X 选臂：x >= split 时使用 J2。
  double arm_split_x{0.40};
  /// 相对抓取高度的预抓取抬高；执行时还会与最低通过高度取 max
  double pregrasp_z_offset{0.15};
  double lift_z_offset{0.15};      ///< 闭合后沿 base_link +Z 抬升距离。
  double box_grasp_z_offset{0.0};  ///< /box_pose 中心的额外 Z 修正。
  /// 腕部 J*_6 → 指尖中点沿 EE +Z 的距离 (m)。夹爪约 20cm。
  double ee_tcp_z_offset{0.20};
  /// 水平移动阶段相对抓取点的最低净空 (m)
  double min_approach_clearance{0.15};
  /// 下降前在盒子上方绕竖直轴的绝对偏航 (deg)；approach 固定 yaw=0，再转到此角
  double grasp_yaw_offset_deg{90.0};
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

/// 根据 base_link 下目标 X 和分界值选择 J1/J2；仅用于盒子抓取。
int choose_arm_id(double target_x, double split_x = 0.53);

/// 将盒子位姿变为指尖抓取目标：使用位置和 box_grasp_z_offset，忽略盒子姿态。
geometry_msgs::msg::Pose box_pose_to_grasp_pose(
  const geometry_msgs::msg::Pose & box,
  const GraspPlannerConfig & cfg);

/// 创建尚未填充过渡路点的基础计划，并按 X 选择机械臂。
GraspPlan plan_grasp_from_pose(
  const geometry_msgs::msg::Pose & grasp_pose,
  const std::string & frame_id,
  const GraspPlannerConfig & cfg);

/// 将指尖 TCP 目标换算为 MoveIt 跟踪的 J*_6 腕部目标。
void apply_tcp_to_wrist(geometry_msgs::msg::Pose & pose, double tcp_along_ee_z);

/// 为盒子固定顶抓填充 raise/pregrasp/reorient/grasp/lift，并执行 TCP→腕部换算。
void fill_plan_waypoints(
  GraspPlan & plan,
  const GraspPlannerConfig & cfg,
  const geometry_msgs::msg::Pose * current_ee);

/// 格式化 xyz、四元数和 RPY，供日志与 UI 使用。
std::string format_pose_xyz_q(const geometry_msgs::msg::Pose & pose);
/// 返回一行计划摘要。
std::string format_plan_summary(const GraspPlan & plan);
/// 每行一个关键点，前缀 [PATH]，供 UI 蓝色显示
std::vector<std::string> format_plan_path_lines(const GraspPlan & plan);

}  // namespace nova_grasp_moveit
