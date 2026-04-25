#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QString>

#include <rclcpp/rclcpp.hpp>

#include "ros_robot_assist_tools/kinematics/arm_kinematics_kdl.hpp"
#include "ros_robot_assist_tools/kinematics/arm_kinematics_moveit.hpp"
#include "ros_robot_assist_tools/kinematics/mobile_kinematics.hpp"
#include "ros_robot_assist_tools/manage/kinematics_solver_data_manager.hpp"

namespace ros_robot_assist_tools::ui
{

struct KinematicsSolveResult
{
  bool ok{false};
  QString message;
};

struct ArmKdlCache
{
  std::string key;
  std::unique_ptr<kinematics::ArmKinematicsKdl> kdl;
};

/// 可复用：同一 URDF+链 的 KDL 句柄（避免每点一次重新解析文件）
KinematicsSolveResult KdlLoadArm(
  ArmKdlCache & cache, const std::string & urdf, const std::string & base, const std::string & tip);

KinematicsSolveResult KdlForward(ArmKdlCache & cache, const std::vector<double> & q);

KinematicsSolveResult KdlInverse(
  ArmKdlCache & cache, const std::vector<double> & seed, const std::array<double, 3> & p,
  const std::array<double, 4> & q_xyzw);

KinematicsSolveResult MoveitInverse(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg,
  const std::array<double, 3> & p, const std::array<double, 4> & q_xyzw);

KinematicsSolveResult ProbeMoveitPlugin(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg);

KinematicsSolveResult RunDiffFromWheels(
  const manage::KinematicsSolverDataManager & cfg, double w_l, double w_r);

KinematicsSolveResult RunDiffFromBody(
  const manage::KinematicsSolverDataManager & cfg, double v, double w);

KinematicsSolveResult RunAckGeometry(const manage::KinematicsSolverDataManager & cfg, double delta_deg);
KinematicsSolveResult RunAckForwardBikeVelocity(
  const manage::KinematicsSolverDataManager & cfg, double delta_deg, double v_ref_mps);
KinematicsSolveResult RunAckInverseBike(
  const manage::KinematicsSolverDataManager & cfg, double target_curvature_1_m);
KinematicsSolveResult RunAckInverseBikeVelocity(
  const manage::KinematicsSolverDataManager & cfg, double v_ref_mps, double omega_target_radps);
KinematicsSolveResult RunAckIdealFromInner(
  const manage::KinematicsSolverDataManager & cfg, double track_m, double delta_inner_deg);
KinematicsSolveResult RunAckIdealFromPair(
  const manage::KinematicsSolverDataManager & cfg, double track_m, double delta_inner_deg,
  double delta_outer_deg);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_
