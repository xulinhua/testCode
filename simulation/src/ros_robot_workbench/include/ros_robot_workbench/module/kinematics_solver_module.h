#ifndef ROS_ROBOT_WORKBENCH__MODULE__KINEMATICS_SOLVER_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__KINEMATICS_SOLVER_MODULE_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QString>

#include <rclcpp/rclcpp.hpp>

#include "ros_robot_workbench/kinematics/mobile_kinematics.hpp"
#include "ros_robot_workbench/manage/kinematics_solver_data_manager.hpp"
#include "ros_robot_workbench/workbench_build_config.hpp"

#if WORKBENCH_WITH_KDL
#include "ros_robot_workbench/kinematics/arm_kinematics_kdl.hpp"
#endif
#if WORKBENCH_WITH_MOVEIT
#include "ros_robot_workbench/kinematics/arm_kinematics_moveit.hpp"
#endif

namespace ros_robot_workbench::ui
{

struct KinematicsSolveResult
{
  bool ok{false};
  QString message;
};

#if WORKBENCH_WITH_KDL
struct ArmKdlCache
{
  std::string key;
  std::unique_ptr<kinematics::ArmKinematicsKdl> kdl;
};

KinematicsSolveResult KdlLoadArm(
  ArmKdlCache & cache, const std::string & urdf, const std::string & base, const std::string & tip);

KinematicsSolveResult KdlForward(ArmKdlCache & cache, const std::vector<double> & q);

KinematicsSolveResult KdlInverse(
  ArmKdlCache & cache, const std::vector<double> & seed, const std::array<double, 3> & p,
  const std::array<double, 4> & q_xyzw);
#endif

#if WORKBENCH_WITH_MOVEIT
KinematicsSolveResult MoveitInverse(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg,
  const std::array<double, 3> & p, const std::array<double, 4> & q_xyzw);

KinematicsSolveResult ProbeMoveitPlugin(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg);
#endif

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

}  // namespace ros_robot_workbench::ui

#endif
