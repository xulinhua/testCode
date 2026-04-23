#include "ros_robot_assist_tools/module/kinematics_solver_module.h"

namespace ros_robot_assist_tools::ui
{

KinematicsSolveResult SolveKinematicsPlaceholder()
{
  KinematicsSolveResult result;
  result.ok = false;
  result.message = "未实现";
  return result;
}

}  // namespace ros_robot_assist_tools::ui
