#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_

#include <QString>

namespace ros_robot_assist_tools::ui
{

struct KinematicsSolveResult
{
  bool ok{false};
  QString message;
};

KinematicsSolveResult SolveKinematicsPlaceholder();

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__KINEMATICS_SOLVER_MODULE_H_
