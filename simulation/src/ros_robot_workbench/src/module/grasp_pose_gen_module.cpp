#include "ros_robot_workbench/module/grasp_pose_gen_module.h"

namespace ros_robot_workbench::ui
{

QString GraspPoseGenModuleSummary()
{
  return QStringLiteral("基于物体位姿生成候选 grasp pose。");
}

}  // namespace ros_robot_workbench::ui
