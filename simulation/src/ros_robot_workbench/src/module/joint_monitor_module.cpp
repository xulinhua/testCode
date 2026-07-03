#include "ros_robot_workbench/module/joint_monitor_module.h"

namespace ros_robot_workbench::ui
{

QString JointMonitorModuleSummary()
{
  return QStringLiteral("订阅 joint_states，表格与曲线显示。");
}

}  // namespace ros_robot_workbench::ui
