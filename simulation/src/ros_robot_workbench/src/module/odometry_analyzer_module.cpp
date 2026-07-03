#include "ros_robot_workbench/module/odometry_analyzer_module.h"

namespace ros_robot_workbench::ui
{

QString OdometryAnalyzerModuleSummary()
{
  return QStringLiteral("odom / TF / GPS 对比与漂移评估。");
}

}  // namespace ros_robot_workbench::ui
