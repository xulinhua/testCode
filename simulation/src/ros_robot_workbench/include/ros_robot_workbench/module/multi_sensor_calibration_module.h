#ifndef ROS_ROBOT_WORKBENCH__MODULE__MULTI_SENSOR_CALIBRATION_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__MULTI_SENSOR_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_workbench::ui
{

std::vector<std::pair<QString, QString>> MultiSensorOnlineFieldDefs();
std::vector<std::pair<QString, QString>> MultiSensorOfflineFieldDefs();

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__MULTI_SENSOR_CALIBRATION_MODULE_H_
