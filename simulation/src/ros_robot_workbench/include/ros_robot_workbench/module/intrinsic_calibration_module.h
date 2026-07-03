#ifndef ROS_ROBOT_WORKBENCH__MODULE__INTRINSIC_CALIBRATION_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__INTRINSIC_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_workbench::ui
{

std::vector<std::pair<QString, QString>> IntrinsicOnlineFieldDefs();
std::vector<std::pair<QString, QString>> IntrinsicOfflineFieldDefs();

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__INTRINSIC_CALIBRATION_MODULE_H_
