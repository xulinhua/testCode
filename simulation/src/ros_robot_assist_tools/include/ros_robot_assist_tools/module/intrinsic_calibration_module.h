#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__INTRINSIC_CALIBRATION_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__INTRINSIC_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_assist_tools::ui
{

std::vector<std::pair<QString, QString>> IntrinsicOnlineFieldDefs();
std::vector<std::pair<QString, QString>> IntrinsicOfflineFieldDefs();

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__INTRINSIC_CALIBRATION_MODULE_H_
