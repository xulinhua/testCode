#ifndef ROS_ROBOT_WORKBENCH__MODULE__STEREO_CALIBRATION_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__STEREO_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_workbench::ui
{

std::vector<std::pair<QString, QString>> StereoOnlineFieldDefs();
std::vector<std::pair<QString, QString>> StereoOfflineFieldDefs();

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__STEREO_CALIBRATION_MODULE_H_
