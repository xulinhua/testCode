#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__POSE_TRANSFORM_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__POSE_TRANSFORM_MODULE_H_

#include <QString>

#include "ros_robot_assist_tools/ui/ui_data_structs.h"

namespace ros_robot_assist_tools::ui
{

Quaternion QuaternionFromEuler(const EulerAngles & euler_rad, const QString & order);
Quaternion NormalizeQuaternion(const Quaternion & q);
EulerAngles EulerFromQuaternionZYX(const Quaternion & q);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__POSE_TRANSFORM_MODULE_H_
