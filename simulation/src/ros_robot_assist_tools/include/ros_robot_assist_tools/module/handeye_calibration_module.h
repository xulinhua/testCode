#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__HANDEYE_CALIBRATION_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__HANDEYE_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_assist_tools::ui
{

/// 眼在手上：相机装在末端；眼在手外：相机相对基座固定，标定目标随末端运动。
enum class HandeyeSetupMode { EyeInHand = 0, EyeToHand = 1 };

QString HandeyeSetupModeToYamlString(HandeyeSetupMode mode);
HandeyeSetupMode HandeyeSetupModeFromYamlString(const QString & value);

/// 第三列坐标系在 YAML 中的键名（眼在手上为 camera_frame，眼在手外为 object_frame 标定板/目标）。
QString HandeyeThirdFrameYamlKey(HandeyeSetupMode mode);
QString HandeyeThirdFrameFieldLabel(HandeyeSetupMode mode);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__HANDEYE_CALIBRATION_MODULE_H_
