#include "ros_robot_assist_tools/module/handeye_calibration_module.h"

namespace ros_robot_assist_tools::ui
{

QString HandeyeSetupModeToYamlString(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? "eye_in_hand" : "eye_to_hand";
}

HandeyeSetupMode HandeyeSetupModeFromYamlString(const QString & value)
{
  const QString v = value.trimmed().toLower();
  if (v == "eye_to_hand" || v == "eye-to-hand" || v == "hand_to_eye" || v.contains("手外")) {
    return HandeyeSetupMode::EyeToHand;
  }
  return HandeyeSetupMode::EyeInHand;
}

QString HandeyeThirdFrameYamlKey(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? "camera_frame" : "object_frame";
}

QString HandeyeThirdFrameFieldLabel(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? QStringLiteral("camera_frame:") : QStringLiteral("object_frame (标定目标):");
}

}  // namespace ros_robot_assist_tools::ui
