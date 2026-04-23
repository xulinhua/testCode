#include "ros_robot_assist_tools/module/handeye_calibration_module.h"

namespace ros_robot_assist_tools::ui
{

std::vector<std::pair<QString, QString>> HandeyeOnlineFieldDefs()
{
  return {
    {"base_frame", "base_link"},
    {"ee_frame", "tool0"},
    {"camera_frame", "camera_link"},
  };
}

std::vector<std::pair<QString, QString>> HandeyeOfflineFieldDefs()
{
  return {
    {"base_frame", "base_link"},
    {"ee_frame", "tool0"},
    {"camera_frame", "camera_link"},
  };
}

}  // namespace ros_robot_assist_tools::ui
