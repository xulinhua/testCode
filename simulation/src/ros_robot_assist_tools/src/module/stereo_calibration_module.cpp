#include "ros_robot_assist_tools/module/stereo_calibration_module.h"

namespace ros_robot_assist_tools::ui
{

std::vector<std::pair<QString, QString>> StereoOnlineFieldDefs()
{
  return {
    {"right_image_topic", "/camera_right/image_raw"},
  };
}

std::vector<std::pair<QString, QString>> StereoOfflineFieldDefs()
{
  return {
    {"right_image_topic", "/camera_right/image_raw"},
  };
}

}  // namespace ros_robot_assist_tools::ui
