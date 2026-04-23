#include "ros_robot_assist_tools/module/multi_sensor_calibration_module.h"

namespace ros_robot_assist_tools::ui
{

std::vector<std::pair<QString, QString>> MultiSensorOnlineFieldDefs()
{
  return {
    {"secondary_topic", "/lidar/points"},
    {"third_topic", "/imu/data"},
  };
}

std::vector<std::pair<QString, QString>> MultiSensorOfflineFieldDefs()
{
  return {
    {"secondary_topic", "/lidar/points"},
    {"third_topic", "/imu/data"},
  };
}

}  // namespace ros_robot_assist_tools::ui
