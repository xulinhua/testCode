#include "ros_robot_workbench/manage/wheel_calib_data_manager.hpp"

namespace ros_robot_workbench::manage
{

WheelCalibDataManager::WheelCalibDataManager()
: FeatureDataManagerBase("wheel_calib.yaml")
{
}

void WheelCalibDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "wheel_calib";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
