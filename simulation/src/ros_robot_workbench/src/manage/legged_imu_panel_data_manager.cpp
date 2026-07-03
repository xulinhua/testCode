#include "ros_robot_workbench/manage/legged_imu_panel_data_manager.hpp"

namespace ros_robot_workbench::manage
{

LeggedImuPanelDataManager::LeggedImuPanelDataManager()
: FeatureDataManagerBase("legged_imu_panel.yaml")
{
}

void LeggedImuPanelDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "legged_imu_panel";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
