#include "ros_robot_workbench/manage/foot_contact_monitor_data_manager.hpp"

namespace ros_robot_workbench::manage
{

FootContactMonitorDataManager::FootContactMonitorDataManager()
: FeatureDataManagerBase("foot_contact_monitor.yaml")
{
}

void FootContactMonitorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "foot_contact_monitor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
