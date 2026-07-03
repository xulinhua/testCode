#include "ros_robot_workbench/manage/humanoid_joint_monitor_data_manager.hpp"

namespace ros_robot_workbench::manage
{

HumanoidJointMonitorDataManager::HumanoidJointMonitorDataManager()
: FeatureDataManagerBase("humanoid_joint_monitor.yaml")
{
}

void HumanoidJointMonitorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "humanoid_joint_monitor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
