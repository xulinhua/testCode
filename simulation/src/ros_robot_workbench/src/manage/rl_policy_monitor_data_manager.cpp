#include "ros_robot_workbench/manage/rl_policy_monitor_data_manager.hpp"

namespace ros_robot_workbench::manage
{

RlPolicyMonitorDataManager::RlPolicyMonitorDataManager()
: FeatureDataManagerBase("rl_policy_monitor.yaml")
{
}

void RlPolicyMonitorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "rl_policy_monitor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
