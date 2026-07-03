#include "ros_robot_workbench/manage/inference_monitor_data_manager.hpp"

namespace ros_robot_workbench::manage
{

InferenceMonitorDataManager::InferenceMonitorDataManager()
: FeatureDataManagerBase("inference_monitor.yaml")
{
}

void InferenceMonitorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "inference_monitor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
