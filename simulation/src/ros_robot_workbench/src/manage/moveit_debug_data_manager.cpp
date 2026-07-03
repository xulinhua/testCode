#include "ros_robot_workbench/manage/moveit_debug_data_manager.hpp"

namespace ros_robot_workbench::manage
{

MoveitDebugDataManager::MoveitDebugDataManager()
: FeatureDataManagerBase("moveit_debug.yaml")
{
}

void MoveitDebugDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "moveit_debug";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
