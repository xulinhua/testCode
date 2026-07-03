#include "ros_robot_workbench/manage/path_compare_data_manager.hpp"

namespace ros_robot_workbench::manage
{

PathCompareDataManager::PathCompareDataManager()
: FeatureDataManagerBase("path_compare.yaml")
{
}

void PathCompareDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "path_compare";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
