#include "ros_robot_workbench/manage/odometry_analyzer_data_manager.hpp"

namespace ros_robot_workbench::manage
{

OdometryAnalyzerDataManager::OdometryAnalyzerDataManager()
: FeatureDataManagerBase("odometry_analyzer.yaml")
{
}

void OdometryAnalyzerDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "odometry_analyzer";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
