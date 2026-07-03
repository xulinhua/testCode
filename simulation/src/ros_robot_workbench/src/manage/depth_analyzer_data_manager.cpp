#include "ros_robot_workbench/manage/depth_analyzer_data_manager.hpp"

namespace ros_robot_workbench::manage
{

DepthAnalyzerDataManager::DepthAnalyzerDataManager()
: FeatureDataManagerBase("depth_analyzer.yaml")
{
}

void DepthAnalyzerDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "depth_analyzer";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
