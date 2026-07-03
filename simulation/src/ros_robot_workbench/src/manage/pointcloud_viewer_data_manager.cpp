#include "ros_robot_workbench/manage/pointcloud_viewer_data_manager.hpp"

namespace ros_robot_workbench::manage
{

PointcloudViewerDataManager::PointcloudViewerDataManager()
: FeatureDataManagerBase("pointcloud_viewer.yaml")
{
}

void PointcloudViewerDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "pointcloud_viewer";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
