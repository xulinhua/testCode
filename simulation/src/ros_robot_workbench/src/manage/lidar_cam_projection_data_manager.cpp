#include "ros_robot_workbench/manage/lidar_cam_projection_data_manager.hpp"

namespace ros_robot_workbench::manage
{

LidarCamProjectionDataManager::LidarCamProjectionDataManager()
: FeatureDataManagerBase("lidar_cam_projection.yaml")
{
}

void LidarCamProjectionDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "lidar_cam_projection";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
