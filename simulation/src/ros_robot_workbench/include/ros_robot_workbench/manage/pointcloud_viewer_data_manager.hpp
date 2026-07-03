#ifndef ROS_ROBOT_WORKBENCH__MANAGE__POINTCLOUD_VIEWER_DATA_MANAGER_HPP
#define ROS_ROBOT_WORKBENCH__MANAGE__POINTCLOUD_VIEWER_DATA_MANAGER_HPP

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class PointcloudViewerDataManager : public FeatureDataManagerBase
{
public:
  PointcloudViewerDataManager();
  void EnsureDefaults() override;
};

}  // namespace ros_robot_workbench::manage

#endif
