#ifndef ROS_ROBOT_WORKBENCH__MANAGE__DETECTION_OVERLAY_DATA_MANAGER_HPP
#define ROS_ROBOT_WORKBENCH__MANAGE__DETECTION_OVERLAY_DATA_MANAGER_HPP

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class DetectionOverlayDataManager : public FeatureDataManagerBase
{
public:
  DetectionOverlayDataManager();
  void EnsureDefaults() override;
};

}  // namespace ros_robot_workbench::manage

#endif
