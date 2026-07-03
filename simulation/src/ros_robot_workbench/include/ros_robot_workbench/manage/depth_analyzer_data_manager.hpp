#ifndef ROS_ROBOT_WORKBENCH__MANAGE__DEPTH_ANALYZER_DATA_MANAGER_HPP
#define ROS_ROBOT_WORKBENCH__MANAGE__DEPTH_ANALYZER_DATA_MANAGER_HPP

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class DepthAnalyzerDataManager : public FeatureDataManagerBase
{
public:
  DepthAnalyzerDataManager();
  void EnsureDefaults() override;
};

}  // namespace ros_robot_workbench::manage

#endif
