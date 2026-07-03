#ifndef ROS_ROBOT_WORKBENCH__MANAGE__MULTI_TCP_MANAGER_DATA_MANAGER_HPP
#define ROS_ROBOT_WORKBENCH__MANAGE__MULTI_TCP_MANAGER_DATA_MANAGER_HPP

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class MultiTcpManagerDataManager : public FeatureDataManagerBase
{
public:
  MultiTcpManagerDataManager();
  void EnsureDefaults() override;
};

}  // namespace ros_robot_workbench::manage

#endif
