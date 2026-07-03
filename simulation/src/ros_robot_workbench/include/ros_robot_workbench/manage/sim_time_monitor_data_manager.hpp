#ifndef ROS_ROBOT_WORKBENCH__MANAGE__SIM_TIME_MONITOR_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__SIM_TIME_MONITOR_DATA_MANAGER_HPP_

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class SimTimeMonitorDataManager : public FeatureDataManagerBase
{
public:
  SimTimeMonitorDataManager();
  void EnsureDefaults() override;

  void SetClockTopic(const std::string & v);
  std::string GetClockTopic() const;
  void SetRefreshHz(int v);
  int GetRefreshHz() const;
};

}  // namespace ros_robot_workbench::manage

#endif
