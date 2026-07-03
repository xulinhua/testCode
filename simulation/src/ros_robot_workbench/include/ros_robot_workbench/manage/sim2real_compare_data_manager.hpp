#ifndef ROS_ROBOT_WORKBENCH__MANAGE__SIM2REAL_COMPARE_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__SIM2REAL_COMPARE_DATA_MANAGER_HPP_

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class Sim2realCompareDataManager : public FeatureDataManagerBase
{
public:
  Sim2realCompareDataManager();
  void EnsureDefaults() override;

  void SetSimTopic(const std::string & v);
  std::string GetSimTopic() const;
  void SetRealTopic(const std::string & v);
  std::string GetRealTopic() const;
  void SetCompareField(const std::string & v);
  std::string GetCompareField() const;
  void SetRefreshHz(int v);
  int GetRefreshHz() const;
};

}  // namespace ros_robot_workbench::manage

#endif
