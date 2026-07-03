#ifndef ROS_ROBOT_WORKBENCH__MANAGE__SIM_CONTROL_PANEL_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__SIM_CONTROL_PANEL_DATA_MANAGER_HPP_

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class SimControlPanelDataManager : public FeatureDataManagerBase
{
public:
  SimControlPanelDataManager();
  void EnsureDefaults() override;

  void SetControlTopic(const std::string & v);
  std::string GetControlTopic() const;
  void SetWorldName(const std::string & v);
  std::string GetWorldName() const;
  void SetIsaacPython(const std::string & v);
  std::string GetIsaacPython() const;
  void SetBackendDefault(const std::string & v);
  std::string GetBackendDefault() const;
  void SetLastScenePath(const std::string & v);
  std::string GetLastScenePath() const;
};

}  // namespace ros_robot_workbench::manage

#endif
