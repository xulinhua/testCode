#include "ros_robot_workbench/manage/nav2_panel_data_manager.hpp"

namespace ros_robot_workbench::manage
{

Nav2PanelDataManager::Nav2PanelDataManager()
: FeatureDataManagerBase("nav2_panel.yaml")
{
}

void Nav2PanelDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "nav2_panel";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
