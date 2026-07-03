#include "ros_robot_workbench/manage/balance_panel_data_manager.hpp"

namespace ros_robot_workbench::manage
{

BalancePanelDataManager::BalancePanelDataManager()
: FeatureDataManagerBase("balance_panel.yaml")
{
}

void BalancePanelDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "balance_panel";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
