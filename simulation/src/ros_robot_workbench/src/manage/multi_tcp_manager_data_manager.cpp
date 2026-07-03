#include "ros_robot_workbench/manage/multi_tcp_manager_data_manager.hpp"

namespace ros_robot_workbench::manage
{

MultiTcpManagerDataManager::MultiTcpManagerDataManager()
: FeatureDataManagerBase("multi_tcp_manager.yaml")
{
}

void MultiTcpManagerDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "multi_tcp_manager";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
