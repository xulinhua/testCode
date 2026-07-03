#include "ros_robot_workbench/manage/obstacle_editor_data_manager.hpp"

namespace ros_robot_workbench::manage
{

ObstacleEditorDataManager::ObstacleEditorDataManager()
: FeatureDataManagerBase("obstacle_editor.yaml")
{
}

void ObstacleEditorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "obstacle_editor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
