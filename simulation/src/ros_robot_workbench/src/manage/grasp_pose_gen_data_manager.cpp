#include "ros_robot_workbench/manage/grasp_pose_gen_data_manager.hpp"

namespace ros_robot_workbench::manage
{

GraspPoseGenDataManager::GraspPoseGenDataManager()
: FeatureDataManagerBase("grasp_pose_gen.yaml")
{
}

void GraspPoseGenDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "grasp_pose_gen";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
