#include "ros_robot_workbench/manage/detection_overlay_data_manager.hpp"

namespace ros_robot_workbench::manage
{

DetectionOverlayDataManager::DetectionOverlayDataManager()
: FeatureDataManagerBase("detection_overlay.yaml")
{
}

void DetectionOverlayDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "detection_overlay";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
}

}  // namespace ros_robot_workbench::manage
