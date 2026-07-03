#include "ros_robot_workbench/manage/feature_data_manager_hub.hpp"

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"
#include "ros_robot_workbench/workbench_build_config.hpp"

namespace ros_robot_workbench::manage
{

void FeatureDataManagerHub::SetConfigRoot(const std::string & config_root)
{
  system_status.SetConfigPath(JoinConfigPath(config_root, "system_status.yaml"));
#if WORKBENCH_KIT_GENERAL
  image_viewer.SetConfigPath(JoinConfigPath(config_root, "image_viewer.yaml"));
#endif
#if WORKBENCH_KIT_KINEMATICS
  pose_transform.SetConfigPath(JoinConfigPath(config_root, "pose_transform.yaml"));
  kinematics_solver.SetConfigPath(JoinConfigPath(config_root, "kinematics_solver.yaml"));
  tf_viewer.SetConfigPath(JoinConfigPath(config_root, "tf_viewer.yaml"));
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  board_generator.SetConfigPath(JoinConfigPath(config_root, "board_generator.yaml"));
#endif
  intrinsic_calibration.SetConfigPath(JoinConfigPath(config_root, "intrinsic_calibration.yaml"));
  stereo_calibration.SetConfigPath(JoinConfigPath(config_root, "stereo_calibration.yaml"));
  multi_sensor_calibration.SetConfigPath(JoinConfigPath(config_root, "multi_sensor_calibration.yaml"));
#if WORKBENCH_WITH_OPENCV
  handeye_calibration.SetConfigPath(JoinConfigPath(config_root, "handeye_calibration.yaml"));
#endif
  tcp_calibration.SetConfigPath(JoinConfigPath(config_root, "tcp_calibration.yaml"));
#endif
}

bool FeatureDataManagerHub::EnsureAllConfigFiles() const
{
  if (!system_status.EnsureFileExists()) {
    return false;
  }
#if WORKBENCH_KIT_GENERAL
  if (!image_viewer.EnsureFileExists()) {
    return false;
  }
#endif
#if WORKBENCH_KIT_KINEMATICS
  if (!pose_transform.EnsureFileExists() || !kinematics_solver.EnsureFileExists() ||
      !tf_viewer.EnsureFileExists())
  {
    return false;
  }
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  if (!board_generator.EnsureFileExists() || !handeye_calibration.EnsureFileExists()) {
    return false;
  }
#endif
  if (!intrinsic_calibration.EnsureFileExists() || !stereo_calibration.EnsureFileExists() ||
      !multi_sensor_calibration.EnsureFileExists() || !tcp_calibration.EnsureFileExists())
  {
    return false;
  }
#endif
  return true;
}

bool FeatureDataManagerHub::LoadAll()
{
  if (!system_status.Load()) {
    return false;
  }
#if WORKBENCH_KIT_GENERAL
  if (!image_viewer.Load()) {
    return false;
  }
#endif
#if WORKBENCH_KIT_KINEMATICS
  if (!pose_transform.Load() || !kinematics_solver.Load() || !tf_viewer.Load()) {
    return false;
  }
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  if (!board_generator.Load() || !handeye_calibration.Load()) {
    return false;
  }
#endif
  if (!intrinsic_calibration.Load() || !stereo_calibration.Load() ||
      !multi_sensor_calibration.Load() || !tcp_calibration.Load())
  {
    return false;
  }
#endif
  return true;
}

bool FeatureDataManagerHub::SaveAll() const
{
  if (!system_status.Save()) {
    return false;
  }
#if WORKBENCH_KIT_GENERAL
  if (!image_viewer.Save()) {
    return false;
  }
#endif
#if WORKBENCH_KIT_KINEMATICS
  if (!pose_transform.Save() || !kinematics_solver.Save() || !tf_viewer.Save()) {
    return false;
  }
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  if (!board_generator.Save() || !handeye_calibration.Save()) {
    return false;
  }
#endif
  if (!intrinsic_calibration.Save() || !stereo_calibration.Save() ||
      !multi_sensor_calibration.Save() || !tcp_calibration.Save())
  {
    return false;
  }
#endif
  return true;
}

}  // namespace ros_robot_workbench::manage
