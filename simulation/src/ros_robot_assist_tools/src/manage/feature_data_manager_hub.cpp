#include "ros_robot_assist_tools/manage/feature_data_manager_hub.hpp"

#include "ros_robot_assist_tools/manage/feature_data_manager_base.hpp"

namespace ros_robot_assist_tools::manage
{

void FeatureDataManagerHub::SetConfigRoot(const std::string & config_root)
{
  system_status.SetConfigPath(JoinConfigPath(config_root, "system_status.yaml"));
  image_viewer.SetConfigPath(JoinConfigPath(config_root, "image_viewer.yaml"));
  board_generator.SetConfigPath(JoinConfigPath(config_root, "board_generator.yaml"));
  pose_transform.SetConfigPath(JoinConfigPath(config_root, "pose_transform.yaml"));
  kinematics_solver.SetConfigPath(JoinConfigPath(config_root, "kinematics_solver.yaml"));
  tf_viewer.SetConfigPath(JoinConfigPath(config_root, "tf_viewer.yaml"));
  intrinsic_calibration.SetConfigPath(JoinConfigPath(config_root, "intrinsic_calibration.yaml"));
  stereo_calibration.SetConfigPath(JoinConfigPath(config_root, "stereo_calibration.yaml"));
  multi_sensor_calibration.SetConfigPath(JoinConfigPath(config_root, "multi_sensor_calibration.yaml"));
  handeye_calibration.SetConfigPath(JoinConfigPath(config_root, "handeye_calibration.yaml"));
}

bool FeatureDataManagerHub::EnsureAllConfigFiles() const
{
  return system_status.EnsureFileExists() &&
         image_viewer.EnsureFileExists() &&
         board_generator.EnsureFileExists() &&
         pose_transform.EnsureFileExists() &&
         kinematics_solver.EnsureFileExists() &&
         tf_viewer.EnsureFileExists() &&
         intrinsic_calibration.EnsureFileExists() &&
         stereo_calibration.EnsureFileExists() &&
         multi_sensor_calibration.EnsureFileExists() &&
         handeye_calibration.EnsureFileExists();
}

bool FeatureDataManagerHub::LoadAll()
{
  return system_status.Load() &&
         image_viewer.Load() &&
         board_generator.Load() &&
         pose_transform.Load() &&
         kinematics_solver.Load() &&
         tf_viewer.Load() &&
         intrinsic_calibration.Load() &&
         stereo_calibration.Load() &&
         multi_sensor_calibration.Load() &&
         handeye_calibration.Load();
}

bool FeatureDataManagerHub::SaveAll() const
{
  return system_status.Save() &&
         image_viewer.Save() &&
         board_generator.Save() &&
         pose_transform.Save() &&
         kinematics_solver.Save() &&
         tf_viewer.Save() &&
         intrinsic_calibration.Save() &&
         stereo_calibration.Save() &&
         multi_sensor_calibration.Save() &&
         handeye_calibration.Save();
}

}  // namespace ros_robot_assist_tools::manage
