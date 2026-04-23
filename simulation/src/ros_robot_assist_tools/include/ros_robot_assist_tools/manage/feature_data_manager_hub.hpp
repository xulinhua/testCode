#ifndef ROS_ROBOT_ASSIST_TOOLS__MANAGE__FEATURE_DATA_MANAGER_HUB_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__MANAGE__FEATURE_DATA_MANAGER_HUB_HPP_

#include <string>

#include "ros_robot_assist_tools/manage/board_generator_data_manager.hpp"
#include "ros_robot_assist_tools/manage/handeye_calibration_data_manager.hpp"
#include "ros_robot_assist_tools/manage/intrinsic_calibration_data_manager.hpp"
#include "ros_robot_assist_tools/manage/kinematics_solver_data_manager.hpp"
#include "ros_robot_assist_tools/manage/multi_sensor_calibration_data_manager.hpp"
#include "ros_robot_assist_tools/manage/pose_transform_data_manager.hpp"
#include "ros_robot_assist_tools/manage/stereo_calibration_data_manager.hpp"
#include "ros_robot_assist_tools/manage/system_status_data_manager.hpp"
#include "ros_robot_assist_tools/manage/tf_viewer_data_manager.hpp"

namespace ros_robot_assist_tools::manage
{

class FeatureDataManagerHub
{
public:
  void SetConfigRoot(const std::string & config_root);
  bool EnsureAllConfigFiles() const;
  bool LoadAll();
  bool SaveAll() const;

  SystemStatusDataManager system_status;
  BoardGeneratorDataManager board_generator;
  PoseTransformDataManager pose_transform;
  KinematicsSolverDataManager kinematics_solver;
  TfViewerDataManager tf_viewer;
  IntrinsicCalibrationDataManager intrinsic_calibration;
  StereoCalibrationDataManager stereo_calibration;
  MultiSensorCalibrationDataManager multi_sensor_calibration;
  HandeyeCalibrationDataManager handeye_calibration;
};

}  // namespace ros_robot_assist_tools::manage

#endif  // ROS_ROBOT_ASSIST_TOOLS__MANAGE__FEATURE_DATA_MANAGER_HUB_HPP_
