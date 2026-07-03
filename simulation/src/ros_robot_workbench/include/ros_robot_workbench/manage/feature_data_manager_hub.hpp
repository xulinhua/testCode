#ifndef ROS_ROBOT_WORKBENCH__MANAGE__FEATURE_DATA_MANAGER_HUB_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__FEATURE_DATA_MANAGER_HUB_HPP_

#include <string>

#include "ros_robot_workbench/manage/system_status_data_manager.hpp"
#include "ros_robot_workbench/workbench_build_config.hpp"

#if WORKBENCH_KIT_GENERAL
#include "ros_robot_workbench/manage/image_viewer_data_manager.hpp"
#endif
#if WORKBENCH_KIT_KINEMATICS
#include "ros_robot_workbench/manage/kinematics_solver_data_manager.hpp"
#include "ros_robot_workbench/manage/pose_transform_data_manager.hpp"
#include "ros_robot_workbench/manage/tf_viewer_data_manager.hpp"
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
#include "ros_robot_workbench/manage/board_generator_data_manager.hpp"
#include "ros_robot_workbench/manage/handeye_calibration_data_manager.hpp"
#endif
#include "ros_robot_workbench/manage/intrinsic_calibration_data_manager.hpp"
#include "ros_robot_workbench/manage/multi_sensor_calibration_data_manager.hpp"
#include "ros_robot_workbench/manage/stereo_calibration_data_manager.hpp"
#include "ros_robot_workbench/manage/tcp_calibration_data_manager.hpp"
#endif

namespace ros_robot_workbench::manage
{

class FeatureDataManagerHub
{
public:
  void SetConfigRoot(const std::string & config_root);
  bool EnsureAllConfigFiles() const;
  bool LoadAll();
  bool SaveAll() const;

  SystemStatusDataManager system_status;
#if WORKBENCH_KIT_GENERAL
  ImageViewerDataManager image_viewer;
#endif
#if WORKBENCH_KIT_KINEMATICS
  PoseTransformDataManager pose_transform;
  KinematicsSolverDataManager kinematics_solver;
  TfViewerDataManager tf_viewer;
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  BoardGeneratorDataManager board_generator;
#endif
  IntrinsicCalibrationDataManager intrinsic_calibration;
  StereoCalibrationDataManager stereo_calibration;
  MultiSensorCalibrationDataManager multi_sensor_calibration;
#if WORKBENCH_WITH_OPENCV
  HandeyeCalibrationDataManager handeye_calibration;
#endif
  TcpCalibrationDataManager tcp_calibration;
#endif
};

}  // namespace ros_robot_workbench::manage

#endif
