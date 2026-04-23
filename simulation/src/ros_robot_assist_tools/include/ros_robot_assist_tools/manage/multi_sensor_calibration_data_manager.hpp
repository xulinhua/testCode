#ifndef ROS_ROBOT_ASSIST_TOOLS__MANAGE__MULTI_SENSOR_CALIBRATION_DATA_MANAGER_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__MANAGE__MULTI_SENSOR_CALIBRATION_DATA_MANAGER_HPP_

#include "ros_robot_assist_tools/manage/feature_data_manager_base.hpp"

namespace ros_robot_assist_tools::manage
{

class MultiSensorCalibrationDataManager : public FeatureDataManagerBase
{
public:
  MultiSensorCalibrationDataManager();
};

}  // namespace ros_robot_assist_tools::manage

#endif  // ROS_ROBOT_ASSIST_TOOLS__MANAGE__MULTI_SENSOR_CALIBRATION_DATA_MANAGER_HPP_
