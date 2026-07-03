#ifndef ROS_ROBOT_ASSIST_TOOLS__MANAGE__TCP_CALIBRATION_DATA_MANAGER_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__MANAGE__TCP_CALIBRATION_DATA_MANAGER_HPP_

#include <string>

#include "ros_robot_assist_tools/manage/feature_data_manager_base.hpp"

namespace ros_robot_assist_tools::manage
{

class TcpCalibrationDataManager : public FeatureDataManagerBase
{
public:
  TcpCalibrationDataManager();

  void EnsureDefaults() override;

  void SetPoseEndpoint(const std::string & v);
  std::string GetPoseEndpoint() const;
  void SetControlEndpoint(const std::string & v);
  std::string GetControlEndpoint() const;
  void SetMode(const std::string & v);
  std::string GetMode() const;
  void SetPoseCsv(const std::string & v);
  std::string GetPoseCsv() const;
  void SetFlangeFrame(const std::string & v);
  std::string GetFlangeFrame() const;
  void SetMinPoses(int v);
  int GetMinPoses() const;
  void SetUnit(const std::string & v);
  std::string GetUnit() const;
  void SetPoseFormat(const std::string & v);
  std::string GetPoseFormat() const;
};

}  // namespace ros_robot_assist_tools::manage

#endif  // ROS_ROBOT_ASSIST_TOOLS__MANAGE__TCP_CALIBRATION_DATA_MANAGER_HPP_
