#ifndef ROS_ROBOT_WORKBENCH__MANAGE__ROSBAG_WORKBENCH_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__ROSBAG_WORKBENCH_DATA_MANAGER_HPP_

#include <string>
#include <vector>

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class RosbagWorkbenchDataManager : public FeatureDataManagerBase
{
public:
  RosbagWorkbenchDataManager();
  void EnsureDefaults() override;

  void SetOutputDir(const std::string & v);
  std::string GetOutputDir() const;
  void SetDefaultPrefix(const std::string & v);
  std::string GetDefaultPrefix() const;
  void SetRecordTopics(const std::vector<std::string> & v);
  std::vector<std::string> GetRecordTopics() const;
  void SetPlayRate(double v);
  double GetPlayRate() const;
  void SetPlayLoop(bool v);
  bool GetPlayLoop() const;
  void SetUseSimTime(bool v);
  bool GetUseSimTime() const;
};

}  // namespace ros_robot_workbench::manage

#endif
