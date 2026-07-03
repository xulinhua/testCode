#ifndef ROS_ROBOT_WORKBENCH__MANAGE__TOPIC_LAB_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__TOPIC_LAB_DATA_MANAGER_HPP_

#include <string>
#include <vector>

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class TopicLabDataManager : public FeatureDataManagerBase
{
public:
  TopicLabDataManager();
  void EnsureDefaults() override;

  void SetDefaultTopic(const std::string & v);
  std::string GetDefaultTopic() const;
  void SetEchoBufferSize(int v);
  int GetEchoBufferSize() const;
  void SetHzSampleSec(int v);
  int GetHzSampleSec() const;
  void SetFavoriteTopics(const std::vector<std::string> & v);
  std::vector<std::string> GetFavoriteTopics() const;
};

}  // namespace ros_robot_workbench::manage

#endif
