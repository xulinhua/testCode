#ifndef ROS_ROBOT_WORKBENCH__MODULE__ROSBAG_WORKBENCH_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__ROSBAG_WORKBENCH_MODULE_H_

#include <cstdint>
#include <QString>
#include <vector>

namespace ros_robot_workbench::ui
{

struct BagTopicInfo
{
  QString name;
  QString type;
  int64_t message_count = 0;
};

struct BagMetadata
{
  QString bag_dir;
  double duration_sec = 0.0;
  uint64_t starting_time_ns = 0;
  std::vector<BagTopicInfo> topics;
};

QString RosbagWorkbenchModuleSummary();

bool ParseBagMetadata(const QString & bag_dir, BagMetadata * meta, QString * err_msg);

QString BuildBagRecordCommand(const QString & output_uri, const std::vector<QString> & topics);

QString BuildBagPlayCommand(
  const QString & bag_dir, double rate, bool loop_play, bool use_sim_time);

QString DefaultBagOutputUri(const QString & output_dir, const QString & prefix);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__ROSBAG_WORKBENCH_MODULE_H_
