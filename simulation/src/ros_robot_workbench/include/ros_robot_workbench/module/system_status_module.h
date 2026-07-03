#ifndef ROS_ROBOT_WORKBENCH__MODULE__SYSTEM_STATUS_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__SYSTEM_STATUS_MODULE_H_

#include <QString>
#include <atomic>
#include <chrono>
#include <vector>

#include "ros_robot_workbench/ui/ui_data_structs.h"

namespace ros_robot_workbench::ui
{

bool ReadCpuCounters(unsigned long long & idle_all, unsigned long long & total_all);
bool ReadNetworkBytes(unsigned long long & rx_bytes, unsigned long long & tx_bytes);
bool ReadDiskBytes(unsigned long long & read_bytes, unsigned long long & write_bytes);
std::vector<ProcessRow> ReadTopProcessRows();
std::vector<QString> ReadRos2NodeRows();
std::vector<NodeInfoRow> ReadRos2NodeInfoRows();
std::vector<QString> ReadRos2SimpleList(const QString & subcommand);
std::vector<TopicTypeRow> ReadRos2TopicTypeRows();
std::vector<ParamRow> ReadRos2ParamRows(
  const QString & node_name, const std::atomic_bool * cancel_flag = nullptr,
  int timeout_ms = 1500);
ResourceUsage ReadResourceUsage(
  unsigned long long & prev_idle, unsigned long long & prev_total,
  unsigned long long & prev_rx, unsigned long long & prev_tx,
  unsigned long long & prev_disk_read, unsigned long long & prev_disk_write,
  const std::chrono::steady_clock::time_point & last_time,
  std::chrono::steady_clock::time_point & now_time);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__SYSTEM_STATUS_MODULE_H_
