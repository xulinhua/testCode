#ifndef ROS_ROBOT_WORKBENCH__MODULE__SIM_TIME_MONITOR_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__SIM_TIME_MONITOR_MODULE_H_

#include <QString>

namespace ros_robot_workbench::ui
{

QString SimTimeMonitorModuleSummary();

struct SimTimeSnapshot
{
  double sim_sec = 0.0;
  double wall_sec = 0.0;
  double rtf = 0.0;
  bool has_clock = false;
  bool rtf_valid = false;
};

SimTimeSnapshot ComputeSimTimeSnapshot(
  double prev_sim_sec,
  double prev_wall_sec,
  double cur_sim_sec,
  double cur_wall_sec,
  bool had_prev);

QString FormatRtfStatus(double rtf, bool valid);

}  // namespace ros_robot_workbench::ui

#endif
