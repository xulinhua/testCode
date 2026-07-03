#include "ros_robot_workbench/module/sim_time_monitor_module.h"

namespace ros_robot_workbench::ui
{

QString SimTimeMonitorModuleSummary()
{
  return QStringLiteral(
    "监视 /clock 仿真时间、墙钟时间与 real-time factor（RTF）。"
    "适用于 Gazebo、Isaac ROS2 bridge 等发布 clock 的仿真器。");
}

SimTimeSnapshot ComputeSimTimeSnapshot(
  double prev_sim_sec,
  double prev_wall_sec,
  double cur_sim_sec,
  double cur_wall_sec,
  bool had_prev)
{
  SimTimeSnapshot out;
  out.sim_sec = cur_sim_sec;
  out.wall_sec = cur_wall_sec;
  out.has_clock = true;
  if (had_prev) {
    const double ds = cur_sim_sec - prev_sim_sec;
    const double dw = cur_wall_sec - prev_wall_sec;
    if (dw > 1e-6) {
      out.rtf = ds / dw;
      out.rtf_valid = true;
    }
  }
  return out;
}

QString FormatRtfStatus(double rtf, bool valid)
{
  if (!valid) {
    return QStringLiteral("计算中…");
  }
  if (rtf <= 0.01) {
    return QStringLiteral("暂停/未推进");
  }
  if (rtf < 0.5) {
    return QStringLiteral("偏慢");
  }
  if (rtf > 1.2) {
    return QStringLiteral("偏快");
  }
  return QStringLiteral("正常");
}

}  // namespace ros_robot_workbench::ui
