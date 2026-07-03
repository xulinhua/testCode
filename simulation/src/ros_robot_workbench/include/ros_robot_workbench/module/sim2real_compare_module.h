#ifndef ROS_ROBOT_WORKBENCH__MODULE__SIM2REAL_COMPARE_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__SIM2REAL_COMPARE_MODULE_H_

#include <QString>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>

namespace ros_robot_workbench::ui
{

enum class Sim2RealField { Position = 0, Velocity = 1 };

struct JointCompareRow
{
  QString name;
  double sim_val = 0.0;
  double real_val = 0.0;
  double error = 0.0;
  bool has_sim = false;
  bool has_real = false;
};

struct Sim2RealStats
{
  double rmse = 0.0;
  double max_abs_error = 0.0;
  int matched = 0;
};

QString Sim2realCompareModuleSummary();

std::vector<JointCompareRow> CompareJointStates(
  const sensor_msgs::msg::JointState & sim,
  const sensor_msgs::msg::JointState & real,
  Sim2RealField field);

Sim2RealStats ComputeSim2RealStats(const std::vector<JointCompareRow> & rows);

QString Sim2RealFieldLabel(Sim2RealField field);

}  // namespace ros_robot_workbench::ui

#endif
