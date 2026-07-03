#include "ros_robot_workbench/module/sim2real_compare_module.h"

#include <cmath>
#include <unordered_map>

namespace ros_robot_workbench::ui
{
namespace
{

double JointFieldValue(const sensor_msgs::msg::JointState & js, size_t idx, Sim2RealField field)
{
  if (field == Sim2RealField::Velocity) {
    return idx < js.velocity.size() ? js.velocity[idx] : 0.0;
  }
  return idx < js.position.size() ? js.position[idx] : 0.0;
}

bool HasField(const sensor_msgs::msg::JointState & js, size_t idx, Sim2RealField field)
{
  if (field == Sim2RealField::Velocity) {
    return idx < js.velocity.size();
  }
  return idx < js.position.size();
}

}  // namespace

QString Sim2realCompareModuleSummary()
{
  return QStringLiteral(
    "对比仿真与实机 JointState（位置/速度）。按关节名对齐，显示误差与 RMSE。"
    "topic 可分别配置，兼容 Isaac/Gazebo 等不同前缀。");
}

QString Sim2RealFieldLabel(Sim2RealField field)
{
  return field == Sim2RealField::Velocity ? QStringLiteral("速度(rad/s)") : QStringLiteral("位置(rad)");
}

std::vector<JointCompareRow> CompareJointStates(
  const sensor_msgs::msg::JointState & sim,
  const sensor_msgs::msg::JointState & real,
  Sim2RealField field)
{
  std::unordered_map<std::string, size_t> sim_idx;
  for (size_t i = 0; i < sim.name.size(); ++i) {
    sim_idx[sim.name[i]] = i;
  }
  std::unordered_map<std::string, size_t> real_idx;
  for (size_t i = 0; i < real.name.size(); ++i) {
    real_idx[real.name[i]] = i;
  }

  std::vector<std::string> names;
  names.reserve(sim.name.size() + real.name.size());
  for (const auto & n : sim.name) {
    names.push_back(n);
  }
  for (const auto & n : real.name) {
    if (sim_idx.find(n) == sim_idx.end()) {
      names.push_back(n);
    }
  }

  std::vector<JointCompareRow> rows;
  rows.reserve(names.size());
  for (const auto & name : names) {
    JointCompareRow row;
    row.name = QString::fromStdString(name);
    const auto si = sim_idx.find(name);
    const auto ri = real_idx.find(name);
    if (si != sim_idx.end()) {
      row.has_sim = HasField(sim, si->second, field);
      row.sim_val = JointFieldValue(sim, si->second, field);
    }
    if (ri != real_idx.end()) {
      row.has_real = HasField(real, ri->second, field);
      row.real_val = JointFieldValue(real, ri->second, field);
    }
    if (row.has_sim && row.has_real) {
      row.error = row.sim_val - row.real_val;
    }
    rows.push_back(row);
  }
  return rows;
}

Sim2RealStats ComputeSim2RealStats(const std::vector<JointCompareRow> & rows)
{
  Sim2RealStats stats;
  double sum_sq = 0.0;
  for (const auto & r : rows) {
    if (!r.has_sim || !r.has_real) {
      continue;
    }
    ++stats.matched;
    const double ae = std::abs(r.error);
    sum_sq += r.error * r.error;
    if (ae > stats.max_abs_error) {
      stats.max_abs_error = ae;
    }
  }
  if (stats.matched > 0) {
    stats.rmse = std::sqrt(sum_sq / static_cast<double>(stats.matched));
  }
  return stats;
}

}  // namespace ros_robot_workbench::ui
