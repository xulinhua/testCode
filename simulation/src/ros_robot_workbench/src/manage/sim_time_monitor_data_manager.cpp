#include "ros_robot_workbench/manage/sim_time_monitor_data_manager.hpp"

namespace ros_robot_workbench::manage
{
namespace
{

std::string GetStr(const YAML::Node & n, const char * k, const std::string & d)
{
  if (n && n[k] && n[k].IsScalar()) {
    return n[k].as<std::string>();
  }
  return d;
}

int GetI(const YAML::Node & n, const char * k, int d)
{
  if (n && n[k] && n[k].IsScalar()) {
    return n[k].as<int>();
  }
  return d;
}

}  // namespace

SimTimeMonitorDataManager::SimTimeMonitorDataManager()
: FeatureDataManagerBase("sim_time_monitor.yaml")
{
}

void SimTimeMonitorDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "sim_time_monitor";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
  if (!data_["clock_topic"]) {
    data_["clock_topic"] = "/clock";
  }
  if (!data_["refresh_hz"]) {
    data_["refresh_hz"] = 2;
  }
}

void SimTimeMonitorDataManager::SetClockTopic(const std::string & v)
{
  EnsureDefaults();
  data_["clock_topic"] = v;
}

std::string SimTimeMonitorDataManager::GetClockTopic() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "clock_topic", "/clock");
}

void SimTimeMonitorDataManager::SetRefreshHz(int v)
{
  EnsureDefaults();
  data_["refresh_hz"] = v;
}

int SimTimeMonitorDataManager::GetRefreshHz() const
{
  return GetI(const_cast<YAML::Node &>(data_), "refresh_hz", 2);
}

}  // namespace ros_robot_workbench::manage
