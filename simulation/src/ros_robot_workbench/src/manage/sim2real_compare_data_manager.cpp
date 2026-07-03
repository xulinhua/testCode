#include "ros_robot_workbench/manage/sim2real_compare_data_manager.hpp"

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

Sim2realCompareDataManager::Sim2realCompareDataManager()
: FeatureDataManagerBase("sim2real_compare.yaml")
{
}

void Sim2realCompareDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "sim2real_compare";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
  if (!data_["sim_topic"]) {
    data_["sim_topic"] = "/sim/joint_states";
  }
  if (!data_["real_topic"]) {
    data_["real_topic"] = "/joint_states";
  }
  if (!data_["compare_field"]) {
    data_["compare_field"] = "position";
  }
  if (!data_["refresh_hz"]) {
    data_["refresh_hz"] = 2;
  }
}

void Sim2realCompareDataManager::SetSimTopic(const std::string & v)
{
  EnsureDefaults();
  data_["sim_topic"] = v;
}

std::string Sim2realCompareDataManager::GetSimTopic() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "sim_topic", "/sim/joint_states");
}

void Sim2realCompareDataManager::SetRealTopic(const std::string & v)
{
  EnsureDefaults();
  data_["real_topic"] = v;
}

std::string Sim2realCompareDataManager::GetRealTopic() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "real_topic", "/joint_states");
}

void Sim2realCompareDataManager::SetCompareField(const std::string & v)
{
  EnsureDefaults();
  data_["compare_field"] = v;
}

std::string Sim2realCompareDataManager::GetCompareField() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "compare_field", "position");
}

void Sim2realCompareDataManager::SetRefreshHz(int v)
{
  EnsureDefaults();
  data_["refresh_hz"] = v;
}

int Sim2realCompareDataManager::GetRefreshHz() const
{
  return GetI(const_cast<YAML::Node &>(data_), "refresh_hz", 2);
}

}  // namespace ros_robot_workbench::manage
