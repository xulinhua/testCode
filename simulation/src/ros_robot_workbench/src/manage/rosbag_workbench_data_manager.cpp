#include "ros_robot_workbench/manage/rosbag_workbench_data_manager.hpp"

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

double GetD(const YAML::Node & n, const char * k, double d)
{
  if (n && n[k] && n[k].IsScalar()) {
    return n[k].as<double>();
  }
  return d;
}

bool GetB(const YAML::Node & n, const char * k, bool d)
{
  if (n && n[k] && n[k].IsScalar()) {
    return n[k].as<bool>();
  }
  return d;
}

}  // namespace

RosbagWorkbenchDataManager::RosbagWorkbenchDataManager()
: FeatureDataManagerBase("rosbag_workbench.yaml")
{
}

void RosbagWorkbenchDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "rosbag_workbench";
  }
  if (!data_["output_dir"]) {
    data_["output_dir"] = "~/.ros_robot_workbench/bags";
  }
  if (!data_["default_prefix"]) {
    data_["default_prefix"] = "run";
  }
  if (!data_["play_rate"]) {
    data_["play_rate"] = 1.0;
  }
  if (!data_["play_loop"]) {
    data_["play_loop"] = false;
  }
  if (!data_["use_sim_time"]) {
    data_["use_sim_time"] = true;
  }
  if (!data_["record_topics"]) {
    data_["record_topics"] = YAML::Node(YAML::NodeType::Sequence);
    data_["record_topics"].push_back("/camera/image_raw");
    data_["record_topics"].push_back("/joint_states");
  }
}

void RosbagWorkbenchDataManager::SetOutputDir(const std::string & v)
{
  EnsureDefaults();
  data_["output_dir"] = v;
}

std::string RosbagWorkbenchDataManager::GetOutputDir() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "output_dir", "~/.ros_robot_workbench/bags");
}

void RosbagWorkbenchDataManager::SetDefaultPrefix(const std::string & v)
{
  EnsureDefaults();
  data_["default_prefix"] = v;
}

std::string RosbagWorkbenchDataManager::GetDefaultPrefix() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "default_prefix", "run");
}

void RosbagWorkbenchDataManager::SetRecordTopics(const std::vector<std::string> & v)
{
  EnsureDefaults();
  YAML::Node seq(YAML::NodeType::Sequence);
  for (const auto & s : v) {
    seq.push_back(s);
  }
  data_["record_topics"] = seq;
}

std::vector<std::string> RosbagWorkbenchDataManager::GetRecordTopics() const
{
  std::vector<std::string> out;
  const YAML::Node rows = const_cast<YAML::Node &>(data_)["record_topics"];
  if (rows && rows.IsSequence()) {
    for (const auto & n : rows) {
      if (n.IsScalar()) {
        out.push_back(n.as<std::string>());
      }
    }
  }
  return out;
}

void RosbagWorkbenchDataManager::SetPlayRate(double v)
{
  EnsureDefaults();
  data_["play_rate"] = v;
}

double RosbagWorkbenchDataManager::GetPlayRate() const
{
  return GetD(const_cast<YAML::Node &>(data_), "play_rate", 1.0);
}

void RosbagWorkbenchDataManager::SetPlayLoop(bool v)
{
  EnsureDefaults();
  data_["play_loop"] = v;
}

bool RosbagWorkbenchDataManager::GetPlayLoop() const
{
  return GetB(const_cast<YAML::Node &>(data_), "play_loop", false);
}

void RosbagWorkbenchDataManager::SetUseSimTime(bool v)
{
  EnsureDefaults();
  data_["use_sim_time"] = v;
}

bool RosbagWorkbenchDataManager::GetUseSimTime() const
{
  return GetB(const_cast<YAML::Node &>(data_), "use_sim_time", true);
}

}  // namespace ros_robot_workbench::manage
