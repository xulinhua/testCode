#include "ros_robot_workbench/manage/sim_control_panel_data_manager.hpp"

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

}  // namespace

SimControlPanelDataManager::SimControlPanelDataManager()
: FeatureDataManagerBase("sim_control_panel.yaml")
{
}

void SimControlPanelDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "sim_control_panel";
  }
  if (!data_["enabled"]) {
    data_["enabled"] = true;
  }
  if (!data_["control_topic"]) {
    data_["control_topic"] = "/sim/control";
  }
  if (!data_["clock_topic"]) {
    data_["clock_topic"] = "/clock";
  }
  if (!data_["world_name"]) {
    data_["world_name"] = "default";
  }
  if (!data_["isaac_python"]) {
    data_["isaac_python"] = "";
  }
  if (!data_["backend_default"]) {
    data_["backend_default"] = "ros_topic";
  }
  if (!data_["last_scene_path"]) {
    data_["last_scene_path"] = "";
  }
}

void SimControlPanelDataManager::SetControlTopic(const std::string & v)
{
  EnsureDefaults();
  data_["control_topic"] = v;
}

std::string SimControlPanelDataManager::GetControlTopic() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "control_topic", "/sim/control");
}

void SimControlPanelDataManager::SetWorldName(const std::string & v)
{
  EnsureDefaults();
  data_["world_name"] = v;
}

std::string SimControlPanelDataManager::GetWorldName() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "world_name", "default");
}

void SimControlPanelDataManager::SetIsaacPython(const std::string & v)
{
  EnsureDefaults();
  data_["isaac_python"] = v;
}

std::string SimControlPanelDataManager::GetIsaacPython() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "isaac_python", "");
}

void SimControlPanelDataManager::SetBackendDefault(const std::string & v)
{
  EnsureDefaults();
  data_["backend_default"] = v;
}

std::string SimControlPanelDataManager::GetBackendDefault() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "backend_default", "ros_topic");
}

void SimControlPanelDataManager::SetLastScenePath(const std::string & v)
{
  EnsureDefaults();
  data_["last_scene_path"] = v;
}

std::string SimControlPanelDataManager::GetLastScenePath() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "last_scene_path", "");
}

}  // namespace ros_robot_workbench::manage
