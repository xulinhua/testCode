#include "ros_robot_workbench/manage/kinematics_solver_data_manager.hpp"

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

}  // namespace

KinematicsSolverDataManager::KinematicsSolverDataManager()
: FeatureDataManagerBase("kinematics_solver.yaml")
{
}

void KinematicsSolverDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["arm"]) {
    data_["arm"] = YAML::Node(YAML::NodeType::Map);
  }
  YAML::Node arm = data_["arm"];
  if (!arm["backend"]) {
    arm["backend"] = "kdl";
  }
  if (!arm["urdf_path"]) {
    arm["urdf_path"] = "";
  }
  if (!arm["base_link"]) {
    arm["base_link"] = "base_link";
  }
  if (!arm["tip_link"]) {
    arm["tip_link"] = "tool0";
  }
  if (!arm["moveit_group"]) {
    arm["moveit_group"] = "arm";
  }
  if (!arm["moveit_ik_link"]) {
    arm["moveit_ik_link"] = "tool0";
  }
  if (!arm["moveit_service"]) {
    arm["moveit_service"] = "/compute_ik";
  }
  if (!arm["moveit_frame_id"]) {
    arm["moveit_frame_id"] = "base_link";
  }
  if (!arm["moveit_node_name"]) {
    arm["moveit_node_name"] = "/move_group";
  }
  if (!arm["moveit_seed_text"]) {
    arm["moveit_seed_text"] = "";
  }
  if (!data_["diff"]) {
    data_["diff"] = YAML::Node(YAML::NodeType::Map);
  }
  YAML::Node di = data_["diff"];
  if (!di["track_m"]) {
    di["track_m"] = 0.3;
  }
  if (!di["wheel_radius_m"]) {
    di["wheel_radius_m"] = 0.05;
  }
  if (!data_["ack"]) {
    data_["ack"] = YAML::Node(YAML::NodeType::Map);
  }
  YAML::Node ac = data_["ack"];
  if (!ac["wheelbase_m"]) {
    ac["wheelbase_m"] = 1.0;
  }
}

void KinematicsSolverDataManager::SetArmBackend(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["backend"] = v;
}

std::string KinematicsSolverDataManager::GetArmBackend() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "backend", "kdl");
}

void KinematicsSolverDataManager::SetArmUrdfPath(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["urdf_path"] = v;
}

std::string KinematicsSolverDataManager::GetArmUrdfPath() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "urdf_path", "");
}

void KinematicsSolverDataManager::SetArmBaseLink(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["base_link"] = v;
}

std::string KinematicsSolverDataManager::GetArmBaseLink() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "base_link", "base_link");
}

void KinematicsSolverDataManager::SetArmTipLink(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["tip_link"] = v;
}

std::string KinematicsSolverDataManager::GetArmTipLink() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "tip_link", "tool0");
}

void KinematicsSolverDataManager::SetMoveitGroup(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_group"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitGroup() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_group", "arm");
}

void KinematicsSolverDataManager::SetMoveitIkLink(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_ik_link"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitIkLink() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_ik_link", "tool0");
}

void KinematicsSolverDataManager::SetMoveitService(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_service"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitService() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_service", "/compute_ik");
}

void KinematicsSolverDataManager::SetMoveitFrameId(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_frame_id"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitFrameId() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_frame_id", "base_link");
}

void KinematicsSolverDataManager::SetMoveitNodeName(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_node_name"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitNodeName() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_node_name", "/move_group");
}

void KinematicsSolverDataManager::SetMoveitSeedText(const std::string & v)
{
  EnsureDefaults();
  data_["arm"]["moveit_seed_text"] = v;
}

std::string KinematicsSolverDataManager::GetMoveitSeedText() const
{
  return GetStr(const_cast<YAML::Node &>(data_)["arm"], "moveit_seed_text", "");
}

void KinematicsSolverDataManager::SetDiffTrackM(double v)
{
  EnsureDefaults();
  data_["diff"]["track_m"] = v;
}

double KinematicsSolverDataManager::GetDiffTrackM() const
{
  return GetD(const_cast<YAML::Node &>(data_)["diff"], "track_m", 0.3);
}

void KinematicsSolverDataManager::SetDiffWheelRadiusM(double v)
{
  EnsureDefaults();
  data_["diff"]["wheel_radius_m"] = v;
}

double KinematicsSolverDataManager::GetDiffWheelRadiusM() const
{
  return GetD(const_cast<YAML::Node &>(data_)["diff"], "wheel_radius_m", 0.05);
}

void KinematicsSolverDataManager::SetAckWheelbaseM(double v)
{
  EnsureDefaults();
  data_["ack"]["wheelbase_m"] = v;
}

double KinematicsSolverDataManager::GetAckWheelbaseM() const
{
  return GetD(const_cast<YAML::Node &>(data_)["ack"], "wheelbase_m", 1.0);
}

}  // namespace ros_robot_workbench::manage
