#include "ros_robot_assist_tools/manage/tcp_calibration_data_manager.hpp"

namespace ros_robot_assist_tools::manage
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

TcpCalibrationDataManager::TcpCalibrationDataManager()
: FeatureDataManagerBase("tcp_calibration.yaml")
{
}

void TcpCalibrationDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["pose_endpoint"]) {
    data_["pose_endpoint"] = "/robot_pose";
  }
  if (!data_["control_endpoint"]) {
    data_["control_endpoint"] = "/robot_control";
  }
  if (!data_["mode"]) {
    data_["mode"] = "offline";
  }
  if (!data_["pose_csv"]) {
    data_["pose_csv"] = "/tmp/tcp_poses.csv";
  }
  if (!data_["flange_frame"]) {
    data_["flange_frame"] = "tool0";
  }
  if (!data_["min_poses"]) {
    data_["min_poses"] = 4;
  }
  if (!data_["unit"]) {
    data_["unit"] = "m";
  }
  if (!data_["pose_format"]) {
    data_["pose_format"] = "quaternion";
  }
}

void TcpCalibrationDataManager::SetPoseEndpoint(const std::string & v)
{
  EnsureDefaults();
  data_["pose_endpoint"] = v;
}

std::string TcpCalibrationDataManager::GetPoseEndpoint() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "pose_endpoint", "/robot_pose");
}

void TcpCalibrationDataManager::SetControlEndpoint(const std::string & v)
{
  EnsureDefaults();
  data_["control_endpoint"] = v;
}

std::string TcpCalibrationDataManager::GetControlEndpoint() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "control_endpoint", "/robot_control");
}

void TcpCalibrationDataManager::SetMode(const std::string & v)
{
  EnsureDefaults();
  data_["mode"] = v;
}

std::string TcpCalibrationDataManager::GetMode() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "mode", "offline");
}

void TcpCalibrationDataManager::SetPoseCsv(const std::string & v)
{
  EnsureDefaults();
  data_["pose_csv"] = v;
}

std::string TcpCalibrationDataManager::GetPoseCsv() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "pose_csv", "/tmp/tcp_poses.csv");
}

void TcpCalibrationDataManager::SetFlangeFrame(const std::string & v)
{
  EnsureDefaults();
  data_["flange_frame"] = v;
}

std::string TcpCalibrationDataManager::GetFlangeFrame() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "flange_frame", "tool0");
}

void TcpCalibrationDataManager::SetMinPoses(int v)
{
  EnsureDefaults();
  data_["min_poses"] = v;
}

int TcpCalibrationDataManager::GetMinPoses() const
{
  return GetI(const_cast<YAML::Node &>(data_), "min_poses", 4);
}

void TcpCalibrationDataManager::SetUnit(const std::string & v)
{
  EnsureDefaults();
  data_["unit"] = v;
}

std::string TcpCalibrationDataManager::GetUnit() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "unit", "m");
}

void TcpCalibrationDataManager::SetPoseFormat(const std::string & v)
{
  EnsureDefaults();
  data_["pose_format"] = v;
}

std::string TcpCalibrationDataManager::GetPoseFormat() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "pose_format", "quaternion");
}

}  // namespace ros_robot_assist_tools::manage
