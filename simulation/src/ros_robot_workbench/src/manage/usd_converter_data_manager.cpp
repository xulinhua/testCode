#include "ros_robot_workbench/manage/usd_converter_data_manager.hpp"

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

bool GetB(const YAML::Node & n, const char * k, bool d)
{
  if (n && n[k] && n[k].IsScalar()) {
    return n[k].as<bool>();
  }
  return d;
}

}  // namespace

UsdConverterDataManager::UsdConverterDataManager()
: FeatureDataManagerBase("usd_converter.yaml")
{
}

void UsdConverterDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "usd_converter";
  }
  if (!data_["default_output_dir"]) {
    data_["default_output_dir"] = "~/.ros_robot_workbench/usd";
  }
  if (!data_["mesh_backend_default"]) {
    data_["mesh_backend_default"] = "openusd";
  }
  if (!data_["urdf_backend_default"]) {
    data_["urdf_backend_default"] = "urdf_usd_converter";
  }
  if (!data_["isaac_python"]) {
    data_["isaac_python"] = "";
  }
  if (!data_["python_venv"]) {
    data_["python_venv"] = "";
  }
  if (!data_["merge_fixed_joints"]) {
    data_["merge_fixed_joints"] = false;
  }
  if (!data_["fix_base"]) {
    data_["fix_base"] = false;
  }
  if (!data_["mesh_root_prim"]) {
    data_["mesh_root_prim"] = "mesh";
  }
  if (!data_["expand_xacro"]) {
    data_["expand_xacro"] = false;
  }
}

void UsdConverterDataManager::SetDefaultOutputDir(const std::string & v)
{
  EnsureDefaults();
  data_["default_output_dir"] = v;
}

std::string UsdConverterDataManager::GetDefaultOutputDir() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "default_output_dir", "~/.ros_robot_workbench/usd");
}

void UsdConverterDataManager::SetMeshBackendDefault(const std::string & v)
{
  EnsureDefaults();
  data_["mesh_backend_default"] = v;
}

std::string UsdConverterDataManager::GetMeshBackendDefault() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "mesh_backend_default", "openusd");
}

void UsdConverterDataManager::SetUrdfBackendDefault(const std::string & v)
{
  EnsureDefaults();
  data_["urdf_backend_default"] = v;
}

std::string UsdConverterDataManager::GetUrdfBackendDefault() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "urdf_backend_default", "urdf_usd_converter");
}

void UsdConverterDataManager::SetIsaacPython(const std::string & v)
{
  EnsureDefaults();
  data_["isaac_python"] = v;
}

std::string UsdConverterDataManager::GetIsaacPython() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "isaac_python", "");
}

void UsdConverterDataManager::SetPythonVenv(const std::string & v)
{
  EnsureDefaults();
  data_["python_venv"] = v;
}

std::string UsdConverterDataManager::GetPythonVenv() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "python_venv", "");
}

void UsdConverterDataManager::SetMergeFixedJoints(bool v)
{
  EnsureDefaults();
  data_["merge_fixed_joints"] = v;
}

bool UsdConverterDataManager::GetMergeFixedJoints() const
{
  return GetB(const_cast<YAML::Node &>(data_), "merge_fixed_joints", false);
}

void UsdConverterDataManager::SetFixBase(bool v)
{
  EnsureDefaults();
  data_["fix_base"] = v;
}

bool UsdConverterDataManager::GetFixBase() const
{
  return GetB(const_cast<YAML::Node &>(data_), "fix_base", false);
}

void UsdConverterDataManager::SetMeshRootPrim(const std::string & v)
{
  EnsureDefaults();
  data_["mesh_root_prim"] = v;
}

std::string UsdConverterDataManager::GetMeshRootPrim() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "mesh_root_prim", "mesh");
}

void UsdConverterDataManager::SetExpandXacro(bool v)
{
  EnsureDefaults();
  data_["expand_xacro"] = v;
}

bool UsdConverterDataManager::GetExpandXacro() const
{
  return GetB(const_cast<YAML::Node &>(data_), "expand_xacro", false);
}

}  // namespace ros_robot_workbench::manage
