#ifndef ROS_ROBOT_WORKBENCH__MANAGE__USD_CONVERTER_DATA_MANAGER_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__USD_CONVERTER_DATA_MANAGER_HPP_

#include <string>

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{

class UsdConverterDataManager : public FeatureDataManagerBase
{
public:
  UsdConverterDataManager();
  void EnsureDefaults() override;

  void SetDefaultOutputDir(const std::string & v);
  std::string GetDefaultOutputDir() const;
  void SetMeshBackendDefault(const std::string & v);
  std::string GetMeshBackendDefault() const;
  void SetUrdfBackendDefault(const std::string & v);
  std::string GetUrdfBackendDefault() const;
  void SetIsaacPython(const std::string & v);
  std::string GetIsaacPython() const;
  void SetPythonVenv(const std::string & v);
  std::string GetPythonVenv() const;
  void SetMergeFixedJoints(bool v);
  bool GetMergeFixedJoints() const;
  void SetFixBase(bool v);
  bool GetFixBase() const;
  void SetMeshRootPrim(const std::string & v);
  std::string GetMeshRootPrim() const;
  void SetExpandXacro(bool v);
  bool GetExpandXacro() const;
};

}  // namespace ros_robot_workbench::manage

#endif
