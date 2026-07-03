#ifndef ROS_ROBOT_WORKBENCH__MANAGE__FEATURE_DATA_MANAGER_BASE_HPP_
#define ROS_ROBOT_WORKBENCH__MANAGE__FEATURE_DATA_MANAGER_BASE_HPP_

#include <string>

#include <yaml-cpp/yaml.h>

namespace ros_robot_workbench::manage
{

class FeatureDataManagerBase
{
public:
  explicit FeatureDataManagerBase(const std::string & default_file_name);
  virtual ~FeatureDataManagerBase() = default;

  void SetConfigPath(const std::string & yaml_path);
  const std::string & GetConfigPath() const;

  bool Load();
  bool Save() const;
  bool EnsureFileExists() const;

protected:
  /// 派生类可覆盖，在每次 Load/构造基类逻辑中合并默认键。
  virtual void EnsureDefaults();

  YAML::Node data_{YAML::NodeType::Map};

private:
  std::string config_path_;
};

std::string JoinConfigPath(const std::string & dir, const std::string & file_name);

}  // namespace ros_robot_workbench::manage

#endif  // ROS_ROBOT_WORKBENCH__MANAGE__FEATURE_DATA_MANAGER_BASE_HPP_
