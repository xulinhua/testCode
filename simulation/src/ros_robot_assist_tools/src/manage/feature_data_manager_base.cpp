#include "ros_robot_assist_tools/manage/feature_data_manager_base.hpp"

#include <filesystem>
#include <fstream>

namespace ros_robot_assist_tools::manage
{
namespace
{

bool EnsureParentDir(const std::string & path)
{
  const std::filesystem::path p(path);
  const std::filesystem::path parent = p.parent_path();
  if (parent.empty()) {
    return true;
  }
  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
  return !ec;
}

}  // namespace

std::string JoinConfigPath(const std::string & dir, const std::string & file_name)
{
  if (dir.empty()) {
    return file_name;
  }
  return (std::filesystem::path(dir) / file_name).string();
}

FeatureDataManagerBase::FeatureDataManagerBase(const std::string & default_file_name)
: config_path_(JoinConfigPath("config", default_file_name))
{
}

void FeatureDataManagerBase::SetConfigPath(const std::string & yaml_path)
{
  config_path_ = yaml_path;
}

const std::string & FeatureDataManagerBase::GetConfigPath() const
{
  return config_path_;
}

bool FeatureDataManagerBase::Load()
{
  EnsureDefaults();
  try {
    if (!std::filesystem::exists(config_path_)) {
      return true;
    }
    const YAML::Node loaded = YAML::LoadFile(config_path_);
    if (loaded && loaded.IsMap()) {
      data_ = loaded;
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool FeatureDataManagerBase::Save() const
{
  if (!EnsureParentDir(config_path_)) {
    return false;
  }
  try {
    YAML::Emitter out;
    out << data_;
    std::ofstream fout(config_path_);
    if (!fout.is_open()) {
      return false;
    }
    fout << out.c_str();
    return true;
  } catch (...) {
    return false;
  }
}

bool FeatureDataManagerBase::EnsureFileExists() const
{
  if (std::filesystem::exists(config_path_)) {
    return true;
  }
  if (!EnsureParentDir(config_path_)) {
    return false;
  }
  std::ofstream fout(config_path_);
  return fout.is_open();
}

void FeatureDataManagerBase::EnsureDefaults()
{
  if (!data_ || !data_.IsMap()) {
    data_ = YAML::Node(YAML::NodeType::Map);
  }
}

}  // namespace ros_robot_assist_tools::manage
