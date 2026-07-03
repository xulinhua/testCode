#include "ros_robot_workbench/manage/topic_lab_data_manager.hpp"

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

TopicLabDataManager::TopicLabDataManager()
: FeatureDataManagerBase("topic_lab.yaml")
{
}

void TopicLabDataManager::EnsureDefaults()
{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {
    data_["module"] = "topic_lab";
  }
  if (!data_["default_topic"]) {
    data_["default_topic"] = "/chatter";
  }
  if (!data_["echo_buffer_size"]) {
    data_["echo_buffer_size"] = 50;
  }
  if (!data_["hz_sample_sec"]) {
    data_["hz_sample_sec"] = 2;
  }
  if (!data_["favorite_topics"]) {
    data_["favorite_topics"] = YAML::Node(YAML::NodeType::Sequence);
    data_["favorite_topics"].push_back("/joint_states");
    data_["favorite_topics"].push_back("/odom");
  }
}

void TopicLabDataManager::SetDefaultTopic(const std::string & v)
{
  EnsureDefaults();
  data_["default_topic"] = v;
}

std::string TopicLabDataManager::GetDefaultTopic() const
{
  return GetStr(const_cast<YAML::Node &>(data_), "default_topic", "/chatter");
}

void TopicLabDataManager::SetEchoBufferSize(int v)
{
  EnsureDefaults();
  data_["echo_buffer_size"] = v;
}

int TopicLabDataManager::GetEchoBufferSize() const
{
  return GetI(const_cast<YAML::Node &>(data_), "echo_buffer_size", 50);
}

void TopicLabDataManager::SetHzSampleSec(int v)
{
  EnsureDefaults();
  data_["hz_sample_sec"] = v;
}

int TopicLabDataManager::GetHzSampleSec() const
{
  return GetI(const_cast<YAML::Node &>(data_), "hz_sample_sec", 2);
}

void TopicLabDataManager::SetFavoriteTopics(const std::vector<std::string> & v)
{
  EnsureDefaults();
  YAML::Node seq(YAML::NodeType::Sequence);
  for (const auto & s : v) {
    seq.push_back(s);
  }
  data_["favorite_topics"] = seq;
}

std::vector<std::string> TopicLabDataManager::GetFavoriteTopics() const
{
  std::vector<std::string> out;
  const YAML::Node fav = const_cast<YAML::Node &>(data_)["favorite_topics"];
  if (fav && fav.IsSequence()) {
    for (const auto & n : fav) {
      if (n.IsScalar()) {
        out.push_back(n.as<std::string>());
      }
    }
  }
  return out;
}

}  // namespace ros_robot_workbench::manage
