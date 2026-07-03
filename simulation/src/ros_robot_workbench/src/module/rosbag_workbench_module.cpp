#include "ros_robot_workbench/module/rosbag_workbench_module.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>

#include <QtGlobal>

#include <yaml-cpp/yaml.h>

#include "ros_robot_workbench/module/calibration_module.h"

namespace ros_robot_workbench::ui
{

QString RosbagWorkbenchModuleSummary()
{
  return QStringLiteral("rosbag2 录制、metadata 浏览与回放；基于 ros2 bag CLI，适用于离线标定与问题复现。");
}

bool ParseBagMetadata(const QString & bag_dir, BagMetadata * meta, QString * err_msg)
{
  if (!meta) {
    if (err_msg) {
      *err_msg = "meta is null";
    }
    return false;
  }
  QString validate_err;
  if (!ValidateRosbagDirectory(bag_dir, &validate_err)) {
    if (err_msg) {
      *err_msg = validate_err;
    }
    return false;
  }

  meta->bag_dir = bag_dir;
  meta->topics.clear();
  meta->duration_sec = 0.0;
  meta->starting_time_ns = 0;

  try {
    const QString meta_path = QDir(bag_dir).filePath("metadata.yaml");
    const YAML::Node root = YAML::LoadFile(meta_path.toStdString());
    const YAML::Node bag_info = root["rosbag2_bagfile_information"];
    if (!bag_info || !bag_info.IsMap()) {
      if (err_msg) {
        *err_msg = "metadata.yaml 缺少 rosbag2_bagfile_information";
      }
      return false;
    }
    if (bag_info["duration"]) {
      const YAML::Node dur = bag_info["duration"];
      if (dur["nanoseconds"]) {
        meta->duration_sec = dur["nanoseconds"].as<uint64_t>() / 1e9;
      }
    }
    if (bag_info["starting_time"]) {
      const YAML::Node st = bag_info["starting_time"];
      if (st["nanoseconds_since_epoch"]) {
        meta->starting_time_ns = st["nanoseconds_since_epoch"].as<uint64_t>();
      }
    }
    const YAML::Node topic_rows = bag_info["topics_with_message_count"];
    if (!topic_rows || !topic_rows.IsSequence()) {
      if (err_msg) {
        *err_msg = "metadata.yaml 中未找到 topics_with_message_count";
      }
      return false;
    }
    for (const auto & row : topic_rows) {
      const YAML::Node topic_meta = row["topic_metadata"];
      if (!topic_meta || !topic_meta.IsMap()) {
        continue;
      }
      BagTopicInfo info;
      info.name = topic_meta["name"] ? QString::fromStdString(topic_meta["name"].as<std::string>()) : QString();
      info.type = topic_meta["type"] ? QString::fromStdString(topic_meta["type"].as<std::string>()) : QString();
      info.message_count = row["message_count"] ? row["message_count"].as<int64_t>() : 0;
      if (!info.name.isEmpty()) {
        meta->topics.push_back(info);
      }
    }
    return !meta->topics.empty();
  } catch (const std::exception & e) {
    if (err_msg) {
      *err_msg = QString("解析 metadata.yaml 失败: %1").arg(e.what());
    }
    return false;
  }
}

QString BuildBagRecordCommand(const QString & output_uri, const std::vector<QString> & topics)
{
  QStringList parts;
  parts << "ros2 bag record -o" << QString("\"%1\"").arg(output_uri);
  for (const QString & t : topics) {
    if (!t.trimmed().isEmpty()) {
      parts << t.trimmed();
    }
  }
  return parts.join(' ');
}

QString BuildBagPlayCommand(
  const QString & bag_dir, double rate, bool loop_play, bool use_sim_time)
{
  QStringList parts;
  parts << "ros2 bag play" << QString("\"%1\"").arg(bag_dir);
  if (rate > 0.0 && qAbs(rate - 1.0) > 1e-6) {
    parts << "--rate" << QString::number(rate, 'g', 4);
  }
  if (loop_play) {
    parts << "--loop";
  }
  if (use_sim_time) {
    parts << "--clock";
  }
  return parts.join(' ');
}

QString DefaultBagOutputUri(const QString & output_dir, const QString & prefix)
{
  const QString base = output_dir.trimmed().isEmpty()
    ? QDir::homePath() + "/.ros_robot_workbench/bags"
    : output_dir.trimmed();
  QDir().mkpath(base);
  const QString p = prefix.trimmed().isEmpty() ? "run" : prefix.trimmed();
  return QDir(base).filePath(p + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
}

}  // namespace ros_robot_workbench::ui
