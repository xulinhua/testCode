#include "ros_robot_workbench/module/calibration_module.h"

#include <fstream>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QStringList>
#include <QTextStream>

#include <yaml-cpp/yaml.h>

namespace ros_robot_workbench::ui
{

QString EndpointTypeText(EndpointType type)
{
  switch (type) {
    case EndpointType::Topic: return "topic";
    case EndpointType::Service: return "service";
    case EndpointType::Action: return "action";
    default: return "unknown";
  }
}

QString BuildRosInterfaceSummary(const RosInterfaceConfig & config)
{
  return QString("image=%1, pose=%2(%3), control=%4(%5)")
    .arg(config.image_topic)
    .arg(config.pose_endpoint)
    .arg(EndpointTypeText(config.pose_endpoint_type))
    .arg(config.control_endpoint)
    .arg(EndpointTypeText(config.control_endpoint_type));
}

bool LoadRosInterfaceConfigFromYaml(const QString & yaml_path, RosInterfaceConfig * config, QString * err_msg)
{
  if (config == nullptr) {
    if (err_msg != nullptr) { *err_msg = "config is null"; }
    return false;
  }
  try {
    YAML::Node node = YAML::LoadFile(yaml_path.toStdString());
    if (node["image_topic"]) { config->image_topic = QString::fromStdString(node["image_topic"].as<std::string>()); }
    if (node["pose_endpoint"]) { config->pose_endpoint = QString::fromStdString(node["pose_endpoint"].as<std::string>()); }
    if (node["control_endpoint"]) { config->control_endpoint = QString::fromStdString(node["control_endpoint"].as<std::string>()); }
    if (node["handeye_setup"]) { config->handeye_setup = QString::fromStdString(node["handeye_setup"].as<std::string>()); }
    if (node["pose_csv"]) { config->handeye_poses_csv = QString::fromStdString(node["pose_csv"].as<std::string>()); }
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = e.what(); }
    return false;
  }
}

QString ResolveDefaultConfigYamlPath(const QString & config_file_name)
{
  const QString file = config_file_name.trimmed();
  if (file.isEmpty()) return QString();

  const QString app_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    QDir::current().filePath(QString("config/%1").arg(file)),
    QDir::current().filePath(QString("simulation/src/ros_robot_workbench/config/%1").arg(file)),
    QDir::current().filePath(QString("src/ros_robot_workbench/config/%1").arg(file)),
    QDir(app_dir).filePath(QString("../../share/ros_robot_workbench/config/%1").arg(file)),
    QDir(app_dir).filePath(QString("../share/ros_robot_workbench/config/%1").arg(file)),
  };
  for (const auto & path : candidates) {
    if (QFileInfo::exists(path)) {
      return QFileInfo(path).canonicalFilePath();
    }
  }
  return candidates.front();
}

std::vector<QString> ListOnlineImageTopics()
{
  std::vector<QString> topics;
  FILE * pipe = popen("bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 topic list -t 2>/dev/null'", "r");
  if (pipe == nullptr) return topics;
  char line[1024] = {0};
  QRegularExpression re("^\\s*([^\\s\\[]+)\\s*\\[([^\\]]+)\\]\\s*$");
  while (fgets(line, sizeof(line), pipe) != nullptr) {
    const QString row = QString::fromUtf8(line).trimmed();
    if (row.isEmpty()) continue;
    const QRegularExpressionMatch m = re.match(row);
    if (!m.hasMatch()) continue;
    const QString topic = m.captured(1).trimmed();
    const QString type = m.captured(2).trimmed();
    if (type == "sensor_msgs/msg/Image") {
      topics.push_back(topic);
    }
  }
  pclose(pipe);
  return topics;
}

std::vector<QString> ListOnlineCameraInfoTopics()
{
  std::vector<QString> topics;
  FILE * pipe = popen("bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 topic list -t 2>/dev/null'", "r");
  if (pipe == nullptr) return topics;
  char line[1024] = {0};
  QRegularExpression re("^\\s*([^\\s\\[]+)\\s*\\[([^\\]]+)\\]\\s*$");
  while (fgets(line, sizeof(line), pipe) != nullptr) {
    const QString row = QString::fromUtf8(line).trimmed();
    if (row.isEmpty()) continue;
    const QRegularExpressionMatch m = re.match(row);
    if (!m.hasMatch()) continue;
    const QString topic = m.captured(1).trimmed();
    const QString type = m.captured(2).trimmed();
    if (type == "sensor_msgs/msg/CameraInfo") {
      topics.push_back(topic);
    }
  }
  pclose(pipe);
  return topics;
}

bool ListImageTopicsFromRosbag(const QString & bag_dir, std::vector<QString> * topics, QString * err_msg)
{
  if (topics == nullptr) {
    if (err_msg != nullptr) { *err_msg = "topics is null"; }
    return false;
  }
  topics->clear();
  const QString meta_path = QDir(bag_dir).filePath("metadata.yaml");
  try {
    const YAML::Node root = YAML::LoadFile(meta_path.toStdString());
    const YAML::Node bag_info = root["rosbag2_bagfile_information"];
    const YAML::Node topic_rows = bag_info ? bag_info["topics_with_message_count"] : YAML::Node();
    if (!topic_rows || !topic_rows.IsSequence()) {
      if (err_msg != nullptr) { *err_msg = "metadata.yaml 中未找到 topics_with_message_count"; }
      return false;
    }
    for (const auto & row : topic_rows) {
      const YAML::Node topic_meta = row["topic_metadata"];
      if (!topic_meta || !topic_meta.IsMap()) continue;
      const std::string type = topic_meta["type"] ? topic_meta["type"].as<std::string>() : "";
      const std::string name = topic_meta["name"] ? topic_meta["name"].as<std::string>() : "";
      if (type == "sensor_msgs/msg/Image" && !name.empty()) {
        topics->push_back(QString::fromStdString(name));
      }
    }
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = QString("解析 metadata.yaml 失败: %1").arg(e.what()); }
    return false;
  }
}

bool ListCameraInfoTopicsFromRosbag(const QString & bag_dir, std::vector<QString> * topics, QString * err_msg)
{
  if (topics == nullptr) {
    if (err_msg != nullptr) { *err_msg = "topics is null"; }
    return false;
  }
  topics->clear();
  const QString meta_path = QDir(bag_dir).filePath("metadata.yaml");
  try {
    const YAML::Node root = YAML::LoadFile(meta_path.toStdString());
    const YAML::Node bag_info = root["rosbag2_bagfile_information"];
    const YAML::Node topic_rows = bag_info ? bag_info["topics_with_message_count"] : YAML::Node();
    if (!topic_rows || !topic_rows.IsSequence()) {
      if (err_msg != nullptr) { *err_msg = "metadata.yaml 中未找到 topics_with_message_count"; }
      return false;
    }
    for (const auto & row : topic_rows) {
      const YAML::Node topic_meta = row["topic_metadata"];
      if (!topic_meta || !topic_meta.IsMap()) continue;
      const std::string type = topic_meta["type"] ? topic_meta["type"].as<std::string>() : "";
      const std::string name = topic_meta["name"] ? topic_meta["name"].as<std::string>() : "";
      if (type == "sensor_msgs/msg/CameraInfo" && !name.empty()) {
        topics->push_back(QString::fromStdString(name));
      }
    }
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = QString("解析 metadata.yaml 失败: %1").arg(e.what()); }
    return false;
  }
}

QString DefaultCalibrationYamlPath(const QString & module_name)
{
  const QString base_dir = QDir::homePath() + "/.ros_robot_workbench/calibration";
  QDir().mkpath(base_dir);
  const QString safe_name = module_name.isEmpty() ? "calibration" : module_name;
  return QString("%1/%2_%3.yaml")
    .arg(base_dir)
    .arg(safe_name)
    .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
}

bool ValidateRosbagDirectory(const QString & bag_dir, QString * err_msg)
{
  QDir dir(bag_dir);
  if (!dir.exists()) {
    if (err_msg != nullptr) { *err_msg = "bag目录不存在"; }
    return false;
  }
  const QFileInfo metadata_info(dir.filePath("metadata.yaml"));
  if (!metadata_info.exists() || !metadata_info.isFile()) {
    if (err_msg != nullptr) { *err_msg = "缺少 metadata.yaml"; }
    return false;
  }
  const QStringList db3_files = dir.entryList(QStringList() << "*.db3", QDir::Files);
  if (db3_files.isEmpty()) {
    if (err_msg != nullptr) { *err_msg = "未找到 .db3 数据文件"; }
    return false;
  }
  return true;
}

QString ImageDirectoryForHandeyePosesCsv(const QString & csv_path)
{
  return QFileInfo(csv_path).absolutePath();
}

namespace
{
QString FirstColumnCell(const QString & line)
{
  const QString t = line.trimmed();
  if (t.isEmpty()) {
    return QString();
  }
  for (QChar c : {',', ';', '\t'}) {
    const int i = t.indexOf(c);
    if (i >= 0) {
      return t.left(i).trimmed();
    }
  }
  return t;
}

bool LikelyHeaderRowFirstColumn(const QString & first_cell)
{
  const QString s = first_cell.toLower();
  if (s == "image" || s == "filename" || s == "file" || s.contains("图像") || s.contains("图片")) {
    return true;
  }
  if (first_cell.contains('.')) {
    return false;
  }
  return s == "id" || s == "name" || s == "index";
}
}  // namespace

bool ValidateHandeyePosesCsvFile(const QString & csv_path, QString * err_msg)
{
  const QFileInfo fi(csv_path);
  if (!fi.exists() || !fi.isFile()) {
    if (err_msg != nullptr) { *err_msg = "CSV 文件不存在"; }
    return false;
  }
  if (fi.suffix().compare("csv", Qt::CaseInsensitive) != 0) {
    if (err_msg != nullptr) { *err_msg = "请选择 .csv 文件"; }
    return false;
  }
  QFile f(csv_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (err_msg != nullptr) { *err_msg = "无法打开 CSV 文件"; }
    return false;
  }
  if (f.size() < 1) {
    if (err_msg != nullptr) { *err_msg = "CSV 文件为空"; }
    return false;
  }
  return true;
}

bool ListImageFilenamesFromHandeyePosesCsv(
  const QString & csv_path, std::vector<QString> * basenames, int * line_count, QString * err_msg)
{
  if (basenames == nullptr) {
    if (err_msg != nullptr) { *err_msg = "basenames is null"; }
    return false;
  }
  basenames->clear();
  if (line_count != nullptr) { *line_count = 0; }
  QFile f(csv_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (err_msg != nullptr) { *err_msg = "无法打开 CSV"; }
    return false;
  }
  QTextStream in(&f);
  int total = 0;
  bool use_raw_image_column = false;
  int raw_image_col = -1;
  bool header_processed = false;
  while (!in.atEnd()) {
    const QString line = in.readLine();
    if (line.trimmed().isEmpty() || line.trimmed().startsWith("#")) {
      continue;
    }
    if (!header_processed) {
      header_processed = true;
      const QStringList header_cols = line.split(',', Qt::KeepEmptyParts);
      raw_image_col = header_cols.indexOf("raw_image");
      if (raw_image_col >= 0) {
        use_raw_image_column = true;
        continue;  // 跳过表头
      }
    }
    QString cell;
    if (use_raw_image_column) {
      const QStringList cols = line.split(',', Qt::KeepEmptyParts);
      if (raw_image_col >= 0 && raw_image_col < cols.size()) {
        cell = cols[raw_image_col].trimmed();
      }
    } else {
      cell = FirstColumnCell(line);
    }
    if (cell.isEmpty()) {
      continue;
    }
    if (!use_raw_image_column && total == 0 && LikelyHeaderRowFirstColumn(cell)) {
      continue;
    }
    basenames->push_back(cell);
    total++;
  }
  if (line_count != nullptr) { *line_count = total; }
  if (basenames->empty()) {
    if (err_msg != nullptr) { *err_msg = "未解析到图像文件名：请使用首列为图像文件名的数据行，可用首行作表头（如 image,filename）"; }
    return false;
  }
  return true;
}

bool SaveCalibrationYaml(
  const QString & output_yaml,
  const QString & module_name,
  const QString & mode_name,
  const QString & source_desc,
  const QString & board_type,
  const QString & distortion_model,
  const QString & image_topic,
  const QString & validation_camera_info_topic,
  const std::vector<std::pair<QString, QString>> & extra_fields,
  QString * err_msg)
{
  if (output_yaml.trimmed().isEmpty()) {
    if (err_msg != nullptr) { *err_msg = "输出路径为空"; }
    return false;
  }
  QFileInfo out_info(output_yaml);
  QDir().mkpath(out_info.absolutePath());

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "module_name" << YAML::Value << module_name.toStdString();
  out << YAML::Key << "mode" << YAML::Value << mode_name.toStdString();
  out << YAML::Key << "source" << YAML::Value << source_desc.toStdString();
  out << YAML::Key << "board_type" << YAML::Value << board_type.toStdString();
  out << YAML::Key << "distortion_model" << YAML::Value << distortion_model.toStdString();
  out << YAML::Key << "image_topic" << YAML::Value << image_topic.toStdString();
  out << YAML::Key << "validation_camera_info_topic" << YAML::Value << validation_camera_info_topic.toStdString();
  out << YAML::Key << "extra_fields" << YAML::Value << YAML::BeginMap;
  for (const auto & kv : extra_fields) {
    out << YAML::Key << kv.first.toStdString() << YAML::Value << kv.second.toStdString();
  }
  out << YAML::EndMap;
  out << YAML::Key << "stamp" << YAML::Value << QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
  out << YAML::Key << "K" << YAML::Value << YAML::Flow << std::vector<double>{
    1000.0, 0.0, 640.0,
    0.0, 1000.0, 360.0,
    0.0, 0.0, 1.0
  };
  out << YAML::Key << "D" << YAML::Value << YAML::Flow << std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};
  out << YAML::Key << "R" << YAML::Value << YAML::Flow << std::vector<double>{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
  out << YAML::Key << "P" << YAML::Value << YAML::Flow << std::vector<double>{
    1000.0, 0.0, 640.0, 0.0,
    0.0, 1000.0, 360.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };
  out << YAML::EndMap;

  std::ofstream fout(output_yaml.toStdString());
  if (!fout.is_open()) {
    if (err_msg != nullptr) { *err_msg = "无法写入输出文件"; }
    return false;
  }
  fout << out.c_str();
  return true;
}

}  // namespace ros_robot_workbench::ui
