#ifndef ROS_ROBOT_WORKBENCH__MODULE__CALIBRATION_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__CALIBRATION_MODULE_H_

#include <QString>
#include <vector>

#include "ros_robot_workbench/ui/ui_data_structs.h"

namespace ros_robot_workbench::ui
{

QString EndpointTypeText(EndpointType type);
QString BuildRosInterfaceSummary(const RosInterfaceConfig & config);
bool LoadRosInterfaceConfigFromYaml(const QString & yaml_path, RosInterfaceConfig * config, QString * err_msg);
QString ResolveDefaultConfigYamlPath(const QString & config_file_name);
std::vector<QString> ListOnlineImageTopics();
std::vector<QString> ListOnlineCameraInfoTopics();
bool ListImageTopicsFromRosbag(const QString & bag_dir, std::vector<QString> * topics, QString * err_msg);
bool ListCameraInfoTopicsFromRosbag(const QString & bag_dir, std::vector<QString> * topics, QString * err_msg);
QString DefaultCalibrationYamlPath(const QString & module_name);
bool ValidateRosbagDirectory(const QString & bag_dir, QString * err_msg);
/// 手眼离线：校验采集列表 CSV 文件存在且可读
bool ValidateHandeyePosesCsvFile(const QString & csv_path, QString * err_msg);
/// 从 CSV 解析每行首列的图像文件名（与 CSV 同目录加载）；可返回样本行数
bool ListImageFilenamesFromHandeyePosesCsv(
  const QString & csv_path, std::vector<QString> * basenames, int * line_count, QString * err_msg);
/// CSV 与图像所在目录
QString ImageDirectoryForHandeyePosesCsv(const QString & csv_path);
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
  QString * err_msg);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__CALIBRATION_MODULE_H_
