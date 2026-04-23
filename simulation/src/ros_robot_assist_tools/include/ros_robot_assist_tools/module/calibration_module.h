#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__CALIBRATION_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__CALIBRATION_MODULE_H_

#include <QString>
#include <vector>

#include "ros_robot_assist_tools/ui/ui_data_structs.h"

namespace ros_robot_assist_tools::ui
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

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__CALIBRATION_MODULE_H_
