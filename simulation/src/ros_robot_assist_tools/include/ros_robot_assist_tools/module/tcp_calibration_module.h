#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__TCP_CALIBRATION_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__TCP_CALIBRATION_MODULE_H_

#include <QString>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ros_robot_assist_tools::ui
{

struct FlangePoseSample
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct TcpCalibrationResult
{
  Eigen::Vector3d tcp_offset_flange = Eigen::Vector3d::Zero();
  Eigen::Vector3d contact_point_base = Eigen::Vector3d::Zero();
  double rms_residual_m = 0.0;
  double max_residual_m = 0.0;
  int num_poses = 0;
};

bool ParseTcpPosesFromCsv(const QString & csv_path, std::vector<FlangePoseSample> * poses, QString * err_msg);

bool SolveTcpOffsetTranslation(
  const std::vector<FlangePoseSample> & poses,
  TcpCalibrationResult * result,
  QString * err_msg);

bool SaveTcpCalibrationYaml(
  const QString & output_yaml,
  const TcpCalibrationResult & result,
  const QString & flange_frame,
  const QString & source_desc,
  QString * err_msg);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__TCP_CALIBRATION_MODULE_H_
