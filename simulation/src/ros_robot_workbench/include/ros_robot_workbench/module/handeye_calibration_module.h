#ifndef ROS_ROBOT_WORKBENCH__MODULE__HANDEYE_CALIBRATION_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__HANDEYE_CALIBRATION_MODULE_H_

#include <QString>
#include <utility>
#include <vector>

namespace ros_robot_workbench::ui
{

/// 眼在手上：相机装在末端；眼在手外：相机相对基座固定，标定目标随末端运动。
enum class HandeyeSetupMode { EyeInHand = 0, EyeToHand = 1 };
enum class HandeyeSolverMethod { Tsai = 0, Park = 1, Horaud = 2, Andreff = 3, Daniilidis = 4 };

QString HandeyeSetupModeToYamlString(HandeyeSetupMode mode);
HandeyeSetupMode HandeyeSetupModeFromYamlString(const QString & value);
QString HandeyeSolverMethodToYamlString(HandeyeSolverMethod method);

/// 第三列坐标系在 YAML 中的键名（眼在手上为 camera_frame，眼在手外为 object_frame 标定板/目标）。
QString HandeyeThirdFrameYamlKey(HandeyeSetupMode mode);
QString HandeyeThirdFrameFieldLabel(HandeyeSetupMode mode);

/// 离线手眼（CSV+图像+内参）求解，当前用于单个 ArUco 标记模式。
bool RunOfflineHandeyeCalibrationForSingleAruco(
  const QString & csv_path,
  HandeyeSetupMode setup_mode,
  HandeyeSolverMethod solver_method,
  double marker_length_m,
  int target_marker_id,
  const QString & output_yaml,
  QString * summary,
  QString * detail_log,
  QString * err_msg);

/// 基于图像自动识别标定板类型；若为单个 ArUco，返回 marker_id。
bool DetectBoardTypeFromImage(
  const QString & image_path,
  QString * board_type,
  int * marker_id,
  QString * err_msg);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__HANDEYE_CALIBRATION_MODULE_H_
