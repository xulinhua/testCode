#ifndef ROS_ROBOT_WORKBENCH__UI__UI_DATA_STRUCTS_H_
#define ROS_ROBOT_WORKBENCH__UI__UI_DATA_STRUCTS_H_

#include <QString>
#include <vector>

namespace ros_robot_workbench::ui
{

enum class EndpointType { Topic = 0, Service = 1, Action = 2 };

struct RosInterfaceConfig
{
  QString image_topic = "/camera/image_raw";
  QString pose_endpoint = "/robot_pose";
  EndpointType pose_endpoint_type = EndpointType::Topic;
  QString control_endpoint = "/robot_control";
  EndpointType control_endpoint_type = EndpointType::Topic;
  /// 手眼标定：eye_in_hand（眼在手上）或 eye_to_hand（眼在手外）
  QString handeye_setup = "eye_in_hand";
  /// 手眼离线：位姿与图像名列表 CSV 路径（图像与 CSV 同目录）
  QString handeye_poses_csv;
};

struct ResourceUsage
{
  int cpu = 0;
  int mem = 0;
  int gpu = 0;
  int vram = 0;
  double disk_read_kbps = 0.0;
  double disk_write_kbps = 0.0;
  double up_kbps = 0.0;
  double down_kbps = 0.0;
};

struct ProcessRow { QString pid; QString cpu; QString mem; QString command; };
struct NodeInfoRow { QString name; QString startup_file; };
struct ParamRow { QString name; QString value; };
struct TopicTypeRow { QString topic; QString type; QString hz; };

struct BoardGeneratorParams
{
  int board_type = 0;
  int dict_index = 0;
  int marker_id = 0;
  int start_id = 0;
  int marker_size = 400;
  /// Aruco 单码：勾选后为四边对称白边宽度（像素/侧），图像边长 = marker_size + 2 * aruco_border_px。
  bool aruco_white_border = false;
  int aruco_border_px = 0;
  int rows = 7;
  int cols = 10;
  double board_width_mm = 297.0;
  double board_height_mm = 210.0;
  double cell_size_mm = 25.0;
  double circle_diameter_mm = 12.0;
  double tag_size_mm = 12.0;
  double marker_ratio = 0.75;
};

struct ArucoDetectResult
{
  bool ok = false;
  QString message;
  std::vector<int> ids;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct EulerAngles
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

}  // namespace ros_robot_workbench::ui

#endif
