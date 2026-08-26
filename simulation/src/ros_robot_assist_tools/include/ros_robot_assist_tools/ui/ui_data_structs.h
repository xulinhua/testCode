#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__UI_DATA_STRUCTS_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__UI_DATA_STRUCTS_H_

#include <QString>
#include <vector>

namespace ros_robot_assist_tools::ui
{

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
  /// Kalibr AprilGrid：Tag 间空白 / tag_size（无量纲，典型 0.3）
  double tag_spacing = 0.3;
  /// 非单码板导出分辨率参考（像素边长上限一侧）；AprilGrid 按物理尺寸×DPI 自适应
  int export_pixel_size = 2400;
  double marker_ratio = 0.75;
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

}  // namespace ros_robot_assist_tools::ui

#endif
