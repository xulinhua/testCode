#ifndef ROS_ROBOT_WORKBENCH__UI__LIDAR_CAM_PROJECTION_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__LIDAR_CAM_PROJECTION_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class LidarCamProjectionWidget : public QWidget
{
public:
  explicit LidarCamProjectionWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
