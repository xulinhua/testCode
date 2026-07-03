#ifndef ROS_ROBOT_WORKBENCH__UI__LEGGED_IMU_PANEL_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__LEGGED_IMU_PANEL_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class LeggedImuPanelWidget : public QWidget
{
public:
  explicit LeggedImuPanelWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
