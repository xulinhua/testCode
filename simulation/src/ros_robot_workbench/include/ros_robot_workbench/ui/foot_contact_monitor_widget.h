#ifndef ROS_ROBOT_WORKBENCH__UI__FOOT_CONTACT_MONITOR_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__FOOT_CONTACT_MONITOR_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class FootContactMonitorWidget : public QWidget
{
public:
  explicit FootContactMonitorWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
