#ifndef ROS_ROBOT_WORKBENCH__UI__HUMANOID_JOINT_MONITOR_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__HUMANOID_JOINT_MONITOR_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class HumanoidJointMonitorWidget : public QWidget
{
public:
  explicit HumanoidJointMonitorWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
