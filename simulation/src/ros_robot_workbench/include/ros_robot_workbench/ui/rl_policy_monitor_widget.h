#ifndef ROS_ROBOT_WORKBENCH__UI__RL_POLICY_MONITOR_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__RL_POLICY_MONITOR_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class RlPolicyMonitorWidget : public QWidget
{
public:
  explicit RlPolicyMonitorWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
