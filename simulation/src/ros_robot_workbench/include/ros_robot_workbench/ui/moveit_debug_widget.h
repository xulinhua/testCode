#ifndef ROS_ROBOT_WORKBENCH__UI__MOVEIT_DEBUG_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__MOVEIT_DEBUG_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class MoveitDebugWidget : public QWidget
{
public:
  explicit MoveitDebugWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
