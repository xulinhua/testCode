#ifndef ROS_ROBOT_WORKBENCH__UI__OBSTACLE_EDITOR_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__OBSTACLE_EDITOR_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class ObstacleEditorWidget : public QWidget
{
public:
  explicit ObstacleEditorWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
