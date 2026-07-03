#ifndef ROS_ROBOT_WORKBENCH__UI__POSE_TRANSFORM_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__POSE_TRANSFORM_WIDGET_H_

#include <QWidget>

namespace ros_robot_workbench::ui
{

class PoseTransformWidget : public QWidget
{
public:
  explicit PoseTransformWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
