#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__POSE_TRANSFORM_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__POSE_TRANSFORM_WIDGET_H_

#include <QWidget>

namespace ros_robot_assist_tools::ui
{

class PoseTransformWidget : public QWidget
{
public:
  explicit PoseTransformWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_assist_tools::ui

#endif
