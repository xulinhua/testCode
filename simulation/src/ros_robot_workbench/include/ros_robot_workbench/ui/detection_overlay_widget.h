#ifndef ROS_ROBOT_WORKBENCH__UI__DETECTION_OVERLAY_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__DETECTION_OVERLAY_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class DetectionOverlayWidget : public QWidget
{
public:
  explicit DetectionOverlayWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
