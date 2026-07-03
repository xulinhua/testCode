#ifndef ROS_ROBOT_WORKBENCH__UI__INFERENCE_MONITOR_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__INFERENCE_MONITOR_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class InferenceMonitorWidget : public QWidget
{
public:
  explicit InferenceMonitorWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
