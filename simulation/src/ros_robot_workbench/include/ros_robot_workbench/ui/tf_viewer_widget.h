#ifndef ROS_ROBOT_WORKBENCH__UI__TF_VIEWER_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__TF_VIEWER_WIDGET_H_

#include <QWidget>

namespace ros_robot_workbench::ui
{

class TfViewerWidget : public QWidget
{
public:
  explicit TfViewerWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
