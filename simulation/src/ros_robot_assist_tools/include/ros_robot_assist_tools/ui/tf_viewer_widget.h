#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__TF_VIEWER_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__TF_VIEWER_WIDGET_H_

#include <QWidget>

namespace ros_robot_assist_tools::ui
{

class TfViewerWidget : public QWidget
{
public:
  explicit TfViewerWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_assist_tools::ui

#endif
