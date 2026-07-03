#ifndef ROS_ROBOT_WORKBENCH__UI__DEPTH_ANALYZER_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__DEPTH_ANALYZER_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class DepthAnalyzerWidget : public QWidget
{
public:
  explicit DepthAnalyzerWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
