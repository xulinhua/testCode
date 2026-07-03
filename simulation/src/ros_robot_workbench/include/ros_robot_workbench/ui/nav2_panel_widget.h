#ifndef ROS_ROBOT_WORKBENCH__UI__NAV2_PANEL_WIDGET_H
#define ROS_ROBOT_WORKBENCH__UI__NAV2_PANEL_WIDGET_H

#include <QWidget>

namespace ros_robot_workbench::ui
{

class Nav2PanelWidget : public QWidget
{
public:
  explicit Nav2PanelWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
