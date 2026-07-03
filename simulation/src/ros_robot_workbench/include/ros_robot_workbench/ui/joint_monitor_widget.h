#ifndef ROS_ROBOT_WORKBENCH__UI__JOINT_MONITOR_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__JOINT_MONITOR_WIDGET_H_

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

namespace ros_robot_workbench::ui
{

class JointMonitorWidget : public QWidget
{
public:
  explicit JointMonitorWidget(QWidget * parent = nullptr);

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace ros_robot_workbench::ui

#endif
