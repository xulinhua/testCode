#ifndef ROS_ROBOT_WORKBENCH__UI__ODOMETRY_ANALYZER_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__ODOMETRY_ANALYZER_WIDGET_H_

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

namespace ros_robot_workbench::ui
{

class OdometryAnalyzerWidget : public QWidget
{
public:
  explicit OdometryAnalyzerWidget(QWidget * parent = nullptr);

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace ros_robot_workbench::ui

#endif
