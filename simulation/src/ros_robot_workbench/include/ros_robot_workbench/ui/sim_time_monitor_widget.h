#ifndef ROS_ROBOT_WORKBENCH__UI__SIM_TIME_MONITOR_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__SIM_TIME_MONITOR_WIDGET_H_

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include "ros_robot_workbench/manage/sim_time_monitor_data_manager.hpp"

class QLabel;
class QLineEdit;
class QTimer;

namespace ros_robot_workbench::ui
{

class SimTimeMonitorWidget : public QWidget
{
public:
  explicit SimTimeMonitorWidget(QWidget * parent = nullptr);
  ~SimTimeMonitorWidget() override;

private:
  manage::SimTimeMonitorDataManager dm_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::SubscriptionBase::SharedPtr clock_sub_;
  QTimer * timer_ = nullptr;
  QLineEdit * clock_topic_ = nullptr;
  QLabel * sim_label_ = nullptr;
  QLabel * wall_label_ = nullptr;
  QLabel * rtf_label_ = nullptr;
  QLabel * status_label_ = nullptr;
  double prev_sim_ = 0.0;
  double prev_wall_ = 0.0;
  bool had_prev_ = false;
};

}  // namespace ros_robot_workbench::ui

#endif
