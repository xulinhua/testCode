#ifndef ROS_ROBOT_WORKBENCH__UI__SIM2REAL_COMPARE_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__SIM2REAL_COMPARE_WIDGET_H_

#include <memory>
#include <mutex>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ros_robot_workbench/manage/sim2real_compare_data_manager.hpp"

class QComboBox;
class QLabel;
class QLineEdit;
class QTableWidget;
class QTimer;

namespace ros_robot_workbench::ui
{

class Sim2realCompareWidget : public QWidget
{
public:
  explicit Sim2realCompareWidget(QWidget * parent = nullptr);

private:
  struct JointStateCache
  {
    std::mutex mu;
    sensor_msgs::msg::JointState sim;
    sensor_msgs::msg::JointState real;
    bool has_sim = false;
    bool has_real = false;
  };

  void resubscribe();
  void updateTableFromState(
    const sensor_msgs::msg::JointState & sim,
    const sensor_msgs::msg::JointState & real);

  manage::Sim2realCompareDataManager dm_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
  std::shared_ptr<JointStateCache> cache_;
  QLineEdit * sim_topic_ = nullptr;
  QLineEdit * real_topic_ = nullptr;
  QComboBox * field_combo_ = nullptr;
  QLabel * stats_label_ = nullptr;
  QTableWidget * table_ = nullptr;
  QTimer * timer_ = nullptr;
};

}  // namespace ros_robot_workbench::ui

#endif
