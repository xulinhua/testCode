#include "ros_robot_workbench/ui/odometry_analyzer_widget.h"

#include <mutex>

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_robot_workbench/module/odometry_analyzer_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{

OdometryAnalyzerWidget::OdometryAnalyzerWidget(QWidget * parent)
: QWidget(parent)
, node_(rclcpp::Node::make_shared("odometry_analyzer_ui", rclcpp::NodeOptions()))
{
  QVBoxLayout * root = new QVBoxLayout(this);
  root->addWidget(new QLabel("里程计分析"));
  root->addWidget(new QLabel(OdometryAnalyzerModuleSummary()));

  QLineEdit * topic = new QLineEdit("/odom");
  QFormLayout * form = new QFormLayout();
  form->addRow("odom:", topic);
  root->addLayout(form);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  root->addWidget(log, 1);

  struct State
  {
    std::mutex mu;
    nav_msgs::msg::Odometry last;
    bool has = false;
  };
  auto st = std::make_shared<State>();
  auto sub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(), [st](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(st->mu);
      st->last = *msg;
      st->has = true;
    });
  (void)sub;
  SharedUiExecutor::instance().add_node(node_);

  QPushButton * btn = new QPushButton("显示当前 odom");
  QObject::connect(btn, &QPushButton::clicked, [log, st]() {
    std::lock_guard<std::mutex> lock(st->mu);
    if (!st->has) {
      log->appendPlainText("未收到 /odom");
      return;
    }
    const auto & p = st->last.pose.pose.position;
    const auto & q = st->last.pose.pose.orientation;
    log->appendPlainText(
      QString("frame=%1 pos=[%2,%3,%4] quat=[%5,%6,%7,%8] vx=%9")
        .arg(QString::fromStdString(st->last.header.frame_id))
        .arg(p.x, 0, 'g', 4).arg(p.y, 0, 'g', 4).arg(p.z, 0, 'g', 4)
        .arg(q.x, 0, 'g', 4).arg(q.y, 0, 'g', 4).arg(q.z, 0, 'g', 4).arg(q.w, 0, 'g', 4)
        .arg(st->last.twist.twist.linear.x, 0, 'g', 4));
  });
  root->addWidget(btn);
}

}  // namespace ros_robot_workbench::ui
