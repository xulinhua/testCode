#include "ros_robot_workbench/ui/joint_monitor_widget.h"

#include <mutex>

#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/joint_monitor_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{

JointMonitorWidget::JointMonitorWidget(QWidget * parent)
: QWidget(parent)
, node_(rclcpp::Node::make_shared("joint_monitor_ui", rclcpp::NodeOptions()))
{
  QVBoxLayout * root = new QVBoxLayout(this);
  QLabel * title = new QLabel("关节监视");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(JointMonitorModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * cfg = new QGroupBox("订阅");
  QFormLayout * form = new QFormLayout(cfg);
  QLineEdit * topic = new QLineEdit("/joint_states");
  form->addRow("joint_states:", topic);
  root->addWidget(cfg);

  QTableWidget * table = new QTableWidget(0, 3);
  table->setHorizontalHeaderLabels({"关节", "位置(rad)", "速度(rad/s)"});
  table->horizontalHeader()->setStretchLastSection(true);
  root->addWidget(table, 1);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  log->setMaximumHeight(80);
  root->addWidget(log);

  struct State
  {
    std::mutex mu;
    sensor_msgs::msg::JointState last;
  };
  auto state = std::make_shared<State>();

  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic->text().toStdString(), rclcpp::SensorDataQoS(),
    [state](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mu);
      state->last = *msg;
    });
  (void)sub;

  SharedUiExecutor::instance().add_node(node_);

  QPushButton * refresh = new QPushButton("刷新");
  QObject::connect(refresh, &QPushButton::clicked, [table, log, state]() {
    sensor_msgs::msg::JointState js;
    {
      std::lock_guard<std::mutex> lock(state->mu);
      js = state->last;
    }
    if (js.name.empty()) {
      log->appendPlainText("尚未收到 joint_states");
      return;
    }
    table->setRowCount(static_cast<int>(js.name.size()));
    for (size_t i = 0; i < js.name.size(); ++i) {
      table->setItem(static_cast<int>(i), 0, new QTableWidgetItem(QString::fromStdString(js.name[i])));
      const double p = i < js.position.size() ? js.position[i] : 0.0;
      const double v = i < js.velocity.size() ? js.velocity[i] : 0.0;
      table->setItem(static_cast<int>(i), 1, new QTableWidgetItem(QString::number(p, 'g', 6)));
      table->setItem(static_cast<int>(i), 2, new QTableWidgetItem(QString::number(v, 'g', 6)));
    }
    log->appendPlainText(QString("更新 %1 个关节").arg(js.name.size()));
  });
  root->addWidget(refresh);
}

}  // namespace ros_robot_workbench::ui
