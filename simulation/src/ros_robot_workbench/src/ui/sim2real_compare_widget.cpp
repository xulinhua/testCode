#include "ros_robot_workbench/ui/sim2real_compare_widget.h"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/sim2real_compare_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{

Sim2realCompareWidget::Sim2realCompareWidget(QWidget * parent)
: QWidget(parent)
, node_(rclcpp::Node::make_shared("sim2real_compare_ui", rclcpp::NodeOptions()))
, cache_(std::make_shared<JointStateCache>())
{
  dm_.SetConfigPath(ResolveDefaultConfigYamlPath("sim2real_compare.yaml").toStdString());
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);

  QLabel * title = new QLabel("Sim2Real对比");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(Sim2realCompareModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * cfg = new QGroupBox("Topic 配置");
  QFormLayout * form = new QFormLayout(cfg);
  sim_topic_ = new QLineEdit(QString::fromStdString(dm_.GetSimTopic()));
  real_topic_ = new QLineEdit(QString::fromStdString(dm_.GetRealTopic()));
  field_combo_ = new QComboBox();
  field_combo_->addItem("位置 (rad)", static_cast<int>(Sim2RealField::Position));
  field_combo_->addItem("速度 (rad/s)", static_cast<int>(Sim2RealField::Velocity));
  if (dm_.GetCompareField() == "velocity") {
    field_combo_->setCurrentIndex(1);
  }
  form->addRow("仿真 topic:", sim_topic_);
  form->addRow("实机 topic:", real_topic_);
  form->addRow("对比字段:", field_combo_);
  root->addWidget(cfg);

  stats_label_ = new QLabel("RMSE: --  |  Max |e|: --  |  匹配关节: 0");
  root->addWidget(stats_label_);

  table_ = new QTableWidget(0, 4);
  table_->setHorizontalHeaderLabels({"关节", "仿真", "实机", "误差"});
  table_->horizontalHeader()->setStretchLastSection(true);
  root->addWidget(table_, 1);

  QPushButton * apply = new QPushButton("应用并重订阅");
  root->addWidget(apply);

  resubscribe();
  SharedUiExecutor::instance().add_node(node_);

  timer_ = new QTimer(this);
  timer_->setInterval(1000 / std::max(1, dm_.GetRefreshHz()));
  QObject::connect(timer_, &QTimer::timeout, [this]() {
    sensor_msgs::msg::JointState sim;
    sensor_msgs::msg::JointState real;
    bool has_sim = false;
    bool has_real = false;
    {
      std::lock_guard<std::mutex> lock(cache_->mu);
      sim = cache_->sim;
      real = cache_->real;
      has_sim = cache_->has_sim;
      has_real = cache_->has_real;
    }
    if (!has_sim || !has_real) {
      stats_label_->setText(has_sim ? "等待实机 joint_states…" : "等待仿真 joint_states…");
      return;
    }
    updateTableFromState(sim, real);
  });
  timer_->start();

  QObject::connect(apply, &QPushButton::clicked, [this]() {
    dm_.SetSimTopic(sim_topic_->text().trimmed().toStdString());
    dm_.SetRealTopic(real_topic_->text().trimmed().toStdString());
    dm_.SetCompareField(field_combo_->currentIndex() == 1 ? "velocity" : "position");
    dm_.Save();
    resubscribe();
  });
}

void Sim2realCompareWidget::resubscribe()
{
  sim_sub_.reset();
  real_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(cache_->mu);
    cache_->has_sim = false;
    cache_->has_real = false;
  }

  const std::string sim_t = sim_topic_->text().trimmed().toStdString();
  const std::string real_t = real_topic_->text().trimmed().toStdString();
  sim_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    sim_t, rclcpp::SensorDataQoS(),
    [cache = cache_](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cache->mu);
      cache->sim = *msg;
      cache->has_sim = true;
    });
  real_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    real_t, rclcpp::SensorDataQoS(),
    [cache = cache_](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cache->mu);
      cache->real = *msg;
      cache->has_real = true;
    });
}

void Sim2realCompareWidget::updateTableFromState(
  const sensor_msgs::msg::JointState & sim,
  const sensor_msgs::msg::JointState & real)
{
  const auto field = static_cast<Sim2RealField>(field_combo_->currentData().toInt());
  const auto rows = CompareJointStates(sim, real, field);
  const Sim2RealStats stats = ComputeSim2RealStats(rows);

  table_->setRowCount(static_cast<int>(rows.size()));
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    const auto & r = rows[static_cast<size_t>(i)];
    table_->setItem(i, 0, new QTableWidgetItem(r.name));
    table_->setItem(i, 1, new QTableWidgetItem(r.has_sim ? QString::number(r.sim_val, 'g', 6) : "--"));
    table_->setItem(i, 2, new QTableWidgetItem(r.has_real ? QString::number(r.real_val, 'g', 6) : "--"));
    table_->setItem(
      i, 3,
      new QTableWidgetItem((r.has_sim && r.has_real) ? QString::number(r.error, 'g', 6) : "--"));
  }
  stats_label_->setText(
    QString("RMSE: %1  |  Max |e|: %2  |  匹配关节: %3")
      .arg(stats.rmse, 0, 'g', 4)
      .arg(stats.max_abs_error, 0, 'g', 4)
      .arg(stats.matched));
}

}  // namespace ros_robot_workbench::ui
