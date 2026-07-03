#include "ros_robot_workbench/ui/sim_time_monitor_widget.h"

#include <chrono>
#include <mutex>

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/sim_time_monitor_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{
namespace
{

double WallSeconds()
{
  using clock = std::chrono::steady_clock;
  static const auto t0 = clock::now();
  const auto dt = std::chrono::duration<double>(clock::now() - t0);
  return dt.count();
}

}  // namespace

SimTimeMonitorWidget::SimTimeMonitorWidget(QWidget * parent)
: QWidget(parent)
, node_(rclcpp::Node::make_shared("sim_time_monitor_ui", rclcpp::NodeOptions()))
{
  dm_.SetConfigPath(ResolveDefaultConfigYamlPath("sim_time_monitor.yaml").toStdString());
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);

  QLabel * title = new QLabel("Sim Time");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(SimTimeMonitorModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * cfg = new QGroupBox("配置");
  QFormLayout * form = new QFormLayout(cfg);
  clock_topic_ = new QLineEdit(QString::fromStdString(dm_.GetClockTopic()));
  form->addRow("clock topic:", clock_topic_);
  root->addWidget(cfg);

  QGroupBox * stats = new QGroupBox("状态");
  QFormLayout * stats_form = new QFormLayout(stats);
  sim_label_ = new QLabel("--");
  wall_label_ = new QLabel("--");
  rtf_label_ = new QLabel("--");
  status_label_ = new QLabel("等待 /clock");
  stats_form->addRow("Sim Time (s):", sim_label_);
  stats_form->addRow("Wall Time (s):", wall_label_);
  stats_form->addRow("RTF:", rtf_label_);
  stats_form->addRow("状态:", status_label_);
  root->addWidget(stats);

  QPushButton * apply = new QPushButton("应用 topic 并重订阅");
  root->addWidget(apply);
  root->addStretch();

  struct ClockState
  {
    std::mutex mu;
    double sim_sec = 0.0;
    bool has = false;
  };
  auto st = std::make_shared<ClockState>();

  auto on_clock = [st](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(st->mu);
    st->sim_sec = rclcpp::Time(msg->clock).seconds();
    st->has = true;
  };

  auto bind_clock = [this, st, on_clock]() {
    clock_sub_.reset();
    had_prev_ = false;
    clock_sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic_->text().trimmed().toStdString(), rclcpp::QoS(10), on_clock);
  };

  bind_clock();
  SharedUiExecutor::instance().add_node(node_);

  timer_ = new QTimer(this);
  const int hz = std::max(1, dm_.GetRefreshHz());
  timer_->setInterval(1000 / hz);
  QObject::connect(timer_, &QTimer::timeout, [this, st]() {
    double sim = 0.0;
    bool has = false;
    {
      std::lock_guard<std::mutex> lock(st->mu);
      sim = st->sim_sec;
      has = st->has;
    }
    if (!has) {
      status_label_->setText("未收到 clock");
      return;
    }
    const double wall = WallSeconds();
    const SimTimeSnapshot snap = ComputeSimTimeSnapshot(prev_sim_, prev_wall_, sim, wall, had_prev_);
    sim_label_->setText(QString::number(snap.sim_sec, 'f', 3));
    wall_label_->setText(QString::number(snap.wall_sec, 'f', 1));
    if (snap.rtf_valid) {
      rtf_label_->setText(QString::number(snap.rtf, 'f', 2));
      status_label_->setText(FormatRtfStatus(snap.rtf, true));
    } else {
      rtf_label_->setText("--");
      status_label_->setText("计算 RTF…");
    }
    prev_sim_ = sim;
    prev_wall_ = wall;
    had_prev_ = true;
  });
  timer_->start();

  QObject::connect(apply, &QPushButton::clicked, [this, bind_clock]() {
    dm_.SetClockTopic(clock_topic_->text().trimmed().toStdString());
    dm_.Save();
    bind_clock();
    had_prev_ = false;
    status_label_->setText("已重订阅");
  });
}

SimTimeMonitorWidget::~SimTimeMonitorWidget()
{
  if (timer_) {
    timer_->stop();
  }
}

}  // namespace ros_robot_workbench::ui
