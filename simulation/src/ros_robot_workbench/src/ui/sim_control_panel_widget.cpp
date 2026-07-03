#include "ros_robot_workbench/ui/sim_control_panel_widget.h"

#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/sim_control_panel_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{
namespace
{

QPushButton * MakeActionButton(const QString & text, const QString & tip)
{
  QPushButton * btn = new QPushButton(text);
  btn->setToolTip(tip);
  btn->setMinimumWidth(72);
  btn->setMinimumHeight(32);
  return btn;
}

}  // namespace

SimControlPanelWidget::SimControlPanelWidget(QWidget * parent)
: QWidget(parent)
, proc_(new QProcess(this))
{
  dm_.SetConfigPath(ResolveDefaultConfigYamlPath("sim_control_panel.yaml").toStdString());
  dm_.Load();

  auto node = rclcpp::Node::make_shared("sim_control_panel_ui", rclcpp::NodeOptions());
  SharedUiExecutor::instance().add_node(node);

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(8);

  QLabel * title = new QLabel("仿真控制");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(SimControlPanelModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  // 顶栏：后端选择
  QHBoxLayout * top_row = new QHBoxLayout();
  QLabel * backend_label = new QLabel("控制后端");
  backend_label->setMinimumWidth(56);
  backend_ = new QComboBox();
  backend_->addItem("ROS 话题（通用）", static_cast<int>(SimControlBackend::RosTopic));
  backend_->addItem("Gazebo (gz service)", static_cast<int>(SimControlBackend::Gazebo));
  backend_->addItem("Isaac Sim 脚本", static_cast<int>(SimControlBackend::IsaacSim));
  const QString def_backend = QString::fromStdString(dm_.GetBackendDefault());
  if (def_backend == "gazebo") {
    backend_->setCurrentIndex(1);
  } else if (def_backend == "isaac") {
    backend_->setCurrentIndex(2);
  }
  backend_->setMinimumWidth(220);
  top_row->addWidget(backend_label);
  top_row->addWidget(backend_, 1);
  root->addLayout(top_row);

  // 仿真操作：主按钮区
  QGroupBox * run_group = new QGroupBox("仿真操作");
  QHBoxLayout * run_row = new QHBoxLayout(run_group);
  run_row->setSpacing(8);
  QPushButton * btn_play = MakeActionButton("▶ Play", "开始仿真");
  QPushButton * btn_pause = MakeActionButton("⏸ Pause", "暂停仿真");
  QPushButton * btn_step = MakeActionButton("⏭ Step", "单步推进");
  QPushButton * btn_reset = MakeActionButton("↺ Reset", "重置仿真");
  run_row->addWidget(btn_play);
  run_row->addWidget(btn_pause);
  run_row->addWidget(btn_step);
  run_row->addWidget(btn_reset);
  run_row->addStretch();
  root->addWidget(run_group);

  // 场景加载：路径 + 浏览 + Load
  QGroupBox * scene_group = new QGroupBox("场景加载");
  QHBoxLayout * scene_row = new QHBoxLayout(scene_group);
  scene_path_ = new QLineEdit(QString::fromStdString(dm_.GetLastScenePath()));
  scene_path_->setPlaceholderText("USD / SDF / world 文件路径");
  QPushButton * browse_scene = new QPushButton("浏览…");
  QPushButton * btn_load = new QPushButton("Load Scene");
  btn_load->setMinimumHeight(32);
  scene_row->addWidget(scene_path_, 1);
  scene_row->addWidget(browse_scene);
  scene_row->addWidget(btn_load);
  root->addWidget(scene_group);

  // 后端专属配置（堆叠，只显示当前后端相关项）
  QGroupBox * backend_group = new QGroupBox("后端参数");
  QVBoxLayout * backend_layout = new QVBoxLayout(backend_group);
  backend_stack_ = new QStackedWidget();

  QWidget * ros_page = new QWidget();
  QFormLayout * ros_form = new QFormLayout(ros_page);
  ros_form->setContentsMargins(0, 0, 0, 0);
  control_topic_ = new QLineEdit(QString::fromStdString(dm_.GetControlTopic()));
  control_topic_->setPlaceholderText("/sim/control");
  ros_form->addRow("control topic:", control_topic_);

  QWidget * gazebo_page = new QWidget();
  QFormLayout * gazebo_form = new QFormLayout(gazebo_page);
  gazebo_form->setContentsMargins(0, 0, 0, 0);
  world_name_ = new QLineEdit(QString::fromStdString(dm_.GetWorldName()));
  world_name_->setPlaceholderText("default");
  gazebo_form->addRow("Gazebo world:", world_name_);

  QWidget * isaac_page = new QWidget();
  QFormLayout * isaac_form = new QFormLayout(isaac_page);
  isaac_form->setContentsMargins(0, 0, 0, 0);
  isaac_python_ = new QLineEdit(QString::fromStdString(dm_.GetIsaacPython()));
  isaac_python_->setPlaceholderText("Isaac python.sh，留空自动探测");
  isaac_form->addRow("Isaac Python:", isaac_python_);

  backend_stack_->addWidget(ros_page);
  backend_stack_->addWidget(gazebo_page);
  backend_stack_->addWidget(isaac_page);
  backend_layout->addWidget(backend_stack_);
  root->addWidget(backend_group);

  // 日志
  QGroupBox * log_group = new QGroupBox("输出");
  QVBoxLayout * log_layout = new QVBoxLayout(log_group);
  log_ = new QPlainTextEdit();
  log_->setReadOnly(true);
  log_->setPlaceholderText("命令与发布日志…");
  log_->setMinimumHeight(100);
  log_layout->addWidget(log_);
  root->addWidget(log_group, 1);

  QObject::connect(proc_, &QProcess::readyReadStandardOutput, [this]() {
    log_->appendPlainText(QString::fromUtf8(proc_->readAllStandardOutput()).trimmed());
  });
  QObject::connect(proc_, &QProcess::readyReadStandardError, [this]() {
    log_->appendPlainText(QString::fromUtf8(proc_->readAllStandardError()).trimmed());
  });
  QObject::connect(
    proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    [this](int code) {
      log_->appendPlainText(QString("命令结束 exit=%1").arg(code));
    });

  QObject::connect(backend_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int) {
    updateBackendUi();
  });
  QObject::connect(browse_scene, &QPushButton::clicked, [this]() {
    const QString f = QFileDialog::getOpenFileName(
      this, "选择场景", scene_path_->text(),
      "场景 (*.usd *.usda *.sdf *.world);;所有 (*.*)");
    if (!f.isEmpty()) {
      scene_path_->setText(f);
    }
  });

  auto dispatch = [this, node](SimControlCommand cmd) {
    const auto backend = static_cast<SimControlBackend>(backend_->currentData().toInt());
    if (SimControlUsesRosPublish(backend)) {
      const QString topic = control_topic_->text().trimmed();
      auto pub = node->create_publisher<std_msgs::msg::String>(topic.toStdString(), rclcpp::QoS(10));
      std_msgs::msg::String msg;
      msg.data = SimControlRosPayload(cmd, scene_path_->text()).toStdString();
      pub->publish(msg);
      log_->appendPlainText(QString("发布 [%1]: %2").arg(topic, QString::fromStdString(msg.data)));
    } else {
      sendCommand(cmd);
    }

    dm_.SetControlTopic(control_topic_->text().trimmed().toStdString());
    dm_.SetLastScenePath(scene_path_->text().trimmed().toStdString());
    dm_.SetBackendDefault(
      backend == SimControlBackend::Gazebo ? "gazebo"
                                           : (backend == SimControlBackend::IsaacSim ? "isaac" : "ros_topic"));
    dm_.SetWorldName(world_name_->text().trimmed().toStdString());
    dm_.SetIsaacPython(isaac_python_->text().trimmed().toStdString());
    dm_.Save();
  };

  QObject::connect(btn_play, &QPushButton::clicked, [dispatch]() { dispatch(SimControlCommand::Play); });
  QObject::connect(btn_pause, &QPushButton::clicked, [dispatch]() { dispatch(SimControlCommand::Pause); });
  QObject::connect(btn_step, &QPushButton::clicked, [dispatch]() { dispatch(SimControlCommand::Step); });
  QObject::connect(btn_reset, &QPushButton::clicked, [dispatch]() { dispatch(SimControlCommand::Reset); });
  QObject::connect(btn_load, &QPushButton::clicked, [dispatch]() { dispatch(SimControlCommand::LoadScene); });

  updateBackendUi();
}

SimControlPanelWidget::~SimControlPanelWidget()
{
  if (proc_->state() != QProcess::NotRunning) {
    proc_->terminate();
    proc_->waitForFinished(2000);
  }
}

void SimControlPanelWidget::updateBackendUi()
{
  const int idx = backend_->currentIndex();
  if (backend_stack_) {
    backend_stack_->setCurrentIndex(idx);
  }
}

void SimControlPanelWidget::sendCommand(SimControlCommand cmd)
{
  if (proc_->state() != QProcess::NotRunning) {
    QMessageBox::information(this, "提示", "上一条命令仍在执行");
    return;
  }

  SimControlRequest req;
  req.backend = static_cast<SimControlBackend>(backend_->currentData().toInt());
  req.command = cmd;
  req.control_topic = control_topic_->text().trimmed();
  req.scene_path = scene_path_->text().trimmed();
  req.world_name = world_name_->text().trimmed();
  req.isaac_python = isaac_python_->text().trimmed();

  QString shell_cmd;
  QString err;
  if (!BuildSimControlShellCommand(req, &shell_cmd, &err)) {
    QMessageBox::warning(this, "无法执行", err);
    return;
  }
  log_->appendPlainText("$ " + shell_cmd);
  proc_->setProgram("bash");
  proc_->setArguments({"-lc", shell_cmd});
  proc_->start();
}

}  // namespace ros_robot_workbench::ui
