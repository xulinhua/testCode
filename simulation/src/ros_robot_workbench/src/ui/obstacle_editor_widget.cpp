#include "ros_robot_workbench/ui/obstacle_editor_widget.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "ros_robot_workbench/manage/obstacle_editor_data_manager.hpp"
#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/obstacle_editor_module.h"

namespace ros_robot_workbench::ui
{

ObstacleEditorWidget::ObstacleEditorWidget(QWidget * parent)
: QWidget(parent)
{
  manage::ObstacleEditorDataManager dm;
  dm.SetConfigPath(ResolveDefaultConfigYamlPath("obstacle_editor.yaml").toStdString());
  dm.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("障碍编辑");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(ObstacleEditorModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * cfg_group = new QGroupBox("配置");
  QFormLayout * form = new QFormLayout(cfg_group);
  QLineEdit * cfg_path = new QLineEdit(QString::fromStdString(dm.GetConfigPath()));
  cfg_path->setReadOnly(true);
  form->addRow("配置文件:", cfg_path);
  root->addWidget(cfg_group);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  log->setPlaceholderText("模块日志与运行输出…");
  root->addWidget(log, 1);

  QPushButton * refresh = new QPushButton("刷新状态");
  QObject::connect(refresh, &QPushButton::clicked, [log, summary = QString(ObstacleEditorModuleSummary())]() {
    log->appendPlainText(summary);
    log->appendPlainText("模块已加载，可在后续版本接入 ROS 接口与业务逻辑。");
  });
  root->addWidget(refresh);
}

}  // namespace ros_robot_workbench::ui
