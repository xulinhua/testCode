#include "ros_robot_assist_tools/ui/kinematics_solver_widget.h"

#include <QGroupBox>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>

namespace ros_robot_assist_tools::ui
{

KinematicsSolverWidget::KinematicsSolverWidget(QWidget * parent)
: QWidget(parent)
{
  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("运动学计算");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QGroupBox * placeholder_group = new QGroupBox("功能占位");
  QVBoxLayout * placeholder_layout = new QVBoxLayout(placeholder_group);
  QLabel * hint = new QLabel("运动学计算模块骨架已接入，后续可在此补充正逆解与参数配置。");
  hint->setWordWrap(true);
  placeholder_layout->addWidget(hint);
  root->addWidget(placeholder_group);

  QGroupBox * log_group = new QGroupBox("日志");
  QVBoxLayout * log_layout = new QVBoxLayout(log_group);
  QTextEdit * log_view = new QTextEdit();
  log_view->setReadOnly(true);
  log_view->setText("等待实现运动学计算逻辑...");
  log_layout->addWidget(log_view);
  root->addWidget(log_group, 1);
}

}  // namespace ros_robot_assist_tools::ui
