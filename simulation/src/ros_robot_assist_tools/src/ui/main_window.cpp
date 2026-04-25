#include "ros_robot_assist_tools/ui/main_window.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QStatusBar>
#include <QVBoxLayout>

#include "ros_robot_assist_tools/ui/board_generator_widget.h"
#include "ros_robot_assist_tools/ui/handeye_calibration_widget.h"
#include "ros_robot_assist_tools/ui/intrinsic_calibration_widget.h"
#include "ros_robot_assist_tools/ui/kinematics_solver_widget.h"
#include "ros_robot_assist_tools/ui/multi_sensor_calibration_widget.h"
#include "ros_robot_assist_tools/ui/pose_transform_widget.h"
#include "ros_robot_assist_tools/ui/stereo_calibration_widget.h"
#include "ros_robot_assist_tools/ui/system_status_widget.h"

namespace ros_robot_assist_tools::ui
{

QMainWindow * CreateMainWindow()
{
  QMainWindow * window = new QMainWindow();
  window->setWindowTitle("Assist Tool");
  window->resize(1400, 880);
  QWidget * central = new QWidget(window);
  window->setCentralWidget(central);
  QHBoxLayout * root = new QHBoxLayout(central);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);
  QWidget * nav = new QWidget();
  nav->setFixedWidth(220);
  nav->setStyleSheet("background:#1f3342;");
  QVBoxLayout * nav_layout = new QVBoxLayout(nav);
  nav_layout->setContentsMargins(14, 18, 14, 18);
  nav_layout->setSpacing(8);
  QStackedWidget * stack = new QStackedWidget();
  stack->setStyleSheet("background:#f7fafc;");

  auto * system_status_widget = new SystemStatusWidget();
  stack->addWidget(system_status_widget);
  stack->addWidget(new BoardGeneratorWidget());
  stack->addWidget(new PoseTransformWidget());
  stack->addWidget(new KinematicsSolverWidget());
  stack->addWidget(new IntrinsicCalibrationWidget());
  stack->addWidget(new StereoCalibrationWidget());
  stack->addWidget(new MultiSensorCalibrationWidget());
  stack->addWidget(new HandeyeCalibrationWidget());

  const QStringList items = {"系统状态", "标定板生成", "姿态转换", "运动学计算", "内参标定", "双目标定", "多传感器标定", "手眼标定"};
  for (int i = 0; i < items.size(); ++i) {
    QPushButton * btn = new QPushButton(items[i]);
    btn->setFixedHeight(40);
    btn->setStyleSheet(
      "QPushButton{background:#2e4557; color:#f6fbff; border:none; border-radius:6px; text-align:left; padding-left:12px;}"
      "QPushButton:hover{background:#3d627a;}");
    QObject::connect(btn, &QPushButton::clicked, [stack, i]() { stack->setCurrentIndex(i); });
    nav_layout->addWidget(btn);
  }
  nav_layout->addStretch();
  QObject::connect(stack, QOverload<int>::of(&QStackedWidget::currentChanged), [system_status_widget](int index) {
    system_status_widget->SetActive(index == 0);
  });
  system_status_widget->SetActive(true);
  root->addWidget(nav);
  root->addWidget(stack, 1);
  window->statusBar()->addPermanentWidget(new QLabel("ROS2 Humble | C++17 | Qt"));
  return window;
}

}  // namespace ros_robot_assist_tools::ui
