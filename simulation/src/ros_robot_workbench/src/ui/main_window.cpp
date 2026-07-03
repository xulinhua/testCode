#include "ros_robot_workbench/ui/main_window.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QStatusBar>
#include <QVBoxLayout>

#include "ros_robot_workbench/ui/board_generator_widget.h"
#include "ros_robot_workbench/ui/handeye_calibration_widget.h"
#include "ros_robot_workbench/ui/image_viewer_widget.h"
#include "ros_robot_workbench/ui/intrinsic_calibration_widget.h"
#include "ros_robot_workbench/ui/lazy_feature_page.hpp"
#include "ros_robot_workbench/ui/kinematics_solver_widget.h"
#include "ros_robot_workbench/ui/multi_sensor_calibration_widget.h"
#include "ros_robot_workbench/ui/pose_transform_widget.h"
#include "ros_robot_workbench/ui/stereo_calibration_widget.h"
#include "ros_robot_workbench/ui/system_status_widget.h"

namespace ros_robot_workbench::ui
{

QMainWindow * CreateMainWindow()
{
  QMainWindow * window = new QMainWindow();
  window->setWindowTitle("Robot Workbench");
  window->resize(1400, 760);
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
  auto * lazy_image = new LazyFeaturePage([](QWidget * p) { return new ImageViewerWidget(p); });
  auto * lazy_board = new LazyFeaturePage([](QWidget * p) { return new BoardGeneratorWidget(p); });
  auto * lazy_pose = new LazyFeaturePage([](QWidget * p) { return new PoseTransformWidget(p); });
  auto * lazy_kin = new LazyFeaturePage([](QWidget * p) { return new KinematicsSolverWidget(p); });
  auto * lazy_intrinsic = new LazyFeaturePage([](QWidget * p) { return new IntrinsicCalibrationWidget(p); });
  auto * lazy_stereo = new LazyFeaturePage([](QWidget * p) { return new StereoCalibrationWidget(p); });
  auto * lazy_multi = new LazyFeaturePage([](QWidget * p) { return new MultiSensorCalibrationWidget(p); });
  auto * lazy_handeye = new LazyFeaturePage([](QWidget * p) { return new HandeyeCalibrationWidget(p); });
  stack->addWidget(system_status_widget);
  stack->addWidget(lazy_image);
  stack->addWidget(lazy_board);
  stack->addWidget(lazy_pose);
  stack->addWidget(lazy_kin);
  stack->addWidget(lazy_intrinsic);
  stack->addWidget(lazy_stereo);
  stack->addWidget(lazy_multi);
  stack->addWidget(lazy_handeye);

  const QStringList items = {"系统状态", "图像查看", "标定板生成", "姿态转换", "运动学计算", "内参标定", "双目标定", "多传感器标定", "手眼标定"};
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
  auto sync_image_active = [lazy_image](bool on) {
    if (on) {
      lazy_image->ensureBuilt();
    }
    if (auto * iv = dynamic_cast<ImageViewerWidget *>(lazy_image->content())) {
      iv->SetActive(on);
    }
  };
  QObject::connect(stack, QOverload<int>::of(&QStackedWidget::currentChanged), [system_status_widget, sync_image_active](int index) {
    system_status_widget->SetActive(index == 0);
    sync_image_active(index == 1);
  });
  system_status_widget->SetActive(true);
  sync_image_active(false);
  root->addWidget(nav);
  root->addWidget(stack, 1);
  window->statusBar()->addPermanentWidget(new QLabel("ROS2 Humble | C++17 | Qt"));
  return window;
}

}  // namespace ros_robot_workbench::ui
