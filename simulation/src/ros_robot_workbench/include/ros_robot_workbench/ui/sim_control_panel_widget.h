#ifndef ROS_ROBOT_WORKBENCH__UI__SIM_CONTROL_PANEL_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__SIM_CONTROL_PANEL_WIDGET_H_

#include <QWidget>

#include "ros_robot_workbench/manage/sim_control_panel_data_manager.hpp"
#include "ros_robot_workbench/module/sim_control_panel_module.h"

class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QProcess;
class QStackedWidget;

namespace ros_robot_workbench::ui
{

class SimControlPanelWidget : public QWidget
{
public:
  explicit SimControlPanelWidget(QWidget * parent = nullptr);
  ~SimControlPanelWidget() override;

private:
  void updateBackendUi();
  void sendCommand(SimControlCommand cmd);

  manage::SimControlPanelDataManager dm_;
  QProcess * proc_ = nullptr;
  QComboBox * backend_ = nullptr;
  QStackedWidget * backend_stack_ = nullptr;
  QLineEdit * control_topic_ = nullptr;
  QLineEdit * world_name_ = nullptr;
  QLineEdit * scene_path_ = nullptr;
  QLineEdit * isaac_python_ = nullptr;
  QPlainTextEdit * log_ = nullptr;
};

}  // namespace ros_robot_workbench::ui

#endif
