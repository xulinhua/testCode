#ifndef ROS_ROBOT_WORKBENCH__MODULE__SIM_CONTROL_PANEL_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__SIM_CONTROL_PANEL_MODULE_H_

#include <QString>

namespace ros_robot_workbench::ui
{

enum class SimControlBackend { RosTopic = 0, Gazebo = 1, IsaacSim = 2 };

enum class SimControlCommand { Play = 0, Pause = 1, Step = 2, Reset = 3, LoadScene = 4 };

struct SimControlRequest
{
  SimControlBackend backend = SimControlBackend::RosTopic;
  SimControlCommand command = SimControlCommand::Play;
  QString control_topic = "/sim/control";
  QString scene_path;
  QString world_name = "default";
  QString isaac_python;
};

QString SimControlPanelModuleSummary();

QString SimControlCommandLabel(SimControlCommand cmd);

QString SimControlRosPayload(SimControlCommand cmd, const QString & scene_path);

bool BuildSimControlShellCommand(const SimControlRequest & req, QString * shell_cmd, QString * err_msg);

bool SimControlUsesRosPublish(SimControlBackend backend);

}  // namespace ros_robot_workbench::ui

#endif
