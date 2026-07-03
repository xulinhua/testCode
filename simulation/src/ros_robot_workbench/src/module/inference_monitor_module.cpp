#include "ros_robot_workbench/module/inference_monitor_module.h"

namespace ros_robot_workbench::ui
{

QString InferenceMonitorModuleSummary()
{
  return QStringLiteral("模型推理延迟、FPS 与 GPU 占用。");
}

}  // namespace ros_robot_workbench::ui
