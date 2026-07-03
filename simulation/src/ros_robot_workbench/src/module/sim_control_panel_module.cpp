#include "ros_robot_workbench/module/sim_control_panel_module.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace ros_robot_workbench::ui
{
namespace
{

QString ShellQuote(QString s)
{
  s.replace("'", "'\\''");
  return QString("'%1'").arg(s);
}

QString ResolveWorkbenchScriptPath(const QString & script_name)
{
  const QString file = script_name.trimmed();
  if (file.isEmpty()) {
    return {};
  }
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    QDir::current().filePath(QString("simulation/src/ros_robot_workbench/scripts/%1").arg(file)),
    QDir::current().filePath(QString("src/ros_robot_workbench/scripts/%1").arg(file)),
    QDir(app_dir).filePath(QString("../../share/ros_robot_workbench/scripts/%1").arg(file)),
    QDir(app_dir).filePath(QString("../share/ros_robot_workbench/scripts/%1").arg(file)),
  };
  for (const QString & p : candidates) {
    const QString canon = QFileInfo(p).canonicalFilePath();
    if (!canon.isEmpty() && QFileInfo(canon).isFile()) {
      return canon;
    }
  }
  return candidates.front();
}

QString DetectIsaacPython(const QString & configured)
{
  const QString cfg = configured.trimmed();
  if (!cfg.isEmpty() && QFileInfo(cfg).isExecutable()) {
    return cfg;
  }
  const QStringList candidates = {
    QDir::homePath() + "/isaacsim/python.sh",
    "/isaac-sim/python.sh",
  };
  for (const QString & c : candidates) {
    if (QFileInfo(c).isExecutable()) {
      return c;
    }
  }
  return cfg;
}

QString CommandToken(SimControlCommand cmd)
{
  switch (cmd) {
    case SimControlCommand::Play:
      return "play";
    case SimControlCommand::Pause:
      return "pause";
    case SimControlCommand::Step:
      return "step";
    case SimControlCommand::Reset:
      return "reset";
    case SimControlCommand::LoadScene:
      return "load_scene";
  }
  return "play";
}

}  // namespace

QString SimControlPanelModuleSummary()
{
  return QStringLiteral(
    "仿真启停/步进/重置与场景加载。后端可选：ROS 话题（通用）、Gazebo 服务、Isaac Sim 脚本。"
    "ROS/Isaac bridge 侧需订阅 control topic 并解析 play/pause/step/reset/load_scene:路径。");
}

QString SimControlCommandLabel(SimControlCommand cmd)
{
  switch (cmd) {
    case SimControlCommand::Play:
      return QStringLiteral("Play");
    case SimControlCommand::Pause:
      return QStringLiteral("Pause");
    case SimControlCommand::Step:
      return QStringLiteral("Step");
    case SimControlCommand::Reset:
      return QStringLiteral("Reset");
    case SimControlCommand::LoadScene:
      return QStringLiteral("Load Scene");
  }
  return {};
}

QString SimControlRosPayload(SimControlCommand cmd, const QString & scene_path)
{
  const QString token = CommandToken(cmd);
  if (cmd == SimControlCommand::LoadScene) {
    return token + ":" + scene_path.trimmed();
  }
  return token;
}

bool SimControlUsesRosPublish(SimControlBackend backend)
{
  return backend == SimControlBackend::RosTopic;
}

bool BuildSimControlShellCommand(const SimControlRequest & req, QString * shell_cmd, QString * err_msg)
{
  if (!shell_cmd) {
    if (err_msg) {
      *err_msg = "shell_cmd is null";
    }
    return false;
  }

  if (req.backend == SimControlBackend::Gazebo) {
    const QString world = req.world_name.trimmed().isEmpty() ? "default" : req.world_name.trimmed();
    switch (req.command) {
      case SimControlCommand::Play:
        *shell_cmd = QString(
          "gz service -s /world/%1/control --reqtype gz.msgs.WorldControl "
          "--reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'")
                       .arg(world);
        return true;
      case SimControlCommand::Pause:
        *shell_cmd = QString(
          "gz service -s /world/%1/control --reqtype gz.msgs.WorldControl "
          "--reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
                       .arg(world);
        return true;
      case SimControlCommand::Reset:
        *shell_cmd = QString(
          "gz service -s /world/%1/control --reqtype gz.msgs.WorldControl "
          "--reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'")
                       .arg(world);
        return true;
      case SimControlCommand::Step:
        if (err_msg) {
          *err_msg = "Gazebo 暂不支持单步，请用 Pause + Step 组合或切换 ROS 后端";
        }
        return false;
      case SimControlCommand::LoadScene:
        if (req.scene_path.trimmed().isEmpty()) {
          if (err_msg) {
            *err_msg = "请填写场景/world 文件路径";
          }
          return false;
        }
        *shell_cmd = QString("gz sim -r %1").arg(ShellQuote(req.scene_path.trimmed()));
        return true;
    }
  }

  if (req.backend == SimControlBackend::IsaacSim) {
    const QString isaac_py = DetectIsaacPython(req.isaac_python);
    if (isaac_py.trimmed().isEmpty() || !QFileInfo(isaac_py).isExecutable()) {
      if (err_msg) {
        *err_msg = "未找到 Isaac python.sh，请填写路径";
      }
      return false;
    }
    const QString script = ResolveWorkbenchScriptPath("sim_control_isaac.py");
    if (!QFileInfo(script).isFile()) {
      if (err_msg) {
        *err_msg = "缺少脚本 sim_control_isaac.py";
      }
      return false;
    }
    QStringList parts;
    parts << ShellQuote(isaac_py) << ShellQuote(script) << "--cmd" << ShellQuote(CommandToken(req.command));
    if (req.command == SimControlCommand::LoadScene) {
      if (req.scene_path.trimmed().isEmpty()) {
        if (err_msg) {
          *err_msg = "请填写 USD 场景路径";
        }
        return false;
      }
      parts << "--usd" << ShellQuote(req.scene_path.trimmed());
    }
    *shell_cmd = parts.join(" ");
    return true;
  }

  if (err_msg) {
    *err_msg = "ROS 后端请直接发布话题";
  }
  return false;
}

}  // namespace ros_robot_workbench::ui
