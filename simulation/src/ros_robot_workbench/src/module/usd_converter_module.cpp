#include "ros_robot_workbench/module/usd_converter_module.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>

namespace ros_robot_workbench::ui
{
namespace
{

QString ExpandHome(QString path)
{
  if (path.startsWith("~/")) {
    return QDir::homePath() + path.mid(1);
  }
  return path;
}

QString ShellQuote(QString s)
{
  s.replace("'", "'\\''");
  return QString("'%1'").arg(s);
}

QString InputExtensionLower(const QString & path)
{
  return QFileInfo(path).suffix().toLower();
}

}  // namespace

QString UsdConverterModuleSummary()
{
  return QStringLiteral(
    "URDF / OBJ / STL 转 USD。Mesh 可选 OpenUSD 或 Isaac Sim；"
    "URDF 可选 urdf-usd-converter 或 Isaac Sim。");
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

QString DefaultUsdOutputPath(const QString & output_dir, const QString & input_path)
{
  const QString base = ExpandHome(output_dir.trimmed().isEmpty()
      ? QDir::homePath() + "/.ros_robot_workbench/usd"
      : output_dir.trimmed());
  QDir().mkpath(base);
  const QString stem = QFileInfo(input_path).completeBaseName();
  const QString safe = stem.isEmpty() ? "asset" : stem;
  return QDir(base).filePath(safe + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".usd");
}

QString DetectIsaacPython(const QString & configured)
{
  const QString cfg = configured.trimmed();
  if (!cfg.isEmpty() && QFileInfo(cfg).isExecutable()) {
    return cfg;
  }
  const QStringList candidates = {
    QDir::homePath() + "/isaacsim/python.sh",
    QDir::homePath() + "/.local/share/ov/pkg/isaac_sim-*/python.sh",
    "/isaac-sim/python.sh",
  };
  for (const QString & c : candidates) {
    if (c.contains('*')) {
      continue;
    }
    if (QFileInfo(c).isExecutable()) {
      return c;
    }
  }
  return cfg;
}

QString DetectPythonVenv(const QString & configured)
{
  const QString cfg = configured.trimmed();
  if (!cfg.isEmpty() && QFileInfo(cfg).isExecutable()) {
    return cfg;
  }
  const QString venv_py = ExpandHome("~/.ros_robot_workbench/venv-usd/bin/python");
  if (QFileInfo(venv_py).isExecutable()) {
    return venv_py;
  }
  return "python3";
}

bool ValidateUsdConvertInput(const UsdConvertRequest & req, QString * err_msg)
{
  const QString input = req.input_path.trimmed();
  if (input.isEmpty()) {
    if (err_msg) {
      *err_msg = "请输入输入文件路径";
    }
    return false;
  }
  if (!QFileInfo(input).isFile()) {
    if (err_msg) {
      *err_msg = "输入文件不存在";
    }
    return false;
  }
  const QString ext = InputExtensionLower(input);
  if (req.input_kind == UsdInputKind::Urdf) {
    if (ext != "urdf" && ext != "xacro") {
      if (err_msg) {
        *err_msg = "URDF 模式需要 .urdf 或 .xacro 文件";
      }
      return false;
    }
  } else if (req.input_kind == UsdInputKind::Obj) {
    if (ext != "obj") {
      if (err_msg) {
        *err_msg = "OBJ 模式需要 .obj 文件";
      }
      return false;
    }
  } else if (req.input_kind == UsdInputKind::Stl) {
    if (ext != "stl" && ext != "ply") {
      if (err_msg) {
        *err_msg = "Mesh 模式需要 .stl 或 .ply 文件";
      }
      return false;
    }
  }
  if (req.output_usd.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "请指定输出 USD 路径";
    }
    return false;
  }
  return true;
}

bool BuildUsdConvertShellCommand(const UsdConvertRequest & req, QString * shell_cmd, QString * err_msg)
{
  if (!shell_cmd) {
    if (err_msg) {
      *err_msg = "shell_cmd is null";
    }
    return false;
  }
  if (!ValidateUsdConvertInput(req, err_msg)) {
    return false;
  }

  const QString input = QFileInfo(req.input_path.trimmed()).absoluteFilePath();
  const QString output = QFileInfo(req.output_usd.trimmed()).absoluteFilePath();
  QDir().mkpath(QFileInfo(output).absolutePath());

  QString urdf_input = input;
  QString prelude;
  if (req.input_kind == UsdInputKind::Urdf && req.expand_xacro && InputExtensionLower(input) == "xacro") {
    const QString tmp = QDir::temp().filePath(
      QString("workbench_xacro_%1.urdf").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")));
    prelude = QString("xacro %1 -o %2 && ").arg(ShellQuote(input), ShellQuote(tmp));
    urdf_input = tmp;
  }

  if (req.input_kind == UsdInputKind::Urdf) {
    if (req.urdf_backend == UsdUrdfBackend::IsaacSim) {
      const QString isaac_py = DetectIsaacPython(req.isaac_python);
      if (isaac_py.trimmed().isEmpty() || !QFileInfo(isaac_py).isExecutable()) {
        if (err_msg) {
          *err_msg = "未找到 Isaac Python，请在配置或界面中填写 isaac_python 路径";
        }
        return false;
      }
      const QString script = ResolveWorkbenchScriptPath("urdf_to_usd_isaac.py");
      if (!QFileInfo(script).isFile()) {
        if (err_msg) {
          *err_msg = "缺少脚本 urdf_to_usd_isaac.py";
        }
        return false;
      }
      QStringList parts;
      parts << ShellQuote(isaac_py) << ShellQuote(script);
      if (req.merge_fixed_joints) {
        parts << "--merge-fixed-joints";
      }
      if (req.fix_base) {
        parts << "--fix-base";
      }
      parts << "--urdf" << ShellQuote(urdf_input) << "--usd" << ShellQuote(output);
      *shell_cmd = prelude + parts.join(" ");
      return true;
    }

    const QString py = DetectPythonVenv(req.python_venv);
    const QString out_arg = output.endsWith(".usd") || output.endsWith(".usda") || output.endsWith(".usdc")
      ? output
      : output + ".usd";
    *shell_cmd = prelude + QString("%1 -m urdf_usd_converter %2 %3")
                           .arg(ShellQuote(py), ShellQuote(urdf_input), ShellQuote(out_arg));
    return true;
  }

  if (req.mesh_backend == UsdMeshBackend::IsaacSim) {
    if (err_msg) {
      *err_msg = "Isaac Sim 后端当前仅支持 URDF；Mesh 请选 OpenUSD";
    }
    return false;
  }

  const QString py = DetectPythonVenv(req.python_venv);
  const QString script = ResolveWorkbenchScriptPath("mesh_to_usd_openusd.py");
  if (!QFileInfo(script).isFile()) {
    if (err_msg) {
      *err_msg = "缺少脚本 mesh_to_usd_openusd.py";
    }
    return false;
  }
  *shell_cmd = QString("%1 %2 --input %3 --usd %4 --root-prim %5")
                 .arg(ShellQuote(py), ShellQuote(script), ShellQuote(input), ShellQuote(output),
                   ShellQuote(req.mesh_root_prim.trimmed().isEmpty() ? "mesh" : req.mesh_root_prim));
  return true;
}

}  // namespace ros_robot_workbench::ui
