#ifndef ROS_ROBOT_WORKBENCH__MODULE__USD_CONVERTER_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__USD_CONVERTER_MODULE_H_

#include <QString>
#include <QStringList>

namespace ros_robot_workbench::ui
{

enum class UsdInputKind { Urdf = 0, Obj = 1, Stl = 2 };

enum class UsdMeshBackend { OpenUsd = 0, IsaacSim = 1 };

enum class UsdUrdfBackend { UrdfUsdConverter = 0, IsaacSim = 1 };

struct UsdConvertRequest
{
  UsdInputKind input_kind = UsdInputKind::Urdf;
  UsdMeshBackend mesh_backend = UsdMeshBackend::OpenUsd;
  UsdUrdfBackend urdf_backend = UsdUrdfBackend::UrdfUsdConverter;
  QString input_path;
  QString output_usd;
  QString mesh_root_prim = "mesh";
  bool merge_fixed_joints = false;
  bool fix_base = false;
  bool expand_xacro = false;
  QString isaac_python;
  QString python_venv;
};

QString UsdConverterModuleSummary();

QString ResolveWorkbenchScriptPath(const QString & script_name);

QString DefaultUsdOutputPath(const QString & output_dir, const QString & input_path);

bool BuildUsdConvertShellCommand(const UsdConvertRequest & req, QString * shell_cmd, QString * err_msg);

bool ValidateUsdConvertInput(const UsdConvertRequest & req, QString * err_msg);

QString DetectIsaacPython(const QString & configured);

QString DetectPythonVenv(const QString & configured);

}  // namespace ros_robot_workbench::ui

#endif
