#include "ros_robot_workbench/ui/usd_converter_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>

#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/usd_converter_module.h"

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

}  // namespace

UsdConverterWidget::UsdConverterWidget(QWidget * parent)
: QWidget(parent)
, dm_()
, convert_proc_(new QProcess(this))
{
  dm_.SetConfigPath(ResolveDefaultConfigYamlPath("usd_converter.yaml").toStdString());
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);

  QLabel * title = new QLabel("USD 转换");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(UsdConverterModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * io_group = new QGroupBox("输入 / 输出");
  QFormLayout * io_form = new QFormLayout(io_group);

  input_kind_ = new QComboBox();
  input_kind_->addItem("URDF", static_cast<int>(UsdInputKind::Urdf));
  input_kind_->addItem("OBJ（扫描 mesh）", static_cast<int>(UsdInputKind::Obj));
  input_kind_->addItem("STL / PLY", static_cast<int>(UsdInputKind::Stl));

  backend_ = new QComboBox();

  input_path_ = new QLineEdit();
  QPushButton * browse_in = new QPushButton("浏览…");
  QWidget * in_row = new QWidget();
  QHBoxLayout * in_l = new QHBoxLayout(in_row);
  in_l->setContentsMargins(0, 0, 0, 0);
  in_l->addWidget(input_path_, 1);
  in_l->addWidget(browse_in);

  output_usd_ = new QLineEdit();
  QPushButton * browse_out = new QPushButton("浏览…");
  QWidget * out_row = new QWidget();
  QHBoxLayout * out_l = new QHBoxLayout(out_row);
  out_l->setContentsMargins(0, 0, 0, 0);
  out_l->addWidget(output_usd_, 1);
  out_l->addWidget(browse_out);

  io_form->addRow("输入类型:", input_kind_);
  io_form->addRow("转换方式:", backend_);
  io_form->addRow("输入文件:", in_row);
  io_form->addRow("输出 USD:", out_row);
  root->addWidget(io_group);

  urdf_opts_ = new QGroupBox("URDF 选项");
  QFormLayout * urdf_form = new QFormLayout(urdf_opts_);
  merge_fixed_ = new QCheckBox("merge_fixed_joints");
  merge_fixed_->setChecked(dm_.GetMergeFixedJoints());
  fix_base_ = new QCheckBox("fix_base");
  fix_base_->setChecked(dm_.GetFixBase());
  expand_xacro_ = new QCheckBox("xacro 先展开为 urdf");
  expand_xacro_->setChecked(dm_.GetExpandXacro());
  urdf_form->addRow("", merge_fixed_);
  urdf_form->addRow("", fix_base_);
  urdf_form->addRow("", expand_xacro_);
  root->addWidget(urdf_opts_);

  mesh_opts_ = new QGroupBox("Mesh 选项");
  QFormLayout * mesh_form = new QFormLayout(mesh_opts_);
  mesh_root_prim_ = new QLineEdit(QString::fromStdString(dm_.GetMeshRootPrim()));
  mesh_form->addRow("根 prim 名:", mesh_root_prim_);
  root->addWidget(mesh_opts_);

  QGroupBox * env_group = new QGroupBox("Python 环境（可选）");
  QFormLayout * env_form = new QFormLayout(env_group);
  isaac_python_ = new QLineEdit(QString::fromStdString(dm_.GetIsaacPython()));
  isaac_python_->setPlaceholderText("Isaac Sim 的 python.sh，留空则自动探测");
  python_venv_ = new QLineEdit(QString::fromStdString(dm_.GetPythonVenv()));
  python_venv_->setPlaceholderText("OpenUSD / urdf-usd-converter 用 python，默认 python3 或 venv");
  env_form->addRow("Isaac Python:", isaac_python_);
  env_form->addRow("通用 Python:", python_venv_);
  root->addWidget(env_group);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  log->setMaximumHeight(160);
  root->addWidget(log, 1);

  QHBoxLayout * actions = new QHBoxLayout();
  QPushButton * start = new QPushButton("开始转换");
  QPushButton * stop = new QPushButton("停止");
  QPushButton * open_dir = new QPushButton("打开输出目录");
  actions->addStretch();
  actions->addWidget(open_dir);
  actions->addWidget(stop);
  actions->addWidget(start);
  root->addLayout(actions);

  QObject::connect(convert_proc_, &QProcess::readyReadStandardOutput, [this, log]() {
    log->appendPlainText(QString::fromUtf8(convert_proc_->readAllStandardOutput()).trimmed());
  });
  QObject::connect(convert_proc_, &QProcess::readyReadStandardError, [this, log]() {
    log->appendPlainText(QString::fromUtf8(convert_proc_->readAllStandardError()).trimmed());
  });
  QObject::connect(
    convert_proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    [log](int code) {
      log->appendPlainText(QString("转换结束, exit=%1").arg(code));
    });

  QObject::connect(input_kind_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int) {
    updateBackendOptions();
  });

  QObject::connect(browse_in, &QPushButton::clicked, [this]() {
    const auto kind = static_cast<UsdInputKind>(input_kind_->currentData().toInt());
    QString filter = "所有 (*.*)";
    if (kind == UsdInputKind::Urdf) {
      filter = "URDF/Xacro (*.urdf *.xacro);;所有 (*.*)";
    } else if (kind == UsdInputKind::Obj) {
      filter = "OBJ (*.obj);;所有 (*.*)";
    } else {
      filter = "Mesh (*.stl *.ply);;所有 (*.*)";
    }
    const QString f = QFileDialog::getOpenFileName(this, "选择输入文件", input_path_->text(), filter);
    if (!f.isEmpty()) {
      input_path_->setText(f);
      if (output_usd_->text().trimmed().isEmpty()) {
        output_usd_->setText(DefaultUsdOutputPath(
          ExpandHome(QString::fromStdString(dm_.GetDefaultOutputDir())), f));
      }
    }
  });

  QObject::connect(browse_out, &QPushButton::clicked, [this]() {
    const QString f = QFileDialog::getSaveFileName(
      this, "输出 USD", output_usd_->text(), "USD (*.usd *.usda);;所有 (*.*)");
    if (!f.isEmpty()) {
      output_usd_->setText(f);
    }
  });

  QObject::connect(open_dir, &QPushButton::clicked, [this]() {
    const QString dir = QFileInfo(output_usd_->text().trimmed()).absolutePath();
    if (dir.isEmpty()) {
      return;
    }
    QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
  });

  QObject::connect(start, &QPushButton::clicked, [this, log]() {
    if (convert_proc_->state() != QProcess::NotRunning) {
      QMessageBox::information(this, "提示", "转换任务进行中");
      return;
    }
    UsdConvertRequest req = buildRequest();
    QString shell_cmd;
    QString err;
    if (!BuildUsdConvertShellCommand(req, &shell_cmd, &err)) {
      QMessageBox::warning(this, "无法启动", err);
      return;
    }
    log->appendPlainText("$ " + shell_cmd);
    convert_proc_->setProgram("bash");
    convert_proc_->setArguments({"-lc", shell_cmd});
    convert_proc_->start();
  });

  QObject::connect(stop, &QPushButton::clicked, [this, log]() {
    if (convert_proc_->state() != QProcess::NotRunning) {
      convert_proc_->terminate();
      log->appendPlainText("已请求停止");
    }
  });

  updateBackendOptions();
}

UsdConverterWidget::~UsdConverterWidget()
{
  if (convert_proc_->state() != QProcess::NotRunning) {
    convert_proc_->terminate();
    convert_proc_->waitForFinished(3000);
  }
}

void UsdConverterWidget::updateBackendOptions()
{
  const auto kind = static_cast<UsdInputKind>(input_kind_->currentData().toInt());
  backend_->clear();
  if (kind == UsdInputKind::Urdf) {
    backend_->addItem("urdf-usd-converter（OpenUSD，轻量）", static_cast<int>(UsdUrdfBackend::UrdfUsdConverter));
    backend_->addItem("Isaac Sim URDF Importer", static_cast<int>(UsdUrdfBackend::IsaacSim));
    const QString def = QString::fromStdString(dm_.GetUrdfBackendDefault());
    backend_->setCurrentIndex(def == "isaac" ? 1 : 0);
    urdf_opts_->setVisible(true);
    mesh_opts_->setVisible(false);
  } else {
    backend_->addItem("OpenUSD（pxr / trimesh）", static_cast<int>(UsdMeshBackend::OpenUsd));
    backend_->setCurrentIndex(0);
    urdf_opts_->setVisible(false);
    mesh_opts_->setVisible(true);
  }
}

UsdConvertRequest UsdConverterWidget::buildRequest() const
{
  UsdConvertRequest req;
  req.input_kind = static_cast<UsdInputKind>(input_kind_->currentData().toInt());
  req.input_path = input_path_->text().trimmed();
  req.output_usd = output_usd_->text().trimmed();
  req.mesh_root_prim = mesh_root_prim_->text().trimmed();
  req.merge_fixed_joints = merge_fixed_->isChecked();
  req.fix_base = fix_base_->isChecked();
  req.expand_xacro = expand_xacro_->isChecked();
  req.isaac_python = isaac_python_->text().trimmed();
  req.python_venv = python_venv_->text().trimmed();
  if (req.input_kind == UsdInputKind::Urdf) {
    req.urdf_backend = static_cast<UsdUrdfBackend>(backend_->currentData().toInt());
  } else {
    req.mesh_backend = static_cast<UsdMeshBackend>(backend_->currentData().toInt());
  }
  return req;
}

}  // namespace ros_robot_workbench::ui
