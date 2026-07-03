#include "ros_robot_assist_tools/ui/tcp_calibration_widget.h"

#include <vector>

#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include "ros_robot_assist_tools/module/calibration_module.h"
#include "ros_robot_assist_tools/module/tcp_calibration_module.h"

namespace ros_robot_assist_tools::ui
{
namespace
{

void SetFormLeftAligned(QFormLayout * form)
{
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
}

std::vector<FlangePoseSample> ReadPosesFromTable(QTableWidget * table, bool * ok, QString * err_msg)
{
  *ok = false;
  std::vector<FlangePoseSample> poses;
  if (!table) {
    if (err_msg) {
      *err_msg = "位姿表为空";
    }
    return poses;
  }

  for (int row = 0; row < table->rowCount(); ++row) {
    auto cellText = [table, row](int col) -> QString {
      QTableWidgetItem * item = table->item(row, col);
      return item ? item->text().trimmed() : QString();
    };
    const QString sx = cellText(0);
    const QString sy = cellText(1);
    const QString sz = cellText(2);
    const QString sqx = cellText(3);
    const QString sqy = cellText(4);
    const QString sqz = cellText(5);
    const QString sqw = cellText(6);
    if (sx.isEmpty() && sy.isEmpty() && sz.isEmpty()) {
      continue;
    }

    bool g = false;
    FlangePoseSample sample;
    sample.position.x() = sx.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 tx 无效").arg(row + 1);
      }
      return poses;
    }
    sample.position.y() = sy.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 ty 无效").arg(row + 1);
      }
      return poses;
    }
    sample.position.z() = sz.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 tz 无效").arg(row + 1);
      }
      return poses;
    }

    double qx = sqx.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 qx 无效").arg(row + 1);
      }
      return poses;
    }
    double qy = sqy.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 qy 无效").arg(row + 1);
      }
      return poses;
    }
    double qz = sqz.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 qz 无效").arg(row + 1);
      }
      return poses;
    }
    double qw = sqw.isEmpty() ? 1.0 : sqw.toDouble(&g);
    if (!g) {
      if (err_msg) {
        *err_msg = QString("第 %1 行 qw 无效").arg(row + 1);
      }
      return poses;
    }

    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() < 1e-12) {
      if (err_msg) {
        *err_msg = QString("第 %1 行四元数无效").arg(row + 1);
      }
      return poses;
    }
    q.normalize();
    sample.orientation = q;
    poses.push_back(sample);
  }

  *ok = true;
  return poses;
}

void FillTableFromPoses(QTableWidget * table, const std::vector<FlangePoseSample> & poses)
{
  if (!table) {
    return;
  }
  table->setRowCount(static_cast<int>(poses.size()));
  for (int row = 0; row < static_cast<int>(poses.size()); ++row) {
    const auto & p = poses[static_cast<size_t>(row)];
    const auto set = [table, row](int col, const QString & text) {
      table->setItem(row, col, new QTableWidgetItem(text));
    };
    set(0, QString::number(p.position.x(), 'g', 8));
    set(1, QString::number(p.position.y(), 'g', 8));
    set(2, QString::number(p.position.z(), 'g', 8));
    set(3, QString::number(p.orientation.x(), 'g', 8));
    set(4, QString::number(p.orientation.y(), 'g', 8));
    set(5, QString::number(p.orientation.z(), 'g', 8));
    set(6, QString::number(p.orientation.w(), 'g', 8));
  }
}

void AppendEmptyPoseRow(QTableWidget * table)
{
  if (!table) {
    return;
  }
  const int row = table->rowCount();
  table->insertRow(row);
  table->setItem(row, 0, new QTableWidgetItem("0"));
  table->setItem(row, 1, new QTableWidgetItem("0"));
  table->setItem(row, 2, new QTableWidgetItem("0"));
  table->setItem(row, 3, new QTableWidgetItem("0"));
  table->setItem(row, 4, new QTableWidgetItem("0"));
  table->setItem(row, 5, new QTableWidgetItem("0"));
  table->setItem(row, 6, new QTableWidgetItem("1"));
}

double UnitScale(const QString & unit)
{
  return unit.compare("mm", Qt::CaseInsensitive) == 0 ? 0.001 : 1.0;
}

}  // namespace

TcpCalibrationWidget::TcpCalibrationWidget(QWidget * parent)
: QWidget(parent)
, dm_()
{
  const QString default_cfg = ResolveDefaultConfigYamlPath("tcp_calibration.yaml");
  dm_.SetConfigPath(default_cfg.toStdString());
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("TCP 标定");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(
    "在同一物理接触点、至少 4 种不同姿态下采集法兰位姿（基座坐标系），"
    "求解 TCP 在法兰坐标系下的平移偏移 (x, y, z)。");
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * param_group = new QGroupBox("参数");
  QFormLayout * param_form = new QFormLayout(param_group);
  SetFormLeftAligned(param_form);

  QLineEdit * pose_csv = new QLineEdit(QString::fromStdString(dm_.GetPoseCsv()));
  QPushButton * browse_csv = new QPushButton("浏览…");
  QWidget * csv_row = new QWidget();
  QHBoxLayout * csv_layout = new QHBoxLayout(csv_row);
  csv_layout->setContentsMargins(0, 0, 0, 0);
  csv_layout->addWidget(pose_csv, 1);
  csv_layout->addWidget(browse_csv);

  QLineEdit * flange_frame = new QLineEdit(QString::fromStdString(dm_.GetFlangeFrame()));
  QSpinBox * min_poses = new QSpinBox();
  min_poses->setRange(4, 64);
  min_poses->setValue(dm_.GetMinPoses());
  QComboBox * unit_combo = new QComboBox();
  unit_combo->addItems({"m", "mm"});
  unit_combo->setCurrentText(QString::fromStdString(dm_.GetUnit()));

  param_form->addRow("位姿 CSV:", csv_row);
  param_form->addRow("法兰 link:", flange_frame);
  param_form->addRow("最少位姿数:", min_poses);
  param_form->addRow("输入单位:", unit_combo);
  root->addWidget(param_group, 0);

  QGroupBox * pose_group = new QGroupBox("法兰位姿（tx, ty, tz, qx, qy, qz, qw）");
  QVBoxLayout * pose_layout = new QVBoxLayout(pose_group);
  QTableWidget * pose_table = new QTableWidget(0, 7);
  pose_table->setHorizontalHeaderLabels({"tx", "ty", "tz", "qx", "qy", "qz", "qw"});
  pose_table->horizontalHeader()->setStretchLastSection(true);
  pose_layout->addWidget(pose_table);

  QHBoxLayout * pose_btn_row = new QHBoxLayout();
  QPushButton * load_csv_btn = new QPushButton("从 CSV 加载");
  QPushButton * add_row_btn = new QPushButton("添加一行");
  QPushButton * remove_row_btn = new QPushButton("删除选中行");
  pose_btn_row->addWidget(load_csv_btn);
  pose_btn_row->addWidget(add_row_btn);
  pose_btn_row->addWidget(remove_row_btn);
  pose_btn_row->addStretch();
  pose_layout->addLayout(pose_btn_row);
  root->addWidget(pose_group, 1);

  QGroupBox * result_group = new QGroupBox("标定结果");
  QVBoxLayout * result_layout = new QVBoxLayout(result_group);
  QPlainTextEdit * result_text = new QPlainTextEdit();
  result_text->setReadOnly(true);
  result_text->setMaximumHeight(140);
  result_layout->addWidget(result_text);

  QLineEdit * output_yaml = new QLineEdit(DefaultCalibrationYamlPath("TCP标定"));
  QHBoxLayout * out_row = new QHBoxLayout();
  QPushButton * browse_out = new QPushButton("浏览…");
  out_row->addWidget(new QLabel("输出 YAML:"));
  out_row->addWidget(output_yaml, 1);
  out_row->addWidget(browse_out);
  result_layout->addLayout(out_row);
  root->addWidget(result_group, 0);

  QHBoxLayout * action_row = new QHBoxLayout();
  QPushButton * run_btn = new QPushButton("运行标定");
  QPushButton * save_btn = new QPushButton("保存结果");
  action_row->addStretch();
  action_row->addWidget(run_btn);
  action_row->addWidget(save_btn);
  root->addLayout(action_row);

  QObject::connect(browse_csv, &QPushButton::clicked, [this, pose_csv]() {
    const QString f = QFileDialog::getOpenFileName(this, "选择位姿 CSV", pose_csv->text(), "CSV (*.csv);;所有 (*.*)");
    if (!f.isEmpty()) {
      pose_csv->setText(f);
    }
  });

  QObject::connect(browse_out, &QPushButton::clicked, [this, output_yaml]() {
    const QString f = QFileDialog::getSaveFileName(this, "保存 TCP 标定结果", output_yaml->text(), "YAML (*.yaml *.yml);;所有 (*.*)");
    if (!f.isEmpty()) {
      output_yaml->setText(f);
    }
  });

  QObject::connect(load_csv_btn, &QPushButton::clicked, [this, pose_csv, pose_table, unit_combo]() {
    QString err;
    std::vector<FlangePoseSample> poses;
    if (!ParseTcpPosesFromCsv(pose_csv->text(), &poses, &err)) {
      QMessageBox::warning(this, "加载失败", err);
      return;
    }
    const double scale = UnitScale(unit_combo->currentText());
    if (scale != 1.0) {
      for (auto & p : poses) {
        p.position *= scale;
      }
    }
    FillTableFromPoses(pose_table, poses);
  });

  QObject::connect(add_row_btn, &QPushButton::clicked, [pose_table]() {
    AppendEmptyPoseRow(pose_table);
  });

  QObject::connect(remove_row_btn, &QPushButton::clicked, [pose_table]() {
    const int row = pose_table->currentRow();
    if (row >= 0) {
      pose_table->removeRow(row);
    }
  });

  QObject::connect(run_btn, &QPushButton::clicked, [this, pose_table, min_poses, unit_combo, result_text]() {
    bool ok = false;
    QString err;
    auto poses = ReadPosesFromTable(pose_table, &ok, &err);
    if (!ok) {
      QMessageBox::warning(this, "标定失败", err);
      return;
    }

    const double scale = UnitScale(unit_combo->currentText());
    if (scale != 1.0) {
      for (auto & p : poses) {
        p.position *= scale;
      }
    }

    if (static_cast<int>(poses.size()) < min_poses->value()) {
      QMessageBox::warning(
        this, "标定失败",
        QString("至少需要 %1 组位姿，当前 %2 组").arg(min_poses->value()).arg(poses.size()));
      return;
    }

    TcpCalibrationResult result;
    if (!SolveTcpOffsetTranslation(poses, &result, &err)) {
      QMessageBox::warning(this, "标定失败", err);
      return;
    }

    last_result_ = result;
    has_result_ = true;
    result_text->setPlainText(
      QString(
        "TCP 偏移 (法兰系, m):\n  x = %1\n  y = %2\n  z = %3\n"
        "接触点 (基座系, m): [%4, %5, %6]\n"
        "位姿数: %7\nRMS 残差: %8 m\n最大残差: %9 m")
        .arg(result.tcp_offset_flange.x(), 0, 'g', 8)
        .arg(result.tcp_offset_flange.y(), 0, 'g', 8)
        .arg(result.tcp_offset_flange.z(), 0, 'g', 8)
        .arg(result.contact_point_base.x(), 0, 'g', 8)
        .arg(result.contact_point_base.y(), 0, 'g', 8)
        .arg(result.contact_point_base.z(), 0, 'g', 8)
        .arg(result.num_poses)
        .arg(result.rms_residual_m, 0, 'g', 6)
        .arg(result.max_residual_m, 0, 'g', 6));
  });

  QObject::connect(save_btn, &QPushButton::clicked, [this, flange_frame, pose_csv, output_yaml]() {
    if (!has_result_) {
      QMessageBox::information(this, "提示", "请先运行标定");
      return;
    }
    QString err;
    const QString source = pose_csv->text().trimmed().isEmpty() ? "manual_table" : pose_csv->text();
    if (!SaveTcpCalibrationYaml(output_yaml->text(), last_result_, flange_frame->text(), source, &err)) {
      QMessageBox::warning(this, "保存失败", err);
      return;
    }
    QMessageBox::information(this, "保存成功", QString("已写入:\n%1").arg(output_yaml->text()));
  });

  for (int i = 0; i < 4; ++i) {
    AppendEmptyPoseRow(pose_table);
  }
}

}  // namespace ros_robot_assist_tools::ui
