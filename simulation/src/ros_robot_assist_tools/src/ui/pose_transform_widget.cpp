#include "ros_robot_assist_tools/ui/pose_transform_widget.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSizePolicy>
#include <QTextEdit>
#include <QVBoxLayout>

namespace ros_robot_assist_tools::ui
{

PoseTransformWidget::PoseTransformWidget(QWidget * parent)
: QWidget(parent)
{
  setStyleSheet("QGroupBox{font-size:16px;font-weight:600;} QPlainTextEdit{font-family:monospace;}");
  QVBoxLayout * layout = new QVBoxLayout(this);

  QGroupBox * input_group = new QGroupBox("Input");
  QVBoxLayout * input_layout = new QVBoxLayout(input_group);
  input_layout->setAlignment(Qt::AlignTop);

  QWidget * in_angle_fmt = new QWidget();
  QHBoxLayout * in_angle_fmt_layout = new QHBoxLayout(in_angle_fmt);
  in_angle_fmt_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * in_angle_label = new QLabel("Input angle format");
  QRadioButton * in_rad = new QRadioButton("Radians");
  QRadioButton * in_deg = new QRadioButton("Degrees");
  in_rad->setChecked(true);
  in_angle_fmt_layout->addWidget(in_angle_label);
  in_angle_fmt_layout->addWidget(in_rad);
  in_angle_fmt_layout->addWidget(in_deg);
  in_angle_fmt_layout->addStretch();
  input_layout->addWidget(in_angle_fmt);

  QWidget * input_type_row = new QWidget();
  QHBoxLayout * input_type_layout = new QHBoxLayout(input_type_row);
  input_type_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * input_type_label = new QLabel("Input type");
  QComboBox * input_type_combo = new QComboBox();
  input_type_combo->addItems({
    "Rotation matrix", "Quaternion", "Axis-angle",
    "Axis with angle magnitude", "Euler angles", "Triple points P/Q/R"
  });
  input_type_combo->setCurrentIndex(4);
  input_type_layout->addWidget(input_type_label);
  input_type_layout->addWidget(input_type_combo);
  input_type_layout->addStretch();
  input_layout->addWidget(input_type_row);

  QGroupBox * rotm_in_group = new QGroupBox("Rotation matrix");
  QGridLayout * rotm_grid = new QGridLayout(rotm_in_group);
  std::vector<QLineEdit *> rotm_in(9);
  for (int i = 0; i < 9; ++i) {
    rotm_in[i] = new QLineEdit((i % 4 == 0) ? "1" : "0");
    rotm_in[i]->setFixedWidth(88);
    rotm_grid->addWidget(rotm_in[i], i / 3, i % 3);
  }
  input_layout->addWidget(rotm_in_group);

  QGroupBox * quat_in_group = new QGroupBox("Quaternion");
  QHBoxLayout * quat_in_layout = new QHBoxLayout(quat_in_group);
  QLineEdit * qx_in = new QLineEdit("0");
  QLineEdit * qy_in = new QLineEdit("0");
  QLineEdit * qz_in = new QLineEdit("0");
  QLineEdit * qw_in = new QLineEdit("1");
  qx_in->setFixedWidth(88);
  qy_in->setFixedWidth(88);
  qz_in->setFixedWidth(88);
  qw_in->setFixedWidth(88);
  quat_in_layout->addWidget(new QLabel("x"));
  quat_in_layout->addWidget(qx_in);
  quat_in_layout->addWidget(new QLabel("y"));
  quat_in_layout->addWidget(qy_in);
  quat_in_layout->addWidget(new QLabel("z"));
  quat_in_layout->addWidget(qz_in);
  quat_in_layout->addWidget(new QLabel("w (real part)"));
  quat_in_layout->addWidget(qw_in);
  quat_in_layout->addStretch();
  input_layout->addWidget(quat_in_group);

  QGroupBox * axis_angle_group = new QGroupBox("Axis-angle");
  QHBoxLayout * axis_angle_layout = new QHBoxLayout(axis_angle_group);
  QLineEdit * ax_in = new QLineEdit("0");
  QLineEdit * ay_in = new QLineEdit("0");
  QLineEdit * az_in = new QLineEdit("0");
  QLineEdit * ang_in = new QLineEdit("0");
  ax_in->setFixedWidth(88);
  ay_in->setFixedWidth(88);
  az_in->setFixedWidth(88);
  ang_in->setFixedWidth(88);
  axis_angle_layout->addWidget(new QLabel("Axis x"));
  axis_angle_layout->addWidget(ax_in);
  axis_angle_layout->addWidget(new QLabel("y"));
  axis_angle_layout->addWidget(ay_in);
  axis_angle_layout->addWidget(new QLabel("z"));
  axis_angle_layout->addWidget(az_in);
  axis_angle_layout->addWidget(new QLabel("Angle (radians)"));
  axis_angle_layout->addWidget(ang_in);
  axis_angle_layout->addStretch();
  input_layout->addWidget(axis_angle_group);

  QGroupBox * axis_mag_group = new QGroupBox("Axis with angle magnitude (radians)");
  QHBoxLayout * axis_mag_layout = new QHBoxLayout(axis_mag_group);
  QLineEdit * amx_in = new QLineEdit("0");
  QLineEdit * amy_in = new QLineEdit("0");
  QLineEdit * amz_in = new QLineEdit("0");
  amx_in->setFixedWidth(88);
  amy_in->setFixedWidth(88);
  amz_in->setFixedWidth(88);
  axis_mag_layout->addWidget(new QLabel("Axis x"));
  axis_mag_layout->addWidget(amx_in);
  axis_mag_layout->addWidget(new QLabel("y"));
  axis_mag_layout->addWidget(amy_in);
  axis_mag_layout->addWidget(new QLabel("z"));
  axis_mag_layout->addWidget(amz_in);
  axis_mag_layout->addStretch();
  input_layout->addWidget(axis_mag_group);

  QGroupBox * euler_in_group = new QGroupBox("Euler angles of multiple axis rotations (radians)");
  QHBoxLayout * euler_in_layout = new QHBoxLayout(euler_in_group);
  QComboBox * euler_order_in = new QComboBox();
  euler_order_in->addItems({"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"});
  euler_order_in->setCurrentText("ZYX");
  QLineEdit * ex_in = new QLineEdit("0");
  QLineEdit * ey_in = new QLineEdit("0");
  QLineEdit * ez_in = new QLineEdit("0");
  ex_in->setFixedWidth(88);
  ey_in->setFixedWidth(88);
  ez_in->setFixedWidth(88);
  euler_in_layout->addWidget(euler_order_in);
  euler_in_layout->addWidget(new QLabel("x"));
  euler_in_layout->addWidget(ex_in);
  euler_in_layout->addWidget(new QLabel("y"));
  euler_in_layout->addWidget(ey_in);
  euler_in_layout->addWidget(new QLabel("z"));
  euler_in_layout->addWidget(ez_in);
  euler_in_layout->addStretch();
  input_layout->addWidget(euler_in_group);

  QGroupBox * pqr_group = new QGroupBox("Triple of points, P, Q, R, such that X // (Q−P), Z // X × (R−P), and Y // Z × X.");
  QGridLayout * pqr_layout = new QGridLayout(pqr_group);
  std::vector<QLineEdit *> pqr_in(9);
  const QStringList labels = {"P:", "Q:", "R:"};
  const QStringList xyz = {"x", "y", "z"};
  const double defaults[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
  for (int r = 0; r < 3; ++r) {
    pqr_layout->addWidget(new QLabel(labels[r]), r, 0);
    for (int c = 0; c < 3; ++c) {
      pqr_layout->addWidget(new QLabel(xyz[c]), r, c * 2 + 1);
      const int idx = r * 3 + c;
      pqr_in[idx] = new QLineEdit(QString::number(defaults[idx]));
      pqr_in[idx]->setFixedWidth(70);
      pqr_layout->addWidget(pqr_in[idx], r, c * 2 + 2);
    }
  }
  input_layout->addWidget(pqr_group);

  layout->addWidget(input_group);

  QHBoxLayout * btn_layout = new QHBoxLayout();
  QPushButton * refresh_btn = new QPushButton("刷新输出");
  QPushButton * from_euler_btn = new QPushButton("使用Euler更新四元数");
  btn_layout->addWidget(refresh_btn);
  btn_layout->addWidget(from_euler_btn);
  btn_layout->addStretch();
  layout->addLayout(btn_layout);

  QGroupBox * output_group = new QGroupBox("Output");
  QVBoxLayout * output_layout = new QVBoxLayout(output_group);
  output_layout->setAlignment(Qt::AlignTop);

  QWidget * out_angle_fmt = new QWidget();
  QHBoxLayout * out_angle_fmt_layout = new QHBoxLayout(out_angle_fmt);
  out_angle_fmt_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * out_angle_label = new QLabel("Output angle format");
  QRadioButton * out_rad = new QRadioButton("Radians");
  QRadioButton * out_deg = new QRadioButton("Degrees");
  out_rad->setChecked(true);
  out_angle_fmt_layout->addWidget(out_angle_label);
  out_angle_fmt_layout->addWidget(out_rad);
  out_angle_fmt_layout->addWidget(out_deg);
  out_angle_fmt_layout->addStretch();
  output_layout->addWidget(out_angle_fmt);

  QWidget * output_type_row = new QWidget();
  QHBoxLayout * output_type_layout = new QHBoxLayout(output_type_row);
  output_type_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * output_type_label = new QLabel("Output type");
  QComboBox * output_type_combo = new QComboBox();
  output_type_combo->addItems({
    "Rotation matrix", "Quaternion", "Axis-angle", "Axis with angle magnitude", "Euler angles"
  });
  output_type_layout->addWidget(output_type_label);
  output_type_layout->addWidget(output_type_combo);
  output_type_layout->addStretch();
  output_layout->addWidget(output_type_row);

  QLabel * rotm_out_label = new QLabel("Rotation matrix");
  QTextEdit * rotm_out = new QTextEdit();
  rotm_out->setReadOnly(true);
  rotm_out->setMaximumHeight(90);
  rotm_out->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  output_layout->addWidget(rotm_out_label);
  output_layout->addWidget(rotm_out);

  QLabel * quat_out_label = new QLabel("Quaternion [x, y, z, w]");
  QTextEdit * quat_out = new QTextEdit();
  quat_out->setReadOnly(true);
  quat_out->setMaximumHeight(50);
  quat_out->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  output_layout->addWidget(quat_out_label);
  output_layout->addWidget(quat_out);

  QLabel * axis_angle_out_label = new QLabel("Axis-Angle {[x, y, z], angle}");
  QTextEdit * axis_angle_out = new QTextEdit();
  axis_angle_out->setReadOnly(true);
  axis_angle_out->setMaximumHeight(50);
  axis_angle_out->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  output_layout->addWidget(axis_angle_out_label);
  output_layout->addWidget(axis_angle_out);

  QLabel * axis_mag_out_label = new QLabel("Axis with angle magnitude [x, y, z]");
  QTextEdit * axis_mag_out = new QTextEdit();
  axis_mag_out->setReadOnly(true);
  axis_mag_out->setMaximumHeight(50);
  axis_mag_out->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  output_layout->addWidget(axis_mag_out_label);
  output_layout->addWidget(axis_mag_out);

  QWidget * euler_out_header = new QWidget();
  QHBoxLayout * euler_out_header_layout = new QHBoxLayout(euler_out_header);
  euler_out_header_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * euler_out_title = new QLabel("Euler angles");
  QComboBox * euler_order_out = new QComboBox();
  euler_order_out->addItems({"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"});
  euler_out_header_layout->addWidget(euler_out_title);
  euler_out_header_layout->addWidget(euler_order_out);
  euler_out_header_layout->addStretch();
  output_layout->addWidget(euler_out_header);
  QTextEdit * euler_out = new QTextEdit();
  euler_out->setReadOnly(true);
  euler_out->setMaximumHeight(50);
  euler_out->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  output_layout->addWidget(euler_out);
  output_layout->addStretch();

  layout->addWidget(output_group, 1);

  auto clamp_val = [](double v) {
    if (v > 1.0) return 1.0;
    if (v < -1.0) return -1.0;
    return v;
  };

  auto quat_from_euler = [&](double x, double y, double z, const QString & order, double & qx, double & qy, double & qz, double & qw) {
    const double c1 = std::cos(x / 2.0);
    const double c2 = std::cos(y / 2.0);
    const double c3 = std::cos(z / 2.0);
    const double s1 = std::sin(x / 2.0);
    const double s2 = std::sin(y / 2.0);
    const double s3 = std::sin(z / 2.0);
    if (order == "XYZ") {
      qx = s1 * c2 * c3 + c1 * s2 * s3;
      qy = c1 * s2 * c3 - s1 * c2 * s3;
      qz = c1 * c2 * s3 + s1 * s2 * c3;
      qw = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == "YXZ") {
      qx = s1 * c2 * c3 + c1 * s2 * s3;
      qy = c1 * s2 * c3 - s1 * c2 * s3;
      qz = c1 * c2 * s3 - s1 * s2 * c3;
      qw = c1 * c2 * c3 + s1 * s2 * s3;
    } else if (order == "ZXY") {
      qx = s1 * c2 * c3 - c1 * s2 * s3;
      qy = c1 * s2 * c3 + s1 * c2 * s3;
      qz = c1 * c2 * s3 + s1 * s2 * c3;
      qw = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == "ZYX") {
      qx = s1 * c2 * c3 - c1 * s2 * s3;
      qy = c1 * s2 * c3 + s1 * c2 * s3;
      qz = c1 * c2 * s3 - s1 * s2 * c3;
      qw = c1 * c2 * c3 + s1 * s2 * s3;
    } else if (order == "YZX") {
      qx = s1 * c2 * c3 + c1 * s2 * s3;
      qy = c1 * s2 * c3 + s1 * c2 * s3;
      qz = c1 * c2 * s3 - s1 * s2 * c3;
      qw = c1 * c2 * c3 - s1 * s2 * s3;
    } else {
      qx = s1 * c2 * c3 - c1 * s2 * s3;
      qy = c1 * s2 * c3 - s1 * c2 * s3;
      qz = c1 * c2 * s3 + s1 * s2 * c3;
      qw = c1 * c2 * c3 + s1 * s2 * s3;
    }
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n > 1e-12) { qx /= n; qy /= n; qz /= n; qw /= n; }
  };

  auto set_input_group_visibility = [=]() {
    const int idx = input_type_combo->currentIndex();
    rotm_in_group->setVisible(idx == 0);
    quat_in_group->setVisible(idx == 1);
    axis_angle_group->setVisible(idx == 2);
    axis_mag_group->setVisible(idx == 3);
    euler_in_group->setVisible(idx == 4);
    pqr_group->setVisible(idx == 5);
    from_euler_btn->setVisible(idx == 4);
  };
  auto set_output_group_visibility = [=]() {
    const int idx = output_type_combo->currentIndex();
    rotm_out_label->setVisible(idx == 0);
    rotm_out->setVisible(idx == 0);
    quat_out_label->setVisible(idx == 1);
    quat_out->setVisible(idx == 1);
    axis_angle_out_label->setVisible(idx == 2);
    axis_angle_out->setVisible(idx == 2);
    axis_mag_out_label->setVisible(idx == 3);
    axis_mag_out->setVisible(idx == 3);
    euler_out_header->setVisible(idx == 4);
    euler_out->setVisible(idx == 4);
  };

  auto update_quaternion_from_selected_input = [=]() {
    const int idx = input_type_combo->currentIndex();
    double qx = qx_in->text().toDouble();
    double qy = qy_in->text().toDouble();
    double qz = qz_in->text().toDouble();
    double qw = qw_in->text().toDouble();
    if (idx == 0) {
      const double m11 = rotm_in[0]->text().toDouble();
      const double m12 = rotm_in[1]->text().toDouble();
      const double m13 = rotm_in[2]->text().toDouble();
      const double m21 = rotm_in[3]->text().toDouble();
      const double m22 = rotm_in[4]->text().toDouble();
      const double m23 = rotm_in[5]->text().toDouble();
      const double m31 = rotm_in[6]->text().toDouble();
      const double m32 = rotm_in[7]->text().toDouble();
      const double m33 = rotm_in[8]->text().toDouble();
      const double trace = m11 + m22 + m33;
      if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (m32 - m23) / s;
        qy = (m13 - m31) / s;
        qz = (m21 - m12) / s;
      } else if ((m11 > m22) && (m11 > m33)) {
        const double s = std::sqrt(1.0 + m11 - m22 - m33) * 2.0;
        qw = (m32 - m23) / s;
        qx = 0.25 * s;
        qy = (m12 + m21) / s;
        qz = (m13 + m31) / s;
      } else if (m22 > m33) {
        const double s = std::sqrt(1.0 + m22 - m11 - m33) * 2.0;
        qw = (m13 - m31) / s;
        qx = (m12 + m21) / s;
        qy = 0.25 * s;
        qz = (m23 + m32) / s;
      } else {
        const double s = std::sqrt(1.0 + m33 - m11 - m22) * 2.0;
        qw = (m21 - m12) / s;
        qx = (m13 + m31) / s;
        qy = (m23 + m32) / s;
        qz = 0.25 * s;
      }
    } else if (idx == 4) {
      double ex = ex_in->text().toDouble();
      double ey = ey_in->text().toDouble();
      double ez = ez_in->text().toDouble();
      if (in_deg->isChecked()) { ex *= M_PI / 180.0; ey *= M_PI / 180.0; ez *= M_PI / 180.0; }
      quat_from_euler(ex, ey, ez, euler_order_in->currentText(), qx, qy, qz, qw);
    } else if (idx == 1) {
      // already quaternion
    } else if (idx == 2) {
      double ax = ax_in->text().toDouble(), ay = ay_in->text().toDouble(), az = az_in->text().toDouble(), angle = ang_in->text().toDouble();
      if (in_deg->isChecked()) angle *= M_PI / 180.0;
      const double n = std::sqrt(ax * ax + ay * ay + az * az);
      if (n > 1e-12) { ax /= n; ay /= n; az /= n; }
      const double half = angle * 0.5, s = std::sin(half);
      qx = ax * s; qy = ay * s; qz = az * s; qw = std::cos(half);
    } else if (idx == 3) {
      double vx = amx_in->text().toDouble();
      double vy = amy_in->text().toDouble();
      double vz = amz_in->text().toDouble();
      double angle = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (in_deg->isChecked()) { angle *= M_PI / 180.0; }
      const double n = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (n > 1e-12) { vx /= n; vy /= n; vz /= n; }
      const double half = angle * 0.5;
      const double s = std::sin(half);
      qx = vx * s; qy = vy * s; qz = vz * s; qw = std::cos(half);
    } else if (idx == 5) {
      const double px = pqr_in[0]->text().toDouble();
      const double py = pqr_in[1]->text().toDouble();
      const double pz = pqr_in[2]->text().toDouble();
      const double qxv = pqr_in[3]->text().toDouble();
      const double qyv = pqr_in[4]->text().toDouble();
      const double qzv = pqr_in[5]->text().toDouble();
      const double rx = pqr_in[6]->text().toDouble();
      const double ry = pqr_in[7]->text().toDouble();
      const double rz = pqr_in[8]->text().toDouble();
      double xx = qxv - px, xy = qyv - py, xz = qzv - pz;
      double zx = xx * (rz - pz) - xz * (ry - py);
      double zy = xz * (rx - px) - xx * (rz - pz);
      double zz = xx * (ry - py) - xy * (rx - px);
      double yx = zy * xz - zz * xy;
      double yy = zz * xx - zx * xz;
      double yz = zx * xy - zy * xx;
      auto norm3 = [](double & a, double & b, double & c) {
        const double n = std::sqrt(a * a + b * b + c * c);
        if (n > 1e-12) { a /= n; b /= n; c /= n; }
      };
      norm3(xx, xy, xz);
      norm3(yx, yy, yz);
      norm3(zx, zy, zz);
      const double m11 = xx, m12 = yx, m13 = zx;
      const double m21 = xy, m22 = yy, m23 = zy;
      const double m31 = xz, m32 = yz, m33 = zz;
      const double trace = m11 + m22 + m33;
      if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (m32 - m23) / s;
        qy = (m13 - m31) / s;
        qz = (m21 - m12) / s;
      } else {
        qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0;
      }
    }
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n > 1e-12) { qx /= n; qy /= n; qz /= n; qw /= n; }
    qx_in->setText(QString::number(qx, 'f', 7));
    qy_in->setText(QString::number(qy, 'f', 7));
    qz_in->setText(QString::number(qz, 'f', 7));
    qw_in->setText(QString::number(qw, 'f', 7));
  };

  auto refresh_output = [=]() {
    update_quaternion_from_selected_input();
    double x = qx_in->text().toDouble(), y = qy_in->text().toDouble(), z = qz_in->text().toDouble(), w = qw_in->text().toDouble();
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm < 1e-10) { return; }
    x /= norm; y /= norm; z /= norm; w /= norm;
    qx_in->setText(QString::number(x, 'f', 7));
    qy_in->setText(QString::number(y, 'f', 7));
    qz_in->setText(QString::number(z, 'f', 7));
    qw_in->setText(QString::number(w, 'f', 7));

    const double m11 = 1.0 - 2.0 * (y * y + z * z);
    const double m12 = 2.0 * (x * y - z * w);
    const double m13 = 2.0 * (x * z + y * w);
    const double m21 = 2.0 * (x * y + z * w);
    const double m22 = 1.0 - 2.0 * (x * x + z * z);
    const double m23 = 2.0 * (y * z - x * w);
    const double m31 = 2.0 * (x * z - y * w);
    const double m32 = 2.0 * (y * z + x * w);
    const double m33 = 1.0 - 2.0 * (x * x + y * y);
    rotm_out->setPlainText(
      QString("[  %1,  %2,  %3;\n   %4,  %5,  %6;\n   %7,  %8,  %9 ]")
        .arg(m11, 0, 'f', 7).arg(m12, 0, 'f', 7).arg(m13, 0, 'f', 7)
        .arg(m21, 0, 'f', 7).arg(m22, 0, 'f', 7).arg(m23, 0, 'f', 7)
        .arg(m31, 0, 'f', 7).arg(m32, 0, 'f', 7).arg(m33, 0, 'f', 7));
    quat_out->setPlainText(QString("[ %1, %2, %3, %4 ]")
                           .arg(x, 0, 'f', 7).arg(y, 0, 'f', 7).arg(z, 0, 'f', 7).arg(w, 0, 'f', 7));

    const double angle = 2.0 * std::acos(clamp_val(w));
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (s > 1e-8) { ax = x / s; ay = y / s; az = z / s; }
    const double angle_out = out_deg->isChecked() ? (angle * 180.0 / M_PI) : angle;
    axis_angle_out->setPlainText(QString("{ [ %1, %2, %3 ], %4 }")
                                 .arg(ax, 0, 'f', 7).arg(ay, 0, 'f', 7).arg(az, 0, 'f', 7).arg(angle_out, 0, 'f', 7));
    axis_mag_out->setPlainText(QString("[ %1, %2, %3 ]")
                               .arg(ax * angle_out, 0, 'f', 7).arg(ay * angle_out, 0, 'f', 7).arg(az * angle_out, 0, 'f', 7));

    double ex = 0.0, ey = 0.0, ez = 0.0;
    const QString order = euler_order_out->currentText();
    if (order == "XYZ") {
      ey = std::asin(clamp_val(m13));
      ex = std::atan2(-m23, m33);
      ez = std::atan2(-m12, m11);
    } else if (order == "YXZ") {
      ex = std::asin(-clamp_val(m23));
      ey = std::atan2(m13, m33);
      ez = std::atan2(m21, m22);
    } else if (order == "ZXY") {
      ex = std::asin(clamp_val(m32));
      ey = std::atan2(-m31, m33);
      ez = std::atan2(-m12, m22);
    } else if (order == "ZYX") {
      ey = std::asin(-clamp_val(m31));
      ex = std::atan2(m32, m33);
      ez = std::atan2(m21, m11);
    } else if (order == "YZX") {
      ez = std::asin(clamp_val(m21));
      ex = std::atan2(-m23, m22);
      ey = std::atan2(-m31, m11);
    } else {
      ez = std::asin(-clamp_val(m12));
      ex = std::atan2(m32, m22);
      ey = std::atan2(m13, m11);
    }
    if (out_deg->isChecked()) { ex *= 180.0 / M_PI; ey *= 180.0 / M_PI; ez *= 180.0 / M_PI; }
    euler_out->setPlainText(QString("[ x: %1, y: %2, z: %3 ]").arg(ex, 0, 'f', 7).arg(ey, 0, 'f', 7).arg(ez, 0, 'f', 7));
  };

  QObject::connect(refresh_btn, &QPushButton::clicked, [=]() { refresh_output(); });
  QObject::connect(out_rad, &QRadioButton::clicked, [=]() { refresh_output(); });
  QObject::connect(out_deg, &QRadioButton::clicked, [=]() { refresh_output(); });
  QObject::connect(euler_order_out, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) { refresh_output(); });
  QObject::connect(input_type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    set_input_group_visibility();
    refresh_output();
  });
  QObject::connect(output_type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    set_output_group_visibility();
  });
  QObject::connect(from_euler_btn, &QPushButton::clicked, [=]() {
    input_type_combo->setCurrentIndex(4);
    refresh_output();
  });

  set_input_group_visibility();
  set_output_group_visibility();
  refresh_output();
}

}  // namespace ros_robot_assist_tools::ui
