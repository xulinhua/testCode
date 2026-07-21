// Qt 抓取测试主窗口：参考 calib_sim_isaac 布局，不含标定控件。
#include <csignal>

#include "nova_grasp_moveit/grasp_qt_ui.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <QApplication>
#include <QBrush>
#include <QCheckBox>
#include <QColor>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QMetaObject>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace nova_grasp_moveit
{

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

bool is_ng_log_line(const QString & line)
{
  const QString u = line.toUpper();
  return u.contains("[ERR]") || u.contains("[ERROR]") || u.contains(" FAILED") ||
    u.contains("FAIL:") || u.contains(" NG") || u.startsWith("NG") ||
    line.contains(QString::fromUtf8("失败")) || line.contains(QString::fromUtf8("不可用")) ||
    line.contains(QString::fromUtf8("异常"));
}

/// 机械臂状态页：只显示逆解 / 关节发送相关调试行
bool is_arm_status_debug_log(const std::string & line)
{
  return line.find("[pose_log]") != std::string::npos ||
    line.find("[joints]") != std::string::npos ||
    line.find("[gripper]") != std::string::npos ||
    line.find("[pose]") != std::string::npos ||
    line.find("[POSE_DEBUG]") != std::string::npos ||
    line.find("IK ") != std::string::npos ||
    line.find("compute_ik") != std::string::npos;
}

void append_log_line_colored(QTextEdit * edit, const QString & line)
{
  if (edit == nullptr) {
    return;
  }
  QTextCursor cursor = edit->textCursor();
  cursor.movePosition(QTextCursor::End);
  QTextCharFormat fmt;
  if (is_ng_log_line(line)) {
    fmt.setForeground(QColor(QStringLiteral("#c0392b")));
    fmt.setFontWeight(QFont::DemiBold);
  } else if (line.contains(QStringLiteral("[PATH]")) || line.contains(QStringLiteral("[STEP]"))) {
    // 规划路径 / 单步前后对比：蓝色
    fmt.setForeground(QColor(QStringLiteral("#1565c0")));
    fmt.setFontWeight(QFont::DemiBold);
  } else {
    fmt.setForeground(QColor(QStringLiteral("#252b38")));
    fmt.setFontWeight(QFont::Normal);
  }
  cursor.insertText(line + QLatin1Char('\n'), fmt);
  edit->setTextCursor(cursor);
}

void refill_log_view(QTextEdit * edit, const std::vector<std::string> & lines)
{
  if (edit == nullptr) {
    return;
  }
  edit->clear();
  for (const auto & line : lines) {
    append_log_line_colored(edit, QString::fromStdString(line));
  }
  edit->moveCursor(QTextCursor::End);
}

void set_grasp_pose_table_row(
  QTableWidget * table,
  int row,
  int source_index,
  double score,
  const geometry_msgs::msg::Pose & pose,
  const QColor & background = QColor(),
  const QColor & rpy_background = QColor())
{
  tf2::Quaternion q(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const QStringList values = {
    QString::number(source_index),
    std::isfinite(score) ? QString::number(score, 'f', 5) : QStringLiteral("--"),
    QString::number(pose.position.x, 'f', 4),
    QString::number(pose.position.y, 'f', 4),
    QString::number(pose.position.z, 'f', 4),
    QString::number(roll * kRadToDeg, 'f', 2),
    QString::number(pitch * kRadToDeg, 'f', 2),
    QString::number(yaw * kRadToDeg, 'f', 2)};
  // 列：# Score X Y Z Roll Pitch Yaw —— RPY 为 5..7
  constexpr int kRpyColBegin = 5;
  constexpr int kRpyColEnd = 8;
  for (int column = 0; column < values.size(); ++column) {
    auto * item = new QTableWidgetItem(values[column]);
    item->setTextAlignment(Qt::AlignCenter);
    const bool is_rpy = column >= kRpyColBegin && column < kRpyColEnd;
    if (is_rpy && rpy_background.isValid()) {
      item->setBackground(QBrush(rpy_background));
    } else if (background.isValid()) {
      item->setBackground(QBrush(background));
    }
    table->setItem(row, column, item);
  }
}

QDoubleSpinBox * make_deg_spin(QWidget * parent, double value_deg = 0.0)
{
  auto * spin = new QDoubleSpinBox(parent);
  spin->setDecimals(2);
  spin->setRange(-360.0, 360.0);
  spin->setSingleStep(1.0);
  spin->setValue(value_deg);
  spin->setFixedWidth(72);
  return spin;
}

QDoubleSpinBox * make_pose_spin(QWidget * parent, double lo, double hi, double v)
{
  auto * spin = new QDoubleSpinBox(parent);
  spin->setDecimals(4);
  spin->setRange(lo, hi);
  spin->setSingleStep(0.01);
  spin->setValue(v);
  spin->setFixedWidth(88);
  return spin;
}

QDoubleSpinBox * make_metric_spin(
  QWidget * parent, double lo, double hi, double step, int decimals, double v, int width = 88)
{
  auto * spin = new QDoubleSpinBox(parent);
  spin->setDecimals(decimals);
  spin->setRange(lo, hi);
  spin->setSingleStep(step);
  spin->setValue(v);
  spin->setFixedWidth(width);
  return spin;
}

/// 一个机械臂状态卡片的全部控件和“用户正在编辑”状态。
/// dirty 标志防止 100 ms UI 刷新把用户尚未发送的目标值覆盖掉。
struct ArmControlPanel
{
  int arm_id{0};
  // J1..J6：臂关节 (deg)
  std::vector<QLabel *> current_labels;
  std::vector<QDoubleSpinBox *> target_spins;
  // J7/J8：夹爪 prismatic (m)
  std::vector<QLabel *> grip_current_labels;
  std::vector<QDoubleSpinBox *> grip_target_spins;
  QLabel * ee_tf_label{nullptr};
  QLabel * ee_quat_label{nullptr};
  QDoubleSpinBox * px{nullptr};
  QDoubleSpinBox * py{nullptr};
  QDoubleSpinBox * pz{nullptr};
  QDoubleSpinBox * roll{nullptr};
  QDoubleSpinBox * pitch{nullptr};
  QDoubleSpinBox * yaw{nullptr};
  QDoubleSpinBox * gripper_opening{nullptr};
  QCheckBox * realtime_cb{nullptr};
  QTimer * joint_rt_timer{nullptr};
  QTimer * pose_rt_timer{nullptr};
  QTimer * grip_rt_timer{nullptr};
  bool joints_dirty{false};
  bool pose_dirty{false};
  /// 仅当用户改过 roll/pitch/yaw 时为 true；只拧 xyz 时保持 false，下发用 TF 四元数
  bool orientation_dirty{false};
  bool gripper_dirty{false};
  bool initialized{false};
};

void mark_pose_dirty(ArmControlPanel & panel)
{
  panel.pose_dirty = true;
}

bool realtime_enabled(const ArmControlPanel & panel)
{
  return panel.realtime_cb != nullptr && panel.realtime_cb->isChecked();
}

/// 组装 8 轴命令：J1-6 为弧度，J7/J8 为米。
std::vector<double> collect_arm_command(const ArmControlPanel & panel)
{
  std::vector<double> positions(8, 0.0);
  for (std::size_t i = 0; i < panel.target_spins.size() && i < 6; ++i) {
    positions[i] = panel.target_spins[i]->value() * kDegToRad;
  }
  for (std::size_t i = 0; i < panel.grip_target_spins.size() && i < 2; ++i) {
    positions[6 + i] = panel.grip_target_spins[i]->value();
  }
  return positions;
}

void sync_opening_from_grip_spins(ArmControlPanel & panel)
{
  if (panel.gripper_opening == nullptr || panel.grip_target_spins.size() < 2) {
    return;
  }
  const double opening = gripper_joint_pair_to_opening(
    panel.grip_target_spins[0]->value(), panel.grip_target_spins[1]->value());
  QSignalBlocker blocker(panel.gripper_opening);
  panel.gripper_opening->setValue(opening);
}

void sync_grip_spins_from_opening(ArmControlPanel & panel, double opening_m)
{
  if (panel.grip_target_spins.size() < 2) {
    return;
  }
  double j7 = 0.0;
  double j8 = 0.0;
  gripper_opening_to_joint_pair(opening_m, j7, j8);
  QSignalBlocker b0(panel.grip_target_spins[0]);
  QSignalBlocker b1(panel.grip_target_spins[1]);
  panel.grip_target_spins[0]->setValue(j7);
  panel.grip_target_spins[1]->setValue(j8);
}

void publish_panel_joints(ArmControlPanel & panel, const std::shared_ptr<GraspRosNode> & ros_node)
{
  ros_node->publish_arm_joints(panel.arm_id, collect_arm_command(panel));
  panel.joints_dirty = true;
  panel.pose_dirty = false;
}

void publish_panel_gripper(ArmControlPanel & panel, const std::shared_ptr<GraspRosNode> & ros_node)
{
  // 只发 J7/J8，避免把面板上的臂关节目标一并下发
  if (panel.grip_target_spins.size() >= 2) {
    ros_node->publish_gripper_joint_pair_detailed(
      panel.arm_id,
      panel.grip_target_spins[0]->value(),
      panel.grip_target_spins[1]->value(),
      nullptr);
  } else {
    ros_node->publish_gripper_opening_detailed(
      panel.arm_id, panel.gripper_opening->value(), nullptr);
  }
  panel.gripper_dirty = true;
  sync_opening_from_grip_spins(panel);
}

void publish_panel_pose(
  ArmControlPanel & panel,
  const std::shared_ptr<GraspRosNode> & ros_node,
  const std::function<EePoseSnapshot()> & get_ee)
{
  const double x = panel.px->value();
  const double y = panel.py->value();
  const double z = panel.pz->value();
  const double roll = panel.roll->value();
  const double pitch = panel.pitch->value();
  const double yaw = panel.yaw->value();

  // 未改姿态：保持 TF 当前四元数，只平移（避免 RPY≈±180° 重建导致低头）
  const auto ee = get_ee();
  if (!panel.orientation_dirty && ee.ok) {
    ros_node->send_arm_pose_goal_quat(
      panel.arm_id, x, y, z, ee.qx, ee.qy, ee.qz, ee.qw);
  } else {
    ros_node->send_arm_pose_goal(panel.arm_id, x, y, z, roll, pitch, yaw);
  }
  // 保持 dirty，避免实时拖动时被 TF 回写覆盖
  panel.pose_dirty = true;
}

void show_grasp_error(QWidget * parent, const GraspComputeResult & result);
void show_grasp_error(QWidget * parent, const GraspExecuteResult & result);

/// 构建 J1/J2 通用控制卡片：关节、腕部 Pose、夹爪和各自实时发送定时器。
QGroupBox * build_arm_panel(
  QWidget * parent, ArmControlPanel & panel, int arm_id, const QString & ee_name,
  const std::shared_ptr<GraspRosNode> & ros_node,
  const std::function<EePoseSnapshot()> & get_ee)
{
  panel.arm_id = arm_id;
  auto * group = new QGroupBox(parent);
  group->setTitle(QString());
  auto * layout = new QVBoxLayout(group);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  auto * header = new QHBoxLayout();
  header->setSpacing(10);
  auto * arm_title = new QLabel(QString("<b>J%1</b>").arg(arm_id + 1), group);
  panel.realtime_cb = new QCheckBox(QString::fromUtf8("实时模式"), group);
  panel.realtime_cb->setToolTip(
    QString::fromUtf8("勾选后，修改关节目标、末端位姿或夹爪位置会立即下发，无需点应用"));
  header->addWidget(arm_title, 0, Qt::AlignVCenter);
  header->addWidget(panel.realtime_cb, 0, Qt::AlignVCenter);
  header->addStretch(1);
  layout->addLayout(header);

  panel.joint_rt_timer = new QTimer(group);
  panel.joint_rt_timer->setSingleShot(true);
  panel.joint_rt_timer->setInterval(80);
  panel.pose_rt_timer = new QTimer(group);
  panel.pose_rt_timer->setSingleShot(true);
  panel.pose_rt_timer->setInterval(120);
  panel.grip_rt_timer = new QTimer(group);
  panel.grip_rt_timer->setSingleShot(true);
  panel.grip_rt_timer->setInterval(80);
  QObject::connect(panel.joint_rt_timer, &QTimer::timeout, [ &panel, ros_node ]() {
      publish_panel_joints(panel, ros_node);
    });
  QObject::connect(panel.pose_rt_timer, &QTimer::timeout, [ &panel, ros_node, get_ee ]() {
      publish_panel_pose(panel, ros_node, get_ee);
    });
  QObject::connect(panel.grip_rt_timer, &QTimer::timeout, [ &panel, ros_node ]() {
      publish_panel_gripper(panel, ros_node);
    });
  QObject::connect(panel.realtime_cb, &QCheckBox::toggled, [ &panel ](bool on) {
      if (on) {
        return;
      }
      if (panel.joint_rt_timer != nullptr) {
        panel.joint_rt_timer->stop();
      }
      if (panel.pose_rt_timer != nullptr) {
        panel.pose_rt_timer->stop();
      }
      if (panel.grip_rt_timer != nullptr) {
        panel.grip_rt_timer->stop();
      }
    });

  auto * upper_row = new QHBoxLayout();
  upper_row->setSpacing(10);
  upper_row->setAlignment(Qt::AlignTop);

  auto * joint_group = new QGroupBox(QString::fromUtf8("关节 (deg)"), group);
  joint_group->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
  auto * joint_grid = new QGridLayout(joint_group);
  joint_grid->setHorizontalSpacing(6);
  joint_grid->setVerticalSpacing(4);
  joint_grid->addWidget(new QLabel(QString::fromUtf8("关节"), joint_group), 0, 0);
  joint_grid->addWidget(new QLabel(QString::fromUtf8("当前"), joint_group), 0, 1);
  joint_grid->addWidget(new QLabel(QString::fromUtf8("目标"), joint_group), 0, 2);
  for (int i = 0; i < 6; ++i) {
    const int row = i + 1;
    auto * name = new QLabel(QString("J%1").arg(i + 1), joint_group);
    name->setFixedWidth(28);
    auto * cur = new QLabel("—", joint_group);
    cur->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    cur->setFixedWidth(52);
    auto * tgt = make_deg_spin(joint_group);
    panel.current_labels.push_back(cur);
    panel.target_spins.push_back(tgt);
    joint_grid->addWidget(name, row, 0);
    joint_grid->addWidget(cur, row, 1);
    joint_grid->addWidget(tgt, row, 2);
    QObject::connect(tgt, qOverload<double>(&QDoubleSpinBox::valueChanged), [ &panel ](double) {
        panel.joints_dirty = true;
        if (realtime_enabled(panel) && panel.joint_rt_timer != nullptr) {
          panel.joint_rt_timer->start();
        }
      });
  }
  auto * btn_copy_j = new QPushButton(QString::fromUtf8("复制当前"), joint_group);
  btn_copy_j->setToolTip(QString::fromUtf8("把左侧当前关节角填入目标控件（不下发）"));
  auto * btn_apply_j = new QPushButton(QString::fromUtf8("应用关节"), joint_group);
  auto * btn_reset_j = new QPushButton(QString::fromUtf8("复位关节"), joint_group);
  auto * joint_btns = new QHBoxLayout();
  joint_btns->setSpacing(6);
  joint_btns->addWidget(btn_copy_j);
  joint_btns->addWidget(btn_apply_j);
  joint_btns->addWidget(btn_reset_j);
  joint_grid->addLayout(joint_btns, 8, 0, 1, 3);
  upper_row->addWidget(joint_group, 0, Qt::AlignTop);

  auto * ee_group = new QGroupBox(QString::fromUtf8("末端 ") + ee_name, group);
  ee_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  panel.ee_tf_label = new QLabel(QString::fromUtf8("TF: —"), ee_group);
  panel.ee_tf_label->setWordWrap(true);
  panel.ee_quat_label = new QLabel(QString::fromUtf8("quat xyzw: —"), ee_group);
  panel.ee_quat_label->setWordWrap(true);
  panel.ee_quat_label->setToolTip(
    QString::fromUtf8("TF 原始四元数 (xyzw)；与下方 roll/pitch/yaw 对应，便于核对姿态"));
  panel.px = make_pose_spin(ee_group, -5.0, 5.0, 0.0);
  panel.py = make_pose_spin(ee_group, -5.0, 5.0, 0.0);
  panel.pz = make_pose_spin(ee_group, -5.0, 5.0, 0.0);
  // RPY 用度：步进 1°（勿与 xyz 的 0.01m 共用，否则拧不动姿态）
  panel.roll = make_metric_spin(ee_group, -180.0, 180.0, 1.0, 2, 0.0, 88);
  panel.pitch = make_metric_spin(ee_group, -180.0, 180.0, 1.0, 2, 0.0, 88);
  panel.yaw = make_metric_spin(ee_group, -180.0, 180.0, 1.0, 2, 0.0, 88);
  auto * ee_grid = new QGridLayout();
  ee_grid->setHorizontalSpacing(8);
  ee_grid->setVerticalSpacing(4);
  ee_grid->addWidget(new QLabel(QString::fromUtf8("字段"), ee_group), 0, 0);
  ee_grid->addWidget(new QLabel(QString::fromUtf8("目标"), ee_group), 0, 1);
  const struct EeRow { const char * label; QDoubleSpinBox * spin; } ee_rows[] = {
    {"x (m)", panel.px}, {"y (m)", panel.py}, {"z (m)", panel.pz},
    {"roll (deg)", panel.roll}, {"pitch (deg)", panel.pitch}, {"yaw (deg)", panel.yaw},
  };
  for (int i = 0; i < 6; ++i) {
    ee_grid->addWidget(new QLabel(ee_rows[i].label, ee_group), i + 1, 0);
    ee_grid->addWidget(ee_rows[i].spin, i + 1, 1);
  }
  auto * btn_copy_p = new QPushButton(QString::fromUtf8("复制当前"), ee_group);
  btn_copy_p->setToolTip(QString::fromUtf8("把上方 TF 当前位姿填入目标控件（不下发）"));
  auto * btn_apply_p = new QPushButton(QString::fromUtf8("应用位姿"), ee_group);
  btn_apply_p->setToolTip(
    QString::fromUtf8("经 MoveIt IK 后发 /joint_command 到 Isaac；实时模式下改目标会自动下发"));
  auto * btn_reset_p = new QPushButton(QString::fromUtf8("复位位姿"), ee_group);
  btn_reset_p->setToolTip(QString::fromUtf8("同「复制当前」：用 TF 当前值覆盖目标控件"));
  auto * pose_btns = new QHBoxLayout();
  pose_btns->setSpacing(6);
  pose_btns->addWidget(btn_copy_p);
  pose_btns->addWidget(btn_apply_p);
  pose_btns->addWidget(btn_reset_p);
  auto * ee_layout = new QVBoxLayout(ee_group);
  ee_layout->setSpacing(4);
  ee_layout->addWidget(panel.ee_tf_label);
  ee_layout->addWidget(panel.ee_quat_label);
  ee_layout->addLayout(ee_grid);
  ee_layout->addLayout(pose_btns);
  upper_row->addWidget(ee_group, 1, Qt::AlignTop);

  layout->addLayout(upper_row, 0);

  auto * grip_group = new QGroupBox(QString::fromUtf8("夹爪 (m)"), group);
  grip_group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  auto * grip_joint_grid = new QGridLayout();
  grip_joint_grid->setHorizontalSpacing(6);
  grip_joint_grid->setVerticalSpacing(4);
  grip_joint_grid->addWidget(new QLabel(QString::fromUtf8("关节"), grip_group), 0, 0);
  grip_joint_grid->addWidget(new QLabel(QString::fromUtf8("当前"), grip_group), 0, 1);
  grip_joint_grid->addWidget(new QLabel(QString::fromUtf8("目标"), grip_group), 0, 2);
  for (int i = 0; i < 2; ++i) {
    auto * name = new QLabel(QString("J%1").arg(i + 7), grip_group);
    name->setFixedWidth(28);
    auto * cur = new QLabel("—", grip_group);
    cur->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    cur->setFixedWidth(64);
    auto * tgt = make_metric_spin(grip_group, -0.05, 0.05, 0.001, 4, 0.0, 88);
    panel.grip_current_labels.push_back(cur);
    panel.grip_target_spins.push_back(tgt);
    grip_joint_grid->addWidget(name, i + 1, 0);
    grip_joint_grid->addWidget(cur, i + 1, 1);
    grip_joint_grid->addWidget(tgt, i + 1, 2);
    QObject::connect(tgt, qOverload<double>(&QDoubleSpinBox::valueChanged), [ &panel ](double) {
        panel.gripper_dirty = true;
        sync_opening_from_grip_spins(panel);
        if (realtime_enabled(panel) && panel.grip_rt_timer != nullptr) {
          panel.grip_rt_timer->start();
        }
      });
  }
  panel.gripper_opening = make_metric_spin(grip_group, 0.0, 0.08, 0.005, 3, 0.08, 88);
  panel.gripper_opening->setToolTip(
    QString::fromUtf8("张开距离：0=完全闭合，0.08=完全张开（对称映射到 J7/J8）"));
  auto * grip_form = new QFormLayout();
  grip_form->addRow(QString::fromUtf8("张开距离 (m)"), panel.gripper_opening);
  auto * btn_apply_g = new QPushButton(QString::fromUtf8("应用夹爪"), grip_group);
  auto * btn_open_g = new QPushButton(QString::fromUtf8("全开"), grip_group);
  auto * btn_close_g = new QPushButton(QString::fromUtf8("全闭"), grip_group);
  auto * grip_btns = new QHBoxLayout();
  grip_btns->addWidget(btn_apply_g);
  grip_btns->addWidget(btn_open_g);
  grip_btns->addWidget(btn_close_g);
  auto * grip_layout = new QVBoxLayout(grip_group);
  grip_layout->setSpacing(4);
  grip_layout->addLayout(grip_joint_grid);
  grip_layout->addLayout(grip_form);
  grip_layout->addLayout(grip_btns);
  layout->addWidget(grip_group, 0, Qt::AlignTop);

  group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  group->setMinimumWidth(420);

  for (QDoubleSpinBox * spin : {panel.px, panel.py, panel.pz}) {
    QObject::connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), [ &panel ](double) {
        mark_pose_dirty(panel);
        if (realtime_enabled(panel) && panel.pose_rt_timer != nullptr) {
          panel.pose_rt_timer->start();
        }
      });
  }
  for (QDoubleSpinBox * spin : {panel.roll, panel.pitch, panel.yaw}) {
    QObject::connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), [ &panel ](double) {
        mark_pose_dirty(panel);
        panel.orientation_dirty = true;
        if (realtime_enabled(panel) && panel.pose_rt_timer != nullptr) {
          panel.pose_rt_timer->start();
        }
      });
  }

  auto copy_current_joints_to_targets = [ &panel ]() {
      bool any = false;
      for (std::size_t i = 0; i < panel.current_labels.size() && i < panel.target_spins.size(); ++i) {
        bool ok = false;
        const double deg = panel.current_labels[i]->text().toDouble(&ok);
        if (!ok) {
          continue;
        }
        QSignalBlocker blocker(panel.target_spins[i]);
        panel.target_spins[i]->setValue(deg);
        any = true;
      }
      if (any) {
        panel.joints_dirty = false;  // 与位姿「复制当前」一致：允许后续用实测同步目标
        if (panel.joint_rt_timer != nullptr) {
          panel.joint_rt_timer->stop();
        }
      }
      return any;
    };
  QObject::connect(btn_copy_j, &QPushButton::clicked, [copy_current_joints_to_targets, parent]() {
      if (!copy_current_joints_to_targets()) {
        GraspComputeResult result;
        result.error_title = "复制当前失败";
        result.error_message = "当前关节角不可用，请确认 Isaac 已 Play 且 /joint_states 正常。";
        show_grasp_error(parent->window(), result);
      }
    });
  QObject::connect(btn_reset_j, &QPushButton::clicked, [ &panel, ros_node, parent ]() {
      // 复位打断抓取序列并清空 busy
      ros_node->abort_grasp_execution();
      // 仅复位本臂 J1–J6→0；不碰本臂夹爪，也不碰另一臂
      panel.joints_dirty = true;
      panel.pose_dirty = false;
      for (auto * spin : panel.target_spins) {
        QSignalBlocker blocker(spin);
        spin->setValue(0.0);
      }
      std::vector<double> arm_only(6, 0.0);
      std::string err;
      if (!ros_node->publish_arm_joints_detailed(panel.arm_id, arm_only, &err)) {
        GraspComputeResult result;
        result.error_title = "关节复位失败";
        result.error_message = err.empty() ?
          "无法发布关节命令，请确认 Isaac 已 Play 且 /joint_states 正常。" :
          err;
        show_grasp_error(parent->window(), result);
      }
    });
  QObject::connect(btn_apply_j, &QPushButton::clicked, [ &panel, ros_node, parent ]() {
      std::string err;
      if (!ros_node->publish_arm_joints_detailed(panel.arm_id, collect_arm_command(panel), &err)) {
        GraspComputeResult result;
        result.error_title = "应用关节失败";
        result.error_message = err.empty() ?
          "无法发布关节命令，请确认 Isaac 已 Play 且 /joint_states 正常。" :
          err;
        show_grasp_error(parent->window(), result);
        return;
      }
      panel.joints_dirty = true;
      panel.pose_dirty = false;
    });

  auto copy_current_pose_to_targets = [ &panel, get_ee ]() {
      const auto ee = get_ee();
      if (!ee.ok) {
        return false;
      }
      QSignalBlocker b0(panel.px);
      QSignalBlocker b1(panel.py);
      QSignalBlocker b2(panel.pz);
      QSignalBlocker b3(panel.roll);
      QSignalBlocker b4(panel.pitch);
      QSignalBlocker b5(panel.yaw);
      panel.px->setValue(ee.x);
      panel.py->setValue(ee.y);
      panel.pz->setValue(ee.z);
      panel.roll->setValue(ee.roll_deg);
      panel.pitch->setValue(ee.pitch_deg);
      panel.yaw->setValue(ee.yaw_deg);
      panel.pose_dirty = false;
      panel.orientation_dirty = false;
      if (panel.pose_rt_timer != nullptr) {
        panel.pose_rt_timer->stop();
      }
      return true;
    };
  QObject::connect(btn_copy_p, &QPushButton::clicked, [copy_current_pose_to_targets, parent]() {
      if (!copy_current_pose_to_targets()) {
        GraspComputeResult result;
        result.error_title = "复制当前失败";
        result.error_message = "TF 当前位姿不可用，请确认 Isaac 已 Play 且 /tf 正常。";
        show_grasp_error(parent->window(), result);
      }
    });
  QObject::connect(btn_reset_p, &QPushButton::clicked, [copy_current_pose_to_targets]() {
      copy_current_pose_to_targets();
    });
  QObject::connect(btn_apply_p, &QPushButton::clicked, [ &panel, ros_node, get_ee ]() {
      publish_panel_pose(panel, ros_node, get_ee);
      panel.pose_dirty = true;  // 保持目标不被 TF 回写覆盖，便于对照到位
    });

  auto apply_gripper_opening = [ &panel, ros_node, parent ](double opening_m) {
      sync_grip_spins_from_opening(panel, opening_m);
      QSignalBlocker blocker(panel.gripper_opening);
      panel.gripper_opening->setValue(opening_m);
      panel.gripper_dirty = true;
      std::string err;
      if (!ros_node->publish_gripper_opening_detailed(panel.arm_id, opening_m, &err)) {
        GraspComputeResult result;
        result.error_title = "应用夹爪失败";
        result.error_message = err.empty() ? "无法发布夹爪关节命令" : err;
        show_grasp_error(parent->window(), result);
      }
    };

  QObject::connect(panel.gripper_opening, qOverload<double>(&QDoubleSpinBox::valueChanged),
    [ &panel ](double opening_m) {
      sync_grip_spins_from_opening(panel, opening_m);
      panel.gripper_dirty = true;
      if (realtime_enabled(panel) && panel.grip_rt_timer != nullptr) {
        panel.grip_rt_timer->start();
      }
    });

  QObject::connect(btn_apply_g, &QPushButton::clicked, [ &panel, ros_node, parent, apply_gripper_opening ]() {
      if (panel.gripper_dirty && panel.grip_target_spins.size() >= 2) {
        std::string err;
        if (!ros_node->publish_gripper_joint_pair_detailed(
            panel.arm_id,
            panel.grip_target_spins[0]->value(),
            panel.grip_target_spins[1]->value(),
            &err))
        {
          GraspComputeResult result;
          result.error_title = "应用夹爪失败";
          result.error_message = err.empty() ? "无法发布夹爪关节命令" : err;
          show_grasp_error(parent->window(), result);
          return;
        }
        sync_opening_from_grip_spins(panel);
        return;
      }
      apply_gripper_opening(panel.gripper_opening->value());
    });
  QObject::connect(btn_open_g, &QPushButton::clicked, [apply_gripper_opening, ros_node]() {
      apply_gripper_opening(ros_node->planner_config().gripper_open_m);
    });
  QObject::connect(btn_close_g, &QPushButton::clicked, [apply_gripper_opening, ros_node]() {
      apply_gripper_opening(ros_node->planner_config().gripper_close_m);
    });

  return group;
}

void show_grasp_error(QWidget * parent, const GraspComputeResult & result)
{
  if (result.ok) {
    return;
  }
  QMessageBox::warning(
    parent,
    result.error_title.empty() ?
    QString::fromUtf8("操作失败") : QString::fromStdString(result.error_title),
    result.error_message.empty() ?
    QString::fromUtf8("未知错误") : QString::fromStdString(result.error_message));
}

void show_grasp_error(QWidget * parent, const GraspExecuteResult & result)
{
  if (result.ok) {
    return;
  }
  QMessageBox::warning(
    parent,
    result.error_title.empty() ?
    QString::fromUtf8("操作失败") : QString::fromStdString(result.error_title),
    result.error_message.empty() ?
    QString::fromUtf8("未知错误") : QString::fromStdString(result.error_message));
}

void sync_arm_panel_ui(ArmControlPanel & panel, const ArmJointSnapshot & joints, const EePoseSnapshot & ee)
{
  const int n = static_cast<int>(joints.positions_rad.size());
  for (int i = 0; i < 6 && i < n && i < static_cast<int>(panel.current_labels.size()); ++i) {
    const double deg = joints.positions_rad[static_cast<std::size_t>(i)] * kRadToDeg;
    panel.current_labels[static_cast<std::size_t>(i)]->setText(QString::number(deg, 'f', 2));
    if (!panel.joints_dirty) {
      QSignalBlocker blocker(panel.target_spins[static_cast<std::size_t>(i)]);
      panel.target_spins[static_cast<std::size_t>(i)]->setValue(deg);
    }
  }
  for (int i = 0; i < 2 && (6 + i) < n && i < static_cast<int>(panel.grip_current_labels.size()); ++i) {
    const double pos_m = joints.positions_rad[static_cast<std::size_t>(6 + i)];
    panel.grip_current_labels[static_cast<std::size_t>(i)]->setText(QString::number(pos_m, 'f', 4));
    if (!panel.gripper_dirty) {
      QSignalBlocker blocker(panel.grip_target_spins[static_cast<std::size_t>(i)]);
      panel.grip_target_spins[static_cast<std::size_t>(i)]->setValue(pos_m);
    }
  }
  if (!panel.gripper_dirty && n >= 8 && panel.gripper_opening != nullptr) {
    const double opening = gripper_joint_pair_to_opening(
      joints.positions_rad[6], joints.positions_rad[7]);
    QSignalBlocker blocker(panel.gripper_opening);
    panel.gripper_opening->setValue(opening);
  }
  if (!panel.initialized && !joints.positions_rad.empty()) {
    panel.initialized = true;
  }

  if (ee.ok) {
    panel.ee_tf_label->setText(
      QString::fromUtf8("TF: %1 → %2  当前 xyz=(%3,%4,%5)  rpy=(%6,%7,%8)°")
        .arg(QString::fromStdString(ee.ref_frame))
        .arg(QString::fromStdString(ee.child_frame))
        .arg(ee.x, 0, 'f', 3).arg(ee.y, 0, 'f', 3).arg(ee.z, 0, 'f', 3)
        .arg(ee.roll_deg, 0, 'f', 1).arg(ee.pitch_deg, 0, 'f', 1).arg(ee.yaw_deg, 0, 'f', 1));
    if (panel.ee_quat_label != nullptr) {
      panel.ee_quat_label->setText(
        QString::fromUtf8("quat xyzw=(%1, %2, %3, %4)")
          .arg(ee.qx, 0, 'f', 4)
          .arg(ee.qy, 0, 'f', 4)
          .arg(ee.qz, 0, 'f', 4)
          .arg(ee.qw, 0, 'f', 4));
    }
    if (!panel.pose_dirty) {
      QSignalBlocker b0(panel.px);
      QSignalBlocker b1(panel.py);
      QSignalBlocker b2(panel.pz);
      QSignalBlocker b3(panel.roll);
      QSignalBlocker b4(panel.pitch);
      QSignalBlocker b5(panel.yaw);
      panel.px->setValue(ee.x);
      panel.py->setValue(ee.y);
      panel.pz->setValue(ee.z);
      panel.roll->setValue(ee.roll_deg);
      panel.pitch->setValue(ee.pitch_deg);
      panel.yaw->setValue(ee.yaw_deg);
    }
  } else {
    panel.ee_tf_label->setText(
      QString::fromUtf8("TF 不可用 (%1 → %2)")
        .arg(QString::fromStdString(ee.ref_frame))
        .arg(QString::fromStdString(ee.child_frame)));
    if (panel.ee_quat_label != nullptr) {
      panel.ee_quat_label->setText(QString::fromUtf8("quat xyzw: —"));
    }
  }
}

}  // namespace

namespace
{
QApplication * g_qapp = nullptr;

void qt_signal_handler(int /*sig*/)
{
  if (g_qapp != nullptr) {
    QMetaObject::invokeMethod(g_qapp, "quit", Qt::QueuedConnection);
  }
}
}  // namespace

int RunGraspQtUiApp(
  const std::shared_ptr<GraspRosNode> & ros_node,
  int argc, char ** argv,
  ShutdownHook on_shutdown)
{
  // 本函数只运行 Qt 事件循环；ROS 节点由 grasp_qt_ui_node.cpp 的 spin 线程驱动。
  QApplication app(argc, argv);
  g_qapp = &app;
  std::signal(SIGINT, qt_signal_handler);
  std::signal(SIGTERM, qt_signal_handler);
  app.setQuitOnLastWindowClosed(true);
  app.setStyleSheet(QString::fromUtf8(R"(
    QMainWindow { background: #eef1f6; }
    QWidget#CentralGrasp { background: #eef1f6; }
    QGroupBox {
      font-weight: 600;
      font-size: 13px;
      border: 1px solid #c5cad6;
      border-radius: 8px;
      margin-top: 12px;
      padding: 10px 12px 12px 12px;
      background: #ffffff;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 12px;
      padding: 0 6px;
      color: #1e2430;
    }
    QPushButton {
      padding: 7px 14px;
      border-radius: 5px;
      border: 1px solid #b4bac7;
      background: #f8f9fc;
      min-height: 22px;
    }
    QPushButton:hover { background: #e9ecf4; border-color: #9aa3b4; }
    QPushButton:pressed { background: #dde2ee; }
    QTextEdit, QComboBox, QTableWidget {
      border: 1px solid #c5cad6;
      border-radius: 5px;
      background: #ffffff;
      padding: 4px;
      selection-background-color: #c9d8f0;
    }
    QTextEdit { padding: 6px; }
    QLabel { color: #252b38; }
    QTabWidget::pane { border: 1px solid #c5cad6; background: #ffffff; border-radius: 6px; }
  )"));

  QMainWindow win;
  win.setWindowTitle(QString::fromUtf8("Nova 抓取测试 (MoveIt2)"));
  auto * central = new QWidget(&win);
  central->setObjectName(QStringLiteral("CentralGrasp"));
  auto * root = new QVBoxLayout(central);
  root->setContentsMargins(14, 12, 14, 12);
  root->setSpacing(10);

  auto * tabs = new QTabWidget(central);
  auto * corner_status = new QWidget(tabs);
  auto * corner_status_layout = new QHBoxLayout(corner_status);
  corner_status_layout->setContentsMargins(0, 0, 4, 0);
  corner_status_layout->setSpacing(8);
  auto * graspnet_comm_label = new QLabel(
    QString::fromUtf8("● GraspNet: 检测中..."), corner_status);
  graspnet_comm_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  graspnet_comm_label->setMinimumWidth(250);
  graspnet_comm_label->setVisible(false);
  auto * comm_label = new QLabel(QString::fromUtf8("● 机械臂通信: 检测中..."), corner_status);
  comm_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  comm_label->setMinimumWidth(220);
  corner_status_layout->addWidget(graspnet_comm_label);
  corner_status_layout->addWidget(comm_label);
  tabs->setCornerWidget(corner_status, Qt::TopRightCorner);

  auto * status_tab = new QWidget();
  auto * status_root = new QVBoxLayout(status_tab);
  status_root->setContentsMargins(0, 0, 0, 0);
  status_root->setSpacing(8);

  auto * status_split = new QSplitter(Qt::Vertical, status_tab);
  status_split->setChildrenCollapsible(false);

  auto * arms_host = new QWidget(status_split);
  auto * arms_layout = new QHBoxLayout(arms_host);
  arms_layout->setContentsMargins(0, 0, 0, 0);
  arms_layout->setSpacing(10);
  arms_layout->setAlignment(Qt::AlignTop);

  ArmControlPanel j1_panel;
  ArmControlPanel j2_panel;
  GraspRosSnapshot last_snap;

  auto get_j1_ee = [&]() { return last_snap.arm1_ee; };
  auto get_j2_ee = [&]() { return last_snap.arm2_ee; };

  arms_layout->addWidget(
    build_arm_panel(arms_host, j1_panel, 0, "J1_6", ros_node, get_j1_ee), 1, Qt::AlignTop);
  arms_layout->addWidget(
    build_arm_panel(arms_host, j2_panel, 1, "J2_6", ros_node, get_j2_ee), 1, Qt::AlignTop);

  auto * status_log_group = new QGroupBox(
    QString::fromUtf8("调试日志（逆解 / 关节发送）"), status_split);
  auto * status_log_view = new QTextEdit(status_log_group);
  status_log_view->setReadOnly(true);
  status_log_view->setMinimumHeight(140);
  status_log_view->setPlaceholderText(
    QString::fromUtf8("本页操作的 IK 请求/结果、/joint_command 发送记录；失败(NG) 以红色显示"));
  auto * btn_clear_status_log = new QPushButton(
    QString::fromUtf8("清空调试日志"), status_log_group);
  auto * status_log_btns = new QHBoxLayout();
  status_log_btns->addStretch(1);
  status_log_btns->addWidget(btn_clear_status_log);
  auto * status_log_layout = new QVBoxLayout(status_log_group);
  status_log_layout->setContentsMargins(8, 8, 8, 8);
  status_log_layout->setSpacing(6);
  status_log_layout->addWidget(status_log_view, 1);
  status_log_layout->addLayout(status_log_btns);

  status_split->addWidget(arms_host);
  status_split->addWidget(status_log_group);
  status_split->setStretchFactor(0, 3);
  status_split->setStretchFactor(1, 1);
  status_split->setSizes({640, 200});
  status_root->addWidget(status_split, 1);
  tabs->addTab(status_tab, QString::fromUtf8("机械臂状态"));

  auto * grasp_tab = new QWidget();
  auto * grasp_layout = new QVBoxLayout(grasp_tab);
  grasp_layout->setContentsMargins(8, 8, 8, 8);
  grasp_layout->setSpacing(8);

  auto * top_split = new QSplitter(Qt::Horizontal, grasp_tab);
  top_split->setChildrenCollapsible(false);

  auto * box_group = new QGroupBox(QString::fromUtf8("盒子位姿 (/box_pose)"), top_split);
  auto * box_view = new QTextEdit(box_group);
  box_view->setReadOnly(true);
  box_view->setMaximumHeight(130);
  box_view->setMinimumHeight(90);
  box_view->setPlainText(QString::fromUtf8("等待 /box_pose ..."));
  auto * box_layout = new QVBoxLayout(box_group);
  box_layout->setContentsMargins(8, 8, 8, 8);
  box_layout->addWidget(box_view);

  auto * plan_group = new QGroupBox(QString::fromUtf8("抓取规划预览"), top_split);
  auto * plan_view = new QTextEdit(plan_group);
  plan_view->setReadOnly(true);
  plan_view->setMaximumHeight(130);
  plan_view->setMinimumHeight(90);
  plan_view->setPlainText(
    QString::fromUtf8("点击「计算抓取」生成 raise → pregrasp → grasp → lift"));
  auto * plan_layout = new QVBoxLayout(plan_group);
  plan_layout->setContentsMargins(8, 8, 8, 8);
  plan_layout->addWidget(plan_view);

  top_split->addWidget(box_group);
  top_split->addWidget(plan_group);
  top_split->setStretchFactor(0, 1);
  top_split->setStretchFactor(1, 1);
  grasp_layout->addWidget(top_split, 0);

  auto * bottom_split = new QSplitter(Qt::Horizontal, grasp_tab);
  bottom_split->setChildrenCollapsible(false);

  auto * log_group = new QGroupBox(QString::fromUtf8("日志"), bottom_split);
  auto * log_view = new QTextEdit(log_group);
  log_view->setReadOnly(true);
  log_view->setMinimumHeight(220);
  auto * log_layout = new QVBoxLayout(log_group);
  log_layout->setContentsMargins(8, 8, 8, 8);
  log_layout->addWidget(log_view);

  auto * control_group = new QGroupBox(QString::fromUtf8("抓取参数 / 操作"), bottom_split);
  control_group->setMinimumWidth(320);
  control_group->setMaximumWidth(380);
  auto * ctrl_layout = new QVBoxLayout(control_group);
  ctrl_layout->setContentsMargins(10, 10, 10, 10);
  ctrl_layout->setSpacing(8);

  auto * ik_label = new QLabel(QString::fromUtf8("MoveIt IK: 检测中..."), control_group);
  auto * status_label = new QLabel(QString::fromUtf8("抓取状态: —"), control_group);
  ik_label->setWordWrap(true);
  status_label->setWordWrap(true);
  ctrl_layout->addWidget(ik_label);
  ctrl_layout->addWidget(status_label);

  auto * traj_group = new QGroupBox(
    QString::fromUtf8("轨迹参数（仅抓取测试 /box_pose）"), control_group);
  auto * traj_form = new QFormLayout(traj_group);
  traj_form->setSpacing(6);
  auto * spin_pregrasp = make_metric_spin(traj_group, 0.0, 0.5, 0.01, 3,
    ros_node->planner_config().pregrasp_z_offset);
  auto * spin_lift = make_metric_spin(traj_group, 0.0, 0.5, 0.01, 3,
    ros_node->planner_config().lift_z_offset);
  auto * spin_box_z = make_metric_spin(traj_group, -0.2, 0.2, 0.005, 3,
    ros_node->planner_config().box_grasp_z_offset);
  auto * spin_split = make_metric_spin(traj_group, -1.0, 2.0, 0.01, 3,
    ros_node->planner_config().arm_split_x);
  spin_split->setToolTip(QString::fromUtf8("盒子 x 大于该值用 J2，否则用 J1"));
  auto * spin_yaw = make_metric_spin(traj_group, -180.0, 180.0, 15.0, 1,
    ros_node->planner_config().grasp_yaw_offset_deg);
  spin_yaw->setToolTip(QString::fromUtf8(
    "下降前在盒子上方绕竖直轴旋转角度；默认 90°，0=不转"));
  traj_form->addRow(QString::fromUtf8("预抓取 Z+ (m)"), spin_pregrasp);
  traj_form->addRow(QString::fromUtf8("抬升 Z+ (m)"), spin_lift);
  traj_form->addRow(QString::fromUtf8("抓取偏航 (°)"), spin_yaw);
  traj_form->addRow(QString::fromUtf8("盒子 Z 偏移 (m)"), spin_box_z);
  traj_form->addRow(QString::fromUtf8("选臂分界 X (m)"), spin_split);
  ctrl_layout->addWidget(traj_group);

  auto * grip_group = new QGroupBox(QString::fromUtf8("夹爪开口"), control_group);
  auto * grip_form = new QFormLayout(grip_group);
  grip_form->setSpacing(6);
  auto * spin_grip_open = make_metric_spin(grip_group, 0.0, 0.08, 0.005, 3,
    ros_node->planner_config().gripper_open_m);
  auto * spin_grip_close = make_metric_spin(grip_group, 0.0, 0.08, 0.005, 3,
    ros_node->planner_config().gripper_close_m);
  spin_grip_open->setToolTip(QString::fromUtf8("接近/执行前夹爪张开距离，0=全闭，0.08=全开"));
  spin_grip_close->setToolTip(QString::fromUtf8("抓取闭合时的开口距离（可留小缝避免夹死）"));
  grip_form->addRow(QString::fromUtf8("张开距离 (m)"), spin_grip_open);
  grip_form->addRow(QString::fromUtf8("闭合距离 (m)"), spin_grip_close);
  ctrl_layout->addWidget(grip_group);

  auto * timing_group = new QGroupBox(QString::fromUtf8("时序"), control_group);
  auto * timing_form = new QFormLayout(timing_group);
  timing_form->setSpacing(6);
  auto * spin_step_settle = make_metric_spin(timing_group, 0.1, 10.0, 0.1, 2,
    ros_node->step_settle_sec());
  auto * spin_grip_settle = make_metric_spin(timing_group, 0.1, 5.0, 0.1, 2,
    ros_node->gripper_settle_sec());
  timing_form->addRow(QString::fromUtf8("位姿停顿 (s)"), spin_step_settle);
  spin_step_settle->setToolTip(
    QString::fromUtf8("末端几乎不动后，再等待这么久，然后判定是否到位（不再用行程墙钟超时）"));
  timing_form->addRow(QString::fromUtf8("夹爪停顿 (s)"), spin_grip_settle);
  ctrl_layout->addWidget(timing_group);

  auto * opt_group = new QGroupBox(QString::fromUtf8("姿态策略"), control_group);
  auto * opt_hint = new QLabel(
    QString::fromUtf8("始终从上往下抓（忽略盒子朝向）"), opt_group);
  opt_hint->setWordWrap(true);
  auto * opt_layout = new QVBoxLayout(opt_group);
  opt_layout->addWidget(opt_hint);
  ctrl_layout->addWidget(opt_group);

  auto * btn_compute = new QPushButton(QString::fromUtf8("计算抓取"), control_group);
  auto * btn_step = new QPushButton(QString::fromUtf8("单步"), control_group);
  auto * btn_execute = new QPushButton(QString::fromUtf8("执行抓取"), control_group);
  auto * btn_clear_log = new QPushButton(QString::fromUtf8("清空日志"), control_group);
  btn_compute->setMinimumHeight(30);
  btn_step->setMinimumHeight(30);
  btn_execute->setMinimumHeight(30);
  btn_step->setToolTip(QString::fromUtf8("计算后点一次走一个路点，直到全部走完"));
  ctrl_layout->addWidget(btn_compute);
  ctrl_layout->addWidget(btn_step);
  ctrl_layout->addWidget(btn_execute);
  ctrl_layout->addWidget(btn_clear_log);
  ctrl_layout->addStretch(1);

  bottom_split->addWidget(log_group);
  bottom_split->addWidget(control_group);
  bottom_split->setStretchFactor(0, 3);
  bottom_split->setStretchFactor(1, 2);
  grasp_layout->addWidget(bottom_split, 1);

  tabs->addTab(grasp_tab, QString::fromUtf8("抓取测试"));

  // GraspNet 页分为“连续输入显示”和“点击后冻结的选择结果”。
  // 这里只负责 UI；TF、排序和双臂 IK 都封装在 GraspRosNode。
  auto * graspnet_tab = new QWidget();
  auto * graspnet_root = new QVBoxLayout(graspnet_tab);
  graspnet_root->setContentsMargins(8, 8, 8, 8);
  graspnet_root->setSpacing(8);

  auto * graspnet_top = new QSplitter(Qt::Horizontal, graspnet_tab);
  auto * graspnet_feed_group = new QGroupBox(
    QString::fromUtf8("实时 GraspNet 候选"), graspnet_top);
  auto * graspnet_feed_table = new QTableWidget(graspnet_feed_group);
  graspnet_feed_table->setColumnCount(8);
  graspnet_feed_table->setHorizontalHeaderLabels(
    {QStringLiteral("#"), QString::fromUtf8("分数"),
      QStringLiteral("X (m)"), QStringLiteral("Y (m)"), QStringLiteral("Z (m)"),
      QString::fromUtf8("Roll (°)"), QString::fromUtf8("Pitch (°)"),
      QString::fromUtf8("Yaw (°)")});
  graspnet_feed_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  graspnet_feed_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  graspnet_feed_table->setAlternatingRowColors(true);
  graspnet_feed_table->verticalHeader()->setVisible(false);
  graspnet_feed_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  graspnet_feed_table->horizontalHeader()->setStretchLastSection(false);
  auto * graspnet_feed_layout = new QVBoxLayout(graspnet_feed_group);
  graspnet_feed_layout->setContentsMargins(8, 8, 8, 8);
  graspnet_feed_layout->addWidget(graspnet_feed_table);

  auto * graspnet_selected_group = new QGroupBox(
    QString::fromUtf8("转换后 base_link 候选"), graspnet_top);
  auto * graspnet_selected_table = new QTableWidget(graspnet_selected_group);
  graspnet_selected_table->setColumnCount(8);
  graspnet_selected_table->setHorizontalHeaderLabels(
    {QStringLiteral("#"), QString::fromUtf8("分数"),
      QStringLiteral("X (m)"), QStringLiteral("Y (m)"), QStringLiteral("Z (m)"),
      QString::fromUtf8("Roll (°)"), QString::fromUtf8("Pitch (°)"),
      QString::fromUtf8("Yaw (°)")});
  graspnet_selected_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  graspnet_selected_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  graspnet_selected_table->verticalHeader()->setVisible(false);
  graspnet_selected_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  graspnet_selected_table->horizontalHeader()->setStretchLastSection(false);
  auto * graspnet_box_ref_label = new QLabel(graspnet_selected_group);
  graspnet_box_ref_label->setWordWrap(true);
  graspnet_box_ref_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  // 三行对比信息需要稳定高度，避免被表格挤成一条缝。
  graspnet_box_ref_label->setMinimumHeight(96);
  graspnet_box_ref_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  graspnet_box_ref_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  graspnet_box_ref_label->setStyleSheet(
    QStringLiteral(
      "QLabel {"
      "  background: #e8f4fc;"
      "  border: 2px solid #5dade2;"
      "  border-radius: 4px;"
      "  padding: 6px 8px;"
      "  color: #2c3e50;"
      "  font-family: monospace;"
      "  font-size: 12px;"
      "}"));
  graspnet_box_ref_label->setText(
    QString::fromUtf8("标准盒子中心 /box_pose：等待数据 ..."));
  auto * graspnet_selected_layout = new QVBoxLayout(graspnet_selected_group);
  graspnet_selected_layout->setContentsMargins(8, 8, 8, 8);
  graspnet_selected_layout->addWidget(graspnet_selected_table, 1);
  graspnet_selected_layout->addWidget(graspnet_box_ref_label, 0);
  graspnet_top->addWidget(graspnet_feed_group);
  graspnet_top->addWidget(graspnet_selected_group);
  graspnet_top->setStretchFactor(0, 1);
  graspnet_top->setStretchFactor(1, 1);

  // 垂直分隔：上=候选表，下=复位(紧凑)+日志(主要)+右侧操作
  auto * graspnet_vsplit = new QSplitter(Qt::Vertical, graspnet_tab);
  graspnet_vsplit->setChildrenCollapsible(false);
  graspnet_vsplit->addWidget(graspnet_top);

  // 复位位姿：两行 arm0 / arm1（紧凑，不占日志高度）
  struct ResetPoseRow
  {
    int arm_id{0};
    QDoubleSpinBox * x{nullptr};
    QDoubleSpinBox * y{nullptr};
    QDoubleSpinBox * z{nullptr};
    QDoubleSpinBox * roll{nullptr};
    QDoubleSpinBox * pitch{nullptr};
    QDoubleSpinBox * yaw{nullptr};
    QPushButton * btn_get{nullptr};
    QPushButton * btn_reset{nullptr};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
    bool have_quat{false};
    bool orientation_dirty{false};
  };

  auto make_reset_spin = [](QWidget * parent, double lo, double hi, double step, int dec, double v) {
      auto * spin = new QDoubleSpinBox(parent);
      spin->setDecimals(dec);
      spin->setRange(lo, hi);
      spin->setSingleStep(step);
      spin->setValue(v);
      spin->setFixedWidth(78);
      return spin;
    };

  auto * graspnet_reset_group = new QGroupBox(QString::fromUtf8("复位位姿"));
  graspnet_reset_group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  auto * graspnet_reset_layout = new QVBoxLayout(graspnet_reset_group);
  graspnet_reset_layout->setContentsMargins(6, 6, 6, 6);
  graspnet_reset_layout->setSpacing(4);

  ResetPoseRow reset_rows[2];
  for (int i = 0; i < 2; ++i) {
    auto & row = reset_rows[i];
    row.arm_id = i;
    auto * line = new QWidget(graspnet_reset_group);
    auto * hl = new QHBoxLayout(line);
    hl->setContentsMargins(0, 0, 0, 0);
    hl->setSpacing(4);
    auto * arm_label = new QLabel(
      i == 0 ? QStringLiteral("arm0") : QStringLiteral("arm1"), line);
    arm_label->setFixedWidth(40);
    row.x = make_reset_spin(line, -2.0, 2.0, 0.01, 4, 0.0);
    row.y = make_reset_spin(line, -2.0, 2.0, 0.01, 4, 0.0);
    row.z = make_reset_spin(line, 0.0, 1.5, 0.01, 4, 0.4);
    row.roll = make_reset_spin(line, -180.0, 180.0, 1.0, 2, 180.0);
    row.pitch = make_reset_spin(line, -180.0, 180.0, 1.0, 2, 0.0);
    row.yaw = make_reset_spin(line, -180.0, 180.0, 1.0, 2, 0.0);
    row.btn_get = new QPushButton(QString::fromUtf8("获取当前位姿"), line);
    row.btn_reset = new QPushButton(QString::fromUtf8("复位"), line);
    row.btn_get->setFixedWidth(110);
    row.btn_reset->setFixedWidth(64);
    hl->addWidget(arm_label);
    hl->addWidget(new QLabel(QStringLiteral("X"), line));
    hl->addWidget(row.x);
    hl->addWidget(new QLabel(QStringLiteral("Y"), line));
    hl->addWidget(row.y);
    hl->addWidget(new QLabel(QStringLiteral("Z"), line));
    hl->addWidget(row.z);
    hl->addWidget(new QLabel(QStringLiteral("R"), line));
    hl->addWidget(row.roll);
    hl->addWidget(new QLabel(QStringLiteral("P"), line));
    hl->addWidget(row.pitch);
    hl->addWidget(new QLabel(QStringLiteral("Y"), line));
    hl->addWidget(row.yaw);
    hl->addStretch(1);
    hl->addWidget(row.btn_get);
    hl->addWidget(row.btn_reset);
    graspnet_reset_layout->addWidget(line);

    for (QDoubleSpinBox * spin : {row.roll, row.pitch, row.yaw}) {
      QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        [&row](double) { row.orientation_dirty = true; });
    }

    QObject::connect(row.btn_get, &QPushButton::clicked, [&row, get_j1_ee, get_j2_ee]() {
        const EePoseSnapshot ee = (row.arm_id == 0) ? get_j1_ee() : get_j2_ee();
        if (!ee.ok) {
          return;
        }
        {
          QSignalBlocker bx(row.x); row.x->setValue(ee.x);
          QSignalBlocker by(row.y); row.y->setValue(ee.y);
          QSignalBlocker bz(row.z); row.z->setValue(ee.z);
          QSignalBlocker br(row.roll); row.roll->setValue(ee.roll_deg);
          QSignalBlocker bp(row.pitch); row.pitch->setValue(ee.pitch_deg);
          QSignalBlocker byaw(row.yaw); row.yaw->setValue(ee.yaw_deg);
        }
        row.qx = ee.qx;
        row.qy = ee.qy;
        row.qz = ee.qz;
        row.qw = ee.qw;
        row.have_quat = true;
        row.orientation_dirty = false;
      });

    QObject::connect(row.btn_reset, &QPushButton::clicked,
      [&row, ros_node]() {
        // 复位打断抓取序列，清空 busy，避免一直报「执行器忙」
        ros_node->abort_grasp_execution();
        const double x = row.x->value();
        const double y = row.y->value();
        const double z = row.z->value();
        if (!row.orientation_dirty && row.have_quat) {
          ros_node->send_arm_pose_goal_quat(
            row.arm_id, x, y, z, row.qx, row.qy, row.qz, row.qw);
        } else {
          ros_node->send_arm_pose_goal(
            row.arm_id, x, y, z,
            row.roll->value(), row.pitch->value(), row.yaw->value());
        }
      });
  }

  auto * graspnet_bottom = new QSplitter(Qt::Horizontal, graspnet_vsplit);
  graspnet_bottom->setChildrenCollapsible(false);

  auto * graspnet_left = new QWidget(graspnet_bottom);
  auto * graspnet_left_layout = new QVBoxLayout(graspnet_left);
  graspnet_left_layout->setContentsMargins(0, 0, 0, 0);
  graspnet_left_layout->setSpacing(6);
  graspnet_left_layout->addWidget(graspnet_reset_group, 0);

  auto * graspnet_log_group = new QGroupBox(QString::fromUtf8("GraspNet 抓取日志"), graspnet_left);
  auto * graspnet_log_view = new QTextEdit(graspnet_log_group);
  graspnet_log_view->setReadOnly(true);
  graspnet_log_view->setMinimumHeight(160);
  auto * graspnet_log_layout = new QVBoxLayout(graspnet_log_group);
  graspnet_log_layout->setContentsMargins(8, 8, 8, 8);
  graspnet_log_layout->addWidget(graspnet_log_view);
  graspnet_left_layout->addWidget(graspnet_log_group, 1);

  auto * graspnet_control = new QGroupBox(QString::fromUtf8("候选选择 / 操作"), graspnet_bottom);
  graspnet_control->setMinimumWidth(320);
  graspnet_control->setMaximumWidth(380);
  auto * graspnet_ctrl_layout = new QVBoxLayout(graspnet_control);
  graspnet_ctrl_layout->setContentsMargins(10, 10, 10, 10);
  graspnet_ctrl_layout->setSpacing(8);
  auto * graspnet_hint = new QLabel(
    QString::fromUtf8(
      "点击时冻结最新一帧；相机系经 TF 转到 base_link。\n"
      "优先局部 +X 接近方向朝下的顶抓，两臂均做 IK，选择关节移动较小的一臂。"),
    graspnet_control);
  graspnet_hint->setWordWrap(true);
  graspnet_ctrl_layout->addWidget(graspnet_hint);

  auto * graspnet_topic_group = new QGroupBox(QString::fromUtf8("订阅话题"), graspnet_control);
  auto * graspnet_topic_layout = new QHBoxLayout(graspnet_topic_group);
  auto * graspnet_topic_edit = new QLineEdit(
    QString::fromStdString(ros_node->get_parameter("graspnet_topic").as_string()),
    graspnet_topic_group);
  auto * btn_graspnet_topic = new QPushButton(QString::fromUtf8("应用"), graspnet_topic_group);
  graspnet_topic_edit->setPlaceholderText(
    QStringLiteral("/yolo_graspnet/collision_free_grasps"));
  graspnet_topic_layout->addWidget(graspnet_topic_edit, 1);
  graspnet_topic_layout->addWidget(btn_graspnet_topic);
  graspnet_ctrl_layout->addWidget(graspnet_topic_group);

  auto * graspnet_filter_group = new QGroupBox(QString::fromUtf8("顶部抓取判定"), graspnet_control);
  auto * graspnet_filter_form = new QFormLayout(graspnet_filter_group);
  auto * spin_graspnet_top_angle = make_metric_spin(
    graspnet_filter_group, 1.0, 89.0, 5.0, 1,
    ros_node->get_parameter("graspnet_top_max_angle_deg").as_double());
  spin_graspnet_top_angle->setSuffix(QString::fromUtf8(" °"));
  spin_graspnet_top_angle->setToolTip(
    QString::fromUtf8("GraspNet 局部 -X 与 base_link 竖直向下方向的最大夹角"));
  graspnet_filter_form->addRow(QString::fromUtf8("最大倾角"), spin_graspnet_top_angle);
  graspnet_ctrl_layout->addWidget(graspnet_filter_group);

  const auto gn_cfg0 = ros_node->planner_config();
  auto * graspnet_traj_group = new QGroupBox(
    QString::fromUtf8("GraspNet 轨迹参数（独立于抓取测试）"), graspnet_control);
  auto * graspnet_traj_form = new QFormLayout(graspnet_traj_group);
  auto * spin_gn_pre = make_metric_spin(
    graspnet_traj_group, 0.05, 0.30, 0.01, 3, gn_cfg0.graspnet_pregrasp_distance);
  auto * spin_gn_lift = make_metric_spin(
    graspnet_traj_group, 0.05, 0.30, 0.01, 3, gn_cfg0.graspnet_lift_z_offset);
  auto * spin_gn_tcp = make_metric_spin(
    graspnet_traj_group, 0.05, 0.30, 0.005, 3, gn_cfg0.graspnet_ee_tcp_z_offset);
  auto * spin_gn_clear = make_metric_spin(
    graspnet_traj_group, 0.05, 0.30, 0.01, 3, gn_cfg0.graspnet_min_approach_clearance);
  spin_gn_pre->setToolTip(QString::fromUtf8("沿接近轴后退距离；与「抓取测试」预抓取 Z+ 无关"));
  spin_gn_lift->setToolTip(QString::fromUtf8("闭合后沿世界 +Z 抬升；与「抓取测试」抬升无关"));
  spin_gn_tcp->setToolTip(QString::fromUtf8("腕部→两指中心；GraspNet 默认 0.12，盒子页默认 0.20"));
  spin_gn_clear->setToolTip(QString::fromUtf8("预抓取最小净空下限（与后退距离取 max）"));
  graspnet_traj_form->addRow(QString::fromUtf8("预抓取后退 (m)"), spin_gn_pre);
  graspnet_traj_form->addRow(QString::fromUtf8("抬升 Z+ (m)"), spin_gn_lift);
  graspnet_traj_form->addRow(QString::fromUtf8("腕部→指尖 (m)"), spin_gn_tcp);
  graspnet_traj_form->addRow(QString::fromUtf8("最小净空 (m)"), spin_gn_clear);
  graspnet_ctrl_layout->addWidget(graspnet_traj_group);

  auto * btn_graspnet_select = new QPushButton(QString::fromUtf8("选择最优并计算"), graspnet_control);
  auto * btn_graspnet_step = new QPushButton(QString::fromUtf8("单步"), graspnet_control);
  auto * btn_graspnet_run = new QPushButton(QString::fromUtf8("使用最优解抓取"), graspnet_control);
  auto * btn_graspnet_execute = new QPushButton(QString::fromUtf8("抓取最新最优位姿"), graspnet_control);
  auto * btn_graspnet_clear = new QPushButton(QString::fromUtf8("清空日志"), graspnet_control);
  btn_graspnet_step->setToolTip(
    QString::fromUtf8("按「选择最优并计算」得到的同一规划，点一次走一个路点"));
  btn_graspnet_run->setToolTip(
    QString::fromUtf8(
      "连续执行「选择最优并计算」冻结的同一规划（不重新选解）。\n"
      "用于和「单步」对比自动序列是否一致。"));
  btn_graspnet_execute->setToolTip(
    QString::fromUtf8("重新从最新候选选解并连续执行（可能与刚才算的规划不同）"));
  for (auto * button : {
      btn_graspnet_select, btn_graspnet_step, btn_graspnet_run,
      btn_graspnet_execute, btn_graspnet_clear})
  {
    button->setMinimumHeight(30);
    graspnet_ctrl_layout->addWidget(button);
  }
  graspnet_ctrl_layout->addStretch(1);
  graspnet_bottom->addWidget(graspnet_left);
  graspnet_bottom->addWidget(graspnet_control);
  graspnet_bottom->setStretchFactor(0, 3);
  graspnet_bottom->setStretchFactor(1, 2);
  graspnet_vsplit->addWidget(graspnet_bottom);
  graspnet_vsplit->setStretchFactor(0, 2);
  graspnet_vsplit->setStretchFactor(1, 3);
  graspnet_vsplit->setSizes({320, 480});
  graspnet_root->addWidget(graspnet_vsplit, 1);
  const int graspnet_tab_index =
    tabs->addTab(graspnet_tab, QString::fromUtf8("GraspNet 抓取"));
  QObject::connect(tabs, &QTabWidget::currentChanged, [=](int index) {
      // 第三方通信状态只在 GraspNet 页出现，避免其它页面顶部信息过载。
      graspnet_comm_label->setVisible(index == graspnet_tab_index);
    });

  root->addWidget(tabs, 1);

  win.setCentralWidget(central);
  win.resize(1200, 860);

  const auto apply_box_params = [&]() {
      GraspPlannerConfig cfg = ros_node->planner_config();
      cfg.pregrasp_z_offset = spin_pregrasp->value();
      cfg.lift_z_offset = spin_lift->value();
      cfg.grasp_yaw_offset_deg = spin_yaw->value();
      cfg.box_grasp_z_offset = spin_box_z->value();
      cfg.arm_split_x = spin_split->value();
      cfg.gripper_open_m = spin_grip_open->value();
      cfg.gripper_close_m = spin_grip_close->value();
      // 不改动 GraspNet 专用字段
      ros_node->set_planner_config(cfg);
      ros_node->set_settle_timing(spin_step_settle->value(), spin_grip_settle->value());
    };
  const auto apply_graspnet_params = [&]() {
      GraspPlannerConfig cfg = ros_node->planner_config();
      cfg.graspnet_pregrasp_distance = spin_gn_pre->value();
      cfg.graspnet_lift_z_offset = spin_gn_lift->value();
      cfg.graspnet_ee_tcp_z_offset = spin_gn_tcp->value();
      cfg.graspnet_min_approach_clearance = spin_gn_clear->value();
      // 不改动抓取测试专用字段
      ros_node->set_planner_config(cfg);
    };

  auto * compute_watcher = new QFutureWatcher<GraspComputeResult>(&win);
  auto * execute_watcher = new QFutureWatcher<GraspExecuteResult>(&win);
  auto * step_watcher = new QFutureWatcher<GraspExecuteResult>(&win);
  auto * graspnet_select_watcher = new QFutureWatcher<GraspComputeResult>(&win);
  auto * graspnet_execute_watcher = new QFutureWatcher<GraspExecuteResult>(&win);
  auto * graspnet_run_watcher = new QFutureWatcher<GraspExecuteResult>(&win);
  auto * graspnet_step_watcher = new QFutureWatcher<GraspExecuteResult>(&win);

  const auto set_grasp_buttons_enabled = [&](bool enabled) {
      btn_compute->setEnabled(enabled);
      btn_step->setEnabled(enabled);
      btn_execute->setEnabled(enabled);
    };
  const auto set_graspnet_buttons_enabled = [&](bool enabled) {
      btn_graspnet_select->setEnabled(enabled);
      btn_graspnet_step->setEnabled(enabled);
      btn_graspnet_run->setEnabled(enabled);
      btn_graspnet_execute->setEnabled(enabled);
    };
  const auto graspnet_busy = [&]() {
      return graspnet_select_watcher->isRunning() ||
        graspnet_execute_watcher->isRunning() ||
        graspnet_run_watcher->isRunning() ||
        graspnet_step_watcher->isRunning();
    };
  const auto disable_manual_realtime = [&]() {
      for (ArmControlPanel * panel : {&j1_panel, &j2_panel}) {
        if (panel->realtime_cb != nullptr) {
          panel->realtime_cb->setChecked(false);
        }
      }
    };

  QObject::connect(compute_watcher, &QFutureWatcher<GraspComputeResult>::finished, [&]() {
      set_grasp_buttons_enabled(true);
      const GraspComputeResult result = compute_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });

  QObject::connect(execute_watcher, &QFutureWatcher<GraspExecuteResult>::finished, [&]() {
      set_grasp_buttons_enabled(true);
      const GraspExecuteResult result = execute_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });

  QObject::connect(step_watcher, &QFutureWatcher<GraspExecuteResult>::finished, [&]() {
      set_grasp_buttons_enabled(true);
      const GraspExecuteResult result = step_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      } else if (result.finished) {
        QMessageBox::information(
          &win, QString::fromUtf8("单步完成"),
          QString::fromUtf8("全部路点已走完（%1/%2）。")
            .arg(result.step_index).arg(result.step_count));
      }
    });

  QObject::connect(
    graspnet_select_watcher, &QFutureWatcher<GraspComputeResult>::finished, [&]() {
      set_graspnet_buttons_enabled(true);
      const auto result = graspnet_select_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });
  QObject::connect(
    graspnet_execute_watcher, &QFutureWatcher<GraspExecuteResult>::finished, [&]() {
      set_graspnet_buttons_enabled(true);
      const auto result = graspnet_execute_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });
  QObject::connect(
    graspnet_run_watcher, &QFutureWatcher<GraspExecuteResult>::finished, [&]() {
      set_graspnet_buttons_enabled(true);
      const auto result = graspnet_run_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });
  QObject::connect(
    graspnet_step_watcher, &QFutureWatcher<GraspExecuteResult>::finished, [&]() {
      set_graspnet_buttons_enabled(true);
      const auto result = graspnet_step_watcher->result();
      if (!result.ok) {
        show_grasp_error(&win, result);
      }
    });

  QObject::connect(btn_compute, &QPushButton::clicked, [&]() {
      if (compute_watcher->isRunning()) {
        return;
      }
      disable_manual_realtime();
      apply_box_params();
      const auto snap = ros_node->snapshot();
      if (!snap.arm_comm_ok) {
        GraspComputeResult err;
        err.error_title = "机械臂通信异常";
        err.error_message = snap.arm_comm_summary.empty() ?
          "未收到有效的 /joint_states，请确认仿真或真机已连接。" :
          snap.arm_comm_summary;
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.has_box_pose) {
        GraspComputeResult err;
        err.error_title = "缺少盒子位姿";
        err.error_message = "尚未收到 /box_pose，请先启动仿真并发布盒子位姿。";
        show_grasp_error(&win, err);
        return;
      }
      set_grasp_buttons_enabled(false);
      compute_watcher->setFuture(QtConcurrent::run(
          [ros_node]() { return ros_node->compute_grasp_from_box_detailed(); }));
    });

  QObject::connect(btn_step, &QPushButton::clicked, [&]() {
      if (step_watcher->isRunning() || execute_watcher->isRunning()) {
        return;
      }
      disable_manual_realtime();
      apply_box_params();
      const auto snap = ros_node->snapshot();
      if (!snap.has_plan) {
        GraspExecuteResult err;
        err.error_title = "无抓取规划";
        err.error_message = "请先点击「计算抓取」。";
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.arm_comm_ok) {
        GraspExecuteResult err;
        err.error_title = "机械臂通信异常";
        err.error_message = snap.arm_comm_summary.empty() ?
          "/joint_states 不可用。" : snap.arm_comm_summary;
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.ik_service_ready) {
        GraspExecuteResult err;
        err.error_title = "IK 不可用";
        err.error_message = "请用 grasp_stack.launch.py 启动。";
        show_grasp_error(&win, err);
        return;
      }
      set_grasp_buttons_enabled(false);
      step_watcher->setFuture(QtConcurrent::run(
          [ros_node]() { return ros_node->execute_step_once_detailed(); }));
    });

  QObject::connect(btn_execute, &QPushButton::clicked, [&]() {
      if (execute_watcher->isRunning()) {
        return;
      }
      disable_manual_realtime();
      apply_box_params();
      const auto snap = ros_node->snapshot();
      if (!snap.arm_comm_ok) {
        GraspExecuteResult err;
        err.error_title = "机械臂通信异常";
        err.error_message = snap.arm_comm_summary.empty() ?
          "无法执行抓取：/joint_states 不可用或已超时。" :
          snap.arm_comm_summary;
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.ik_service_ready) {
        GraspExecuteResult err;
        err.error_title = "IK 不可用";
        err.error_message = "本包 MoveIt /compute_ik 未就绪，请用 ros2 launch nova_grasp_moveit grasp_stack.launch.py 启动。";
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.has_plan) {
        GraspExecuteResult err;
        err.error_title = "无抓取规划";
        err.error_message = "请先点击「计算抓取」生成规划。";
        show_grasp_error(&win, err);
        return;
      }
      set_grasp_buttons_enabled(false);
      execute_watcher->setFuture(QtConcurrent::run(
          [ros_node]() { return ros_node->execute_last_plan_detailed(); }));
    });

  QObject::connect(btn_graspnet_topic, &QPushButton::clicked, [&]() {
      std::string error;
      const std::string topic = graspnet_topic_edit->text().trimmed().toStdString();
      if (!ros_node->set_graspnet_topic(topic, &error)) {
        GraspComputeResult failed;
        failed.error_title = "切换 GraspNet 话题失败";
        failed.error_message = error;
        show_grasp_error(&win, failed);
        return;
      }
      graspnet_topic_edit->setText(QString::fromStdString(topic));
    });
  QObject::connect(
    graspnet_topic_edit, &QLineEdit::returnPressed,
    btn_graspnet_topic, &QPushButton::click);

  QObject::connect(btn_graspnet_select, &QPushButton::clicked, [&]() {
      if (graspnet_busy()) {
        return;
      }
      disable_manual_realtime();
      apply_graspnet_params();
      const auto snap = ros_node->snapshot();
      if (!snap.arm_comm_ok || !snap.ik_service_ready || !snap.has_graspnet_candidates) {
        GraspComputeResult err;
        err.error_title = "GraspNet 抓取未就绪";
        if (!snap.has_graspnet_candidates) {
          err.error_message =
            "尚未收到 " + snap.graspnet_topic + " 中的有效 GraspNet JSON 候选。";
        } else if (!snap.arm_comm_ok) {
          err.error_message = snap.arm_comm_summary;
        } else {
          err.error_message = "/compute_ik 未就绪。";
        }
        show_grasp_error(&win, err);
        return;
      }
      // 候选筛选最多会进行多次同步 IK 检查，必须放到 QtConcurrent，
      // 否则 MoveIt 响应期间窗口会停止刷新。
      set_graspnet_buttons_enabled(false);
      const double angle = spin_graspnet_top_angle->value();
      graspnet_select_watcher->setFuture(QtConcurrent::run(
          [ros_node, angle]() {
            return ros_node->compute_grasp_from_graspnet_detailed(angle);
          }));
    });

  QObject::connect(btn_graspnet_step, &QPushButton::clicked, [&]() {
      if (graspnet_busy()) {
        return;
      }
      disable_manual_realtime();
      apply_graspnet_params();
      const auto snap = ros_node->snapshot();
      if (!snap.has_selected_graspnet_pose || !snap.has_plan) {
        GraspExecuteResult err;
        err.error_title = "未选择 GraspNet 位姿";
        err.error_message = "请先点击「选择最优并计算」。";
        show_grasp_error(&win, err);
        return;
      }
      set_graspnet_buttons_enabled(false);
      graspnet_step_watcher->setFuture(QtConcurrent::run(
          [ros_node]() { return ros_node->execute_step_once_detailed(); }));
    });

  QObject::connect(btn_graspnet_run, &QPushButton::clicked, [&]() {
      if (graspnet_busy()) {
        return;
      }
      disable_manual_realtime();
      apply_graspnet_params();
      const auto snap = ros_node->snapshot();
      if (!snap.has_selected_graspnet_pose || !snap.has_plan) {
        GraspExecuteResult err;
        err.error_title = "未选择 GraspNet 位姿";
        err.error_message =
          "请先点击「选择最优并计算」，再点「使用最优解抓取」。\n"
          "本按钮连续执行已冻结规划，与「单步」用同一套路点。";
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.arm_comm_ok) {
        GraspExecuteResult err;
        err.error_title = "机械臂通信异常";
        err.error_message = snap.arm_comm_summary.empty() ?
          "/joint_states 不可用。" : snap.arm_comm_summary;
        show_grasp_error(&win, err);
        return;
      }
      if (!snap.ik_service_ready) {
        GraspExecuteResult err;
        err.error_title = "IK 不可用";
        err.error_message = "/compute_ik 未就绪。";
        show_grasp_error(&win, err);
        return;
      }
      set_graspnet_buttons_enabled(false);
      graspnet_run_watcher->setFuture(QtConcurrent::run(
          [ros_node]() { return ros_node->execute_last_plan_detailed(); }));
    });

  QObject::connect(btn_graspnet_execute, &QPushButton::clicked, [&]() {
      if (graspnet_busy()) {
        return;
      }
      disable_manual_realtime();
      apply_graspnet_params();
      const auto snap = ros_node->snapshot();
      if (!snap.arm_comm_ok || !snap.ik_service_ready || !snap.has_graspnet_candidates) {
        GraspExecuteResult err;
        err.error_title = "GraspNet 抓取未就绪";
        if (!snap.has_graspnet_candidates) {
          err.error_message =
            "尚未收到 " + snap.graspnet_topic + " 中的有效 GraspNet JSON 候选。";
        } else if (!snap.arm_comm_ok) {
          err.error_message = snap.arm_comm_summary;
        } else {
          err.error_message = "/compute_ik 未就绪。";
        }
        show_grasp_error(&win, err);
        return;
      }
      set_graspnet_buttons_enabled(false);
      const double angle = spin_graspnet_top_angle->value();
      graspnet_execute_watcher->setFuture(QtConcurrent::run(
          [ros_node, angle]() {
            const auto computed = ros_node->compute_grasp_from_graspnet_detailed(angle);
            if (!computed.ok) {
              GraspExecuteResult failed;
              failed.error_title = computed.error_title;
              failed.error_message = computed.error_message;
              return failed;
            }
            return ros_node->execute_last_plan_detailed();
          }));
    });

  auto clear_all_log_views = [&]() {
      ros_node->clear_logs();
      log_view->clear();
      status_log_view->clear();
      graspnet_log_view->clear();
    };
  QObject::connect(btn_clear_log, &QPushButton::clicked, clear_all_log_views);
  QObject::connect(btn_clear_status_log, &QPushButton::clicked, clear_all_log_views);
  QObject::connect(btn_graspnet_clear, &QPushButton::clicked, clear_all_log_views);

  std::vector<std::string> rendered_logs;
  std::vector<std::string> rendered_status_logs;
  std::vector<std::string> rendered_graspnet_logs;
  int rendered_graspnet_message_count = -1;
  int rendered_graspnet_base_revision = -1;
  int rendered_graspnet_selected_index = -2;

  QTimer ui_timer;
  QTimer shutdown_watchdog;

  QObject::connect(&app, &QCoreApplication::aboutToQuit, [&]() {
      ui_timer.stop();
      shutdown_watchdog.stop();
      if (on_shutdown) {
        on_shutdown();
      }
      g_qapp = nullptr;
    });

  QObject::connect(&ui_timer, &QTimer::timeout, [&]() {
      last_snap = ros_node->snapshot();

      if (last_snap.arm_comm_ok) {
        comm_label->setText(QString::fromUtf8("● 机械臂通信: 正常"));
        comm_label->setStyleSheet(
          "color: #1a8f42; font-weight: 600; padding: 2px 10px; background: #e8f7ee; "
          "border-radius: 4px;");
      } else if (last_snap.arm_comm_stale) {
        comm_label->setText(QString::fromUtf8("● 机械臂通信: 延迟"));
        comm_label->setStyleSheet(
          "color: #b36b00; font-weight: 600; padding: 2px 10px; background: #fff6e6; "
          "border-radius: 4px;");
      } else {
        comm_label->setText(QString::fromUtf8("● 机械臂通信: 异常"));
        comm_label->setStyleSheet(
          "color: #c0392b; font-weight: 600; padding: 2px 10px; background: #fdecea; "
          "border-radius: 4px;");
      }
      comm_label->setToolTip(QString::fromStdString(last_snap.arm_comm_summary));

      if (last_snap.graspnet_comm_ok && last_snap.has_graspnet_candidates) {
        graspnet_comm_label->setText(
          QString::fromUtf8("● GraspNet: 正常 (%1)")
          .arg(last_snap.graspnet_candidates.poses.size()));
        graspnet_comm_label->setStyleSheet(
          "color: #1a8f42; font-weight: 600; padding: 2px 10px; background: #e8f7ee; "
          "border-radius: 4px;");
      } else if (last_snap.graspnet_comm_ok) {
        graspnet_comm_label->setText(QString::fromUtf8("● GraspNet: 在线 / 无位姿"));
        graspnet_comm_label->setStyleSheet(
          "color: #b36b00; font-weight: 600; padding: 2px 10px; background: #fff6e6; "
          "border-radius: 4px;");
      } else if (last_snap.graspnet_comm_stale) {
        graspnet_comm_label->setText(QString::fromUtf8("● GraspNet: 发布延迟"));
        graspnet_comm_label->setStyleSheet(
          "color: #b36b00; font-weight: 600; padding: 2px 10px; background: #fff6e6; "
          "border-radius: 4px;");
      } else {
        graspnet_comm_label->setText(QString::fromUtf8("● GraspNet: 通信异常"));
        graspnet_comm_label->setStyleSheet(
          "color: #c0392b; font-weight: 600; padding: 2px 10px; background: #fdecea; "
          "border-radius: 4px;");
      }
      graspnet_comm_label->setToolTip(
        QString::fromUtf8("%1\n话题: %2")
        .arg(QString::fromStdString(last_snap.graspnet_comm_summary))
        .arg(QString::fromStdString(last_snap.graspnet_topic)));

      ik_label->setText(
        last_snap.ik_service_ready ?
        QString::fromUtf8("MoveIt IK: 就绪 (/compute_ik)") :
        QString::fromUtf8("MoveIt IK: 未就绪"));
      status_label->setText(
        QString::fromUtf8("抓取状态: %1%2")
          .arg(QString::fromStdString(last_snap.grasp_status))
          .arg(last_snap.executor_busy ? QString::fromUtf8(" [执行中]") : QString()));

      sync_arm_panel_ui(j1_panel, last_snap.arm1_joints, last_snap.arm1_ee);
      sync_arm_panel_ui(j2_panel, last_snap.arm2_joints, last_snap.arm2_ee);

      if (last_snap.has_box_pose) {
        const auto & p = last_snap.box_pose.pose.position;
        const auto & q = last_snap.box_pose.pose.orientation;
        box_view->setPlainText(
          QString::fromUtf8(
            "frame=%1\n"
            "pos=(%2, %3, %4)\n"
            "quat=(%5, %6, %7, %8)  ← 盒子姿态（抓取不用）\n"
            "执行时夹爪保持当前 EE 姿态，只改位置")
            .arg(QString::fromStdString(last_snap.box_pose.header.frame_id))
            .arg(p.x, 0, 'f', 4).arg(p.y, 0, 'f', 4).arg(p.z, 0, 'f', 4)
            .arg(q.x, 0, 'f', 4).arg(q.y, 0, 'f', 4).arg(q.z, 0, 'f', 4).arg(q.w, 0, 'f', 4));

        tf2::Quaternion box_q(q.x, q.y, q.z, q.w);
        box_q.normalize();
        double box_roll = 0.0;
        double box_pitch = 0.0;
        double box_yaw = 0.0;
        tf2::Matrix3x3(box_q).getRPY(box_roll, box_pitch, box_yaw);
        QString box_ref_text = QString::fromUtf8(
          "标准盒子中心 /box_pose  frame=%1\n"
          "XYZ=(%2, %3, %4) m    RPY=(%5, %6, %7)°")
          .arg(QString::fromStdString(
            last_snap.box_pose.header.frame_id.empty() ?
            "base_link" : last_snap.box_pose.header.frame_id))
          .arg(p.x, 0, 'f', 4).arg(p.y, 0, 'f', 4).arg(p.z, 0, 'f', 4)
          .arg(box_roll * kRadToDeg, 0, 'f', 2)
          .arg(box_pitch * kRadToDeg, 0, 'f', 2)
          .arg(box_yaw * kRadToDeg, 0, 'f', 2);

        // 与转换后表格对比：优先选中行，否则用最近候选。
        int cmp_index = -1;
        geometry_msgs::msg::Pose cmp_pose;
        bool have_cmp = false;
        if (last_snap.has_selected_graspnet_pose) {
          cmp_index = last_snap.selected_graspnet_index;
          cmp_pose = last_snap.selected_graspnet_pose.pose;
          have_cmp = true;
        } else if (!last_snap.graspnet_base_candidates.poses.empty()) {
          double best_err = std::numeric_limits<double>::infinity();
          for (size_t i = 0; i < last_snap.graspnet_base_candidates.poses.size(); ++i) {
            const auto & gp = last_snap.graspnet_base_candidates.poses[i].position;
            const double dx = gp.x - p.x;
            const double dy = gp.y - p.y;
            const double dz = gp.z - p.z;
            const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (err < best_err) {
              best_err = err;
              cmp_pose = last_snap.graspnet_base_candidates.poses[i];
              cmp_index =
                i < last_snap.graspnet_base_candidate_indices.size() ?
                last_snap.graspnet_base_candidate_indices[i] : static_cast<int>(i);
              have_cmp = true;
            }
          }
        }
        if (have_cmp) {
          const auto & gp = cmp_pose.position;
          const double dx = gp.x - p.x;
          const double dy = gp.y - p.y;
          const double dz = gp.z - p.z;
          const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
          box_ref_text += QString::fromUtf8(
            "\n对比#%1  XYZ=(%2, %3, %4)  Δ=(%5, %6, %7) cm  |Δ|=%8 cm")
            .arg(cmp_index)
            .arg(gp.x, 0, 'f', 4).arg(gp.y, 0, 'f', 4).arg(gp.z, 0, 'f', 4)
            .arg(dx * 100.0, 0, 'f', 1)
            .arg(dy * 100.0, 0, 'f', 1)
            .arg(dz * 100.0, 0, 'f', 1)
            .arg(err * 100.0, 0, 'f', 1);
        }
        graspnet_box_ref_label->setText(box_ref_text);
      } else {
        graspnet_box_ref_label->setText(
          QString::fromUtf8("标准盒子中心：等待 /box_pose ..."));
      }

      if (last_snap.graspnet_message_count != rendered_graspnet_message_count) {
        const auto & poses = last_snap.graspnet_candidates.poses;
        graspnet_feed_table->setRowCount(static_cast<int>(poses.size()));
        for (size_t i = 0; i < poses.size(); ++i) {
          const double score =
            i < last_snap.graspnet_scores.size() ?
            last_snap.graspnet_scores[i] : std::numeric_limits<double>::quiet_NaN();
          set_grasp_pose_table_row(
            graspnet_feed_table, static_cast<int>(i), static_cast<int>(i), score, poses[i]);
        }
        rendered_graspnet_message_count = last_snap.graspnet_message_count;
      }

      if (last_snap.graspnet_base_revision != rendered_graspnet_base_revision ||
        last_snap.selected_graspnet_index != rendered_graspnet_selected_index)
      {
        const auto & poses = last_snap.graspnet_base_candidates.poses;
        graspnet_selected_table->setRowCount(static_cast<int>(poses.size()));
        for (size_t row = 0; row < poses.size(); ++row) {
          const int source_index =
            row < last_snap.graspnet_base_candidate_indices.size() ?
            last_snap.graspnet_base_candidate_indices[row] : static_cast<int>(row);
          const double score =
            source_index >= 0 &&
            static_cast<size_t>(source_index) < last_snap.graspnet_scores.size() ?
            last_snap.graspnet_scores[static_cast<size_t>(source_index)] :
            std::numeric_limits<double>::quiet_NaN();
          const bool selected =
            last_snap.has_selected_graspnet_pose &&
            source_index == last_snap.selected_graspnet_index;
          const bool ik_ok =
            row < last_snap.graspnet_base_candidate_ik_ok.size() &&
            last_snap.graspnet_base_candidate_ik_ok[row] != 0;
          const bool a_corrected =
            row < last_snap.graspnet_base_candidate_a_corrected.size() &&
            last_snap.graspnet_base_candidate_a_corrected[row] != 0;
          QColor row_bg;
          if (selected) {
            row_bg = QColor(QStringLiteral("#c6efce"));  // 选中：绿
          } else if (!ik_ok) {
            row_bg = QColor(QStringLiteral("#f4c7c3"));  // IK 失败：红
          } else {
            row_bg = QColor(QStringLiteral("#fff2a8"));  // 可达未选：黄
          }
          // A 方案修正后的 RPY：仅 Roll/Pitch/Yaw 单元格浅蓝，不整行。
          const QColor rpy_bg = a_corrected ?
            QColor(QStringLiteral("#9ec5fe")) : QColor();
          set_grasp_pose_table_row(
            graspnet_selected_table,
            static_cast<int>(row),
            source_index,
            score,
            poses[row],
            row_bg,
            rpy_bg);
        }
        rendered_graspnet_base_revision = last_snap.graspnet_base_revision;
        rendered_graspnet_selected_index = last_snap.selected_graspnet_index;
      }

      if (last_snap.has_plan) {
        plan_view->setPlainText(QString::fromStdString(format_plan_summary(last_snap.last_plan)));
      }

      if (!log_view->textCursor().hasSelection() && last_snap.logs != rendered_logs) {
        refill_log_view(log_view, last_snap.logs);
        rendered_logs = last_snap.logs;
      }

      std::vector<std::string> status_lines;
      status_lines.reserve(last_snap.logs.size());
      for (const auto & line : last_snap.logs) {
        if (is_arm_status_debug_log(line)) {
          status_lines.push_back(line);
        }
      }
      if (!status_log_view->textCursor().hasSelection() && status_lines != rendered_status_logs) {
        refill_log_view(status_log_view, status_lines);
        rendered_status_logs = std::move(status_lines);
      }
      if (!graspnet_log_view->textCursor().hasSelection() &&
        last_snap.logs != rendered_graspnet_logs)
      {
        refill_log_view(graspnet_log_view, last_snap.logs);
        rendered_graspnet_logs = last_snap.logs;
      }
    });
  ui_timer.start(100);

  QObject::connect(&shutdown_watchdog, &QTimer::timeout, [&app]() {
      if (!rclcpp::ok()) {
        app.quit();
      }
    });
  shutdown_watchdog.start(100);

  win.show();
  const int rc = app.exec();
  return rc;
}

}  // namespace nova_grasp_moveit
