#include "ros_robot_workbench/ui/kinematics_solver_widget.h"

#include <cmath>
#include <vector>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QAbstractItemView>
#include <QColor>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QCoreApplication>
#include <QSizePolicy>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextBrowser>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "ros_robot_workbench/kinematics/arm_kinematics_kdl.hpp"
#include "ros_robot_workbench/manage/kinematics_solver_data_manager.hpp"
#include "ros_robot_workbench/module/kinematics_solver_module.h"
#include "ros_robot_workbench/workbench_build_config.hpp"

namespace ros_robot_workbench::ui
{
namespace
{
QString KinematicsAssetPath(const QString & name)
{
  // install/bin: <ws>/install/<pkg>/lib/<pkg>/exe -> ../share/<pkg>/assets/kinematics
  const QString base = QCoreApplication::applicationDirPath();
  return base + "/../../share/ros_robot_workbench/assets/kinematics/" + name;
}

std::vector<double> ParseCommaDoubles(const QString & t, int expect, bool * ok)
{
  *ok = false;
  std::vector<double> out;
  const QStringList p = t.split(QChar(','), Qt::SkipEmptyParts);
  for (const auto & s : p) {
    bool g = false;
    const double v = s.trimmed().toDouble(&g);
    if (!g) {
      return out;
    }
    out.push_back(v);
  }
  if (expect > 0 && static_cast<int>(out.size()) != expect) {
    return out;
  }
  *ok = true;
  return out;
}

QStringList CollectUrdfLinks(const QString & urdf_path)
{
  QStringList out;
  if (urdf_path.trimmed().isEmpty()) {
    return out;
  }
  urdf::Model model;
  if (!model.initFile(urdf_path.toStdString())) {
    return out;
  }
  for (const auto & kv : model.links_) {
    if (!kv.first.empty()) {
      out << QString::fromStdString(kv.first);
    }
  }
  out.sort();
  return out;
}

void UpdatePluginStatusBadge(QLabel * badge, const QString & detail, bool ok)
{
  if (!badge) {
    return;
  }
  QString level = "未探测";
  QString style = "background:#e5e7eb;color:#334155;border:1px solid #cbd5e1;border-radius:10px;padding:3px 10px;";
  const QString d = detail.toLower();
  if (!ok) {
    level = "探测失败";
    style = "background:#fee2e2;color:#991b1b;border:1px solid #fecaca;border-radius:10px;padding:3px 10px;";
  } else if (d.contains("ikfast")) {
    level = "解析/准解析 (IKFast)";
    style = "background:#dcfce7;color:#166534;border:1px solid #bbf7d0;border-radius:10px;padding:3px 10px;";
  } else if (d.contains("trac-ik") || d.contains("trac_ik")) {
    level = "数值迭代 (TRAC-IK)";
    style = "background:#fef3c7;color:#92400e;border:1px solid #fde68a;border-radius:10px;padding:3px 10px;";
  } else if (d.contains("kdl")) {
    level = "数值迭代 (KDL)";
    style = "background:#fef3c7;color:#92400e;border:1px solid #fde68a;border-radius:10px;padding:3px 10px;";
  } else if (ok) {
    level = "未知插件";
    style = "background:#e0f2fe;color:#075985;border:1px solid #bae6fd;border-radius:10px;padding:3px 10px;";
  }
  badge->setText("MoveIt IK 等级: " + level);
  badge->setStyleSheet(style);
  badge->setToolTip(detail);
}

}  // namespace

KinematicsSolverWidget::KinematicsSolverWidget(QWidget * parent)
: QWidget(parent)
, dm_()
, ros_node_(rclcpp::Node::make_shared("kinematics_assist_tools_ui", rclcpp::NodeOptions()))
, kdl_cache_()
{
  dm_.SetConfigPath("config/kinematics_solver.yaml");
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(4, 2, 4, 2);
  root->setSpacing(3);

  QLabel * title = new QLabel("运动学计算");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QTabWidget * tabs = new QTabWidget();
  tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root->addWidget(tabs, 1);

  // ----- 机械臂 -----
  QWidget * arm_page = new QWidget();
  QVBoxLayout * arm_root = new QVBoxLayout(arm_page);
  QScrollArea * arm_scroll = new QScrollArea();
  arm_scroll->setWidgetResizable(true);
  arm_scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QWidget * arm_content = new QWidget();
  QVBoxLayout * arm_l = new QVBoxLayout(arm_content);
  arm_l->setContentsMargins(1, 1, 1, 1);
  arm_l->setSpacing(3);
  QGroupBox * g_cfg = new QGroupBox("机械臂 — 解算库");
  QVBoxLayout * g_cfg_l = new QVBoxLayout(g_cfg);
  g_cfg_l->setSpacing(3);
  backend_ = new QComboBox();
  backend_->addItem("KDL（正解闭式 / 逆解为位姿级数值，Eigen+KDL）", "kdl");
#if WORKBENCH_WITH_MOVEIT
  backend_->addItem("MoveIt2（/compute_ik 等 GetPositionIK 服务，与系统插件一致）", "moveit2");
#endif
  {
    int ix = 0;
#if WORKBENCH_WITH_MOVEIT
    if (dm_.GetArmBackend() == "moveit2") {
      ix = 1;
    }
#endif
    backend_->setCurrentIndex(ix);
  }
#if WORKBENCH_WITH_MOVEIT
  QLabel * h1 = new QLabel("KDL：本地从 URDF 建链。MoveIt2：需有 move_group 等提供服务；逆解由 SRDF/IK 插件（含 IKFast 时即解析/准解析路径）。");
#else
  QLabel * h1 = new QLabel("KDL：本地从 URDF 建链，正解闭式、逆解为位姿级数值迭代。");
#endif
  h1->setWordWrap(true);
  h1->setStyleSheet("color:#445566;font-size:12px;");
  g_cfg_l->addWidget(backend_);
  g_cfg_l->addWidget(h1);
#if WORKBENCH_WITH_MOVEIT
  g_cfg_l->addWidget(new QLabel("KDL: base→tip 链；MoveIt2: 填写规划组、末端 link 与初值行（关节名: rad）。"));
#else
  g_cfg_l->addWidget(new QLabel("KDL: base→tip 链；填写 URDF 与初值后应用/重载。"));
#endif
  QFormLayout * form = new QFormLayout();
  QHBoxLayout * ur = new QHBoxLayout();
  urdf_path_ = new QLineEdit(QString::fromStdString(dm_.GetArmUrdfPath()));
  QPushButton * br = new QPushButton("浏览…");
  QObject::connect(br, &QPushButton::clicked, [this]() {
    const QString f = QFileDialog::getOpenFileName(this, "选择 URDF", QString(), "URDF (*.urdf *.xacro);;所有 (*.*)");
    if (!f.isEmpty()) {
      urdf_path_->setText(f);
      refreshUrdfLinkOptions();
    }
  });
  QObject::connect(urdf_path_, &QLineEdit::editingFinished, [this]() {
    refreshUrdfLinkOptions();
    refreshFkJointInputs();
  });
  ur->addWidget(urdf_path_, 1);
  ur->addWidget(br);
  form->addRow("URDF 路径 (KDL 必需):", ur);
  base_link_ = new QComboBox();
  base_link_->setEditable(true);
  tip_link_ = new QComboBox();
  tip_link_->setEditable(true);
  refreshUrdfLinkOptions();
  base_link_->setCurrentText(QString::fromStdString(dm_.GetArmBaseLink()));
  tip_link_->setCurrentText(QString::fromStdString(dm_.GetArmTipLink()));
  form->addRow("基座 link:", base_link_);
  form->addRow("末端 link:", tip_link_);
  QObject::connect(base_link_, &QComboBox::currentTextChanged, [this](const QString &) {
    refreshFkJointInputs();
  });
  QObject::connect(tip_link_, &QComboBox::currentTextChanged, [this](const QString &) {
    refreshFkJointInputs();
  });
  g_cfg_l->addLayout(form);

#if WORKBENCH_WITH_MOVEIT
  moveit_block_ = new QGroupBox("MoveIt2 参数");
  QFormLayout * mform = new QFormLayout(moveit_block_);
  moveit_group_edit_ = new QLineEdit(QString::fromStdString(dm_.GetMoveitGroup()));
  moveit_iklink_edit_ = new QLineEdit(QString::fromStdString(dm_.GetMoveitIkLink()));
  moveit_service_edit_ = new QLineEdit(QString::fromStdString(dm_.GetMoveitService()));
  moveit_frame_edit_ = new QLineEdit(QString::fromStdString(dm_.GetMoveitFrameId()));
  moveit_node_edit_ = new QLineEdit(QString::fromStdString(dm_.GetMoveitNodeName()));
  mform->addRow("规划组", moveit_group_edit_);
  mform->addRow("IK 链末端 link", moveit_iklink_edit_);
  mform->addRow("服务名", moveit_service_edit_);
  mform->addRow("位姿 frame_id", moveit_frame_edit_);
  mform->addRow("move_group 节点名", moveit_node_edit_);
  moveit_plugin_status_ = new QLabel("MoveIt IK 等级: 未探测");
  moveit_plugin_status_->setStyleSheet(
    "background:#e5e7eb;color:#334155;border:1px solid #cbd5e1;border-radius:10px;padding:3px 10px;");
  mform->addRow("插件能力", moveit_plugin_status_);
  seed_text_ = new QPlainTextEdit();
  seed_text_->setPlaceholderText("每行: joint_name: 0.0  （与 move_group 中关节名一致）");
  seed_text_->setPlainText(QString::fromStdString(dm_.GetMoveitSeedText()));
  mform->addRow("初值(关节) seed", seed_text_);
  g_cfg_l->addWidget(moveit_block_);

  QPushButton * probe_plugin = new QPushButton("探测 MoveIt IK 插件");
  QObject::connect(probe_plugin, &QPushButton::clicked, [this]() {
    storeFields();
    const auto r = ProbeMoveitPlugin(ros_node_, dm_);
    UpdatePluginStatusBadge(moveit_plugin_status_, r.message, r.ok);
    appendLog(r.message);
  });

  auto sync_moveit_fields = [this, probe_plugin]() {
    const bool m = (backend_->currentData().toString() == "moveit2");
    if (moveit_block_) {
      moveit_block_->setVisible(m);
    }
    if (probe_plugin) {
      probe_plugin->setVisible(m);
    }
  };
  QObject::connect(backend_, qOverload<int>(&QComboBox::currentIndexChanged), [sync_moveit_fields](int) {
    sync_moveit_fields();
  });
  g_cfg_l->addWidget(probe_plugin);
#endif

  QPushButton * apply_cfg = new QPushButton("应用 / 重载 KDL 链");
  QObject::connect(apply_cfg, &QPushButton::clicked, [this]() { applyConfiguration(); });
  QPushButton * save_cfg = new QPushButton("保存设置到 config");
  QObject::connect(save_cfg, &QPushButton::clicked, [this]() {
    storeFields();
    if (dm_.Save()) {
      appendLog("已保存 kinematics_solver.yaml");
    } else {
      appendLog("保存失败（检查工作目录可写或路径）。");
    }
  });
  {
    QHBoxLayout * cfg_btn_row = new QHBoxLayout();
    cfg_btn_row->setSpacing(6);
    cfg_btn_row->addWidget(apply_cfg, 1);
    cfg_btn_row->addWidget(save_cfg, 1);
    g_cfg_l->addLayout(cfg_btn_row);
  }
#if WORKBENCH_WITH_MOVEIT
  sync_moveit_fields();
#endif

  arm_l->addWidget(g_cfg);

  QGroupBox * g_mdh = new QGroupBox("DH / MDH 参数表");
  QVBoxLayout * g_mdh_l = new QVBoxLayout(g_mdh);
  {
    QHBoxLayout * mode_row = new QHBoxLayout();
    mode_row->addWidget(new QLabel("参数约定:"));
    dh_mode_combo_ = new QComboBox();
    dh_mode_combo_->addItem("MDH (Craig 1989)", "mdh");
    dh_mode_combo_->addItem("DH (Denavit-Hartenberg 1955)", "dh");
    mode_row->addWidget(dh_mode_combo_, 1);
    mode_row->addWidget(new QLabel("角度单位:"));
    mdh_angle_unit_combo_ = new QComboBox();
    mdh_angle_unit_combo_->addItem("rad", "rad");
    mdh_angle_unit_combo_->addItem("deg", "deg");
    mdh_angle_unit_combo_->setCurrentIndex(0);
    mode_row->addWidget(mdh_angle_unit_combo_);
    mode_row->addWidget(new QLabel("长度单位:"));
    mdh_length_unit_combo_ = new QComboBox();
    mdh_length_unit_combo_->addItem("m", "m");
    mdh_length_unit_combo_->addItem("mm", "mm");
    mdh_length_unit_combo_->setCurrentIndex(0);
    mode_row->addWidget(mdh_length_unit_combo_);
    g_mdh_l->addLayout(mode_row);
    QObject::connect(
      dh_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
      [this](int) { refreshMdhTable(); });
    QObject::connect(
      mdh_angle_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
      [this](int) { refreshMdhTable(); });
    QObject::connect(
      mdh_length_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
      [this](int) { refreshMdhTable(); });
  }
  {
    auto * mdh_hint = new QLabel(
      "每行对应上一处活动关节。可在 DH(1955) 与 MDH(Craig 1989) 之间切换。"
      " 角度列与长度列的「关节变量」以橙色标注。");
    mdh_hint->setWordWrap(true);
    mdh_hint->setStyleSheet("color:#64748b;font-size:12px;");
    g_mdh_l->addWidget(mdh_hint);
  }
  mdh_table_ = new QTableWidget(0, 5);
  mdh_table_->setHorizontalHeaderLabels(
    {QString::fromUtf8("i"), QString::fromUtf8("α_{i-1} (rad)"), QString::fromUtf8("a_{i-1} (m)"),
     QString::fromUtf8("d_i (m)"), QString::fromUtf8("θ_i / 变量 (rad)")});
  mdh_table_->verticalHeader()->setVisible(false);
  mdh_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  mdh_table_->setSelectionMode(QAbstractItemView::NoSelection);
  mdh_table_->setMinimumHeight(76);
  mdh_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  mdh_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  mdh_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  mdh_table_->horizontalHeader()->setMinimumSectionSize(60);
  g_mdh_l->addWidget(mdh_table_);
  arm_l->addWidget(g_mdh);

  QGroupBox * g_io = new QGroupBox("位姿与关节 (弧度)");
  QVBoxLayout * g_io_l = new QVBoxLayout(g_io);
  g_io_l->setSpacing(3);
  QTabWidget * io_tabs = new QTabWidget();
  QWidget * fk_tab = new QWidget();
  QVBoxLayout * fk_l = new QVBoxLayout(fk_tab);
  fk_l->setContentsMargins(4, 4, 4, 4);
  fk_l->setSpacing(3);
  fk_l->addWidget(
    new QLabel("关节角 q (rad)：数量由 base_link→末端 link 的链自动确定，加载链后显示。"));
  fk_joints_form_ = new QFormLayout();
  fk_joints_form_->setSpacing(3);
  fk_joints_form_->setFormAlignment(Qt::AlignTop);
  fk_l->addLayout(fk_joints_form_);
  QPushButton * fk = new QPushButton("正解 (FK)");
  fk->setMinimumHeight(26);
  fk_l->addWidget(fk);
  fk_l->addStretch();
  io_tabs->addTab(fk_tab, "正解 (FK)");

  QWidget * ik_tab = new QWidget();
  QVBoxLayout * ik_l = new QVBoxLayout(ik_tab);
  ik_l->setContentsMargins(4, 4, 4, 4);
  ik_l->setSpacing(3);
  seed_line_ = new QLineEdit("0,0,0,0,0,0");
  seed_line_->setPlaceholderText("逆解初值(逗号，与 KDL 关节数一致)");
  ik_l->addWidget(new QLabel("KDL 逆解初值"));
  ik_l->addWidget(seed_line_);
  QHBoxLayout * p_row = new QHBoxLayout();
  px_ = new QDoubleSpinBox();
  py_ = new QDoubleSpinBox();
  pz_ = new QDoubleSpinBox();
  for (auto * b : {px_, py_, pz_}) {
    b->setRange(-20, 20);
    b->setDecimals(5);
    b->setMinimumHeight(22);
  }
  p_row->addWidget(new QLabel("px"));
  p_row->addWidget(px_);
  p_row->addWidget(new QLabel("py"));
  p_row->addWidget(py_);
  p_row->addWidget(new QLabel("pz"));
  p_row->addWidget(pz_);
  ik_l->addLayout(p_row);
  QHBoxLayout * q_row = new QHBoxLayout();
  qx_ = new QDoubleSpinBox();
  qy_ = new QDoubleSpinBox();
  qz_ = new QDoubleSpinBox();
  qw_ = new QDoubleSpinBox();
  for (auto * b : {qx_, qy_, qz_, qw_}) {
    b->setRange(-1, 1);
    b->setDecimals(5);
    b->setMinimumHeight(22);
  }
  qw_->setValue(1.0);
  q_row->addWidget(new QLabel("qx"));
  q_row->addWidget(qx_);
  q_row->addWidget(new QLabel("qy"));
  q_row->addWidget(qy_);
  q_row->addWidget(new QLabel("qz"));
  q_row->addWidget(qz_);
  q_row->addWidget(new QLabel("qw"));
  q_row->addWidget(qw_);
  ik_l->addLayout(q_row);
  QPushButton * ik = new QPushButton("逆解 (IK)");
  ik->setMinimumHeight(26);
  ik_l->addWidget(ik);
  ik_l->addStretch();
  io_tabs->addTab(ik_tab, "逆解 (IK)");

  g_io_l->addWidget(io_tabs);
  arm_l->addWidget(g_io);

  refreshFkJointInputs();

  QObject::connect(fk, &QPushButton::clicked, [this]() { runFk(); });
  QObject::connect(ik, &QPushButton::clicked, [this]() { runIk(); });

  arm_scroll->setWidget(arm_content);
  arm_root->addWidget(arm_scroll, 1);
  tabs->addTab(arm_page, "机械臂");

  // ----- 差速 -----
  QWidget * dpg = new QWidget();
  QVBoxLayout * dvl = new QVBoxLayout(dpg);
  dvl->setContentsMargins(3, 2, 3, 2);
  dvl->setSpacing(2);
  dvl->addWidget(new QLabel("轮距(轴间距) L、轮半径 r、左右轮角速度(rad/s)。标准差速模型；假定左右轮心对称。"));
  QHBoxLayout * dform = new QHBoxLayout();
  diff_L_ = new QDoubleSpinBox();
  diff_r_ = new QDoubleSpinBox();
  for (QDoubleSpinBox * b : {diff_L_, diff_r_}) {
    b->setRange(0.0001, 10.0);
    b->setDecimals(4);
  }
  diff_L_->setValue(dm_.GetDiffTrackM());
  diff_r_->setValue(dm_.GetDiffWheelRadiusM());
  dform->addWidget(new QLabel("L (m):"));
  dform->addWidget(diff_L_, 1);
  dform->addWidget(new QLabel("r (m):"));
  dform->addWidget(diff_r_, 1);
  dvl->addLayout(dform);
  QGroupBox * w2b = new QGroupBox("轮速 → 车体");
  QHBoxLayout * w2b_l = new QHBoxLayout(w2b);
  QHBoxLayout * wlr = new QHBoxLayout();
  diff_wl_ = new QDoubleSpinBox();
  diff_wr_ = new QDoubleSpinBox();
  for (QDoubleSpinBox * b : {diff_wl_, diff_wr_}) {
    b->setRange(-200, 200);
    b->setDecimals(4);
  }
  wlr->addWidget(new QLabel("ω_l (rad/s)"));
  wlr->addWidget(diff_wl_);
  wlr->addWidget(new QLabel("ω_r (rad/s)"));
  wlr->addWidget(diff_wr_);
  w2b_l->addLayout(wlr, 1);
  QPushButton * run_w2b = new QPushButton("计算 v, ω");
  w2b_l->addWidget(run_w2b, 1);
  dvl->addWidget(w2b);
  QGroupBox * b2w = new QGroupBox("车体 → 轮速");
  QHBoxLayout * b2w_l = new QHBoxLayout(b2w);
  QHBoxLayout * vb = new QHBoxLayout();
  diff_v_ = new QDoubleSpinBox();
  diff_omega_ = new QDoubleSpinBox();
  diff_v_->setRange(-10, 10);
  diff_omega_->setRange(-20, 20);
  for (QDoubleSpinBox * b : {diff_v_, diff_omega_}) {
    b->setDecimals(5);
  }
  vb->addWidget(new QLabel("v (m/s)"));
  vb->addWidget(diff_v_);
  vb->addWidget(new QLabel("ω (rad/s)"));
  vb->addWidget(diff_omega_);
  b2w_l->addLayout(vb, 1);
  QPushButton * run_b2w = new QPushButton("计算 ω_l, ω_r");
  b2w_l->addWidget(run_b2w, 1);
  dvl->addWidget(b2w);
  QGroupBox * diff_formula_group = new QGroupBox("差速模型正逆解");
  QVBoxLayout * diff_formula_l = new QVBoxLayout(diff_formula_group);
  QTextBrowser * diff_formula_text = new QTextBrowser();
  diff_formula_text->setOpenExternalLinks(true);
  diff_formula_text->setMinimumHeight(96);
  diff_formula_text->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  diff_formula_text->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  diff_formula_text->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  diff_formula_text->document()->setDocumentMargin(2);
  diff_formula_text->setHtml(
    QString("<div style='font-family:Arial,sans-serif; font-size:12px; margin:0; padding:0;'>"
            "<div><b>正逆解（矩阵形式）</b></div>"
            "<div><img src='file://%1' width='100%%' style='display:block;'></div>"
            "</div>")
      .arg(KinematicsAssetPath("diff_formula_local.png")));
  diff_formula_l->addWidget(diff_formula_text, 1);
  dvl->addWidget(diff_formula_group, 0);
  dvl->addStretch();
  tabs->addTab(dpg, "差速");
  QObject::connect(run_w2b, &QPushButton::clicked, [this]() {
    storeDiff();
    const auto a = RunDiffFromWheels(dm_, diff_wl_->value(), diff_wr_->value());
    appendLog(a.ok ? a.message : ("差速: " + a.message));
  });
  QObject::connect(run_b2w, &QPushButton::clicked, [this]() {
    storeDiff();
    const auto a = RunDiffFromBody(dm_, diff_v_->value(), diff_omega_->value());
    appendLog(a.ok ? a.message : ("差速: " + a.message));
  });

  // ----- 阿克曼 -----
  QWidget * apg = new QWidget();
  QVBoxLayout * avl = new QVBoxLayout(apg);
  avl->setContentsMargins(3, 2, 3, 2);
  avl->setSpacing(2);
  avl->addWidget(new QLabel("支持两种模型：自行车等效（简化）和理想阿克曼（双轨几何约束）。"));
  ack_model_ = new QComboBox();
  ack_model_->addItem("自行车等效模型", "bike");
  ack_model_->addItem("理想阿克曼（由内轮角）", "ideal_inner");
  ack_model_->addItem("理想阿克曼（由内外轮角）", "ideal_pair");
  QFormLayout * model_form = new QFormLayout();
  model_form->addRow("模型", ack_model_);
  avl->addLayout(model_form);
  ack_L_ = new QDoubleSpinBox();
  ack_L_->setRange(0.0001, 20);
  ack_L_->setDecimals(4);
  ack_L_->setValue(dm_.GetAckWheelbaseM());
  ack_W_ = new QDoubleSpinBox();
  ack_W_->setRange(0.0001, 20);
  ack_W_->setDecimals(4);
  ack_W_->setValue(dm_.GetDiffTrackM());
  ack_delta_ = new QDoubleSpinBox();
  ack_delta_->setRange(-89, 89);
  ack_delta_->setDecimals(2);
  ack_delta_in_ = new QDoubleSpinBox();
  ack_delta_in_->setRange(-89, 89);
  ack_delta_in_->setDecimals(2);
  ack_delta_in_->setValue(15.0);
  ack_delta_out_ = new QDoubleSpinBox();
  ack_delta_out_->setRange(-89, 89);
  ack_delta_out_->setDecimals(2);
  ack_delta_out_->setValue(12.0);
  ack_k_target_ = new QDoubleSpinBox();
  ack_k_target_->setRange(-10.0, 10.0);
  ack_k_target_->setDecimals(5);
  ack_k_target_->setValue(0.2);
  ack_v_ref_fwd_ = new QDoubleSpinBox();
  ack_v_ref_fwd_->setRange(-20.0, 20.0);
  ack_v_ref_fwd_->setDecimals(4);
  ack_v_ref_fwd_->setValue(2.0);
  ack_v_ref_inv_ = new QDoubleSpinBox();
  ack_v_ref_inv_->setRange(-20.0, 20.0);
  ack_v_ref_inv_->setDecimals(4);
  ack_v_ref_inv_->setValue(2.0);
  ack_omega_target_ = new QDoubleSpinBox();
  ack_omega_target_->setRange(-20.0, 20.0);
  ack_omega_target_->setDecimals(5);
  ack_omega_target_->setValue(0.2);
  QGroupBox * ack_forward_group = new QGroupBox("正解参数（由转角/几何量求 k、R）");
  QGroupBox * ack_inverse_group = new QGroupBox("逆解参数（由目标运动量求转角）");
  QFormLayout * fwd_form = new QFormLayout(ack_forward_group);
  QFormLayout * inv_form = new QFormLayout(ack_inverse_group);
  auto * lab_L = new QLabel("轴距 L (m)");
  auto * lab_W = new QLabel("轮距 W (m)");
  auto * lab_delta = new QLabel("等效前轮角 δ (deg)");
  auto * lab_din = new QLabel("内轮角 δ_in (deg)");
  auto * lab_dout = new QLabel("外轮角 δ_out (deg)");
  auto * lab_k = new QLabel("目标曲率 k (1/m)");
  auto * lab_vf = new QLabel("参考点线速度 v_ref (m/s)");
  auto * lab_vi = new QLabel("参考点线速度 v_ref (m/s)");
  auto * lab_om = new QLabel("目标角速度 ω (rad/s)");
  fwd_form->addRow(lab_L, ack_L_);
  fwd_form->addRow(lab_W, ack_W_);
  fwd_form->addRow(lab_delta, ack_delta_);
  fwd_form->addRow(lab_din, ack_delta_in_);
  fwd_form->addRow(lab_dout, ack_delta_out_);
  fwd_form->addRow(lab_vf, ack_v_ref_fwd_);
  inv_form->addRow(lab_k, ack_k_target_);
  inv_form->addRow(lab_vi, ack_v_ref_inv_);
  inv_form->addRow(lab_om, ack_omega_target_);
  QHBoxLayout * ack_param_row = new QHBoxLayout();
  ack_param_row->addWidget(ack_forward_group, 1);
  ack_param_row->addWidget(ack_inverse_group, 1);
  avl->addLayout(ack_param_row);
  auto sync_ack_visibility = [this, lab_W, lab_delta, lab_din, lab_dout, lab_k, lab_vf, lab_vi, lab_om, ack_inverse_group]() {
    if (!ack_model_ || !ack_delta_ || !ack_delta_in_ || !ack_delta_out_ || !ack_W_) {
      return;
    }
    const QString m = ack_model_->currentData().toString();
    const bool bike = (m == "bike");
    const bool ideal_inner = (m == "ideal_inner");
    const bool ideal_pair = (m == "ideal_pair");
    ack_delta_->setVisible(bike);
    lab_delta->setVisible(bike);
    ack_v_ref_fwd_->setVisible(bike);
    lab_vf->setVisible(bike);
    ack_k_target_->setVisible(bike);
    lab_k->setVisible(bike);
    ack_v_ref_inv_->setVisible(bike);
    lab_vi->setVisible(bike);
    ack_omega_target_->setVisible(bike);
    lab_om->setVisible(bike);
    ack_W_->setVisible(!bike);
    lab_W->setVisible(!bike);
    ack_delta_in_->setVisible(!bike);
    lab_din->setVisible(!bike);
    ack_delta_out_->setVisible(ideal_pair);
    lab_dout->setVisible(ideal_pair);
    // 逆解当前仅在自行车模型提供：由目标曲率反解等效前轮角
    ack_inverse_group->setVisible(bike);
    (void)ideal_inner;
  };
  QGroupBox * ack_formula_group = new QGroupBox("阿克曼模型正逆解");
  QVBoxLayout * ack_formula_l = new QVBoxLayout(ack_formula_group);
  QTextBrowser * ack_formula_text = new QTextBrowser();
  ack_formula_text->setOpenExternalLinks(true);
  ack_formula_text->setMinimumHeight(96);
  ack_formula_text->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  ack_formula_text->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  ack_formula_text->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  ack_formula_text->document()->setDocumentMargin(2);
  ack_formula_l->addWidget(ack_formula_text, 1);
  auto update_ack_formula = [this, ack_formula_text]() {
    if (!ack_formula_text || !ack_model_) {
      return;
    }
    const QString m = ack_model_->currentData().toString();
    if (m == "bike") {
      ack_formula_text->setHtml(
        QString("<div style='font-family:Arial,sans-serif; font-size:12px; margin:0; padding:0;'>"
                "<div><b>自行车模型（矩阵/公式）</b></div>"
                "<div><img src='file://%1' width='100%%' style='display:block;'></div>"
                "</div>")
          .arg(KinematicsAssetPath("ack_bike_formula_local.png")));
      return;
    }
    if (m == "ideal_inner") {
      ack_formula_text->setHtml(
        QString("<div style='font-family:Arial,sans-serif; font-size:12px; margin:0; padding:0;'>"
                "<div><b>理想阿克曼（内轮角）</b></div>"
                "<div><img src='file://%1' width='100%%' style='display:block;'></div>"
                "</div>")
          .arg(KinematicsAssetPath("ack_inner_formula_local.png")));
      return;
    }
    ack_formula_text->setHtml(
      QString("<div style='font-family:Arial,sans-serif; font-size:12px; margin:0; padding:0;'>"
              "<div><b>理想阿克曼（内外轮角）</b></div>"
              "<div><img src='file://%1' width='100%%' style='display:block;'></div>"
              "</div>")
        .arg(KinematicsAssetPath("ack_pair_formula_local.png")));
  };
  QObject::connect(ack_model_, qOverload<int>(&QComboBox::currentIndexChanged), [sync_ack_visibility, update_ack_formula](int) {
    sync_ack_visibility();
    update_ack_formula();
  });
  QHBoxLayout * ack_btn_row = new QHBoxLayout();
  QPushButton * run_ack = new QPushButton("正解: 由转角算 k/R/ω");
  QPushButton * run_ack_inv = new QPushButton("逆解: 由 v_ref 和 ω 算转角(自行车)");
  ack_btn_row->addWidget(run_ack, 1);
  ack_btn_row->addWidget(run_ack_inv, 1);
  avl->addLayout(ack_btn_row);
  avl->addWidget(ack_formula_group, 0);
  avl->addStretch();
  tabs->addTab(apg, "阿克曼");
  QObject::connect(
    run_ack, &QPushButton::clicked, [this]() { storeAck();
      const QString m = ack_model_ ? ack_model_->currentData().toString() : "bike";
      KinematicsSolveResult a;
      if (m == "bike") {
        a = RunAckForwardBikeVelocity(dm_, ack_delta_->value(), ack_v_ref_fwd_->value());
      } else if (m == "ideal_inner") {
        a = RunAckIdealFromInner(dm_, ack_W_->value(), ack_delta_in_->value());
      } else {
        a = RunAckIdealFromPair(dm_, ack_W_->value(), ack_delta_in_->value(), ack_delta_out_->value());
      }
      appendLog(a.ok ? a.message : ("阿克曼: " + a.message));
  });
  QObject::connect(
    run_ack_inv, &QPushButton::clicked, [this]() { storeAck();
      const QString m = ack_model_ ? ack_model_->currentData().toString() : "bike";
      if (m != "bike") {
        appendLog("阿克曼逆解目前针对‘自行车等效模型’，请切换模型后使用。");
        return;
      }
      const auto a = RunAckInverseBikeVelocity(dm_, ack_v_ref_inv_->value(), ack_omega_target_->value());
      appendLog(a.ok ? a.message : ("阿克曼: " + a.message));
  });
  sync_ack_visibility();
  update_ack_formula();

  QGroupBox * log_group = new QGroupBox("输出");
  QVBoxLayout * log_layout = new QVBoxLayout(log_group);
  log_ = new QTextEdit();
  log_->setReadOnly(true);
  log_->setMinimumHeight(60);
  log_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  log_->setText("KDL: 先选择 URDF 与 base/tip 后点「应用」。\n"
                 "MoveIt2: 先启动带 /compute_ik 的 move_group。\n"
                 "差速/阿克曼 参数在各自 Tab 中计算。");
  log_layout->addWidget(log_, 1);
  root->addWidget(log_group, 0);
}

void KinematicsSolverWidget::appendLog(const QString & s)
{
  if (!log_) {
    return;
  }
  log_->append(s);
  log_->append("");
}

void KinematicsSolverWidget::storeFields()
{
  const QString b = backend_->currentData().toString();
  dm_.SetArmBackend(b.toStdString());
  dm_.SetArmUrdfPath(urdf_path_->text().toStdString());
  dm_.SetArmBaseLink(base_link_->currentText().toStdString());
  dm_.SetArmTipLink(tip_link_->currentText().toStdString());
  if (moveit_group_edit_) {
    dm_.SetMoveitGroup(moveit_group_edit_->text().toStdString());
  }
  if (moveit_iklink_edit_) {
    dm_.SetMoveitIkLink(moveit_iklink_edit_->text().toStdString());
  }
  if (moveit_service_edit_) {
    dm_.SetMoveitService(moveit_service_edit_->text().toStdString());
  }
  if (moveit_frame_edit_) {
    dm_.SetMoveitFrameId(moveit_frame_edit_->text().toStdString());
  }
  if (moveit_node_edit_) {
    dm_.SetMoveitNodeName(moveit_node_edit_->text().toStdString());
  }
  if (seed_text_) {
    dm_.SetMoveitSeedText(seed_text_->toPlainText().toStdString());
  }
}

void KinematicsSolverWidget::storeDiff()
{
  dm_.SetDiffTrackM(diff_L_->value());
  dm_.SetDiffWheelRadiusM(diff_r_->value());
}

void KinematicsSolverWidget::storeAck()
{
  dm_.SetAckWheelbaseM(ack_L_->value());
}

void KinematicsSolverWidget::applyConfiguration()
{
  storeFields();
  const auto a = KdlLoadArm(
    kdl_cache_, dm_.GetArmUrdfPath(), dm_.GetArmBaseLink(), dm_.GetArmTipLink());
  appendLog(a.message);
  if (a.ok) {
    refreshFkJointInputs();
  }
}

void KinematicsSolverWidget::runFk()
{
  const QString b = backend_->currentData().toString();
  if (b != "kdl") {
    appendLog("正解 KDL: 请切换到 KDL 后端后使用本按钮（MoveIt2 可另用 /fk 或本工具中未来扩展）。");
    return;
  }
  dm_.SetArmUrdfPath(urdf_path_->text().toStdString());
  dm_.SetArmBaseLink(base_link_->currentText().toStdString());
  dm_.SetArmTipLink(tip_link_->currentText().toStdString());
  const bool ok_load = KdlLoadArm(
    kdl_cache_, dm_.GetArmUrdfPath(), dm_.GetArmBaseLink(), dm_.GetArmTipLink()).ok;
  if (!ok_load) {
    applyConfiguration();
  } else {
    refreshFkJointInputs();
  }
  int nj = 0;
  if (kdl_cache_.kdl) {
    nj = static_cast<int>(kdl_cache_.kdl->GetJointCount());
  }
  if (nj < 1 || static_cast<int>(fk_joints_spins_.size()) != nj) {
    appendLog("正解: 请先在未报错情况下点「应用/重载 KDL 链」，以生成本链关节输入。");
    return;
  }
  std::vector<double> qv;
  qv.reserve(static_cast<size_t>(nj));
  for (QDoubleSpinBox * sp : fk_joints_spins_) {
    qv.push_back(sp->value());
  }
  const auto r = KdlForward(kdl_cache_, qv);
  appendLog(r.message);
}

void KinematicsSolverWidget::runIk()
{
  std::array<double, 3> p{{px_->value(), py_->value(), pz_->value()}};
  std::array<double, 4> qv{{qx_->value(), qy_->value(), qz_->value(), qw_->value()}};

  const QString b = backend_->currentData().toString();
  if (b == "kdl") {
    dm_.SetArmUrdfPath(urdf_path_->text().toStdString());
    dm_.SetArmBaseLink(base_link_->currentText().toStdString());
    dm_.SetArmTipLink(tip_link_->currentText().toStdString());
    if (!KdlLoadArm(
      kdl_cache_, dm_.GetArmUrdfPath(), dm_.GetArmBaseLink(), dm_.GetArmTipLink()).ok) {
      applyConfiguration();
    } else {
      refreshFkJointInputs();
    }
    int nj = 0;
    if (kdl_cache_.kdl) {
      nj = static_cast<int>(kdl_cache_.kdl->GetJointCount());
    }
    bool ok = false;
    auto seed = ParseCommaDoubles(seed_line_->text(), nj, &ok);
    if (!ok || (nj > 0 && static_cast<int>(seed.size()) != nj)) {
      appendLog(
        QString("初值需 %1 个逗号分隔数。当前: %2")
          .arg(nj)
          .arg(seed_line_->text()));
      return;
    }
    if (static_cast<int>(seed.size()) != nj || nj < 1) {
      appendLog("请先成功加载 KDL 链与初值。");
      return;
    }
    const auto r = KdlInverse(kdl_cache_, seed, p, qv);
    appendLog(r.message);
    return;
  }

#if WORKBENCH_WITH_MOVEIT
  if (b == "moveit2") {
    storeFields();
    const auto probe = ProbeMoveitPlugin(ros_node_, dm_);
    UpdatePluginStatusBadge(moveit_plugin_status_, probe.message, probe.ok);
    appendLog("自动探测插件: " + probe.message);
    const auto r = MoveitInverse(ros_node_, dm_, p, qv);
    appendLog(r.message);
    return;
  }
#endif

  appendLog("未知后端: " + b);
}

void KinematicsSolverWidget::refreshUrdfLinkOptions()
{
  if (!base_link_ || !tip_link_ || !urdf_path_) {
    return;
  }
  const QString prev_base = base_link_->currentText();
  const QString prev_tip = tip_link_->currentText();
  const QStringList links = CollectUrdfLinks(urdf_path_->text());
  if (links.isEmpty()) {
    return;
  }
  base_link_->blockSignals(true);
  tip_link_->blockSignals(true);
  base_link_->clear();
  tip_link_->clear();
  base_link_->addItems(links);
  tip_link_->addItems(links);
  if (!prev_base.isEmpty()) {
    base_link_->setCurrentText(prev_base);
  }
  if (!prev_tip.isEmpty()) {
    tip_link_->setCurrentText(prev_tip);
  }
  base_link_->blockSignals(false);
  tip_link_->blockSignals(false);
  refreshFkJointInputs();
}

void KinematicsSolverWidget::refreshFkJointInputs()
{
  if (!fk_joints_form_ || !urdf_path_ || !base_link_ || !tip_link_) {
    refreshMdhTable();
    return;
  }
  std::vector<double> saved;
  for (QDoubleSpinBox * sp : fk_joints_spins_) {
    if (sp) {
      saved.push_back(sp->value());
    }
  }
  while (fk_joints_form_->rowCount() > 0) {
    fk_joints_form_->removeRow(0);
  }
  fk_joints_spins_.clear();

  const std::string key =
    urdf_path_->text().toStdString() + "|" + base_link_->currentText().toStdString() + "|" +
    tip_link_->currentText().toStdString();
  if (!kdl_cache_.kdl || kdl_cache_.key != key) {
    auto * hint = new QLabel("请先点「应用 / 重载 KDL 链」以按 base/tip 生成各关节输入框");
    hint->setWordWrap(true);
    fk_joints_form_->addRow(hint);
    refreshMdhTable();
    return;
  }

  const int nj = static_cast<int>(kdl_cache_.kdl->GetJointCount());
  if (nj < 1) {
    fk_joints_form_->addRow(new QLabel("链上无活动关节。"));
    refreshMdhTable();
    return;
  }

  fk_joints_spins_.reserve(static_cast<size_t>(nj));
  constexpr int kPerRow = 3;
  QHBoxLayout * row_layout = nullptr;
  int row_used = 0;
  for (int i = 0; i < nj; ++i) {
    if (i % kPerRow == 0) {
      row_layout = new QHBoxLayout();
      row_layout->setSpacing(6);
      fk_joints_form_->addRow(row_layout);
      row_used = 0;
    }
    auto * sp = new QDoubleSpinBox();
    sp->setDecimals(5);
    sp->setMinimumHeight(22);
    sp->setSingleStep(0.01);
    double lo = -10.0;
    double hi = 10.0;
    if (kdl_cache_.kdl->GetJointLimits(static_cast<size_t>(i), lo, hi) && std::isfinite(lo) &&
      std::isfinite(hi) && lo < hi) {
      sp->setRange(lo, hi);
    } else {
      sp->setRange(-20.0, 20.0);
    }
    double v = 0.0;
    if (static_cast<size_t>(i) < saved.size()) {
      v = saved[static_cast<size_t>(i)];
    }
    if (v < sp->minimum() || v > sp->maximum()) {
      v = 0.0;
    }
    sp->setValue(v);
    const QString jn = QString::fromStdString(kdl_cache_.kdl->GetJointName(static_cast<size_t>(i)));
    auto * lb = new QLabel(jn.isEmpty() ? QString("关节 %1 (rad)").arg(i) : (jn + " (rad)"));
    lb->setMinimumWidth(88);
    fk_joints_spins_.push_back(sp);
    if (row_layout) {
      auto * cell = new QWidget();
      auto * cell_l = new QHBoxLayout(cell);
      cell_l->setContentsMargins(0, 0, 0, 0);
      cell_l->setSpacing(4);
      cell_l->addWidget(lb);
      cell_l->addWidget(sp, 1);
      row_layout->addWidget(cell, 1);
      ++row_used;
    }
  }
  if (row_layout && row_used > 0 && row_used < kPerRow) {
    for (int pad = row_used; pad < kPerRow; ++pad) {
      auto * placeholder = new QWidget();
      placeholder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      row_layout->addWidget(placeholder, 1);
    }
  }
  refreshMdhTable();
}

void KinematicsSolverWidget::refreshMdhTable()
{
  if (!mdh_table_ || !urdf_path_ || !base_link_ || !tip_link_) {
    return;
  }
  const bool ang_deg = mdh_angle_unit_combo_ && mdh_angle_unit_combo_->currentData().toString() == "deg";
  const bool len_mm = mdh_length_unit_combo_ && mdh_length_unit_combo_->currentData().toString() == "mm";
  const QString au = ang_deg ? QStringLiteral("deg") : QStringLiteral("rad");
  const QString lu = len_mm ? QStringLiteral("mm") : QStringLiteral("m");
  mdh_table_->setHorizontalHeaderLabels({
    QString::fromUtf8("i"),
    QString::fromUtf8("α_{i-1} (%1)").arg(au),
    QString::fromUtf8("a_{i-1} (%1)").arg(lu),
    QString::fromUtf8("d_i (%1)").arg(lu),
    QString::fromUtf8("θ_i / 变量 (%1)").arg(au),
  });

  mdh_table_->clearContents();
  mdh_table_->clearSpans();
  mdh_table_->setRowCount(0);
  const std::string key = urdf_path_->text().toStdString() + "|" + base_link_->currentText().toStdString() +
    "|" + tip_link_->currentText().toStdString();
  const auto fmt_len = [len_mm](double m_m) {
    const double v = len_mm ? m_m * 1000.0 : m_m;
    return QString::number(v, 'g', 6);
  };
  const auto fmt_ang = [ang_deg](double rad) {
    const double v = ang_deg ? rad * (180.0 / M_PI) : rad;
    return QString::number(v, 'g', 6);
  };
  if (!kdl_cache_.kdl || kdl_cache_.key != key) {
    return;
  }
  std::vector<kinematics::MdhCraig1989TableRow> rows;
  std::string emsg;
  kinematics::DhConvention mode = kinematics::DhConvention::ModifiedCraig1989;
  if (dh_mode_combo_ && dh_mode_combo_->currentData().toString() == "dh") {
    mode = kinematics::DhConvention::Standard1955;
  }
  if (!kdl_cache_.kdl->GetMdhCraig1989Table(rows, emsg, mode)) {
    mdh_table_->setRowCount(1);
    mdh_table_->setColumnCount(5);
    auto * err_it = new QTableWidgetItem(QString::fromStdString(emsg));
    err_it->setTextAlignment(Qt::AlignCenter);
    err_it->setForeground(QBrush(QColor("#b91c1c")));
    mdh_table_->setItem(0, 0, err_it);
    mdh_table_->setSpan(0, 0, 1, 5);
    return;
  }
  mdh_table_->setColumnCount(5);
  const QColor black(30, 41, 59);
  const QColor orange(194, 65, 12);
  mdh_table_->setRowCount(static_cast<int>(rows.size()));
  int fallback_rows = 0;
  for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
    const auto & row = rows[static_cast<size_t>(r)];
    const QString jlabel = QString::fromStdString(row.joint_name);
    auto * i_i = new QTableWidgetItem(QString::number(row.i_1based));
    i_i->setTextAlignment(Qt::AlignCenter);
    i_i->setForeground(QBrush(black));
    i_i->setToolTip(jlabel);
    mdh_table_->setItem(r, 0, i_i);
    if (!row.fit_ok) {
      ++fallback_rows;
      auto * fb = new QTableWidgetItem(QString::fromUtf8("FB(xyz/rpy)"));
      fb->setTextAlignment(Qt::AlignCenter);
      fb->setForeground(QBrush(orange));
      if (!row.fit_error.empty()) {
        fb->setToolTip(QString::fromStdString(row.fit_error));
      }
      auto * xyz = new QTableWidgetItem(
        QString::fromUtf8("x=%1, y=%2, z=%3")
          .arg(fmt_len(row.fallback_x), fmt_len(row.fallback_y), fmt_len(row.fallback_z)));
      xyz->setTextAlignment(Qt::AlignCenter);
      xyz->setForeground(QBrush(black));
      auto * rpy = new QTableWidgetItem(
        QString::fromUtf8("r=%1, p=%2, y=%3")
          .arg(fmt_ang(row.fallback_roll), fmt_ang(row.fallback_pitch), fmt_ang(row.fallback_yaw)));
      rpy->setTextAlignment(Qt::AlignCenter);
      rpy->setForeground(QBrush(black));
      const bool pris = (row.kind == kinematics::MdhCraig1989TableRow::JointKind::Prismatic);
      const bool revo = (row.kind == kinematics::MdhCraig1989TableRow::JointKind::Revolute);
      QString kind_text = QString::fromUtf8("固定/其他关节");
      if (revo) {
        kind_text = QString::fromUtf8("旋转关节: θ=θ0+q");
      } else if (pris) {
        kind_text = QString::fromUtf8("移动关节: d=d0+q");
      }
      auto * kitem = new QTableWidgetItem(kind_text);
      kitem->setTextAlignment(Qt::AlignCenter);
      kitem->setForeground(QBrush(pris || revo ? orange : black));
      mdh_table_->setItem(r, 1, fb);
      mdh_table_->setItem(r, 2, xyz);
      mdh_table_->setItem(r, 3, rpy);
      mdh_table_->setItem(r, 4, kitem);
      continue;
    }
    auto * i_al = new QTableWidgetItem(fmt_ang(row.alpha_im1));
    i_al->setTextAlignment(Qt::AlignCenter);
    i_al->setForeground(QBrush(black));
    auto * i_a = new QTableWidgetItem(fmt_len(row.a_im1));
    i_a->setTextAlignment(Qt::AlignCenter);
    i_a->setForeground(QBrush(black));
    const bool pris = (row.kind == kinematics::MdhCraig1989TableRow::JointKind::Prismatic);
    const bool revo = (row.kind == kinematics::MdhCraig1989TableRow::JointKind::Revolute);
    QString d_cell = fmt_len(row.d_i);
    QString t_cell = fmt_ang(row.theta_offset);
    if (pris) {
      d_cell = QString::fromUtf8("d0=%1  +q (%2)").arg(fmt_len(row.d_i), lu);
    }
    if (revo) {
      const QString jt = jlabel.isEmpty() ? QString::fromUtf8("关节") : jlabel;
      t_cell = QString::fromUtf8("θ0=%1  +q [%2] (%3)").arg(fmt_ang(row.theta_offset), jt, au);
    }
    auto * i_d = new QTableWidgetItem(d_cell);
    i_d->setTextAlignment(Qt::AlignCenter);
    i_d->setForeground(QBrush(pris ? orange : black));
    auto * i_th = new QTableWidgetItem(t_cell);
    i_th->setTextAlignment(Qt::AlignCenter);
    i_th->setForeground(QBrush(revo ? orange : black));
    mdh_table_->setItem(r, 1, i_al);
    mdh_table_->setItem(r, 2, i_a);
    mdh_table_->setItem(r, 3, i_d);
    mdh_table_->setItem(r, 4, i_th);
  }
  if (fallback_rows > 0) {
    appendLog(
      QString::fromUtf8("MDH 拟合失败 %1 行，已自动回退显示 xyz/rpy。").arg(fallback_rows));
  } else {
    appendLog(QString::fromUtf8("MDH 拟合全部成功。"));
  }
}

}  // namespace ros_robot_workbench::ui
