#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"

#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <algorithm>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace hs_calib {
namespace gui {

namespace {

/// \brief 创建带范围的双精度微调框
QDoubleSpinBox *make_dbl(
    QWidget *parent, double v, double min_v, double max_v, int decimals, double step) {
  auto *s = new QDoubleSpinBox(parent);
  s->setDecimals(decimals);
  s->setRange(min_v, max_v);
  s->setSingleStep(step);
  s->setValue(v);
  return s;
}

/// \brief 创建带范围的整数微调框
QSpinBox *make_int(QWidget *parent, int v, int min_v, int max_v) {
  auto *s = new QSpinBox(parent);
  s->setRange(min_v, max_v);
  s->setValue(v);
  return s;
}

}  // namespace

/// \brief 构建 Launcher 风格分组配置面板
LauncherConfigPanel::LauncherConfigPanel(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(10);

  // ---- ROS / Topics（图像源、话题、帧名） ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    combo_source_mode_ = new QComboBox(host);
    combo_source_mode_->addItem(
        QStringLiteral("离线 · 图片目录"), static_cast<int>(SourceMode::Offline));
    combo_source_mode_->addItem(
        QStringLiteral("在线 · ROS 图像话题"), static_cast<int>(SourceMode::RosTopic));
    connect(
        combo_source_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        &LauncherConfigPanel::source_mode_changed);

    edit_image_dir_ = new QLineEdit(host);
    edit_image_dir_->setPlaceholderText(QStringLiteral("…/images"));
    auto *browse_dir = new QPushButton(QStringLiteral("浏览…"), host);
    browse_dir->setObjectName(QStringLiteral("GhostButton"));
    connect(browse_dir, &QPushButton::clicked, this, &LauncherConfigPanel::browse_image_dir_clicked);
    offline_row_ = new QWidget(host);
    auto *dir_lay = new QHBoxLayout(offline_row_);
    dir_lay->setContentsMargins(0, 0, 0, 0);
    dir_lay->addWidget(edit_image_dir_, 1);
    dir_lay->addWidget(browse_dir);

    combo_image_topic_ = new QComboBox(host);
    combo_image_topic_->setEditable(true);
    combo_image_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_image_topic_->lineEdit()->setPlaceholderText(
        QStringLiteral("/camera/image_raw"));
    connect(
        combo_image_topic_, &QComboBox::currentTextChanged, this,
        &LauncherConfigPanel::image_topic_changed);
    btn_refresh_topics_ = new QPushButton(QStringLiteral("刷新"), host);
    btn_refresh_topics_->setObjectName(QStringLiteral("GhostButton"));
    connect(
        btn_refresh_topics_, &QPushButton::clicked, this,
        &LauncherConfigPanel::refresh_topics_clicked);
    topic_row_ = new QWidget(host);
    auto *topic_lay = new QHBoxLayout(topic_row_);
    topic_lay->setContentsMargins(0, 0, 0, 0);
    topic_lay->addWidget(combo_image_topic_, 1);
    topic_lay->addWidget(btn_refresh_topics_);

    edit_camera_info_topic_ = new QLineEdit(QStringLiteral("/camera/camera_info"), host);
    chk_use_rectified_ = new QCheckBox(QStringLiteral("图像已去畸变 / rectified"), host);
    chk_use_rectified_->setChecked(false);
    edit_image_frame_ = new QLineEdit(QStringLiteral("camera_optical_frame"), host);
    edit_camera_link_frame_ = new QLineEdit(QStringLiteral("camera_link"), host);
    edit_config_path_ = new QLineEdit(host);
    edit_config_path_->setReadOnly(true);
    auto *reload = new QPushButton(QStringLiteral("从 YAML 加载"), host);
    reload->setObjectName(QStringLiteral("GhostButton"));
    connect(reload, &QPushButton::clicked, this, &LauncherConfigPanel::reload_yaml_clicked);
    auto *cfg_row = new QWidget(host);
    auto *cfg_lay = new QHBoxLayout(cfg_row);
    cfg_lay->setContentsMargins(0, 0, 0, 0);
    cfg_lay->addWidget(edit_config_path_, 1);
    cfg_lay->addWidget(reload);

    form->addRow(QStringLiteral("数据源"), combo_source_mode_);
    form->addRow(QStringLiteral("图片目录"), offline_row_);
    form->addRow(QStringLiteral("image_topic"), topic_row_);
    form->addRow(QStringLiteral("camera_info_topic"), edit_camera_info_topic_);
    form->addRow(QStringLiteral("use_rectified_image"), chk_use_rectified_);
    form->addRow(QStringLiteral("image_frame"), edit_image_frame_);
    form->addRow(QStringLiteral("camera_link_frame"), edit_camera_link_frame_);
    form->addRow(QStringLiteral("config YAML"), cfg_row);
    root->addWidget(make_group(QStringLiteral("ROS / Topics"), host));
  }

  // ---- 靶标 / 板参数 ----
  {
    auto *host = new QWidget(this);
    form_target_ = new_form(host);
    auto *form = form_target_;
    combo_target_type_ = new QComboBox(host);
    combo_target_type_->addItems(
        {QStringLiteral("chessboard"), QStringLiteral("charuco"),
         QStringLiteral("aruco_grid"), QStringLiteral("circles_symmetric"),
         QStringLiteral("circles_asymmetric"), QStringLiteral("trihedral_chess"),
         QStringLiteral("trihedral_charuco"), QStringLiteral("trihedral_aruco")});
    combo_dictionary_ = new QComboBox(host);
    combo_dictionary_->addItems(
        {QStringLiteral("DICT_4X4_250"), QStringLiteral("DICT_4X4_50"),
         QStringLiteral("DICT_4X4_100"), QStringLiteral("DICT_5X5_100"),
         QStringLiteral("DICT_6X6_250"), QStringLiteral("DICT_7X7_1000"),
         QStringLiteral("DICT_APRILTAG_36h11")});
    spin_squares_x_ = make_int(host, 9, 2, 40);
    spin_squares_y_ = make_int(host, 6, 2, 40);
    spin_square_length_ = make_dbl(host, 0.025, 0.001, 1.0, 4, 0.001);
    spin_square_length_->setSuffix(QStringLiteral(" m"));
    spin_marker_length_ = make_dbl(host, 0.018, 0.001, 1.0, 4, 0.001);
    spin_marker_length_->setSuffix(QStringLiteral(" m"));
    spin_min_board_area_ = make_dbl(host, 0.04, 0.001, 0.9, 3, 0.01);
    spin_max_board_area_ = make_dbl(host, 0.45, 0.01, 0.95, 3, 0.01);
    spin_max_tag_distance_ = make_dbl(host, 5.0, 0.1, 50.0, 2, 0.1);
    spin_max_tag_distance_->setSuffix(QStringLiteral(" m"));
    form->addRow(QStringLiteral("target"), combo_target_type_);
    form->addRow(QStringLiteral("dictionary"), combo_dictionary_);
    form->addRow(QStringLiteral("squares_x"), spin_squares_x_);
    form->addRow(QStringLiteral("squares_y"), spin_squares_y_);
    form->addRow(QStringLiteral("square_length"), spin_square_length_);
    form->addRow(QStringLiteral("marker_length"), spin_marker_length_);
    form->addRow(QStringLiteral("min_board_area_ratio"), spin_min_board_area_);
    form->addRow(QStringLiteral("max_board_area_ratio"), spin_max_board_area_);
    form->addRow(QStringLiteral("max_tag_distance"), spin_max_tag_distance_);
    connect(
        combo_target_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) { update_board_param_visibility(); });
    root->addWidget(make_group(QStringLiteral("Target / Board"), host));
  }

  // ---- 检测 / 采集 ----
  {
    auto *host = new QWidget(this);
    form_detect_ = new_form(host);
    auto *form = form_detect_;
    spin_min_views_ = make_int(host, 12, 1, 200);
    spin_min_confidence_ = make_dbl(host, 0.55, 0.1, 1.0, 2, 0.05);
    spin_min_diversity_ = make_dbl(host, 0.12, 0.03, 2.0, 2, 0.02);
    spin_auto_cooldown_ms_ = make_int(host, 900, 100, 10000);
    spin_auto_cooldown_ms_->setSuffix(QStringLiteral(" ms"));
    chk_auto_capture_default_ = new QCheckBox(
        QStringLiteral("进入工作台后默认开启自动采集"), host);
    chk_auto_capture_default_->setChecked(true);
    chk_cb_adaptive_ = new QCheckBox(QStringLiteral("ADAPTIVE_THRESH"), host);
    chk_cb_adaptive_->setChecked(true);
    chk_cb_normalize_ = new QCheckBox(QStringLiteral("NORMALIZE_IMAGE"), host);
    chk_cb_normalize_->setChecked(true);
    chk_cb_filter_quads_ = new QCheckBox(QStringLiteral("FILTER_QUADS"), host);
    chk_cb_fast_check_ = new QCheckBox(QStringLiteral("FAST_CHECK"), host);
    chk_cb_fast_check_->setChecked(true);
    spin_subpix_win_ = make_int(host, 11, 3, 31);
    form->addRow(QStringLiteral("calibration_pairs (最少姿态)"), spin_min_views_);
    form->addRow(QStringLiteral("min_confidence"), spin_min_confidence_);
    form->addRow(QStringLiteral("pairs_min_distance (多样性)"), spin_min_diversity_);
    form->addRow(QStringLiteral("auto_capture_cooldown"), spin_auto_cooldown_ms_);
    form->addRow(QStringLiteral("auto_capture"), chk_auto_capture_default_);
    form->addRow(QStringLiteral("findChessboard flags"), chk_cb_adaptive_);
    form->addRow(QString(), chk_cb_normalize_);
    form->addRow(QString(), chk_cb_filter_quads_);
    form->addRow(QString(), chk_cb_fast_check_);
    form->addRow(QStringLiteral("cornerSubPix window"), spin_subpix_win_);
    root->addWidget(make_group(QStringLiteral("Detection / Capture"), host));
  }

  // ---- 求解器（OpenCV calibrateCamera flags） ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    combo_camera_model_ = new QComboBox(host);
    combo_camera_model_->addItems({QStringLiteral("pinhole"), QStringLiteral("fisheye")});
    chk_fix_principal_ = new QCheckBox(QStringLiteral("FIX_PRINCIPAL_POINT"), host);
    chk_fix_aspect_ = new QCheckBox(QStringLiteral("FIX_ASPECT_RATIO"), host);
    chk_zero_tangent_ = new QCheckBox(QStringLiteral("ZERO_TANGENT_DIST"), host);
    chk_fix_k1_ = new QCheckBox(QStringLiteral("FIX_K1"), host);
    chk_fix_k2_ = new QCheckBox(QStringLiteral("FIX_K2"), host);
    chk_fix_k3_ = new QCheckBox(QStringLiteral("FIX_K3"), host);
    chk_fix_k3_->setChecked(true);
    chk_rational_model_ = new QCheckBox(QStringLiteral("RATIONAL_MODEL"), host);
    chk_thin_prism_ = new QCheckBox(QStringLiteral("THIN_PRISM_MODEL"), host);
    chk_use_intrinsic_guess_ = new QCheckBox(QStringLiteral("USE_INTRINSIC_GUESS"), host);
    form->addRow(QStringLiteral("camera_model"), combo_camera_model_);
    form->addRow(QStringLiteral("flags"), chk_fix_principal_);
    form->addRow(QString(), chk_fix_aspect_);
    form->addRow(QString(), chk_zero_tangent_);
    form->addRow(QString(), chk_fix_k1_);
    form->addRow(QString(), chk_fix_k2_);
    form->addRow(QString(), chk_fix_k3_);
    form->addRow(QString(), chk_rational_model_);
    form->addRow(QString(), chk_thin_prism_);
    form->addRow(QString(), chk_use_intrinsic_guess_);
    root->addWidget(make_group(QStringLiteral("Solver / Intrinsics"), host));
  }

  // ---- 坐标系 / 导出 ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    edit_parent_frame_ = new QLineEdit(QStringLiteral("camera_link"), host);
    edit_child_frame_ = new QLineEdit(QStringLiteral("camera_optical_frame"), host);
    edit_export_path_ = new QLineEdit(host);
    edit_export_path_->setPlaceholderText(QStringLiteral("~/calib_out"));
    form->addRow(QStringLiteral("parent_frame"), edit_parent_frame_);
    form->addRow(QStringLiteral("child_frame"), edit_child_frame_);
    form->addRow(QStringLiteral("export_dir"), edit_export_path_);
    root->addWidget(make_group(QStringLiteral("Frames / Output"), host));
  }

  // ---- 手眼 ----
  {
    handeye_block_ = new QWidget(this);
    auto *form = new_form(handeye_block_);
    edit_camera_yaml_ = new QLineEdit(handeye_block_);
    edit_camera_yaml_->setPlaceholderText(QStringLiteral("camera_intrinsics.yaml"));
    auto *browse_cam = new QPushButton(QStringLiteral("浏览…"), handeye_block_);
    browse_cam->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_cam, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_camera_yaml_clicked);
    auto *cam_row = new QWidget(handeye_block_);
    auto *cam_lay = new QHBoxLayout(cam_row);
    cam_lay->setContentsMargins(0, 0, 0, 0);
    cam_lay->addWidget(edit_camera_yaml_, 1);
    cam_lay->addWidget(browse_cam);

    combo_pose_source_ = new QComboBox(handeye_block_);
    combo_pose_source_->addItem(
        QStringLiteral("CSV"), static_cast<int>(PoseSource::Csv));
    combo_pose_source_->addItem(
        QStringLiteral("TF"), static_cast<int>(PoseSource::Tf));
    connect(
        combo_pose_source_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        &LauncherConfigPanel::pose_source_changed);

    edit_pose_csv_ = new QLineEdit(handeye_block_);
    auto *browse_csv = new QPushButton(QStringLiteral("浏览…"), handeye_block_);
    browse_csv->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_csv, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_pose_csv_clicked);
    auto *csv_row = new QWidget(handeye_block_);
    auto *csv_lay = new QHBoxLayout(csv_row);
    csv_lay->setContentsMargins(0, 0, 0, 0);
    csv_lay->addWidget(edit_pose_csv_, 1);
    csv_lay->addWidget(browse_csv);

    edit_base_frame_ = new QLineEdit(QStringLiteral("base"), handeye_block_);
    edit_gripper_frame_ = new QLineEdit(QStringLiteral("tool0"), handeye_block_);
    combo_handeye_method_ = new QComboBox(handeye_block_);
    combo_handeye_method_->addItems(
        {QStringLiteral("tsai"), QStringLiteral("park"), QStringLiteral("horaud"),
         QStringLiteral("andreff"), QStringLiteral("daniilidis")});

    form->addRow(QStringLiteral("camera_yaml"), cam_row);
    form->addRow(QStringLiteral("pose_source"), combo_pose_source_);
    form->addRow(QStringLiteral("pose_csv"), csv_row);
    form->addRow(QStringLiteral("base_frame"), edit_base_frame_);
    form->addRow(QStringLiteral("gripper_frame"), edit_gripper_frame_);
    form->addRow(QStringLiteral("handeye_method"), combo_handeye_method_);
    root->addWidget(make_group(QStringLiteral("Hand-eye"), handeye_block_));
    handeye_block_->setVisible(false);
  }

  // ---- 可视化默认项 ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    chk_viz_corners_ = new QCheckBox(QStringLiteral("draw chessboard corners"), host);
    chk_viz_corners_->setChecked(true);
    chk_viz_hull_ = new QCheckBox(QStringLiteral("draw board hull"), host);
    chk_viz_hull_->setChecked(true);
    chk_viz_conf_ = new QCheckBox(QStringLiteral("show confidence text"), host);
    chk_viz_conf_->setChecked(true);
    chk_viz_aruco_ = new QCheckBox(QStringLiteral("draw ArUco markers + IDs"), host);
    chk_viz_aruco_->setChecked(true);
    spin_viz_marker_radius_ = make_int(host, 4, 1, 20);
    form->addRow(QStringLiteral("overlay"), chk_viz_corners_);
    form->addRow(QString(), chk_viz_hull_);
    form->addRow(QString(), chk_viz_conf_);
    form->addRow(QString(), chk_viz_aruco_);
    form->addRow(QStringLiteral("marker_size (px)"), spin_viz_marker_radius_);
    root->addWidget(make_group(QStringLiteral("Visualization"), host));
  }

  root->addStretch(1);
  update_board_param_visibility();
}

/// \brief 用 QGroupBox 包装一组配置
QWidget *LauncherConfigPanel::make_group(const QString &title, QWidget *body) {
  auto *box = new QGroupBox(title, this);
  box->setObjectName(QStringLiteral("LauncherGroup"));
  auto *lay = new QVBoxLayout(box);
  lay->setContentsMargins(12, 14, 12, 12);
  lay->addWidget(body);
  return box;
}

/// \brief 在宿主上创建紧凑表单布局
QFormLayout *LauncherConfigPanel::new_form(QWidget *host) {
  auto *form = new QFormLayout(host);
  form->setContentsMargins(0, 0, 0, 0);
  form->setSpacing(8);
  form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  return form;
}

/// \brief 按字段显隐表单行
void LauncherConfigPanel::set_form_row_visible(
    QFormLayout *form, QWidget *field, bool visible) {
  if (field == nullptr) {
    return;
  }
  field->setVisible(visible);
  if (form != nullptr) {
    if (QWidget *lab = form->labelForField(field)) {
      lab->setVisible(visible);
    }
  }
}

/// \brief 按靶标类型显隐板参数与检测选项
void LauncherConfigPanel::update_board_param_visibility() {
  if (combo_target_type_ == nullptr) {
    return;
  }
  const QString t = combo_target_type_->currentText();
  const bool chess =
      t == QStringLiteral("chessboard") || t == QStringLiteral("trihedral_chess");
  const bool charuco =
      t == QStringLiteral("charuco") || t == QStringLiteral("trihedral_charuco");
  const bool aruco =
      t == QStringLiteral("aruco_grid") || t == QStringLiteral("trihedral_aruco");
  const bool circles = t == QStringLiteral("circles_symmetric") ||
                       t == QStringLiteral("circles_asymmetric");
  const bool tag_family = charuco || aruco;

  // Tag / ChArUco：字典、标记边长、最远距离
  set_form_row_visible(form_target_, combo_dictionary_, tag_family);
  set_form_row_visible(form_target_, spin_marker_length_, tag_family);
  set_form_row_visible(form_target_, spin_max_tag_distance_, tag_family);

  // 几何尺寸：各类靶标都需要行列 + 尺度；标签文案随类型变化
  set_form_row_visible(form_target_, spin_squares_x_, true);
  set_form_row_visible(form_target_, spin_squares_y_, true);
  set_form_row_visible(form_target_, spin_square_length_, true);
  set_form_row_visible(form_target_, spin_min_board_area_, true);
  set_form_row_visible(form_target_, spin_max_board_area_, true);

  auto set_lab = [this](QWidget *field, const QString &text) {
    if (form_target_ == nullptr || field == nullptr) {
      return;
    }
    if (QWidget *lab = form_target_->labelForField(field)) {
      if (auto *ql = qobject_cast<QLabel *>(lab)) {
        ql->setText(text);
      }
    }
  };

  if (chess) {
    set_lab(spin_squares_x_, QStringLiteral("squares_x (内角点列)"));
    set_lab(spin_squares_y_, QStringLiteral("squares_y (内角点行)"));
    set_lab(spin_square_length_, QStringLiteral("square_length (方格边长)"));
    if (t.startsWith(QStringLiteral("trihedral"))) {
      set_lab(spin_squares_x_, QStringLiteral("squares_n (正方形内角点)"));
      set_lab(spin_squares_y_, QStringLiteral("squares_n (同左，正方形)"));
      // 保持正方形：两轴同步为较大值
      if (spin_squares_x_ && spin_squares_y_) {
        const int n = std::max(spin_squares_x_->value(), spin_squares_y_->value());
        if (spin_squares_x_->value() != n) {
          spin_squares_x_->setValue(n);
        }
        if (spin_squares_y_->value() != n) {
          spin_squares_y_->setValue(n);
        }
      }
    }
  } else if (charuco) {
    if (t == QStringLiteral("trihedral_charuco")) {
      set_lab(spin_squares_x_, QStringLiteral("squares_n (内角点；方格=n+1)"));
      set_lab(spin_squares_y_, QStringLiteral("squares_n (同左，正方形)"));
      set_lab(spin_square_length_, QStringLiteral("square_length (方格边长)"));
      set_lab(spin_marker_length_, QStringLiteral("marker_length (DICT_4X4_250)"));
      if (spin_squares_x_ && spin_squares_y_) {
        const int n = std::max(spin_squares_x_->value(), spin_squares_y_->value());
        if (spin_squares_x_->value() != n) {
          spin_squares_x_->setValue(n);
        }
        if (spin_squares_y_->value() != n) {
          spin_squares_y_->setValue(n);
        }
      }
    } else {
      set_lab(spin_squares_x_, QStringLiteral("squares_x (方格列)"));
      set_lab(spin_squares_y_, QStringLiteral("squares_y (方格行)"));
      set_lab(spin_square_length_, QStringLiteral("square_length (方格边长)"));
      set_lab(spin_marker_length_, QStringLiteral("marker_length (标记边长)"));
    }
  } else if (aruco) {
    set_lab(spin_squares_x_, QStringLiteral("markers_x (标记列)"));
    set_lab(spin_squares_y_, QStringLiteral("markers_y (标记行)"));
    set_lab(spin_square_length_, QStringLiteral("cell_pitch (标记+间距)"));
    set_lab(spin_marker_length_, QStringLiteral("marker_length (标记边长)"));
  } else if (circles) {
    set_lab(spin_squares_x_, QStringLiteral("circles_x (圆点列)"));
    set_lab(spin_squares_y_, QStringLiteral("circles_y (圆点行)"));
    set_lab(spin_square_length_, QStringLiteral("center_distance (圆心距)"));
  }

  // findChessboard 选项仅棋盘格 / 三面棋盘
  const bool show_cb_flags = chess;
  set_form_row_visible(form_detect_, chk_cb_adaptive_, show_cb_flags);
  set_form_row_visible(form_detect_, chk_cb_normalize_, show_cb_flags);
  set_form_row_visible(form_detect_, chk_cb_filter_quads_, show_cb_flags);
  set_form_row_visible(form_detect_, chk_cb_fast_check_, show_cb_flags);
  set_form_row_visible(form_detect_, spin_subpix_win_, show_cb_flags);
}

/// \brief 切换标定器并调整手眼块/默认靶标
void LauncherConfigPanel::set_calibrator_id(const QString &id) {
  const QString prev = calibrator_id_;
  calibrator_id_ = id;
  const bool he =
      id == QStringLiteral("eye_in_hand") || id == QStringLiteral("eye_to_hand");
  if (handeye_block_ != nullptr) {
    handeye_block_->setVisible(he);
  }
  const bool tri = id == QStringLiteral("trihedral_oneshot");
  // 仅在「刚选中」三面标定器时写入默认靶标；反复进设置页/同步会话不得覆盖用户已选的 chess
  const bool entering_tri = tri && prev != id;
  if (tri) {
    if (entering_tri && combo_target_type_ != nullptr) {
      const int idx = combo_target_type_->findText(QStringLiteral("trihedral_charuco"));
      if (idx >= 0) {
        combo_target_type_->setCurrentIndex(idx);
      }
    }
    if (entering_tri && combo_dictionary_ != nullptr) {
      const int didx = combo_dictionary_->findText(QStringLiteral("DICT_4X4_250"));
      if (didx >= 0) {
        combo_dictionary_->setCurrentIndex(didx);
      }
    }
    // 三面靶强制正方形网格；与 Isaac 默认 8 内角点对齐
    if (spin_squares_x_ != nullptr && spin_squares_y_ != nullptr) {
      const int n = std::max(spin_squares_x_->value(), spin_squares_y_->value());
      const int use = (n == 9 || n <= 6) ? 8 : std::max(3, n);
      spin_squares_x_->setValue(use);
      spin_squares_y_->setValue(use);
    }
    if (spin_min_views_ != nullptr) {
      spin_min_views_->setMinimum(1);
      if (spin_min_views_->value() < 1 || spin_min_views_->value() > 3) {
        spin_min_views_->setValue(1);
      }
    }
  } else if (spin_min_views_ != nullptr) {
    spin_min_views_->setMinimum(1);
  }
  update_board_param_visibility();
}

/// \brief 将面板参数写入 SessionController
void LauncherConfigPanel::apply_to_session(SessionController *session) const {
  if (session == nullptr) {
    return;
  }
  session->set_calibrator_id(calibrator_id_);
  session->set_board_params(squares_x(), squares_y(), square_length());
  session->set_capture_options(
      min_views(), min_confidence(), min_diversity(), auto_cooldown_ms());
  session->set_solve_options(to_config_map());
  if (edit_camera_yaml_ != nullptr) {
    session->set_camera_yaml(edit_camera_yaml_->text().trimmed());
  }
  if (edit_base_frame_ != nullptr && edit_gripper_frame_ != nullptr) {
    session->set_handeye_frames(
        edit_base_frame_->text().trimmed(), edit_gripper_frame_->text().trimmed());
  }
  if (combo_handeye_method_ != nullptr) {
    session->set_handeye_method(combo_handeye_method_->currentText());
  }
  if (combo_pose_source_ != nullptr && session->is_handeye()) {
    session->set_pose_source(
        static_cast<PoseSource>(combo_pose_source_->currentData().toInt()));
  } else {
    session->set_pose_source(PoseSource::None);
  }
  if (combo_source_mode_ != nullptr) {
    session->set_source_mode(
        static_cast<SourceMode>(combo_source_mode_->currentData().toInt()));
  }
}

/// \brief 导出求解用配置字典
std::map<std::string, std::string> LauncherConfigPanel::to_config_map() const {
  auto b = [](bool v) { return v ? "1" : "0"; };
  std::map<std::string, std::string> m = {
      {"squares_x", std::to_string(squares_x())},
      {"squares_y", std::to_string(squares_y())},
      {"square_length", std::to_string(square_length())},
      {"marker_length", std::to_string(marker_length())},
      {"target", combo_target_type_ ? combo_target_type_->currentText().toStdString()
                                    : "chessboard"},
      {"dictionary", combo_dictionary_ ? combo_dictionary_->currentText().toStdString()
                                       : "DICT_4X4_250"},
      {"model", combo_camera_model_ ? combo_camera_model_->currentText().toStdString()
                                    : "pinhole"},
      {"min_views", std::to_string(min_views())},
      {"min_confidence", std::to_string(min_confidence())},
      {"min_diversity", std::to_string(min_diversity())},
      {"auto_cooldown_ms", std::to_string(auto_cooldown_ms())},
      {"min_board_area", std::to_string(spin_min_board_area_->value())},
      {"max_board_area", std::to_string(spin_max_board_area_->value())},
      {"max_tag_distance", std::to_string(spin_max_tag_distance_->value())},
      {"cb_adaptive", b(chk_cb_adaptive_->isChecked())},
      {"cb_normalize", b(chk_cb_normalize_->isChecked())},
      {"cb_filter_quads", b(chk_cb_filter_quads_->isChecked())},
      {"cb_fast_check", b(chk_cb_fast_check_->isChecked())},
      {"subpix_win", std::to_string(spin_subpix_win_->value())},
      {"fix_principal", b(chk_fix_principal_->isChecked())},
      {"fix_aspect", b(chk_fix_aspect_->isChecked())},
      {"zero_tangent", b(chk_zero_tangent_->isChecked())},
      {"fix_k1", b(chk_fix_k1_->isChecked())},
      {"fix_k2", b(chk_fix_k2_->isChecked())},
      {"fix_k3", b(chk_fix_k3_->isChecked())},
      {"rational_model", b(chk_rational_model_->isChecked())},
      {"thin_prism", b(chk_thin_prism_->isChecked())},
      {"use_intrinsic_guess", b(chk_use_intrinsic_guess_->isChecked())},
      {"use_rectified_image", b(chk_use_rectified_->isChecked())},
      {"image_frame", edit_image_frame_->text().trimmed().toStdString()},
      {"camera_link_frame", edit_camera_link_frame_->text().trimmed().toStdString()},
      {"camera_info_topic", edit_camera_info_topic_->text().trimmed().toStdString()},
      {"parent_frame", edit_parent_frame_->text().trimmed().toStdString()},
      {"child_frame", edit_child_frame_->text().trimmed().toStdString()},
      {"viz_corners", b(chk_viz_corners_->isChecked())},
      {"viz_hull", b(chk_viz_hull_->isChecked())},
      {"viz_conf", b(chk_viz_conf_->isChecked())},
      {"viz_aruco", b(chk_viz_aruco_ && chk_viz_aruco_->isChecked())},
      {"viz_marker_radius", std::to_string(spin_viz_marker_radius_->value())},
  };
  if (combo_handeye_method_ != nullptr) {
    m["method"] = combo_handeye_method_->currentText().toStdString();
  }
  if (edit_camera_yaml_ != nullptr && !edit_camera_yaml_->text().trimmed().isEmpty()) {
    m["camera_yaml"] = edit_camera_yaml_->text().trimmed().toStdString();
  }
  return m;
}

/// \brief 方格/内角点 X 数
int LauncherConfigPanel::squares_x() const {
  return spin_squares_x_ ? spin_squares_x_->value() : 9;
}
/// \brief 方格/内角点 Y 数
int LauncherConfigPanel::squares_y() const {
  return spin_squares_y_ ? spin_squares_y_->value() : 6;
}
/// \brief 方格边长（米）
double LauncherConfigPanel::square_length() const {
  return spin_square_length_ ? spin_square_length_->value() : 0.025;
}
/// \brief ArUco/ChArUco 码边长（米）
double LauncherConfigPanel::marker_length() const {
  return spin_marker_length_ ? spin_marker_length_->value() : 0.018;
}
/// \brief 最少采集视角数
int LauncherConfigPanel::min_views() const {
  return spin_min_views_ ? spin_min_views_->value() : 12;
}
/// \brief 自动采集最低置信度
double LauncherConfigPanel::min_confidence() const {
  return spin_min_confidence_ ? spin_min_confidence_->value() : 0.55;
}
/// \brief 自动采集最小视角差异
double LauncherConfigPanel::min_diversity() const {
  return spin_min_diversity_ ? spin_min_diversity_->value() : 0.12;
}
/// \brief 自动采集冷却毫秒
int LauncherConfigPanel::auto_cooldown_ms() const {
  return spin_auto_cooldown_ms_ ? spin_auto_cooldown_ms_->value() : 900;
}
/// \brief 是否默认开启自动采集
bool LauncherConfigPanel::auto_capture_default() const {
  return chk_auto_capture_default_ && chk_auto_capture_default_->isChecked();
}
/// \brief 是否画角点
bool LauncherConfigPanel::viz_draw_corners() const {
  return !chk_viz_corners_ || chk_viz_corners_->isChecked();
}
/// \brief 是否画凸包
bool LauncherConfigPanel::viz_draw_hull() const {
  return !chk_viz_hull_ || chk_viz_hull_->isChecked();
}
/// \brief 是否显示置信度文字
bool LauncherConfigPanel::viz_show_conf() const {
  return !chk_viz_conf_ || chk_viz_conf_->isChecked();
}
/// \brief 是否叠加 ArUco
bool LauncherConfigPanel::viz_draw_aruco() const {
  return !chk_viz_aruco_ || chk_viz_aruco_->isChecked();
}
/// \brief 角点/码绘制半径
int LauncherConfigPanel::viz_marker_radius() const {
  return spin_viz_marker_radius_ ? spin_viz_marker_radius_->value() : 4;
}

/// \brief 写入板参数到控件
void LauncherConfigPanel::set_board_params(
    int sx, int sy, double square_m, double marker_m) {
  if (spin_squares_x_) {
    spin_squares_x_->setValue(sx);
  }
  if (spin_squares_y_) {
    spin_squares_y_->setValue(sy);
  }
  if (spin_square_length_) {
    spin_square_length_->setValue(square_m);
  }
  if (spin_marker_length_) {
    spin_marker_length_->setValue(marker_m);
  }
}

/// \brief 设置配置路径编辑框
void LauncherConfigPanel::set_config_path(const QString &path) {
  if (edit_config_path_) {
    edit_config_path_->setText(path);
  }
}

/// \brief 设置离线图片目录
void LauncherConfigPanel::set_image_dir(const QString &path) {
  if (edit_image_dir_) {
    edit_image_dir_->setText(path);
  }
}

/// \brief 导出目录提示路径
QString LauncherConfigPanel::export_dir_hint() const {
  return edit_export_path_ ? edit_export_path_->text().trimmed() : QString();
}

}  // namespace gui
}  // namespace hs_calib
