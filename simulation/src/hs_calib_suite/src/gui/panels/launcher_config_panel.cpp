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
#include <QSizePolicy>
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

/// \brief 紧凑「标签 + 控件」单元
QWidget *make_field_unit(QWidget *parent, const QString &text, QWidget *field, QLabel **lab_out) {
  auto *unit = new QWidget(parent);
  auto *lay = new QHBoxLayout(unit);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(6);
  auto *lab = new QLabel(text, unit);
  lab->setObjectName(QStringLiteral("Muted"));
  lay->addWidget(lab);
  lay->addWidget(field, 1);
  if (lab_out != nullptr) {
    *lab_out = lab;
  }
  return unit;
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
    form_ros_ = new_form(host);
    auto *form = form_ros_;
    combo_source_mode_ = new QComboBox(host);
    combo_source_mode_->addItem(
        QStringLiteral("离线 · 图片目录"), static_cast<int>(SourceMode::Offline));
    combo_source_mode_->addItem(
        QStringLiteral("在线 · ROS 图像话题"), static_cast<int>(SourceMode::RosTopic));
    connect(
        combo_source_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int index) {
          refresh_source_mode_rows();
          emit source_mode_changed(index);
        });

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

    combo_camera_info_topic_ = new QComboBox(host);
    combo_camera_info_topic_->setEditable(true);
    combo_camera_info_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_camera_info_topic_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    combo_camera_info_topic_->setMinimumWidth(200);
    combo_camera_info_topic_->lineEdit()->setPlaceholderText(
        QStringLiteral("/camera/camera_info"));
    combo_camera_info_topic_->setEditText(QStringLiteral("/camera/camera_info"));
    connect(
        combo_camera_info_topic_, &QComboBox::currentTextChanged, this,
        &LauncherConfigPanel::camera_info_topic_changed);
    camera_info_row_ = new QWidget(host);
    auto *info_lay = new QHBoxLayout(camera_info_row_);
    info_lay->setContentsMargins(0, 0, 0, 0);
    info_lay->addWidget(combo_camera_info_topic_, 1);
    chk_use_rectified_ = new QCheckBox(QStringLiteral("已去畸变"), host);
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

    auto *frame_row = new QWidget(host);
    auto *frame_lay = new QHBoxLayout(frame_row);
    frame_lay->setContentsMargins(0, 0, 0, 0);
    frame_lay->setSpacing(10);
    frame_lay->addWidget(make_field_unit(frame_row, QStringLiteral("optical"), edit_image_frame_, nullptr), 1);
    frame_lay->addWidget(
        make_field_unit(frame_row, QStringLiteral("link"), edit_camera_link_frame_, nullptr), 1);

    form->addRow(QStringLiteral("数据源"), combo_source_mode_);
    form->addRow(QStringLiteral("图片目录"), offline_row_);
    form->addRow(QStringLiteral("image_topic"), topic_row_);
    form->addRow(QStringLiteral("camera_info"), camera_info_row_);
    form->addRow(QStringLiteral("rectified"), chk_use_rectified_);
    form->addRow(QStringLiteral("frames"), frame_row);
    form->addRow(QStringLiteral("config"), cfg_row);
    root->addWidget(make_group(QStringLiteral("ROS / Topics"), host));
    refresh_source_mode_rows();
  }

  // ---- 靶标 / 板参数 ----
  {
    auto *host = new QWidget(this);
    form_target_ = new_form(host);
    auto *form = form_target_;
    combo_target_type_ = new QComboBox(host);
    combo_target_type_->addItems(
        {QStringLiteral("chessboard"), QStringLiteral("charuco"),
         QStringLiteral("aruco"), QStringLiteral("aruco_grid"),
         QStringLiteral("circles_symmetric"), QStringLiteral("circles_asymmetric"),
         QStringLiteral("trihedral_chess"), QStringLiteral("trihedral_charuco"),
         QStringLiteral("trihedral_aruco")});
    combo_dictionary_ = new QComboBox(host);
    combo_dictionary_->addItems(
        {QStringLiteral("DICT_6X6_1000"), QStringLiteral("DICT_6X6_250"),
         QStringLiteral("DICT_4X4_250"), QStringLiteral("DICT_4X4_50"),
         QStringLiteral("DICT_4X4_100"), QStringLiteral("DICT_5X5_50"),
         QStringLiteral("DICT_5X5_100"), QStringLiteral("DICT_5X5_250"),
         QStringLiteral("DICT_6X6_50"), QStringLiteral("DICT_7X7_1000"),
         QStringLiteral("DICT_ARUCO_ORIGINAL"),
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

    squares_row_ = new QWidget(host);
    auto *sq_lay = new QHBoxLayout(squares_row_);
    sq_lay->setContentsMargins(0, 0, 0, 0);
    sq_lay->setSpacing(10);
    sq_lay->addWidget(
        make_field_unit(squares_row_, QStringLiteral("X"), spin_squares_x_, &lab_squares_x_), 1);
    sq_lay->addWidget(
        make_field_unit(squares_row_, QStringLiteral("Y"), spin_squares_y_, &lab_squares_y_), 1);

    lengths_row_ = new QWidget(host);
    auto *len_lay = new QHBoxLayout(lengths_row_);
    len_lay->setContentsMargins(0, 0, 0, 0);
    len_lay->setSpacing(10);
    len_lay->addWidget(
        make_field_unit(
            lengths_row_, QStringLiteral("方格"), spin_square_length_, &lab_square_length_),
        1);
    len_lay->addWidget(
        make_field_unit(
            lengths_row_, QStringLiteral("标记"), spin_marker_length_, &lab_marker_length_),
        1);

    area_row_ = new QWidget(host);
    auto *area_lay = new QHBoxLayout(area_row_);
    area_lay->setContentsMargins(0, 0, 0, 0);
    area_lay->setSpacing(10);
    area_lay->addWidget(
        make_field_unit(area_row_, QStringLiteral("面积min"), spin_min_board_area_, nullptr), 1);
    area_lay->addWidget(
        make_field_unit(area_row_, QStringLiteral("面积max"), spin_max_board_area_, nullptr), 1);
    area_lay->addWidget(
        make_field_unit(area_row_, QStringLiteral("最远"), spin_max_tag_distance_, nullptr), 1);

    form->addRow(QStringLiteral("target"), combo_target_type_);
    form->addRow(QStringLiteral("dictionary"), combo_dictionary_);
    form->addRow(QStringLiteral("网格"), squares_row_);
    form->addRow(QStringLiteral("尺寸"), lengths_row_);
    form->addRow(QStringLiteral("过滤"), area_row_);
    connect(
        spin_squares_x_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int v) {
          if (combo_target_type_ == nullptr || spin_squares_y_ == nullptr) {
            return;
          }
          if (combo_target_type_->currentText().startsWith(QStringLiteral("trihedral")) &&
              spin_squares_y_->value() != v) {
            spin_squares_y_->setValue(v);
          }
        });
    connect(
        combo_target_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) {
          update_board_param_visibility();
          if (combo_target_type_ == nullptr) {
            return;
          }
          if (combo_target_type_->currentText() == QStringLiteral("aruco")) {
            if (combo_dictionary_ != nullptr) {
              const int didx =
                  combo_dictionary_->findText(QStringLiteral("DICT_6X6_1000"));
              if (didx >= 0) {
                combo_dictionary_->setCurrentIndex(didx);
              }
            }
            if (spin_marker_length_ != nullptr &&
                spin_marker_length_->value() < 0.05) {
              spin_marker_length_->setValue(0.10);
            }
          }
        });
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
        QStringLiteral("进入工作台默认自动采集"), host);
    chk_auto_capture_default_->setChecked(true);
    chk_cb_adaptive_ = new QCheckBox(QStringLiteral("ADAPTIVE"), host);
    chk_cb_adaptive_->setChecked(true);
    chk_cb_normalize_ = new QCheckBox(QStringLiteral("NORMALIZE"), host);
    chk_cb_normalize_->setChecked(true);
    chk_cb_filter_quads_ = new QCheckBox(QStringLiteral("FILTER_QUADS"), host);
    chk_cb_fast_check_ = new QCheckBox(QStringLiteral("FAST_CHECK"), host);
    chk_cb_fast_check_->setChecked(true);
    spin_subpix_win_ = make_int(host, 11, 3, 31);
    combo_stereo_side_ = new QComboBox(host);
    combo_stereo_side_->addItem(QStringLiteral("left"), QStringLiteral("left"));
    combo_stereo_side_->addItem(QStringLiteral("right"), QStringLiteral("right"));

    auto *capture_row = new QWidget(host);
    auto *cap_lay = new QHBoxLayout(capture_row);
    cap_lay->setContentsMargins(0, 0, 0, 0);
    cap_lay->setSpacing(10);
    cap_lay->addWidget(
        make_field_unit(capture_row, QStringLiteral("最少姿态"), spin_min_views_, nullptr), 1);
    cap_lay->addWidget(
        make_field_unit(capture_row, QStringLiteral("置信度"), spin_min_confidence_, nullptr), 1);
    cap_lay->addWidget(
        make_field_unit(capture_row, QStringLiteral("多样性"), spin_min_diversity_, nullptr), 1);

    auto *auto_row = new QWidget(host);
    auto *auto_lay = new QHBoxLayout(auto_row);
    auto_lay->setContentsMargins(0, 0, 0, 0);
    auto_lay->setSpacing(10);
    auto_lay->addWidget(
        make_field_unit(auto_row, QStringLiteral("冷却"), spin_auto_cooldown_ms_, nullptr), 1);
    auto_lay->addWidget(chk_auto_capture_default_, 1);

    auto *flags_row = new QWidget(host);
    chess_flags_row_ = flags_row;
    auto *flags_lay = new QHBoxLayout(flags_row);
    flags_lay->setContentsMargins(0, 0, 0, 0);
    flags_lay->setSpacing(8);
    flags_lay->addWidget(chk_cb_adaptive_);
    flags_lay->addWidget(chk_cb_normalize_);
    flags_lay->addWidget(chk_cb_filter_quads_);
    flags_lay->addWidget(chk_cb_fast_check_);
    flags_lay->addStretch(1);
    flags_lay->addWidget(
        make_field_unit(flags_row, QStringLiteral("subpix"), spin_subpix_win_, nullptr));

    form->addRow(QStringLiteral("stereo_side"), combo_stereo_side_);
    form->addRow(QStringLiteral("采集"), capture_row);
    form->addRow(QStringLiteral("自动"), auto_row);
    form->addRow(QStringLiteral("棋盘 flags"), flags_row);
    root->addWidget(make_group(QStringLiteral("Detection / Capture"), host));
  }

  // ---- 求解器（OpenCV calibrateCamera flags） ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    combo_camera_model_ = new QComboBox(host);
    combo_camera_model_->addItem(
        QStringLiteral("Brown-Conrady (pinhole)"), QStringLiteral("brown_conrady"));
    combo_camera_model_->addItem(
        QStringLiteral("Kannala-Brandt (fisheye)"), QStringLiteral("kannala_brandt"));
    combo_camera_model_->addItem(
        QStringLiteral("CMei (omnidir)"), QStringLiteral("cmei"));
    combo_camera_model_->setCurrentIndex(0);
    chk_fix_principal_ = new QCheckBox(QStringLiteral("FIX_PP"), host);
    chk_fix_aspect_ = new QCheckBox(QStringLiteral("FIX_ASPECT"), host);
    chk_zero_tangent_ = new QCheckBox(QStringLiteral("ZERO_TANGENT"), host);
    chk_fix_k1_ = new QCheckBox(QStringLiteral("FIX_K1"), host);
    chk_fix_k2_ = new QCheckBox(QStringLiteral("FIX_K2"), host);
    chk_fix_k3_ = new QCheckBox(QStringLiteral("FIX_K3"), host);
    chk_fix_k3_->setChecked(true);
    chk_rational_model_ = new QCheckBox(QStringLiteral("RATIONAL"), host);
    chk_thin_prism_ = new QCheckBox(QStringLiteral("THIN_PRISM"), host);
    chk_use_intrinsic_guess_ = new QCheckBox(QStringLiteral("USE_GUESS"), host);

    auto *flags1 = new QWidget(host);
    auto *f1 = new QHBoxLayout(flags1);
    f1->setContentsMargins(0, 0, 0, 0);
    f1->setSpacing(8);
    for (auto *c : {chk_fix_principal_, chk_fix_aspect_, chk_zero_tangent_,
                    chk_fix_k1_, chk_fix_k2_}) {
      f1->addWidget(c);
    }
    f1->addStretch(1);

    auto *flags2 = new QWidget(host);
    auto *f2 = new QHBoxLayout(flags2);
    f2->setContentsMargins(0, 0, 0, 0);
    f2->setSpacing(8);
    for (auto *c :
         {chk_fix_k3_, chk_rational_model_, chk_thin_prism_, chk_use_intrinsic_guess_}) {
      f2->addWidget(c);
    }
    f2->addStretch(1);

    form->addRow(QStringLiteral("model"), combo_camera_model_);
    form->addRow(QStringLiteral("flags"), flags1);
    form->addRow(QString(), flags2);
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
    auto *tf_row = new QWidget(host);
    auto *tf_lay = new QHBoxLayout(tf_row);
    tf_lay->setContentsMargins(0, 0, 0, 0);
    tf_lay->setSpacing(10);
    tf_lay->addWidget(
        make_field_unit(tf_row, QStringLiteral("parent"), edit_parent_frame_, nullptr), 1);
    tf_lay->addWidget(
        make_field_unit(tf_row, QStringLiteral("child"), edit_child_frame_, nullptr), 1);
    form->addRow(QStringLiteral("frames"), tf_row);
    form->addRow(QStringLiteral("export"), edit_export_path_);
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

  // ---- 双目外参（左右内参 YAML）----
  {
    stereo_extrinsics_block_ = new QWidget(this);
    auto *form = new_form(stereo_extrinsics_block_);

    edit_left_camera_yaml_ = new QLineEdit(stereo_extrinsics_block_);
    edit_left_camera_yaml_->setPlaceholderText(QStringLiteral("camera_left.yaml"));
    auto *browse_l = new QPushButton(QStringLiteral("浏览…"), stereo_extrinsics_block_);
    browse_l->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_l, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_left_camera_yaml_clicked);
    auto *left_row = new QWidget(stereo_extrinsics_block_);
    auto *left_lay = new QHBoxLayout(left_row);
    left_lay->setContentsMargins(0, 0, 0, 0);
    left_lay->addWidget(edit_left_camera_yaml_, 1);
    left_lay->addWidget(browse_l);

    edit_right_camera_yaml_ = new QLineEdit(stereo_extrinsics_block_);
    edit_right_camera_yaml_->setPlaceholderText(QStringLiteral("camera_right.yaml"));
    auto *browse_r = new QPushButton(QStringLiteral("浏览…"), stereo_extrinsics_block_);
    browse_r->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_r, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_right_camera_yaml_clicked);
    auto *right_row = new QWidget(stereo_extrinsics_block_);
    auto *right_lay = new QHBoxLayout(right_row);
    right_lay->setContentsMargins(0, 0, 0, 0);
    right_lay->addWidget(edit_right_camera_yaml_, 1);
    right_lay->addWidget(browse_r);

    form->addRow(QStringLiteral("left_camera_yaml"), left_row);
    form->addRow(QStringLiteral("right_camera_yaml"), right_row);
    root->addWidget(
        make_group(QStringLiteral("Stereo extrinsics"), stereo_extrinsics_block_));
    stereo_extrinsics_block_->setVisible(false);
  }

  // ---- 可视化默认项 ----
  {
    auto *host = new QWidget(this);
    auto *form = new_form(host);
    chk_viz_corners_ = new QCheckBox(QStringLiteral("角点"), host);
    chk_viz_corners_->setChecked(true);
    chk_viz_hull_ = new QCheckBox(QStringLiteral("凸包"), host);
    chk_viz_hull_->setChecked(true);
    chk_viz_conf_ = new QCheckBox(QStringLiteral("置信度"), host);
    chk_viz_conf_->setChecked(true);
    chk_viz_aruco_ = new QCheckBox(QStringLiteral("ArUco"), host);
    chk_viz_aruco_->setChecked(true);
    spin_viz_marker_radius_ = make_int(host, 4, 1, 20);
    auto *viz_row = new QWidget(host);
    auto *viz_lay = new QHBoxLayout(viz_row);
    viz_lay->setContentsMargins(0, 0, 0, 0);
    viz_lay->setSpacing(8);
    viz_lay->addWidget(chk_viz_corners_);
    viz_lay->addWidget(chk_viz_hull_);
    viz_lay->addWidget(chk_viz_conf_);
    viz_lay->addWidget(chk_viz_aruco_);
    viz_lay->addStretch(1);
    viz_lay->addWidget(
        make_field_unit(viz_row, QStringLiteral("半径"), spin_viz_marker_radius_, nullptr));
    form->addRow(QStringLiteral("overlay"), viz_row);
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

/// \brief 按离线/ROS 显隐对应表单行（含左侧标签）
void LauncherConfigPanel::refresh_source_mode_rows() {
  const bool ros_mode =
      combo_source_mode_ != nullptr &&
      combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic);
  set_form_row_visible(form_ros_, offline_row_, !ros_mode);
  set_form_row_visible(form_ros_, topic_row_, ros_mode);
  set_form_row_visible(form_ros_, camera_info_row_, ros_mode);
  set_form_row_visible(form_ros_, chk_use_rectified_, ros_mode);
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
      t == QStringLiteral("aruco") || t == QStringLiteral("aruco_grid") ||
      t == QStringLiteral("trihedral_aruco");
  const bool circles = t == QStringLiteral("circles_symmetric") ||
                       t == QStringLiteral("circles_asymmetric");
  const bool tag_family = charuco || aruco;
  const bool aruco_single = t == QStringLiteral("aruco");
  const bool square_board = t.startsWith(QStringLiteral("trihedral"));

  set_form_row_visible(form_target_, combo_dictionary_, tag_family);
  set_form_row_visible(form_target_, squares_row_, !aruco_single);
  set_form_row_visible(form_target_, lengths_row_, true);
  set_form_row_visible(form_target_, area_row_, true);

  if (spin_square_length_ != nullptr) {
    spin_square_length_->setVisible(!aruco_single);
  }
  if (lab_square_length_ != nullptr) {
    lab_square_length_->setVisible(!aruco_single);
  }
  if (spin_marker_length_ != nullptr) {
    spin_marker_length_->setVisible(tag_family);
  }
  if (lab_marker_length_ != nullptr) {
    lab_marker_length_->setVisible(tag_family);
  }
  if (spin_max_tag_distance_ != nullptr) {
    spin_max_tag_distance_->setVisible(tag_family);
    if (QWidget *p = spin_max_tag_distance_->parentWidget()) {
      // 单元容器
      if (p != area_row_) {
        p->setVisible(tag_family);
      }
    }
  }

  // 正方形三面靶：只显示一个 n，Y 同步隐藏
  if (spin_squares_y_ != nullptr) {
    spin_squares_y_->setVisible(!square_board);
    if (QWidget *unit = spin_squares_y_->parentWidget()) {
      if (unit != squares_row_) {
        unit->setVisible(!square_board);
      }
    }
  }
  if (lab_squares_y_ != nullptr) {
    lab_squares_y_->setVisible(!square_board);
  }

  auto set_unit_lab = [](QLabel *lab, const QString &text) {
    if (lab != nullptr) {
      lab->setText(text);
    }
  };
  auto set_form_lab = [this](QWidget *field, const QString &text) {
    if (form_target_ == nullptr || field == nullptr) {
      return;
    }
    if (QWidget *lab = form_target_->labelForField(field)) {
      if (auto *ql = qobject_cast<QLabel *>(lab)) {
        ql->setText(text);
      }
    }
  };

  if (square_board) {
    set_form_lab(squares_row_, QStringLiteral("网格 n"));
    set_unit_lab(lab_squares_x_, QStringLiteral("n"));
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
    set_form_lab(squares_row_, QStringLiteral("网格"));
    set_unit_lab(lab_squares_x_, QStringLiteral("X"));
    set_unit_lab(lab_squares_y_, QStringLiteral("Y"));
  }

  if (chess) {
    set_unit_lab(lab_square_length_, QStringLiteral("方格"));
  } else if (charuco) {
    set_unit_lab(lab_square_length_, QStringLiteral("方格"));
    set_unit_lab(lab_marker_length_, QStringLiteral("标记"));
  } else if (aruco) {
    if (aruco_single) {
      set_unit_lab(lab_marker_length_, QStringLiteral("码边长"));
    } else {
      set_unit_lab(lab_squares_x_, QStringLiteral("列"));
      set_unit_lab(lab_squares_y_, QStringLiteral("行"));
      set_unit_lab(lab_square_length_, QStringLiteral("间距"));
      set_unit_lab(lab_marker_length_, QStringLiteral("标记"));
    }
  } else if (circles) {
    set_unit_lab(lab_squares_x_, QStringLiteral("列"));
    set_unit_lab(lab_squares_y_, QStringLiteral("行"));
    set_unit_lab(lab_square_length_, QStringLiteral("圆心距"));
  }

  set_form_row_visible(form_detect_, chess_flags_row_, chess);
  set_form_row_visible(
      form_detect_, combo_stereo_side_,
      calibrator_id_ == QStringLiteral("stereo_intrinsics") ||
          calibrator_id_ == QStringLiteral("stereo_extrinsics"));
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
  if (combo_stereo_side_ != nullptr && form_detect_ != nullptr) {
    const bool stereo = id == QStringLiteral("stereo_intrinsics") ||
                        id == QStringLiteral("stereo_extrinsics");
    set_form_row_visible(form_detect_, combo_stereo_side_, stereo);
    // 左右各至少约 3 帧；默认总数 6
    if (stereo && spin_min_views_ != nullptr && prev != id) {
      spin_min_views_->setValue(6);
    }
  }
  if (stereo_extrinsics_block_ != nullptr) {
    const bool se = id == QStringLiteral("stereo_extrinsics");
    stereo_extrinsics_block_->setVisible(se);
    if (se && prev != id) {
      if (edit_parent_frame_ != nullptr) {
        edit_parent_frame_->setText(QStringLiteral("left"));
      }
      if (edit_child_frame_ != nullptr) {
        edit_child_frame_->setText(QStringLiteral("right"));
      }
    }
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
                                       : "DICT_6X6_1000"},
      {"model",
       [&]() -> std::string {
         if (!combo_camera_model_) {
           return "brown_conrady";
         }
         const QVariant d = combo_camera_model_->currentData();
         if (d.isValid() && !d.toString().isEmpty()) {
           return d.toString().toStdString();
         }
         return combo_camera_model_->currentText().toStdString();
       }()},
      {"stereo_side",
       [&]() -> std::string {
         if (!combo_stereo_side_) {
           return "left";
         }
         const QVariant d = combo_stereo_side_->currentData();
         if (d.isValid() && !d.toString().isEmpty()) {
           return d.toString().toStdString();
         }
         return "left";
       }()},
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
      {"camera_info_topic",
       combo_camera_info_topic_ ? combo_camera_info_topic_->currentText().trimmed().toStdString()
                                : ""},
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
  if (edit_left_camera_yaml_ != nullptr &&
      !edit_left_camera_yaml_->text().trimmed().isEmpty()) {
    m["left_camera_yaml"] = edit_left_camera_yaml_->text().trimmed().toStdString();
  }
  if (edit_right_camera_yaml_ != nullptr &&
      !edit_right_camera_yaml_->text().trimmed().isEmpty()) {
    m["right_camera_yaml"] = edit_right_camera_yaml_->text().trimmed().toStdString();
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

/// \brief 应用项目默认坐标系
void LauncherConfigPanel::apply_project_frames(
    const QString &parent_frame,
    const QString &child_frame,
    const QString &base_frame,
    const QString &gripper_frame,
    const QString &image_frame,
    const QString &camera_link_frame) {
  if (edit_parent_frame_ && !parent_frame.isEmpty()) {
    edit_parent_frame_->setText(parent_frame);
  }
  if (edit_child_frame_ && !child_frame.isEmpty()) {
    edit_child_frame_->setText(child_frame);
  }
  if (edit_base_frame_ && !base_frame.isEmpty()) {
    edit_base_frame_->setText(base_frame);
  }
  if (edit_gripper_frame_ && !gripper_frame.isEmpty()) {
    edit_gripper_frame_->setText(gripper_frame);
  }
  if (edit_image_frame_ && !image_frame.isEmpty()) {
    edit_image_frame_->setText(image_frame);
  }
  if (edit_camera_link_frame_ && !camera_link_frame.isEmpty()) {
    edit_camera_link_frame_->setText(camera_link_frame);
  }
}

/// \brief 导出目录提示路径
QString LauncherConfigPanel::export_dir_hint() const {
  return edit_export_path_ ? edit_export_path_->text().trimmed() : QString();
}

}  // namespace gui
}  // namespace hs_calib
