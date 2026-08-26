#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"

#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/task_flow/task_flow.hpp"

#include <algorithm>
#include <cmath>
#include <map>

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
#include <QStackedWidget>
#include <QStringList>
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

/// \brief 表单左侧标签：用上下 stretch 保证相对高行（如 LineEdit）垂直居中
QWidget *make_vcenter_form_label(const QString &text, QWidget *parent = nullptr) {
  auto *wrap = new QWidget(parent);
  auto *v = new QVBoxLayout(wrap);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  auto *lab = new QLabel(text, wrap);
  lab->setObjectName(QStringLiteral("FormRowLabel"));
  lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  v->addStretch(1);
  v->addWidget(lab, 0, Qt::AlignRight | Qt::AlignVCenter);
  v->addStretch(1);
  return wrap;
}

/// \brief 更新表单行左侧文字（兼容标签外包一层居中 wrap）
void set_form_row_label(QFormLayout *form, QWidget *field, const QString &text) {
  if (form == nullptr || field == nullptr) {
    return;
  }
  QWidget *lab = form->labelForField(field);
  if (lab == nullptr) {
    return;
  }
  if (auto *ql = qobject_cast<QLabel *>(lab)) {
    ql->setText(text);
    return;
  }
  if (auto *inner = lab->findChild<QLabel *>(QStringLiteral("FormRowLabel"))) {
    inner->setText(text);
  } else if (auto *any = lab->findChild<QLabel *>()) {
    any->setText(text);
  }
}

/// \brief 添加表单行（空标题则只放字段）
void add_form_row(QFormLayout *form, const QString &text, QWidget *field) {
  if (form == nullptr || field == nullptr) {
    return;
  }
  if (text.isEmpty()) {
    form->addRow(field);
    return;
  }
  form->addRow(make_vcenter_form_label(text, form->parentWidget()), field);
}

/// \brief 主控件 + 动作按钮横排，垂直居中对齐
void pack_field_row(QHBoxLayout *lay, QWidget *primary, QWidget *action = nullptr) {
  if (lay == nullptr) {
    return;
  }
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(8);
  lay->setAlignment(Qt::AlignVCenter);
  if (primary != nullptr) {
    lay->addWidget(primary, 1, Qt::AlignVCenter);
  }
  if (action != nullptr) {
    action->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
    lay->addWidget(action, 0);
  }
}

/// \brief 数值控件统一最小宽度，避免并排时忽大忽小
void harden_compact_spin(QWidget *w, int min_w = 120) {
  if (w == nullptr) {
    return;
  }
  w->setMinimumWidth(min_w);
  w->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

/// 标定板长度类参数：UI 毫米，整数位/小数位各 4（0.0001～9999.9999）
constexpr double kLenMmMin = 0.0001;
constexpr double kLenMmMax = 9999.9999;
constexpr int kLenMmDecimals = 4;
constexpr double kLenMmStep = 0.1;

void configure_length_mm_spin(QDoubleSpinBox *s) {
  if (s == nullptr) {
    return;
  }
  s->setSuffix(QStringLiteral(" mm"));
  s->setDecimals(kLenMmDecimals);
  s->setRange(kLenMmMin, kLenMmMax);
  s->setSingleStep(kLenMmStep);
}

/// \brief 紧凑「标签 + 控件」单元（用于同行并排，不再套外层表单标签）
QWidget *make_field_unit(QWidget *parent, const QString &text, QWidget *field, QLabel **lab_out) {
  auto *unit = new QWidget(parent);
  auto *lay = new QHBoxLayout(unit);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(8);
  lay->setAlignment(Qt::AlignVCenter);
  auto *lab = new QLabel(text, unit);
  lab->setObjectName(QStringLiteral("Muted"));
  lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  lab->setMinimumWidth(36);
  harden_compact_spin(field);
  lay->addWidget(lab, 0, Qt::AlignVCenter);
  lay->addWidget(field, 1, Qt::AlignVCenter);
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

  // 图像源 + 内参源单独成页，不加入本面板主 layout
  data_source_panel_ = new QWidget;
  auto *ds_root = new QVBoxLayout(data_source_panel_);
  ds_root->setContentsMargins(0, 0, 0, 0);
  ds_root->setSpacing(10);

  // ---- 图像源 ----
  {
    auto *host = new QWidget(data_source_panel_);
    form_ros_ = new_form(host);
    auto *form = form_ros_;
    combo_source_mode_ = new QComboBox(host);
    combo_source_mode_->addItem(
        QStringLiteral("离线图像目录"), static_cast<int>(SourceMode::Offline));
    combo_source_mode_->addItem(
        QStringLiteral("在线 · ROS 图像话题"), static_cast<int>(SourceMode::RosTopic));
    combo_source_mode_->addItem(
        QStringLiteral("ROS Bag（rosbag2）"), static_cast<int>(SourceMode::RosBag));
    connect(
        combo_source_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int index) {
          refresh_source_mode_rows();
          emit source_mode_changed(index);
        });

    // 离线目录
    edit_image_dir_ = new QLineEdit(host);
    edit_image_dir_->setPlaceholderText(QStringLiteral("选择含 png/jpg 的图片目录…"));
    auto *browse_dir = new QPushButton(QStringLiteral("浏览…"), host);
    browse_dir->setObjectName(QStringLiteral("GhostButton"));
    connect(browse_dir, &QPushButton::clicked, this, &LauncherConfigPanel::browse_image_dir_clicked);
    offline_row_ = new QWidget(host);
    auto *dir_lay = new QHBoxLayout(offline_row_);
    pack_field_row(dir_lay, edit_image_dir_, browse_dir);

    // 在线话题
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
    pack_field_row(topic_lay, combo_image_topic_, btn_refresh_topics_);

    combo_left_image_topic_ = new QComboBox(host);
    combo_left_image_topic_->setEditable(true);
    combo_left_image_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_left_image_topic_->lineEdit()->setPlaceholderText(
        QStringLiteral("/cam_left/color/image_raw"));
    combo_right_image_topic_ = new QComboBox(host);
    combo_right_image_topic_->setEditable(true);
    combo_right_image_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_right_image_topic_->lineEdit()->setPlaceholderText(
        QStringLiteral("/cam_right/color/image_raw"));
    connect(
        combo_left_image_topic_, &QComboBox::currentTextChanged, this,
        &LauncherConfigPanel::image_topic_changed);
    connect(
        combo_right_image_topic_, &QComboBox::currentTextChanged, this,
        &LauncherConfigPanel::image_topic_changed);
    stereo_topic_row_ = new QWidget(host);
    auto *stereo_lay = new QVBoxLayout(stereo_topic_row_);
    stereo_lay->setContentsMargins(0, 0, 0, 0);
    stereo_lay->setSpacing(6);
    auto *left_row = new QWidget(host);
    auto *left_lay = new QHBoxLayout(left_row);
    left_lay->setContentsMargins(0, 0, 0, 0);
    auto *left_lbl = new QLabel(QStringLiteral("左目"), host);
    left_lbl->setObjectName(QStringLiteral("Muted"));
    left_lay->addWidget(left_lbl);
    left_lay->addWidget(combo_left_image_topic_, 1);
    auto *right_row = new QWidget(host);
    auto *right_lay = new QHBoxLayout(right_row);
    right_lay->setContentsMargins(0, 0, 0, 0);
    auto *right_lbl = new QLabel(QStringLiteral("右目"), host);
    right_lbl->setObjectName(QStringLiteral("Muted"));
    right_lay->addWidget(right_lbl);
    right_lay->addWidget(combo_right_image_topic_, 1);
    auto *stereo_refresh_row = new QWidget(host);
    auto *stereo_refresh_lay = new QHBoxLayout(stereo_refresh_row);
    stereo_refresh_lay->setContentsMargins(0, 0, 0, 0);
    stereo_refresh_lay->addStretch(1);
    auto *btn_refresh_stereo = new QPushButton(QStringLiteral("刷新"), host);
    btn_refresh_stereo->setObjectName(QStringLiteral("GhostButton"));
    connect(
        btn_refresh_stereo, &QPushButton::clicked, this,
        &LauncherConfigPanel::refresh_topics_clicked);
    stereo_refresh_lay->addWidget(btn_refresh_stereo);
    stereo_lay->addWidget(left_row);
    stereo_lay->addWidget(right_row);
    stereo_lay->addWidget(stereo_refresh_row);

    chk_use_rectified_ = new QCheckBox(QStringLiteral("图像已去畸变"), host);
    chk_use_rectified_->setChecked(false);

    // rosbag2
    edit_bag_path_ = new QLineEdit(host);
    edit_bag_path_->setPlaceholderText(
        QStringLiteral("选择 rosbag2 目录（含 metadata.yaml）…"));
    auto *browse_bag = new QPushButton(QStringLiteral("浏览…"), host);
    browse_bag->setObjectName(QStringLiteral("GhostButton"));
    connect(browse_bag, &QPushButton::clicked, this, &LauncherConfigPanel::browse_bag_clicked);
    bag_path_row_ = new QWidget(host);
    auto *bag_path_lay = new QHBoxLayout(bag_path_row_);
    pack_field_row(bag_path_lay, edit_bag_path_, browse_bag);

    combo_bag_topic_ = new QComboBox(host);
    combo_bag_topic_->setEditable(true);
    combo_bag_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_bag_topic_->lineEdit()->setPlaceholderText(QStringLiteral("/camera/image_raw"));
    btn_load_bag_ = new QPushButton(QStringLiteral("加载帧"), host);
    btn_load_bag_->setObjectName(QStringLiteral("GhostButton"));
    connect(btn_load_bag_, &QPushButton::clicked, this, &LauncherConfigPanel::load_bag_clicked);
    bag_topic_row_ = new QWidget(host);
    auto *bag_topic_lay = new QHBoxLayout(bag_topic_row_);
    pack_field_row(bag_topic_lay, combo_bag_topic_, btn_load_bag_);

    spin_bag_max_frames_ = make_int(host, 500, 10, 2000);
    spin_bag_max_frames_->setSuffix(QStringLiteral(" 帧"));

    add_form_row(form, QStringLiteral("模式"), combo_source_mode_);
    add_form_row(form, QStringLiteral("图片目录"), offline_row_);
    add_form_row(form, QStringLiteral("图像话题"), topic_row_);
    add_form_row(form, QStringLiteral("双目话题"), stereo_topic_row_);
    add_form_row(form, QStringLiteral("Bag 路径"), bag_path_row_);
    add_form_row(form, QStringLiteral("Bag 话题"), bag_topic_row_);
    add_form_row(form, QStringLiteral("最多导出"), spin_bag_max_frames_);
    add_form_row(form, QStringLiteral("图像标记"), chk_use_rectified_);
    image_source_group_ = make_group(QStringLiteral("图像源"), host);
    ds_root->addWidget(image_source_group_);
    refresh_source_mode_rows();
  }

  // ---- 内参源（可选，与图像源独立） ----
  {
    auto *host = new QWidget(data_source_panel_);
    form_intrinsics_ = new_form(host);
    auto *form = form_intrinsics_;

    combo_intrinsics_source_ = new QComboBox(host);
    combo_intrinsics_source_->addItem(
        QStringLiteral("不使用（内参标定推荐）"), 0);
    combo_intrinsics_source_->addItem(
        QStringLiteral("ROS CameraInfo 话题"), 1);
    combo_intrinsics_source_->addItem(
        QStringLiteral("相机内参 YAML 文件"), 2);
    connect(
        combo_intrinsics_source_, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) {
          refresh_intrinsics_source_rows();
          emit intrinsics_source_changed(intrinsics_source_mode());
        });

    combo_camera_info_topic_ = new QComboBox(host);
    combo_camera_info_topic_->setEditable(true);
    combo_camera_info_topic_->setInsertPolicy(QComboBox::NoInsert);
    combo_camera_info_topic_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    combo_camera_info_topic_->setMinimumWidth(200);
    combo_camera_info_topic_->lineEdit()->setPlaceholderText(
        QStringLiteral("可选，例如 /camera/camera_info"));
    combo_camera_info_topic_->setEditText(QString());
    connect(
        combo_camera_info_topic_, &QComboBox::currentTextChanged, this,
        &LauncherConfigPanel::camera_info_topic_changed);
    btn_refresh_camera_info_ = new QPushButton(QStringLiteral("刷新"), host);
    btn_refresh_camera_info_->setObjectName(QStringLiteral("GhostButton"));
    connect(
        btn_refresh_camera_info_, &QPushButton::clicked, this,
        &LauncherConfigPanel::refresh_topics_clicked);
    camera_info_row_ = new QWidget(host);
    auto *info_lay = new QHBoxLayout(camera_info_row_);
    pack_field_row(info_lay, combo_camera_info_topic_, btn_refresh_camera_info_);

    edit_intrinsics_yaml_ = new QLineEdit(host);
    edit_intrinsics_yaml_->setPlaceholderText(
        QStringLiteral("可选：已有内参 YAML（手眼/检测轴等）"));
    auto *browse_intr = new QPushButton(QStringLiteral("浏览…"), host);
    browse_intr->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_intr, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_intrinsics_yaml_clicked);
    intrinsics_yaml_row_ = new QWidget(host);
    auto *iy_lay = new QHBoxLayout(intrinsics_yaml_row_);
    pack_field_row(iy_lay, edit_intrinsics_yaml_, browse_intr);

    auto *hint = new QLabel(
        QStringLiteral("说明：标定相机内参时一般无需内参源；仅在需要初值、"
                       "画坐标轴/PnP，或手眼标定等场景再开启。"),
        host);
    hint->setObjectName(QStringLiteral("Muted"));
    hint->setWordWrap(true);

    add_form_row(form, QStringLiteral("模式"), combo_intrinsics_source_);
    add_form_row(form, QStringLiteral("camera_info"), camera_info_row_);
    add_form_row(form, QStringLiteral("内参 YAML"), intrinsics_yaml_row_);
    add_form_row(form, QString(), hint);
    intrinsics_source_block_ = make_group(QStringLiteral("内参源"), host);
    ds_root->addWidget(intrinsics_source_block_);
    refresh_intrinsics_source_rows();
  }

  // ---- 双目左右内参（双目外参任务，归入数据源·内参）----
  {
    stereo_extrinsics_block_ = new QWidget(data_source_panel_);
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
    pack_field_row(left_lay, edit_left_camera_yaml_, browse_l);

    edit_right_camera_yaml_ = new QLineEdit(stereo_extrinsics_block_);
    edit_right_camera_yaml_->setPlaceholderText(QStringLiteral("camera_right.yaml"));
    auto *browse_r = new QPushButton(QStringLiteral("浏览…"), stereo_extrinsics_block_);
    browse_r->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_r, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_right_camera_yaml_clicked);
    auto *right_row = new QWidget(stereo_extrinsics_block_);
    auto *right_lay = new QHBoxLayout(right_row);
    pack_field_row(right_lay, edit_right_camera_yaml_, browse_r);

    add_form_row(form, QStringLiteral("左目内参"), left_row);
    add_form_row(form, QStringLiteral("右目内参"), right_row);
    stereo_extrinsics_group_ =
        make_group(QStringLiteral("双目内参 YAML"), stereo_extrinsics_block_);
    ds_root->addWidget(stereo_extrinsics_group_);
  }

  // ---- TF / 位姿源（手眼等；内参标定不显示）----
  {
    auto *host = new QWidget(data_source_panel_);
    auto *form = new_form(host);

    edit_image_frame_ = new QLineEdit(QStringLiteral("camera_optical_frame"), host);
    edit_camera_link_frame_ = new QLineEdit(QStringLiteral("camera_link"), host);
    auto *cam_frame_row = new QWidget(host);
    auto *cam_frame_lay = new QHBoxLayout(cam_frame_row);
    cam_frame_lay->setContentsMargins(0, 0, 0, 0);
    cam_frame_lay->setSpacing(10);
    cam_frame_lay->setAlignment(Qt::AlignVCenter);
    cam_frame_lay->addWidget(
        make_field_unit(cam_frame_row, QStringLiteral("optical"), edit_image_frame_, nullptr), 1,
        Qt::AlignVCenter);
    cam_frame_lay->addWidget(
        make_field_unit(
            cam_frame_row, QStringLiteral("link"), edit_camera_link_frame_, nullptr),
        1, Qt::AlignVCenter);

    edit_base_frame_ = new QLineEdit(QStringLiteral("base"), host);
    edit_gripper_frame_ = new QLineEdit(QStringLiteral("tool0"), host);
    auto *robot_frame_row = new QWidget(host);
    auto *robot_frame_lay = new QHBoxLayout(robot_frame_row);
    robot_frame_lay->setContentsMargins(0, 0, 0, 0);
    robot_frame_lay->setSpacing(10);
    robot_frame_lay->setAlignment(Qt::AlignVCenter);
    robot_frame_lay->addWidget(
        make_field_unit(robot_frame_row, QStringLiteral("base"), edit_base_frame_, nullptr), 1,
        Qt::AlignVCenter);
    robot_frame_lay->addWidget(
        make_field_unit(
            robot_frame_row, QStringLiteral("gripper"), edit_gripper_frame_, nullptr),
        1, Qt::AlignVCenter);

    combo_pose_source_ = new QComboBox(host);
    combo_pose_source_->addItem(QStringLiteral("CSV"), static_cast<int>(PoseSource::Csv));
    combo_pose_source_->addItem(QStringLiteral("TF"), static_cast<int>(PoseSource::Tf));
    connect(
        combo_pose_source_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        &LauncherConfigPanel::pose_source_changed);

    edit_pose_csv_ = new QLineEdit(host);
    edit_pose_csv_->setPlaceholderText(QStringLiteral("image,pose CSV…"));
    auto *browse_csv = new QPushButton(QStringLiteral("浏览…"), host);
    browse_csv->setObjectName(QStringLiteral("GhostButton"));
    connect(
        browse_csv, &QPushButton::clicked, this,
        &LauncherConfigPanel::browse_pose_csv_clicked);
    auto *csv_row = new QWidget(host);
    auto *csv_lay = new QHBoxLayout(csv_row);
    pack_field_row(csv_lay, edit_pose_csv_, browse_csv);

    edit_parent_frame_ = new QLineEdit(QStringLiteral("camera_link"), host);
    edit_child_frame_ = new QLineEdit(QStringLiteral("camera_optical_frame"), host);
    export_tf_row_ = new QWidget(host);
    auto *tf_lay = new QHBoxLayout(export_tf_row_);
    tf_lay->setContentsMargins(0, 0, 0, 0);
    tf_lay->setSpacing(10);
    tf_lay->setAlignment(Qt::AlignVCenter);
    tf_lay->addWidget(
        make_field_unit(export_tf_row_, QStringLiteral("parent"), edit_parent_frame_, nullptr),
        1, Qt::AlignVCenter);
    tf_lay->addWidget(
        make_field_unit(export_tf_row_, QStringLiteral("child"), edit_child_frame_, nullptr), 1,
        Qt::AlignVCenter);

    // 兼容旧手眼 YAML 字段（优先用上方「内参源」）
    edit_camera_yaml_ = new QLineEdit(host);
    edit_camera_yaml_->setPlaceholderText(QStringLiteral("可选回退：camera_intrinsics.yaml"));
    edit_camera_yaml_->setVisible(false);

    add_form_row(form, QStringLiteral("相机 frames"), cam_frame_row);
    add_form_row(form, QStringLiteral("机器人 frames"), robot_frame_row);
    add_form_row(form, QStringLiteral("位姿源"), combo_pose_source_);
    add_form_row(form, QStringLiteral("pose_csv"), csv_row);
    add_form_row(form, QStringLiteral("结果 frames"), export_tf_row_);
    tf_source_block_ = make_group(QStringLiteral("TF / 位姿"), host);
    ds_root->addWidget(tf_source_block_);
  }

  // ========== 标定设置页：三块 ==========

  // ① 标定板参数
  {
    auto *host = new QWidget(this);
    form_target_ = new_form(host);
    auto *form = form_target_;

    edit_config_path_ = new QLineEdit(host);
    edit_config_path_->setReadOnly(true);
    edit_config_path_->setPlaceholderText(
        QStringLiteral("包内 YAML，例如 eye_in_hand.yaml"));
    auto *reload = new QPushButton(QStringLiteral("重新加载"), host);
    reload->setObjectName(QStringLiteral("GhostButton"));
    reload->setToolTip(
        QStringLiteral("从当前标定类型对应的 YAML 读取话题、坐标系、靶标等"));
    connect(reload, &QPushButton::clicked, this, &LauncherConfigPanel::reload_yaml_clicked);
    auto *cfg_row = new QWidget(host);
    auto *cfg_lay = new QHBoxLayout(cfg_row);
    pack_field_row(cfg_lay, edit_config_path_, reload);

    combo_target_type_ = new QComboBox(host);
    refresh_target_type_options();
    combo_dictionary_ = new QComboBox(host);
    combo_dictionary_->addItems(
        {QStringLiteral("DICT_6X6_1000"), QStringLiteral("DICT_6X6_250"),
         QStringLiteral("DICT_4X4_250"), QStringLiteral("DICT_4X4_50"),
         QStringLiteral("DICT_4X4_100"), QStringLiteral("DICT_5X5_50"),
         QStringLiteral("DICT_5X5_100"), QStringLiteral("DICT_5X5_250"),
         QStringLiteral("DICT_6X6_50"), QStringLiteral("DICT_7X7_1000"),
         QStringLiteral("DICT_ARUCO_ORIGINAL"),
         QStringLiteral("DICT_APRILTAG_36h11"),
         QStringLiteral("DICT_APRILTAG_36h10"),
         QStringLiteral("DICT_APRILTAG_25h9"),
         QStringLiteral("DICT_APRILTAG_16h5")});
    spin_squares_x_ = make_int(host, 9, 2, 40);
    spin_squares_y_ = make_int(host, 6, 2, 40);
    // UI 用 mm（4+4 位）；square_length()/marker_length() 对外仍返回米
    spin_square_length_ =
        make_dbl(host, 25.0, kLenMmMin, kLenMmMax, kLenMmDecimals, kLenMmStep);
    configure_length_mm_spin(spin_square_length_);
    spin_marker_length_ =
        make_dbl(host, 18.0, kLenMmMin, kLenMmMax, kLenMmDecimals, kLenMmStep);
    configure_length_mm_spin(spin_marker_length_);
    spin_min_board_area_ = make_dbl(host, 0.04, 0.001, 0.9, 3, 0.01);
    spin_max_board_area_ = make_dbl(host, 0.45, 0.01, 0.95, 3, 0.01);
    spin_max_tag_distance_ =
        make_dbl(host, 5000.0, kLenMmMin, kLenMmMax, kLenMmDecimals, kLenMmStep);
    configure_length_mm_spin(spin_max_tag_distance_);

    squares_row_ = new QWidget(host);
    auto *sq_lay = new QHBoxLayout(squares_row_);
    sq_lay->setContentsMargins(0, 0, 0, 0);
    sq_lay->setSpacing(16);
    sq_lay->setAlignment(Qt::AlignVCenter);
    sq_lay->addWidget(
        make_field_unit(squares_row_, QStringLiteral("列"), spin_squares_x_, &lab_squares_x_),
        1);
    sq_lay->addWidget(
        make_field_unit(squares_row_, QStringLiteral("行"), spin_squares_y_, &lab_squares_y_),
        1);

    area_row_ = new QWidget(host);
    auto *area_lay = new QHBoxLayout(area_row_);
    area_lay->setContentsMargins(0, 0, 0, 0);
    area_lay->setSpacing(16);
    area_lay->setAlignment(Qt::AlignVCenter);
    area_lay->addWidget(
        make_field_unit(area_row_, QStringLiteral("最小"), spin_min_board_area_, nullptr), 1);
    area_lay->addWidget(
        make_field_unit(area_row_, QStringLiteral("最大"), spin_max_board_area_, nullptr), 1);

    chk_cb_adaptive_ = new QCheckBox(QStringLiteral("自适应阈值"), host);
    chk_cb_adaptive_->setChecked(false);
    chk_cb_normalize_ = new QCheckBox(QStringLiteral("归一化"), host);
    chk_cb_normalize_->setChecked(false);
    chk_cb_filter_quads_ = new QCheckBox(QStringLiteral("过滤四边形"), host);
    chk_cb_fast_check_ = new QCheckBox(QStringLiteral("快速预检"), host);
    chk_cb_fast_check_->setChecked(false);
    spin_subpix_win_ = make_int(host, 11, 3, 31);
    harden_compact_spin(spin_subpix_win_, 90);
    chess_flags_row_ = new QWidget(host);
    auto *flags_lay = new QHBoxLayout(chess_flags_row_);
    flags_lay->setContentsMargins(0, 0, 0, 0);
    flags_lay->setSpacing(10);
    flags_lay->setAlignment(Qt::AlignVCenter);
    flags_lay->addWidget(chk_cb_adaptive_);
    flags_lay->addWidget(chk_cb_normalize_);
    flags_lay->addWidget(chk_cb_filter_quads_);
    flags_lay->addWidget(chk_cb_fast_check_);
    flags_lay->addStretch(1);
    flags_lay->addWidget(
        make_field_unit(chess_flags_row_, QStringLiteral("亚像素窗"), spin_subpix_win_, nullptr));

    harden_compact_spin(spin_square_length_, 168);
    harden_compact_spin(spin_marker_length_, 168);
    harden_compact_spin(spin_max_tag_distance_, 168);

    add_form_row(form, QStringLiteral("靶标 YAML"), cfg_row);
    add_form_row(form, QStringLiteral("类型"), combo_target_type_);
    add_form_row(form, QStringLiteral("字典"), combo_dictionary_);
    add_form_row(form, QStringLiteral("网格"), squares_row_);
    add_form_row(form, QStringLiteral("方格边长"), spin_square_length_);
    add_form_row(form, QStringLiteral("码边长"), spin_marker_length_);
    add_form_row(form, QStringLiteral("成像占比"), area_row_);
    add_form_row(form, QStringLiteral("最大码距"), spin_max_tag_distance_);
    add_form_row(form, QStringLiteral("棋盘检测"), chess_flags_row_);
    connect(
        spin_squares_x_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int v) {
          if (combo_target_type_ == nullptr || spin_squares_y_ == nullptr) {
            return;
          }
          if (target_type_id().startsWith(QStringLiteral("trihedral")) &&
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
          const QString tid = target_type_id();
          if (tid == QStringLiteral("aruco")) {
            if (combo_dictionary_ != nullptr) {
              const int didx =
                  combo_dictionary_->findText(QStringLiteral("DICT_6X6_1000"));
              if (didx >= 0) {
                combo_dictionary_->setCurrentIndex(didx);
              }
            }
            if (spin_marker_length_ != nullptr && spin_marker_length_->value() < 50.0) {
              spin_marker_length_->setValue(100.0);
            }
          }
          if (tid == QStringLiteral("charuco") ||
              tid == QStringLiteral("trihedral_charuco")) {
            if (combo_dictionary_ != nullptr) {
              const int didx =
                  combo_dictionary_->findText(QStringLiteral("DICT_4X4_50"));
              if (didx >= 0) {
                combo_dictionary_->setCurrentIndex(didx);
              }
            }
            if (spin_squares_x_ != nullptr) {
              spin_squares_x_->setValue(5);
            }
            if (spin_squares_y_ != nullptr) {
              spin_squares_y_->setValue(7);
            }
            if (spin_square_length_ != nullptr) {
              spin_square_length_->setValue(40.0);
            }
            if (spin_marker_length_ != nullptr) {
              spin_marker_length_->setValue(30.0);
            }
          }
          if (tid == QStringLiteral("aprilgrid")) {
            if (spin_squares_x_ != nullptr) {
              spin_squares_x_->setValue(6);
            }
            if (spin_squares_y_ != nullptr) {
              spin_squares_y_->setValue(6);
            }
            if (spin_marker_length_ != nullptr) {
              spin_marker_length_->setValue(88.0);
            }
            if (spin_square_length_ != nullptr) {
              // UI：标签间距 = 空白边距（mm）；算法：tagSpacing = 间距/边长
              spin_square_length_->setValue(0.3 * 88.0);
            }
            if (combo_dictionary_ != nullptr) {
              const int didx =
                  combo_dictionary_->findText(QStringLiteral("DICT_APRILTAG_36h11"));
              if (didx >= 0) {
                combo_dictionary_->setCurrentIndex(didx);
              }
            }
          }
          if (tid == QStringLiteral("circles_symmetric")) {
            if (spin_squares_x_ != nullptr) {
              spin_squares_x_->setValue(7);
            }
            if (spin_squares_y_ != nullptr) {
              spin_squares_y_->setValue(7);
            }
            if (spin_square_length_ != nullptr) {
              spin_square_length_->setValue(40.0);
              configure_length_mm_spin(spin_square_length_);
            }
            if (spin_marker_length_ != nullptr) {
              spin_marker_length_->setValue(20.0);  // 直径
            }
          }
          if (tid == QStringLiteral("circles_asymmetric")) {
            if (spin_squares_x_ != nullptr) {
              spin_squares_x_->setValue(3);  // 每行圆点数
            }
            if (spin_squares_y_ != nullptr) {
              spin_squares_y_->setValue(6);  // 行数（交错）
            }
            if (spin_square_length_ != nullptr) {
              // UI：calib.io Diagonal Spacing；内部再 /√2 → OpenCV squareSize
              spin_square_length_->setValue(12.0);
              configure_length_mm_spin(spin_square_length_);
            }
            if (spin_marker_length_ != nullptr) {
              spin_marker_length_->setValue(5.0);  // 直径
            }
          }
        });
    board_params_block_ = make_group(QStringLiteral("① 标定板参数"), host);
    root->addWidget(board_params_block_);
  }

  // ② 相机模型 / 求解
  {
    auto *host = new QWidget(this);
    form_solver_ = new_form(host);
    auto *form = form_solver_;
    combo_camera_model_ = new QComboBox(host);
    combo_camera_model_->addItem(
        QStringLiteral("Brown-Conrady (pinhole)"), QStringLiteral("brown_conrady"));
    combo_camera_model_->addItem(
        QStringLiteral("Kannala-Brandt (fisheye)"), QStringLiteral("kannala_brandt"));
    combo_camera_model_->addItem(
        QStringLiteral("CMei (omnidir)"), QStringLiteral("cmei"));
    combo_camera_model_->setCurrentIndex(0);
    combo_intrinsics_mode_ = new QComboBox(host);
    combo_intrinsics_mode_->addItem(QStringLiteral("经典标定（OpenCV）"), QStringLiteral("classic"));
    combo_intrinsics_mode_->addItem(QStringLiteral("Tier4 流水线"), QStringLiteral("tier4"));
    combo_intrinsics_mode_->setToolTip(
        QStringLiteral("经典：标定设置页的相机模型与 OpenCV flags；"
                       "Tier4：训练/评估分流、RANSAC 与专用参数（仅 Brown 模型）。"));
    combo_tier4_profile_ = new QComboBox(host);
    combo_tier4_profile_->addItem(QStringLiteral("General"), QStringLiteral("general"));
    combo_tier4_profile_->addItem(QStringLiteral("C1（严格采集）"), QStringLiteral("c1"));
    combo_tier4_profile_->addItem(QStringLiteral("Ceres"), QStringLiteral("ceres"));
    combo_tier4_profile_->addItem(QStringLiteral("C2（Ceres+FOV）"), QStringLiteral("c2"));
    combo_tier4_profile_->setToolTip(
        QStringLiteral("Tier4 求解预设；切换会同步更新下方 OpenCV flags 默认值。"));
    connect(
        combo_intrinsics_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) { update_intrinsics_mode_rows(); });
    connect(
        combo_tier4_profile_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) {
          if (!intrinsics_mode_is_tier4() || combo_tier4_profile_ == nullptr) {
            return;
          }
          apply_tier4_profile_to_solver_flags(combo_tier4_profile_->currentData().toString());
        });
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

    solver_flags_row1_ = new QWidget(host);
    auto *f1 = new QHBoxLayout(solver_flags_row1_);
    f1->setContentsMargins(0, 0, 0, 0);
    f1->setSpacing(8);
    for (auto *c : {chk_fix_principal_, chk_fix_aspect_, chk_zero_tangent_,
                    chk_fix_k1_, chk_fix_k2_}) {
      f1->addWidget(c);
    }
    f1->addStretch(1);

    solver_flags_row2_ = new QWidget(host);
    auto *f2 = new QHBoxLayout(solver_flags_row2_);
    f2->setContentsMargins(0, 0, 0, 0);
    f2->setSpacing(8);
    for (auto *c :
         {chk_fix_k3_, chk_rational_model_, chk_thin_prism_, chk_use_intrinsic_guess_}) {
      f2->addWidget(c);
    }
    f2->addStretch(1);

    combo_handeye_method_ = new QComboBox(host);
    combo_handeye_method_->addItems(
        {QStringLiteral("tsai"), QStringLiteral("park"), QStringLiteral("horaud"),
         QStringLiteral("andreff"), QStringLiteral("daniilidis")});
    combo_stereo_side_ = new QComboBox(host);
    combo_stereo_side_->addItem(QStringLiteral("left"), QStringLiteral("left"));
    combo_stereo_side_->addItem(QStringLiteral("right"), QStringLiteral("right"));
    chk_stereo_joint_refine_ = new QCheckBox(QStringLiteral("联合精化内参 (stereoCalibrate)"), host);
    chk_stereo_joint_refine_->setChecked(false);
    chk_stereo_joint_refine_->setToolTip(
        QStringLiteral("开启后 stereoCalibrate 同时优化左右内参；默认固定已标定内参"));
    edit_export_path_ = new QLineEdit(host);
    edit_export_path_->setPlaceholderText(QStringLiteral("~/calib_out"));

    add_form_row(form, QStringLiteral("标定模式"), combo_intrinsics_mode_);
    add_form_row(form, QStringLiteral("Tier4 预设"), combo_tier4_profile_);
    add_form_row(form, QStringLiteral("相机模型"), combo_camera_model_);
    add_form_row(form, QStringLiteral("标定 flags"), solver_flags_row1_);
    add_form_row(form, QString(), solver_flags_row2_);
    add_form_row(form, QStringLiteral("手眼算法"), combo_handeye_method_);
    add_form_row(form, QStringLiteral("双目采集侧"), combo_stereo_side_);
    add_form_row(form, QStringLiteral("立体联合精化"), chk_stereo_joint_refine_);
    add_form_row(form, QStringLiteral("导出目录"), edit_export_path_);
    solver_intrinsics_block_ = make_group(QStringLiteral("② 相机模型与求解"), host);
    root->addWidget(solver_intrinsics_block_);
  }

  // ③ 采集 / 求解标准：每项一行，避免「外层标签 + 内嵌标签」叠在一起
  {
    auto *host = new QWidget(this);
    form_detect_ = new_form(host);
    auto *form = form_detect_;
    spin_min_views_ = make_int(host, 12, 1, 200);
    spin_min_confidence_ = make_dbl(host, 0.40, 0.1, 1.0, 2, 0.05);
    spin_min_diversity_ = make_dbl(host, 0.08, 0.03, 2.0, 2, 0.02);
    spin_auto_cooldown_ms_ = make_int(host, 900, 100, 10000);
    spin_auto_cooldown_ms_->setSuffix(QStringLiteral(" ms"));
    chk_auto_capture_default_ = new QCheckBox(
        QStringLiteral("进入工作台时默认开启自动采集"), host);
    chk_auto_capture_default_->setChecked(true);

    harden_compact_spin(spin_min_views_, 140);
    harden_compact_spin(spin_min_confidence_, 140);
    harden_compact_spin(spin_min_diversity_, 140);
    harden_compact_spin(spin_auto_cooldown_ms_, 160);
    spin_min_views_->setToolTip(QStringLiteral("求解前建议至少采集的有效帧数"));
    spin_min_confidence_->setToolTip(QStringLiteral("检测置信度低于此值时不计入有效帧"));
    spin_min_diversity_->setToolTip(QStringLiteral("位姿多样性阈值，过低表示视角过于集中"));
    spin_auto_cooldown_ms_->setToolTip(QStringLiteral("自动采集两次之间的最短间隔"));

    add_form_row(form, QStringLiteral("最少帧数"), spin_min_views_);
    add_form_row(form, QStringLiteral("最低置信度"), spin_min_confidence_);
    add_form_row(form, QStringLiteral("位姿覆盖"), spin_min_diversity_);
    add_form_row(form, QStringLiteral("自动冷却"), spin_auto_cooldown_ms_);
    add_form_row(form, QStringLiteral("默认行为"), chk_auto_capture_default_);
    combo_stats_backend_ = new QComboBox(host);
    combo_stats_backend_->addItem(QStringLiteral("Qt 轻量图"), QStringLiteral("qt"));
    combo_stats_backend_->addItem(QStringLiteral("matplotlib"), QStringLiteral("matplotlib"));
    combo_stats_backend_->setToolTip(
        QStringLiteral("内参采集统计图后端（gui.stats_backend）"));
    add_form_row(form, QStringLiteral("统计图后端"), combo_stats_backend_);
    capture_criteria_block_ = make_group(QStringLiteral("③ 采集与求解标准"), host);
    root->addWidget(capture_criteria_block_);
  }

  finalize_task_flow_layout();
  apply_task_flow_layout();
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
  form->setHorizontalSpacing(12);
  form->setVerticalSpacing(10);
  form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  form->setRowWrapPolicy(QFormLayout::DontWrapRows);
  return form;
}

/// \brief 按离线 / ROS 话题 / Bag 显隐图像源表单行
void LauncherConfigPanel::refresh_source_mode_rows() {
  const int mode =
      combo_source_mode_ != nullptr ? combo_source_mode_->currentData().toInt()
                                    : static_cast<int>(SourceMode::Offline);
  const bool offline = mode == static_cast<int>(SourceMode::Offline);
  const bool ros = mode == static_cast<int>(SourceMode::RosTopic);
  const bool bag = mode == static_cast<int>(SourceMode::RosBag);

  set_form_row_visible(form_ros_, offline_row_, offline);
  set_form_row_visible(form_ros_, topic_row_, ros);
  set_form_row_visible(form_ros_, bag_path_row_, bag);
  set_form_row_visible(form_ros_, bag_topic_row_, bag);
  set_form_row_visible(form_ros_, spin_bag_max_frames_, bag);
  // 去畸变标记与图像内容相关，各模式均可选
  set_form_row_visible(form_ros_, chk_use_rectified_, true);
  refresh_image_topic_rows();
}

/// \brief 按内参源模式显隐 CameraInfo / YAML 行
void LauncherConfigPanel::refresh_intrinsics_source_rows() {
  const int mode = intrinsics_source_mode();
  const bool use_info = mode == 1;
  const bool use_yaml = mode == 2;
  set_form_row_visible(form_intrinsics_, camera_info_row_, use_info);
  set_form_row_visible(form_intrinsics_, intrinsics_yaml_row_, use_yaml);
}

/// \brief 内参源模式：0=无 1=CameraInfo 2=YAML
int LauncherConfigPanel::intrinsics_source_mode() const {
  if (combo_intrinsics_source_ == nullptr) {
    return 0;
  }
  return combo_intrinsics_source_->currentData().toInt();
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

namespace {

/// \brief 靶标 ID → 下拉显示名
QString target_type_label(const QString &id) {
  if (id == QStringLiteral("chessboard")) {
    return QStringLiteral("棋盘格");
  }
  if (id == QStringLiteral("charuco")) {
    return QStringLiteral("ChArUco");
  }
  if (id == QStringLiteral("aruco")) {
    return QStringLiteral("单码 ArUco");
  }
  if (id == QStringLiteral("aruco_grid")) {
    return QStringLiteral("ArUco 网格");
  }
  if (id == QStringLiteral("aprilgrid")) {
    return QStringLiteral("AprilGrid");
  }
  if (id == QStringLiteral("circles_symmetric")) {
    return QStringLiteral("对称圆");
  }
  if (id == QStringLiteral("circles_asymmetric")) {
    return QStringLiteral("非对称圆");
  }
  if (id == QStringLiteral("trihedral_chess")) {
    return QStringLiteral("三面棋盘");
  }
  if (id == QStringLiteral("trihedral_charuco")) {
    return QStringLiteral("三面 ChArUco");
  }
  if (id == QStringLiteral("trihedral_aruco")) {
    return QStringLiteral("三面 ArUco");
  }
  return id;
}

/// \brief 当前任务允许的靶标 ID（与各 Calibrator::supported_targets 对齐）
QStringList allowed_target_ids(const QString &calibrator_id) {
  if (calibrator_id == QStringLiteral("trihedral_oneshot")) {
    return {
        QStringLiteral("trihedral_chess"),
        QStringLiteral("trihedral_charuco"),
        QStringLiteral("trihedral_aruco")};
  }
  if (calibrator_id == QStringLiteral("eye_in_hand") ||
      calibrator_id == QStringLiteral("eye_to_hand")) {
    return {
        QStringLiteral("chessboard"), QStringLiteral("charuco"),
        QStringLiteral("aruco"), QStringLiteral("aruco_grid"),
        QStringLiteral("aprilgrid")};
  }
  // cam_intrinsics / stereo_intrinsics / stereo_extrinsics / 默认：平面靶
  return {
      QStringLiteral("chessboard"),
      QStringLiteral("charuco"),
      QStringLiteral("aruco"),
      QStringLiteral("aruco_grid"),
      QStringLiteral("aprilgrid"),
      QStringLiteral("circles_symmetric"),
      QStringLiteral("circles_asymmetric")};
}

QString default_target_for_calibrator(const QString &calibrator_id) {
  if (calibrator_id == QStringLiteral("trihedral_oneshot")) {
    return QStringLiteral("trihedral_charuco");
  }
  return QStringLiteral("chessboard");
}

}  // namespace

/// \brief 按标定任务筛选「类型」下拉，避免平面/三面混在一起
void LauncherConfigPanel::refresh_target_type_options() {
  if (combo_target_type_ == nullptr) {
    return;
  }
  const QString prev = target_type_id();
  const QStringList allowed = allowed_target_ids(calibrator_id_);
  const QString fallback = default_target_for_calibrator(calibrator_id_);

  const bool blocked = combo_target_type_->blockSignals(true);
  combo_target_type_->clear();
  for (const QString &id : allowed) {
    combo_target_type_->addItem(target_type_label(id), id);
  }

  int idx = combo_target_type_->findData(prev);
  if (idx < 0) {
    idx = combo_target_type_->findData(fallback);
  }
  if (idx < 0 && combo_target_type_->count() > 0) {
    idx = 0;
  }
  if (idx >= 0) {
    combo_target_type_->setCurrentIndex(idx);
  }
  combo_target_type_->blockSignals(blocked);
  update_board_param_visibility();
}

/// \brief 按靶标类型显隐板参数与检测选项
void LauncherConfigPanel::update_board_param_visibility() {
  if (combo_target_type_ == nullptr) {
    return;
  }
  const QString t = target_type_id();
  const bool chess =
      t == QStringLiteral("chessboard") || t == QStringLiteral("trihedral_chess");
  const bool charuco =
      t == QStringLiteral("charuco") || t == QStringLiteral("trihedral_charuco");
  const bool aruco =
      t == QStringLiteral("aruco") || t == QStringLiteral("aruco_grid") ||
      t == QStringLiteral("trihedral_aruco");
  const bool aprilgrid = t == QStringLiteral("aprilgrid");
  const bool circles = t == QStringLiteral("circles_symmetric") ||
                       t == QStringLiteral("circles_asymmetric");
  const bool tag_family = charuco || aruco || aprilgrid;
  const bool aruco_single = t == QStringLiteral("aruco");
  const bool square_board = t.startsWith(QStringLiteral("trihedral"));

  if (spin_square_length_ != nullptr) {
    configure_length_mm_spin(spin_square_length_);
  }
  if (spin_marker_length_ != nullptr) {
    configure_length_mm_spin(spin_marker_length_);
  }
  if (spin_max_tag_distance_ != nullptr) {
    configure_length_mm_spin(spin_max_tag_distance_);
  }

  set_form_row_visible(form_target_, combo_dictionary_, tag_family);
  set_form_row_visible(form_target_, squares_row_, !aruco_single);
  // 面积阈值各靶标通用；最大码距仅码类靶标
  set_form_row_visible(form_target_, area_row_, true);
  set_form_row_visible(form_target_, spin_max_tag_distance_, tag_family || aprilgrid);

  const bool show_square = !aruco_single;  // aprilgrid 用 square 槽位显示 tagSpacing
  const bool show_marker = tag_family || aprilgrid || circles;
  set_form_row_visible(form_target_, spin_square_length_, show_square);
  set_form_row_visible(form_target_, spin_marker_length_, show_marker);

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
    set_form_row_label(form_target_, field, text);
  };

  if (square_board) {
    set_form_lab(squares_row_, QStringLiteral("网格边长"));
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
    set_unit_lab(lab_squares_x_, QStringLiteral("列"));
    set_unit_lab(lab_squares_y_, QStringLiteral("行"));
  }

  set_form_lab(spin_square_length_, QStringLiteral("方格边长"));
  set_form_lab(spin_marker_length_, QStringLiteral("码边长"));

  if (chess) {
    set_form_lab(squares_row_, QStringLiteral("内角点"));
    set_form_lab(spin_square_length_, QStringLiteral("方格边长"));
  } else if (charuco) {
    set_form_lab(squares_row_, QStringLiteral("方格数"));
    set_unit_lab(lab_squares_x_, QStringLiteral("列"));
    set_unit_lab(lab_squares_y_, QStringLiteral("行"));
    set_form_lab(spin_square_length_, QStringLiteral("方格边长"));
    set_form_lab(spin_marker_length_, QStringLiteral("标记边长"));
  } else if (aruco) {
    if (aruco_single) {
      set_form_lab(spin_marker_length_, QStringLiteral("码边长"));
    } else {
      set_unit_lab(lab_squares_x_, QStringLiteral("列"));
      set_unit_lab(lab_squares_y_, QStringLiteral("行"));
      set_form_lab(spin_square_length_, QStringLiteral("码间距"));
      set_form_lab(spin_marker_length_, QStringLiteral("码边长"));
    }
  } else if (aprilgrid) {
    set_unit_lab(lab_squares_x_, QStringLiteral("列"));
    set_unit_lab(lab_squares_y_, QStringLiteral("行"));
    set_form_lab(squares_row_, QStringLiteral("April 网格"));
    set_form_lab(spin_marker_length_, QStringLiteral("标签边长"));
    set_form_lab(spin_square_length_, QStringLiteral("标签间距"));
  } else if (circles) {
    set_form_lab(squares_row_, QStringLiteral("圆点阵列"));
    if (t == QStringLiteral("circles_asymmetric")) {
      set_unit_lab(lab_squares_x_, QStringLiteral("每行"));
      set_unit_lab(lab_squares_y_, QStringLiteral("行数"));
      set_form_lab(squares_row_, QStringLiteral("非对称圆"));
      // 与 calib.io「Diagonal Spacing」一致；写入时 /√2 得 OpenCV squareSize
      set_form_lab(spin_square_length_, QStringLiteral("对角间距"));
    } else {
      set_unit_lab(lab_squares_x_, QStringLiteral("列"));
      set_unit_lab(lab_squares_y_, QStringLiteral("行"));
      set_form_lab(spin_square_length_, QStringLiteral("圆心距"));
    }
    set_form_lab(spin_marker_length_, QStringLiteral("圆直径"));
  }

  set_form_row_visible(form_target_, chess_flags_row_, chess);

  // 防止历史误把 form 宿主藏掉（曾对「最大码距」parent 误调 setVisible）
  if (combo_target_type_ != nullptr) {
    if (QWidget *host = combo_target_type_->parentWidget()) {
      host->setVisible(true);
    }
  }
}

/// \brief 切换标定器：按类型组合数据源 / 标定设置控件
void LauncherConfigPanel::set_calibrator_id(const QString &id) {
  const QString prev = calibrator_id_;
  calibrator_id_ = id;
  const bool he =
      id == QStringLiteral("eye_in_hand") || id == QStringLiteral("eye_to_hand");
  const bool intrinsic = id == QStringLiteral("cam_intrinsics") ||
                         id == QStringLiteral("stereo_intrinsics");
  const bool se = id == QStringLiteral("stereo_extrinsics");
  const bool tri = id == QStringLiteral("trihedral_oneshot");
  const bool stereo = id == QStringLiteral("stereo_intrinsics") || se;
  const bool needs_tf = he || tri;

  if (intrinsics_source_block_ != nullptr) {
    (void)intrinsic;
  }
  if (stereo_extrinsics_group_ != nullptr) {
    (void)se;
  }
  if (tf_source_block_ != nullptr) {
    (void)needs_tf;
  }

  if (form_solver_ != nullptr) {
    set_form_row_visible(form_solver_, combo_intrinsics_mode_, intrinsic);
    set_form_row_visible(
        form_solver_, combo_tier4_profile_, intrinsic && intrinsics_mode_is_tier4());
    set_form_row_visible(form_solver_, combo_camera_model_, intrinsic);
    set_form_row_visible(form_solver_, solver_flags_row1_, intrinsic);
    set_form_row_visible(form_solver_, solver_flags_row2_, intrinsic);
    set_form_row_visible(form_solver_, combo_handeye_method_, he);
    set_form_row_visible(form_solver_, combo_stereo_side_, stereo);
    set_form_row_visible(form_solver_, chk_stereo_joint_refine_, stereo && !se);
    if (combo_stereo_side_) {
      combo_stereo_side_->blockSignals(true);
      const QString prev = combo_stereo_side_->currentData().toString();
      combo_stereo_side_->clear();
      if (id == QStringLiteral("stereo_intrinsics")) {
        combo_stereo_side_->addItem(
            QStringLiteral("成对采集"), QStringLiteral("paired"));
        combo_stereo_side_->addItem(
            QStringLiteral("左目补采"), QStringLiteral("left"));
        combo_stereo_side_->addItem(
            QStringLiteral("右目补采"), QStringLiteral("right"));
        const int pidx = combo_stereo_side_->findData(
            prev.isEmpty() ? QStringLiteral("paired") : prev);
        combo_stereo_side_->setCurrentIndex(pidx >= 0 ? pidx : 0);
      } else if (se) {
        combo_stereo_side_->addItem(QStringLiteral("left"), QStringLiteral("left"));
        combo_stereo_side_->addItem(QStringLiteral("right"), QStringLiteral("right"));
        const int pidx = combo_stereo_side_->findData(
            prev.isEmpty() ? QStringLiteral("left") : prev);
        combo_stereo_side_->setCurrentIndex(pidx >= 0 ? pidx : 0);
      }
      combo_stereo_side_->blockSignals(false);
    }
  }

  const bool entering_tri = tri && prev != id;
  // 按任务筛选靶标类型（平面 vs 三面 vs 手眼）
  refresh_target_type_options();
  if (tri) {
    if (entering_tri && combo_dictionary_ != nullptr) {
      const int didx = combo_dictionary_->findText(QStringLiteral("DICT_4X4_250"));
      if (didx >= 0) {
        combo_dictionary_->setCurrentIndex(didx);
      }
    }
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
  if (stereo && spin_min_views_ != nullptr && prev != id) {
    spin_min_views_->setValue(6);
  }
  if (se && prev != id) {
    if (edit_parent_frame_ != nullptr) {
      edit_parent_frame_->setText(QStringLiteral("left"));
    }
    if (edit_child_frame_ != nullptr) {
      edit_child_frame_->setText(QStringLiteral("right"));
    }
  }
  if (he && prev != id) {
    if (combo_intrinsics_source_ != nullptr && intrinsics_source_mode() == 0) {
      const int yidx = combo_intrinsics_source_->findData(2);
      if (yidx >= 0) {
        combo_intrinsics_source_->setCurrentIndex(yidx);
      }
    }
    if (spin_min_views_ != nullptr && spin_min_views_->value() > 20) {
      spin_min_views_->setValue(8);
    }
    if (edit_parent_frame_ != nullptr && id == QStringLiteral("eye_in_hand")) {
      edit_parent_frame_->setText(QStringLiteral("tool0"));
    }
    if (edit_parent_frame_ != nullptr && id == QStringLiteral("eye_to_hand")) {
      edit_parent_frame_->setText(QStringLiteral("base"));
    }
    if (edit_child_frame_ != nullptr) {
      edit_child_frame_->setText(QStringLiteral("camera_optical_frame"));
    }
  }
  if (intrinsic && prev != id && combo_intrinsics_source_ != nullptr) {
    const int nidx = combo_intrinsics_source_->findData(0);
    if (nidx >= 0) {
      combo_intrinsics_source_->setCurrentIndex(nidx);
    }
  }
  update_board_param_visibility();
  apply_task_flow_layout();
}

bool LauncherConfigPanel::intrinsics_mode_is_tier4() const {
  if (combo_intrinsics_mode_ == nullptr) {
    return false;
  }
  return combo_intrinsics_mode_->currentData().toString() == QStringLiteral("tier4");
}

std::string LauncherConfigPanel::tier4_profile_id() const {
  if (combo_tier4_profile_ == nullptr) {
    return "general";
  }
  const QVariant d = combo_tier4_profile_->currentData();
  if (d.isValid() && !d.toString().isEmpty()) {
    return d.toString().toStdString();
  }
  return "general";
}

void LauncherConfigPanel::update_intrinsics_mode_rows() {
  const bool intrinsic = calibrator_id_ == QStringLiteral("cam_intrinsics") ||
                         calibrator_id_ == QStringLiteral("stereo_intrinsics");
  if (form_solver_ != nullptr) {
    set_form_row_visible(
        form_solver_, combo_tier4_profile_, intrinsic && intrinsics_mode_is_tier4());
  }
}

void LauncherConfigPanel::apply_tier4_profile_to_solver_flags(const QString &profile_id) {
  std::map<std::string, std::string> bundle;
  core::apply_tier4_profile_bundle(profile_id.toStdString(), &bundle);
  auto flag = [&](const char *k) {
    const auto it = bundle.find(k);
    return it != bundle.end() && (it->second == "1" || it->second == "true");
  };
  if (chk_fix_principal_ != nullptr) {
    chk_fix_principal_->setChecked(
        flag("fix_principal_point") || flag("fix_principal"));
  }
  if (chk_fix_aspect_ != nullptr) {
    chk_fix_aspect_->setChecked(flag("fix_aspect_ratio") || flag("fix_aspect"));
  }
  if (chk_zero_tangent_ != nullptr) {
    chk_zero_tangent_->setChecked(flag("zero_tangent"));
  }
  if (chk_rational_model_ != nullptr) {
    chk_rational_model_->setChecked(flag("rational_model"));
  }
  if (chk_thin_prism_ != nullptr) {
    chk_thin_prism_->setChecked(
        flag("enable_prism_model") || flag("thin_prism"));
  }
}

namespace {

void set_editable_combo_text(QComboBox *combo, const QString &text) {
  if (combo == nullptr || text.trimmed().isEmpty()) {
    return;
  }
  const QString t = text.trimmed();
  const bool blocked = combo->blockSignals(true);
  const int idx = combo->findText(t);
  if (idx >= 0) {
    combo->setCurrentIndex(idx);
  } else {
    combo->setEditText(t);
  }
  combo->blockSignals(blocked);
}

QString yaml_q(const std::map<std::string, std::string> &kv, const char *key) {
  const auto it = kv.find(key);
  if (it == kv.end()) {
    return {};
  }
  return QString::fromStdString(it->second).trimmed();
}

int parse_source_mode(const QString &s) {
  const QString t = s.toLower();
  if (t == QStringLiteral("ros") || t == QStringLiteral("ros_topic") ||
      t == QStringLiteral("topic") || t == QStringLiteral("online")) {
    return static_cast<int>(SourceMode::RosTopic);
  }
  if (t == QStringLiteral("bag") || t == QStringLiteral("rosbag") ||
      t == QStringLiteral("rosbag2")) {
    return static_cast<int>(SourceMode::RosBag);
  }
  if (t == QStringLiteral("offline") || t == QStringLiteral("images") ||
      t == QStringLiteral("dir")) {
    return static_cast<int>(SourceMode::Offline);
  }
  return -1;
}

int parse_pose_source(const QString &s) {
  const QString t = s.toLower();
  if (t == QStringLiteral("tf")) {
    return static_cast<int>(PoseSource::Tf);
  }
  if (t == QStringLiteral("csv")) {
    return static_cast<int>(PoseSource::Csv);
  }
  return -1;
}

int parse_intrinsics_source(const QString &s) {
  const QString t = s.toLower();
  if (t == QStringLiteral("1") || t == QStringLiteral("camera_info") ||
      t == QStringLiteral("info")) {
    return 1;
  }
  if (t == QStringLiteral("2") || t == QStringLiteral("yaml") ||
      t == QStringLiteral("file")) {
    return 2;
  }
  if (t == QStringLiteral("0") || t == QStringLiteral("none") ||
      t == QStringLiteral("unused")) {
    return 0;
  }
  return -1;
}

void set_combo_data(QComboBox *combo, int data) {
  if (combo == nullptr || data < 0) {
    return;
  }
  const int idx = combo->findData(data);
  if (idx >= 0) {
    combo->setCurrentIndex(idx);
  }
}

}  // namespace

/// \brief 用扁平 YAML 字典填充数据源 / 标定设置控件
void LauncherConfigPanel::apply_config_map(const std::map<std::string, std::string> &kv) {
  if (kv.empty()) {
    return;
  }

  const QString target = yaml_q(kv, "target");
  if (!target.isEmpty() && combo_target_type_ != nullptr) {
    const bool blocked = combo_target_type_->blockSignals(true);
    int idx = combo_target_type_->findData(target);
    if (idx < 0) {
      idx = combo_target_type_->findText(target);
    }
    if (idx >= 0) {
      combo_target_type_->setCurrentIndex(idx);
    }
    combo_target_type_->blockSignals(blocked);
    update_board_param_visibility();
  }

  const QString dict = yaml_q(kv, "dictionary");
  if (!dict.isEmpty() && combo_dictionary_ != nullptr) {
    const int didx = combo_dictionary_->findText(dict);
    if (didx >= 0) {
      combo_dictionary_->setCurrentIndex(didx);
    }
  }

  auto to_int = [&](const char *k, int fallback) {
    const auto it = kv.find(k);
    if (it == kv.end() || it->second.empty()) {
      return fallback;
    }
    try {
      return std::stoi(it->second);
    } catch (...) {
      return fallback;
    }
  };
  auto to_dbl = [&](const char *k, double fallback) {
    const auto it = kv.find(k);
    if (it == kv.end() || it->second.empty()) {
      return fallback;
    }
    try {
      return std::stod(it->second);
    } catch (...) {
      return fallback;
    }
  };

  const int sx = to_int("squares_x", squares_x());
  const int sy = to_int("squares_y", squares_y());
  const double sq = to_dbl("square_length", square_length());
  const double mk = to_dbl("marker_length", marker_length());
  if (kv.count("squares_x") || kv.count("squares_y") || kv.count("square_length") ||
      kv.count("marker_length")) {
    set_board_params(sx, sy, sq, mk);
  }

  const int src = parse_source_mode(yaml_q(kv, "source_mode"));
  if (src >= 0 && combo_source_mode_ != nullptr) {
    set_combo_data(combo_source_mode_, src);
    refresh_source_mode_rows();
  }

  set_editable_combo_text(combo_image_topic_, yaml_q(kv, "image_topic"));
  set_editable_combo_text(combo_left_image_topic_, yaml_q(kv, "left_image_topic"));
  set_editable_combo_text(combo_right_image_topic_, yaml_q(kv, "right_image_topic"));
  set_editable_combo_text(combo_bag_topic_, yaml_q(kv, "bag_topic"));
  if (yaml_q(kv, "bag_topic").isEmpty() && !yaml_q(kv, "image_topic").isEmpty()) {
    set_editable_combo_text(combo_bag_topic_, yaml_q(kv, "image_topic"));
  }

  const int intrins = parse_intrinsics_source(yaml_q(kv, "intrinsics_source"));
  if (intrins >= 0 && combo_intrinsics_source_ != nullptr) {
    set_combo_data(combo_intrinsics_source_, intrins);
    refresh_intrinsics_source_rows();
  }
  const QString cam_info = yaml_q(kv, "camera_info_topic");
  if (!cam_info.isEmpty()) {
    if (intrins < 0 && combo_intrinsics_source_ != nullptr &&
        yaml_q(kv, "camera_yaml").isEmpty()) {
      set_combo_data(combo_intrinsics_source_, 1);
      refresh_intrinsics_source_rows();
    }
    set_editable_combo_text(combo_camera_info_topic_, cam_info);
  }
  const QString cam_yaml = yaml_q(kv, "camera_yaml");
  if (!cam_yaml.isEmpty()) {
    if (edit_intrinsics_yaml_ != nullptr) {
      edit_intrinsics_yaml_->setText(cam_yaml);
    }
    if (edit_camera_yaml_ != nullptr) {
      edit_camera_yaml_->setText(cam_yaml);
    }
    if (intrins < 0 && combo_intrinsics_source_ != nullptr) {
      set_combo_data(combo_intrinsics_source_, 2);
      refresh_intrinsics_source_rows();
    }
  }
  const QString left_yaml = yaml_q(kv, "left_camera_yaml");
  if (!left_yaml.isEmpty() && edit_left_camera_yaml_ != nullptr) {
    edit_left_camera_yaml_->setText(left_yaml);
  }
  const QString right_yaml = yaml_q(kv, "right_camera_yaml");
  if (!right_yaml.isEmpty() && edit_right_camera_yaml_ != nullptr) {
    edit_right_camera_yaml_->setText(right_yaml);
  }

  const int pose = parse_pose_source(yaml_q(kv, "pose_source"));
  if (pose >= 0 && combo_pose_source_ != nullptr) {
    set_combo_data(combo_pose_source_, pose);
  }
  const QString pose_csv = yaml_q(kv, "pose_csv");
  if (!pose_csv.isEmpty() && edit_pose_csv_ != nullptr) {
    edit_pose_csv_->setText(pose_csv);
  }

  auto set_line = [](QLineEdit *edit, const QString &v) {
    if (edit != nullptr && !v.isEmpty()) {
      edit->setText(v);
    }
  };
  set_line(edit_base_frame_, yaml_q(kv, "base_frame"));
  set_line(edit_gripper_frame_, yaml_q(kv, "gripper_frame"));
  set_line(edit_parent_frame_, yaml_q(kv, "parent_frame"));
  set_line(edit_child_frame_, yaml_q(kv, "child_frame"));
  set_line(edit_image_frame_, yaml_q(kv, "image_frame"));
  set_line(edit_camera_link_frame_, yaml_q(kv, "camera_link_frame"));
  set_line(edit_image_dir_, yaml_q(kv, "image_dir"));
  set_line(edit_bag_path_, yaml_q(kv, "bag_path"));
  set_line(edit_export_path_, yaml_q(kv, "export_dir"));

  const QString method = yaml_q(kv, "method");
  if (!method.isEmpty() && combo_handeye_method_ != nullptr) {
    const int midx = combo_handeye_method_->findText(method);
    if (midx >= 0) {
      combo_handeye_method_->setCurrentIndex(midx);
    }
  }

  if (kv.count("min_views") && spin_min_views_ != nullptr) {
    spin_min_views_->setValue(std::max(1, to_int("min_views", min_views())));
  }
  if (kv.count("min_confidence") && spin_min_confidence_ != nullptr) {
    spin_min_confidence_->setValue(to_dbl("min_confidence", min_confidence()));
  }
  if (kv.count("min_diversity") && spin_min_diversity_ != nullptr) {
    spin_min_diversity_->setValue(to_dbl("min_diversity", min_diversity()));
  }
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
  // 内参 YAML：优先可选内参源；手眼块字段作兼容回退
  QString cam_yaml;
  if (intrinsics_source_mode() == 2 && edit_intrinsics_yaml_ != nullptr) {
    cam_yaml = edit_intrinsics_yaml_->text().trimmed();
  }
  if (cam_yaml.isEmpty() && edit_camera_yaml_ != nullptr) {
    cam_yaml = edit_camera_yaml_->text().trimmed();
  }
  session->set_camera_yaml(cam_yaml);
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
      {"target", target_type_id().toStdString()},
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
      {"intrinsics_profile",
       [this]() -> std::string {
         if (!intrinsics_mode_is_tier4()) {
           return "classic";
         }
         return tier4_profile_id();
       }()},
      {"stereo_capture_mode",
       [&]() -> std::string {
         if (!combo_stereo_side_) {
           return "paired";
         }
         const QVariant d = combo_stereo_side_->currentData();
         if (d.isValid() && !d.toString().isEmpty()) {
           return d.toString().toStdString();
         }
         return "paired";
       }()},
      {"stereo_side",
       [&]() -> std::string {
         if (!combo_stereo_side_) {
           return "left";
         }
         const QVariant d = combo_stereo_side_->currentData();
         const QString mode =
             d.isValid() && !d.toString().isEmpty() ? d.toString() : QStringLiteral("left");
         if (mode == QStringLiteral("paired")) {
           return "left";
         }
         return mode.toStdString();
       }()},
      {"generate_stereo_rectified", "true"},
      {"stereo_joint_refine",
       chk_stereo_joint_refine_ && chk_stereo_joint_refine_->isChecked() ? "true" : "false"},
      {"stereo_max_sync_ms", "30"},
      {"left_image_topic",
       [&]() -> std::string {
         if (!combo_left_image_topic_) {
           return {};
         }
         return combo_left_image_topic_->currentText().trimmed().toStdString();
       }()},
      {"right_image_topic",
       [&]() -> std::string {
         if (!combo_right_image_topic_) {
           return {};
         }
         return combo_right_image_topic_->currentText().trimmed().toStdString();
       }()},
      {"image_topic",
       [&]() -> std::string {
         if (!combo_image_topic_) {
           return {};
         }
         return combo_image_topic_->currentText().trimmed().toStdString();
       }()},
      {"bag_topic",
       [&]() -> std::string {
         if (!combo_bag_topic_) {
           return {};
         }
         return combo_bag_topic_->currentText().trimmed().toStdString();
       }()},
      {"min_views", std::to_string(min_views())},
      {"min_confidence", std::to_string(min_confidence())},
      {"min_diversity", std::to_string(min_diversity())},
      {"auto_cooldown_ms", std::to_string(auto_cooldown_ms())},
      {"min_board_area", std::to_string(spin_min_board_area_->value())},
      {"max_board_area", std::to_string(spin_max_board_area_->value())},
      {"max_tag_distance",
       std::to_string(spin_max_tag_distance_->value() / 1000.0)},
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
       combo_camera_info_topic_
           ? combo_camera_info_topic_->currentText().trimmed().toStdString()
           : ""},
      {"intrinsics_source", std::to_string(intrinsics_source_mode())},
      {"parent_frame", edit_parent_frame_->text().trimmed().toStdString()},
      {"child_frame", edit_child_frame_->text().trimmed().toStdString()},
      {"viz_corners", "1"},
      {"viz_hull", "1"},
      {"viz_conf", "1"},
      {"viz_aruco", "1"},
      {"viz_marker_radius", "4"},
      {"gui.stats_backend",
       combo_stats_backend_ != nullptr
           ? combo_stats_backend_->currentData().toString().toStdString()
           : "qt"},
  };
  if (combo_handeye_method_ != nullptr) {
    m["method"] = combo_handeye_method_->currentText().toStdString();
  }
  if (combo_source_mode_ != nullptr) {
    const int sm = combo_source_mode_->currentData().toInt();
    if (sm == static_cast<int>(SourceMode::RosTopic)) {
      m["source_mode"] = "ros_topic";
    } else if (sm == static_cast<int>(SourceMode::RosBag)) {
      m["source_mode"] = "rosbag";
    } else {
      m["source_mode"] = "offline";
    }
  }
  if (combo_pose_source_ != nullptr) {
    const int ps = combo_pose_source_->currentData().toInt();
    if (ps == static_cast<int>(PoseSource::Tf)) {
      m["pose_source"] = "tf";
    } else if (ps == static_cast<int>(PoseSource::Csv)) {
      m["pose_source"] = "csv";
    }
  }
  if (target_type_id() == QStringLiteral("aprilgrid")) {
    m["tag_spacing"] = std::to_string(square_length());
  }
  if (target_type_id() == QStringLiteral("circles_symmetric") ||
      target_type_id() == QStringLiteral("circles_asymmetric")) {
    // 圆直径：标定物点不用，但用于 Blob 面积约束，减少错检导致的大 RMS
    m["circle_diameter"] = std::to_string(marker_length());
  }
  if (intrinsics_source_mode() == 2 && edit_intrinsics_yaml_ != nullptr &&
      !edit_intrinsics_yaml_->text().trimmed().isEmpty()) {
    m["camera_yaml"] = edit_intrinsics_yaml_->text().trimmed().toStdString();
  } else if (edit_camera_yaml_ != nullptr && !edit_camera_yaml_->text().trimmed().isEmpty()) {
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
  if (intrinsics_mode_is_tier4()) {
    core::apply_tier4_profile_bundle(tier4_profile_id(), &m);
  } else {
    m["intrinsics_profile"] = "classic";
  }
  return m;
}

/// \brief 当前靶标类型 ID（优先 itemData）
QString LauncherConfigPanel::target_type_id() const {
  if (combo_target_type_ == nullptr) {
    return QStringLiteral("chessboard");
  }
  const QVariant d = combo_target_type_->currentData();
  if (d.isValid() && !d.toString().isEmpty()) {
    return d.toString();
  }
  return combo_target_type_->currentText();
}

/// \brief 方格/内角点 X 数
int LauncherConfigPanel::squares_x() const {
  return spin_squares_x_ ? spin_squares_x_->value() : 9;
}
/// \brief 方格/内角点 Y 数
int LauncherConfigPanel::squares_y() const {
  return spin_squares_y_ ? spin_squares_y_->value() : 6;
}
/// \brief 方格边长（米）；Aprilgrid 时返回无量纲 tagSpacing（= 间距mm / 边长mm）
///
/// 非对称圆：UI 为 calib.io Diagonal Spacing（最近邻圆心距），返回 OpenCV
/// squareSize = 对角距 / √2（物点公式 (2j+i%2)*s, i*s）。
double LauncherConfigPanel::square_length() const {
  if (spin_square_length_ == nullptr) {
    return 0.025;
  }
  if (target_type_id() == QStringLiteral("aprilgrid")) {
    const double tag_mm =
        spin_marker_length_ != nullptr ? spin_marker_length_->value() : 88.0;
    if (tag_mm <= 1e-6) {
      return 0.3;
    }
    return spin_square_length_->value() / tag_mm;
  }
  const double mm = spin_square_length_->value();
  if (target_type_id() == QStringLiteral("circles_asymmetric")) {
    return (mm / 1000.0) / std::sqrt(2.0);
  }
  return mm / 1000.0;
}
/// \brief ArUco/ChArUco 码边长（米）
double LauncherConfigPanel::marker_length() const {
  if (spin_marker_length_ == nullptr) {
    return 0.018;
  }
  return spin_marker_length_->value() / 1000.0;
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
QString LauncherConfigPanel::stats_backend() const {
  if (combo_stats_backend_ == nullptr) {
    return QStringLiteral("qt");
  }
  return combo_stats_backend_->currentData().toString();
}
/// \brief 是否画角点
bool LauncherConfigPanel::viz_draw_corners() const {
  return true;
}
/// \brief 是否画凸包
bool LauncherConfigPanel::viz_draw_hull() const {
  return true;
}
/// \brief 是否显示置信度文字
bool LauncherConfigPanel::viz_show_conf() const {
  return true;
}
/// \brief 是否叠加 ArUco
bool LauncherConfigPanel::viz_draw_aruco() const {
  return true;
}
/// \brief 角点/码绘制半径
int LauncherConfigPanel::viz_marker_radius() const {
  return 4;
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
  if (spin_marker_length_) {
    spin_marker_length_->setValue(marker_m * 1000.0);
  }
  if (spin_square_length_) {
    if (target_type_id() == QStringLiteral("aprilgrid")) {
      // square_m 实为无量纲 tagSpacing；UI 显示空白边距 mm
      const double tag_mm =
          spin_marker_length_ != nullptr ? spin_marker_length_->value()
                                         : (marker_m * 1000.0);
      // 兼容旧值：若像 0.01~2 的比例则换算；若已是毫米量级则直接用
      if (square_m > 0.0 && square_m <= 2.0) {
        spin_square_length_->setValue(square_m * std::max(1.0, tag_mm));
      } else {
        spin_square_length_->setValue(square_m);
      }
    } else if (target_type_id() == QStringLiteral("circles_asymmetric")) {
      // 内部 square_m = OpenCV squareSize；UI 显示对角间距 = s·√2
      spin_square_length_->setValue(square_m * 1000.0 * std::sqrt(2.0));
    } else {
      spin_square_length_->setValue(square_m * 1000.0);
    }
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

/// \brief Bag 最多导出帧数
int LauncherConfigPanel::bag_max_frames() const {
  return spin_bag_max_frames_ ? spin_bag_max_frames_->value() : 500;
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

void LauncherConfigPanel::move_block_to_page(QWidget *block, QWidget *page) {
  if (block == nullptr || page == nullptr) {
    return;
  }
  if (block->parentWidget() == page) {
    return;
  }
  block->setParent(page);
  if (auto *lay = qobject_cast<QVBoxLayout *>(page->layout())) {
    lay->addWidget(block);
  }
}

void LauncherConfigPanel::finalize_task_flow_layout() {
  auto *ds_root = qobject_cast<QVBoxLayout *>(data_source_panel_->layout());
  if (ds_root == nullptr) {
    return;
  }

  task_data_source_title_ = new QLabel(data_source_panel_);
  task_data_source_title_->setObjectName(QStringLiteral("TaskFlowTitle"));
  task_data_source_title_->setWordWrap(true);

  ds_root->removeWidget(intrinsics_source_block_);
  ds_root->removeWidget(stereo_extrinsics_group_);
  ds_root->removeWidget(tf_source_block_);

  data_source_stack_ = new QStackedWidget(data_source_panel_);
  auto *page_intrinsics = new QWidget(data_source_panel_);
  page_intrinsics->setLayout(new QVBoxLayout());
  page_intrinsics->layout()->setContentsMargins(0, 0, 0, 0);
  move_block_to_page(intrinsics_source_block_, page_intrinsics);

  auto *page_stereo_yaml = new QWidget(data_source_panel_);
  page_stereo_yaml->setLayout(new QVBoxLayout());
  page_stereo_yaml->layout()->setContentsMargins(0, 0, 0, 0);
  move_block_to_page(stereo_extrinsics_group_, page_stereo_yaml);

  auto *page_placeholder = new QWidget(data_source_panel_);
  auto *placeholder_lay = new QVBoxLayout(page_placeholder);
  placeholder_lay->addStretch(1);
  data_source_stack_->addWidget(page_intrinsics);
  data_source_stack_->addWidget(page_stereo_yaml);
  data_source_stack_->addWidget(page_placeholder);

  ds_root->insertWidget(0, task_data_source_title_);
  ds_root->insertWidget(2, data_source_stack_);
  ds_root->addWidget(tf_source_block_);
  ds_root->addStretch(1);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  if (root == nullptr) {
    return;
  }
  task_setup_title_ = new QLabel(this);
  task_setup_title_->setObjectName(QStringLiteral("TaskFlowTitle"));
  task_setup_title_->setWordWrap(true);

  root->removeWidget(board_params_block_);
  root->removeWidget(solver_intrinsics_block_);
  root->removeWidget(capture_criteria_block_);

  setup_stack_ = new QStackedWidget(this);
  setup_stack_pages_.clear();
  for (int i = 0; i < 5; ++i) {
    auto *page = new QWidget(this);
    page->setLayout(new QVBoxLayout());
    page->layout()->setContentsMargins(0, 0, 0, 0);
    setup_stack_pages_.push_back(page);
    setup_stack_->addWidget(page);
  }
  move_block_to_page(board_params_block_, setup_stack_pages_[0]);
  move_block_to_page(solver_intrinsics_block_, setup_stack_pages_[0]);
  move_block_to_page(capture_criteria_block_, setup_stack_pages_[0]);

  root->insertWidget(0, task_setup_title_);
  root->insertWidget(1, setup_stack_, 1);
  root->addStretch(1);
}

bool LauncherConfigPanel::uses_stereo_dual_topics() const {
  return calibrator_id_ == QStringLiteral("stereo_intrinsics");
}

void LauncherConfigPanel::refresh_image_topic_rows() {
  const int mode =
      combo_source_mode_ != nullptr ? combo_source_mode_->currentData().toInt()
                                    : static_cast<int>(SourceMode::Offline);
  const bool ros = mode == static_cast<int>(SourceMode::RosTopic);
  const bool bag = mode == static_cast<int>(SourceMode::RosBag);
  const bool dual = uses_stereo_dual_topics() && (ros || bag);
  set_form_row_visible(form_ros_, topic_row_, ros && !dual);
  set_form_row_visible(form_ros_, stereo_topic_row_, dual);
  set_form_row_visible(form_ros_, bag_topic_row_, bag && !dual);
}

QString LauncherConfigPanel::active_image_topic() const {
  if (uses_stereo_dual_topics() && combo_left_image_topic_ != nullptr &&
      combo_right_image_topic_ != nullptr && combo_stereo_side_ != nullptr) {
    const QString side = combo_stereo_side_->currentData().toString();
    if (side == QStringLiteral("right")) {
      return combo_right_image_topic_->currentText().trimmed();
    }
    return combo_left_image_topic_->currentText().trimmed();
  }
  if (combo_image_topic_ == nullptr) {
    return {};
  }
  return combo_image_topic_->currentText().trimmed();
}

void LauncherConfigPanel::apply_task_flow_layout() {
  const TaskFlowKind flow = task_flow_from_calibrator_id(calibrator_id_);
  if (task_data_source_title_ != nullptr) {
    task_data_source_title_->setText(
        task_flow_step_title(flow, QStringLiteral("数据源设置")));
  }
  if (task_setup_title_ != nullptr) {
    task_setup_title_->setText(
        task_flow_step_title(flow, QStringLiteral("标定设置")));
  }

  refresh_image_topic_rows();

  if (data_source_stack_ != nullptr) {
    switch (flow) {
      case TaskFlowKind::MonoIntrinsics:
      case TaskFlowKind::StereoIntrinsics:
      case TaskFlowKind::HandEye:
        data_source_stack_->setCurrentIndex(0);
        break;
      case TaskFlowKind::StereoExtrinsics:
        data_source_stack_->setCurrentIndex(1);
        break;
      case TaskFlowKind::Trihedral:
        data_source_stack_->setCurrentIndex(2);
        break;
    }
  }

  if (tf_source_block_ != nullptr) {
    tf_source_block_->setVisible(
        flow == TaskFlowKind::Trihedral || flow == TaskFlowKind::HandEye);
  }

  if (setup_stack_ != nullptr && !setup_stack_pages_.empty()) {
    const int idx = static_cast<int>(flow);
    setup_stack_->setCurrentIndex(idx);
    if (idx >= 0 && idx < setup_stack_pages_.size()) {
      move_block_to_page(board_params_block_, setup_stack_pages_[idx]);
      move_block_to_page(solver_intrinsics_block_, setup_stack_pages_[idx]);
      move_block_to_page(capture_criteria_block_, setup_stack_pages_[idx]);
    }
  }
}

}  // namespace gui
}  // namespace hs_calib
