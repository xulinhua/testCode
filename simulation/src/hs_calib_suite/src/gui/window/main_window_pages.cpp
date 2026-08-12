#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/io/board_config_yaml.hpp"

#include <functional>
#include <memory>

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAction>
#include <QActionGroup>
#include <QButtonGroup>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QCursor>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QPushButton>
#include <QRectF>
#include <QScrollArea>
#include <QShortcut>
#include <QSize>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTextEdit>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>

namespace hs_calib {
namespace gui {

using hs_calib::gui::window_detail::make_label;
using hs_calib::gui::window_detail::TileClickFilter;
using hs_calib::gui::window_detail::TbGlyph;
using hs_calib::gui::window_detail::bind_toolbar_action;
using hs_calib::gui::window_detail::set_metric_value;


/// \brief 带标题头的面板容器
QWidget *MainWindow::make_panel(const QString &title, QWidget *body) {
  auto *panel = new QFrame;
  panel->setObjectName(QStringLiteral("Panel"));
  auto *layout = new QVBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto *header = new QFrame(panel);
  header->setObjectName(QStringLiteral("PanelHeader"));
  auto *header_layout = new QHBoxLayout(header);
  header_layout->setContentsMargins(16, 10, 16, 10);
  header_layout->addWidget(make_label(title, QStringLiteral("SectionTitle"), header));
  layout->addWidget(header);

  auto *body_wrap = new QWidget(panel);
  auto *body_layout = new QVBoxLayout(body_wrap);
  body_layout->setContentsMargins(12, 12, 12, 12);
  body_layout->addWidget(body);
  layout->addWidget(body_wrap, 1);
  return panel;
}

/// \brief 大号指标卡片
QFrame *MainWindow::make_metric_card(const QString &name, const QString &value) {
  auto *card = new QFrame;
  card->setObjectName(QStringLiteral("MetricCard"));
  auto *layout = new QVBoxLayout(card);
  layout->setContentsMargins(16, 14, 16, 14);
  layout->setSpacing(4);
  auto *v = make_label(value, QStringLiteral("MetricValue"), card);
  auto *n = make_label(name, QStringLiteral("MetricName"), card);
  layout->addWidget(v);
  layout->addWidget(n);
  if (name == QStringLiteral("已采集帧数")) {
    metric_frames_ = v;
  } else if (name == QStringLiteral("检测置信度")) {
    metric_detect_ = v;
  } else if (name == QStringLiteral("覆盖提示")) {
    metric_coverage_ = v;
  } else if (name == QStringLiteral("重投影 RMSE")) {
    review_rmse_ = v;
  } else if (name == QStringLiteral("有效观测")) {
    review_views_ = v;
  } else if (name == QStringLiteral("图像尺寸")) {
    review_size_ = v;
  }
  return card;
}

/// \brief 紧凑指标卡片
QFrame *MainWindow::make_compact_metric_card(const QString &name, const QString &value) {
  auto *card = new QFrame;
  card->setObjectName(QStringLiteral("CompactMetricCard"));
  auto *layout = new QVBoxLayout(card);
  layout->setContentsMargins(10, 8, 10, 8);
  layout->setSpacing(2);
  auto *v = make_label(value, QStringLiteral("MetricValue"), card);
  auto *n = make_label(name, QStringLiteral("MetricName"), card);
  layout->addWidget(v);
  layout->addWidget(n);
  if (name == QStringLiteral("已采集帧数")) {
    metric_frames_ = v;
  } else if (name == QStringLiteral("检测置信度")) {
    metric_detect_ = v;
  } else if (name == QStringLiteral("覆盖提示")) {
    metric_coverage_ = v;
  }
  return card;
}

/// \brief 选中首页标定器磁贴
void MainWindow::select_calib_tile(QFrame *tile) {
  if (tile == nullptr) {
    return;
  }
  const bool planned = tile->property("planned").toBool();
  if (planned) {
    append_log(
        LogLevel::Warn,
        QStringLiteral("› 「%1」尚未实现，请选择已接通标定器")
            .arg(tile->property("calibrator_title").toString()));
    return;
  }
  if (selected_tile_ != nullptr && selected_tile_ != tile) {
    selected_tile_->setProperty("selected", false);
    selected_tile_->style()->unpolish(selected_tile_);
    selected_tile_->style()->polish(selected_tile_);
  }
  selected_tile_ = tile;
  tile->setProperty("selected", true);
  tile->style()->unpolish(tile);
  tile->style()->polish(tile);

  selected_calibrator_id_ = tile->property("calibrator_id").toString();
  if (launcher_panel_ != nullptr) {
    launcher_panel_->set_calibrator_id(selected_calibrator_id_);
  }
  if (session_) {
    session_->set_calibrator_id(selected_calibrator_id_);
  }
  refresh_handeye_ui();
  refresh_setup_readiness();
  const QString title = tile->property("calibrator_title").toString();
  if (home_selection_ != nullptr) {
    home_selection_->setText(
        QStringLiteral("当前：default_robot  ·  %1（%2）")
            .arg(title, selected_calibrator_id_));
  }
  append_log(LogLevel::Info, QStringLiteral("› 选定标定器：%1").arg(selected_calibrator_id_));
}

/// \brief 创建标定器选择磁贴
QFrame *MainWindow::make_calib_tile(
    const QString &title,
    const QString &subtitle,
    const QString &id,
    bool implemented,
    const QString &prerequisite) {
  auto *tile = new QFrame;
  tile->setObjectName(QStringLiteral("CalibTile"));
  tile->setCursor(implemented ? Qt::PointingHandCursor : Qt::ForbiddenCursor);
  tile->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  tile->setMinimumHeight(implemented ? 78 : 72);
  tile->setProperty("calibrator_id", id);
  tile->setProperty("calibrator_title", title);
  tile->setProperty("selected", false);
  tile->setProperty("planned", !implemented);
  tile->setProperty("implemented", implemented);

  auto *row = new QHBoxLayout(tile);
  row->setContentsMargins(14, 10, 14, 10);
  row->setSpacing(12);

  auto *accent = new QFrame(tile);
  accent->setObjectName(QStringLiteral("CalibTileAccent"));
  accent->setFixedSize(4, 44);
  row->addWidget(accent, 0, Qt::AlignVCenter);

  auto *text_col = new QVBoxLayout;
  text_col->setContentsMargins(0, 0, 0, 0);
  text_col->setSpacing(2);
  const QString title_text =
      implemented ? title : (title + QStringLiteral(" · 即将推出"));
  auto *title_lbl = make_label(title_text, QStringLiteral("CalibTileTitle"), tile);
  title_lbl->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  auto *sub = make_label(subtitle, QStringLiteral("CalibTileSub"), tile);
  sub->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  text_col->addWidget(title_lbl);
  text_col->addWidget(sub);
  if (!prerequisite.isEmpty()) {
    auto *pre = make_label(
        QStringLiteral("前置：%1").arg(prerequisite), QStringLiteral("CalibTilePre"),
        tile);
    pre->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    text_col->addWidget(pre);
  }
  row->addLayout(text_col, 1);

  tile->installEventFilter(
      new TileClickFilter([this, tile]() { select_calib_tile(tile); }, tile));
  return tile;
}

/// \brief 首页分类切换
void MainWindow::on_home_category_changed(int category) {
  home_category_ = category;
  refresh_home_calibrator_grid();
}

/// \brief 按分类刷新标定器磁贴网格
void MainWindow::refresh_home_calibrator_grid() {
  if (home_tile_grid_ == nullptr || home_tile_host_ == nullptr) {
    return;
  }
  while (QLayoutItem *item = home_tile_grid_->takeAt(0)) {
    if (QWidget *w = item->widget()) {
      if (w == selected_tile_) {
        selected_tile_ = nullptr;
      }
      w->deleteLater();
    }
    delete item;
  }

  struct Entry {
    const char *title;
    const char *sub;
    const char *id;
    bool implemented;
    const char *pre;
  };

  // —— 按首页分类填充磁贴 ——
  // category: 0内参 1手眼 2外参 3多传感器
  QVector<Entry> entries;
  switch (home_category_) {
    case 1:  // hand-eye
      entries = {
          {"眼在手上", "gripper → camera · 棋盘 + 位姿", "eye_in_hand", true,
           "相机内参 YAML"},
          {"眼在手外", "base → camera · 棋盘 + 位姿", "eye_to_hand", true,
           "相机内参 YAML"},
      };
      break;
    case 2:  // extrinsics
      entries = {
          {"双目相对外参", "左右相机 · 立体校正", "stereo_extrinsics", false, "双目内参"},
          {"直角三面标定", "单帧/多帧 · 已知夹角三维靶", "trihedral_oneshot", true,
           "三面 ChArUco"},
          {"相机–激光", "相机与 LiDAR 外参", "cam_lidar", false, "相机内参 + 点云"},
      };
      break;
    case 3:  // multi / system
      entries = {
          {"传感器套件联合", "按序调用多个标定器", "sensor_kit_bundle", false, "—"},
          {"时间偏移", "相机 / 激光时间同步", "time_offset", false, "—"},
      };
      break;
    case 0:
    default:  // intrinsics
      entries = {
          {"单目内参", "多姿态平面靶 · 已接通", "cam_intrinsics", true, "棋盘/ChArUco/ArUco/圆点"},
          {"直角三面内参", "单帧即可 · 也可多帧精化", "trihedral_oneshot", true,
           "三面 ChArUco"},
          {"双目各自内参", "左右目分别标定", "stereo_intrinsics", false, "双目图像"},
      };
      break;
  }

  QFrame *first_impl = nullptr;
  for (int i = 0; i < entries.size(); ++i) {
    const auto &e = entries[i];
    auto *tile = make_calib_tile(
        QString::fromUtf8(e.title), QString::fromUtf8(e.sub), QString::fromUtf8(e.id),
        e.implemented, QString::fromUtf8(e.pre));
    home_tile_grid_->addWidget(tile, i / 2, i % 2);
    if (e.implemented && first_impl == nullptr) {
      first_impl = tile;
    }
  }
  home_tile_grid_->setRowStretch((entries.size() + 1) / 2, 1);

  if (first_impl != nullptr) {
    select_calib_tile(first_impl);
  } else if (home_selection_ != nullptr) {
    home_selection_->setText(
        QStringLiteral("当前类别暂无已接通标定器，请切换「内参」或「手眼」"));
    selected_calibrator_id_.clear();
  }
}

/// \brief 构建首页
QWidget *MainWindow::build_home_page() {
  auto *page = new QWidget;
  auto *root = new QHBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 8);
  root->setSpacing(16);

  auto *project_list = new QListWidget;
  project_list->addItems(
      {QStringLiteral("default_robot"), QStringLiteral("arm_cell_A"),
       QStringLiteral("mobile_base_01")});
  project_list->setCurrentRow(0);
  auto *project_panel = make_panel(QStringLiteral("项目（占位）"), project_list);
  project_panel->setMinimumWidth(240);
  project_panel->setMaximumWidth(280);

  auto *right = new QWidget;
  auto *right_layout = new QVBoxLayout(right);
  right_layout->setContentsMargins(0, 0, 0, 0);
  right_layout->setSpacing(8);
  right_layout->addWidget(
      make_label(QStringLiteral("选择标定任务"), QStringLiteral("PageTitle"), right));
  right_layout->addWidget(make_label(
      QStringLiteral("先选类别，再选标定器（对齐 Tier4：Category → Calibrator）。"),
      QStringLiteral("PageSubtitle"), right));

  auto *cat_row = new QHBoxLayout;
  cat_row->setSpacing(8);
  home_category_group_ = new QButtonGroup(page);
  home_category_group_->setExclusive(true);
  const char *cats[] = {"内参", "手眼", "外参", "多传感器"};
  for (int i = 0; i < 4; ++i) {
    auto *btn = new QPushButton(QString::fromUtf8(cats[i]), right);
    btn->setObjectName(QStringLiteral("CategoryChip"));
    btn->setCheckable(true);
    btn->setCursor(Qt::PointingHandCursor);
    home_category_group_->addButton(btn, i);
    cat_row->addWidget(btn);
  }
  cat_row->addStretch(1);
  right_layout->addLayout(cat_row);
  connect(
      home_category_group_, QOverload<int>::of(&QButtonGroup::idClicked), this,
      &MainWindow::on_home_category_changed);

  auto *scroll = new QScrollArea(right);
  scroll->setObjectName(QStringLiteral("HomeTaskScroll"));
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->viewport()->setAutoFillBackground(false);
  scroll->viewport()->setStyleSheet(QStringLiteral("background: transparent;"));

  home_tile_host_ = new QWidget;
  home_tile_host_->setObjectName(QStringLiteral("HomeTaskHost"));
  home_tile_grid_ = new QGridLayout(home_tile_host_);
  home_tile_grid_->setContentsMargins(0, 4, 0, 4);
  home_tile_grid_->setHorizontalSpacing(10);
  home_tile_grid_->setVerticalSpacing(8);
  home_tile_grid_->setAlignment(Qt::AlignTop);
  scroll->setWidget(home_tile_host_);
  right_layout->addWidget(scroll, 1);

  auto *footer = new QHBoxLayout;
  home_selection_ = make_label(QString(), QStringLiteral("Muted"), right);
  footer->addWidget(home_selection_, 1);
  auto *next = new QPushButton(QStringLiteral("下一步：会话配置"), right);
  next->setObjectName(QStringLiteral("PrimaryButton"));
  connect(next, &QPushButton::clicked, this, [this]() {
    QString err;
    if (!ensure_implemented_calibrator(&err)) {
      append_log(LogLevel::Error, QStringLiteral("› %1").arg(err));
      return;
    }
    go_to(PageId::Setup);
  });
  footer->addWidget(next);
  right_layout->addLayout(footer);

  root->addWidget(project_panel);
  root->addWidget(right, 1);

  if (QAbstractButton *b = home_category_group_->button(0)) {
    b->setChecked(true);
  }
  home_category_ = 0;
  refresh_home_calibrator_grid();
  return page;
}

// ===== 流程门禁 =====

/// \brief 校验当前标定器已实现
bool MainWindow::ensure_implemented_calibrator(QString *error_out) const {
  if (selected_calibrator_id_ == QStringLiteral("cam_intrinsics") ||
      selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
      selected_calibrator_id_ == QStringLiteral("eye_to_hand") ||
      selected_calibrator_id_ == QStringLiteral("trihedral_oneshot")) {
    return true;
  }
  if (error_out) {
    *error_out = QStringLiteral(
        "当前仅支持：cam_intrinsics / trihedral_oneshot / eye_in_hand / eye_to_hand");
  }
  return false;
}


/// \brief 构建设置页并桥接 Launcher 面板
QWidget *MainWindow::build_setup_page() {
  // —— Launcher 配置 + 就绪条 ——
  // Tier4-style Launcher configuration: dense grouped params + compact readiness.
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 12, 20, 8);
  root->setSpacing(10);

  launcher_panel_ = new LauncherConfigPanel(page);
  connect(
      launcher_panel_, &LauncherConfigPanel::source_mode_changed, this,
      &MainWindow::on_source_mode_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::image_topic_changed, this,
      &MainWindow::on_topic_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::refresh_topics_clicked, this,
      &MainWindow::refresh_topic_list);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_image_dir_clicked, this,
      &MainWindow::on_browse_image_dir);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_camera_yaml_clicked, this,
      &MainWindow::on_browse_camera_yaml);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_pose_csv_clicked, this,
      &MainWindow::on_browse_pose_csv);
  connect(
      launcher_panel_, &LauncherConfigPanel::reload_yaml_clicked, this,
      &MainWindow::on_reload_default_board_config);
  connect(
      launcher_panel_, &LauncherConfigPanel::pose_source_changed, this,
      [this](int) { refresh_setup_readiness(); });

  // —— 桥接面板控件到 MainWindow 成员别名 ——
  // Alias legacy pointers to panel widgets (keep rest of MainWindow wiring).
  combo_source_mode_ = launcher_panel_->combo_source_mode();
  combo_image_topic_ = launcher_panel_->combo_image_topic();
  edit_image_dir_ = launcher_panel_->edit_image_dir();
  offline_row_ = launcher_panel_->offline_row();
  topic_row_ = launcher_panel_->topic_row();
  btn_refresh_topics_ = launcher_panel_->btn_refresh_topics();
  handeye_block_ = launcher_panel_->handeye_block();
  edit_camera_yaml_ = launcher_panel_->edit_camera_yaml();
  edit_pose_csv_ = launcher_panel_->edit_pose_csv();
  combo_pose_source_ = launcher_panel_->combo_pose_source();
  edit_base_frame_ = launcher_panel_->edit_base_frame();
  edit_gripper_frame_ = launcher_panel_->edit_gripper_frame();
  combo_handeye_method_ = launcher_panel_->combo_handeye_method();
  edit_config_path_ = launcher_panel_->edit_config_path();
  combo_target_type_ = launcher_panel_->combo_target_type();
  combo_camera_model_ = launcher_panel_->combo_camera_model();
  spin_min_views_ = launcher_panel_->spin_min_views();
  if (combo_target_type_ != nullptr) {
    connect(
        combo_target_type_, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { refresh_setup_readiness(); });
  }
  if (combo_camera_model_ != nullptr) {
    connect(
        combo_camera_model_, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { refresh_setup_readiness(); });
  }
  if (spin_min_views_ != nullptr) {
    connect(
        spin_min_views_, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { refresh_setup_readiness(); });
  }

  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(launcher_panel_);
  auto *params_panel = make_panel(QStringLiteral("Launcher configuration"), scroll);

  setup_check_list_ = new QListWidget;
  setup_check_list_->setObjectName(QStringLiteral("ReadyCheckList"));
  setup_check_list_->setSelectionMode(QAbstractItemView::NoSelection);
  setup_check_list_->setFocusPolicy(Qt::NoFocus);
  setup_check_list_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setup_check_list_->setMaximumHeight(120);
  setup_check_list_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

  auto *ready_strip = new QFrame;
  ready_strip->setObjectName(QStringLiteral("ReadyStrip"));
  auto *ready_lay = new QHBoxLayout(ready_strip);
  ready_lay->setContentsMargins(12, 8, 12, 8);
  ready_lay->setSpacing(12);
  auto *ready_left = new QVBoxLayout;
  ready_left->setSpacing(4);
  ready_left->addWidget(make_label(
      QStringLiteral("就绪检查"), QStringLiteral("SectionTitle"), ready_strip));
  ready_left->addWidget(setup_check_list_, 1);
  ready_lay->addLayout(ready_left, 1);

  auto *nav = new QVBoxLayout;
  nav->setSpacing(8);
  auto *back = new QPushButton(QStringLiteral("返回首页"), ready_strip);
  back->setObjectName(QStringLiteral("GhostButton"));
  connect(back, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  btn_start_session_ = new QPushButton(QStringLiteral("开始会话 / Launch"), ready_strip);
  btn_start_session_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_start_session_->setEnabled(false);
  connect(btn_start_session_, &QPushButton::clicked, this, &MainWindow::on_start_session);
  nav->addStretch(1);
  nav->addWidget(back);
  nav->addWidget(btn_start_session_);
  ready_lay->addLayout(nav);

  root->addWidget(params_panel, 1);
  root->addWidget(ready_strip, 0);

  apply_board_config_from_package();
  {
    const std::string path = core::resolve_package_config("cam_intrinsics.yaml");
    if (!path.empty()) {
      launcher_panel_->set_config_path(QString::fromStdString(path));
    }
  }
  launcher_panel_->set_calibrator_id(selected_calibrator_id_);
  refresh_setup_source_ui();
  refresh_handeye_ui();
  refresh_setup_readiness();
  return page;
}

/// \brief 构建工作台：预览、观测、采集控件
QWidget *MainWindow::build_workbench_page() {
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 8);
  root->setSpacing(10);

  auto *header = new QHBoxLayout;
  header->setSpacing(8);
  auto *titles = new QVBoxLayout;
  titles->setSpacing(2);
  titles->addWidget(
      make_label(QStringLiteral("工作台 · 采集与求解"), QStringLiteral("PageTitle"), page));
  workbench_path_label_ = make_label(
      QStringLiteral("尚未加载图片"), QStringLiteral("PageSubtitle"), page);
  titles->addWidget(workbench_path_label_);
  header->addLayout(titles, 1);

  btn_prev_ = new QPushButton(QStringLiteral("上一张"), page);
  btn_prev_->setObjectName(QStringLiteral("GhostButton"));
  btn_next_ = new QPushButton(QStringLiteral("下一张"), page);
  btn_next_->setObjectName(QStringLiteral("GhostButton"));
  btn_detect_ = new QPushButton(QStringLiteral("检测"), page);
  btn_detect_->setObjectName(QStringLiteral("GhostButton"));
  btn_capture_wb_ = new QPushButton(QStringLiteral("采集帧"), page);
  btn_capture_wb_->setObjectName(QStringLiteral("GhostButton"));
  btn_solve_wb_ = new QPushButton(QStringLiteral("求解"), page);
  btn_solve_wb_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_solve_wb_->setEnabled(false);
  chk_auto_capture_ = new QCheckBox(QStringLiteral("自动采集"), page);
  chk_auto_capture_->setChecked(true);
  chk_auto_capture_->setToolTip(
      QStringLiteral("仅实时预览时生效；冻结画面后暂停自动采集"));
  auto *to_review = new QPushButton(QStringLiteral("去复核"), page);
  to_review->setObjectName(QStringLiteral("GhostButton"));
  connect(btn_prev_, &QPushButton::clicked, this, [this]() {
    if (session_) {
      session_->set_current_index(session_->current_index() - 1);
      on_detect_and_preview();
    }
  });
  connect(btn_next_, &QPushButton::clicked, this, [this]() {
    if (session_) {
      session_->set_current_index(session_->current_index() + 1);
      on_detect_and_preview();
    }
  });
  connect(btn_detect_, &QPushButton::clicked, this, &MainWindow::on_detect_and_preview);
  connect(btn_capture_wb_, &QPushButton::clicked, this, &MainWindow::on_capture_observation);
  connect(btn_solve_wb_, &QPushButton::clicked, this, &MainWindow::on_solve);
  connect(to_review, &QPushButton::clicked, this, [this]() { go_to(PageId::Review); });
  for (auto *b : {btn_prev_, btn_next_, btn_detect_, btn_capture_wb_, btn_solve_wb_, to_review}) {
    header->addWidget(b);
  }
  header->addWidget(chk_auto_capture_);
  root->addLayout(header);

  auto *split = new QSplitter(Qt::Horizontal, page);
  split->setChildrenCollapsible(false);

  // ---- Left: preview + viz strip (Tier4: options by the image) ----
  auto *preview_col = new QWidget(split);
  auto *preview_lay = new QVBoxLayout(preview_col);
  preview_lay->setContentsMargins(0, 0, 0, 0);
  preview_lay->setSpacing(8);

  preview_view_ = new ImageViewWidget(preview_col);
  preview_view_->setMinimumSize(520, 360);
  preview_view_->set_placeholder(QStringLiteral("预览区"));
  preview_view_->set_async_refresh(true, 33);

  auto *preview_panel = new QFrame(preview_col);
  preview_panel->setObjectName(QStringLiteral("Panel"));
  auto *preview_panel_lay = new QVBoxLayout(preview_panel);
  preview_panel_lay->setContentsMargins(0, 0, 0, 0);
  preview_panel_lay->setSpacing(0);
  auto *preview_header = new QFrame(preview_panel);
  preview_header->setObjectName(QStringLiteral("PanelHeader"));
  auto *preview_header_lay = new QHBoxLayout(preview_header);
  preview_header_lay->setContentsMargins(16, 8, 12, 8);
  preview_header_lay->setSpacing(8);
  preview_title_label_ =
      make_label(QStringLiteral("实时预览"), QStringLiteral("SectionTitle"), preview_header);
  preview_header_lay->addWidget(preview_title_label_);
  preview_header_lay->addStretch(1);
  btn_preview_live_ = new QPushButton(QStringLiteral("实时"), preview_header);
  btn_preview_live_->setObjectName(QStringLiteral("ModeChip"));
  btn_preview_live_->setCheckable(true);
  btn_preview_live_->setChecked(true);
  btn_preview_live_->setToolTip(
      QStringLiteral("持续刷新话题图像并叠加检测；空格可切换"));
  btn_preview_freeze_ = new QPushButton(QStringLiteral("冻结"), preview_header);
  btn_preview_freeze_->setObjectName(QStringLiteral("ModeChip"));
  btn_preview_freeze_->setCheckable(true);
  btn_preview_freeze_->setToolTip(
      QStringLiteral("暂停画面与角点叠加，便于查看三面检测；检测/采集仍可用当前帧"));
  auto *preview_mode_group = new QButtonGroup(preview_panel);
  preview_mode_group->setExclusive(true);
  preview_mode_group->addButton(btn_preview_live_);
  preview_mode_group->addButton(btn_preview_freeze_);
  connect(btn_preview_live_, &QPushButton::clicked, this, [this]() {
    set_preview_live(true);
  });
  connect(btn_preview_freeze_, &QPushButton::clicked, this, [this]() {
    set_preview_live(false);
  });
  preview_header_lay->addWidget(btn_preview_live_);
  preview_header_lay->addWidget(btn_preview_freeze_);
  preview_panel_lay->addWidget(preview_header);
  auto *preview_body = new QWidget(preview_panel);
  auto *preview_body_lay = new QVBoxLayout(preview_body);
  preview_body_lay->setContentsMargins(12, 12, 12, 12);
  preview_body_lay->addWidget(preview_view_);
  preview_panel_lay->addWidget(preview_body, 1);
  preview_lay->addWidget(preview_panel, 1);

  auto *viz_strip = new QFrame(preview_col);
  viz_strip->setObjectName(QStringLiteral("VizStrip"));
  auto *viz_row = new QHBoxLayout(viz_strip);
  viz_row->setContentsMargins(12, 8, 12, 8);
  viz_row->setSpacing(14);
  viz_row->addWidget(make_label(
      QStringLiteral("叠加显示"), QStringLiteral("SectionTitle"), viz_strip));
  chk_viz_corners_wb_ = new QCheckBox(QStringLiteral("角点"), viz_strip);
  chk_viz_corners_wb_->setChecked(true);
  chk_viz_hull_wb_ = new QCheckBox(QStringLiteral("外轮廓"), viz_strip);
  chk_viz_hull_wb_->setChecked(true);
  chk_viz_conf_wb_ = new QCheckBox(QStringLiteral("置信度"), viz_strip);
  chk_viz_conf_wb_->setChecked(true);
  chk_viz_aruco_wb_ = new QCheckBox(QStringLiteral("ArUco"), viz_strip);
  chk_viz_aruco_wb_->setChecked(true);
  chk_viz_aruco_wb_->setToolTip(QStringLiteral("叠加检测到的 ArUco 边框与 ID"));
  viz_row->addWidget(chk_viz_corners_wb_);
  viz_row->addWidget(chk_viz_hull_wb_);
  viz_row->addWidget(chk_viz_conf_wb_);
  viz_row->addWidget(chk_viz_aruco_wb_);
  viz_row->addSpacing(8);
  viz_row->addWidget(make_label(
      QStringLiteral("标记"), QStringLiteral("Muted"), viz_strip));
  spin_viz_marker_wb_ = new QSpinBox(viz_strip);
  spin_viz_marker_wb_->setRange(1, 20);
  spin_viz_marker_wb_->setValue(4);
  spin_viz_marker_wb_->setSuffix(QStringLiteral(" px"));
  spin_viz_marker_wb_->setFixedWidth(88);
  viz_row->addWidget(spin_viz_marker_wb_);
  viz_row->addStretch(1);
  preview_lay->addWidget(viz_strip, 0);

  auto refresh_viz = [this]() {
    apply_workbench_viz_to_session();
    if (session_ == nullptr) {
      return;
    }
    // 离线或冻结时立刻重画；实时模式等下一帧
    if (session_->source_mode() != SourceMode::RosTopic || !preview_live_) {
      pending_detect_log_ = false;
      session_->request_detect(false);
    }
  };
  connect(chk_viz_corners_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_hull_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_conf_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_aruco_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(
      spin_viz_marker_wb_, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [refresh_viz](int) { refresh_viz(); });

  // ---- Middle: observation / file list (beside preview) ----
  auto *obs_col = new QWidget(split);
  obs_col->setMinimumWidth(200);
  obs_col->setMaximumWidth(360);
  auto *obs_lay = new QVBoxLayout(obs_col);
  obs_lay->setContentsMargins(0, 0, 0, 0);
  obs_lay->setSpacing(8);

  obs_list_ = new QListWidget(obs_col);
  obs_list_->setAlternatingRowColors(false);
  obs_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  obs_list_->setWordWrap(false);
  obs_list_->setTextElideMode(Qt::ElideMiddle);
  auto *obs_panel = make_panel(QStringLiteral("观测列表"), obs_list_);
  obs_lay->addWidget(obs_panel, 1);

  auto *obs_btns = new QHBoxLayout;
  obs_btns->setSpacing(8);
  auto *remove_btn = new QPushButton(QStringLiteral("删除选中"), obs_col);
  remove_btn->setObjectName(QStringLiteral("GhostButton"));
  auto *clear_btn = new QPushButton(QStringLiteral("全部清空"), obs_col);
  clear_btn->setObjectName(QStringLiteral("GhostButton"));
  connect(remove_btn, &QPushButton::clicked, this, [this]() {
    if (obs_list_ == nullptr || session_ == nullptr) {
      return;
    }
    const int row = obs_list_->currentRow();
    if (row < 0) {
      append_log(LogLevel::Warn, QStringLiteral("› 请先选中一条观测"));
      return;
    }
    session_->remove_observation(row);
    append_log(LogLevel::Info, QStringLiteral("› 已删除观测"));
  });
  connect(clear_btn, &QPushButton::clicked, this, [this]() {
    if (session_ == nullptr) {
      return;
    }
    if (session_->observation_count() <= 0) {
      append_log(LogLevel::Warn, QStringLiteral("› 观测列表已空"));
      return;
    }
    const auto ret = QMessageBox::question(
        this, QStringLiteral("清空观测"),
        QStringLiteral("确定清空全部 %1 条已采集观测？")
            .arg(session_->observation_count()),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret != QMessageBox::Yes) {
      return;
    }
    session_->clear_observations();
    append_log(LogLevel::Info, QStringLiteral("› 已清空全部观测"));
    refresh_workbench_view(false);
  });
  obs_btns->addWidget(remove_btn, 1);
  obs_btns->addWidget(clear_btn, 1);
  obs_lay->addLayout(obs_btns);

  // ---- Right rail: compact session metrics ----
  auto *right = new QWidget(split);
  right->setMinimumWidth(200);
  right->setMaximumWidth(280);
  auto *right_lay = new QVBoxLayout(right);
  right_lay->setContentsMargins(0, 0, 0, 0);
  right_lay->setSpacing(8);

  auto *metrics_host = new QWidget(right);
  auto *metrics_layout = new QVBoxLayout(metrics_host);
  metrics_layout->setContentsMargins(0, 0, 0, 0);
  metrics_layout->setSpacing(6);
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("已采集帧数"), QStringLiteral("0")));
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("检测置信度"), QStringLiteral("—")));
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("覆盖提示"), QStringLiteral("—")));
  right_lay->addWidget(make_panel(QStringLiteral("会话指标"), metrics_host), 0);
  right_lay->addStretch(1);

  split->addWidget(preview_col);
  split->addWidget(obs_col);
  split->addWidget(right);
  split->setStretchFactor(0, 6);
  split->setStretchFactor(1, 2);
  split->setStretchFactor(2, 1);
  split->setSizes({720, 280, 220});
  root->addWidget(split, 1);
  return page;
}

/// \brief 构建结果复查页
QWidget *MainWindow::build_review_page() {
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 8);
  root->setSpacing(12);

  auto *header = new QHBoxLayout;
  auto *titles = new QVBoxLayout;
  titles->addWidget(
      make_label(QStringLiteral("复核与导出"), QStringLiteral("PageTitle"), page));
  titles->addWidget(make_label(
      QStringLiteral("检查重投影误差与内参矩阵，确认后导出结果文件夹（图像 + YAML + 配置）"),
      QStringLiteral("PageSubtitle"), page));
  header->addLayout(titles, 1);
  auto *export_btn = new QPushButton(QStringLiteral("导出结果"), page);
  export_btn->setObjectName(QStringLiteral("PrimaryButton"));
  connect(export_btn, &QPushButton::clicked, this, &MainWindow::on_export_yaml);
  auto *done = new QPushButton(QStringLiteral("完成"), page);
  done->setObjectName(QStringLiteral("GhostButton"));
  connect(done, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  header->addWidget(export_btn);
  header->addWidget(done);
  root->addLayout(header);

  auto *metrics_row = new QHBoxLayout;
  metrics_row->addWidget(make_metric_card(QStringLiteral("重投影 RMSE"), QStringLiteral("— px")));
  metrics_row->addWidget(make_metric_card(QStringLiteral("有效观测"), QStringLiteral("—")));
  metrics_row->addWidget(make_metric_card(QStringLiteral("图像尺寸"), QStringLiteral("—")));
  root->addLayout(metrics_row);

  review_text_ = new QTextEdit(page);
  review_text_->setReadOnly(true);
  review_text_->setPlainText(QStringLiteral("求解后显示 K / D / RMSE。"));
  root->addWidget(make_panel(QStringLiteral("内参结果"), review_text_), 1);
  return page;
}

}  // namespace gui
}  // namespace hs_calib
