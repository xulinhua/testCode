#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/widgets/review_charts_widget.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_workbench_panels.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_parameter_dialog.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/io/board_config_yaml.hpp"

#include <functional>
#include <memory>
#include <algorithm>

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
#include <QDesktopServices>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTabWidget>
#include <QStyle>
#include <QTextEdit>
#include <QTimer>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
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
using hs_calib::gui::window_detail::make_toolbar_icon;
using hs_calib::gui::window_detail::glyph_for_task_id;
using hs_calib::gui::window_detail::glyph_for_category;
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

/// \brief 大号指标卡片（仅数值；名称作 tooltip，留给结果摘要更多高度）
QFrame *MainWindow::make_metric_card(const QString &name, const QString &value) {
  auto *card = new QFrame;
  card->setObjectName(QStringLiteral("MetricCard"));
  card->setToolTip(name);
  auto *layout = new QVBoxLayout(card);
  layout->setContentsMargins(16, 12, 16, 12);
  layout->setSpacing(0);
  auto *v = make_label(value, QStringLiteral("MetricValue"), card);
  v->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  layout->addWidget(v, 1);
  if (name == QStringLiteral("已采集帧数")) {
    metric_frames_ = v;
  } else if (name == QStringLiteral("检测置信度")) {
    metric_detect_ = v;
  } else if (name == QStringLiteral("重投影")) {
    metric_reproj_ = v;
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
  layout->setContentsMargins(8, 6, 8, 6);
  layout->setSpacing(2);
  auto *v = make_label(value, QStringLiteral("MetricValue"), card);
  auto *n = make_label(name, QStringLiteral("MetricName"), card);
  v->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  n->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  v->setWordWrap(false);
  n->setWordWrap(false);
  layout->addWidget(v);
  layout->addWidget(n);
  if (name == QStringLiteral("已采集帧数")) {
    metric_frames_ = v;
  } else if (name == QStringLiteral("检测置信度")) {
    metric_detect_ = v;
  } else if (name == QStringLiteral("重投影")) {
    metric_reproj_ = v;
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
  const bool lab_task =
      detect_lab_mode_from_task_id(selected_calibrator_id_) != DetectLabMode::None;
  if (launcher_panel_ != nullptr) {
    launcher_panel_->set_calibrator_id(
        lab_task ? QStringLiteral("cam_intrinsics") : selected_calibrator_id_);
  }
  if (session_) {
    session_->sync_detect_lab_mode_from_task_id(selected_calibrator_id_);
    if (lab_task) {
      session_->set_calibrator_id(QStringLiteral("cam_intrinsics"));
      session_->set_detect_lab_mode(
          detect_lab_mode_from_task_id(selected_calibrator_id_));
    } else {
      session_->set_detect_lab_mode(DetectLabMode::None);
      session_->set_calibrator_id(selected_calibrator_id_);
    }
  }
  refresh_handeye_ui();
  refresh_setup_readiness();
  update_home_selection_label();
  if (btn_home_next_ != nullptr) {
    btn_home_next_->setText(
        lab_task ? QStringLiteral("下一步：数据源设置（调试）")
                 : QStringLiteral("下一步：数据源设置"));
  }
  append_log(LogLevel::Info, QStringLiteral("› 选定标定器：%1").arg(selected_calibrator_id_));
  refresh_status_task();
}

/// \brief 创建任务画廊卡（工程标定 / 检测调试共用）
QFrame *MainWindow::make_calib_tile(
    const QString &title, const QString &id, bool implemented,
    const QString &subtitle, bool gallery) {
  auto *tile = new QFrame;
  tile->setObjectName(QStringLiteral("CalibTile"));
  tile->setCursor(implemented ? Qt::PointingHandCursor : Qt::ForbiddenCursor);
  tile->setProperty("calibrator_id", id);
  tile->setProperty("calibrator_title", title);
  tile->setProperty("selected", false);
  tile->setProperty("planned", !implemented);
  tile->setProperty("implemented", implemented);
  tile->setProperty("gallery", true);  // 统一画廊样式
  (void)gallery;

  const QString title_text =
      implemented ? title : (title + QStringLiteral(" · 即将推出"));

  tile->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  tile->setMinimumHeight(148);
  auto *col = new QVBoxLayout(tile);
  col->setContentsMargins(16, 16, 16, 16);
  col->setSpacing(10);

  const QColor ink(theme_icon_ink(theme_id_));
  const QColor accent(theme_icon_accent(theme_id_));
  auto *glyph = new QLabel(tile);
  glyph->setObjectName(QStringLiteral("CalibGalleryGlyph"));
  glyph->setFixedHeight(52);
  glyph->setAlignment(Qt::AlignCenter);
  glyph->setPixmap(
      make_toolbar_icon(glyph_for_task_id(id), ink, accent).pixmap(QSize(36, 36)));

  auto *title_lbl = make_label(title_text, QStringLiteral("CalibTileTitle"), tile);
  title_lbl->setWordWrap(true);
  col->addWidget(glyph);
  col->addWidget(title_lbl);
  if (!subtitle.isEmpty()) {
    auto *sub = make_label(subtitle, QStringLiteral("CalibTileSubtitle"), tile);
    sub->setWordWrap(true);
    col->addWidget(sub, 1);
  } else {
    col->addStretch(1);
  }

  tile->installEventFilter(
      new TileClickFilter([this, tile]() { select_calib_tile(tile); }, tile));
  return tile;
}

/// \brief 首页产品线切换：工程标定 / 检测调试
void MainWindow::on_home_product_line_changed(int line) {
  home_product_line_ = line;
  refresh_home_category_chips();
  if (home_product_line_ == 1) {
    home_category_ = 4;
  } else if (home_category_ >= 4) {
    home_category_ = 0;
    if (home_category_group_ != nullptr) {
      if (QAbstractButton *b = home_category_group_->button(0)) {
        b->setChecked(true);
      }
    }
  }
  refresh_home_calibrator_grid();
}

void MainWindow::refresh_home_category_chips() {
  if (home_category_row_host_ == nullptr) {
    return;
  }
  // 隐藏整块分类面板（含标题），调试模式全宽画廊
  QWidget *panel = home_category_row_host_->parentWidget();
  if (panel != nullptr) {
    panel->setVisible(home_product_line_ == 0);
  } else {
    home_category_row_host_->setVisible(home_product_line_ == 0);
  }
}

/// \brief 首页分类切换（仅工程标定产品线）
void MainWindow::on_home_category_changed(int category) {
  if (home_product_line_ != 0) {
    return;
  }
  home_category_ = category;
  refresh_home_calibrator_grid();
}

/// \brief 按产品线刷新任务画廊
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
    const char *id;
    const char *subtitle;
    bool implemented;
  };

  QVector<Entry> entries;
  if (home_product_line_ == 1) {
    entries = {
        {"标定板类型识别", "detect_lab_identify",
         "不填尺寸，自动试探板类型并给出 Top-K", true},
        {"局部特征检测", "detect_lab",
         "已知类型下尽量检出残缺视野中的特征", true},
        {"完整标定板检测", "detect_lab_full",
         "只认整面及格，适合采集前验收", true},
    };
  } else {
    switch (home_category_) {
      case 1:
        entries = {
            {"眼在手上", "eye_in_hand", "末端相机：求解 gripper→camera", true},
            {"眼在手外", "eye_to_hand", "固定相机：求解 base→camera", true},
        };
        break;
      case 2:
        entries = {
            {"双目相对外参", "stereo_extrinsics",
             "固定内参，求解左右相机相对位姿", true},
            {"直角三面标定", "trihedral_oneshot",
             "正交三面靶，单帧/少帧内参外参", true},
            {"相机–激光", "cam_lidar", "相机与激光雷达外参（规划中）", false},
        };
        break;
      case 3:
        entries = {
            {"传感器套件联合", "sensor_kit_bundle",
             "多传感器联合标定（规划中）", false},
            {"时间偏移", "time_offset", "传感器时间同步偏移（规划中）", false},
        };
        break;
      case 0:
      default:
        entries = {
            {"单目内参", "cam_intrinsics", "多视角求解相机 K 与畸变", true},
            {"直角三面内参", "trihedral_oneshot",
             "三面靶单帧/少帧内参求解", true},
            {"双目内参", "stereo_intrinsics",
             "左右相机分别求解内参", true},
        };
        break;
    }
  }

  // 重置列拉伸
  for (int c = 0; c < 4; ++c) {
    home_tile_grid_->setColumnStretch(c, 0);
  }

  QFrame *first_impl = nullptr;
  const int n = entries.size();
  const int cols = (n <= 2) ? n : 3;
  for (int i = 0; i < n; ++i) {
    const auto &e = entries[i];
    auto *tile = make_calib_tile(
        QString::fromUtf8(e.title), QString::fromUtf8(e.id), e.implemented,
        QString::fromUtf8(e.subtitle), true);
    const int row = i / cols;
    const int col = i % cols;
    home_tile_grid_->addWidget(tile, row, col);
    home_tile_grid_->setColumnStretch(col, 1);
    if (e.implemented && first_impl == nullptr) {
      first_impl = tile;
    }
  }
  home_tile_grid_->setRowStretch((n + cols - 1) / cols, 1);

  if (first_impl != nullptr) {
    select_calib_tile(first_impl);
  } else if (home_selection_ != nullptr) {
    selected_calibrator_id_.clear();
    update_home_selection_label();
  }
}

/// \brief 构建首页
QWidget *MainWindow::build_home_page() {
  auto *page = new QWidget;
  auto *root = new QHBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 8);
  root->setSpacing(16);

  // —— 左侧：项目目录 ——
  auto *project_host = new QWidget(page);
  auto *project_lay = new QVBoxLayout(project_host);
  project_lay->setContentsMargins(0, 0, 0, 0);
  project_lay->setSpacing(8);

  project_list_ = new QListWidget(project_host);
  project_list_->setObjectName(QStringLiteral("ProjectList"));
  project_list_->setMinimumHeight(160);
  project_list_->setContextMenuPolicy(Qt::CustomContextMenu);

  auto *proj_btns = new QHBoxLayout;
  proj_btns->setContentsMargins(0, 0, 0, 0);
  proj_btns->setSpacing(6);
  auto *btn_new = new QPushButton(QStringLiteral("新建…"), project_host);
  auto *btn_import = new QPushButton(QStringLiteral("导入项目…"), project_host);
  for (auto *b : {btn_new, btn_import}) {
    b->setObjectName(QStringLiteral("CompactButton"));
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    b->setMinimumWidth(0);
  }
  proj_btns->addWidget(btn_new);
  proj_btns->addWidget(btn_import);

  project_lay->addWidget(project_list_, 1);
  project_lay->addLayout(proj_btns);

  auto *project_panel = make_panel(QStringLiteral("项目工作区"), project_host);
  project_panel->setMinimumWidth(240);
  project_panel->setMaximumWidth(360);

  connect(project_list_, &QListWidget::currentRowChanged, this, [this](int) {
    on_project_selection_changed();
  });
  connect(
      project_list_, &QListWidget::customContextMenuRequested, this,
      &MainWindow::on_project_context_menu);
  connect(btn_new, &QPushButton::clicked, this, &MainWindow::on_new_project);
  connect(btn_import, &QPushButton::clicked, this, &MainWindow::on_import_project);

  auto *right = new QWidget;
  auto *right_layout = new QVBoxLayout(right);
  right_layout->setContentsMargins(0, 0, 0, 0);
  right_layout->setSpacing(8);
  right_layout->addWidget(
      make_label(QStringLiteral("选择任务"), QStringLiteral("PageTitle"), right));
  right_layout->addWidget(make_label(
      QStringLiteral("先选产品线与项目，再选具体任务；目录含 config / images / results。"),
      QStringLiteral("PageSubtitle"), right));

  // —— 顶层：工程标定 / 检测调试 ——
  auto *product_row = new QHBoxLayout;
  product_row->setSpacing(8);
  home_product_group_ = new QButtonGroup(page);
  home_product_group_->setExclusive(true);
  auto *btn_formal = new QPushButton(QStringLiteral("工程标定"), right);
  auto *btn_debug = new QPushButton(QStringLiteral("检测调试"), right);
  for (auto *btn : {btn_formal, btn_debug}) {
    btn->setObjectName(QStringLiteral("ProductLineChip"));
    btn->setCheckable(true);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setMinimumWidth(0);
    btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  }
  home_product_group_->addButton(btn_formal, 0);
  home_product_group_->addButton(btn_debug, 1);
  product_row->addWidget(btn_formal);
  product_row->addWidget(btn_debug);
  product_row->addStretch(1);
  right_layout->addLayout(product_row);
  connect(
      home_product_group_, QOverload<int>::of(&QButtonGroup::idClicked), this,
      &MainWindow::on_home_product_line_changed);

  // —— 任务区：左侧大气分类面板 + 右侧画廊 ——
  home_task_split_ = new QWidget(right);
  auto *split_lay = new QHBoxLayout(home_task_split_);
  split_lay->setContentsMargins(0, 0, 0, 0);
  split_lay->setSpacing(16);

  auto *rail_panel = new QFrame(home_task_split_);
  rail_panel->setObjectName(QStringLiteral("HomeCategoryPanel"));
  auto *rail_lay = new QVBoxLayout(rail_panel);
  rail_lay->setContentsMargins(14, 16, 14, 16);
  rail_lay->setSpacing(10);
  auto *rail_title =
      make_label(QStringLiteral("任务分类"), QStringLiteral("HomeCategoryTitle"), rail_panel);
  rail_lay->addWidget(rail_title);

  home_category_row_host_ = new QWidget(rail_panel);
  home_category_row_host_->setObjectName(QStringLiteral("HomeCategoryRail"));
  auto *cat_col = new QVBoxLayout(home_category_row_host_);
  cat_col->setContentsMargins(0, 8, 0, 0);
  cat_col->setSpacing(8);
  home_category_group_ = new QButtonGroup(page);
  home_category_group_->setExclusive(true);
  const char *cats[] = {"内参", "手眼", "外参", "多传感器"};
  const char *cat_hints[] = {
      "相机内部参数", "机械臂与相机", "传感器相对位姿", "系统级联合"};
  const QColor ink(theme_icon_ink(theme_id_));
  const QColor accent(theme_icon_accent(theme_id_));
  for (int i = 0; i < 4; ++i) {
    auto *btn = new QToolButton(home_category_row_host_);
    btn->setObjectName(QStringLiteral("CategoryNav"));
    btn->setCheckable(true);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    btn->setIcon(make_toolbar_icon(glyph_for_category(i), ink, accent));
    btn->setIconSize(QSize(22, 22));
    btn->setText(
        QStringLiteral("%1\n%2")
            .arg(QString::fromUtf8(cats[i]), QString::fromUtf8(cat_hints[i])));
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    btn->setMinimumHeight(58);
    home_category_group_->addButton(btn, i);
    cat_col->addWidget(btn);
  }
  cat_col->addStretch(1);
  rail_lay->addWidget(home_category_row_host_, 1);
  split_lay->addWidget(rail_panel, 0);
  connect(
      home_category_group_, QOverload<int>::of(&QButtonGroup::idClicked), this,
      &MainWindow::on_home_category_changed);

  auto *scroll = new QScrollArea(home_task_split_);
  scroll->setObjectName(QStringLiteral("HomeTaskScroll"));
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->viewport()->setAutoFillBackground(false);
  scroll->viewport()->setStyleSheet(QStringLiteral("background: transparent;"));

  home_tile_host_ = new QWidget;
  home_tile_host_->setObjectName(QStringLiteral("HomeTaskHost"));
  home_tile_grid_ = new QGridLayout(home_tile_host_);
  home_tile_grid_->setContentsMargins(0, 4, 0, 4);
  home_tile_grid_->setHorizontalSpacing(12);
  home_tile_grid_->setVerticalSpacing(8);
  home_tile_grid_->setAlignment(Qt::AlignTop);
  scroll->setWidget(home_tile_host_);
  split_lay->addWidget(scroll, 1);
  right_layout->addWidget(home_task_split_, 1);

  auto *footer = new QHBoxLayout;
  home_selection_ = make_label(QString(), QStringLiteral("Muted"), right);
  footer->addWidget(home_selection_, 1);
  auto *next = new QPushButton(QStringLiteral("下一步：数据源设置"), right);
  next->setObjectName(QStringLiteral("PrimaryButton"));
  btn_home_next_ = next;
  connect(next, &QPushButton::clicked, this, [this]() {
    QString err;
    if (!ensure_implemented_calibrator(&err)) {
      append_log(LogLevel::Error, QStringLiteral("› %1").arg(err));
      return;
    }
    apply_selected_project_to_setup();
    // 检测调试也先设图像源，再进检测台
    go_to(PageId::DataSource);
  });
  connect(home_category_group_, QOverload<int>::of(&QButtonGroup::idClicked), this,
          [this](int) {
            if (btn_home_next_ != nullptr) {
              btn_home_next_->setText(QStringLiteral("下一步：数据源设置"));
            }
          });
  footer->addWidget(next);
  right_layout->addLayout(footer);

  root->addWidget(project_panel);
  root->addWidget(right, 1);

  btn_formal->setChecked(true);
  home_product_line_ = 0;
  if (QAbstractButton *b = home_category_group_->button(0)) {
    b->setChecked(true);
  }
  home_category_ = 0;
  refresh_home_category_chips();
  refresh_project_list();
  refresh_home_calibrator_grid();
  return page;
}

QString MainWindow::selected_project_display_name() const {
  if (const ProjectInfo *p = project_catalog_.find(selected_project_id_)) {
    return p->display_name.isEmpty() ? p->id : p->display_name;
  }
  return selected_project_id_.isEmpty() ? QStringLiteral("—") : selected_project_id_;
}

void MainWindow::update_home_selection_label() {
  if (home_selection_ == nullptr) {
    return;
  }
  if (selected_calibrator_id_.isEmpty()) {
    home_selection_->setText(
        QStringLiteral("当前项目：%1 · 本类别暂无已接通标定器")
            .arg(selected_project_display_name()));
    return;
  }
  QString title = selected_calibrator_id_;
  if (selected_tile_ != nullptr) {
    title = selected_tile_->property("calibrator_title").toString();
  }
  home_selection_->setText(
      QStringLiteral("当前：%1  ·  %2（%3）")
          .arg(selected_project_display_name(), title, selected_calibrator_id_));
}

void MainWindow::refresh_project_list() {
  project_catalog_.reload();
  if (project_list_ == nullptr) {
    return;
  }
  project_list_->blockSignals(true);
  project_list_->clear();
  int select_row = 0;
  for (int i = 0; i < project_catalog_.projects().size(); ++i) {
    const auto &p = project_catalog_.projects()[i];
    auto *item = new QListWidgetItem(p.display_name);
    item->setData(Qt::UserRole, p.id);
    if (!p.root_path.isEmpty()) {
      item->setToolTip(p.root_path);
    }
    project_list_->addItem(item);
    if (p.id == selected_project_id_) {
      select_row = i;
    }
  }
  if (project_list_->count() > 0) {
    project_list_->setCurrentRow(select_row);
  }
  project_list_->blockSignals(false);
  on_project_selection_changed();
}

bool MainWindow::ensure_project_workspace_open(QString *error_out) {
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  if (p == nullptr) {
    if (error_out) {
      *error_out = QStringLiteral("未选择项目");
    }
    return false;
  }
  if (p->is_folder_project && !p->root_path.isEmpty()) {
    if (project_workspace_.is_open() && project_workspace_.root_path() == p->root_path) {
      return true;
    }
    return project_workspace_.open(p->root_path, error_out);
  }
  if (error_out) {
    *error_out = QStringLiteral("所选不是有效文件夹项目");
  }
  return false;
}

void MainWindow::on_project_selection_changed() {
  if (project_list_ == nullptr) {
    return;
  }
  auto *item = project_list_->currentItem();
  if (item == nullptr) {
    return;
  }
  selected_project_id_ = item->data(Qt::UserRole).toString();
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  project_workspace_.close();
  if (p != nullptr && p->is_folder_project && !p->root_path.isEmpty()) {
    QString err;
    if (!project_workspace_.open(p->root_path, &err)) {
      append_log(LogLevel::Warn, QStringLiteral("› 打开项目失败：%1").arg(err));
    }
  }
  update_home_selection_label();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 选定项目：%1").arg(selected_project_display_name()));
}

void MainWindow::apply_selected_project_to_setup() {
  QString err;
  if (!ensure_project_workspace_open(&err)) {
    append_log(LogLevel::Warn, QStringLiteral("› 项目工作区：%1").arg(err));
  }
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  if (p == nullptr && project_workspace_.is_open()) {
    p = &project_workspace_.meta();
  }
  if (p == nullptr) {
    return;
  }
  if (launcher_panel_ != nullptr) {
    launcher_panel_->apply_project_frames(
        p->parent_frame, p->child_frame, p->base_frame, p->gripper_frame,
        p->image_frame, p->camera_link_frame);
    if (project_workspace_.is_open()) {
      const QString img = project_workspace_.preferred_image_dir();
      if (!img.isEmpty() && launcher_panel_->edit_image_dir() != nullptr) {
        launcher_panel_->edit_image_dir()->setText(img);
      }
      // 默认导出到项目 results/
      if (launcher_panel_->edit_export_path() != nullptr) {
        launcher_panel_->edit_export_path()->setText(project_workspace_.results_dir());
      }
    }
  }
  if (session_ != nullptr) {
    session_->set_handeye_frames(p->base_frame, p->gripper_frame);
  }
  if (project_workspace_.is_open() && !selected_calibrator_id_.isEmpty()) {
    project_workspace_.meta().last_calibrator_id = selected_calibrator_id_;
    project_workspace_.save_meta(nullptr);
  }
}

void MainWindow::on_import_project_images() {
  QString err;
  if (!ensure_project_workspace_open(&err)) {
    append_log(LogLevel::Error, QStringLiteral("› %1").arg(err));
    return;
  }
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择要导入的图片目录"), project_workspace_.images_dir());
  if (dir.isEmpty()) {
    return;
  }
  bool ok = false;
  const QString sub = QInputDialog::getText(
                          this, QStringLiteral("导入图像"),
                          QStringLiteral("存到 images/ 下的子目录名："), QLineEdit::Normal,
                          QFileInfo(dir).fileName(), &ok)
                          .trimmed();
  if (!ok) {
    return;
  }
  if (!project_workspace_.import_image_directory(dir, sub, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 导入失败：%1").arg(err));
    return;
  }
  project_workspace_.meta().default_image_subdir = sub;
  project_workspace_.save_meta(nullptr);
  on_project_selection_changed();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已导入图像目录 → %1/images/%2")
          .arg(project_workspace_.root_path(), sub));
}

void MainWindow::on_import_project() {
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择要导入的项目文件夹（需含 project.yaml）"),
      ProjectCatalog::user_projects_dir());
  if (dir.isEmpty()) {
    return;
  }
  QString root;
  QString err;
  if (!project_catalog_.import_user_project(dir, QString(), &root, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 导入项目失败：%1").arg(err));
    QMessageBox::warning(this, QStringLiteral("导入项目"), err);
    return;
  }
  ProjectWorkspace ws;
  if (ws.open(root, nullptr)) {
    selected_project_id_ = ws.meta().id;
  }
  refresh_project_list();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已导入项目 → %1").arg(root));
}

void MainWindow::on_open_project_dir() {
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  if (p != nullptr && p->is_folder_project && !p->root_path.isEmpty()) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(p->root_path));
    return;
  }
  if (project_workspace_.is_open()) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(project_workspace_.root_path()));
    return;
  }
  append_log(LogLevel::Warn, QStringLiteral("› 当前项不是用户文件夹项目，无法打开目录"));
}

void MainWindow::on_open_projects_root_dir() {
  ProjectCatalog::open_user_projects_dir();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已打开项目总目录：%1").arg(ProjectCatalog::user_projects_dir()));
}

void MainWindow::on_configure_project() {
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  if (p == nullptr) {
    append_log(LogLevel::Warn, QStringLiteral("› 未选择项目"));
    return;
  }
  if (!p->is_folder_project || p->root_path.isEmpty()) {
    append_log(LogLevel::Warn, QStringLiteral("› 请先选择有效的文件夹项目"));
    return;
  }
  QString err;
  if (!project_workspace_.open(p->root_path, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 打开项目失败：%1").arg(err));
    return;
  }

  ProjectInfo &meta = project_workspace_.meta();
  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("配置项目信息"));
  dlg.resize(480, 420);
  auto *form = new QFormLayout(&dlg);
  auto *edit_name = new QLineEdit(meta.display_name, &dlg);
  auto *edit_desc = new QLineEdit(meta.description, &dlg);
  auto *edit_notes = new QTextEdit(&dlg);
  edit_notes->setPlainText(meta.notes);
  edit_notes->setMaximumHeight(80);
  auto *edit_base = new QLineEdit(meta.base_frame, &dlg);
  auto *edit_gripper = new QLineEdit(meta.gripper_frame, &dlg);
  auto *edit_cam_link = new QLineEdit(meta.camera_link_frame, &dlg);
  auto *edit_optical = new QLineEdit(meta.image_frame, &dlg);
  auto *edit_parent = new QLineEdit(meta.parent_frame, &dlg);
  auto *edit_child = new QLineEdit(meta.child_frame, &dlg);
  auto *edit_rec = new QLineEdit(meta.recommended_calibrators.join(QStringLiteral(", ")), &dlg);
  edit_rec->setPlaceholderText(QStringLiteral("cam_intrinsics, eye_in_hand, …"));

  form->addRow(QStringLiteral("显示名称"), edit_name);
  form->addRow(QStringLiteral("说明"), edit_desc);
  form->addRow(QStringLiteral("备注"), edit_notes);
  form->addRow(QStringLiteral("base_frame"), edit_base);
  form->addRow(QStringLiteral("gripper_frame"), edit_gripper);
  form->addRow(QStringLiteral("camera_link"), edit_cam_link);
  form->addRow(QStringLiteral("optical_frame"), edit_optical);
  form->addRow(QStringLiteral("parent_frame"), edit_parent);
  form->addRow(QStringLiteral("child_frame"), edit_child);
  form->addRow(QStringLiteral("推荐标定器"), edit_rec);

  auto *buttons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  form->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted) {
    return;
  }

  meta.display_name = edit_name->text().trimmed();
  meta.description = edit_desc->text().trimmed();
  meta.notes = edit_notes->toPlainText().trimmed();
  meta.base_frame = edit_base->text().trimmed();
  meta.gripper_frame = edit_gripper->text().trimmed();
  meta.camera_link_frame = edit_cam_link->text().trimmed();
  meta.image_frame = edit_optical->text().trimmed();
  meta.parent_frame = edit_parent->text().trimmed();
  meta.child_frame = edit_child->text().trimmed();
  QStringList rec;
  for (const QString &part : edit_rec->text().split(QLatin1Char(','))) {
    const QString t = part.trimmed();
    if (!t.isEmpty()) {
      rec.push_back(t);
    }
  }
  meta.recommended_calibrators = rec;
  if (!project_workspace_.save_meta(&err)) {
    append_log(LogLevel::Error, QStringLiteral("› 保存项目配置失败：%1").arg(err));
    return;
  }
  refresh_project_list();
  append_log(LogLevel::Info, QStringLiteral("› 已保存项目配置：%1").arg(meta.display_name));
}

void MainWindow::on_delete_project() {
  const ProjectInfo *p = project_catalog_.find(selected_project_id_);
  if (p == nullptr) {
    append_log(LogLevel::Warn, QStringLiteral("› 未选择项目"));
    return;
  }
  if (!p->is_folder_project) {
    append_log(LogLevel::Warn, QStringLiteral("› 请先选择有效的文件夹项目"));
    return;
  }
  const QString path = p->root_path;
  const QString name = p->display_name.isEmpty() ? p->id : p->display_name;
  const auto ret = QMessageBox::warning(
      this, QStringLiteral("删除项目"),
      QStringLiteral("将删除整个项目文件夹（含 config / images / results）：\n\n%1\n%2\n\n"
                     "此操作不可轻易恢复，确定继续？")
          .arg(name, path),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (ret != QMessageBox::Yes) {
    return;
  }
  const QString deleted_id = p->id;
  project_workspace_.close();
  QString err;
  if (!project_catalog_.delete_user_project(deleted_id, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 删除失败：%1").arg(err));
    QMessageBox::warning(this, QStringLiteral("删除项目"), err);
    return;
  }
  if (selected_project_id_ == deleted_id) {
    selected_project_id_.clear();
  }
  refresh_project_list();
  append_log(LogLevel::Info, QStringLiteral("› 已删除项目：%1").arg(name));
}

void MainWindow::on_project_context_menu(const QPoint &pos) {
  if (project_list_ == nullptr) {
    return;
  }
  QListWidgetItem *item = project_list_->itemAt(pos);
  QMenu menu(this);

  if (item != nullptr) {
    project_list_->setCurrentItem(item);
    const QString id = item->data(Qt::UserRole).toString();
    const ProjectInfo *p = project_catalog_.find(id);
    const bool is_user = p != nullptr && p->is_folder_project && !p->is_template;

    if (is_user) {
      menu.addAction(QStringLiteral("配置…"), this, &MainWindow::on_configure_project);
      menu.addAction(QStringLiteral("导入图像…"), this, &MainWindow::on_import_project_images);
      menu.addAction(QStringLiteral("打开目录"), this, &MainWindow::on_open_project_dir);
      menu.addSeparator();
      menu.addAction(QStringLiteral("删除项目…"), this, &MainWindow::on_delete_project);
    }
    menu.addSeparator();
  }

  menu.addAction(QStringLiteral("刷新列表"), this, [this]() {
    refresh_project_list();
    append_log(LogLevel::Info, QStringLiteral("› 已刷新项目列表"));
  });
  menu.addAction(QStringLiteral("打开项目总目录"), this, &MainWindow::on_open_projects_root_dir);
  menu.exec(project_list_->mapToGlobal(pos));
}

void MainWindow::on_new_project() {
  bool ok = false;
  const QString id = QInputDialog::getText(
                         this, QStringLiteral("新建文件夹项目"),
                         QStringLiteral("项目 ID（字母数字/_/-）："), QLineEdit::Normal,
                         QStringLiteral("my_cell"), &ok)
                         .trimmed();
  if (!ok || id.isEmpty()) {
    return;
  }
  const QString name = QInputDialog::getText(
                           this, QStringLiteral("新建文件夹项目"), QStringLiteral("显示名称："),
                           QLineEdit::Normal, id, &ok)
                           .trimmed();
  if (!ok) {
    return;
  }
  ProjectInfo info;
  info.id = id;
  info.display_name = name.isEmpty() ? id : name;
  info.description = QStringLiteral("用户文件夹项目（config / images / results）");
  info.notes = QStringLiteral("根目录：") + ProjectCatalog::user_projects_dir();
  if (const ProjectInfo *cur = project_catalog_.find(selected_project_id_)) {
    info.parent_frame = cur->parent_frame;
    info.child_frame = cur->child_frame;
    info.base_frame = cur->base_frame;
    info.gripper_frame = cur->gripper_frame;
    info.image_frame = cur->image_frame;
    info.camera_link_frame = cur->camera_link_frame;
    info.recommended_calibrators = cur->recommended_calibrators;
  }
  QString err;
  if (!project_catalog_.create_user_project(info, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 新建项目失败：%1").arg(err));
    return;
  }
  selected_project_id_ = id;
  refresh_project_list();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已创建文件夹项目 %1 → %2/%1")
          .arg(id, ProjectCatalog::user_projects_dir()));
}

// ===== 流程门禁 =====

/// \brief 校验当前标定器已实现
bool MainWindow::ensure_implemented_calibrator(QString *error_out) const {
  if (selected_calibrator_id_ == QStringLiteral("cam_intrinsics") ||
      selected_calibrator_id_ == QStringLiteral("stereo_intrinsics") ||
      selected_calibrator_id_ == QStringLiteral("stereo_extrinsics") ||
      selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
      selected_calibrator_id_ == QStringLiteral("eye_to_hand") ||
      selected_calibrator_id_ == QStringLiteral("trihedral_oneshot") ||
      selected_calibrator_id_ == QStringLiteral("detect_lab") ||
      selected_calibrator_id_ == QStringLiteral("detect_lab_full") ||
      selected_calibrator_id_ == QStringLiteral("detect_lab_identify")) {
    return true;
  }
  if (error_out) {
    *error_out = QStringLiteral(
        "当前仅支持：cam_intrinsics / stereo_intrinsics / stereo_extrinsics / "
        "trihedral_oneshot / eye_in_hand / eye_to_hand / detect_lab_identify / "
        "detect_lab / detect_lab_full");
  }
  return false;
}


/// \brief 一次性桥接 Launcher 面板信号与控件别名
void MainWindow::wire_launcher_panel_once() {
  if (launcher_wired_ || launcher_panel_ == nullptr) {
    return;
  }
  launcher_wired_ = true;
  connect(
      launcher_panel_, &LauncherConfigPanel::source_mode_changed, this,
      &MainWindow::on_source_mode_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::image_topic_changed, this,
      &MainWindow::on_topic_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::camera_info_topic_changed, this,
      &MainWindow::on_camera_info_topic_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::intrinsics_source_changed, this,
      &MainWindow::on_intrinsics_source_changed);
  connect(
      launcher_panel_, &LauncherConfigPanel::refresh_topics_clicked, this,
      &MainWindow::refresh_topic_list);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_image_dir_clicked, this,
      &MainWindow::on_browse_image_dir);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_bag_clicked, this,
      &MainWindow::on_browse_bag);
  connect(
      launcher_panel_, &LauncherConfigPanel::load_bag_clicked, this,
      &MainWindow::on_load_bag);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_intrinsics_yaml_clicked, this,
      &MainWindow::on_browse_intrinsics_yaml);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_camera_yaml_clicked, this,
      &MainWindow::on_browse_camera_yaml);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_left_camera_yaml_clicked, this,
      &MainWindow::on_browse_left_camera_yaml);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_right_camera_yaml_clicked, this,
      &MainWindow::on_browse_right_camera_yaml);
  connect(
      launcher_panel_, &LauncherConfigPanel::browse_pose_csv_clicked, this,
      &MainWindow::on_browse_pose_csv);
  connect(
      launcher_panel_, &LauncherConfigPanel::reload_yaml_clicked, this,
      &MainWindow::on_reload_default_board_config);
  connect(
      launcher_panel_, &LauncherConfigPanel::pose_source_changed, this,
      [this](int) { refresh_setup_readiness(); });

  combo_source_mode_ = launcher_panel_->combo_source_mode();
  combo_image_topic_ = launcher_panel_->combo_image_topic();
  combo_camera_info_topic_ = launcher_panel_->combo_camera_info_topic();
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
        this, [this](int) {
          refresh_setup_readiness();
          refresh_status_task();
        });
  }
  if (launcher_panel_ != nullptr) {
    if (QSpinBox *sx = launcher_panel_->spin_squares_x()) {
      connect(sx, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {
        refresh_status_task();
      });
    }
    if (QSpinBox *sy = launcher_panel_->spin_squares_y()) {
      connect(sy, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {
        refresh_status_task();
      });
    }
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
}

/// \brief 构建数据源设置页（图像源 + 内参源）
QWidget *MainWindow::build_data_source_page() {
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 12, 20, 8);
  root->setSpacing(10);

  if (launcher_panel_ == nullptr) {
    launcher_panel_ = new LauncherConfigPanel(nullptr);
  }
  wire_launcher_panel_once();

  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(launcher_panel_->data_source_panel());
  auto *params_panel = make_panel(QStringLiteral("数据源设置"), scroll);

  auto *ready_strip = new QFrame;
  ready_strip->setObjectName(QStringLiteral("ReadyStrip"));
  auto *ready_lay = new QVBoxLayout(ready_strip);
  ready_lay->setContentsMargins(12, 8, 12, 8);
  ready_lay->setSpacing(8);

  auto *nav = new QHBoxLayout;
  nav->setSpacing(8);
  nav->addStretch(1);
  auto *back = new QPushButton(QStringLiteral("返回首页"), ready_strip);
  back->setObjectName(QStringLiteral("GhostButton"));
  connect(back, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  btn_datasource_next_ = new QPushButton(QStringLiteral("下一步：标定设置"), ready_strip);
  btn_datasource_next_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_datasource_next_->setEnabled(false);
  connect(btn_datasource_next_, &QPushButton::clicked, this, [this]() {
    if (is_detect_lab_mode()) {
      go_to(PageId::DetectLab);
    } else {
      go_to(PageId::Setup);
    }
  });
  nav->addWidget(back);
  nav->addWidget(btn_datasource_next_);
  ready_lay->addLayout(nav);

  root->addWidget(params_panel, 1);
  root->addWidget(ready_strip, 0);

  launcher_panel_->set_calibrator_id(selected_calibrator_id_);
  refresh_setup_source_ui();
  refresh_setup_readiness();
  return page;
}

/// \brief 构建标定设置页（靶标 / 检测 / 求解）
QWidget *MainWindow::build_setup_page() {
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 12, 20, 8);
  root->setSpacing(10);

  if (launcher_panel_ == nullptr) {
    launcher_panel_ = new LauncherConfigPanel(nullptr);
  }
  wire_launcher_panel_once();

  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(launcher_panel_);
  auto *params_panel = make_panel(QStringLiteral("标定设置"), scroll);

  auto *ready_strip = new QFrame;
  ready_strip->setObjectName(QStringLiteral("ReadyStrip"));
  auto *ready_lay = new QVBoxLayout(ready_strip);
  ready_lay->setContentsMargins(12, 8, 12, 8);
  ready_lay->setSpacing(8);

  auto *nav = new QHBoxLayout;
  nav->setSpacing(8);
  nav->addStretch(1);
  auto *back = new QPushButton(QStringLiteral("返回数据源"), ready_strip);
  back->setObjectName(QStringLiteral("GhostButton"));
  connect(back, &QPushButton::clicked, this, [this]() { go_to(PageId::DataSource); });
  btn_start_session_ = new QPushButton(QStringLiteral("开始会话 / Launch"), ready_strip);
  btn_start_session_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_start_session_->setEnabled(false);
  connect(btn_start_session_, &QPushButton::clicked, this, &MainWindow::on_start_session);
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
  root->setContentsMargins(12, 10, 12, 6);
  root->setSpacing(8);

  auto *header = new QHBoxLayout;
  header->setSpacing(16);
  header->setContentsMargins(0, 0, 0, 0);

  // 左：可收缩，路径强制省略，绝不把右侧挤走
  auto *left_host = new QWidget(page);
  left_host->setObjectName(QStringLiteral("WorkbenchHeaderLeft"));
  left_host->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  left_host->setMinimumWidth(0);
  auto *titles = new QVBoxLayout(left_host);
  titles->setContentsMargins(0, 0, 12, 0);
  titles->setSpacing(2);
  auto *title_lbl =
      make_label(QStringLiteral("工作台 · 采集与求解"), QStringLiteral("PageTitle"), left_host);
  title_lbl->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  titles->addWidget(title_lbl);
  workbench_path_label_ = make_label(
      QStringLiteral("尚未加载图片"), QStringLiteral("PageSubtitle"), left_host);
  workbench_path_label_->setWordWrap(false);
  workbench_path_label_->setMinimumWidth(0);
  workbench_path_label_->setMaximumWidth(40);
  workbench_path_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  workbench_path_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  titles->addWidget(workbench_path_label_);
  workbench_header_left_ = left_host;
  left_host->installEventFilter(this);

  // 右：主操作条（等宽大气按钮）
  auto *actions = new QWidget(page);
  actions->setObjectName(QStringLiteral("WorkbenchHeaderActions"));
  actions->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  auto *action_lay = new QHBoxLayout(actions);
  action_lay->setContentsMargins(10, 8, 12, 8);
  action_lay->setSpacing(10);
  action_lay->setAlignment(Qt::AlignVCenter);

  btn_prev_ = new QPushButton(QStringLiteral("上一张"), actions);
  btn_next_ = new QPushButton(QStringLiteral("下一张"), actions);
  btn_detect_ = new QPushButton(QStringLiteral("检测"), actions);
  btn_capture_wb_ = new QPushButton(QStringLiteral("采集帧"), actions);
  btn_solve_wb_ = new QPushButton(QStringLiteral("标定"), actions);
  btn_solve_wb_->setEnabled(false);
  auto *to_review = new QPushButton(QStringLiteral("复核"), actions);
  chk_auto_capture_ = new QCheckBox(QStringLiteral("自动采集"), actions);
  chk_auto_capture_->setObjectName(QStringLiteral("WorkbenchAutoCapture"));
  chk_auto_capture_->setChecked(true);
  chk_auto_capture_->setToolTip(
      QStringLiteral("仅实时预览时生效；冻结画面后暂停自动采集"));

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

  // 等宽固定：QSS min/max-width=108 + 代码再锁一层（防换文案/主题后撑开）
  {
    constexpr int kBtnW = 108;
    constexpr int kBtnH = 40;

    auto lock_size = [](QPushButton *b) {
      if (b == nullptr) {
        return;
      }
      b->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      b->setMinimumSize(kBtnW, kBtnH);
      b->setMaximumSize(kBtnW, kBtnH);
      b->setFixedSize(kBtnW, kBtnH);
      b->setCursor(Qt::PointingHandCursor);
    };

    auto style_nav = [&](QPushButton *b) {
      b->setObjectName(QStringLiteral("WorkbenchNavButton"));
      lock_size(b);
      action_lay->addWidget(b, 0, Qt::AlignVCenter);
    };
    auto style_action = [&](QPushButton *b, bool primary) {
      b->setObjectName(
          primary ? QStringLiteral("WorkbenchActionPrimary")
                  : QStringLiteral("WorkbenchActionButton"));
      lock_size(b);
      action_lay->addWidget(b, 0, Qt::AlignVCenter);
    };

    style_nav(btn_prev_);
    style_nav(btn_next_);
    style_action(btn_detect_, false);
    style_action(btn_capture_wb_, true);
    style_action(btn_solve_wb_, false);
    style_action(to_review, false);

    auto *sep = new QFrame(actions);
    sep->setObjectName(QStringLiteral("WorkbenchActionSep"));
    sep->setFrameShape(QFrame::NoFrame);
    sep->setFixedSize(1, 28);
    action_lay->addWidget(sep, 0, Qt::AlignVCenter);
    action_lay->addWidget(chk_auto_capture_, 0, Qt::AlignVCenter);
    actions->setMinimumHeight(kBtnH + 16);
  }
  header->addWidget(left_host, /*stretch*/ 1, Qt::AlignVCenter);
  header->addWidget(actions, /*stretch*/ 0, Qt::AlignRight | Qt::AlignVCenter);
  root->addLayout(header);

  auto *split = new QSplitter(Qt::Horizontal, page);
  split->setChildrenCollapsible(false);
  split->setHandleWidth(6);

  // ---- Left: preview（缩放显示 640/1280 图，列宽收窄） ----
  auto *preview_col = new QWidget(split);
  preview_col->setMinimumWidth(320);
  preview_col->setMaximumWidth(720);
  auto *preview_lay = new QVBoxLayout(preview_col);
  preview_lay->setContentsMargins(0, 0, 0, 0);
  preview_lay->setSpacing(8);

  preview_view_ = new ImageViewWidget(preview_col);
  preview_view_->setMinimumSize(280, 240);
  preview_view_->set_placeholder(QStringLiteral("预览区"));
  preview_view_->set_async_refresh(true, 33);
  preview_view_->set_toolbar_style(ImageViewToolbarStyle::Hidden);

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
  auto make_preview_tool_btn = [preview_header](const QString &text, const QString &tip) {
    auto *b = new QPushButton(text, preview_header);
    b->setObjectName(QStringLiteral("CompactButton"));
    b->setToolTip(tip);
    b->setFixedHeight(28);
    b->setMinimumWidth(0);
    return b;
  };
  btn_preview_zoom_in_ =
      make_preview_tool_btn(QStringLiteral("+"), QStringLiteral("放大"));
  btn_preview_zoom_out_ =
      make_preview_tool_btn(QStringLiteral("−"), QStringLiteral("缩小"));
  btn_preview_fit_ =
      make_preview_tool_btn(QStringLiteral("适应"), QStringLiteral("适应窗口"));
  btn_preview_one_to_one_ =
      make_preview_tool_btn(QStringLiteral("1:1"), QStringLiteral("原始像素 1:1"));
  btn_preview_save_ =
      make_preview_tool_btn(QStringLiteral("保存"), QStringLiteral("保存当前帧"));
  connect(btn_preview_zoom_in_, &QPushButton::clicked, preview_view_, &ImageViewWidget::zoom_in);
  connect(btn_preview_zoom_out_, &QPushButton::clicked, preview_view_, &ImageViewWidget::zoom_out);
  connect(btn_preview_fit_, &QPushButton::clicked, preview_view_, &ImageViewWidget::fit_to_window);
  connect(
      btn_preview_one_to_one_, &QPushButton::clicked, preview_view_,
      &ImageViewWidget::reset_view);
  connect(
      btn_preview_save_, &QPushButton::clicked, preview_view_,
      &ImageViewWidget::prompt_save_image);
  preview_header_lay->addWidget(btn_preview_zoom_in_);
  preview_header_lay->addWidget(btn_preview_zoom_out_);
  preview_header_lay->addWidget(btn_preview_fit_);
  preview_header_lay->addWidget(btn_preview_one_to_one_);
  preview_header_lay->addWidget(btn_preview_save_);
  preview_header_lay->addSpacing(8);
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
  preview_body_lay->setContentsMargins(8, 8, 8, 8);
  preview_body_lay->addWidget(preview_view_);
  preview_panel_lay->addWidget(preview_body, 1);
  preview_lay->addWidget(preview_panel, 1);

  auto *viz_strip = new QFrame(preview_col);
  viz_strip->setObjectName(QStringLiteral("VizStrip"));
  auto *viz_col = new QVBoxLayout(viz_strip);
  viz_col->setContentsMargins(10, 8, 10, 8);
  viz_col->setSpacing(6);

  auto *viz_row1 = new QWidget(viz_strip);
  viz_classic_row_ = viz_row1;
  auto *viz_row = new QHBoxLayout(viz_row1);
  viz_row->setContentsMargins(0, 0, 0, 0);
  viz_row->setSpacing(10);
  viz_row->setAlignment(Qt::AlignVCenter);
  viz_row->addWidget(make_label(
      QStringLiteral("叠加"), QStringLiteral("SectionTitle"), viz_row1));
  chk_viz_corners_wb_ = new QCheckBox(QStringLiteral("角点"), viz_row1);
  chk_viz_corners_wb_->setChecked(true);
  chk_viz_hull_wb_ = new QCheckBox(QStringLiteral("外轮廓"), viz_row1);
  chk_viz_hull_wb_->setChecked(true);
  chk_viz_conf_wb_ = new QCheckBox(QStringLiteral("置信度"), viz_row1);
  chk_viz_conf_wb_->setChecked(true);
  chk_viz_aruco_wb_ = new QCheckBox(QStringLiteral("ArUco"), viz_row1);
  chk_viz_aruco_wb_->setChecked(true);
  chk_viz_aruco_wb_->setToolTip(QStringLiteral("叠加检测到的 ArUco 边框与 ID"));
  viz_row->addWidget(chk_viz_corners_wb_);
  viz_row->addWidget(chk_viz_hull_wb_);
  viz_row->addWidget(chk_viz_conf_wb_);
  viz_row->addWidget(chk_viz_aruco_wb_);
  viz_row->addSpacing(6);
  viz_row->addWidget(make_label(
      QStringLiteral("标记"), QStringLiteral("Muted"), viz_row1));
  spin_viz_marker_wb_ = new QSpinBox(viz_row1);
  spin_viz_marker_wb_->setRange(1, 20);
  spin_viz_marker_wb_->setValue(4);
  spin_viz_marker_wb_->setSuffix(QStringLiteral(" px"));
  spin_viz_marker_wb_->setFixedWidth(88);
  spin_viz_marker_wb_->setMinimumHeight(32);
  viz_row->addWidget(spin_viz_marker_wb_, 0, Qt::AlignVCenter);
  viz_row->addStretch(1);
  viz_col->addWidget(viz_row1);

  viz_tier4_row_ = new QWidget(viz_strip);
  auto *tier4_row = new QHBoxLayout(viz_tier4_row_);
  tier4_row->setContentsMargins(0, 0, 0, 0);
  tier4_row->setSpacing(10);
  tier4_row->addWidget(make_label(
      QStringLiteral("Tier4 视图"), QStringLiteral("Muted"), viz_tier4_row_));
  combo_intrinsics_view_mode_ = new QComboBox(viz_tier4_row_);
  combo_intrinsics_view_mode_->setMinimumHeight(32);
  combo_intrinsics_view_mode_->setMinimumWidth(140);
  combo_intrinsics_view_mode_->addItem(
      QStringLiteral("Source"),
      static_cast<int>(IntrinsicsImageViewMode::Source));
  combo_intrinsics_view_mode_->addItem(
      QStringLiteral("Source rectified"),
      static_cast<int>(IntrinsicsImageViewMode::SourceRectified));
  combo_intrinsics_view_mode_->addItem(
      QStringLiteral("Undistortion alpha"),
      static_cast<int>(IntrinsicsImageViewMode::UndistortionAlpha));
  combo_intrinsics_view_mode_->addItem(
      QStringLiteral("Drawings"),
      static_cast<int>(IntrinsicsImageViewMode::Drawings));
  combo_intrinsics_view_mode_->addItem(
      QStringLiteral("Indicators"),
      static_cast<int>(IntrinsicsImageViewMode::Indicators));
  tier4_row->addWidget(combo_intrinsics_view_mode_);
  btn_tier4_viz_options_wb_ =
      new QPushButton(QStringLiteral("可视化设置…"), viz_tier4_row_);
  btn_tier4_viz_options_wb_->setObjectName(QStringLiteral("CompactButton"));
  btn_tier4_viz_options_wb_->setToolTip(
      QStringLiteral("打开 Tier4 Visualization options 设置页"));
  connect(
      btn_tier4_viz_options_wb_, &QPushButton::clicked, this,
      &MainWindow::show_intrinsics_viz_options_dialog);
  tier4_row->addWidget(btn_tier4_viz_options_wb_);
  tier4_row->addStretch(1);
  viz_col->addWidget(viz_tier4_row_);
  preview_lay->addWidget(viz_strip, 0);

  intrinsics_metrics_strip_ = new IntrinsicsMetricsStrip(preview_col);
  intrinsics_metrics_strip_->setVisible(false);
  preview_lay->addWidget(intrinsics_metrics_strip_, 0);

  auto refresh_viz = [this]() { refresh_workbench_preview_viz(); };
  connect(chk_viz_corners_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_hull_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_conf_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(chk_viz_aruco_wb_, &QCheckBox::toggled, this, refresh_viz);
  connect(
      spin_viz_marker_wb_, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [refresh_viz](int) { refresh_viz(); });
  connect(combo_intrinsics_view_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, refresh_viz](int) {
            if (session_ != nullptr && combo_intrinsics_view_mode_ != nullptr) {
              session_->set_intrinsics_image_view_mode(static_cast<IntrinsicsImageViewMode>(
                  combo_intrinsics_view_mode_->currentData().toInt()));
            }
            refresh_viz();
          });

  // ---- Middle: observation / file list (beside preview) ----
  auto *obs_col = new QWidget(split);
  obs_col->setMinimumWidth(180);
  obs_col->setMaximumWidth(320);
  auto *obs_lay = new QVBoxLayout(obs_col);
  obs_lay->setContentsMargins(0, 0, 0, 0);
  obs_lay->setSpacing(8);

  obs_list_ = new QListWidget(obs_col);
  obs_list_->setAlternatingRowColors(false);
  obs_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  obs_list_->setWordWrap(false);
  obs_list_->setTextElideMode(Qt::ElideMiddle);
  obs_eval_list_ = new QListWidget(obs_col);
  obs_eval_list_->setAlternatingRowColors(false);
  obs_eval_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  obs_eval_list_->setWordWrap(false);
  obs_eval_list_->setTextElideMode(Qt::ElideMiddle);
  obs_tabs_ = new QTabWidget(obs_col);
  obs_tabs_->addTab(obs_list_, QStringLiteral("训练"));
  obs_tabs_->addTab(obs_eval_list_, QStringLiteral("评估"));
  auto *obs_panel = make_panel(QStringLiteral("观测列表"), obs_tabs_);
  obs_lay->addWidget(obs_panel, 1);

  auto *obs_btns = new QHBoxLayout;
  obs_btns->setSpacing(8);
  auto *remove_btn = new QPushButton(QStringLiteral("删除选中"), obs_col);
  remove_btn->setObjectName(QStringLiteral("CompactButton"));
  auto *clear_btn = new QPushButton(QStringLiteral("全部清空"), obs_col);
  clear_btn->setObjectName(QStringLiteral("CompactButton"));
  connect(remove_btn, &QPushButton::clicked, this, [this]() {
    if (obs_tabs_ == nullptr || session_ == nullptr) {
      return;
    }
    const bool eval_tab = obs_tabs_->currentIndex() == 1;
    QListWidget *list = eval_tab ? obs_eval_list_ : obs_list_;
    const int row = list->currentRow();
    if (row < 0) {
      append_log(LogLevel::Warn, QStringLiteral("› 请先选中一条观测"));
      return;
    }
    if (eval_tab) {
      session_->remove_intrinsics_sample(core::IntrinsicsSampleSplit::Evaluation, row);
    } else {
      session_->remove_observation(row);
    }
    append_log(LogLevel::Info, QStringLiteral("› 已删除观测"));
    refresh_workbench_view(false);
  });
  connect(clear_btn, &QPushButton::clicked, this, [this]() {
    if (session_ == nullptr) {
      return;
    }
    if (!session_->has_observations()) {
      append_log(LogLevel::Warn, QStringLiteral("› 观测列表已空"));
      return;
    }
    const int clear_count = session_->uses_tier4_intrinsics()
        ? session_->training_sample_count() + session_->evaluation_sample_count()
        : session_->observation_count();
    const auto ret = QMessageBox::question(
        this, QStringLiteral("清空观测"),
        QStringLiteral("确定清空全部 %1 条已采集观测？")
            .arg(clear_count),
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

  intrinsics_sample_slider_label_ = new QLabel(QStringLiteral("样本浏览：实时"), obs_col);
  intrinsics_sample_slider_label_->setObjectName(QStringLiteral("Muted"));
  intrinsics_sample_slider_ = new QSlider(Qt::Horizontal, obs_col);
  intrinsics_sample_slider_->setRange(-1, 0);
  intrinsics_sample_slider_->setValue(-1);
  intrinsics_sample_slider_->setEnabled(false);
  obs_lay->addWidget(intrinsics_sample_slider_label_);
  obs_lay->addWidget(intrinsics_sample_slider_);
  connect(intrinsics_sample_slider_, &QSlider::valueChanged, this, [this](int v) {
    if (session_ == nullptr || intrinsics_sample_slider_label_ == nullptr) {
      return;
    }
    if (v < 0) {
      session_->clear_intrinsics_browse();
      intrinsics_sample_slider_label_->setText(QStringLiteral("样本浏览：实时"));
    } else {
      const bool eval_tab = obs_tabs_ != nullptr && obs_tabs_->currentIndex() == 1;
      session_->set_intrinsics_browse_sample(v, eval_tab);
      intrinsics_sample_slider_label_->setText(
          QStringLiteral("样本浏览：%1 #%2").arg(eval_tab ? QStringLiteral("评估")
                                                          : QStringLiteral("训练"))
              .arg(v + 1));
    }
    refresh_workbench_view(true);
  });

  // ---- Right rail: default metrics OR Tier4 intrinsics control ----
  auto *right = new QWidget(split);
  right->setMinimumWidth(260);
  right->setMaximumWidth(420);
  right->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
  auto *right_lay = new QVBoxLayout(right);
  right_lay->setContentsMargins(0, 0, 0, 0);
  right_lay->setSpacing(8);

  workbench_default_right_ = new QWidget(right);
  auto *default_right_lay = new QVBoxLayout(workbench_default_right_);
  default_right_lay->setContentsMargins(0, 0, 0, 0);
  auto *metrics_host = new QWidget(workbench_default_right_);
  auto *metrics_layout = new QVBoxLayout(metrics_host);
  metrics_layout->setContentsMargins(0, 0, 0, 0);
  metrics_layout->setSpacing(6);
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("已采集帧数"), QStringLiteral("0")));
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("检测置信度"), QStringLiteral("—")));
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("重投影"), QStringLiteral("—")));
  metrics_layout->addWidget(
      make_compact_metric_card(QStringLiteral("覆盖提示"), QStringLiteral("—")));
  default_right_lay->addWidget(make_panel(QStringLiteral("会话指标"), metrics_host), 0);
  default_right_lay->addStretch(1);

  intrinsics_control_rail_ = new IntrinsicsControlRail(right);
  intrinsics_control_rail_->setVisible(false);
  connect(intrinsics_control_rail_, &IntrinsicsControlRail::calibration_params_requested,
          this, [this]() { show_intrinsics_parameter_dialog(0); });
  connect(intrinsics_control_rail_, &IntrinsicsControlRail::detector_params_requested,
          this, [this]() { show_intrinsics_parameter_dialog(2); });
  connect(intrinsics_control_rail_, &IntrinsicsControlRail::collector_params_requested,
          this, [this]() { show_intrinsics_parameter_dialog(1); });
  connect(intrinsics_control_rail_, &IntrinsicsControlRail::statistics_requested,
          this, [this]() { show_intrinsics_stats_dialog(); });
  connect(intrinsics_control_rail_, &IntrinsicsControlRail::status_details_requested,
          this, [this]() { show_intrinsics_calibration_status_dialog(); });
  connect(intrinsics_metrics_strip_, &IntrinsicsMetricsStrip::details_requested, this,
          [this]() { show_intrinsics_detection_details_dialog(); });
  connect(intrinsics_control_rail_->solver_combo(),
          QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
            if (session_ == nullptr || intrinsics_control_rail_ == nullptr) {
              return;
            }
            session_->set_intrinsics_solver_kind(
                intrinsics_control_rail_->solver_combo()->currentData().toString().toStdString());
          });

  right_lay->addWidget(workbench_default_right_);
  right_lay->addWidget(intrinsics_control_rail_);

  split->addWidget(preview_col);
  split->addWidget(obs_col);
  split->addWidget(right);
  // 窄窗口：预览为主，指标栏不抢宽度
  split->setStretchFactor(0, 5);
  split->setStretchFactor(1, 2);
  split->setStretchFactor(2, 0);
  split->setSizes({540, 280, 130});
  root->addWidget(split, 1);
  return page;
}

/// \brief 构建结果复查页
QWidget *MainWindow::build_review_page() {
  auto *page = new QWidget;
  auto *root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 8);
  root->setSpacing(10);

  auto *header = new QHBoxLayout;
  auto *titles = new QVBoxLayout;
  titles->addWidget(
      make_label(QStringLiteral("复核与导出"), QStringLiteral("PageTitle"), page));
  titles->addWidget(make_label(
      QStringLiteral("残差 · 观测 · 覆盖可视化，确认后导出"),
      QStringLiteral("PageSubtitle"), page));
  header->addLayout(titles, 1);
  auto *stats_btn = new QPushButton(QStringLiteral("标定统计…"), page);
  stats_btn->setObjectName(QStringLiteral("GhostButton"));
  stats_btn->setToolTip(QStringLiteral("重新打开 Tier4 采集统计与标定结果统计图"));
  connect(stats_btn, &QPushButton::clicked, this,
          &MainWindow::show_intrinsics_tier4_statistics_dialogs);
  auto *export_btn = new QPushButton(QStringLiteral("导出结果"), page);
  export_btn->setObjectName(QStringLiteral("PrimaryButton"));
  connect(export_btn, &QPushButton::clicked, this, &MainWindow::on_export_yaml);
  auto *done = new QPushButton(QStringLiteral("完成"), page);
  done->setObjectName(QStringLiteral("GhostButton"));
  connect(done, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  header->addWidget(stats_btn);
  header->addWidget(export_btn);
  header->addWidget(done);
  root->addLayout(header);

  auto *metrics_row = new QHBoxLayout;
  metrics_row->addWidget(make_metric_card(QStringLiteral("重投影 RMSE"), QStringLiteral("— px")));
  metrics_row->addWidget(make_metric_card(QStringLiteral("有效观测"), QStringLiteral("—")));
  metrics_row->addWidget(make_metric_card(QStringLiteral("图像尺寸"), QStringLiteral("—")));
  root->addLayout(metrics_row);

  review_diag_label_ = make_label(QString(), QStringLiteral("Muted"), page);
  root->addWidget(review_diag_label_);

  auto *split = new QSplitter(Qt::Horizontal, page);
  split->setChildrenCollapsible(false);

  review_obs_list_ = new QListWidget(split);
  review_obs_list_->setObjectName(QStringLiteral("ReviewObsList"));
  review_obs_list_->setMinimumWidth(200);
  review_obs_list_->setMaximumWidth(320);
  connect(
      review_obs_list_, &QListWidget::itemClicked, this,
      &MainWindow::on_review_obs_clicked);
  split->addWidget(make_panel(QStringLiteral("观测列表"), review_obs_list_));

  review_residual_bars_ = new ResidualBarWidget(split);
  connect(
      review_residual_bars_, &ResidualBarWidget::bar_clicked, this,
      &MainWindow::on_review_bar_clicked);
  {
    auto *panel = make_panel(QStringLiteral("残差图"), review_residual_bars_);
    if (auto *header = panel->findChild<QFrame *>(QStringLiteral("PanelHeader"))) {
      if (auto *hlay = qobject_cast<QHBoxLayout *>(header->layout())) {
        auto *zoom_btn = new QPushButton(QStringLiteral("放大"), header);
        zoom_btn->setObjectName(QStringLiteral("CompactButton"));
        zoom_btn->setToolTip(QStringLiteral("打开放大窗口（也可双击图表）"));
        connect(zoom_btn, &QPushButton::clicked, this, [this]() {
          if (review_residual_bars_ != nullptr) {
            review_residual_bars_->open_enlarged();
          }
        });
        hlay->addStretch(1);
        hlay->addWidget(zoom_btn);
      }
    }
    split->addWidget(panel);
  }

  review_coverage_map_ = new CoverageMapWidget(split);
  {
    auto *panel = make_panel(QStringLiteral("覆盖 / 重投影"), review_coverage_map_);
    if (auto *header = panel->findChild<QFrame *>(QStringLiteral("PanelHeader"))) {
      if (auto *hlay = qobject_cast<QHBoxLayout *>(header->layout())) {
        auto *zoom_btn = new QPushButton(QStringLiteral("放大"), header);
        zoom_btn->setObjectName(QStringLiteral("CompactButton"));
        zoom_btn->setToolTip(QStringLiteral("打开放大窗口（也可双击图表）"));
        connect(zoom_btn, &QPushButton::clicked, this, [this]() {
          if (review_coverage_map_ != nullptr) {
            review_coverage_map_->open_enlarged();
          }
        });
        hlay->addStretch(1);
        hlay->addWidget(zoom_btn);
      }
    }
    split->addWidget(panel);
  }

  split->setStretchFactor(0, 0);
  split->setStretchFactor(1, 1);
  split->setStretchFactor(2, 1);
  root->addWidget(split, 1);

  review_text_ = new QTextEdit(page);
  review_text_->setReadOnly(true);
  review_text_->setMinimumHeight(280);
  review_text_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  review_text_->setPlainText(QStringLiteral("求解后显示 K / D / RMSE。"));
  root->addWidget(make_panel(QStringLiteral("结果摘要"), review_text_), 2);
  return page;
}

}  // namespace gui
}  // namespace hs_calib
