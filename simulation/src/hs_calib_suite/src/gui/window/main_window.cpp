#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/log/app_logger.hpp"
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
#include <QPalette>
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
#include <QTextCharFormat>
#include <QTextCursor>
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

// ===== 构造 =====

/// \brief 构造主窗口：会话、ROS/TF 桥、页面与定时泵
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      session_(std::make_unique<SessionController>(this)),
      ros_bridge_(std::make_unique<RosImageBridge>(this)),
      tf_bridge_(std::make_unique<TfPoseBridge>(this)) {
  setWindowTitle(QStringLiteral("HS Calib Suite"));
  resize(1480, 900);
  theme_id_ = load_theme_id();
  selected_calibrator_id_ = QStringLiteral("cam_intrinsics");
  session_->set_calibrator_id(selected_calibrator_id_);
  session_->set_tf_bridge(tf_bridge_.get());

  // —— 组装壳层 UI ——
  setup_menu_bar();
  setup_tool_bar();
  setup_status_bar();
  setup_central_widget();
  apply_theme();

  // —— ROS/TF 定时泵 ——
  ros_spin_timer_ = new QTimer(this);
  ros_spin_timer_->setInterval(33);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() {
    if (ros_bridge_) {
      ros_bridge_->spin_some();
    }
    if (tf_bridge_) {
      tf_bridge_->spin_some();
    }
  });
  ros_spin_timer_->start();

  connect(session_.get(), &SessionController::images_changed, this, [this]() {
    refresh_setup_readiness();
  });
  connect(session_.get(), &SessionController::current_changed, this, [this]() {
    if (stack_ != nullptr &&
        stack_->currentIndex() == static_cast<int>(PageId::DetectLab)) {
      refresh_detect_lab_view(false);
    } else {
      refresh_workbench_view();
    }
  });
  connect(session_.get(), &SessionController::observations_changed, this, [this]() {
    refresh_workbench_view();
  });
  connect(session_.get(), &SessionController::result_changed, this, [this]() {
    refresh_review_view();
    if (act_export_ != nullptr) {
      act_export_->setEnabled(session_->has_result());
    }
  });
  connect(
      session_.get(), &SessionController::detect_started, this,
      &MainWindow::on_async_detect_started);
  connect(
      session_.get(), &SessionController::detect_finished, this,
      &MainWindow::on_async_detect_finished);
  connect(
      ros_bridge_.get(), &RosImageBridge::frame_received, this,
      &MainWindow::on_ros_frame);
  connect(
      ros_bridge_.get(), &RosImageBridge::camera_info_received, this, [this]() {
        sync_detect_intrinsics_from_sources();
      });

  auto *preview_mode_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  preview_mode_shortcut->setContext(Qt::WindowShortcut);
  connect(preview_mode_shortcut, &QShortcut::activated, this, [this]() {
    if (stack_ == nullptr ||
        stack_->currentIndex() != static_cast<int>(PageId::Workbench)) {
      return;
    }
    if (session_ == nullptr || session_->source_mode() != SourceMode::RosTopic) {
      return;
    }
    set_preview_live(!preview_live_);
  });

  if (!ros_bridge_->is_ready()) {
    append_log(LogLevel::Warn, QStringLiteral("› ROS 未就绪：仅离线目录可用"));
  } else {
    append_log(LogLevel::Info, QStringLiteral("› ROS 图像桥已就绪"));
  }

  go_to(PageId::Home);
}

/// \brief 析构：停转 ROS 泵并退订图像
MainWindow::~MainWindow() {
  AppLogger::write(LogLevel::Info, QStringLiteral("HS Calib Suite window closing"));
  if (ros_spin_timer_ != nullptr) {
    ros_spin_timer_->stop();
  }
  if (ros_bridge_) {
    ros_bridge_->unsubscribe();
  }
}

// ===== 主题 =====

/// \brief 应用当前主题 QSS 与工具栏图标
void MainWindow::apply_theme() {
  setStyleSheet(application_style_sheet(theme_id_));
  refresh_toolbar_icons();
  if (act_theme_dark_ != nullptr) {
    act_theme_dark_->setChecked(theme_id_ == ThemeId::Dark);
    act_theme_light_->setChecked(theme_id_ == ThemeId::Light);
    act_theme_blue_->setChecked(theme_id_ == ThemeId::Blue);
  }
  if (stack_ != nullptr) {
    update_step_rail(static_cast<PageId>(stack_->currentIndex()));
    update_status_bar(static_cast<PageId>(stack_->currentIndex()));
  }
  if (status_mode_ != nullptr) {
    status_mode_->style()->unpolish(status_mode_);
    status_mode_->style()->polish(status_mode_);
  }
}

/// \brief 切换并持久化主题
void MainWindow::set_theme(ThemeId id) {
  if (theme_id_ == id) {
    return;
  }
  theme_id_ = id;
  save_theme_id(id);
  apply_theme();
  append_log(LogLevel::Info, QStringLiteral("› 主题：%1").arg(theme_display_name(id)));
}

/// \brief 按主题色重绘工具栏图标
void MainWindow::refresh_toolbar_icons() {
  const QColor ink(theme_icon_ink(theme_id_));
  const QColor accent(theme_icon_accent(theme_id_));
  bind_toolbar_action(act_home_, TbGlyph::Home, QStringLiteral("首页"), ink, accent);
  bind_toolbar_action(act_setup_, TbGlyph::Setup, QStringLiteral("会话配置"), ink, accent);
  bind_toolbar_action(
      act_workbench_, TbGlyph::Workbench, QStringLiteral("工作台"), ink, accent);
  bind_toolbar_action(act_review_, TbGlyph::Review, QStringLiteral("复核导出"), ink, accent);
  bind_toolbar_action(act_offline_, TbGlyph::Offline, QStringLiteral("离线模式"), ink, accent);
  bind_toolbar_action(
      act_online_, TbGlyph::Online, QStringLiteral("在线模式（ROS）"), ink, accent);
  bind_toolbar_action(act_capture_, TbGlyph::Capture, QStringLiteral("采集帧"), ink, accent);
  bind_toolbar_action(act_solve_, TbGlyph::Solve, QStringLiteral("求解"), ink, accent);
  bind_toolbar_action(act_export_, TbGlyph::Export, QStringLiteral("导出结果…"), ink, accent);
}

/// \brief 更新顶部步骤条状态
void MainWindow::update_step_rail(PageId page) {
  static const char *titles[] = {
      "1  选择任务", "2  会话配置", "3  采集求解", "4  复核导出"};
  for (int i = 0; i < 4; ++i) {
    if (step_labels_[i] == nullptr) {
      continue;
    }
    step_labels_[i]->setText(QString::fromUtf8(titles[i]));
    if (page == PageId::DetectLab) {
      step_labels_[i]->setObjectName(QStringLiteral("StepIdle"));
    } else if (i < static_cast<int>(page)) {
      step_labels_[i]->setObjectName(QStringLiteral("StepDone"));
    } else if (i == static_cast<int>(page)) {
      step_labels_[i]->setObjectName(QStringLiteral("StepActive"));
    } else {
      step_labels_[i]->setObjectName(QStringLiteral("StepIdle"));
    }
    step_labels_[i]->style()->unpolish(step_labels_[i]);
    step_labels_[i]->style()->polish(step_labels_[i]);
  }
}

/// \brief 向日志区追加一行
void MainWindow::append_log(LogLevel level, const QString &line) {
  AppLogger::write(level, line);
  if (log_ == nullptr) {
    return;
  }

  const bool light = (theme_id_ == ThemeId::Light);
  QColor color;
  switch (level) {
    case LogLevel::Debug:
      color = light ? QColor(0x6b, 0x7a, 0x8a) : QColor(0x8b, 0x9c, 0xb3);
      break;
    case LogLevel::Info:
      color = log_->palette().color(QPalette::Text);
      break;
    case LogLevel::Warn:
      color = light ? QColor(0xb8, 0x86, 0x0b) : QColor(0xe8, 0xb8, 0x4a);
      break;
    case LogLevel::Error:
      color = light ? QColor(0xc6, 0x28, 0x28) : QColor(0xef, 0x53, 0x50);
      break;
  }

  QTextCharFormat fmt;
  fmt.setForeground(QBrush(color));
  const QString text = QStringLiteral("[%1] [%2] %3\n")
                           .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")),
                                QString::fromUtf8(AppLogger::level_tag(level)), line);

  QTextCursor cursor = log_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(text, fmt);
  log_->setTextCursor(cursor);
  log_->ensureCursorVisible();
}

/// \brief 刷新状态栏模式/页面/提示
void MainWindow::update_status_bar(PageId page) {
  static const char *names[] = {
      "PAGE / HOME", "PAGE / SETUP", "PAGE / WORKBENCH", "PAGE / REVIEW",
      "PAGE / DETECT LAB"};
  if (status_page_ != nullptr) {
    status_page_->setText(QString::fromUtf8(names[static_cast<int>(page)]));
  }
  if (status_hint_ != nullptr) {
    if (page == PageId::DetectLab) {
      status_hint_->setText(
          QStringLiteral("检测台 · 调角点/码/圆点 · Thorough 更完整 · Fast 更快"));
    } else if (page == PageId::Workbench && online_mode_) {
      const bool busy = session_ && session_->detect_busy();
      if (busy) {
        status_hint_->setText(QStringLiteral("在线 · 后台检测中… 画面保持实时"));
      } else if (preview_live_) {
        status_hint_->setText(QStringLiteral("在线 · 实时预览（空格冻结画面）"));
      } else {
        status_hint_->setText(QStringLiteral("在线 · 画面已冻结（空格恢复实时）"));
      }
    } else {
      status_hint_->setText(
          online_mode_ ? QStringLiteral("在线 · Setup 中选择图像话题后采集")
                       : QStringLiteral("离线 · 选图片目录后采集求解"));
    }
  }
}

/// \brief 切换在线/离线模式并同步源 UI
void MainWindow::set_online_mode(bool online) {
  online_mode_ = online;
  if (act_online_ != nullptr) {
    act_online_->setChecked(online);
    act_offline_->setChecked(!online);
  }
  if (status_mode_ != nullptr) {
    status_mode_->setObjectName(
        online ? QStringLiteral("StatusBarModeOnline")
               : QStringLiteral("StatusBarModeOffline"));
    status_mode_->setText(online ? QStringLiteral("ROS 在线") : QStringLiteral("离线"));
    status_mode_->style()->unpolish(status_mode_);
    status_mode_->style()->polish(status_mode_);
  }
  if (combo_source_mode_ != nullptr) {
    combo_source_mode_->blockSignals(true);
    combo_source_mode_->setCurrentIndex(
        online ? static_cast<int>(SourceMode::RosTopic)
               : static_cast<int>(SourceMode::Offline));
    combo_source_mode_->blockSignals(false);
    on_source_mode_changed(combo_source_mode_->currentIndex());
  }
  append_log(
      LogLevel::Info,
      online ? QStringLiteral("› 模式：在线（ROS 图像话题）")
             : QStringLiteral("› 模式：离线（图片目录）"));
  const int idx = stack_ != nullptr ? stack_->currentIndex() : 0;
  update_status_bar(static_cast<PageId>(idx));
}

/// \brief 切换到指定流程页
void MainWindow::go_to(PageId page) {
  if (stack_ != nullptr) {
    stack_->setCurrentIndex(static_cast<int>(page));
  }
  update_step_rail(page);
  update_status_bar(page);

  if (act_home_ != nullptr) {
    act_home_->setChecked(page == PageId::Home);
    act_setup_->setChecked(page == PageId::Setup);
    act_workbench_->setChecked(page == PageId::Workbench);
    act_review_->setChecked(page == PageId::Review);
  }

  const bool on_work = (page == PageId::Workbench);
  if (act_capture_ != nullptr) {
    act_capture_->setEnabled(on_work);
  }
  if (act_solve_ != nullptr) {
    act_solve_->setEnabled(on_work && session_ && session_->observation_count() >= 3);
  }
  if (act_export_ != nullptr) {
    act_export_->setEnabled(session_ && session_->has_result());
  }

  if (page == PageId::Setup) {
    refresh_handeye_ui();
    refresh_setup_readiness();
  } else if (page == PageId::Workbench) {
    refresh_workbench_view();
  } else if (page == PageId::Review) {
    refresh_review_view();
  } else if (page == PageId::DetectLab) {
    refresh_lab_mode_ui();
    refresh_detect_lab_view(false);
  }

  static const char *names[] = {
      "首页", "会话配置", "工作台", "复核导出", "特征检测台"};
  append_log(
      LogLevel::Info,
      QStringLiteral("› 进入「%1」")
          .arg(QString::fromUtf8(names[static_cast<int>(page)])));
}

// ===== 控件工厂 =====


/// \brief 创建菜单栏动作
void MainWindow::setup_menu_bar() {
  auto *file_menu = menuBar()->addMenu(QStringLiteral("文件(&F)"));
  auto *act_reload = file_menu->addAction(QStringLiteral("重新加载默认棋盘配置"));
  connect(act_reload, &QAction::triggered, this, &MainWindow::on_reload_default_board_config);
  act_export_ = file_menu->addAction(QStringLiteral("导出结果文件夹…"));
  act_export_->setEnabled(false);
  connect(act_export_, &QAction::triggered, this, &MainWindow::on_export_yaml);
  file_menu->addSeparator();
  auto *act_quit = file_menu->addAction(QStringLiteral("退出"));
  act_quit->setShortcut(QKeySequence::Quit);
  connect(act_quit, &QAction::triggered, this, &QWidget::close);

  auto *view_menu = menuBar()->addMenu(QStringLiteral("视图(&V)"));
  act_home_ = view_menu->addAction(QStringLiteral("首页"));
  act_setup_ = view_menu->addAction(QStringLiteral("会话配置"));
  act_workbench_ = view_menu->addAction(QStringLiteral("工作台"));
  act_review_ = view_menu->addAction(QStringLiteral("复核导出"));
  for (QAction *a : {act_home_, act_setup_, act_workbench_, act_review_}) {
    a->setCheckable(true);
  }
  connect(act_home_, &QAction::triggered, this, [this]() { go_to(PageId::Home); });
  connect(act_setup_, &QAction::triggered, this, [this]() { go_to(PageId::Setup); });
  connect(act_workbench_, &QAction::triggered, this, [this]() { go_to(PageId::Workbench); });
  connect(act_review_, &QAction::triggered, this, [this]() { go_to(PageId::Review); });

  auto *session_menu = menuBar()->addMenu(QStringLiteral("会话(&S)"));
  act_online_ = session_menu->addAction(QStringLiteral("在线模式（ROS）"));
  act_offline_ = session_menu->addAction(QStringLiteral("离线模式"));
  act_online_->setCheckable(true);
  act_offline_->setCheckable(true);
  act_offline_->setChecked(true);
  connect(act_online_, &QAction::triggered, this, [this]() { set_online_mode(true); });
  connect(act_offline_, &QAction::triggered, this, [this]() { set_online_mode(false); });
  session_menu->addSeparator();
  act_capture_ = session_menu->addAction(QStringLiteral("采集帧"));
  act_solve_ = session_menu->addAction(QStringLiteral("求解"));
  act_capture_->setEnabled(false);
  act_solve_->setEnabled(false);
  connect(act_capture_, &QAction::triggered, this, &MainWindow::on_capture_observation);
  connect(act_solve_, &QAction::triggered, this, &MainWindow::on_solve);

  auto *settings_menu = menuBar()->addMenu(QStringLiteral("设置(&O)"));
  auto *theme_menu = settings_menu->addMenu(QStringLiteral("主题"));
  theme_group_ = new QActionGroup(this);
  theme_group_->setExclusive(true);
  act_theme_dark_ = theme_menu->addAction(QStringLiteral("深色"));
  act_theme_light_ = theme_menu->addAction(QStringLiteral("浅色"));
  act_theme_blue_ = theme_menu->addAction(QStringLiteral("蓝色"));
  for (QAction *a : {act_theme_dark_, act_theme_light_, act_theme_blue_}) {
    a->setCheckable(true);
    theme_group_->addAction(a);
  }
  connect(act_theme_dark_, &QAction::triggered, this, [this]() { set_theme(ThemeId::Dark); });
  connect(act_theme_light_, &QAction::triggered, this, [this]() { set_theme(ThemeId::Light); });
  connect(act_theme_blue_, &QAction::triggered, this, [this]() { set_theme(ThemeId::Blue); });

  auto *help_menu = menuBar()->addMenu(QStringLiteral("帮助(&H)"));
  connect(help_menu->addAction(QStringLiteral("关于")), &QAction::triggered, this, [this]() {
    append_log(LogLevel::Info, QStringLiteral("› HS Calib Suite · 棋盘格单目内参 P1"));
  });
}

/// \brief 创建工具栏动作与布局
void MainWindow::setup_tool_bar() {
  auto *tb = addToolBar(QStringLiteral("主工具栏"));
  tb->setObjectName(QStringLiteral("MainToolBar"));
  tb->setMovable(false);
  tb->setFloatable(false);
  tb->setIconSize(QSize(20, 20));
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);

  tb->addAction(act_home_);
  tb->addAction(act_setup_);
  tb->addAction(act_workbench_);
  tb->addAction(act_review_);
  tb->addSeparator();
  tb->addAction(act_capture_);
  tb->addAction(act_solve_);
  tb->addSeparator();
  tb->addAction(act_export_);

  auto *spacer = new QWidget(tb);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  tb->addWidget(spacer);
  tb->addAction(act_offline_);
  tb->addAction(act_online_);
}

/// \brief 创建状态栏标签
void MainWindow::setup_status_bar() {
  status_mode_ = new QLabel(QStringLiteral("离线"), this);
  status_mode_->setObjectName(QStringLiteral("StatusBarModeOffline"));
  status_page_ = new QLabel(QStringLiteral("PAGE / HOME"), this);
  status_page_->setObjectName(QStringLiteral("StatusBarPage"));
  status_hint_ = new QLabel(QStringLiteral("就绪"), this);
  statusBar()->addWidget(status_mode_);
  statusBar()->addWidget(status_page_);
  statusBar()->addWidget(status_hint_, 1);
}

/// \brief 组装中央堆叠页与步骤轨
void MainWindow::setup_central_widget() {
  auto *root = new QWidget(this);
  root->setObjectName(QStringLiteral("CentralRoot"));
  auto *root_layout = new QVBoxLayout(root);
  root_layout->setContentsMargins(0, 0, 0, 0);
  root_layout->setSpacing(0);

  auto *steps = new QFrame(root);
  steps->setObjectName(QStringLiteral("StepRail"));
  auto *steps_layout = new QHBoxLayout(steps);
  steps_layout->setContentsMargins(20, 0, 20, 0);
  for (int i = 0; i < 4; ++i) {
    step_labels_[i] = new QLabel(steps);
    steps_layout->addWidget(step_labels_[i]);
    if (i < 3) {
      steps_layout->addWidget(make_label(QStringLiteral("→"), QStringLiteral("Muted"), steps));
    }
  }
  steps_layout->addStretch(1);
  steps_layout->addWidget(
      make_label(QStringLiteral("HS CALIB SUITE"), QStringLiteral("BrandMark"), steps));
  root_layout->addWidget(steps);

  stack_ = new QStackedWidget(root);
  stack_->addWidget(build_home_page());
  stack_->addWidget(build_setup_page());
  stack_->addWidget(build_workbench_page());
  stack_->addWidget(build_review_page());
  stack_->addWidget(build_detect_lab_page());
  root_layout->addWidget(stack_, 1);

  auto *log_body = new QWidget;
  auto *log_layout = new QVBoxLayout(log_body);
  log_layout->setContentsMargins(0, 0, 0, 0);
  log_ = new QTextEdit(log_body);
  log_->setReadOnly(true);
  log_->setUndoRedoEnabled(false);
  log_->setMaximumHeight(120);
  log_->document()->setMaximumBlockCount(800);
  log_layout->addWidget(log_);
  append_log(
      LogLevel::Info,
      QStringLiteral("就绪。棋盘格单目内参：离线选图 → 检测采集 → core 求解 → 导出。"));
  auto *log_wrap = new QWidget(root);
  auto *log_wrap_layout = new QVBoxLayout(log_wrap);
  log_wrap_layout->setContentsMargins(16, 8, 16, 12);
  log_wrap_layout->addWidget(make_panel(QStringLiteral("系统日志"), log_body));
  root_layout->addWidget(log_wrap);

  setCentralWidget(root);
}

// ===== 页面 =====

}  // namespace gui
}  // namespace hs_calib
