#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/log/app_logger.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/bridges/ros_stereo_image_bridge.hpp"
#include "hs_calib_suite/gui/bridges/ros_executor_hub.hpp"
#include "hs_calib_suite/gui/bridges/ros_stereo_image_bridge.hpp"
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
#include <QDialog>
#include <QDialogButtonBox>
#include <QApplication>
#include <QtGlobal>

#include <opencv2/core/version.hpp>
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
using hs_calib::gui::window_detail::set_metric_value;

// ===== 构造 =====

/// \brief 构造主窗口：会话、ROS/TF 桥、页面与定时泵
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      session_(std::make_unique<SessionController>(this)),
      ros_bridge_(std::make_unique<RosImageBridge>(this)),
      ros_stereo_bridge_(std::make_unique<RosStereoImageBridge>(this)),
      ros_executor_hub_(std::make_unique<RosExecutorHub>()),
      tf_bridge_(std::make_unique<TfPoseBridge>(this)) {
  setWindowTitle(QStringLiteral("HS Calib Suite"));
  // 收窄整体宽度；高度保持可用。预览缩放适配 1280×720 / 640×480
  resize(1120, 900);
  setMinimumSize(980, 780);
  theme_id_ = load_theme_id();
  selected_calibrator_id_ = QStringLiteral("cam_intrinsics");
  session_->set_calibrator_id(selected_calibrator_id_);
  session_->set_tf_bridge(tf_bridge_.get());

  // —— 组装壳层 UI ——
  setup_menu_bar();
  setup_mode_corner();
  setup_status_bar();
  setup_central_widget();
  apply_theme();

  if (ros_executor_hub_) {
    if (ros_bridge_ && ros_bridge_->ros_node()) {
      ros_executor_hub_->add_node(ros_bridge_->ros_node());
    }
    if (ros_stereo_bridge_ && ros_stereo_bridge_->ros_node()) {
      ros_executor_hub_->add_node(ros_stereo_bridge_->ros_node());
    }
    if (tf_bridge_ && tf_bridge_->ros_node()) {
      ros_executor_hub_->add_node(tf_bridge_->ros_node());
    }
    ros_executor_hub_->start(2);
  }

  ros_stereo_sub_debounce_ = new QTimer(this);
  ros_stereo_sub_debounce_->setSingleShot(true);
  ros_stereo_sub_debounce_->setInterval(450);
  connect(ros_stereo_sub_debounce_, &QTimer::timeout, this, [this]() {
    sync_ros_image_subscription();
  });

  workbench_refresh_timer_ = new QTimer(this);
  workbench_refresh_timer_->setSingleShot(true);
  workbench_refresh_timer_->setInterval(0);
  connect(workbench_refresh_timer_, &QTimer::timeout, this, [this]() {
    refresh_workbench_view(workbench_refresh_update_preview_);
    workbench_refresh_update_preview_ = false;
  });

  connect(session_.get(), &SessionController::images_changed, this, [this]() {
    refresh_setup_readiness();
  });
  connect(session_.get(), &SessionController::current_changed, this, [this]() {
    if (stack_ != nullptr &&
        stack_->currentIndex() == static_cast<int>(PageId::DetectLab)) {
      refresh_detect_lab_view(false);
    } else {
      schedule_workbench_view_refresh(true);
    }
  });
  connect(session_.get(), &SessionController::observations_changed, this, [this]() {
    schedule_workbench_view_refresh(false);
  });
  connect(session_.get(), &SessionController::offline_ingest_started, this,
          [this](int total) {
            append_log(
                LogLevel::Info,
                QStringLiteral("› 离线批量检测入库中（%1 帧）…").arg(total));
            update_solve_action_enabled();
          });
  connect(session_.get(), &SessionController::offline_ingest_finished, this,
          [this](int added, int skipped) {
            append_log(
                added > 0 ? LogLevel::Info : LogLevel::Warn,
                QStringLiteral("› 离线入库完成：成功 %1，跳过 %2")
                    .arg(added)
                    .arg(skipped));
            schedule_workbench_view_refresh(false);
            update_solve_action_enabled();
          });
  connect(session_.get(), &SessionController::intrinsics_state_changed, this,
          [this]() {
            if (session_ == nullptr || stack_ == nullptr) {
              return;
            }
            if (stack_->currentIndex() != static_cast<int>(PageId::Workbench)) {
              return;
            }
            refresh_intrinsics_workbench_ui();
          });
  connect(session_.get(), &SessionController::live_preview_updated, this, [this]() {
    if (session_ == nullptr || stack_ == nullptr ||
        stack_->currentIndex() != static_cast<int>(PageId::Workbench)) {
      return;
    }
    if (!preview_live_ || session_->detect_busy() || session_->has_current_detection()) {
      return;
    }
    const QImage img = session_->cached_live_preview_qimage();
    if (!img.isNull()) {
      show_preview_image(img);
    }
  });
  connect(session_.get(), &SessionController::stereo_live_preview_updated, this, [this]() {
    if (session_ == nullptr || stack_ == nullptr ||
        stack_->currentIndex() != static_cast<int>(PageId::Workbench)) {
      return;
    }
    if (!preview_live_ || !session_->uses_stereo_dual_session()) {
      return;
    }
    apply_stereo_raw_previews();
  });
  connect(session_.get(), &SessionController::offline_preview_updated, this, [this]() {
    if (session_ == nullptr || stack_ == nullptr ||
        stack_->currentIndex() != static_cast<int>(PageId::Workbench)) {
      return;
    }
    if (session_->source_mode() == SourceMode::RosTopic) {
      return;
    }
    if (!session_->last_preview().isNull()) {
      return;
    }
    const QImage img = session_->cached_offline_preview_qimage();
    if (!img.isNull()) {
      show_preview_image(img);
    }
  });
  connect(session_.get(), &SessionController::result_changed, this, [this]() {
    if (stack_ != nullptr) {
      const int idx = stack_->currentIndex();
      if (idx == static_cast<int>(PageId::Review) ||
          idx == static_cast<int>(PageId::StereoRectify)) {
        refresh_review_view();
      }
    }
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
      session_.get(), &SessionController::identify_started, this,
      &MainWindow::on_async_identify_started);
  connect(
      session_.get(), &SessionController::identify_finished, this,
      &MainWindow::on_async_identify_finished);
  connect(
      session_.get(), &SessionController::solve_started, this,
      &MainWindow::on_async_solve_started);
  connect(
      session_.get(), &SessionController::solve_progress, this,
      &MainWindow::on_async_solve_progress);
  connect(
      session_.get(), &SessionController::solve_finished, this,
      &MainWindow::on_async_solve_finished);
  connect(
      ros_bridge_.get(), &RosImageBridge::frame_received, this,
      &MainWindow::on_ros_frame);
  connect(
      ros_stereo_bridge_.get(), &RosStereoImageBridge::stereo_frames_updated, this,
      &MainWindow::on_stereo_ros_frames);
  connect(
      ros_bridge_.get(), &RosImageBridge::camera_info_received, this, [this]() {
        sync_detect_intrinsics_from_sources();
      });

  auto *preview_mode_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  preview_mode_shortcut->setContext(Qt::WindowShortcut);
  connect(preview_mode_shortcut, &QShortcut::activated, this, [this]() {
    if (stack_ == nullptr) {
      return;
    }
    const int idx = stack_->currentIndex();
    if (idx != static_cast<int>(PageId::Workbench) &&
        idx != static_cast<int>(PageId::DetectLab)) {
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
  if (ros_executor_hub_) {
    ros_executor_hub_->stop();
  }
  if (ros_bridge_) {
    ros_bridge_->unsubscribe();
  }
  if (ros_stereo_bridge_) {
    ros_stereo_bridge_->unsubscribe();
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
  bind_toolbar_action(act_setup_, TbGlyph::Setup, QStringLiteral("标定设置"), ink, accent);
  bind_toolbar_action(
      act_workbench_, TbGlyph::Workbench, QStringLiteral("工作台"), ink, accent);
  bind_toolbar_action(act_review_, TbGlyph::Review, QStringLiteral("复核导出"), ink, accent);
  bind_toolbar_action(act_offline_, TbGlyph::Offline, QStringLiteral("离线模式"), ink, accent);
  bind_toolbar_action(
      act_online_, TbGlyph::Online, QStringLiteral("在线模式（ROS）"), ink, accent);
  bind_toolbar_action(act_capture_, TbGlyph::Capture, QStringLiteral("采集帧"), ink, accent);
  bind_toolbar_action(act_solve_, TbGlyph::Solve, QStringLiteral("求解"), ink, accent);
  bind_toolbar_action(act_export_, TbGlyph::Export, QStringLiteral("导出结果…"), ink, accent);
  refresh_ready_indicator(ready_indicator_ok_, ready_indicator_tip_);
  refresh_online_indicator();
}

/// \brief 右上角准备就绪图标：OK 用主题色，未就绪灰色
void MainWindow::refresh_ready_indicator(bool ready, const QString &detail_tip) {
  ready_indicator_ok_ = ready;
  ready_indicator_tip_ = detail_tip;
  if (btn_ready_indicator_ == nullptr) {
    return;
  }
  const QColor accent(theme_icon_accent(theme_id_));
  const QColor gray =
      (theme_id_ == ThemeId::Light) ? QColor(0x9a, 0xa0, 0xa6) : QColor(0x6a, 0x75, 0x84);
  const QColor color = ready ? accent : gray;
  btn_ready_indicator_->setIcon(
      make_toolbar_icon(TbGlyph::Ready, color, color));
  const QString tip =
      ready ? QStringLiteral("准备就绪") : QStringLiteral("未就绪");
  btn_ready_indicator_->setToolTip(
      detail_tip.isEmpty() ? tip : (tip + QStringLiteral("\n") + detail_tip));
  btn_ready_indicator_->setStatusTip(tip);
}

/// \brief 右上角单一在线图标：点亮=ROS 在线，灰态=离线
void MainWindow::refresh_online_indicator() {
  if (btn_online_indicator_ == nullptr) {
    return;
  }
  const QColor accent(theme_icon_accent(theme_id_));
  const QColor gray =
      (theme_id_ == ThemeId::Light) ? QColor(0x9a, 0xa0, 0xa6) : QColor(0x6a, 0x75, 0x84);
  const QColor color = online_mode_ ? accent : gray;
  btn_online_indicator_->blockSignals(true);
  btn_online_indicator_->setChecked(online_mode_);
  btn_online_indicator_->blockSignals(false);
  btn_online_indicator_->setIcon(make_toolbar_icon(TbGlyph::Online, color, color));
  const QString tip =
      online_mode_ ? QStringLiteral("ROS 在线 · 点击切换为离线")
                   : QStringLiteral("离线 · 点击切换为 ROS 在线");
  btn_online_indicator_->setToolTip(tip);
  btn_online_indicator_->setStatusTip(
      online_mode_ ? QStringLiteral("ROS 在线") : QStringLiteral("离线"));
}

/// \brief 更新顶部步骤条状态（双目内参 6 步，其余 5 步；DetectLab 不计入）
void MainWindow::update_step_rail(PageId page) {
  static const char *titles_mono[] = {
      "1  选择任务", "2  数据源设置", "3  标定设置", "4  采集求解", "5  复核导出"};
  static const char *titles_stereo[] = {"1  选择任务", "2  数据源设置", "3  标定设置",
                                        "4  采集求解", "5  校正验证", "6  复核导出"};
  const bool stereo_flow = uses_stereo_rectify_flow();
  const int step_count = stereo_flow ? 6 : 5;
  const int active_step = step_index_for_page(page);

  for (int i = 0; i < 6; ++i) {
    if (step_labels_[i] == nullptr) {
      continue;
    }
    const bool visible = i < step_count;
    step_labels_[i]->setVisible(visible);
    if (i < 5 && step_arrows_[i] != nullptr) {
      step_arrows_[i]->setVisible(visible && i < step_count - 1);
    }
    if (!visible) {
      continue;
    }
    const char *title = stereo_flow ? titles_stereo[i] : titles_mono[i];
    step_labels_[i]->setText(QString::fromUtf8(title));
    if (page == PageId::DetectLab) {
      step_labels_[i]->setObjectName(QStringLiteral("StepIdle"));
    } else if (active_step >= 0 && i < active_step) {
      step_labels_[i]->setObjectName(QStringLiteral("StepDone"));
    } else if (active_step >= 0 && i == active_step) {
      step_labels_[i]->setObjectName(QStringLiteral("StepActive"));
    } else {
      step_labels_[i]->setObjectName(QStringLiteral("StepIdle"));
    }
    step_labels_[i]->style()->unpolish(step_labels_[i]);
    step_labels_[i]->style()->polish(step_labels_[i]);
  }
}

bool MainWindow::uses_stereo_rectify_flow() const {
  return selected_calibrator_id_ == QStringLiteral("stereo_intrinsics");
}

MainWindow::PageId MainWindow::page_id_for_step_index(int step) const {
  if (uses_stereo_rectify_flow()) {
    switch (step) {
      case 0:
        return PageId::Home;
      case 1:
        return PageId::DataSource;
      case 2:
        return PageId::Setup;
      case 3:
        return PageId::Workbench;
      case 4:
        return PageId::StereoRectify;
      case 5:
        return PageId::Review;
      default:
        return PageId::Home;
    }
  }
  switch (step) {
    case 0:
      return PageId::Home;
    case 1:
      return PageId::DataSource;
    case 2:
      return PageId::Setup;
    case 3:
      return PageId::Workbench;
    case 4:
      return PageId::Review;
    default:
      return PageId::Home;
  }
}

int MainWindow::step_index_for_page(PageId page) const {
  if (page == PageId::DetectLab) {
    return -1;
  }
  if (uses_stereo_rectify_flow()) {
    if (static_cast<int>(page) <= static_cast<int>(PageId::Review)) {
      return static_cast<int>(page);
    }
    return -1;
  }
  switch (page) {
    case PageId::Home:
      return 0;
    case PageId::DataSource:
      return 1;
    case PageId::Setup:
      return 2;
    case PageId::Workbench:
      return 3;
    case PageId::StereoRectify:
      return 4;
    case PageId::Review:
      return 4;
    default:
      return -1;
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
                           .arg(QDateTime::currentDateTime().toString(
                                    QStringLiteral("yyyyMMdd-HHmmss-zzz")),
                                QString::fromUtf8(AppLogger::level_tag(level)), line);

  QTextCursor cursor = log_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(text, fmt);
  log_->setTextCursor(cursor);
  log_->ensureCursorVisible();
}

/// \brief 当前任务类型短文案（状态栏右侧）
QString MainWindow::current_task_status_text() const {
  const QString id = selected_calibrator_id_;
  if (id.isEmpty()) {
    return QStringLiteral("未选择任务");
  }

  auto target_zh = [](const QString &t) -> QString {
    if (t == QStringLiteral("chessboard") || t == QStringLiteral("trihedral_chess")) {
      return QStringLiteral("棋盘格");
    }
    if (t == QStringLiteral("charuco") || t == QStringLiteral("trihedral_charuco")) {
      return QStringLiteral("ChArUco");
    }
    if (t == QStringLiteral("aruco") || t == QStringLiteral("trihedral_aruco")) {
      return QStringLiteral("ArUco");
    }
    if (t == QStringLiteral("aruco_grid")) {
      return QStringLiteral("ArUco 阵列");
    }
    if (t == QStringLiteral("aprilgrid")) {
      return QStringLiteral("AprilGrid");
    }
    if (t == QStringLiteral("circles_symmetric")) {
      return QStringLiteral("对称圆点");
    }
    if (t == QStringLiteral("circles_asymmetric")) {
      return QStringLiteral("非对称圆点");
    }
    return t.isEmpty() ? QString() : t;
  };

  auto grid_suffix = [](const QString &tid, int sx, int sy) -> QString {
    if (tid == QStringLiteral("aruco") || tid.isEmpty()) {
      return QString();
    }
    if (sx <= 0 || sy <= 0) {
      return QString();
    }
    if (tid.startsWith(QStringLiteral("trihedral"))) {
      return QStringLiteral(" %1×%1").arg(sx);
    }
    return QStringLiteral(" %1×%2").arg(sx).arg(sy);
  };

  if (id == QStringLiteral("detect_lab_identify")) {
    return QStringLiteral("调试 · 标定板类型识别");
  }

  QString target;
  int sx = 0;
  int sy = 0;
  const bool use_lab_board =
      (id == QStringLiteral("detect_lab") || id == QStringLiteral("detect_lab_full")) &&
      combo_lab_target_ != nullptr;
  if (use_lab_board) {
    target = combo_lab_target_->currentData().toString();
    if (target.isEmpty()) {
      target = combo_lab_target_->currentText();
    }
    sx = spin_lab_squares_x_ != nullptr ? spin_lab_squares_x_->value() : 0;
    sy = spin_lab_squares_y_ != nullptr ? spin_lab_squares_y_->value() : 0;
  } else if (launcher_panel_ != nullptr) {
    target = launcher_panel_->target_type_id();
    sx = launcher_panel_->squares_x();
    sy = launcher_panel_->squares_y();
  }

  const QString board = target_zh(target) + grid_suffix(target, sx, sy);

  if (id == QStringLiteral("detect_lab")) {
    return board.isEmpty() ? QStringLiteral("调试 · 局部特征检测")
                           : QStringLiteral("调试 · 局部特征检测 · %1").arg(board);
  }
  if (id == QStringLiteral("detect_lab_full")) {
    return board.isEmpty() ? QStringLiteral("调试 · 完整板检测")
                           : QStringLiteral("调试 · 完整板检测 · %1").arg(board);
  }

  QString head;
  if (id == QStringLiteral("cam_intrinsics")) {
    head = QStringLiteral("单目内参");
  } else if (id == QStringLiteral("stereo_intrinsics")) {
    head = QStringLiteral("双目内参");
  } else if (id == QStringLiteral("stereo_extrinsics")) {
    head = QStringLiteral("双目外参");
  } else if (id == QStringLiteral("eye_in_hand")) {
    head = QStringLiteral("眼在手上");
  } else if (id == QStringLiteral("eye_to_hand")) {
    head = QStringLiteral("眼在手外");
  } else if (id == QStringLiteral("trihedral_oneshot")) {
    head = QStringLiteral("三面靶");
  } else if (selected_tile_ != nullptr) {
    head = selected_tile_->property("calibrator_title").toString();
    if (head.isEmpty()) {
      head = id;
    }
  } else {
    head = id;
  }

  if (board.isEmpty()) {
    return head;
  }
  return QStringLiteral("%1 · %2").arg(head, board);
}

/// \brief 刷新状态栏右侧任务类型
void MainWindow::refresh_status_task() {
  if (status_task_ == nullptr) {
    return;
  }
  const QString text = current_task_status_text();
  status_task_->setText(text);
  status_task_->setToolTip(text);
}

/// \brief 刷新状态栏模式/页面/提示
void MainWindow::update_status_bar(PageId page) {
  static const char *names[] = {
      "PAGE / HOME", "PAGE / DATA SOURCE", "PAGE / SETUP", "PAGE / WORKBENCH",
      "PAGE / STEREO RECTIFY", "PAGE / REVIEW", "PAGE / DETECT LAB"};
  if (status_page_ != nullptr) {
    status_page_->setText(QString::fromUtf8(names[static_cast<int>(page)]));
  }
  if (status_hint_ != nullptr) {
    if (page == PageId::DetectLab) {
      if (online_mode_ && session_ &&
          session_->source_mode() == SourceMode::RosTopic) {
        status_hint_->setText(
            preview_live_
                ? QStringLiteral("调试 · 实时预览（空格冻结画面）")
                : QStringLiteral("调试 · 画面已冻结（空格恢复实时）"));
      } else {
        status_hint_->setText(
            QStringLiteral("调试台 · 只做检测/识别，不转入采集求解"));
      }
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
          online_mode_ ? QStringLiteral("在线 · 进入「采集求解」后开始订阅图像")
                       : QStringLiteral("离线 · 选图片目录后采集求解"));
    }
  }
  refresh_status_task();
}

/// \brief 切换在线/离线模式并同步源 UI
void MainWindow::set_online_mode(bool online) {
  online_mode_ = online;
  if (act_online_ != nullptr) {
    act_online_->blockSignals(true);
    act_online_->setChecked(online);
    act_online_->blockSignals(false);
  }
  if (act_offline_ != nullptr) {
    act_offline_->blockSignals(true);
    act_offline_->setChecked(!online);
    act_offline_->blockSignals(false);
  }
  refresh_online_indicator();
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
  const PageId prev =
      stack_ != nullptr ? static_cast<PageId>(stack_->currentIndex()) : PageId::Home;
  const bool was_live =
      prev == PageId::Workbench || prev == PageId::DetectLab;
  const bool will_live = page == PageId::Workbench || page == PageId::DetectLab;
  if (was_live && !will_live) {
    stop_ros_image_pipeline();
  }

  if (stack_ != nullptr) {
    stack_->setCurrentIndex(static_cast<int>(page));
  }
  update_step_rail(page);
  update_status_bar(page);

  // 复核页隐藏底部系统日志，把高度留给结果摘要
  if (log_panel_host_ != nullptr) {
    log_panel_host_->setVisible(page != PageId::Review && page != PageId::StereoRectify);
  }

  if (act_home_ != nullptr) {
    act_home_->setChecked(page == PageId::Home);
    if (act_data_source_ != nullptr) {
      act_data_source_->setChecked(page == PageId::DataSource);
    }
    act_setup_->setChecked(page == PageId::Setup);
    act_workbench_->setChecked(page == PageId::Workbench);
    act_review_->setChecked(page == PageId::Review);
  }

  const bool on_work = (page == PageId::Workbench);
  if (act_capture_ != nullptr) {
    act_capture_->setEnabled(on_work);
  }
  if (act_solve_ != nullptr) {
    act_solve_->setEnabled(
        on_work && session_ && !session_->solve_busy() && session_->can_solve() &&
        !session_->offline_ingest_busy());
  }
  if (act_export_ != nullptr) {
    act_export_->setEnabled(session_ && session_->has_result());
  }

  if (page == PageId::DataSource) {
    refresh_setup_source_ui();
    refresh_setup_readiness();
    refresh_task_flow_chrome();
  } else if (page == PageId::Setup) {
    refresh_handeye_ui();
    refresh_setup_readiness();
    refresh_task_flow_chrome();
  } else if (page == PageId::Workbench) {
    maybe_clear_observations_on_workbench_enter();
    refresh_workbench_view(false);
    QTimer::singleShot(0, this, [this]() { schedule_workbench_preview_load(); });
    if (session_ != nullptr && session_->source_mode() == SourceMode::RosTopic) {
      preview_live_ = true;
      start_ros_image_pipeline();
    }
    refresh_task_flow_chrome();
  } else if (page == PageId::StereoRectify) {
    refresh_stereo_rectify_view();
    refresh_task_flow_chrome();
  } else if (page == PageId::Review) {
    refresh_review_view();
    refresh_task_flow_chrome();
  } else if (page == PageId::DetectLab) {
    if (session_ != nullptr) {
      session_->sync_detect_lab_mode_from_task_id(selected_calibrator_id_);
      if (session_->source_mode() == SourceMode::RosTopic) {
        preview_live_ = true;
      }
    }
    refresh_lab_mode_ui();
    // 沿用数据源页已配置的图像 / CameraInfo
    if (session_ != nullptr && launcher_panel_ != nullptr) {
      if (edit_lab_image_dir_ != nullptr && edit_image_dir_ != nullptr &&
          !edit_image_dir_->text().trimmed().isEmpty()) {
        edit_lab_image_dir_->setText(edit_image_dir_->text().trimmed());
      }
      if (combo_lab_camera_info_ != nullptr && combo_camera_info_topic_ != nullptr &&
          combo_lab_camera_info_->currentText().trimmed().isEmpty() &&
          !combo_camera_info_topic_->currentText().trimmed().isEmpty()) {
        combo_lab_camera_info_->setEditText(
            combo_camera_info_topic_->currentText().trimmed());
      }
      if (session_->source_mode() == SourceMode::Offline &&
          session_->image_paths().isEmpty() && edit_lab_image_dir_ != nullptr &&
          !edit_lab_image_dir_->text().trimmed().isEmpty()) {
        session_->load_image_dir(edit_lab_image_dir_->text().trimmed());
      }
    }
    if (session_ != nullptr && session_->source_mode() == SourceMode::RosTopic) {
      on_lab_refresh_camera_info();
      start_ros_image_pipeline();
    }
    apply_lab_camera_info_subscription();
    sync_detect_intrinsics_from_sources();
    refresh_detect_lab_view(false);
    update_preview_mode_ui();
  }

  QString page_name = QStringLiteral("未知");
  switch (page) {
    case PageId::Home:
      page_name = QStringLiteral("首页");
      break;
    case PageId::DataSource:
      page_name = QStringLiteral("数据源设置");
      break;
    case PageId::Setup:
      page_name = QStringLiteral("标定设置");
      break;
    case PageId::Workbench:
      page_name = QStringLiteral("工作台");
      break;
    case PageId::StereoRectify:
      page_name = QStringLiteral("校正验证");
      break;
    case PageId::Review:
      page_name = QStringLiteral("复核导出");
      break;
    case PageId::DetectLab:
      if (is_detect_lab_identify_mode()) {
        page_name = QStringLiteral("类型识别");
      } else if (is_detect_lab_full_mode()) {
        page_name = QStringLiteral("完整板检测");
      } else {
        page_name = QStringLiteral("局部特征检测");
      }
      break;
  }
  append_log(LogLevel::Info, QStringLiteral("› 进入「%1」").arg(page_name));
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
  auto *act_refresh_projects = file_menu->addAction(QStringLiteral("刷新项目列表"));
  connect(act_refresh_projects, &QAction::triggered, this, [this]() {
    refresh_project_list();
    append_log(LogLevel::Info, QStringLiteral("› 已刷新项目列表"));
  });
  auto *act_open_projects_root = file_menu->addAction(QStringLiteral("打开项目总目录"));
  connect(
      act_open_projects_root, &QAction::triggered, this,
      &MainWindow::on_open_projects_root_dir);
  auto *act_import_project = file_menu->addAction(QStringLiteral("导入项目…"));
  connect(act_import_project, &QAction::triggered, this, &MainWindow::on_import_project);
  auto *act_new_project = file_menu->addAction(QStringLiteral("新建项目…"));
  connect(act_new_project, &QAction::triggered, this, &MainWindow::on_new_project);
  file_menu->addSeparator();
  auto *act_quit = file_menu->addAction(QStringLiteral("退出"));
  act_quit->setShortcut(QKeySequence::Quit);
  connect(act_quit, &QAction::triggered, this, &QWidget::close);

  auto *view_menu = menuBar()->addMenu(QStringLiteral("视图(&V)"));
  act_home_ = view_menu->addAction(QStringLiteral("首页"));
  act_data_source_ = view_menu->addAction(QStringLiteral("数据源设置"));
  act_setup_ = view_menu->addAction(QStringLiteral("标定设置"));
  act_workbench_ = view_menu->addAction(QStringLiteral("工作台"));
  act_review_ = view_menu->addAction(QStringLiteral("复核导出"));
  for (QAction *a :
       {act_home_, act_data_source_, act_setup_, act_workbench_, act_review_}) {
    a->setCheckable(true);
  }
  connect(act_home_, &QAction::triggered, this, [this]() { go_to(PageId::Home); });
  connect(
      act_data_source_, &QAction::triggered, this, [this]() { go_to(PageId::DataSource); });
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
  auto *act_about = help_menu->addAction(QStringLiteral("关于 HS Calib Suite(&A)…"));
  act_about->setMenuRole(QAction::AboutRole);
  connect(act_about, &QAction::triggered, this, &MainWindow::show_about_dialog);
}

/// \brief 关于对话框（产品名 / 版本 / 运行时 / 许可）
void MainWindow::show_about_dialog() {
  auto *dlg = new QDialog(this);
  dlg->setAttribute(Qt::WA_DeleteOnClose);
  dlg->setWindowTitle(QStringLiteral("关于 HS Calib Suite"));
  dlg->setModal(true);
  dlg->setFixedWidth(460);

  auto *root = new QVBoxLayout(dlg);
  root->setContentsMargins(24, 22, 24, 18);
  root->setSpacing(14);

  auto *hero = new QHBoxLayout;
  hero->setSpacing(16);
  auto *icon = new QLabel(dlg);
  icon->setFixedSize(64, 64);
  icon->setAlignment(Qt::AlignCenter);
  const QColor accent(theme_icon_accent(theme_id_));
  icon->setPixmap(make_toolbar_icon(TbGlyph::Ready, accent, accent).pixmap(QSize(48, 48)));
  hero->addWidget(icon, 0, Qt::AlignTop);

  auto *titles = new QVBoxLayout;
  titles->setSpacing(4);
  auto *name = new QLabel(QStringLiteral("HS Calib Suite"), dlg);
  name->setObjectName(QStringLiteral("PageTitle"));
  auto *ver = new QLabel(QStringLiteral("版本 0.1.0"), dlg);
  ver->setObjectName(QStringLiteral("PageSubtitle"));
  auto *tag = new QLabel(
      QStringLiteral("机器人现场标定工作台 · 工程标定 / 检测调试"), dlg);
  tag->setObjectName(QStringLiteral("Muted"));
  tag->setWordWrap(true);
  titles->addWidget(name);
  titles->addWidget(ver);
  titles->addWidget(tag);
  titles->addStretch(1);
  hero->addLayout(titles, 1);
  root->addLayout(hero);

  auto *line = new QFrame(dlg);
  line->setFrameShape(QFrame::HLine);
  line->setObjectName(QStringLiteral("AboutSep"));
  root->addWidget(line);

  auto *info = new QLabel(dlg);
  info->setObjectName(QStringLiteral("Muted"));
  info->setWordWrap(true);
  info->setTextInteractionFlags(Qt::TextSelectableByMouse);
  info->setText(QStringLiteral(
      "<p style='margin:0 0 8px 0; line-height:1.45;'>"
      "面向机器人现场的标定工作台：正式路径产出可用标定结果；"
      "调试路径帮助确认靶标可见、类型可识别、参数设置正确。"
      "</p>"
      "<p style='margin:0; line-height:1.55;'>"
      "<b>运行环境</b><br/>"
      "Qt %1<br/>"
      "OpenCV %2<br/>"
      "ROS 2 Humble · rclcpp<br/>"
      "主题：%3"
      "</p>")
                     .arg(QString::fromLatin1(qVersion()))
                     .arg(QStringLiteral("%1.%2.%3")
                              .arg(CV_VERSION_MAJOR)
                              .arg(CV_VERSION_MINOR)
                              .arg(CV_VERSION_REVISION))
                     .arg(theme_display_name(theme_id_)));
  root->addWidget(info);

  auto *copy = new QLabel(
      QStringLiteral("© %1 Hoson Soft · License Apache-2.0")
          .arg(QDateTime::currentDateTime().date().year()),
      dlg);
  copy->setObjectName(QStringLiteral("Muted"));
  copy->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  root->addWidget(copy);

  auto *btns = new QDialogButtonBox(QDialogButtonBox::Close, dlg);
  btns->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
  btns->button(QDialogButtonBox::Close)->setObjectName(QStringLiteral("PrimaryButton"));
  connect(btns, &QDialogButtonBox::rejected, dlg, &QDialog::reject);
  connect(btns, &QDialogButtonBox::accepted, dlg, &QDialog::accept);
  connect(btns->button(QDialogButtonBox::Close), &QPushButton::clicked, dlg, &QDialog::accept);
  root->addWidget(btns);

  dlg->adjustSize();
  dlg->exec();
}

/// \brief 菜单栏右上角：准备就绪 / ROS 在线（点亮）
void MainWindow::setup_mode_corner() {
  auto *corner = new QWidget(menuBar());
  corner->setObjectName(QStringLiteral("ModeCorner"));
  auto *lay = new QHBoxLayout(corner);
  lay->setContentsMargins(4, 0, 10, 0);
  lay->setSpacing(4);

  btn_ready_indicator_ = new QToolButton(corner);
  btn_ready_indicator_->setObjectName(QStringLiteral("ReadyIndicator"));
  btn_ready_indicator_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  btn_ready_indicator_->setAutoRaise(true);
  btn_ready_indicator_->setIconSize(QSize(18, 18));
  btn_ready_indicator_->setFocusPolicy(Qt::NoFocus);
  btn_ready_indicator_->setCursor(Qt::ArrowCursor);
  lay->addWidget(btn_ready_indicator_);

  btn_online_indicator_ = new QToolButton(corner);
  btn_online_indicator_->setObjectName(QStringLiteral("OnlineIndicator"));
  btn_online_indicator_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  btn_online_indicator_->setAutoRaise(true);
  btn_online_indicator_->setCheckable(true);
  btn_online_indicator_->setIconSize(QSize(18, 18));
  btn_online_indicator_->setFocusPolicy(Qt::NoFocus);
  btn_online_indicator_->setCursor(Qt::PointingHandCursor);
  connect(btn_online_indicator_, &QToolButton::clicked, this, [this](bool on) {
    set_online_mode(on);
  });
  lay->addWidget(btn_online_indicator_);

  menuBar()->setCornerWidget(corner, Qt::TopRightCorner);
  refresh_ready_indicator(false, QStringLiteral("尚未完成就绪检查"));
  refresh_online_indicator();
}

/// \brief 创建状态栏标签
void MainWindow::setup_status_bar() {
  status_mode_ = new QLabel(QStringLiteral("离线"), this);
  status_mode_->setObjectName(QStringLiteral("StatusBarModeOffline"));
  status_page_ = new QLabel(QStringLiteral("PAGE / HOME"), this);
  status_page_->setObjectName(QStringLiteral("StatusBarPage"));
  status_hint_ = new QLabel(QStringLiteral("就绪"), this);
  status_task_ = new QLabel(QStringLiteral("未选择任务"), this);
  status_task_->setObjectName(QStringLiteral("StatusBarTask"));
  status_task_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  statusBar()->addWidget(status_mode_);
  statusBar()->addWidget(status_page_);
  statusBar()->addWidget(status_hint_, 1);
  statusBar()->addPermanentWidget(status_task_);
  refresh_status_task();
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
  for (int i = 0; i < 6; ++i) {
    step_labels_[i] = new QLabel(steps);
    step_labels_[i]->setCursor(Qt::PointingHandCursor);
    step_labels_[i]->setToolTip(QStringLiteral("点击切换到此步骤"));
    step_labels_[i]->installEventFilter(this);
    steps_layout->addWidget(step_labels_[i]);
    if (i < 5) {
      step_arrows_[i] = make_label(QStringLiteral("→"), QStringLiteral("Muted"), steps);
      steps_layout->addWidget(step_arrows_[i]);
    }
  }
  steps_layout->addStretch(1);
  steps_layout->addWidget(
      make_label(QStringLiteral("HS CALIB SUITE"), QStringLiteral("BrandMark"), steps));
  root_layout->addWidget(steps);

  stack_ = new QStackedWidget(root);
  stack_->addWidget(build_home_page());
  stack_->addWidget(build_data_source_page());
  stack_->addWidget(build_setup_page());
  stack_->addWidget(build_workbench_page());
  stack_->addWidget(build_stereo_rectify_page());
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
  log_panel_host_ = new QWidget(root);
  auto *log_wrap_layout = new QVBoxLayout(log_panel_host_);
  log_wrap_layout->setContentsMargins(16, 8, 16, 12);
  log_wrap_layout->addWidget(make_panel(QStringLiteral("系统日志"), log_body));
  root_layout->addWidget(log_panel_host_);

  setCentralWidget(root);
}

// ===== 页面 =====

}  // namespace gui
}  // namespace hs_calib
