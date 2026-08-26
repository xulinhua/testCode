#include <QImage>
#include <QSignalBlocker>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/gui/window/main_window.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_parameter_dialog.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_workbench_panels.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_calibration_bars_dialog.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_calibration_rms_dialog.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_export.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/widgets/review_charts_widget.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/gui/task_flow/task_flow.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/bridges/ros_stereo_image_bridge.hpp"
#include "hs_calib_suite/gui/bridges/bag_image_loader.hpp"
#include "hs_calib_suite/gui/bridges/ros_bag_frame_reader.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"
#include <QMessageBox>

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/io/board_config_yaml.hpp"
#include "hs_calib_suite/core/review/review_diagnostics.hpp"

#include <functional>
#include <thread>
#include <memory>
#include <algorithm>
#include <map>

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QCursor>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QIODevice>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QProgressDialog>
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
#include <QTextStream>
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


/// \brief 把设置页/Launcher 参数同步进会话
void MainWindow::sync_session_from_setup_ui() {
  if (session_ == nullptr) {
    return;
  }
  if (launcher_panel_ != nullptr) {
    launcher_panel_->set_calibrator_id(selected_calibrator_id_);
    launcher_panel_->apply_to_session(session_.get());
    stats_backend_ = launcher_panel_->stats_backend().toStdString();
    // 显示叠加只在工作台预览区改，实时生效；此处不覆盖
    apply_workbench_viz_to_session();
    return;
  }
  session_->set_calibrator_id(selected_calibrator_id_);
}

/// \brief 把会话可视化选项同步到工作台勾选
void MainWindow::sync_workbench_viz_from_session() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->uses_tier4_intrinsics()) {
    if (intrinsics_viz_options_dlg_ != nullptr) {
      intrinsics_viz_options_dlg_->load_from_session();
    }
    return;
  }
  if (chk_viz_corners_wb_ != nullptr) {
    chk_viz_corners_wb_->setChecked(session_->viz_corners());
  }
  if (chk_viz_hull_wb_ != nullptr) {
    chk_viz_hull_wb_->setChecked(session_->viz_hull());
  }
  if (chk_viz_conf_wb_ != nullptr) {
    chk_viz_conf_wb_->setChecked(session_->viz_conf());
  }
  if (chk_viz_aruco_wb_ != nullptr) {
    chk_viz_aruco_wb_->setChecked(session_->viz_aruco());
  }
  if (spin_viz_marker_wb_ != nullptr) {
    spin_viz_marker_wb_->setValue(session_->viz_marker_radius());
  }
}

/// \brief 把工作台可视化勾选写回会话
void MainWindow::apply_workbench_viz_to_session() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->uses_tier4_intrinsics()) {
    const IntrinsicsVizOptions opts = session_->intrinsics_viz_options();
    const int radius =
        spin_viz_marker_wb_ != nullptr ? spin_viz_marker_wb_->value() : 4;
    session_->set_viz_options(
        opts.draw_detection, opts.draw_detection, opts.draw_detection, radius,
        opts.draw_detection);
    return;
  }
  const bool corners =
      chk_viz_corners_wb_ != nullptr ? chk_viz_corners_wb_->isChecked() : true;
  const bool hull = chk_viz_hull_wb_ != nullptr ? chk_viz_hull_wb_->isChecked() : true;
  const bool conf = chk_viz_conf_wb_ != nullptr ? chk_viz_conf_wb_->isChecked() : true;
  const bool aruco = chk_viz_aruco_wb_ != nullptr ? chk_viz_aruco_wb_->isChecked() : true;
  const int radius =
      spin_viz_marker_wb_ != nullptr ? spin_viz_marker_wb_->value() : 4;
  session_->set_viz_options(corners, hull, conf, radius, aruco);
}

void MainWindow::refresh_workbench_preview_viz() {
  apply_workbench_viz_to_session();
  if (session_ == nullptr) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic || !preview_live_) {
    pending_detect_log_ = false;
    session_->request_detect(false);
  }
}

/// \brief 按标定类型显隐标定设置相关块
void MainWindow::refresh_handeye_ui() {
  if (launcher_panel_ != nullptr) {
    launcher_panel_->set_calibrator_id(selected_calibrator_id_);
  }
}

/// \brief 刷新设置页就绪检查清单
void MainWindow::refresh_setup_readiness() {
  if (session_ == nullptr) {
    return;
  }
  const bool he = session_->is_handeye() ||
                  selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
                  selected_calibrator_id_ == QStringLiteral("eye_to_hand");
  const bool ros_mode =
      (combo_source_mode_ != nullptr &&
       combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic)) ||
      session_->source_mode() == SourceMode::RosTopic;
  const bool bag_mode =
      (combo_source_mode_ != nullptr &&
       combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosBag)) ||
      session_->source_mode() == SourceMode::RosBag;

  QStringList parts;
  bool can_start = true;
  if (ros_mode) {
    const bool ros_ok = ros_bridge_ && ros_bridge_->is_ready();
    bool topic_ok = false;
    QString topic;
    if (launcher_panel_ != nullptr && launcher_panel_->uses_stereo_dual_topics()) {
      const QString left =
          launcher_panel_->combo_left_image_topic() != nullptr
              ? launcher_panel_->combo_left_image_topic()->currentText().trimmed()
              : QString();
      const QString right =
          launcher_panel_->combo_right_image_topic() != nullptr
              ? launcher_panel_->combo_right_image_topic()->currentText().trimmed()
              : QString();
      topic_ok = !left.isEmpty() && !right.isEmpty();
      topic = launcher_panel_->active_image_topic();
    } else {
      topic =
          combo_image_topic_ != nullptr ? combo_image_topic_->currentText().trimmed()
                                        : QString();
      topic_ok = !topic.isEmpty();
    }
    const bool frame_ok = ros_bridge_ && ros_bridge_->has_frame();
    parts << (ros_ok ? QStringLiteral("ROS✓") : QStringLiteral("ROS○"));
    parts << (topic_ok ? QStringLiteral("话题✓") : QStringLiteral("话题○"));
    parts << (frame_ok ? QStringLiteral("有图✓") : QStringLiteral("有图○"));
    can_start = ros_ok && topic_ok;
  } else if (bag_mode) {
    const QString bag =
        launcher_panel_ && launcher_panel_->edit_bag_path()
            ? launcher_panel_->edit_bag_path()->text().trimmed()
            : QString();
    const int n = session_->image_paths().size();
    const bool bag_ok = !bag.isEmpty();
    const bool frames_ok = n > 0;
    parts << (bag_ok ? QStringLiteral("Bag✓") : QStringLiteral("Bag○"));
    parts << (frames_ok ? QStringLiteral("帧✓ %1").arg(n) : QStringLiteral("帧○ 点「加载帧」"));
    can_start = bag_ok && frames_ok;
  } else {
    const QString dir =
        edit_image_dir_ != nullptr ? edit_image_dir_->text().trimmed() : QString();
    const int n = session_->image_paths().size();
    const bool dir_ok = !dir.isEmpty() && n > 0;
    parts << (dir_ok ? QStringLiteral("目录✓ %1张").arg(n)
                     : QStringLiteral("目录○"));
    can_start = dir_ok;
  }
  {
    QString target = QStringLiteral("chessboard");
    QString model = QStringLiteral("brown_conrady");
    int min_views = 12;
    if (launcher_panel_ != nullptr) {
      if (launcher_panel_->combo_target_type() != nullptr) {
        target = launcher_panel_->target_type_id();
      }
      if (launcher_panel_->combo_camera_model() != nullptr) {
        const QVariant d = launcher_panel_->combo_camera_model()->currentData();
        model = d.isValid() && !d.toString().isEmpty()
            ? d.toString()
            : launcher_panel_->combo_camera_model()->currentText();
      }
      min_views = launcher_panel_->min_views();
    }
    parts << QStringLiteral("%1 · %2 · ≥%3姿").arg(target, model).arg(min_views);
  }

  if (he) {
    QString cam;
    if (launcher_panel_ != nullptr && launcher_panel_->intrinsics_source_mode() == 2 &&
        launcher_panel_->edit_intrinsics_yaml() != nullptr) {
      cam = launcher_panel_->edit_intrinsics_yaml()->text().trimmed();
    }
    if (cam.isEmpty() && edit_camera_yaml_ != nullptr) {
      cam = edit_camera_yaml_->text().trimmed();
    }
    const bool cam_ok = !cam.isEmpty();
    parts << (cam_ok ? QStringLiteral("内参✓") : QStringLiteral("内参○"));
    can_start = can_start && cam_ok;
    const int pose_mode =
        combo_pose_source_ != nullptr ? combo_pose_source_->currentData().toInt()
                                      : static_cast<int>(PoseSource::Csv);
    if (pose_mode == static_cast<int>(PoseSource::Csv)) {
      const bool csv_ok = session_->pose_csv_count() > 0;
      parts << (csv_ok ? QStringLiteral("CSV✓%1").arg(session_->pose_csv_count())
                       : QStringLiteral("CSV○"));
      can_start = can_start && csv_ok;
    } else {
      const bool tf_ok = tf_bridge_ && tf_bridge_->is_ready();
      parts << (tf_ok ? QStringLiteral("TF✓") : QStringLiteral("TF○"));
      can_start = can_start && tf_ok;
    }
  }

  const bool stereo_ext =
      selected_calibrator_id_ == QStringLiteral("stereo_extrinsics") ||
      (session_ && session_->is_stereo_extrinsics());
  if (stereo_ext && launcher_panel_ != nullptr) {
    const QString left =
        launcher_panel_->edit_left_camera_yaml()
            ? launcher_panel_->edit_left_camera_yaml()->text().trimmed()
            : QString();
    const QString right =
        launcher_panel_->edit_right_camera_yaml()
            ? launcher_panel_->edit_right_camera_yaml()->text().trimmed()
            : QString();
    const bool ok = !left.isEmpty() && !right.isEmpty();
    parts << (ok ? QStringLiteral("左右YAML✓") : QStringLiteral("左右YAML○"));
    can_start = can_start && ok;
  }

  const QString detail = parts.join(QStringLiteral("\n"));
  refresh_ready_indicator(can_start, detail);

  if (btn_start_session_ != nullptr) {
    btn_start_session_->setEnabled(can_start);
  }
  if (btn_datasource_next_ != nullptr) {
    // 数据源步：只要求图像源就绪（内参源可选）
    bool source_ok = true;
    if (ros_mode) {
      source_ok = ros_bridge_ && ros_bridge_->is_ready();
      if (launcher_panel_ != nullptr && launcher_panel_->uses_stereo_dual_topics()) {
        const QString left =
            launcher_panel_->combo_left_image_topic() != nullptr
                ? launcher_panel_->combo_left_image_topic()->currentText().trimmed()
                : QString();
        const QString right =
            launcher_panel_->combo_right_image_topic() != nullptr
                ? launcher_panel_->combo_right_image_topic()->currentText().trimmed()
                : QString();
        source_ok = source_ok && !left.isEmpty() && !right.isEmpty();
      } else {
        source_ok = source_ok && combo_image_topic_ != nullptr &&
                    !combo_image_topic_->currentText().trimmed().isEmpty();
      }
    } else if (bag_mode) {
      const QString bag =
          launcher_panel_ && launcher_panel_->edit_bag_path()
              ? launcher_panel_->edit_bag_path()->text().trimmed()
              : QString();
      source_ok = !bag.isEmpty() && session_->image_paths().size() > 0;
    } else {
      const QString dir =
          edit_image_dir_ != nullptr ? edit_image_dir_->text().trimmed() : QString();
      source_ok = !dir.isEmpty() && session_->image_paths().size() > 0;
    }
    btn_datasource_next_->setEnabled(source_ok);
    const bool lab = is_detect_lab_mode();
    btn_datasource_next_->setText(
        lab ? QStringLiteral("下一步：打开检测台")
            : QStringLiteral("下一步：标定设置"));
  }
}

/// \brief 按离线/ROS/Bag 切换源相关控件
void MainWindow::refresh_setup_source_ui() {
  if (launcher_panel_ != nullptr) {
    launcher_panel_->refresh_source_mode_rows();
  } else {
    const int mode =
        combo_source_mode_ != nullptr ? combo_source_mode_->currentData().toInt()
                                      : static_cast<int>(SourceMode::Offline);
    const bool ros_mode = mode == static_cast<int>(SourceMode::RosTopic);
    const bool offline = mode == static_cast<int>(SourceMode::Offline);
    if (offline_row_ != nullptr) {
      offline_row_->setVisible(offline);
    }
    if (topic_row_ != nullptr) {
      topic_row_->setVisible(ros_mode);
    }
  }
  update_workbench_mode_actions();
}

/// \brief 按任务流更新工作台/复核页标题
void MainWindow::refresh_task_flow_chrome() {
  const TaskFlowKind flow = task_flow_from_calibrator_id(selected_calibrator_id_);
  if (flow_workbench_title_ != nullptr) {
    flow_workbench_title_->setText(
        task_flow_step_title(flow, QStringLiteral("采集求解")));
  }
  if (flow_review_title_ != nullptr) {
    flow_review_title_->setText(
        task_flow_step_title(flow, QStringLiteral("复核导出")));
  }
  if (flow_rectify_title_ != nullptr) {
    flow_rectify_title_->setText(
        task_flow_step_title(flow, QStringLiteral("校正验证")));
  }
}

/// \brief 是否应在当前页订阅 ROS 图像（仅采集求解 / 调试台）
bool MainWindow::needs_ros_image_subscription() const {
  if (session_ == nullptr || session_->source_mode() != SourceMode::RosTopic) {
    return false;
  }
  if (stack_ == nullptr) {
    return false;
  }
  const int idx = stack_->currentIndex();
  return idx == static_cast<int>(PageId::Workbench) ||
         idx == static_cast<int>(PageId::DetectLab);
}

/// \brief 仅同步话题配置到会话，不触发订阅
void MainWindow::sync_pending_ros_topics() {
  if (session_ == nullptr || launcher_panel_ == nullptr) {
    return;
  }
  if (session_->uses_stereo_dual_session()) {
    const QString left =
        launcher_panel_->combo_left_image_topic() != nullptr
            ? launcher_panel_->combo_left_image_topic()->currentText().trimmed()
            : QString();
    const QString right =
        launcher_panel_->combo_right_image_topic() != nullptr
            ? launcher_panel_->combo_right_image_topic()->currentText().trimmed()
            : QString();
    if (!left.isEmpty() && !right.isEmpty()) {
      session_->set_stereo_ros_topics(left, right);
    }
    return;
  }
  const QString topic = launcher_panel_->active_image_topic();
  if (!topic.isEmpty()) {
    session_->set_ros_topic_name(topic);
  }
}

/// \brief 离开采集页：退订、停检、清预览
void MainWindow::stop_ros_image_pipeline() {
  if (ros_stereo_sub_debounce_ != nullptr) {
    ros_stereo_sub_debounce_->stop();
  }
  allow_auto_on_detect_finish_ = false;
  pending_detect_log_ = false;
  pending_capture_after_detect_ = false;
  if (session_ != nullptr) {
    session_->cancel_pending_detect();
    session_->clear_live_ros_frames();
  }
  if (ros_bridge_) {
    ros_bridge_->unsubscribe();
  }
  if (ros_stereo_bridge_) {
    ros_stereo_bridge_->unsubscribe();
  }
  clear_workbench_live_previews();
  update_detect_status_ui();
}

void MainWindow::clear_workbench_live_previews() {
  show_preview_image(QImage());
  if (stereo_preview_left_ != nullptr) {
    stereo_preview_left_->clear_image();
    stereo_preview_left_->set_placeholder(QStringLiteral("等待 ROS 图像…"));
  }
  if (stereo_preview_right_ != nullptr) {
    stereo_preview_right_->clear_image();
    stereo_preview_right_->set_placeholder(QStringLiteral("等待 ROS 图像…"));
  }
}

/// \brief 进入采集页：按当前话题订阅
void MainWindow::start_ros_image_pipeline() {
  if (!needs_ros_image_subscription()) {
    return;
  }
  sync_ros_image_subscription();
}

/// \brief 订阅当前任务对应的 ROS 图像话题
void MainWindow::schedule_ros_image_subscription() {
  sync_pending_ros_topics();
  if (!needs_ros_image_subscription()) {
    return;
  }
  if (session_ != nullptr && session_->uses_stereo_dual_session() &&
      session_->source_mode() == SourceMode::RosTopic) {
    if (ros_stereo_sub_debounce_ != nullptr) {
      ros_stereo_sub_debounce_->start();
      return;
    }
  }
  sync_ros_image_subscription();
}

void MainWindow::sync_ros_image_subscription() {
  sync_pending_ros_topics();
  if (!needs_ros_image_subscription()) {
    return;
  }
  if (session_ != nullptr && session_->uses_stereo_dual_session() &&
      session_->source_mode() == SourceMode::RosTopic) {
    sync_ros_stereo_subscription();
    return;
  }
  if (launcher_panel_ == nullptr) {
    return;
  }
  on_topic_changed(launcher_panel_->active_image_topic());
}

void MainWindow::sync_ros_stereo_subscription() {
  if (launcher_panel_ == nullptr || ros_stereo_bridge_ == nullptr) {
    return;
  }
  if (ros_bridge_) {
    ros_bridge_->unsubscribe();
  }
  const QString left =
      launcher_panel_->combo_left_image_topic() != nullptr
          ? launcher_panel_->combo_left_image_topic()->currentText().trimmed()
          : QString();
  const QString right =
      launcher_panel_->combo_right_image_topic() != nullptr
          ? launcher_panel_->combo_right_image_topic()->currentText().trimmed()
          : QString();
  if (left.isEmpty() || right.isEmpty()) {
    return;
  }
  ros_stereo_bridge_->subscribe(left, right, 30);
  if (session_ != nullptr) {
    session_->set_stereo_ros_topics(left, right);
  }
  append_log(
      LogLevel::Info,
      QStringLiteral("› 双目订阅 L=%1 · R=%2").arg(left, right));
}

static void fill_topic_combo(
    QComboBox *combo, const QStringList &topics, const QString &previous) {
  if (combo == nullptr) {
    return;
  }
  combo->blockSignals(true);
  combo->clear();
  combo->addItems(topics);
  const int idx = combo->findText(previous);
  if (idx >= 0) {
    combo->setCurrentIndex(idx);
  } else if (!previous.isEmpty()) {
    combo->setEditText(previous);
  } else if (!topics.isEmpty()) {
    combo->setCurrentIndex(0);
  }
  combo->blockSignals(false);
}

/// \brief 主线程应用异步刷新得到的话题列表
void MainWindow::apply_refreshed_topics(
    const QStringList &image_topics, const QStringList &info_topics) {
  if (launcher_panel_ == nullptr) {
    return;
  }
  const QString prev_single =
      combo_image_topic_ != nullptr ? combo_image_topic_->currentText() : QString();
  const QString prev_left =
      launcher_panel_->combo_left_image_topic() != nullptr
          ? launcher_panel_->combo_left_image_topic()->currentText()
          : QString();
  const QString prev_right =
      launcher_panel_->combo_right_image_topic() != nullptr
          ? launcher_panel_->combo_right_image_topic()->currentText()
          : QString();

  fill_topic_combo(combo_image_topic_, image_topics, prev_single);
  fill_topic_combo(launcher_panel_->combo_left_image_topic(), image_topics, prev_left);
  fill_topic_combo(launcher_panel_->combo_right_image_topic(), image_topics, prev_right);

  append_log(
      LogLevel::Info,
      QStringLiteral("› 刷新图像话题：%1 个").arg(image_topics.size()));

  if (combo_camera_info_topic_ != nullptr &&
      launcher_panel_->intrinsics_source_mode() == 1) {
    const QString prev_info = combo_camera_info_topic_->currentText();
    fill_topic_combo(combo_camera_info_topic_, info_topics, prev_info);
    append_log(
        LogLevel::Info,
        QStringLiteral("› 刷新 CameraInfo 话题：%1 个").arg(info_topics.size()));
    apply_intrinsics_source_subscription();
  }

  sync_pending_ros_topics();
  if (needs_ros_image_subscription()) {
    sync_ros_image_subscription();
  }
  refresh_setup_readiness();

  if (btn_refresh_topics_ != nullptr) {
    btn_refresh_topics_->setEnabled(true);
    btn_refresh_topics_->setText(QStringLiteral("刷新"));
  }
  topic_refresh_busy_.store(false);
}

/// \brief 异步刷新 ROS 图像 / CameraInfo 话题下拉
void MainWindow::refresh_topic_list() {
  if (ros_bridge_ == nullptr || topic_refresh_busy_.exchange(true)) {
    return;
  }
  if (btn_refresh_topics_ != nullptr) {
    btn_refresh_topics_->setEnabled(false);
    btn_refresh_topics_->setText(QStringLiteral("刷新中…"));
  }

  RosImageBridge *bridge = ros_bridge_.get();
  std::thread([this, bridge]() {
    QStringList images;
    QStringList infos;
    if (bridge != nullptr) {
      images = bridge->list_image_topics();
      infos = bridge->list_camera_info_topics();
    }
    QMetaObject::invokeMethod(
        this,
        [this, images, infos]() { apply_refreshed_topics(images, infos); },
        Qt::QueuedConnection);
  }).detach();
}

/// \brief CameraInfo 话题变更（仅内参源=CameraInfo 时订阅）
void MainWindow::on_camera_info_topic_changed(const QString &topic) {
  apply_intrinsics_source_subscription();
  if (launcher_panel_ != nullptr && launcher_panel_->intrinsics_source_mode() == 1 &&
      !topic.trimmed().isEmpty()) {
    append_log(LogLevel::Info, QStringLiteral("› 订阅 CameraInfo：%1").arg(topic));
  } else if (topic.trimmed().isEmpty()) {
    append_log(LogLevel::Info, QStringLiteral("› 未使用 CameraInfo 话题"));
  }
  sync_detect_intrinsics_from_sources();
}

/// \brief 内参源模式变更
void MainWindow::on_intrinsics_source_changed(int mode) {
  (void)mode;
  if (launcher_panel_ != nullptr) {
    launcher_panel_->refresh_intrinsics_source_rows();
  }
  apply_intrinsics_source_subscription();
  sync_detect_intrinsics_from_sources();
  refresh_setup_readiness();
}

/// \brief 按内参源模式订阅/退订 CameraInfo
void MainWindow::apply_intrinsics_source_subscription() {
  if (ros_bridge_ == nullptr) {
    return;
  }
  const int mode =
      launcher_panel_ != nullptr ? launcher_panel_->intrinsics_source_mode() : 0;
  if (mode != 1) {
    ros_bridge_->unsubscribe_camera_info();
    return;
  }
  const QString topic =
      combo_camera_info_topic_ != nullptr ? combo_camera_info_topic_->currentText().trimmed()
                                          : QString();
  if (topic.isEmpty()) {
    ros_bridge_->unsubscribe_camera_info();
    return;
  }
  ros_bridge_->subscribe_camera_info(topic);
}

/// \brief 从 CameraInfo / YAML 同步检测用内参（尊重内参源模式；调试台可直填话题）
void MainWindow::sync_detect_intrinsics_from_sources() {
  if (session_ == nullptr) {
    return;
  }

  const bool on_lab =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab);
  const QString lab_info =
      combo_lab_camera_info_ != nullptr
          ? combo_lab_camera_info_->currentText().trimmed()
          : QString();
  if (on_lab && !lab_info.isEmpty()) {
    if (ros_bridge_ != nullptr && ros_bridge_->has_camera_info()) {
      session_->set_detect_intrinsics(
          ros_bridge_->camera_matrix(), ros_bridge_->dist_coeffs(),
          ros_bridge_->distortion_model().toStdString(), 0.0);
    }
    return;
  }

  const int mode =
      launcher_panel_ != nullptr ? launcher_panel_->intrinsics_source_mode() : 0;

  if (mode == 1) {
    if (ros_bridge_ != nullptr && ros_bridge_->has_camera_info()) {
      session_->set_detect_intrinsics(
          ros_bridge_->camera_matrix(), ros_bridge_->dist_coeffs(),
          ros_bridge_->distortion_model().toStdString(), 0.0);
    }
    return;
  }

  if (mode == 2) {
    QString path;
    if (launcher_panel_ != nullptr && launcher_panel_->edit_intrinsics_yaml() != nullptr) {
      path = launcher_panel_->edit_intrinsics_yaml()->text().trimmed();
    }
    if (path.isEmpty() && edit_camera_yaml_ != nullptr) {
      path = edit_camera_yaml_->text().trimmed();
    }
    if (!path.isEmpty()) {
      session_->set_camera_yaml(path);
    }
    return;
  }

  // mode==0：内参标定等场景不注入先验内参；双目外参仍用左右 YAML
  if (session_->is_stereo_extrinsics() && launcher_panel_ != nullptr) {
    QString path;
    const auto &opts = session_->solve_options();
    const auto it = opts.find("stereo_side");
    const bool right =
        it != opts.end() &&
        (it->second == "right" || it->second == "RIGHT" || it->second == "R");
    if (right && launcher_panel_->edit_right_camera_yaml() != nullptr) {
      path = launcher_panel_->edit_right_camera_yaml()->text().trimmed();
    } else if (launcher_panel_->edit_left_camera_yaml() != nullptr) {
      path = launcher_panel_->edit_left_camera_yaml()->text().trimmed();
    }
    if (!path.isEmpty()) {
      session_->set_camera_yaml(path);
    }
  }
}

/// \brief 源模式变更处理
void MainWindow::on_source_mode_changed(int index) {
  if (combo_source_mode_ == nullptr || session_ == nullptr) {
    return;
  }
  const auto mode = static_cast<SourceMode>(
      combo_source_mode_->itemData(index).toInt());
  session_->set_source_mode(mode);
  online_mode_ = (mode == SourceMode::RosTopic);
  if (act_online_ != nullptr) {
    act_online_->blockSignals(true);
    act_online_->setChecked(online_mode_);
    act_online_->blockSignals(false);
  }
  if (act_offline_ != nullptr) {
    act_offline_->blockSignals(true);
    act_offline_->setChecked(!online_mode_);
    act_offline_->blockSignals(false);
  }
  refresh_online_indicator();
  if (status_mode_ != nullptr) {
    QString mode_text = QStringLiteral("离线");
    QString mode_obj = QStringLiteral("StatusBarModeOffline");
    if (mode == SourceMode::RosTopic) {
      mode_text = QStringLiteral("ROS 在线");
      mode_obj = QStringLiteral("StatusBarModeOnline");
    } else if (mode == SourceMode::RosBag) {
      mode_text = QStringLiteral("ROS Bag");
      mode_obj = QStringLiteral("StatusBarModeOffline");
    }
    status_mode_->setObjectName(mode_obj);
    status_mode_->setText(mode_text);
    status_mode_->style()->unpolish(status_mode_);
    status_mode_->style()->polish(status_mode_);
  }
  refresh_setup_source_ui();
  if (mode == SourceMode::RosTopic) {
    refresh_topic_list();
  } else {
    if (ros_bridge_) {
      ros_bridge_->unsubscribe();
      ros_bridge_->unsubscribe_camera_info();
    }
    if (ros_stereo_bridge_) {
      ros_stereo_bridge_->unsubscribe();
    }
    if (mode == SourceMode::RosBag) {
      refresh_bag_topic_list();
    }
  }
  refresh_setup_readiness();
  const int page = stack_ != nullptr ? stack_->currentIndex() : 0;
  update_status_bar(static_cast<PageId>(page));
}

/// \brief 浏览 rosbag2 目录并列出图像话题
void MainWindow::on_browse_bag() {
  if (launcher_panel_ == nullptr || launcher_panel_->edit_bag_path() == nullptr) {
    return;
  }
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择 rosbag2 目录（含 metadata.yaml）"),
      launcher_panel_->edit_bag_path()->text());
  if (dir.isEmpty()) {
    return;
  }
  launcher_panel_->edit_bag_path()->setText(dir);
  refresh_bag_topic_list();
  refresh_setup_readiness();
}

/// \brief 扫描 bag 内图像话题填入下拉（后台线程枚举 metadata）
void MainWindow::refresh_bag_topic_list() {
  if (launcher_panel_ == nullptr) {
    return;
  }
  const bool stereo_bag = launcher_panel_->uses_stereo_dual_topics();
  if (!stereo_bag && launcher_panel_->combo_bag_topic() == nullptr) {
    return;
  }
  const QString bag = launcher_panel_->edit_bag_path()
                          ? launcher_panel_->edit_bag_path()->text().trimmed()
                          : QString();
  if (bag.isEmpty() || bag_topic_refresh_busy_.exchange(true)) {
    return;
  }
  if (launcher_panel_->btn_load_bag() != nullptr) {
    launcher_panel_->btn_load_bag()->setEnabled(false);
  }

  std::thread([this, bag]() {
    QString err;
    const auto topics = BagImageLoader::list_image_topics(bag, &err);
    QMetaObject::invokeMethod(
        this,
        [this, topics, err]() { apply_refreshed_bag_topics(topics, err); },
        Qt::QueuedConnection);
  }).detach();
}

/// \brief 主线程应用异步扫描得到的 bag 话题列表
void MainWindow::apply_refreshed_bag_topics(
    const QList<BagTopicInfo> &topics, const QString &err) {
  if (launcher_panel_ == nullptr) {
    bag_topic_refresh_busy_.store(false);
    return;
  }
  const bool stereo_bag =
      launcher_panel_->uses_stereo_dual_topics() &&
      launcher_panel_->combo_source_mode() != nullptr &&
      launcher_panel_->combo_source_mode()->currentData().toInt() ==
          static_cast<int>(SourceMode::RosBag);

  auto fill_combo = [&](QComboBox *combo, const QString &prev) {
    if (combo == nullptr) {
      return;
    }
    combo->blockSignals(true);
    combo->clear();
    for (const auto &t : topics) {
      combo->addItem(t.name);
    }
    const int idx = combo->findText(prev);
    if (idx >= 0) {
      combo->setCurrentIndex(idx);
    } else if (!topics.isEmpty()) {
      combo->setCurrentIndex(0);
    } else if (!prev.isEmpty()) {
      combo->setEditText(prev);
    }
    combo->blockSignals(false);
  };

  if (stereo_bag) {
    const QString prev_l =
        launcher_panel_->combo_left_image_topic() != nullptr
            ? launcher_panel_->combo_left_image_topic()->currentText()
            : QString();
    const QString prev_r =
        launcher_panel_->combo_right_image_topic() != nullptr
            ? launcher_panel_->combo_right_image_topic()->currentText()
            : QString();
    fill_combo(launcher_panel_->combo_left_image_topic(), prev_l);
    fill_combo(launcher_panel_->combo_right_image_topic(), prev_r);
  } else if (launcher_panel_->combo_bag_topic() != nullptr) {
    fill_combo(
        launcher_panel_->combo_bag_topic(),
        launcher_panel_->combo_bag_topic()->currentText());
  }
  if (!err.isEmpty()) {
    append_log(LogLevel::Warn, QStringLiteral("› Bag 话题：%1").arg(err));
  } else {
    append_log(
        LogLevel::Info,
        QStringLiteral("› Bag 图像话题：%1 个").arg(topics.size()));
  }
  refresh_setup_readiness();
  if (launcher_panel_->btn_load_bag() != nullptr) {
    launcher_panel_->btn_load_bag()->setEnabled(true);
  }
  bag_topic_refresh_busy_.store(false);
}

/// \brief 从 bag 直接解码图像帧并载入会话（后台解码）
void MainWindow::on_load_bag() {
  if (session_ == nullptr || launcher_panel_ == nullptr) {
    return;
  }
  const QString bag = launcher_panel_->edit_bag_path()
                          ? launcher_panel_->edit_bag_path()->text().trimmed()
                          : QString();
  const bool stereo_bag = launcher_panel_->uses_stereo_dual_topics();
  QString topic;
  QString left_topic;
  QString right_topic;
  if (stereo_bag) {
    left_topic =
        launcher_panel_->combo_left_image_topic() != nullptr
            ? launcher_panel_->combo_left_image_topic()->currentText().trimmed()
            : QString();
    right_topic =
        launcher_panel_->combo_right_image_topic() != nullptr
            ? launcher_panel_->combo_right_image_topic()->currentText().trimmed()
            : QString();
    if (bag.isEmpty() || left_topic.isEmpty() || right_topic.isEmpty()) {
      append_log(
          LogLevel::Warn,
          QStringLiteral("› 请先选择 Bag 路径与左/右目图像话题"));
      return;
    }
  } else {
    topic = launcher_panel_->combo_bag_topic()
                ? launcher_panel_->combo_bag_topic()->currentText().trimmed()
                : QString();
    if (bag.isEmpty() || topic.isEmpty()) {
      append_log(LogLevel::Warn, QStringLiteral("› 请先选择 Bag 路径与图像话题"));
      return;
    }
  }
  if (bag_load_busy_.exchange(true)) {
    return;
  }
  if (combo_source_mode_ != nullptr) {
    const int idx =
        combo_source_mode_->findData(static_cast<int>(SourceMode::RosBag));
    if (idx >= 0) {
      combo_source_mode_->setCurrentIndex(idx);
    }
  }
  if (launcher_panel_->btn_load_bag() != nullptr) {
    launcher_panel_->btn_load_bag()->setEnabled(false);
    launcher_panel_->btn_load_bag()->setText(QStringLiteral("解码中…"));
  }
  append_log(LogLevel::Info, QStringLiteral("› 正在后台解码 Bag…"));

  const int max_frames = launcher_panel_->bag_max_frames();
  if (stereo_bag) {
    std::thread([this, bag, left_topic, right_topic, max_frames]() {
      RosBagStereoFrameReader reader;
      QString err;
      const int loaded = reader.open(bag, left_topic, right_topic, max_frames, 30, &err);
      QMetaObject::invokeMethod(
          this,
          [this, loaded, err, left_topic, right_topic,
           reader = std::move(reader)]() mutable {
            apply_stereo_bag_load_result(
                loaded, err, left_topic, right_topic, std::move(reader));
          },
          Qt::QueuedConnection);
    }).detach();
    return;
  }

  std::thread([this, bag, topic, max_frames]() {
    RosBagFrameReader reader;
    QString err;
    const int loaded = reader.open(bag, topic, max_frames, &err);
    QMetaObject::invokeMethod(
        this,
        [this, loaded, err, topic, reader = std::move(reader)]() mutable {
          apply_bag_load_result(loaded, err, topic, std::move(reader));
        },
        Qt::QueuedConnection);
  }).detach();
}

void MainWindow::apply_bag_load_result(
    int loaded, const QString &err, const QString &topic, RosBagFrameReader reader) {
  bag_load_busy_.store(false);
  if (launcher_panel_ != nullptr && launcher_panel_->btn_load_bag() != nullptr) {
    launcher_panel_->btn_load_bag()->setEnabled(true);
    launcher_panel_->btn_load_bag()->setText(QStringLiteral("加载帧"));
  }
  if (session_ == nullptr) {
    return;
  }
  if (loaded <= 0) {
    append_log(
        LogLevel::Warn,
        QStringLiteral("› Bag 加载失败：%1").arg(err.isEmpty() ? QStringLiteral("无帧") : err));
    refresh_setup_readiness();
    return;
  }
  const int n = session_->apply_loaded_bag(std::move(reader), topic);
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已从 Bag 解码 %1 帧：%2").arg(n).arg(topic));
  refresh_setup_readiness();
}

void MainWindow::apply_stereo_bag_load_result(
    int loaded,
    const QString &err,
    const QString &left_topic,
    const QString &right_topic,
    RosBagStereoFrameReader reader) {
  bag_load_busy_.store(false);
  if (launcher_panel_ != nullptr && launcher_panel_->btn_load_bag() != nullptr) {
    launcher_panel_->btn_load_bag()->setEnabled(true);
    launcher_panel_->btn_load_bag()->setText(QStringLiteral("加载帧"));
  }
  if (session_ == nullptr) {
    return;
  }
  if (loaded <= 0) {
    append_log(
        LogLevel::Warn,
        QStringLiteral("› Bag 立体加载失败：%1")
            .arg(err.isEmpty() ? QStringLiteral("无配对帧") : err));
    refresh_setup_readiness();
    return;
  }
  const int n = session_->apply_loaded_stereo_bag(
      std::move(reader), left_topic, right_topic);
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已从 Bag 解码 %1 组成对帧：L=%2 · R=%3")
          .arg(n)
          .arg(left_topic, right_topic));
  refresh_setup_readiness();
}

/// \brief 图像话题变更并重新订阅
void MainWindow::on_topic_changed(const QString &topic) {
  if (ros_bridge_ == nullptr || session_ == nullptr) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  session_->set_ros_topic_name(topic);
  ros_bridge_->subscribe(topic);
  append_log(
      topic.isEmpty() ? LogLevel::Warn : LogLevel::Info,
      topic.isEmpty() ? QStringLiteral("› 已清空图像话题订阅")
                      : QStringLiteral("› 订阅图像话题：%1").arg(topic));
  // 内参源与图像源独立：不再因图像话题自动订阅 camera_info
  refresh_setup_readiness();
}

void MainWindow::on_stereo_ros_frames() {
  if (session_ == nullptr || ros_stereo_bridge_ == nullptr) {
    return;
  }
  if (!session_->uses_stereo_dual_session() ||
      session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  const int idx = stack_ != nullptr ? stack_->currentIndex() : -1;
  const bool on_workbench = idx == static_cast<int>(PageId::Workbench);
  if (!on_workbench) {
    return;
  }
  session_->set_live_stereo_bgr(
      ros_stereo_bridge_->take_latest_left_bgr(),
      ros_stereo_bridge_->take_latest_right_bgr(),
      ros_stereo_bridge_->last_match_delta_ms());
  if (!preview_live_) {
    return;
  }
  if (stereo_sync_label_ != nullptr) {
    const int64_t dt = session_->last_stereo_sync_delta_ms();
    if (dt < 0) {
      stereo_sync_label_->setText(QStringLiteral("Δt —"));
    } else {
      const bool ok = dt <= 30;
      stereo_sync_label_->setText(
          QStringLiteral("Δt %1ms %2")
              .arg(dt)
              .arg(ok ? QStringLiteral("✓") : QStringLiteral("!")));
    }
  }
  session_->schedule_stereo_live_preview_update();
  run_stereo_live_preview_tick(true);
}

void MainWindow::apply_stereo_raw_previews() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->stereo_left_has_detection() && session_->stereo_right_has_detection()) {
    if (stereo_preview_left_ != nullptr) {
      const QImage detect_l = session_->last_stereo_left_preview();
      if (!detect_l.isNull()) {
        stereo_preview_left_->set_image(detect_l);
      }
    }
    if (stereo_preview_right_ != nullptr) {
      const QImage detect_r = session_->last_stereo_right_preview();
      if (!detect_r.isNull()) {
        stereo_preview_right_->set_image(detect_r);
      }
    }
    return;
  }
  QImage detect_l = session_->last_stereo_left_preview();
  QImage detect_r = session_->last_stereo_right_preview();
  if (stereo_preview_left_ != nullptr) {
    if (!detect_l.isNull()) {
      stereo_preview_left_->set_image(detect_l);
    } else {
      const QImage l = session_->cached_stereo_live_preview_left();
      if (!l.isNull()) {
        stereo_preview_left_->set_image(l);
      }
    }
  }
  if (stereo_preview_right_ != nullptr) {
    if (!detect_r.isNull()) {
      stereo_preview_right_->set_image(detect_r);
    } else {
      const QImage r = session_->cached_stereo_live_preview_right();
      if (!r.isNull()) {
        stereo_preview_right_->set_image(r);
      }
    }
  }
}

void MainWindow::run_stereo_live_preview_tick(bool allow_auto_capture) {
  if (session_ == nullptr || !preview_live_) {
    return;
  }
  allow_auto_on_detect_finish_ = allow_auto_capture;
  if (session_->detect_busy() || session_->stereo_detect_busy()) {
    return;
  }
  sync_detect_intrinsics_from_sources();
  session_->request_stereo_detect(true);
}

void MainWindow::on_capture_paired_observation() {
  if (session_ == nullptr) {
    return;
  }
  sync_session_from_setup_ui();
  if (session_->stereo_detect_busy()) {
    append_log(LogLevel::Warn, QStringLiteral("› 双目检测进行中…"));
    return;
  }
  if (!session_->stereo_left_has_detection() ||
      !session_->stereo_right_has_detection()) {
    session_->request_stereo_detect(false);
    pending_capture_after_detect_ = true;
    append_log(LogLevel::Info, QStringLiteral("› 正在检测左右目，完成后自动采集成对"));
    return;
  }
  QString err;
  if (!session_->capture_paired_observation(&err)) {
    append_log(LogLevel::Error, QStringLiteral("› 成对采集失败：%1").arg(err));
    return;
  }
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已采集成对 #%1（L=%2 R=%3 · Δt=%4ms）")
          .arg(session_->stereo_pair_count())
          .arg(session_->stereo_left_sample_count())
          .arg(session_->stereo_right_sample_count())
          .arg(session_->last_stereo_sync_delta_ms()));
  refresh_workbench_view(false);
  refresh_intrinsics_workbench_ui();
}

/// \brief 收到 ROS 帧：工作台实时检测，或检测台刷新预览
void MainWindow::on_ros_frame() {
  if (session_ == nullptr || ros_bridge_ == nullptr) {
    return;
  }
  if (session_->uses_stereo_dual_session() &&
      session_->source_mode() == SourceMode::RosTopic) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  const int idx = stack_ != nullptr ? stack_->currentIndex() : -1;
  const bool on_workbench = idx == static_cast<int>(PageId::Workbench);
  const bool on_lab = idx == static_cast<int>(PageId::DetectLab);
  if (!on_workbench && !on_lab) {
    return;
  }

  // 检测台：实时刷新预览；局部/完整模式静默 Fast 叠加，冻结则停住画面
  if (on_lab) {
    if (!preview_live_) {
      return;
    }
    session_->set_live_bgr(ros_bridge_->take_latest_bgr());
    run_lab_live_preview_tick();
    return;
  }

  // 冻结模式：不刷新 live 帧，画面与检测叠加保持不动
  if (!preview_live_) {
    return;
  }
  session_->set_live_bgr(ros_bridge_->take_latest_bgr());
  run_live_preview_tick(true);
}

/// \brief 检测调试台：实时刷新；局部/完整静默 Fast 检测，识别模式只刷裸图
void MainWindow::run_lab_live_preview_tick() {
  if (session_ == nullptr || !preview_live_) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  if (lab_path_label_ != nullptr) {
    lab_path_label_->setText(
        QStringLiteral("源：图像话题 · %1").arg(session_->current_path()));
  }

  // 类型识别较重：实时只刷图像，等用户点「识别类型」
  if (is_detect_lab_identify_mode()) {
    if (session_->detect_busy()) {
      return;
    }
    if (lab_preview_ != nullptr) {
      const QImage img = session_->load_current_qimage();
      if (!img.isNull()) {
        lab_preview_->set_image(img);
      }
    }
    return;
  }

  if (session_->last_preview().isNull() && lab_preview_ != nullptr) {
    const QImage img = session_->load_current_qimage();
    if (!img.isNull()) {
      lab_preview_->set_image(img);
    }
  }
  if (session_->detect_busy()) {
    return;
  }
  apply_lab_params_to_session();
  sync_detect_intrinsics_from_sources();
  // 实时静默 Fast，不写「检测成功」日志
  session_->request_detect(true);
}

/// \brief 实时预览节流检测与可选自动采集
void MainWindow::run_live_preview_tick(bool allow_auto_capture) {
  if (session_ == nullptr || !preview_live_) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  allow_auto_on_detect_finish_ = allow_auto_capture;
  session_->schedule_live_preview_update();
  if (session_->detect_busy()) {
    return;
  }
  sync_detect_intrinsics_from_sources();
  session_->request_detect(true);
}

/// \brief 异步检测开始：更新忙状态 UI
void MainWindow::on_async_detect_started() {
  update_detect_status_ui();
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测中…"));
    btn_detect_->setEnabled(false);
    btn_detect_->setFixedSize(108, 40);
  }
  if (stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab)) {
    if (btn_lab_detect_ != nullptr) {
      btn_lab_detect_->setEnabled(false);
    }
    if (lab_stats_ != nullptr && lab_pending_log_) {
      lab_stats_->setText(QStringLiteral("检测中…"));
    }
  }
}

/// \brief 异步检测结束：刷新预览/日志/自动采集
void MainWindow::on_async_detect_finished(bool ok, const QString &err) {
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测"));
    btn_detect_->setEnabled(true);
    btn_detect_->setFixedSize(108, 40);
  }
  update_detect_status_ui();

  const bool on_lab =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab);

  // —— 刷新叠加预览 ——
  // 实时/冻结都显示带叠加的预览（否则实时几乎看不到渲染）
  QImage img = session_ != nullptr ? session_->last_preview() : QImage();
  if (img.isNull() && session_ != nullptr &&
      session_->source_mode() != SourceMode::RosTopic) {
    img = session_->cached_offline_preview_qimage();
  }
  if (on_lab) {
    refresh_detect_lab_view(true);
    if (lab_pending_log_) {
      lab_pending_log_ = false;
      if (ok) {
        append_log(
            LogLevel::Info,
            QStringLiteral("› 检测台成功：pts=%1 conf=%2% faces=%3")
                .arg(session_->last_point_count())
                .arg(qRound(session_->last_confidence() * 100.0))
                .arg(session_->last_faces_found()));
      } else {
        append_log(LogLevel::Error, QStringLiteral("› 检测台失败：%1").arg(err));
        if (lab_stats_ != nullptr) {
          lab_stats_->setText(QStringLiteral("失败：%1").arg(err));
        }
      }
    }
    return;
  }

  if (!img.isNull()) {
    show_preview_image(img);
  }

  if (session_ != nullptr && session_->uses_stereo_dual_session()) {
    refresh_intrinsics_workbench_ui();
    if (stereo_preview_left_ != nullptr && session_->stereo_left_has_detection()) {
      stereo_preview_left_->set_image(session_->last_stereo_left_preview());
    }
    if (stereo_preview_right_ != nullptr && session_->stereo_right_has_detection()) {
      stereo_preview_right_->set_image(session_->last_stereo_right_preview());
    }
  } else {
    refresh_intrinsics_workbench_ui();
  }

  // 仅手动「检测」写日志；实时后台检测静默，避免冻结后日志刷屏
  if (pending_detect_log_) {
    pending_detect_log_ = false;
    if (ok) {
      append_log(
          LogLevel::Info,
          QStringLiteral("› 检测成功：%1  conf=%2%")
              .arg(session_->current_path())
              .arg(qRound(session_->last_confidence() * 100.0)));
    } else {
      append_log(LogLevel::Error, QStringLiteral("› 检测失败：%1").arg(err));
    }
  }

  if (pending_capture_after_detect_) {
    pending_capture_after_detect_ = false;
    if (ok) {
      if (session_ != nullptr && session_->uses_stereo_dual_session()) {
        on_capture_paired_observation();
      } else {
        on_capture_observation();
      }
    }
  }

  // 自动采集：必须仍处于实时预览，且勾选自动采集
  if (allow_auto_on_detect_finish_ && preview_live_ && ok && session_ != nullptr) {
    const bool auto_on =
        chk_auto_capture_ != nullptr && chk_auto_capture_->isChecked();
    if (auto_on) {
      const qint64 now = QDateTime::currentMSecsSinceEpoch();
      if (now - last_auto_capture_ms_ >= session_->auto_cooldown_ms()) {
        QString cap_err;
        const bool captured =
            session_->uses_stereo_dual_session()
                ? session_->try_auto_capture_paired(
                      session_->min_confidence(), session_->min_diversity(), &cap_err)
                : session_->try_auto_capture(
                      session_->min_confidence(), session_->min_diversity(), &cap_err);
        if (captured) {
          last_auto_capture_ms_ = now;
          append_log(
              LogLevel::Info,
              session_->uses_stereo_dual_session()
                  ? QStringLiteral("› 自动成对采集 #%1 · Δt=%2ms")
                        .arg(session_->stereo_pair_count())
                        .arg(session_->last_stereo_sync_delta_ms())
                  : QStringLiteral("› 自动采集 #%1  conf=%2%")
                        .arg(session_->observation_count())
                        .arg(qRound(session_->last_confidence() * 100.0)));
          refresh_workbench_view(false);
        }
      }
    }
  }

  if (metric_coverage_ != nullptr && session_ != nullptr) {
    set_metric_value(
        metric_coverage_,
        session_->observation_count() >= 8 ? QStringLiteral("较充分")
                                           : QStringLiteral("继续采集"));
  }
}

/// \brief 更新检测状态与指标文案
void MainWindow::update_detect_status_ui() {
  if (session_ == nullptr) {
    return;
  }
  const bool busy = session_->detect_busy();
  if (metric_detect_ != nullptr) {
    if (busy) {
      set_metric_value(metric_detect_, QStringLiteral("检测中…"));
    } else if (session_->has_current_detection()) {
      QString text =
          QStringLiteral("%1%").arg(qRound(session_->last_confidence() * 100.0));
      if (session_->is_trihedral() && session_->last_faces_found() > 0) {
        text += QStringLiteral(" · %1面").arg(session_->last_faces_found());
      }
      set_metric_value(metric_detect_, text);
    } else {
      set_metric_value(metric_detect_, QStringLiteral("—"));
    }
  }
  if (metric_reproj_ != nullptr) {
    if (busy) {
      set_metric_value(metric_reproj_, QStringLiteral("…"));
    } else if (session_->last_aruco_reproj_px() >= 0.0) {
      set_metric_value(
          metric_reproj_,
          QStringLiteral("%1 px").arg(session_->last_aruco_reproj_px(), 0, 'f', 3));
    } else {
      set_metric_value(metric_reproj_, QStringLiteral("—"));
    }
  }
  update_preview_mode_ui();
  if (stack_ != nullptr) {
    update_status_bar(static_cast<PageId>(stack_->currentIndex()));
  }
}

/// \brief 切换实时/冻结预览
void MainWindow::set_preview_live(bool live) {
  const bool ros_mode =
      session_ != nullptr && session_->source_mode() == SourceMode::RosTopic;
  if (!ros_mode) {
    preview_live_ = false;
    update_preview_mode_ui();
    return;
  }
  if (preview_live_ == live) {
    update_preview_mode_ui();
    return;
  }
  preview_live_ = live;
  update_preview_mode_ui();
  if (stack_ != nullptr) {
    update_status_bar(static_cast<PageId>(stack_->currentIndex()));
  }
  if (live) {
    append_log(LogLevel::Info, QStringLiteral("› 预览模式：实时"));
    if (ros_bridge_ != nullptr && session_ != nullptr) {
      session_->set_live_bgr(ros_bridge_->take_latest_bgr());
      if (stack_ != nullptr &&
          stack_->currentIndex() == static_cast<int>(PageId::DetectLab)) {
        run_lab_live_preview_tick();
      } else {
        run_live_preview_tick(false);
      }
    }
  } else {
    // 冻结：立刻停掉实时异步检测链（排队 + 进行中任务）
    allow_auto_on_detect_finish_ = false;
    pending_detect_log_ = false;
    pending_capture_after_detect_ = false;
    if (session_ != nullptr) {
      session_->cancel_pending_detect();
    }
    update_detect_status_ui();
    append_log(
        LogLevel::Info,
        QStringLiteral("› 预览模式：冻结（画面已停住，可细看或手动检测）"));
  }
}

/// \brief 刷新实时/冻结按钮与提示
void MainWindow::update_preview_mode_ui() {
  const bool ros_mode =
      session_ != nullptr && session_->source_mode() == SourceMode::RosTopic;
  const bool live = ros_mode && preview_live_;

  if (btn_preview_live_ != nullptr) {
    btn_preview_live_->blockSignals(true);
    btn_preview_freeze_->blockSignals(true);
    btn_preview_live_->setEnabled(ros_mode);
    btn_preview_freeze_->setEnabled(ros_mode);
    btn_preview_live_->setChecked(live);
    btn_preview_freeze_->setChecked(ros_mode && !preview_live_);
    btn_preview_live_->blockSignals(false);
    btn_preview_freeze_->blockSignals(false);
  }

  if (btn_lab_preview_live_ != nullptr) {
    btn_lab_preview_live_->blockSignals(true);
    btn_lab_preview_freeze_->blockSignals(true);
    btn_lab_preview_live_->setEnabled(ros_mode);
    btn_lab_preview_freeze_->setEnabled(ros_mode);
    btn_lab_preview_live_->setChecked(live);
    btn_lab_preview_freeze_->setChecked(ros_mode && !preview_live_);
    btn_lab_preview_live_->blockSignals(false);
    btn_lab_preview_freeze_->blockSignals(false);
  }

  if (preview_title_label_ != nullptr) {
    if (!ros_mode) {
      preview_title_label_->setText(QStringLiteral("检测预览"));
    } else if (preview_live_) {
      preview_title_label_->setText(QStringLiteral("实时预览"));
    } else {
      preview_title_label_->setText(QStringLiteral("冻结画面"));
    }
  }

  if (lab_preview_title_label_ != nullptr) {
    if (!ros_mode) {
      lab_preview_title_label_->setText(QStringLiteral("检测预览"));
    } else if (preview_live_) {
      lab_preview_title_label_->setText(QStringLiteral("实时预览"));
    } else {
      lab_preview_title_label_->setText(QStringLiteral("冻结画面"));
    }
  }

  if (workbench_path_label_ != nullptr && ros_mode && session_ != nullptr) {
    const QString path = session_->current_path();
    QString extra;
    if (session_->detect_busy()) {
      extra = QStringLiteral("  ·  检测中…");
    } else if (session_->has_current_detection()) {
      extra = QStringLiteral("  ·  conf=%1%").arg(
          qRound(session_->last_confidence() * 100.0));
      if (session_->is_trihedral() && session_->last_faces_found() > 0) {
        extra += QStringLiteral(" · %1面").arg(session_->last_faces_found());
      }
    }
    if (!session_->has_live_frame() && preview_live_) {
      set_workbench_path_text(QStringLiteral("图像话题 · %1（等待帧）").arg(path));
    } else if (preview_live_) {
      set_workbench_path_text(QStringLiteral("图像话题 · %1%2").arg(path, extra));
    } else {
      set_workbench_path_text(
          QStringLiteral("冻结画面 · %1%2  （空格切换）").arg(path, extra));
    }
  }

  if (chk_auto_capture_ != nullptr) {
    chk_auto_capture_->setEnabled(ros_mode && preview_live_);
  }
}

/// \brief 在线/离线切换工作台按钮显隐（右侧动作区宽度不随左侧文案变化）
void MainWindow::update_workbench_mode_actions() {
  const bool ros_mode =
      (session_ != nullptr && session_->source_mode() == SourceMode::RosTopic) ||
      (combo_source_mode_ != nullptr &&
       combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic));
  if (btn_prev_ != nullptr) {
    btn_prev_->setVisible(!ros_mode);
  }
  if (btn_next_ != nullptr) {
    btn_next_->setVisible(!ros_mode);
  }
  if (chk_auto_capture_ != nullptr) {
    chk_auto_capture_->setVisible(ros_mode);
    if (!ros_mode) {
      chk_auto_capture_->setChecked(false);
    }
  }
}

/// \brief 设置工作台路径文案（按左侧容器宽度中间省略，右侧按钮位置不变）
void MainWindow::set_workbench_path_text(const QString &text) {
  workbench_path_full_ = text;
  if (workbench_path_label_ == nullptr) {
    return;
  }
  workbench_path_label_->setToolTip(text);

  int avail = 120;
  if (workbench_header_left_ != nullptr && workbench_header_left_->width() > 0) {
    avail = std::max(40, workbench_header_left_->width() - 8);
  } else if (workbench_path_label_->width() > 0) {
    avail = std::max(40, workbench_path_label_->width());
  }
  workbench_path_label_->setMaximumWidth(avail);
  workbench_path_label_->setText(
      workbench_path_label_->fontMetrics().elidedText(text, Qt::ElideMiddle, avail));
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event) {
  if (event->type() == QEvent::MouseButtonRelease) {
    const auto *me = static_cast<QMouseEvent *>(event);
    if (me->button() == Qt::LeftButton) {
      const int step_count = uses_stereo_rectify_flow() ? 6 : 5;
      for (int i = 0; i < step_count; ++i) {
        if (watched == step_labels_[i]) {
          go_to(page_id_for_step_index(i));
          return true;
        }
      }
    }
  }
  if (event->type() == QEvent::Resize) {
    if (watched == workbench_header_left_ || watched == workbench_path_label_) {
      if (!workbench_path_full_.isEmpty()) {
        set_workbench_path_text(workbench_path_full_);
      }
    }
  }
  return QMainWindow::eventFilter(watched, event);
}

/// \brief 在预览控件显示图像
void MainWindow::show_preview_image(const QImage &img) {
  if (preview_view_ == nullptr) {
    return;
  }
  if (img.isNull()) {
    preview_view_->clear_image();
    const bool ros =
        session_ != nullptr && session_->source_mode() == SourceMode::RosTopic;
    preview_view_->set_placeholder(
        ros ? QStringLiteral("等待 ROS 图像…") : QStringLiteral("无预览"));
    return;
  }
  preview_view_->set_image(img);
}

void MainWindow::show_intrinsics_parameter_dialog(int kind) {
  if (session_ == nullptr) {
    return;
  }
  IntrinsicsParameterDialog **slot = nullptr;
  IntrinsicsParameterDialog::Kind k = IntrinsicsParameterDialog::Kind::Calibration;
  if (kind == 1) {
    slot = &intrinsics_collector_params_dlg_;
    k = IntrinsicsParameterDialog::Kind::Collector;
  } else if (kind == 2) {
    slot = &intrinsics_detector_params_dlg_;
    k = IntrinsicsParameterDialog::Kind::Detector;
  } else {
    slot = &intrinsics_calib_params_dlg_;
  }
  if (*slot == nullptr) {
    *slot = new IntrinsicsParameterDialog(k, this);
    (*slot)->bind_session(session_.get());
  }
  (*slot)->load_from_session();
  (*slot)->show();
  (*slot)->raise();
}

void MainWindow::show_intrinsics_stats_dialog() {
  if (session_ == nullptr) {
    return;
  }
  if (intrinsics_stats_dlg_ == nullptr) {
    intrinsics_stats_dlg_ = new IntrinsicsStatsDialog(this);
  }
  intrinsics_stats_dlg_->show();
  intrinsics_stats_dlg_->raise();
  intrinsics_stats_dlg_->refresh(session_.get(), stats_backend_);
}

void MainWindow::show_intrinsics_detection_details_dialog() {
  if (session_ == nullptr) {
    return;
  }
  if (intrinsics_detection_details_dlg_ == nullptr) {
    intrinsics_detection_details_dlg_ = new IntrinsicsDetectionDetailsDialog(this);
  }
  intrinsics_detection_details_dlg_->refresh(session_.get());
  intrinsics_detection_details_dlg_->show();
  intrinsics_detection_details_dlg_->raise();
}

void MainWindow::show_intrinsics_calibration_bars_dialog() {
  if (session_ == nullptr) {
    return;
  }
  if (intrinsics_calibration_bars_dlg_ == nullptr) {
    intrinsics_calibration_bars_dlg_ = new IntrinsicsCalibrationBarsDialog(this);
  }
  intrinsics_calibration_bars_dlg_->show();
  intrinsics_calibration_bars_dlg_->raise();
  intrinsics_calibration_bars_dlg_->refresh(session_.get(), stats_backend_);
}

void MainWindow::show_intrinsics_calibration_rms_dialog() {
  if (session_ == nullptr) {
    return;
  }
  if (intrinsics_calibration_rms_dlg_ == nullptr) {
    intrinsics_calibration_rms_dlg_ = new IntrinsicsCalibrationRmsDialog(this);
  }
  intrinsics_calibration_rms_dlg_->show();
  intrinsics_calibration_rms_dlg_->raise();
  intrinsics_calibration_rms_dlg_->refresh(session_.get(), stats_backend_);
}

void MainWindow::show_intrinsics_tier4_statistics_dialogs() {
  if (session_ == nullptr) {
    QMessageBox::warning(
        this, QStringLiteral("标定统计"), QStringLiteral("当前无活动会话。"));
    return;
  }
  if (!session_->is_intrinsics()) {
    QMessageBox::warning(
        this, QStringLiteral("标定统计"),
        QStringLiteral("仅内参标定任务支持 Tier4 统计图。"));
    return;
  }
  const auto &col = session_->uses_stereo_dual_session()
                        ? session_->intrinsics_state_for_side(QStringLiteral("left"))
                              .collector()
                        : session_->intrinsics_state().collector();
  const bool has_cal = session_->uses_stereo_dual_session()
      ? session_->intrinsics_state_for_side(QStringLiteral("left"))
                .has_calibrated_model() &&
            session_->intrinsics_state_for_side(QStringLiteral("right"))
                .has_calibrated_model()
      : session_->intrinsics_state().has_calibrated_model();
  if (session_->uses_stereo_dual_session()) {
    const int l = session_->intrinsics_state_for_side(QStringLiteral("left"))
                      .collector()
                      .training_count();
    const int r = session_->intrinsics_state_for_side(QStringLiteral("right"))
                      .collector()
                      .training_count();
    if (l < 3 || r < 3) {
      QMessageBox::warning(
          this, QStringLiteral("标定统计"),
          QStringLiteral("左右训练样本均不足（各需 ≥3 帧），无法生成采集统计图。"));
      return;
    }
  } else if (col.training_count() < 3) {
    QMessageBox::warning(
        this, QStringLiteral("标定统计"),
        QStringLiteral("训练样本不足（至少 3 帧），无法生成采集统计图。"));
    return;
  }
  if (!has_cal) {
    QMessageBox::warning(
        this, QStringLiteral("标定统计"),
        QStringLiteral(
            "尚未完成标定，无法显示标定结果统计图（柱状对比与 RMS 热力图）。"
            "请先在工作台执行标定；采集统计图仍可查看。"));
    show_intrinsics_stats_dialog();
    return;
  }
  show_intrinsics_stats_dialog();
  show_intrinsics_calibration_bars_dialog();
  show_intrinsics_calibration_rms_dialog();
}

void MainWindow::show_intrinsics_calibration_status_dialog() {
  if (session_ == nullptr) {
    return;
  }
  if (intrinsics_calibration_status_dlg_ == nullptr) {
    intrinsics_calibration_status_dlg_ = new IntrinsicsCalibrationStatusDialog(this);
  }
  intrinsics_calibration_status_dlg_->refresh(session_.get());
  intrinsics_calibration_status_dlg_->show();
  intrinsics_calibration_status_dlg_->raise();
}

void MainWindow::show_intrinsics_viz_options_dialog() {
  if (session_ == nullptr || !session_->uses_tier4_intrinsics()) {
    return;
  }
  if (intrinsics_viz_options_dlg_ == nullptr) {
    intrinsics_viz_options_dlg_ = new IntrinsicsVizOptionsDialog(this);
    connect(
        intrinsics_viz_options_dlg_, &IntrinsicsVizOptionsDialog::options_applied, this,
        [this]() { refresh_workbench_preview_viz(); });
  }
  intrinsics_viz_options_dlg_->bind_session(session_.get());
  intrinsics_viz_options_dlg_->show();
  intrinsics_viz_options_dlg_->raise();
}

/// \brief 内参任务：经典工作台 vs Tier4 右栏与观测 Tab 显隐
void MainWindow::update_workbench_layout_for_task() {
  const bool stereo_wb =
      session_ != nullptr && session_->is_stereo_intrinsics();
  if (mono_preview_host_ != nullptr) {
    mono_preview_host_->setVisible(!stereo_wb);
  }
  if (stereo_preview_host_ != nullptr) {
    stereo_preview_host_->setVisible(stereo_wb);
  }
  if (btn_capture_wb_ != nullptr && session_ != nullptr) {
    if (stereo_wb && session_->stereo_capture_mode() == QStringLiteral("paired")) {
      btn_capture_wb_->setText(QStringLiteral("采集成对"));
    } else {
      btn_capture_wb_->setText(QStringLiteral("采集帧"));
    }
  }
  if (preview_title_label_ != nullptr) {
    preview_title_label_->setText(
        stereo_wb ? QStringLiteral("双目实时预览") : QStringLiteral("实时预览"));
  }
  const bool intrinsics =
      session_ != nullptr &&
      (session_->calibrator_id() == QStringLiteral("cam_intrinsics") ||
       session_->calibrator_id() == QStringLiteral("stereo_intrinsics"));
  const bool tier4 = session_ != nullptr && session_->uses_tier4_intrinsics();
  if (workbench_default_right_ != nullptr) {
    workbench_default_right_->setVisible(!tier4);
  }
  if (intrinsics_control_rail_ != nullptr) {
    intrinsics_control_rail_->setVisible(tier4);
    if (tier4 && session_ != nullptr) {
      intrinsics_control_rail_->set_session(session_.get());
    }
  }
  if (intrinsics_metrics_strip_ != nullptr) {
    intrinsics_metrics_strip_->setVisible(tier4);
  }
  if (obs_tabs_ != nullptr) {
    obs_tabs_->setTabVisible(1, tier4);
    obs_tabs_->setTabText(0, tier4 ? QStringLiteral("训练") : QStringLiteral("观测"));
  }
  if (btn_solve_wb_ != nullptr) {
    btn_solve_wb_->setText(intrinsics ? QStringLiteral("标定")
                                      : QStringLiteral("求解"));
  }
  if (combo_intrinsics_view_mode_ != nullptr) {
    combo_intrinsics_view_mode_->setVisible(tier4 && !stereo_wb);
  }
  if (viz_tier4_row_ != nullptr) {
    viz_tier4_row_->setVisible(tier4 && !stereo_wb);
  }
  if (btn_tier4_viz_options_wb_ != nullptr) {
    btn_tier4_viz_options_wb_->setVisible(tier4);
  }
  if (viz_classic_row_ != nullptr) {
    viz_classic_row_->setVisible(!tier4);
  }
  if (tier4 && session_ != nullptr) {
    sync_workbench_viz_from_session();
  } else if (!tier4) {
    sync_workbench_viz_from_session();
  }
  if (intrinsics_sample_slider_ != nullptr) {
    intrinsics_sample_slider_->setVisible(tier4);
  }
  if (intrinsics_sample_slider_label_ != nullptr) {
    intrinsics_sample_slider_label_->setVisible(tier4);
  }
}

void MainWindow::refresh_intrinsics_workbench_ui() {
  if (session_ == nullptr || !session_->uses_tier4_intrinsics()) {
    return;
  }
  if (intrinsics_control_rail_ != nullptr) {
    intrinsics_control_rail_->refresh();
  }
  if (intrinsics_metrics_strip_ != nullptr) {
    intrinsics_metrics_strip_->refresh(session_.get());
  }
  if (intrinsics_detection_details_dlg_ != nullptr &&
      intrinsics_detection_details_dlg_->isVisible()) {
    intrinsics_detection_details_dlg_->refresh(session_.get());
  }
  if (intrinsics_calibration_status_dlg_ != nullptr &&
      intrinsics_calibration_status_dlg_->isVisible()) {
    intrinsics_calibration_status_dlg_->refresh(session_.get());
  }
}

/// \brief 刷新工作台路径、列表与预览
void MainWindow::refresh_workbench_view(bool update_preview) {
  if (session_ == nullptr) {
    return;
  }
  const bool ros_mode = session_->source_mode() == SourceMode::RosTopic;
  if (workbench_path_label_ != nullptr && !ros_mode) {
    const QString path = session_->current_path();
    const int total_frames = session_->uses_stereo_dual_session()
        ? std::min(session_->stereo_left_image_paths().size(),
                   session_->stereo_right_image_paths().size())
        : session_->image_paths().size();
    const int cur_idx = session_->uses_stereo_dual_session()
        ? session_->stereo_pair_index()
        : session_->current_index();
    set_workbench_path_text(
        path.isEmpty()
            ? QStringLiteral("无图片")
            : QStringLiteral("%1 / %2  ·  %3")
                  .arg(cur_idx + 1)
                  .arg(total_frames)
                  .arg(path));
  }
  update_preview_mode_ui();
  update_workbench_mode_actions();
  const QString layout_token =
      session_->calibrator_id() + QLatin1Char('|') +
      (session_->uses_tier4_intrinsics() ? QStringLiteral("t4") : QStringLiteral("cl")) +
      QLatin1Char('|') +
      (session_->uses_stereo_dual_session() ? QStringLiteral("st") : QStringLiteral("mo"));
  if (layout_token != workbench_layout_task_token_) {
    workbench_layout_task_token_ = layout_token;
    update_workbench_layout_for_task();
  }

  if (update_preview && preview_view_ != nullptr) {
    QImage img;
    if (session_->uses_tier4_intrinsics() && session_->intrinsics_browse_sample_index() >= 0) {
      session_->intrinsics_browse_preview(&img);
    }
    if (img.isNull()) {
      img = session_->last_preview();
    }
    if (img.isNull() && ros_mode) {
      img = session_->cached_live_preview_qimage();
    }
    if (img.isNull() && !ros_mode) {
      img = session_->cached_offline_preview_qimage();
    }
    if (img.isNull()) {
      if (ros_mode) {
        preview_view_->clear_image();
        preview_view_->set_placeholder(QStringLiteral("等待 ROS 图像…"));
      } else {
        preview_view_->set_placeholder(QStringLiteral("加载预览…"));
        session_->schedule_offline_preview_update();
      }
    } else {
      show_preview_image(img);
    }
  }

  const core::ObservationBatch train_obs = session_->training_observations();
  if (obs_list_ != nullptr) {
    obs_list_->clear();
    for (const auto &obs : train_obs.items) {
      QString path = QString::fromStdString(obs.source_path);
      QString side_tag;
      if (obs.frame_id == "left" || path.startsWith(QStringLiteral("left:"))) {
        side_tag = QStringLiteral("[L] ");
        if (path.startsWith(QStringLiteral("left:"))) {
          path = path.mid(5);
        }
      } else if (obs.frame_id == "right" ||
                 path.startsWith(QStringLiteral("right:"))) {
        side_tag = QStringLiteral("[R] ");
        if (path.startsWith(QStringLiteral("right:"))) {
          path = path.mid(6);
        }
      }
      QString line = side_tag + path;
      if (obs.has_base_gripper) {
        line += QStringLiteral("  [pose]");
      }
      obs_list_->addItem(line);
    }
  }
  if (obs_eval_list_ != nullptr && session_->uses_tier4_intrinsics()) {
    obs_eval_list_->clear();
    for (const auto &obs : session_->evaluation_batch().items) {
      obs_eval_list_->addItem(QString::fromStdString(obs.source_path));
    }
  }
  refresh_intrinsics_workbench_ui();

  if (intrinsics_sample_slider_ != nullptr && session_->uses_tier4_intrinsics()) {
    const int n = obs_tabs_ != nullptr && obs_tabs_->currentIndex() == 1
                      ? session_->evaluation_sample_count()
                      : session_->training_sample_count();
    intrinsics_sample_slider_->setEnabled(n > 0);
    intrinsics_sample_slider_->setMaximum(std::max(0, n - 1));
    if (intrinsics_sample_slider_->value() >= n) {
      intrinsics_sample_slider_->setValue(-1);
    }
  }

  int n_left = 0;
  int n_right = 0;
  if (session_->is_stereo_side_tagged()) {
    for (const auto &obs : train_obs.items) {
      if (obs.frame_id == "right" ||
          obs.source_path.rfind("right:", 0) == 0) {
        ++n_right;
      } else {
        ++n_left;
      }
    }
    set_metric_value(
        metric_frames_,
        QStringLiteral("L%1 / R%2").arg(n_left).arg(n_right));
  } else {
    set_metric_value(metric_frames_, QString::number(session_->observation_count()));
  }
  if (session_->detect_busy()) {
    set_metric_value(metric_detect_, QStringLiteral("检测中…"));
  } else if (session_->has_current_detection()) {
    QString text =
        QStringLiteral("%1%").arg(qRound(session_->last_confidence() * 100.0));
    if (session_->is_trihedral() && session_->last_faces_found() > 0) {
      text += QStringLiteral(" · %1面").arg(session_->last_faces_found());
    }
    set_metric_value(metric_detect_, text);
  } else {
    set_metric_value(metric_detect_, QStringLiteral("—"));
  }
  if (metric_reproj_ != nullptr) {
    if (session_->detect_busy()) {
      set_metric_value(metric_reproj_, QStringLiteral("…"));
    } else if (session_->last_aruco_reproj_px() >= 0.0) {
      set_metric_value(
          metric_reproj_,
          QStringLiteral("%1 px").arg(session_->last_aruco_reproj_px(), 0, 'f', 3));
    } else {
      set_metric_value(metric_reproj_, QStringLiteral("—"));
    }
  }
  if (session_->is_stereo_side_tagged()) {
    const bool enough =
        n_left >= 3 && n_right >= 3;
    set_metric_value(
        metric_coverage_,
        enough ? QStringLiteral("左右均≥3")
               : QStringLiteral("L≥3 且 R≥3"));
  } else {
    set_metric_value(
        metric_coverage_,
        session_->observation_count() >= 8 ? QStringLiteral("较充分")
                                           : QStringLiteral("继续采集"));
  }

  if (act_solve_ != nullptr) {
    act_solve_->setEnabled(session_->can_solve());
  }
  if (btn_solve_wb_ != nullptr) {
    btn_solve_wb_->setEnabled(session_->can_solve() && !session_->solve_busy());
  }
}

/// \brief 刷新复查页指标与文本
void MainWindow::refresh_review_view() {
  if (session_ == nullptr) {
    return;
  }
  const auto &r = session_->last_result();
  if (review_text_ != nullptr) {
    if (session_->is_handeye() || session_->is_stereo_extrinsics()) {
      review_text_->setPlainText(QString::fromStdString(core::format_extrinsics_text(
          r, session_->result_parent_frame().toStdString(),
          session_->result_child_frame().toStdString())));
    } else {
      review_text_->setPlainText(
          QString::fromStdString(core::format_intrinsics_text(r)));
    }
  }
  if (r.success) {
    if (session_->is_handeye() || session_->is_stereo_extrinsics()) {
      const bool stereo = session_->is_stereo_extrinsics();
      const double err = stereo
          ? (r.metrics.count("stereo_rms") ? r.metrics.at("stereo_rms") : 0.0)
          : (r.metrics.count("handeye_rmse") ? r.metrics.at("handeye_rmse") : 0.0);
      set_metric_value(
          review_rmse_,
          QString::number(err, 'f', stereo ? 4 : 3) +
              (stereo ? QStringLiteral(" px") : QStringLiteral(" deg")));
      set_metric_value(
          review_views_,
          QString::number(
              static_cast<int>(
                  r.metrics.count("num_pairs") ? r.metrics.at("num_pairs") : 0)));
      set_metric_value(
          review_size_,
          session_->result_parent_frame() + QStringLiteral("→") +
              session_->result_child_frame());
    } else if (session_->is_stereo_intrinsics()) {
      const double lrms = r.metrics.count("left_reprojection_rmse")
          ? r.metrics.at("left_reprojection_rmse")
          : -1.0;
      const double rrms = r.metrics.count("right_reprojection_rmse")
          ? r.metrics.at("right_reprojection_rmse")
          : -1.0;
      QString rms_txt;
      if (lrms >= 0.0) {
        rms_txt += QStringLiteral("L %1").arg(lrms, 0, 'f', 3);
      }
      if (rrms >= 0.0) {
        if (!rms_txt.isEmpty()) {
          rms_txt += QStringLiteral(" / ");
        }
        rms_txt += QStringLiteral("R %1").arg(rrms, 0, 'f', 3);
      }
      if (rms_txt.isEmpty()) {
        rms_txt = QStringLiteral("—");
      } else {
        rms_txt += QStringLiteral(" px");
      }
      set_metric_value(review_rmse_, rms_txt);
      set_metric_value(
          review_views_,
          QStringLiteral("L%1 / R%2")
              .arg(static_cast<int>(
                  r.metrics.count("num_views_left") ? r.metrics.at("num_views_left")
                                                    : 0))
              .arg(static_cast<int>(
                  r.metrics.count("num_views_right")
                      ? r.metrics.at("num_views_right")
                      : 0)));
      set_metric_value(review_size_, QStringLiteral("camera_left/right.yaml"));
    } else {
      set_metric_value(
          review_rmse_,
          QString::number(
              r.metrics.count("reprojection_rmse") ? r.metrics.at("reprojection_rmse")
                                                   : 0.0,
              'f', 4) +
              QStringLiteral(" px"));
      set_metric_value(
          review_views_,
          session_->uses_tier4_intrinsics()
              ? QStringLiteral("训%1 / 评%2")
                    .arg(session_->training_sample_count())
                    .arg(session_->evaluation_sample_count())
              : QString::number(static_cast<int>(
                    r.metrics.count("num_views") ? r.metrics.at("num_views") : 0)));
      const auto &m = r.intrinsics_meta;
      const QString w = m.count("image_width") ? QString::fromStdString(m.at("image_width"))
                                               : QStringLiteral("—");
      const QString h = m.count("image_height")
          ? QString::fromStdString(m.at("image_height"))
          : QStringLiteral("—");
      set_metric_value(review_size_, w + QStringLiteral("×") + h);
    }
  } else {
    set_metric_value(review_rmse_, QStringLiteral("—"));
    set_metric_value(review_views_, QStringLiteral("—"));
    set_metric_value(review_size_, QStringLiteral("—"));
  }

  // —— 观测列表 + 残差/覆盖诊断（内参类）——
  review_selected_view_ = -1;
  if (review_obs_list_ != nullptr) {
    review_obs_list_->blockSignals(true);
    review_obs_list_->clear();
    const auto &batch = session_->batch();
    for (int i = 0; i < static_cast<int>(batch.items.size()); ++i) {
      const auto &obs = batch.items[static_cast<size_t>(i)];
      QString path = QString::fromStdString(obs.source_path);
      QString side;
      if (obs.frame_id == "left" || path.startsWith(QStringLiteral("left:"))) {
        side = QStringLiteral("[L] ");
        if (path.startsWith(QStringLiteral("left:"))) {
          path = path.mid(5);
        }
      } else if (
          obs.frame_id == "right" || path.startsWith(QStringLiteral("right:"))) {
        side = QStringLiteral("[R] ");
        if (path.startsWith(QStringLiteral("right:"))) {
          path = path.mid(6);
        }
      }
      const QFileInfo fi(path);
      QString name = fi.fileName().isEmpty() ? QStringLiteral("view_%1").arg(i)
                                            : fi.fileName();
      int npts = 0;
      if (!obs.correspondences.empty()) {
        npts = static_cast<int>(obs.correspondences.front().image_points.rows());
      }
      auto *item = new QListWidgetItem(
          QStringLiteral("%1%2  ·  %3 pts").arg(side, name).arg(npts));
      item->setData(Qt::UserRole, i);
      review_obs_list_->addItem(item);
    }
    review_obs_list_->blockSignals(false);
  }

  const bool can_diag =
      r.success && !session_->is_handeye() && !session_->is_stereo_extrinsics();
  if (can_diag) {
    if (review_diag_label_ != nullptr) {
      review_diag_label_->setText(QStringLiteral("正在计算残差诊断…"));
    }
    schedule_review_diagnostics_async();
  } else {
    if (review_diag_label_ != nullptr) {
      if (!r.success) {
        review_diag_label_->setText(QStringLiteral("尚未求解或求解失败"));
      } else {
        review_diag_label_->setText(
            QStringLiteral("手眼/外参：残差图暂以观测列表与结果摘要为主"));
      }
    }
    if (review_residual_bars_ != nullptr) {
      review_residual_bars_->set_bars({});
      review_residual_bars_->set_highlight_view(-1);
    }
    if (review_coverage_map_ != nullptr) {
      review_coverage_map_->clear();
      review_coverage_map_->set_image_size(0, 0);
    }
  }
}

void MainWindow::schedule_review_diagnostics_async() {
  if (session_ == nullptr) {
    return;
  }
  const auto result = session_->last_result();
  if (!result.success) {
    return;
  }
  const core::ObservationBatch batch = session_->batch();
  const bool stereo_intr = session_->is_stereo_intrinsics();
  const uint64_t epoch = ++review_diag_epoch_;

  std::thread([this, result, batch, stereo_intr, epoch]() {
    core::ReviewDiagnostics diag;
    if (stereo_intr) {
      diag = core::compute_stereo_review_diagnostics(result, batch);
    } else {
      diag = core::compute_review_diagnostics(result, batch);
    }
    QMetaObject::invokeMethod(
        this,
        [this, diag, epoch]() {
          if (epoch != review_diag_epoch_) {
            return;
          }
          if (stack_ == nullptr ||
              stack_->currentIndex() != static_cast<int>(PageId::Review)) {
            return;
          }
          apply_review_diagnostics(diag);
        },
        Qt::QueuedConnection);
  }).detach();
}

void MainWindow::apply_review_diagnostics(const core::ReviewDiagnostics &diag) {
  if (review_diag_label_ != nullptr) {
    review_diag_label_->setText(
        diag.valid ? QString::fromStdString(diag.message)
                   : QString::fromStdString(
                         diag.message.empty() ? "未能计算重投影残差" : diag.message));
  }
  if (review_residual_bars_ != nullptr) {
    std::vector<ResidualBarWidget::Bar> bars;
    if (diag.valid) {
      bars.reserve(diag.views.size());
      for (const auto &v : diag.views) {
        ResidualBarWidget::Bar b;
        b.label = QString::fromStdString(v.label);
        b.rms_px = v.ok ? v.rms_px : 0.0;
        b.ok = v.ok;
        b.view_index = v.index;
        bars.push_back(b);
      }
    }
    review_residual_bars_->set_bars(bars);
    review_residual_bars_->set_highlight_view(review_selected_view_);
  }
  if (review_coverage_map_ != nullptr) {
    if (diag.valid) {
      review_coverage_map_->set_image_size(diag.image_width, diag.image_height);
      std::vector<CoverageMapWidget::Point> pts;
      pts.reserve(diag.points.size());
      for (const auto &p : diag.points) {
        CoverageMapWidget::Point q;
        q.u = p.u;
        q.v = p.v;
        q.err_px = p.err_px;
        q.view_index = p.view_index;
        pts.push_back(q);
      }
      review_coverage_map_->set_points(pts);
      review_coverage_map_->set_filter_view(review_selected_view_);
    } else {
      review_coverage_map_->clear();
      review_coverage_map_->set_image_size(0, 0);
    }
  }
}

void MainWindow::apply_review_view_filter() {
  const int view = review_selected_view_;
  if (review_residual_bars_ != nullptr) {
    review_residual_bars_->set_highlight_view(view);
  }
  if (review_coverage_map_ != nullptr) {
    review_coverage_map_->set_filter_view(view);
  }
}

void MainWindow::on_review_obs_clicked(QListWidgetItem *item) {
  if (review_obs_list_ == nullptr || item == nullptr) {
    return;
  }
  const int view = item->data(Qt::UserRole).toInt();
  if (review_selected_view_ == view) {
    review_selected_view_ = -1;
    review_obs_list_->blockSignals(true);
    review_obs_list_->clearSelection();
    review_obs_list_->setCurrentItem(nullptr);
    review_obs_list_->blockSignals(false);
  } else {
    review_selected_view_ = view;
  }
  apply_review_view_filter();
}

void MainWindow::on_review_bar_clicked(int view_index) {
  if (review_obs_list_ == nullptr || view_index < 0) {
    return;
  }
  if (review_selected_view_ == view_index) {
    review_selected_view_ = -1;
    review_obs_list_->blockSignals(true);
    review_obs_list_->clearSelection();
    review_obs_list_->setCurrentItem(nullptr);
    review_obs_list_->blockSignals(false);
    apply_review_view_filter();
    return;
  }
  review_selected_view_ = view_index;
  for (int i = 0; i < review_obs_list_->count(); ++i) {
    auto *item = review_obs_list_->item(i);
    if (item != nullptr && item->data(Qt::UserRole).toInt() == view_index) {
      review_obs_list_->blockSignals(true);
      review_obs_list_->setCurrentRow(i);
      review_obs_list_->blockSignals(false);
      break;
    }
  }
  apply_review_view_filter();
}

/// \brief 从 YAML 填充数据源/标定设置（话题、坐标系、靶标等）
bool MainWindow::apply_yaml_config_file(const QString &path, QString *error_out) {
  if (launcher_panel_ == nullptr) {
    if (error_out) {
      *error_out = QStringLiteral("配置面板未就绪");
    }
    return false;
  }
  if (path.trimmed().isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("配置路径为空");
    }
    return false;
  }
  std::map<std::string, std::string> kv;
  std::string err;
  if (!core::load_simple_yaml_map(path.toStdString(), &kv, &err)) {
    if (error_out) {
      *error_out = QString::fromStdString(err.empty() ? "读取失败" : err);
    }
    return false;
  }
  launcher_panel_->apply_config_map(kv);
  launcher_panel_->set_config_path(path);
  if (session_ != nullptr) {
    launcher_panel_->apply_to_session(session_.get());
  }
  return true;
}

/// \brief 按当前标定器加载包内默认 YAML
void MainWindow::load_calibrator_default_config() {
  if (launcher_panel_ == nullptr) {
    return;
  }
  const QString id = selected_calibrator_id_.isEmpty()
      ? launcher_panel_->calibrator_id()
      : selected_calibrator_id_;
  const std::string fname = core::default_config_filename(id.toStdString());
  const std::string path = core::resolve_package_config(fname);
  if (path.empty()) {
    return;
  }
  QString err;
  if (!apply_yaml_config_file(QString::fromStdString(path), &err)) {
    append_log(LogLevel::Warn, QStringLiteral("› 加载默认配置失败：%1").arg(err));
    return;
  }
  sync_pending_ros_topics();
  refresh_setup_source_ui();
  refresh_setup_readiness();
}

/// \brief 从包内默认配置填充控件
void MainWindow::apply_board_config_from_package() {
  load_calibrator_default_config();
}

/// \brief 重载当前标定类型对应的包内 YAML
void MainWindow::on_reload_default_board_config() {
  const QString id = selected_calibrator_id_.isEmpty() && launcher_panel_ != nullptr
      ? launcher_panel_->calibrator_id()
      : selected_calibrator_id_;
  const std::string fname = core::default_config_filename(id.toStdString());
  const std::string path = core::resolve_package_config(fname);
  if (path.empty()) {
    append_log(
        LogLevel::Error,
        QStringLiteral("› 未找到包内 config/%1").arg(QString::fromStdString(fname)));
    return;
  }
  QString err;
  if (!apply_yaml_config_file(QString::fromStdString(path), &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 读取配置失败：%1").arg(err));
    return;
  }
  sync_pending_ros_topics();
  refresh_setup_source_ui();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已加载 %1（话题 / 坐标系 / 靶标）")
          .arg(QString::fromStdString(path)));
  refresh_setup_readiness();
}

/// \brief 合并短时间内的观测列表刷新，避免主线程反复重建列表
void MainWindow::schedule_workbench_view_refresh(bool update_preview) {
  if (workbench_refresh_timer_ == nullptr) {
    refresh_workbench_view(update_preview);
    return;
  }
  workbench_refresh_update_preview_ =
      workbench_refresh_update_preview_ || update_preview;
  workbench_refresh_timer_->start();
}

/// \brief 进入工作台后异步加载预览，避免阻塞页面切换
void MainWindow::schedule_workbench_preview_load() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->uses_stereo_dual_session()) {
    if (session_->source_mode() == SourceMode::RosTopic) {
      session_->schedule_stereo_live_preview_update();
    } else {
      session_->schedule_offline_preview_update();
    }
    return;
  }
  if (session_->source_mode() == SourceMode::RosTopic) {
    if (!session_->last_preview().isNull()) {
      show_preview_image(session_->last_preview());
      return;
    }
    session_->schedule_live_preview_update();
    return;
  }
  if (!session_->last_preview().isNull()) {
    show_preview_image(session_->last_preview());
    return;
  }
  const QImage cached = session_->cached_offline_preview_qimage();
  if (!cached.isNull()) {
    show_preview_image(cached);
    return;
  }
  session_->schedule_offline_preview_update();
}

/// \brief 浏览离线图片目录（后台扫描文件列表）
void MainWindow::on_browse_image_dir() {
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择棋盘图片目录"),
      edit_image_dir_ != nullptr ? edit_image_dir_->text() : QString());
  if (dir.isEmpty() || edit_image_dir_ == nullptr || session_ == nullptr) {
    return;
  }
  edit_image_dir_->setText(dir);
  if (image_dir_load_busy_.exchange(true)) {
    return;
  }
  append_log(LogLevel::Info, QStringLiteral("› 正在扫描图片目录…"));

  std::thread([this, dir]() {
    QStringList paths;
    QDir folder(dir);
    if (folder.exists()) {
      const QStringList filters = {
          QStringLiteral("*.png"),  QStringLiteral("*.jpg"),
          QStringLiteral("*.jpeg"), QStringLiteral("*.bmp"),
          QStringLiteral("*.PNG"),  QStringLiteral("*.JPG"),
          QStringLiteral("*.JPEG"), QStringLiteral("*.BMP"),
      };
      const QFileInfoList files =
          folder.entryInfoList(filters, QDir::Files, QDir::Name);
      paths.reserve(files.size());
      for (const QFileInfo &fi : files) {
        paths.push_back(fi.absoluteFilePath());
      }
    }
    QMetaObject::invokeMethod(
        this,
        [this, dir, paths]() { apply_image_dir_scan(dir, paths); },
        Qt::QueuedConnection);
  }).detach();
}

void MainWindow::apply_image_dir_scan(const QString &dir, const QStringList &paths) {
  image_dir_load_busy_.store(false);
  if (session_ == nullptr) {
    return;
  }
  int n = 0;
  if (session_->uses_stereo_dual_session()) {
    n = session_->load_stereo_image_dir(dir);
    append_log(
        n > 0 ? LogLevel::Info : LogLevel::Warn,
        QStringLiteral("› 双目目录：%1（%2 对）").arg(dir).arg(n));
  } else {
    n = session_->apply_image_paths(dir, paths);
    append_log(
        n > 0 ? LogLevel::Info : LogLevel::Warn,
        QStringLiteral("› 加载图片目录：%1（%2 张）").arg(dir).arg(n));
  }
  refresh_setup_readiness();
}

/// \brief 浏览可选内参 YAML（内参源）
void MainWindow::on_browse_intrinsics_yaml() {
  if (launcher_panel_ == nullptr || launcher_panel_->edit_intrinsics_yaml() == nullptr) {
    return;
  }
  auto *edit = launcher_panel_->edit_intrinsics_yaml();
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择相机内参 YAML"), edit->text(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty()) {
    return;
  }
  edit->setText(path);
  if (launcher_panel_->combo_intrinsics_source() != nullptr) {
    const int yidx = launcher_panel_->combo_intrinsics_source()->findData(2);
    if (yidx >= 0) {
      launcher_panel_->combo_intrinsics_source()->setCurrentIndex(yidx);
    }
  }
  if (edit_camera_yaml_ != nullptr && edit_camera_yaml_->text().trimmed().isEmpty()) {
    edit_camera_yaml_->setText(path);
  }
  if (session_) {
    session_->set_camera_yaml(path);
  }
  append_log(LogLevel::Info, QStringLiteral("› 内参源 YAML：%1").arg(path));
  sync_detect_intrinsics_from_sources();
  refresh_setup_readiness();
}

/// \brief 浏览相机内参 YAML
void MainWindow::on_browse_camera_yaml() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择相机内参 YAML"),
      edit_camera_yaml_ != nullptr ? edit_camera_yaml_->text() : QString(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty() || edit_camera_yaml_ == nullptr) {
    return;
  }
  edit_camera_yaml_->setText(path);
  if (session_) {
    session_->set_camera_yaml(path);
  }
  append_log(LogLevel::Info, QStringLiteral("› 内参 YAML：%1").arg(path));
  refresh_setup_readiness();
}

/// \brief 浏览左目内参 YAML（双目外参）
void MainWindow::on_browse_left_camera_yaml() {
  if (launcher_panel_ == nullptr || launcher_panel_->edit_left_camera_yaml() == nullptr) {
    return;
  }
  auto *edit = launcher_panel_->edit_left_camera_yaml();
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择左目内参 YAML"), edit->text(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty()) {
    return;
  }
  edit->setText(path);
  append_log(LogLevel::Info, QStringLiteral("› 左目内参 YAML：%1").arg(path));
  refresh_setup_readiness();
}

/// \brief 浏览右目内参 YAML（双目外参）
void MainWindow::on_browse_right_camera_yaml() {
  if (launcher_panel_ == nullptr || launcher_panel_->edit_right_camera_yaml() == nullptr) {
    return;
  }
  auto *edit = launcher_panel_->edit_right_camera_yaml();
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择右目内参 YAML"), edit->text(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty()) {
    return;
  }
  edit->setText(path);
  append_log(LogLevel::Info, QStringLiteral("› 右目内参 YAML：%1").arg(path));
  refresh_setup_readiness();
}

/// \brief 浏览并加载位姿 CSV
void MainWindow::on_browse_pose_csv() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择位姿 CSV"),
      edit_pose_csv_ != nullptr ? edit_pose_csv_->text() : QString(),
      QStringLiteral("CSV (*.csv)"));
  if (path.isEmpty() || session_ == nullptr) {
    return;
  }
  if (edit_pose_csv_ != nullptr) {
    edit_pose_csv_->setText(path);
  }
  QString err;
  if (!session_->load_pose_csv(path, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 位姿 CSV 失败：%1").arg(err));
  } else {
    append_log(
        LogLevel::Info,
        QStringLiteral("› 位姿 CSV：%1（%2 条）").arg(path).arg(session_->pose_csv_count()));
  }
  refresh_setup_readiness();
}

/// \brief 校验就绪后进入工作台会话
void MainWindow::on_start_session() {
  QString err;
  if (!ensure_implemented_calibrator(&err)) {
    append_log(LogLevel::Error, QStringLiteral("› %1").arg(err));
    return;
  }
  if (session_ == nullptr) {
    return;
  }
  apply_selected_project_to_setup();
  sync_session_from_setup_ui();

  // —— 按离线/在线/Bag 准备源数据 ——
  const bool ros_mode = session_->source_mode() == SourceMode::RosTopic;
  const bool stereo_ros =
      ros_mode && launcher_panel_ != nullptr && launcher_panel_->uses_stereo_dual_topics();
  if (ros_mode) {
    if (ros_bridge_ == nullptr || !ros_bridge_->is_ready()) {
      append_log(LogLevel::Error, QStringLiteral("› ROS 未就绪，无法开始在线会话"));
      return;
    }
    if (stereo_ros) {
      if (launcher_panel_->combo_left_image_topic() == nullptr ||
          launcher_panel_->combo_right_image_topic() == nullptr) {
        append_log(LogLevel::Warn, QStringLiteral("› 请先填写左/右目图像话题"));
        return;
      }
      const QString left =
          launcher_panel_->combo_left_image_topic()->currentText().trimmed();
      const QString right =
          launcher_panel_->combo_right_image_topic()->currentText().trimmed();
      if (left.isEmpty() || right.isEmpty()) {
        append_log(LogLevel::Warn, QStringLiteral("› 请先填写左/右目图像话题"));
        return;
      }
    } else {
      const QString topic =
          combo_image_topic_ != nullptr ? combo_image_topic_->currentText().trimmed()
                                        : QString();
      if (topic.isEmpty()) {
        append_log(LogLevel::Warn, QStringLiteral("› 请先选择图像话题"));
        return;
      }
    }
  } else if (session_->image_paths().isEmpty()) {
    if (session_->source_mode() == SourceMode::RosBag) {
      append_log(LogLevel::Warn, QStringLiteral("› 请先在数据源中加载 Bag 图像帧"));
    } else {
      append_log(LogLevel::Warn, QStringLiteral("› 请先选择含标定板图的目录"));
    }
    return;
  }
  if (session_->is_handeye() && !session_->has_camera_yaml()) {
    append_log(LogLevel::Warn, QStringLiteral("› 手眼需要相机内参 YAML"));
    return;
  }
  if (session_->is_stereo_extrinsics() && launcher_panel_ != nullptr) {
    const QString left =
        launcher_panel_->edit_left_camera_yaml()
            ? launcher_panel_->edit_left_camera_yaml()->text().trimmed()
            : QString();
    const QString right =
        launcher_panel_->edit_right_camera_yaml()
            ? launcher_panel_->edit_right_camera_yaml()->text().trimmed()
            : QString();
    if (left.isEmpty() || right.isEmpty()) {
      append_log(LogLevel::Warn, QStringLiteral("› 双目外参需要左右内参 YAML"));
      return;
    }
  }

  const int sx =
      launcher_panel_ != nullptr ? launcher_panel_->squares_x() : session_->squares_x();
  const int sy =
      launcher_panel_ != nullptr ? launcher_panel_->squares_y() : session_->squares_y();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 开始会话：%1，靶标 %2，棋盘 %3×%4 · 建议姿态 ≥%5")
          .arg(selected_calibrator_id_)
          .arg(
              launcher_panel_ != nullptr
                  ? launcher_panel_->target_type_id()
                  : QStringLiteral("—"))
          .arg(sx)
          .arg(sy)
          .arg(session_->min_views()));
  if (chk_auto_capture_ != nullptr && launcher_panel_ != nullptr) {
    chk_auto_capture_->setChecked(launcher_panel_->auto_capture_default());
  }
  preview_live_ = ros_mode;
  sync_workbench_viz_from_session();
  workbench_solve_fingerprint_.clear();
  go_to(PageId::Workbench);
  QTimer::singleShot(80, this, [this, ros_mode, stereo_ros]() {
    if (session_ == nullptr) {
      return;
    }
    if (ros_mode) {
      sync_detect_intrinsics_from_sources();
      if (stereo_ros) {
        session_->request_stereo_detect(true);
      } else {
        session_->request_detect(true);
      }
    } else {
      if (stereo_ros) {
        session_->request_stereo_detect(false);
      } else {
        on_detect_and_preview();
      }
      session_->request_offline_batch_ingest();
    }
  });
}

/// \brief 手动请求检测当前帧
void MainWindow::on_detect_and_preview() {
  if (session_ == nullptr || preview_view_ == nullptr) {
    return;
  }
  sync_detect_intrinsics_from_sources();
  pending_detect_log_ = true;
  update_detect_status_ui();
  if (session_->uses_stereo_dual_session()) {
    session_->request_stereo_detect(false);
    return;
  }
  session_->request_detect(false);
}

/// \brief 手动采集当前观测
void MainWindow::on_capture_observation() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->uses_stereo_dual_session() &&
      session_->stereo_capture_mode() == QStringLiteral("paired")) {
    on_capture_paired_observation();
    return;
  }
  // 同步 stereo_side 等面板选项，避免切换左右目后仍用旧侧标记
  sync_session_from_setup_ui();
  if (session_->detect_busy()) {
    pending_capture_after_detect_ = true;
    append_log(LogLevel::Warn, QStringLiteral("› 检测进行中，完成后自动采集当前帧"));
    return;
  }
  if (!session_->has_current_detection()) {
    pending_capture_after_detect_ = true;
    on_detect_and_preview();
    return;
  }
  QString err;
  if (!session_->add_current_observation(&err)) {
    append_log(LogLevel::Error, QStringLiteral("› 采集失败：%1").arg(err));
    return;
  }
  QString side_note;
  if (session_->is_stereo_side_tagged()) {
    const auto &opts = session_->solve_options();
    const auto it = opts.find("stereo_side");
    const QString side =
        (it != opts.end() && (it->second == "right" || it->second == "RIGHT" ||
                              it->second == "R"))
            ? QStringLiteral("右目")
            : QStringLiteral("左目");
    side_note = QStringLiteral(" · %1").arg(side);
  }
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已采集 %1 帧（conf=%2%%3）")
          .arg(session_->observation_count())
          .arg(qRound(session_->last_confidence() * 100.0))
          .arg(side_note));
  if (session_->source_mode() == SourceMode::RosTopic) {
    refresh_workbench_view(false);
    return;
  }
  if (session_->current_index() + 1 < session_->image_paths().size()) {
    session_->set_current_index(session_->current_index() + 1);
    on_detect_and_preview();
  } else {
    refresh_workbench_view(false);
  }
}

/// \brief 求解标定并跳转复查
void MainWindow::on_solve() {
  if (session_ == nullptr) {
    return;
  }
  if (session_->solve_busy()) {
    return;
  }
  if (session_->observation_count() < 3 && !session_->can_solve()) {
    append_log(LogLevel::Warn, QStringLiteral("› 观测不足，无法求解"));
    return;
  }
  allow_auto_on_detect_finish_ = false;
  pending_detect_log_ = false;
  pending_capture_after_detect_ = false;
  session_->cancel_pending_detect();
  update_detect_status_ui();
  sync_session_from_setup_ui();
  if (session_->observation_count() < session_->min_views()) {
    append_log(
        LogLevel::Warn,
        QStringLiteral("› 当前 %1 帧，少于建议姿态数 %2，仍继续求解…")
            .arg(session_->observation_count())
            .arg(session_->min_views()));
  }
  append_log(
      LogLevel::Info,
      QStringLiteral("› 开始求解（core::%1）…").arg(session_->calibrator_id()));
  session_->request_solve();
}

QString MainWindow::compute_workbench_solve_fingerprint() const {
  if (session_ == nullptr) {
    return {};
  }
  QString fp = session_->calibrator_id();
  fp += QLatin1Char('|');
  const auto opts = session_->solve_options();
  for (const auto &kv : opts) {
    fp += QString::fromStdString(kv.first);
    fp += QLatin1Char('=');
    fp += QString::fromStdString(kv.second);
    fp += QLatin1Char(';');
  }
  if (launcher_panel_ != nullptr) {
    fp += QStringLiteral("board=%1x%2@%3;")
              .arg(launcher_panel_->squares_x())
              .arg(launcher_panel_->squares_y())
              .arg(launcher_panel_->square_length(), 0, 'f', 6);
    fp += QStringLiteral("target=%1;").arg(launcher_panel_->target_type_id());
  }
  return QString::number(qHash(fp));
}

void MainWindow::maybe_clear_observations_on_workbench_enter() {
  if (session_ == nullptr) {
    return;
  }
  sync_session_from_setup_ui();
  const QString fp = compute_workbench_solve_fingerprint();
  if (!workbench_solve_fingerprint_.isEmpty() && fp != workbench_solve_fingerprint_) {
    if (session_->has_observations()) {
      session_->clear_observations();
      append_log(
          LogLevel::Info, QStringLiteral("› 标定参数已变更，已自动清空观测列表"));
    }
  }
  workbench_solve_fingerprint_ = fp;
}

void MainWindow::update_solve_action_enabled() {
  const bool on_work =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::Workbench);
  const bool can_solve =
      on_work && session_ != nullptr && !session_->solve_busy() &&
      session_->can_solve() && !session_->offline_ingest_busy();
  if (act_solve_ != nullptr) {
    act_solve_->setEnabled(can_solve);
  }
  if (btn_solve_wb_ != nullptr) {
    btn_solve_wb_->setEnabled(can_solve);
  }
}

void MainWindow::on_async_solve_started() {
  if (solve_progress_dlg_ == nullptr) {
    solve_progress_dlg_ = new QProgressDialog(this);
    solve_progress_dlg_->setWindowTitle(QStringLiteral("标定求解"));
    solve_progress_dlg_->setWindowModality(Qt::WindowModal);
    solve_progress_dlg_->setCancelButton(nullptr);
    solve_progress_dlg_->setMinimumDuration(0);
    solve_progress_dlg_->setAutoClose(false);
    solve_progress_dlg_->setAutoReset(false);
    solve_progress_dlg_->setMinimumWidth(360);
  }
  solve_progress_dlg_->setRange(0, 100);
  solve_progress_dlg_->setValue(0);
  solve_progress_dlg_->setLabelText(QStringLiteral("准备中…"));
  solve_progress_dlg_->show();
  update_solve_action_enabled();
  if (btn_solve_wb_ != nullptr) {
    btn_solve_wb_->setText(QStringLiteral("标定中…"));
  }
}

void MainWindow::on_async_solve_progress(int percent, const QString &message) {
  if (solve_progress_dlg_ == nullptr) {
    return;
  }
  solve_progress_dlg_->setValue(std::max(0, std::min(100, percent)));
  if (!message.isEmpty()) {
    solve_progress_dlg_->setLabelText(message);
  }
  QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void MainWindow::on_async_solve_finished(bool ok, const QString &err) {
  if (solve_progress_dlg_ != nullptr) {
    solve_progress_dlg_->setValue(100);
    solve_progress_dlg_->hide();
  }
  if (btn_solve_wb_ != nullptr) {
    const bool intrinsics = session_ != nullptr && session_->is_intrinsics();
    btn_solve_wb_->setText(intrinsics ? QStringLiteral("标定")
                                      : QStringLiteral("求解"));
  }
  update_solve_action_enabled();

  if (session_ == nullptr) {
    return;
  }
  if (!ok) {
    append_log(LogLevel::Error, QStringLiteral("› 求解失败：%1").arg(err));
    return;
  }
  if (session_->is_handeye()) {
    const double he = session_->last_result().metrics.count("handeye_rmse")
        ? session_->last_result().metrics.at("handeye_rmse")
        : 0.0;
    append_log(
        LogLevel::Info,
        QStringLiteral("› 手眼求解成功，rot_err≈%1 deg").arg(he, 0, 'f', 3));
  } else if (session_->is_stereo_extrinsics()) {
    const auto &m = session_->last_result().metrics;
    const double rms = m.count("stereo_rms") ? m.at("stereo_rms") : 0.0;
    const double base = m.count("baseline_m") ? m.at("baseline_m") : 0.0;
    append_log(
        LogLevel::Info,
        QStringLiteral("› 双目外参求解成功，RMS=%1 px · baseline=%2 m")
            .arg(rms, 0, 'f', 4)
            .arg(base, 0, 'f', 4));
  } else if (session_->is_stereo_intrinsics()) {
    const auto &m = session_->last_result().metrics;
    const double lrms =
        m.count("left_reprojection_rmse") ? m.at("left_reprojection_rmse") : -1.0;
    const double rrms =
        m.count("right_reprojection_rmse") ? m.at("right_reprojection_rmse") : -1.0;
    append_log(
        LogLevel::Info,
        QStringLiteral("› 双目各自内参求解成功，L RMSE=%1 px · R RMSE=%2 px")
            .arg(lrms >= 0.0 ? QString::number(lrms, 'f', 4) : QStringLiteral("—"))
            .arg(rrms >= 0.0 ? QString::number(rrms, 'f', 4)
                             : QStringLiteral("—")));
  } else {
    const double rms = session_->last_result().metrics.count("reprojection_rmse")
        ? session_->last_result().metrics.at("reprojection_rmse")
        : 0.0;
    append_log(
        LogLevel::Info,
        QStringLiteral("› 求解成功，RMSE = %1 px").arg(rms, 0, 'f', 4));
    if (session_->uses_tier4_intrinsics()) {
      const auto extras =
          core::calibration_extras_from_config(session_->solve_options());
      if (extras.plot_calibration_data_statistics) {
        show_intrinsics_stats_dialog();
      }
      if (extras.plot_calibration_results_statistics) {
        show_intrinsics_calibration_bars_dialog();
        show_intrinsics_calibration_rms_dialog();
      }
    }
  }
  go_to(session_->is_stereo_intrinsics() && session_->has_stereo_rectified()
            ? PageId::StereoRectify
            : PageId::Review);
}

/// \brief 导出结果包/YAML
void MainWindow::on_export_yaml() {
  if (session_ == nullptr || !session_->has_result()) {
    append_log(LogLevel::Warn, QStringLiteral("› 无结果可导出"));
    return;
  }

  QString hint;
  if (project_workspace_.is_open()) {
    hint = project_workspace_.suggest_export_dir(session_->calibrator_id());
  }
  if (hint.isEmpty() && launcher_panel_ != nullptr) {
    hint = launcher_panel_->export_dir_hint();
    if (hint.startsWith(QLatin1Char('~'))) {
      hint = QDir::homePath() + hint.mid(1);
    }
  }
  if (hint.isEmpty()) {
    hint = QDir::homePath();
  }
  QString suggested = hint;
  if (!project_workspace_.is_open() || !hint.contains(QStringLiteral("hs_calib_"))) {
    const QFileInfo hint_info(hint);
    QString parent = hint_info.isDir() || hint.endsWith(QLatin1Char('/'))
        ? hint
        : hint_info.absolutePath();
    if (parent.isEmpty() || parent == QStringLiteral(".")) {
      parent = QDir::homePath();
    }
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    suggested = QDir(parent).filePath(
        QStringLiteral("hs_calib_%1_%2").arg(session_->calibrator_id()).arg(stamp));
  }

  QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("导出标定结果文件夹"), suggested);
  if (path.isEmpty()) {
    return;
  }
  if (path.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive) ||
      path.endsWith(QStringLiteral(".yml"), Qt::CaseInsensitive)) {
    path = QFileInfo(path).completeBaseName().isEmpty()
        ? suggested
        : QDir(QFileInfo(path).absolutePath()).filePath(QFileInfo(path).completeBaseName());
  }

  QString err;
  if (!session_->export_bundle(path, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 导出失败：%1").arg(err));
    return;
  }
  if (!err.isEmpty()) {
    append_log(LogLevel::Warn, QStringLiteral("› %1").arg(err));
  }

  // 残差图 / 重投影热力图：先刷新复核诊断再离屏导出
  refresh_review_view();
  QStringList chart_notes;
  const QDir out_dir(path);
  if (review_residual_bars_ != nullptr && review_residual_bars_->has_data()) {
    const QString residual_png = out_dir.filePath(QStringLiteral("residual_rms.png"));
    if (review_residual_bars_->export_png(residual_png, QSize(1400, 520))) {
      chart_notes << QStringLiteral("residual_rms.png");
    }
  }
  if (review_coverage_map_ != nullptr && review_coverage_map_->has_data()) {
    const QString heat_png =
        out_dir.filePath(QStringLiteral("reprojection_heatmap.png"));
    if (review_coverage_map_->export_png(heat_png, QSize(1400, 1000))) {
      chart_notes << QStringLiteral("reprojection_heatmap.png");
    }
  }
  if (session_->is_intrinsics() && session_->uses_tier4_intrinsics()) {
    QString tier4_summary;
    append_log(LogLevel::Info, QStringLiteral("› 正在导出 Tier4 统计图…"));
    chart_notes.append(export_intrinsics_statistics_pngs(
        *session_, path, stats_backend_, &tier4_summary));
    if (!tier4_summary.isEmpty()) {
      append_log(LogLevel::Warn, QStringLiteral("› %1").arg(tier4_summary));
    }
  }

  if (!chart_notes.isEmpty()) {
    QFile cfg(out_dir.filePath(QStringLiteral("session_config.yaml")));
    if (cfg.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
      QTextStream ts(&cfg);
      ts << "\ncharts:\n";
      for (const QString &name : chart_notes) {
        if (name.contains(QStringLiteral("residual"))) {
          ts << "  residual_rms: " << name << "\n";
        } else if (name.contains(QStringLiteral("reprojection_heatmap"))) {
          ts << "  reprojection_heatmap: " << name << "\n";
        } else if (name == QStringLiteral("calibration_data_statistics.png")) {
          ts << "  calibration_data_statistics: " << name << "\n";
        } else if (name == QStringLiteral("calibration_result_vs_singleshot.png")) {
          ts << "  calibration_result_vs_singleshot: " << name << "\n";
        } else if (name == QStringLiteral("calibration_result_rms.png")) {
          ts << "  calibration_result_rms: " << name << "\n";
        }
      }
    }
    append_log(
        LogLevel::Info,
        QStringLiteral("› 已导出图表：%1").arg(chart_notes.join(QStringLiteral(", "))));
  } else if (
      session_->is_handeye() || session_->is_stereo_extrinsics()) {
    append_log(
        LogLevel::Info,
        QStringLiteral("› 手眼/外参任务暂无残差图与重投影热力图可导出"));
  }

  append_log(LogLevel::Info, QStringLiteral("› 已导出文件夹：%1").arg(path));

  // 若导出不在项目 results/ 内，再归档一份到当前项目
  if (project_workspace_.is_open()) {
    const QString results = project_workspace_.results_dir();
    if (!path.startsWith(results)) {
      QString archived;
      QString aerr;
      if (project_workspace_.archive_result_bundle(path, QFileInfo(path).fileName(), &archived, &aerr)) {
        append_log(
            LogLevel::Info,
            QStringLiteral("› 已归档到项目 results：%1").arg(archived));
      } else if (!aerr.isEmpty()) {
        append_log(LogLevel::Warn, QStringLiteral("› 项目归档失败：%1").arg(aerr));
      }
    }
  }
}

void MainWindow::refresh_stereo_rectify_view() {
  if (session_ == nullptr) {
    return;
  }
  if (!session_->has_stereo_rectified()) {
    session_->ensure_stereo_rectification();
  }
  if (session_->stereo_pair_index() < 0 && session_->stereo_rectify_pair_count() > 0) {
    session_->set_stereo_pair_index(0);
  }
  const int n = session_->stereo_rectify_pair_count();
  if (rectify_pair_slider_ != nullptr) {
    const QSignalBlocker blocker(rectify_pair_slider_);
    rectify_pair_slider_->setEnabled(n > 0);
    rectify_pair_slider_->setMaximum(std::max(0, n - 1));
    if (n > 0 && session_->stereo_pair_index() >= 0) {
      rectify_pair_slider_->setValue(session_->stereo_pair_index());
    }
  }
  if (rectify_pair_slider_label_ != nullptr) {
    const int idx = std::max(0, session_->stereo_pair_index());
    rectify_pair_slider_label_->setText(
        n > 0 ? QStringLiteral("图像对：%1 / %2").arg(idx + 1).arg(n)
              : QStringLiteral("图像对：—"));
  }
  if (rectify_path_label_ != nullptr) {
    const QString path = session_->current_path();
    rectify_path_label_->setText(
        path.isEmpty() ? QStringLiteral("无可用图像对")
                       : QStringLiteral("%1").arg(path));
  }

  const auto &r = session_->last_result();
  if (rectify_metric_baseline_ != nullptr) {
    const double base = r.metrics.count("baseline_m") ? r.metrics.at("baseline_m") : -1.0;
    set_metric_value(
        rectify_metric_baseline_,
        base >= 0.0 ? QStringLiteral("%1 m").arg(base, 0, 'f', 4) : QStringLiteral("—"));
  }
  if (rectify_metric_rms_ != nullptr) {
    const double rms = r.metrics.count("stereo_rms") ? r.metrics.at("stereo_rms") : -1.0;
    set_metric_value(
        rectify_metric_rms_,
        rms >= 0.0 ? QStringLiteral("%1 px").arg(rms, 0, 'f', 4) : QStringLiteral("—"));
  }
  if (rectify_metric_brightness_ != nullptr) {
    set_metric_value(rectify_metric_brightness_, session_->stereo_pair_brightness_hint());
  }

  QImage left;
  QImage right;
  const bool has_rect = session_->has_stereo_rectified();
  const bool has_preview =
      has_rect && session_->stereo_rectified_preview(&left, &right);

  if (rectify_hint_label_ != nullptr) {
    if (!has_rect) {
      const auto &note = session_->last_result().intrinsics_meta.find("stereo_rectify_note");
      if (note != session_->last_result().intrinsics_meta.end()) {
        rectify_hint_label_->setText(QString::fromStdString(note->second));
      } else {
        rectify_hint_label_->setText(
            QStringLiteral("尚未生成立体校正参数，请在工作台完成标定（需成对采集 ≥3 组）。"));
      }
    } else if (!has_preview) {
      rectify_hint_label_->setText(
          QStringLiteral(
              "立体校正参数已就绪；当前图像对无本地原图缓存，请在工作台重新成对采集几帧。"));
    } else {
      rectify_hint_label_->setText(
          QStringLiteral("水平极线应基本对齐；若偏差明显，请检查采集姿态或重新标定。"));
    }
  }

  if (has_preview) {
    if (rectify_preview_left_ != nullptr && !left.isNull()) {
      rectify_preview_left_->set_image(left);
    }
    if (rectify_preview_right_ != nullptr && !right.isNull()) {
      rectify_preview_right_->set_image(right);
    }
  } else {
    if (rectify_preview_left_ != nullptr) {
      rectify_preview_left_->clear_image();
      rectify_preview_left_->set_placeholder(QStringLiteral("无校正预览"));
    }
    if (rectify_preview_right_ != nullptr) {
      rectify_preview_right_->clear_image();
      rectify_preview_right_->set_placeholder(QStringLiteral("无校正预览"));
    }
  }
}

// ===== UI 构建 =====

}  // namespace gui
}  // namespace hs_calib
