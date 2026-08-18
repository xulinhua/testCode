#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/widgets/review_charts_widget.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/io/board_config_yaml.hpp"
#include "hs_calib_suite/core/review/review_diagnostics.hpp"

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
#include <QDir>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
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


/// \brief 把设置页/Launcher 参数同步进会话
void MainWindow::sync_session_from_setup_ui() {
  if (session_ == nullptr) {
    return;
  }
  if (launcher_panel_ != nullptr) {
    launcher_panel_->set_calibrator_id(selected_calibrator_id_);
    launcher_panel_->apply_to_session(session_.get());
    session_->set_viz_options(
        launcher_panel_->viz_draw_corners(), launcher_panel_->viz_draw_hull(),
        launcher_panel_->viz_show_conf(), launcher_panel_->viz_marker_radius(),
        launcher_panel_->viz_draw_aruco());
    return;
  }
  session_->set_calibrator_id(selected_calibrator_id_);
}

/// \brief 把会话可视化选项同步到工作台勾选
void MainWindow::sync_workbench_viz_from_session() {
  if (session_ == nullptr) {
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
  const bool corners =
      chk_viz_corners_wb_ != nullptr ? chk_viz_corners_wb_->isChecked() : true;
  const bool hull = chk_viz_hull_wb_ != nullptr ? chk_viz_hull_wb_->isChecked() : true;
  const bool conf = chk_viz_conf_wb_ != nullptr ? chk_viz_conf_wb_->isChecked() : true;
  const bool aruco = chk_viz_aruco_wb_ != nullptr ? chk_viz_aruco_wb_->isChecked() : true;
  const int radius =
      spin_viz_marker_wb_ != nullptr ? spin_viz_marker_wb_->value() : 4;
  session_->set_viz_options(corners, hull, conf, radius, aruco);
}

/// \brief 按是否手眼 / 双目外参显隐相关块
void MainWindow::refresh_handeye_ui() {
  const bool he =
      selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
      selected_calibrator_id_ == QStringLiteral("eye_to_hand");
  if (handeye_block_ != nullptr) {
    handeye_block_->setVisible(he);
  }
  if (launcher_panel_ != nullptr && launcher_panel_->stereo_extrinsics_block() != nullptr) {
    launcher_panel_->stereo_extrinsics_block()->setVisible(
        selected_calibrator_id_ == QStringLiteral("stereo_extrinsics"));
  }
}

/// \brief 刷新设置页就绪检查清单
void MainWindow::refresh_setup_readiness() {
  if (setup_ready_label_ == nullptr || session_ == nullptr) {
    return;
  }
  const bool he = session_->is_handeye() ||
                  selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
                  selected_calibrator_id_ == QStringLiteral("eye_to_hand");
  const bool ros_mode =
      (combo_source_mode_ != nullptr &&
       combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic)) ||
      session_->source_mode() == SourceMode::RosTopic;

  QStringList parts;
  bool can_start = true;
  if (ros_mode) {
    const bool ros_ok = ros_bridge_ && ros_bridge_->is_ready();
    const QString topic =
        combo_image_topic_ != nullptr ? combo_image_topic_->currentText() : QString();
    const bool topic_ok = !topic.isEmpty();
    const bool frame_ok = ros_bridge_ && ros_bridge_->has_frame();
    parts << (ros_ok ? QStringLiteral("ROS✓") : QStringLiteral("ROS○"));
    parts << (topic_ok ? QStringLiteral("话题✓") : QStringLiteral("话题○"));
    parts << (frame_ok ? QStringLiteral("有图✓") : QStringLiteral("有图○"));
    can_start = ros_ok && topic_ok;
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
        target = launcher_panel_->combo_target_type()->currentText();
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
    const QString cam =
        edit_camera_yaml_ != nullptr ? edit_camera_yaml_->text().trimmed() : QString();
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

  setup_ready_label_->setText(parts.join(QStringLiteral("  ·  ")));
  setup_ready_label_->setToolTip(parts.join(QStringLiteral("\n")));
  if (btn_start_session_ != nullptr) {
    btn_start_session_->setEnabled(can_start);
  }
}

/// \brief 按离线/ROS 切换源相关控件
void MainWindow::refresh_setup_source_ui() {
  if (launcher_panel_ != nullptr) {
    launcher_panel_->refresh_source_mode_rows();
  } else {
    const bool ros_mode =
        combo_source_mode_ != nullptr &&
        combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic);
    if (offline_row_ != nullptr) {
      offline_row_->setVisible(!ros_mode);
    }
    if (topic_row_ != nullptr) {
      topic_row_->setVisible(ros_mode);
    }
  }
  update_workbench_mode_actions();
}

/// \brief 刷新 ROS 图像 / CameraInfo 话题下拉
void MainWindow::refresh_topic_list() {
  if (ros_bridge_ == nullptr) {
    return;
  }
  if (combo_image_topic_ != nullptr) {
    const QString prev = combo_image_topic_->currentText();
    const QStringList topics = ros_bridge_->list_image_topics();
    combo_image_topic_->blockSignals(true);
    combo_image_topic_->clear();
    combo_image_topic_->addItems(topics);
    int idx = combo_image_topic_->findText(prev);
    if (idx >= 0) {
      combo_image_topic_->setCurrentIndex(idx);
    } else if (!topics.isEmpty()) {
      combo_image_topic_->setCurrentIndex(0);
    } else if (!prev.isEmpty()) {
      combo_image_topic_->setEditText(prev);
    }
    combo_image_topic_->blockSignals(false);
    append_log(LogLevel::Info, QStringLiteral("› 刷新图像话题：%1 个").arg(topics.size()));
    if (!combo_image_topic_->currentText().isEmpty()) {
      on_topic_changed(combo_image_topic_->currentText());
    }
  }

  if (combo_camera_info_topic_ != nullptr) {
    const QString prev_info = combo_camera_info_topic_->currentText();
    const QStringList info_topics = ros_bridge_->list_camera_info_topics();
    combo_camera_info_topic_->blockSignals(true);
    combo_camera_info_topic_->clear();
    combo_camera_info_topic_->addItems(info_topics);
    int idx = combo_camera_info_topic_->findText(prev_info);
    if (idx >= 0) {
      combo_camera_info_topic_->setCurrentIndex(idx);
    } else {
      // 常见约定：image_raw → camera_info
      QString guess;
      if (combo_image_topic_ != nullptr) {
        const QString img = combo_image_topic_->currentText();
        if (img.contains(QStringLiteral("image_raw"))) {
          guess = img;
          guess.replace(QStringLiteral("image_raw"), QStringLiteral("camera_info"));
        } else if (img.contains(QStringLiteral("image"))) {
          guess = img;
          guess.replace(QStringLiteral("/image"), QStringLiteral("/camera_info"));
        }
      }
      const int gidx = combo_camera_info_topic_->findText(guess);
      if (gidx >= 0) {
        combo_camera_info_topic_->setCurrentIndex(gidx);
      } else if (!info_topics.isEmpty()) {
        combo_camera_info_topic_->setCurrentIndex(0);
      } else if (!prev_info.isEmpty()) {
        combo_camera_info_topic_->setEditText(prev_info);
      } else if (!guess.isEmpty()) {
        combo_camera_info_topic_->setEditText(guess);
      }
    }
    combo_camera_info_topic_->blockSignals(false);
    append_log(
        LogLevel::Info,
        QStringLiteral("› 刷新 CameraInfo 话题：%1 个").arg(info_topics.size()));
    if (!combo_camera_info_topic_->currentText().isEmpty()) {
      on_camera_info_topic_changed(combo_camera_info_topic_->currentText());
    }
  }
  refresh_setup_readiness();
}

/// \brief CameraInfo 话题变更并订阅内参
void MainWindow::on_camera_info_topic_changed(const QString &topic) {
  if (ros_bridge_ == nullptr) {
    return;
  }
  ros_bridge_->subscribe_camera_info(topic);
  append_log(
      topic.isEmpty() ? LogLevel::Warn : LogLevel::Info,
      topic.isEmpty() ? QStringLiteral("› 已清空 CameraInfo 订阅")
                      : QStringLiteral("› 订阅 CameraInfo：%1").arg(topic));
  sync_detect_intrinsics_from_sources();
}

/// \brief 从 CameraInfo / camera_yaml 同步检测用内参
void MainWindow::sync_detect_intrinsics_from_sources() {
  if (session_ == nullptr) {
    return;
  }
  if (ros_bridge_ != nullptr && ros_bridge_->has_camera_info()) {
    session_->set_detect_intrinsics(
        ros_bridge_->camera_matrix(), ros_bridge_->dist_coeffs(),
        ros_bridge_->distortion_model().toStdString(), 0.0);
    return;
  }
  // 双目外参：按当前采集侧选用左右 YAML
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
      return;
    }
  }
  if (edit_camera_yaml_ != nullptr && !edit_camera_yaml_->text().trimmed().isEmpty()) {
    session_->set_camera_yaml(edit_camera_yaml_->text().trimmed());
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
    act_online_->setChecked(online_mode_);
    act_offline_->setChecked(!online_mode_);
  }
  if (status_mode_ != nullptr) {
    status_mode_->setObjectName(
        online_mode_ ? QStringLiteral("StatusBarModeOnline")
                     : QStringLiteral("StatusBarModeOffline"));
    status_mode_->setText(online_mode_ ? QStringLiteral("ROS 在线") : QStringLiteral("离线"));
    status_mode_->style()->unpolish(status_mode_);
    status_mode_->style()->polish(status_mode_);
  }
  refresh_setup_source_ui();
  if (mode == SourceMode::RosTopic) {
    refresh_topic_list();
  } else if (ros_bridge_) {
    ros_bridge_->unsubscribe();
    ros_bridge_->unsubscribe_camera_info();
  }
  refresh_setup_readiness();
  const int page = stack_ != nullptr ? stack_->currentIndex() : 0;
  update_status_bar(static_cast<PageId>(page));
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
  if (combo_camera_info_topic_ != nullptr && !topic.isEmpty()) {
    QString guess = topic;
    if (guess.contains(QStringLiteral("image_raw"))) {
      guess.replace(QStringLiteral("image_raw"), QStringLiteral("camera_info"));
      if (combo_camera_info_topic_->findText(guess) >= 0 ||
          combo_camera_info_topic_->currentText().isEmpty() ||
          combo_camera_info_topic_->currentText() == QStringLiteral("/camera/camera_info")) {
        combo_camera_info_topic_->setEditText(guess);
        on_camera_info_topic_changed(guess);
      }
    }
  }
  refresh_setup_readiness();
}

/// \brief 收到 ROS 帧：写入会话并触发实时预览
void MainWindow::on_ros_frame() {
  if (session_ == nullptr || ros_bridge_ == nullptr) {
    return;
  }
  if (session_->source_mode() != SourceMode::RosTopic) {
    return;
  }
  const bool on_workbench =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::Workbench);
  if (!on_workbench) {
    refresh_setup_readiness();
    return;
  }

  // 冻结模式：不刷新 live 帧，画面与检测叠加保持不动
  if (!preview_live_) {
    return;
  }
  session_->set_live_bgr(ros_bridge_->latest_bgr());
  run_live_preview_tick(true);
}

/// \brief 实时预览节流检测与可选自动采集
void MainWindow::run_live_preview_tick(bool allow_auto_capture) {
  if (session_ == nullptr) {
    return;
  }
  allow_auto_on_detect_finish_ = allow_auto_capture;
  // 有叠加预览时不要每帧用裸图盖掉；等检测完成再刷新
  if (session_->last_preview().isNull()) {
    show_preview_image(session_->load_current_qimage());
  }
  update_detect_status_ui();
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  // 实时检测节流：约 4–5 Hz；Fast 路径下足够流畅
  if (now - last_live_detect_ms_ < 200) {
    return;
  }
  if (session_->detect_busy()) {
    return;
  }
  last_live_detect_ms_ = now;
  sync_detect_intrinsics_from_sources();
  // 实时预览一律 Fast，避免 ArUco 多字典/2× 把 UI 卡在「检测中…」
  session_->request_detect(true);
}

/// \brief 异步检测开始：更新忙状态 UI
void MainWindow::on_async_detect_started() {
  update_detect_status_ui();
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测中…"));
  }
  if (stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab)) {
    if (btn_lab_detect_ != nullptr) {
      btn_lab_detect_->setEnabled(false);
    }
    if (btn_lab_detect_fast_ != nullptr) {
      btn_lab_detect_fast_->setEnabled(false);
    }
    if (lab_stats_ != nullptr) {
      lab_stats_->setText(QStringLiteral("检测中…"));
    }
  }
}

/// \brief 异步检测结束：刷新预览/日志/自动采集
void MainWindow::on_async_detect_finished(bool ok, const QString &err) {
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测"));
  }
  update_detect_status_ui();

  const bool on_lab =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab);

  // —— 刷新叠加预览 ——
  // 实时/冻结都显示带叠加的预览（否则实时几乎看不到渲染）
  QImage img = session_ != nullptr ? session_->last_preview() : QImage();
  if (img.isNull() && session_ != nullptr) {
    img = session_->load_current_qimage();
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
      on_capture_observation();
    }
  }

  if (allow_auto_on_detect_finish_ && preview_live_ && ok && session_ != nullptr) {
    const bool auto_on =
        chk_auto_capture_ != nullptr && chk_auto_capture_->isChecked();
    if (auto_on) {
      const qint64 now = QDateTime::currentMSecsSinceEpoch();
      if (now - last_auto_capture_ms_ >= session_->auto_cooldown_ms()) {
        QString cap_err;
        if (session_->try_auto_capture(
                session_->min_confidence(), session_->min_diversity(), &cap_err)) {
          last_auto_capture_ms_ = now;
          append_log(
              LogLevel::Info,
              QStringLiteral("› 自动采集 #%1  conf=%2%")
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
          QStringLiteral("%1 px").arg(session_->last_aruco_reproj_px(), 0, 'f', 1));
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
      session_->set_live_bgr(ros_bridge_->latest_bgr());
      run_live_preview_tick(false);
    }
  } else {
    append_log(
        LogLevel::Info,
        QStringLiteral("› 预览模式：冻结（画面与检测已停，可点检测/采集细看）"));
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

  if (preview_title_label_ != nullptr) {
    if (!ros_mode) {
      preview_title_label_->setText(QStringLiteral("检测预览"));
    } else if (preview_live_) {
      preview_title_label_->setText(QStringLiteral("实时预览"));
    } else {
      preview_title_label_->setText(QStringLiteral("冻结画面"));
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
      set_workbench_path_text(QStringLiteral("等待话题图像 · %1").arg(path));
    } else if (preview_live_) {
      set_workbench_path_text(QStringLiteral("实时 · %1%2").arg(path, extra));
    } else {
      set_workbench_path_text(
          QStringLiteral("冻结 · %1%2  （空格切换）").arg(path, extra));
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

/// \brief 设置工作台路径文案（中间省略，避免挤占右侧按钮）
void MainWindow::set_workbench_path_text(const QString &text) {
  if (workbench_path_label_ == nullptr) {
    return;
  }
  workbench_path_label_->setToolTip(text);
  int avail = workbench_path_label_->width();
  if (avail < 60) {
    if (auto *parent = workbench_path_label_->parentWidget()) {
      avail = std::max(60, parent->width() - 28);
    } else {
      avail = 360;
    }
  }
  workbench_path_label_->setText(
      workbench_path_label_->fontMetrics().elidedText(text, Qt::ElideMiddle, avail));
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

/// \brief 刷新工作台路径、列表与预览
void MainWindow::refresh_workbench_view(bool update_preview) {
  if (session_ == nullptr) {
    return;
  }
  const bool ros_mode = session_->source_mode() == SourceMode::RosTopic;
  if (workbench_path_label_ != nullptr && !ros_mode) {
    const QString path = session_->current_path();
    set_workbench_path_text(
        path.isEmpty()
            ? QStringLiteral("无图片")
            : QStringLiteral("%1 / %2  ·  %3")
                  .arg(session_->current_index() + 1)
                  .arg(session_->image_paths().size())
                  .arg(path));
  }
  update_preview_mode_ui();
  update_workbench_mode_actions();

  if (update_preview && preview_view_ != nullptr) {
    QImage img;
    // 优先显示最近一次检测叠加图（实时也不例外）
    img = session_->last_preview();
    if (img.isNull()) {
      img = session_->load_current_qimage();
    }
    if (img.isNull()) {
      preview_view_->clear_image();
      preview_view_->set_placeholder(
          ros_mode ? QStringLiteral("等待 ROS 图像…") : QStringLiteral("无预览"));
    } else {
      show_preview_image(img);
    }
  }

  if (obs_list_ != nullptr) {
    obs_list_->clear();
    for (const auto &obs : session_->batch().items) {
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

  int n_left = 0;
  int n_right = 0;
  if (session_->is_stereo_side_tagged()) {
    for (const auto &obs : session_->batch().items) {
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
          QStringLiteral("%1 px").arg(session_->last_aruco_reproj_px(), 0, 'f', 1));
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
    act_solve_->setEnabled(session_->observation_count() >= 3);
  }
  if (btn_solve_wb_ != nullptr) {
    btn_solve_wb_->setEnabled(
        session_->observation_count() >= session_->min_views());
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
          QString::number(static_cast<int>(
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

  core::ReviewDiagnostics diag;
  const bool can_diag =
      r.success && !session_->is_handeye() && !session_->is_stereo_extrinsics();
  if (can_diag) {
    if (session_->is_stereo_intrinsics()) {
      diag = core::compute_stereo_review_diagnostics(r, session_->batch());
    } else {
      diag = core::compute_review_diagnostics(r, session_->batch());
    }
  }

  if (review_diag_label_ != nullptr) {
    if (!r.success) {
      review_diag_label_->setText(QStringLiteral("尚未求解或求解失败"));
    } else if (!can_diag) {
      review_diag_label_->setText(
          QStringLiteral("手眼/外参：残差图暂以观测列表与结果摘要为主"));
    } else {
      review_diag_label_->setText(QString::fromStdString(diag.message));
    }
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
    review_residual_bars_->set_highlight_view(-1);
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
      review_coverage_map_->set_filter_view(-1);
    } else {
      review_coverage_map_->clear();
      review_coverage_map_->set_image_size(0, 0);
    }
  }
}

void MainWindow::on_review_obs_selected() {
  if (review_obs_list_ == nullptr) {
    return;
  }
  auto *item = review_obs_list_->currentItem();
  if (item == nullptr) {
    if (review_residual_bars_ != nullptr) {
      review_residual_bars_->set_highlight_view(-1);
    }
    if (review_coverage_map_ != nullptr) {
      review_coverage_map_->set_filter_view(-1);
    }
    return;
  }
  const int view = item->data(Qt::UserRole).toInt();
  if (review_residual_bars_ != nullptr) {
    review_residual_bars_->set_highlight_view(view);
  }
  if (review_coverage_map_ != nullptr) {
    review_coverage_map_->set_filter_view(view);
  }
}

void MainWindow::on_review_bar_clicked(int view_index) {
  if (review_obs_list_ == nullptr || view_index < 0) {
    return;
  }
  for (int i = 0; i < review_obs_list_->count(); ++i) {
    auto *item = review_obs_list_->item(i);
    if (item != nullptr && item->data(Qt::UserRole).toInt() == view_index) {
      review_obs_list_->setCurrentRow(i);
      break;
    }
  }
}

/// \brief 从包内默认板配置填充控件
void MainWindow::apply_board_config_from_package() {
  if (launcher_panel_ == nullptr) {
    return;
  }
  const std::string path = core::resolve_package_config("cam_intrinsics.yaml");
  if (path.empty()) {
    return;
  }
  core::BoardConfigYaml cfg;
  if (!core::load_board_config_yaml(path, &cfg) || !cfg.valid) {
    return;
  }
  launcher_panel_->set_board_params(cfg.squares_x, cfg.squares_y, cfg.square_length);
  launcher_panel_->set_config_path(QString::fromStdString(path));
  if (session_) {
    session_->set_board_params(cfg.squares_x, cfg.squares_y, cfg.square_length);
  }
}

/// \brief 重载默认板配置
void MainWindow::on_reload_default_board_config() {
  const std::string path = core::resolve_package_config("cam_intrinsics.yaml");
  if (path.empty()) {
    append_log(LogLevel::Error, QStringLiteral("› 未找到包内 config/cam_intrinsics.yaml"));
    return;
  }
  core::BoardConfigYaml cfg;
  if (!core::load_board_config_yaml(path, &cfg) || !cfg.valid) {
    append_log(
        LogLevel::Error,
        QStringLiteral("› 读取棋盘配置失败：%1")
            .arg(QString::fromStdString(cfg.message)));
    return;
  }
  apply_board_config_from_package();
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已加载 %1 → %2×%3，方格 %4 m")
          .arg(QString::fromStdString(path))
          .arg(cfg.squares_x)
          .arg(cfg.squares_y)
          .arg(cfg.square_length, 0, 'f', 4));
  refresh_setup_readiness();
}

/// \brief 浏览离线图片目录
void MainWindow::on_browse_image_dir() {
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择棋盘图片目录"),
      edit_image_dir_ != nullptr ? edit_image_dir_->text() : QString());
  if (dir.isEmpty() || edit_image_dir_ == nullptr || session_ == nullptr) {
    return;
  }
  edit_image_dir_->setText(dir);
  const int n = session_->load_image_dir(dir);
  append_log(
      n > 0 ? LogLevel::Info : LogLevel::Warn,
      QStringLiteral("› 加载图片目录：%1（%2 张）").arg(dir).arg(n));
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

  // —— 按离线/在线准备源数据 ——
  const bool ros_mode = session_->source_mode() == SourceMode::RosTopic;
  if (ros_mode) {
    if (ros_bridge_ == nullptr || !ros_bridge_->is_ready()) {
      append_log(LogLevel::Error, QStringLiteral("› ROS 未就绪，无法开始在线会话"));
      return;
    }
    const QString topic =
        combo_image_topic_ != nullptr ? combo_image_topic_->currentText() : QString();
    if (topic.isEmpty()) {
      append_log(LogLevel::Warn, QStringLiteral("› 请先选择图像话题"));
      return;
    }
    session_->set_ros_topic_name(topic);
    ros_bridge_->subscribe(topic);
  } else if (session_->image_paths().isEmpty()) {
    append_log(LogLevel::Warn, QStringLiteral("› 请先选择含棋盘图的目录"));
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
              launcher_panel_ != nullptr && launcher_panel_->combo_target_type() != nullptr
                  ? launcher_panel_->combo_target_type()->currentText()
                  : QStringLiteral("—"))
          .arg(sx)
          .arg(sy)
          .arg(session_->min_views()));
  if (chk_auto_capture_ != nullptr && launcher_panel_ != nullptr) {
    chk_auto_capture_->setChecked(launcher_panel_->auto_capture_default());
  }
  preview_live_ = ros_mode;
  sync_workbench_viz_from_session();
  go_to(PageId::Workbench);
  refresh_workbench_view();
  if (ros_mode) {
    if (session_->has_live_frame()) {
      show_preview_image(session_->load_current_qimage());
    }
    sync_detect_intrinsics_from_sources();
    session_->request_detect(true);
  } else {
    on_detect_and_preview();
  }
}

/// \brief 手动请求检测当前帧
void MainWindow::on_detect_and_preview() {
  if (session_ == nullptr || preview_view_ == nullptr) {
    return;
  }
  sync_detect_intrinsics_from_sources();
  pending_detect_log_ = true;
  update_detect_status_ui();
  session_->request_detect(false);
}

/// \brief 手动采集当前观测
void MainWindow::on_capture_observation() {
  if (session_ == nullptr) {
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
  sync_session_from_setup_ui();
  QString err;
  append_log(
      LogLevel::Info,
      QStringLiteral("› 开始求解（core::%1）…").arg(session_->calibrator_id()));
  if (!session_->solve(&err)) {
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
  }
  go_to(PageId::Review);
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

// ===== UI 构建 =====

}  // namespace gui
}  // namespace hs_calib
