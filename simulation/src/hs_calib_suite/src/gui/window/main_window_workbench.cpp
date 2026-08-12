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

/// \brief 按是否手眼显隐相关块
void MainWindow::refresh_handeye_ui() {
  const bool he =
      selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
      selected_calibrator_id_ == QStringLiteral("eye_to_hand");
  if (handeye_block_ != nullptr) {
    handeye_block_->setVisible(he);
  }
}

/// \brief 刷新设置页就绪检查清单
void MainWindow::refresh_setup_readiness() {
  if (setup_check_list_ == nullptr || session_ == nullptr) {
    return;
  }
  setup_check_list_->clear();
  const bool he = session_->is_handeye() ||
                  selected_calibrator_id_ == QStringLiteral("eye_in_hand") ||
                  selected_calibrator_id_ == QStringLiteral("eye_to_hand");
  const bool ros_mode =
      (combo_source_mode_ != nullptr &&
       combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic)) ||
      session_->source_mode() == SourceMode::RosTopic;

  // —— 按模式汇总就绪项 ——
  bool can_start = true;
  if (ros_mode) {
    const bool ros_ok = ros_bridge_ && ros_bridge_->is_ready();
    const QString topic =
        combo_image_topic_ != nullptr ? combo_image_topic_->currentText() : QString();
    const bool topic_ok = !topic.isEmpty();
    const bool frame_ok = ros_bridge_ && ros_bridge_->has_frame();
    setup_check_list_->addItem(
        (ros_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("ROS 节点就绪"));
    setup_check_list_->addItem(
        (topic_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("已选择图像话题"));
    setup_check_list_->addItem(
        (frame_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("话题已有图像数据"));
    can_start = ros_ok && topic_ok;
  } else {
    const QString dir =
        edit_image_dir_ != nullptr ? edit_image_dir_->text().trimmed() : QString();
    const int n = session_->image_paths().size();
    const bool dir_ok = !dir.isEmpty() && n > 0;
    setup_check_list_->addItem(
        (dir_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("图片目录可读（%1 张）").arg(n));
    setup_check_list_->addItem(
        (n >= 3 ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("建议至少 3 张可用于求解"));
    can_start = dir_ok;
  }
  {
    QString target = QStringLiteral("chessboard");
    QString model = QStringLiteral("pinhole");
    int min_views = 12;
    if (launcher_panel_ != nullptr) {
      if (launcher_panel_->combo_target_type() != nullptr) {
        target = launcher_panel_->combo_target_type()->currentText();
      }
      if (launcher_panel_->combo_camera_model() != nullptr) {
        model = launcher_panel_->combo_camera_model()->currentText();
      }
      min_views = launcher_panel_->min_views();
    }
    setup_check_list_->addItem(
        QStringLiteral("● 靶标：%1 · 模型：%2 · 建议姿态 ≥%3")
            .arg(target, model)
            .arg(min_views));
  }

  if (he) {
    const QString cam =
        edit_camera_yaml_ != nullptr ? edit_camera_yaml_->text().trimmed() : QString();
    const bool cam_ok = !cam.isEmpty();
    setup_check_list_->addItem(
        (cam_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
        QStringLiteral("已指定相机内参 YAML"));
    can_start = can_start && cam_ok;
    const int pose_mode =
        combo_pose_source_ != nullptr ? combo_pose_source_->currentData().toInt()
                                      : static_cast<int>(PoseSource::Csv);
    if (pose_mode == static_cast<int>(PoseSource::Csv)) {
      const bool csv_ok = session_->pose_csv_count() > 0;
      setup_check_list_->addItem(
          (csv_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
          QStringLiteral("位姿 CSV 已加载（%1）").arg(session_->pose_csv_count()));
      can_start = can_start && csv_ok;
    } else {
      const bool tf_ok = tf_bridge_ && tf_bridge_->is_ready();
      setup_check_list_->addItem(
          (tf_ok ? QStringLiteral("● ") : QStringLiteral("○ ")) +
          QStringLiteral("TF 桥就绪（采集时查询）"));
      can_start = can_start && tf_ok;
    }
  }

  if (btn_start_session_ != nullptr) {
    btn_start_session_->setEnabled(can_start);
  }
}

/// \brief 按离线/ROS 切换源相关控件
void MainWindow::refresh_setup_source_ui() {
  const bool ros_mode =
      combo_source_mode_ != nullptr &&
      combo_source_mode_->currentData().toInt() == static_cast<int>(SourceMode::RosTopic);
  if (offline_row_ != nullptr) {
    offline_row_->setVisible(!ros_mode);
  }
  if (topic_row_ != nullptr) {
    topic_row_->setVisible(ros_mode);
  }
  if (btn_prev_ != nullptr) {
    btn_prev_->setEnabled(!ros_mode);
    btn_next_->setEnabled(!ros_mode);
  }
}

/// \brief 刷新 ROS 图像话题下拉
void MainWindow::refresh_topic_list() {
  if (combo_image_topic_ == nullptr || ros_bridge_ == nullptr) {
    return;
  }
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
  }
  combo_image_topic_->blockSignals(false);
  append_log(LogLevel::Info, QStringLiteral("› 刷新图像话题：%1 个").arg(topics.size()));
  if (combo_image_topic_->currentIndex() >= 0) {
    on_topic_changed(combo_image_topic_->currentText());
  }
  refresh_setup_readiness();
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
  // 实时检测节流：约 5–8 Hz，避免一直 busy 且看不到叠加
  if (now - last_live_detect_ms_ < 150) {
    return;
  }
  if (session_->detect_busy()) {
    return;
  }
  last_live_detect_ms_ = now;
  session_->request_detect(session_->is_trihedral());
}

/// \brief 异步检测开始：更新忙状态 UI
void MainWindow::on_async_detect_started() {
  update_detect_status_ui();
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测中…"));
  }
}

/// \brief 异步检测结束：刷新预览/日志/自动采集
void MainWindow::on_async_detect_finished(bool ok, const QString &err) {
  if (btn_detect_ != nullptr) {
    btn_detect_->setText(QStringLiteral("检测"));
  }
  update_detect_status_ui();

  // —— 刷新叠加预览 ——
  // 实时/冻结都显示带叠加的预览（否则实时几乎看不到渲染）
  QImage img = session_ != nullptr ? session_->last_preview() : QImage();
  if (img.isNull() && session_ != nullptr) {
    img = session_->load_current_qimage();
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
      workbench_path_label_->setText(
          QStringLiteral("等待话题图像 · %1").arg(path));
    } else if (preview_live_) {
      workbench_path_label_->setText(
          QStringLiteral("实时 · %1%2").arg(path, extra));
    } else {
      workbench_path_label_->setText(
          QStringLiteral("冻结 · %1%2  （空格切换）").arg(path, extra));
    }
  }

  if (chk_auto_capture_ != nullptr) {
    chk_auto_capture_->setEnabled(ros_mode && preview_live_);
  }
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
    workbench_path_label_->setText(
        path.isEmpty()
            ? QStringLiteral("无图片")
            : QStringLiteral("%1 / %2  ·  %3")
                  .arg(session_->current_index() + 1)
                  .arg(session_->image_paths().size())
                  .arg(path));
  }
  update_preview_mode_ui();

  if (btn_prev_ != nullptr) {
    btn_prev_->setEnabled(!ros_mode);
    btn_next_->setEnabled(!ros_mode);
  }
  if (chk_auto_capture_ != nullptr && !ros_mode) {
    chk_auto_capture_->setChecked(false);
  }

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
      QString line = QString::fromStdString(obs.source_path);
      if (obs.has_base_gripper) {
        line += QStringLiteral("  [pose]");
      }
      obs_list_->addItem(line);
    }
  }

  set_metric_value(metric_frames_, QString::number(session_->observation_count()));
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
  set_metric_value(
      metric_coverage_,
      session_->observation_count() >= 8 ? QStringLiteral("较充分")
                                         : QStringLiteral("继续采集"));

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
    if (session_->is_handeye()) {
      review_text_->setPlainText(QString::fromStdString(core::format_extrinsics_text(
          r, session_->result_parent_frame().toStdString(),
          session_->result_child_frame().toStdString())));
    } else {
      review_text_->setPlainText(
          QString::fromStdString(core::format_intrinsics_text(r)));
    }
  }
  if (r.success) {
    if (session_->is_handeye()) {
      set_metric_value(
          review_rmse_,
          QString::number(
              r.metrics.count("handeye_rmse") ? r.metrics.at("handeye_rmse") : 0.0,
              'f', 3) +
              QStringLiteral(" deg"));
      set_metric_value(
          review_views_,
          QString::number(
              static_cast<int>(
                  r.metrics.count("num_pairs") ? r.metrics.at("num_pairs") : 0)));
      set_metric_value(
          review_size_,
          session_->result_parent_frame() + QStringLiteral("→") +
              session_->result_child_frame());
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
    session_->request_detect(session_->is_trihedral());
  } else {
    on_detect_and_preview();
  }
}

/// \brief 手动请求检测当前帧
void MainWindow::on_detect_and_preview() {
  if (session_ == nullptr || preview_view_ == nullptr) {
    return;
  }
  pending_detect_log_ = true;
  update_detect_status_ui();
  session_->request_detect(false);
}

/// \brief 手动采集当前观测
void MainWindow::on_capture_observation() {
  if (session_ == nullptr) {
    return;
  }
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
  append_log(
      LogLevel::Info,
      QStringLiteral("› 已采集 %1 帧（conf=%2%）")
          .arg(session_->observation_count())
          .arg(qRound(session_->last_confidence() * 100.0)));
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
  if (launcher_panel_ != nullptr) {
    hint = launcher_panel_->export_dir_hint();
    if (hint.startsWith(QLatin1Char('~'))) {
      hint = QDir::homePath() + hint.mid(1);
    }
  }
  if (hint.isEmpty()) {
    hint = QDir::homePath();
  }
  const QFileInfo hint_info(hint);
  QString parent = hint_info.isDir() || hint.endsWith(QLatin1Char('/'))
      ? hint
      : hint_info.absolutePath();
  if (parent.isEmpty() || parent == QStringLiteral(".")) {
    parent = QDir::homePath();
  }
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
  const QString suggested = QDir(parent).filePath(
      QStringLiteral("hs_calib_%1_%2").arg(session_->calibrator_id()).arg(stamp));

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
}

// ===== UI 构建 =====

}  // namespace gui
}  // namespace hs_calib
