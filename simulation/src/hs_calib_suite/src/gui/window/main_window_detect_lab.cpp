#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/bridges/ros_image_bridge.hpp"
#include "hs_calib_suite/gui/panels/launcher_config_panel.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <map>
#include <string>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace hs_calib {
namespace gui {

using hs_calib::gui::window_detail::make_label;

namespace {

bool lab_target_needs_dictionary(const QString &target) {
  return target == QStringLiteral("charuco") ||
         target == QStringLiteral("aruco") ||
         target == QStringLiteral("aruco_grid") ||
         target == QStringLiteral("trihedral_charuco");
}

bool lab_target_needs_tag_spacing(const QString &target) {
  return target == QStringLiteral("aprilgrid");
}

bool lab_target_needs_marker_len(const QString &target) {
  return lab_target_needs_dictionary(target) || lab_target_needs_tag_spacing(target);
}

bool lab_target_is_trihedral(const QString &target) {
  return target.startsWith(QStringLiteral("trihedral_"));
}

QString lab_target_id(const QComboBox *combo) {
  if (combo == nullptr) {
    return QStringLiteral("chessboard");
  }
  const QVariant d = combo->currentData();
  if (d.isValid() && !d.toString().isEmpty()) {
    return d.toString();
  }
  return combo->currentText();
}

void harden_form_field(QWidget *w) {
  if (w == nullptr) {
    return;
  }
  w->setMinimumHeight(34);
  w->setMaximumHeight(40);
  w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

}  // namespace

/// \brief 当前检测调试子模式（以 Session 为准，回退到任务 ID）
DetectLabMode MainWindow::detect_lab_mode() const {
  if (session_ != nullptr && session_->is_detect_lab()) {
    return session_->detect_lab_mode();
  }
  return detect_lab_mode_from_task_id(selected_calibrator_id_);
}

bool MainWindow::is_detect_lab_mode() const {
  return detect_lab_mode() != DetectLabMode::None;
}

bool MainWindow::is_detect_lab_full_mode() const {
  return detect_lab_mode() == DetectLabMode::Full;
}

bool MainWindow::is_detect_lab_identify_mode() const {
  return detect_lab_mode() == DetectLabMode::Identify;
}

/// \brief 按三模式切换独立右侧页，并刷新图像源控件
void MainWindow::refresh_lab_mode_ui() {
  const DetectLabMode mode = detect_lab_mode();
  const bool identify = (mode == DetectLabMode::Identify);
  const bool full = (mode == DetectLabMode::Full);

  if (lab_mode_stack_ != nullptr) {
    if (identify) {
      lab_mode_stack_->setCurrentIndex(0);
    } else if (full) {
      lab_mode_stack_->setCurrentIndex(2);
    } else {
      lab_mode_stack_->setCurrentIndex(1);
    }
    // 识别页需要拉高列表；局部/完整页按内容收紧，避免中间大块空白
    if (identify) {
      lab_mode_stack_->setMinimumHeight(280);
      lab_mode_stack_->setMaximumHeight(QWIDGETSIZE_MAX);
      lab_mode_stack_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    } else if (QWidget *cur = lab_mode_stack_->currentWidget()) {
      cur->adjustSize();
      const int h = std::max(1, cur->sizeHint().height());
      lab_mode_stack_->setMinimumHeight(0);
      lab_mode_stack_->setMaximumHeight(h + 4);
      lab_mode_stack_->setFixedHeight(h + 4);
      lab_mode_stack_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    }
  }

  if (lab_title_label_ != nullptr) {
    if (identify) {
      lab_title_label_->setText(QStringLiteral("标定板类型识别"));
    } else if (full) {
      lab_title_label_->setText(QStringLiteral("完整标定板检测"));
    } else {
      lab_title_label_->setText(QStringLiteral("局部特征检测"));
    }
  }
  if (lab_board_form_panel_ != nullptr) {
    lab_board_form_panel_->setVisible(!identify);
  }
  if (lab_detect_action_host_ != nullptr) {
    lab_detect_action_host_->setVisible(!identify);
  }

  if (lab_stats_ != nullptr && session_ != nullptr && !session_->detect_busy()) {
    if (identify && session_->last_identify_ranked().empty()) {
      lab_stats_->setText(QStringLiteral("等待识别"));
    } else if (!identify && !session_->has_current_detection()) {
      lab_stats_->setText(
          full ? QStringLiteral("等待完整检测")
               : QStringLiteral("等待局部检测"));
    }
  }

  refresh_lab_source_ui();
  refresh_status_task();
}

/// \brief 按图像源显隐：ROS 不显示离线目录/翻页；显示内参话题
void MainWindow::refresh_lab_source_ui() {
  if (session_ == nullptr) {
    return;
  }
  const SourceMode mode = session_->source_mode();
  const bool offline_like =
      (mode == SourceMode::Offline || mode == SourceMode::RosBag);
  const bool ros_topic = (mode == SourceMode::RosTopic);
  if (lab_offline_host_ != nullptr) {
    lab_offline_host_->setVisible(offline_like);
  }
  if (lab_nav_host_ != nullptr) {
    lab_nav_host_->setVisible(offline_like && !session_->image_paths().isEmpty());
  }
  if (lab_camera_info_host_ != nullptr) {
    lab_camera_info_host_->setVisible(ros_topic);
  }
  if (ros_topic) {
    apply_lab_camera_info_subscription();
  }
}

void MainWindow::apply_lab_camera_info_subscription() {
  if (ros_bridge_ == nullptr || session_ == nullptr) {
    return;
  }
  const bool on_lab =
      stack_ != nullptr &&
      stack_->currentIndex() == static_cast<int>(PageId::DetectLab);
  const bool ros_topic =
      session_->source_mode() == SourceMode::RosTopic;
  if (!on_lab || !ros_topic) {
    return;
  }
  const QString topic =
      combo_lab_camera_info_ != nullptr
          ? combo_lab_camera_info_->currentText().trimmed()
          : QString();
  if (topic.isEmpty()) {
    // 调试台未填内参话题：若工程标定也未用 CameraInfo，则退订并清检测内参
    const int setup_mode =
        launcher_panel_ != nullptr ? launcher_panel_->intrinsics_source_mode() : 0;
    if (setup_mode != 1) {
      ros_bridge_->unsubscribe_camera_info();
      session_->clear_detect_intrinsics();
    }
    return;
  }
  if (ros_bridge_->subscribed_camera_info_topic() != topic) {
    ros_bridge_->subscribe_camera_info(topic);
  }
  if (ros_bridge_->has_camera_info()) {
    session_->set_detect_intrinsics(
        ros_bridge_->camera_matrix(), ros_bridge_->dist_coeffs(),
        ros_bridge_->distortion_model().toStdString(), 0.0);
  }
}

void MainWindow::on_lab_camera_info_changed(const QString &topic) {
  (void)topic;
  apply_lab_camera_info_subscription();
  sync_detect_intrinsics_from_sources();
  if (combo_lab_camera_info_ != nullptr) {
    const QString t = combo_lab_camera_info_->currentText().trimmed();
    if (!t.isEmpty()) {
      append_log(LogLevel::Info, QStringLiteral("› 调试台订阅 CameraInfo：%1").arg(t));
    } else {
      append_log(LogLevel::Info, QStringLiteral("› 调试台未使用 CameraInfo（将 guess_K）"));
    }
  }
}

void MainWindow::on_lab_refresh_camera_info() {
  if (ros_bridge_ == nullptr || combo_lab_camera_info_ == nullptr) {
    return;
  }
  const QString prev = combo_lab_camera_info_->currentText();
  const QStringList topics = ros_bridge_->list_camera_info_topics();
  combo_lab_camera_info_->blockSignals(true);
  combo_lab_camera_info_->clear();
  combo_lab_camera_info_->addItems(topics);
  const int idx = combo_lab_camera_info_->findText(prev);
  if (idx >= 0) {
    combo_lab_camera_info_->setCurrentIndex(idx);
  } else if (!prev.isEmpty()) {
    combo_lab_camera_info_->setEditText(prev);
  } else if (!topics.isEmpty()) {
    combo_lab_camera_info_->setCurrentIndex(0);
  }
  combo_lab_camera_info_->blockSignals(false);
  append_log(
      LogLevel::Info,
      QStringLiteral("› 调试台刷新 CameraInfo 话题：%1 个").arg(topics.size()));
  apply_lab_camera_info_subscription();
  sync_detect_intrinsics_from_sources();
}

/// \brief 把检测台参数写入会话（标定器 / 靶标 / 可视化 / 完整度）
void MainWindow::apply_lab_params_to_session() {
  if (session_ == nullptr) {
    return;
  }
  if (is_detect_lab_identify_mode()) {
    // 识别模式不写板尺寸；检测路由仍用 cam_intrinsics 占位
    session_->set_calibrator_id(QStringLiteral("cam_intrinsics"));
    session_->set_board_params(9, 6, 0.025);
    std::map<std::string, std::string> opts = session_->solve_options();
    opts["detect_completeness"] = "partial";
    if (combo_lab_dict_hint_ != nullptr) {
      const QString hint = combo_lab_dict_hint_->currentText();
      if (!hint.isEmpty() && hint != QStringLiteral("（自动）")) {
        opts["dictionary"] = hint.toStdString();
      }
    }
    session_->set_solve_options(opts);
    session_->set_viz_options(true, true, true, 6, true);
    return;
  }
  if (combo_lab_target_ == nullptr) {
    return;
  }
  const QString target = lab_target_id(combo_lab_target_);
  const bool full = is_detect_lab_full_mode();
  session_->set_calibrator_id(
      lab_target_is_trihedral(target) ? QStringLiteral("trihedral_oneshot")
                                      : QStringLiteral("cam_intrinsics"));
  session_->set_board_params(
      spin_lab_squares_x_ != nullptr ? spin_lab_squares_x_->value() : 8,
      spin_lab_squares_y_ != nullptr ? spin_lab_squares_y_->value() : 8,
      spin_lab_square_len_ != nullptr
          ? (lab_target_needs_tag_spacing(target)
                 ? spin_lab_square_len_->value()              // AprilGrid: 无量纲间距
                 : spin_lab_square_len_->value() / 1000.0)    // UI mm → m
          : 0.025);

  std::map<std::string, std::string> opts = session_->solve_options();
  opts["target"] = target.toStdString();
  opts["detect_completeness"] = full ? "full" : "partial";
  if (combo_lab_dictionary_ != nullptr) {
    opts["dictionary"] = combo_lab_dictionary_->currentText().toStdString();
  }
  if (spin_lab_marker_len_ != nullptr) {
    opts["marker_length"] =
        std::to_string(spin_lab_marker_len_->value() / 1000.0);  // UI mm → m
  }
  if (spin_lab_square_len_ != nullptr && lab_target_needs_tag_spacing(target)) {
    opts["tag_spacing"] = std::to_string(spin_lab_square_len_->value());
  }
  // 局部棋盘：关 fast_check；完整：开 fast_check
  if (target == QStringLiteral("chessboard")) {
    opts["cb_fast_check"] = full ? "1" : "0";
  }
  session_->set_solve_options(opts);
  session_->set_viz_options(true, true, true, 6, true);
}

/// \brief 靶标切换时给出常用默认尺寸（不强制覆盖用户已改参数）
void MainWindow::sync_lab_target_defaults() {
  if (combo_lab_target_ == nullptr) {
    return;
  }
  const QString target = lab_target_id(combo_lab_target_);
  const bool need_dict = lab_target_needs_dictionary(target);
  const bool need_marker = lab_target_needs_marker_len(target);
  if (combo_lab_dictionary_ != nullptr) {
    combo_lab_dictionary_->setEnabled(need_dict);
  }
  if (spin_lab_marker_len_ != nullptr) {
    spin_lab_marker_len_->setEnabled(need_marker);
    spin_lab_marker_len_->setSuffix(QStringLiteral(" mm"));
    spin_lab_marker_len_->setDecimals(2);
    spin_lab_marker_len_->setRange(1.0, 1000.0);
    spin_lab_marker_len_->setSingleStep(0.5);
  }
  if (spin_lab_square_len_ != nullptr) {
    spin_lab_square_len_->setEnabled(true);
    if (lab_target_needs_tag_spacing(target)) {
      spin_lab_square_len_->setSuffix(QString());
      spin_lab_square_len_->setDecimals(3);
      spin_lab_square_len_->setRange(0.01, 2.0);
      spin_lab_square_len_->setSingleStep(0.05);
    } else {
      spin_lab_square_len_->setSuffix(QStringLiteral(" mm"));
      spin_lab_square_len_->setDecimals(2);
      spin_lab_square_len_->setRange(1.0, 1000.0);
      spin_lab_square_len_->setSingleStep(0.5);
    }
  }

  if (spin_lab_squares_x_ == nullptr || spin_lab_squares_y_ == nullptr) {
    return;
  }
  // 仅在仍为「上一靶标默认」时自动切换，避免打断手动调试
  static QString last_target;
  if (last_target == target) {
    return;
  }
  last_target = target;
  if (lab_target_is_trihedral(target)) {
    spin_lab_squares_x_->setValue(8);
    spin_lab_squares_y_->setValue(8);
  } else if (target == QStringLiteral("chessboard")) {
    spin_lab_squares_x_->setValue(9);
    spin_lab_squares_y_->setValue(6);
  } else if (target == QStringLiteral("circles_asymmetric")) {
    spin_lab_squares_x_->setValue(4);
    spin_lab_squares_y_->setValue(11);
  } else if (
      target == QStringLiteral("circles_symmetric") ||
      target == QStringLiteral("circle_grid")) {
    spin_lab_squares_x_->setValue(7);
    spin_lab_squares_y_->setValue(7);
  } else if (target == QStringLiteral("aruco")) {
    if (combo_lab_dictionary_ != nullptr) {
      const int didx = combo_lab_dictionary_->findText(QStringLiteral("DICT_6X6_1000"));
      if (didx >= 0) {
        combo_lab_dictionary_->setCurrentIndex(didx);
      }
    }
    if (spin_lab_marker_len_ != nullptr) {
      spin_lab_marker_len_->setValue(100.0);
    }
  } else if (target == QStringLiteral("aruco_grid")) {
    spin_lab_squares_x_->setValue(5);
    spin_lab_squares_y_->setValue(7);
  } else if (target == QStringLiteral("aprilgrid")) {
    spin_lab_squares_x_->setValue(6);
    spin_lab_squares_y_->setValue(6);
    if (spin_lab_marker_len_ != nullptr) {
      spin_lab_marker_len_->setValue(88.0);
    }
    if (spin_lab_square_len_ != nullptr) {
      spin_lab_square_len_->setValue(0.3);  // tagSpacing 无量纲
    }
  } else if (target == QStringLiteral("charuco")) {
    spin_lab_squares_x_->setValue(5);
    spin_lab_squares_y_->setValue(7);
    if (combo_lab_dictionary_ != nullptr) {
      const int didx = combo_lab_dictionary_->findText(QStringLiteral("DICT_4X4_50"));
      if (didx >= 0) {
        combo_lab_dictionary_->setCurrentIndex(didx);
      }
    }
    if (spin_lab_square_len_ != nullptr) {
      spin_lab_square_len_->setValue(40.0);
    }
    if (spin_lab_marker_len_ != nullptr) {
      spin_lab_marker_len_->setValue(30.0);
    }
  }
}

/// \brief 刷新检测台预览与统计
void MainWindow::refresh_detect_lab_view(bool prefer_preview) {
  if (session_ == nullptr) {
    return;
  }
  if (lab_path_label_ != nullptr) {
    const QStringList &paths = session_->image_paths();
    const int n = paths.size();
    const int idx = session_->current_index();
    if (n <= 0) {
      lab_path_label_->setText(
          session_->source_mode() == SourceMode::RosTopic
              ? QStringLiteral("源：ROS 实时帧（无离线目录）")
              : QStringLiteral("未加载图片目录"));
    } else {
      lab_path_label_->setText(
          QStringLiteral("[%1/%2]  %3")
              .arg(idx + 1)
              .arg(n)
              .arg(session_->current_path()));
    }
  }

  QImage img;
  if (prefer_preview && session_->has_current_detection()) {
    img = session_->last_preview();
  }
  if (img.isNull()) {
    img = session_->last_preview();
  }
  if (img.isNull()) {
    img = session_->load_current_qimage();
  }
  if (lab_preview_ != nullptr) {
    if (img.isNull()) {
      lab_preview_->clear_image();
    } else {
      lab_preview_->set_image(img);
    }
  }

  if (lab_stats_ != nullptr) {
    if (session_->detect_busy()) {
      lab_stats_->setText(
          is_detect_lab_identify_mode() ? QStringLiteral("识别中…")
                                        : QStringLiteral("检测中…"));
    } else if (is_detect_lab_identify_mode()) {
      const auto &ranked = session_->last_identify_ranked();
      if (ranked.empty()) {
        lab_stats_->setText(QStringLiteral("无识别结果\n点击「识别类型」试探当前帧"));
      } else {
        const auto &top = ranked.front();
        QString s = QStringLiteral("最可能 %1 · score %2\n特征 %3")
                        .arg(QString::fromStdString(top.type_id))
                        .arg(top.score, 0, 'f', 2)
                        .arg(top.feature_count);
        if (!top.dict_hint.empty()) {
          s += QStringLiteral(" · 字典 %1")
                   .arg(QString::fromStdString(top.dict_hint));
        }
        lab_stats_->setText(s);
      }
      populate_lab_identify_results();
    } else if (session_->has_current_detection()) {
      QString s = QStringLiteral("点数 %1 · conf %2%")
                      .arg(session_->last_point_count())
                      .arg(qRound(session_->last_confidence() * 100.0));
      if (session_->last_aruco_reproj_px() >= 0.0) {
        s += QStringLiteral(" · reproj %1 px")
                 .arg(session_->last_aruco_reproj_px(), 0, 'f', 3);
      }
      if (session_->is_trihedral() && session_->last_faces_found() > 0) {
        s += QStringLiteral(" · %1 面").arg(session_->last_faces_found());
      }
      s += QStringLiteral("\n模式 %1 · 目标 %2 · 网格 %3×%4")
               .arg(
                   is_detect_lab_full_mode() ? QStringLiteral("完整")
                                            : QStringLiteral("局部"))
               .arg(
                   combo_lab_target_ != nullptr ? combo_lab_target_->currentText()
                                                : QStringLiteral("—"))
               .arg(session_->squares_x())
               .arg(session_->squares_y());
      const QString lab_tid = lab_target_id(combo_lab_target_);
      if (lab_tid == QStringLiteral("aruco") ||
          lab_tid == QStringLiteral("aruco_grid") ||
          lab_tid == QStringLiteral("charuco") ||
          lab_tid.contains(QStringLiteral("charuco"))) {
        const int n_markers = session_->last_point_count() / 4;
        if (lab_tid == QStringLiteral("aruco") && n_markers > 0) {
          s += QStringLiteral(" · 码数 ~%1").arg(n_markers);
        }
      }
      lab_stats_->setText(s);
    } else {
      lab_stats_->setText(
          QStringLiteral("无有效检测（%1）\n目标 %2 · 网格 %3×%4")
              .arg(
                  is_detect_lab_full_mode() ? QStringLiteral("完整模式")
                                           : QStringLiteral("局部模式"))
              .arg(
                  combo_lab_target_ != nullptr ? combo_lab_target_->currentText()
                                               : QStringLiteral("—"))
              .arg(
                  spin_lab_squares_x_ != nullptr ? spin_lab_squares_x_->value() : 0)
              .arg(
                  spin_lab_squares_y_ != nullptr ? spin_lab_squares_y_->value()
                                                 : 0));
    }
  }

  const bool busy = session_->detect_busy();
  auto set_en = [busy](QPushButton *b) {
    if (b != nullptr) {
      b->setEnabled(!busy);
    }
  };
  set_en(btn_lab_identify_);
  set_en(btn_lab_detect_);
  if (btn_lab_export_json_ != nullptr) {
    btn_lab_export_json_->setEnabled(
        !busy && is_detect_lab_identify_mode() &&
        !session_->last_identify_ranked().empty());
  }
  refresh_lab_source_ui();
}

/// \brief 选择离线图片目录
void MainWindow::on_lab_browse_images() {
  const QString dir = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择检测图片目录"),
      edit_lab_image_dir_ != nullptr ? edit_lab_image_dir_->text() : QString());
  if (dir.isEmpty() || session_ == nullptr) {
    return;
  }
  if (edit_lab_image_dir_ != nullptr) {
    edit_lab_image_dir_->setText(dir);
  }
  session_->set_source_mode(SourceMode::Offline);
  const int n = session_->load_image_dir(dir);
  append_log(
      LogLevel::Info,
      QStringLiteral("› 检测台加载目录：%1 张").arg(n));
  if (n > 0) {
    session_->set_current_index(0);
  }
  refresh_detect_lab_view(false);
}

/// \brief 运行局部 / 完整检测
void MainWindow::on_lab_detect(bool fast) {
  if (session_ == nullptr) {
    return;
  }
  if (is_detect_lab_identify_mode()) {
    on_lab_identify();
    return;
  }
  apply_lab_params_to_session();
  sync_detect_intrinsics_from_sources();
  if (session_->source_mode() == SourceMode::Offline &&
      session_->image_paths().isEmpty()) {
    append_log(LogLevel::Error, QStringLiteral("› 请先加载图片目录"));
    return;
  }
  if (session_->source_mode() == SourceMode::RosTopic &&
      !session_->has_live_frame()) {
    append_log(LogLevel::Error, QStringLiteral("› 尚无 ROS 图像帧"));
    return;
  }
  lab_pending_log_ = true;
  auto disable = [](QPushButton *b) {
    if (b != nullptr) {
      b->setEnabled(false);
    }
  };
  disable(btn_lab_detect_);
  if (lab_stats_ != nullptr) {
    const bool full = is_detect_lab_full_mode();
    lab_stats_->setText(
        fast ? (full ? QStringLiteral("快速检测中…")
                     : QStringLiteral("快速检测中…"))
             : (full ? QStringLiteral("完整检测中…")
                     : QStringLiteral("局部检测中…")));
  }
  session_->request_detect(fast);
}

/// \brief 收集识别选项（候选类型 + 字典提示 + 短扫上限）
core::BoardTypeIdentifyOptions MainWindow::collect_lab_identify_options() const {
  core::BoardTypeIdentifyOptions opts;
  opts.draw_overlay = true;
  if (spin_lab_dict_scan_ != nullptr) {
    opts.max_dictionary_scan = spin_lab_dict_scan_->value();
  }
  if (combo_lab_dict_hint_ != nullptr) {
    const QString hint = combo_lab_dict_hint_->currentText();
    if (!hint.isEmpty() && hint != QStringLiteral("（自动）")) {
      opts.dictionary_hints.push_back(hint.toStdString());
    }
  }
  auto push_if = [&](QCheckBox *chk, const char *id) {
    if (chk != nullptr && chk->isChecked()) {
      opts.candidate_types.emplace_back(id);
    }
  };
  push_if(chk_lab_cand_chessboard_, "chessboard");
  push_if(chk_lab_cand_charuco_, "charuco");
  push_if(chk_lab_cand_aruco_, "aruco");
  push_if(chk_lab_cand_aruco_grid_, "aruco_grid");
  push_if(chk_lab_cand_aprilgrid_, "aprilgrid");
  push_if(chk_lab_cand_circles_sym_, "circles_symmetric");
  push_if(chk_lab_cand_circles_asym_, "circles_asymmetric");
  push_if(chk_lab_cand_tri_chess_, "trihedral_chess");
  push_if(chk_lab_cand_tri_charuco_, "trihedral_charuco");
  return opts;
}

/// \brief 运行标定板类型识别（忽略尺寸）
void MainWindow::on_lab_identify() {
  if (session_ == nullptr) {
    return;
  }
  apply_lab_params_to_session();
  if (session_->source_mode() == SourceMode::Offline &&
      session_->image_paths().isEmpty()) {
    append_log(LogLevel::Error, QStringLiteral("› 请先加载图片目录"));
    return;
  }
  if (session_->source_mode() == SourceMode::RosTopic &&
      !session_->has_live_frame()) {
    append_log(LogLevel::Error, QStringLiteral("› 尚无 ROS 图像帧"));
    return;
  }
  const auto opts = collect_lab_identify_options();
  if (opts.candidate_types.empty()) {
    append_log(LogLevel::Error, QStringLiteral("› 请至少勾选一种候选类型"));
    return;
  }
  lab_pending_identify_log_ = true;
  if (btn_lab_identify_ != nullptr) {
    btn_lab_identify_->setEnabled(false);
  }
  if (lab_stats_ != nullptr) {
    lab_stats_->setText(QStringLiteral("识别类型中…"));
  }
  session_->set_detect_lab_mode(DetectLabMode::Identify);
  session_->request_identify(opts);
}

void MainWindow::populate_lab_identify_results() {
  if (lab_identify_list_ == nullptr || session_ == nullptr) {
    return;
  }
  lab_identify_list_->clear();
  const auto &ranked = session_->last_identify_ranked();
  for (size_t i = 0; i < ranked.size(); ++i) {
    const auto &h = ranked[i];
    QString line = QStringLiteral("%1. %2    score %3    feats %4")
                       .arg(static_cast<int>(i) + 1)
                       .arg(QString::fromStdString(h.type_id))
                       .arg(h.score, 0, 'f', 2)
                       .arg(h.feature_count);
    if (!h.dict_hint.empty()) {
      line += QStringLiteral("    [%1]").arg(QString::fromStdString(h.dict_hint));
    }
    if (!h.note.empty()) {
      line += QStringLiteral("  · %1").arg(QString::fromStdString(h.note));
    }
    auto *item = new QListWidgetItem(line, lab_identify_list_);
    item->setData(Qt::UserRole, QString::fromStdString(h.type_id));
    item->setData(Qt::UserRole + 1, QString::fromStdString(h.dict_hint));
  }
  if (!ranked.empty()) {
    lab_identify_list_->setCurrentRow(0);
  }
}

void MainWindow::on_async_identify_started() {
  if (stack_ == nullptr ||
      stack_->currentIndex() != static_cast<int>(PageId::DetectLab)) {
    return;
  }
  if (btn_lab_identify_ != nullptr) {
    btn_lab_identify_->setEnabled(false);
  }
  if (lab_stats_ != nullptr) {
    lab_stats_->setText(QStringLiteral("识别中…"));
  }
}

void MainWindow::on_async_identify_finished(bool ok, const QString &err) {
  if (stack_ == nullptr ||
      stack_->currentIndex() != static_cast<int>(PageId::DetectLab)) {
    return;
  }
  refresh_detect_lab_view(true);
  if (!lab_pending_identify_log_) {
    return;
  }
  lab_pending_identify_log_ = false;
  if (ok) {
    const auto &ranked = session_->last_identify_ranked();
    const QString top =
        ranked.empty() ? QStringLiteral("—")
                       : QString::fromStdString(ranked.front().type_id);
    append_log(
        LogLevel::Info,
        QStringLiteral("› 类型识别成功：top=%1（%2）").arg(top, err));
  } else {
    append_log(LogLevel::Error, QStringLiteral("› 类型识别失败：%1").arg(err));
    if (lab_stats_ != nullptr) {
      lab_stats_->setText(QStringLiteral("失败：%1").arg(err));
    }
  }
}

/// \brief 导出识别报告（调试用途，不跳转工程标定）
void MainWindow::on_lab_export_identify_json() {
  if (session_ == nullptr) {
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("导出识别报告 JSON"),
      QStringLiteral("board_type_identify.json"),
      QStringLiteral("JSON (*.json)"));
  if (path.isEmpty()) {
    return;
  }
  QString err;
  if (!session_->export_identify_json(path, &err)) {
    append_log(LogLevel::Error, QStringLiteral("› 导出失败：%1").arg(err));
    return;
  }
  append_log(LogLevel::Info, QStringLiteral("› 已导出识别报告：%1").arg(path));
}

/// \brief 构建靶标参数表单（局部 / 完整共用）
QWidget *MainWindow::build_lab_board_form(QWidget *parent) {
  auto *form_host = new QWidget(parent);
  form_host->setObjectName(QStringLiteral("DetectLabForm"));
  auto *form = new QFormLayout(form_host);
  form->setContentsMargins(0, 4, 0, 4);
  form->setHorizontalSpacing(12);
  form->setVerticalSpacing(10);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);

  combo_lab_target_ = new QComboBox(form_host);
  auto add_lab_target = [this](const QString &id, const QString &label) {
    combo_lab_target_->addItem(label, id);
  };
  add_lab_target(QStringLiteral("chessboard"), QStringLiteral("chessboard"));
  add_lab_target(QStringLiteral("charuco"), QStringLiteral("charuco"));
  add_lab_target(QStringLiteral("aruco"), QStringLiteral("aruco"));
  add_lab_target(QStringLiteral("aruco_grid"), QStringLiteral("aruco_grid"));
  add_lab_target(QStringLiteral("aprilgrid"), QStringLiteral("aprilgrid"));
  add_lab_target(QStringLiteral("circles_symmetric"), QStringLiteral("Circles"));
  add_lab_target(
      QStringLiteral("circles_asymmetric"), QStringLiteral("Asymmetric Circles"));
  add_lab_target(QStringLiteral("trihedral_chess"), QStringLiteral("trihedral_chess"));
  add_lab_target(
      QStringLiteral("trihedral_charuco"), QStringLiteral("trihedral_charuco"));
  harden_form_field(combo_lab_target_);
  form->addRow(QStringLiteral("靶标类型"), combo_lab_target_);

  combo_lab_dictionary_ = new QComboBox(form_host);
  combo_lab_dictionary_->addItems(
      {QStringLiteral("DICT_6X6_1000"), QStringLiteral("DICT_6X6_250"),
       QStringLiteral("DICT_4X4_250"), QStringLiteral("DICT_4X4_50"),
       QStringLiteral("DICT_4X4_100"), QStringLiteral("DICT_5X5_50"),
       QStringLiteral("DICT_5X5_100"), QStringLiteral("DICT_5X5_250"),
       QStringLiteral("DICT_6X6_50"), QStringLiteral("DICT_7X7_1000"),
       QStringLiteral("DICT_ARUCO_ORIGINAL"),
       QStringLiteral("DICT_APRILTAG_36h11")});
  harden_form_field(combo_lab_dictionary_);
  form->addRow(QStringLiteral("字典"), combo_lab_dictionary_);

  spin_lab_squares_x_ = new QSpinBox(form_host);
  spin_lab_squares_x_->setRange(2, 40);
  spin_lab_squares_x_->setValue(9);
  harden_form_field(spin_lab_squares_x_);
  form->addRow(QStringLiteral("squares_x"), spin_lab_squares_x_);

  spin_lab_squares_y_ = new QSpinBox(form_host);
  spin_lab_squares_y_->setRange(2, 40);
  spin_lab_squares_y_->setValue(6);
  harden_form_field(spin_lab_squares_y_);
  form->addRow(QStringLiteral("squares_y"), spin_lab_squares_y_);

  spin_lab_square_len_ = new QDoubleSpinBox(form_host);
  spin_lab_square_len_->setDecimals(2);
  spin_lab_square_len_->setRange(1.0, 1000.0);
  spin_lab_square_len_->setSingleStep(0.5);
  spin_lab_square_len_->setValue(25.0);
  spin_lab_square_len_->setSuffix(QStringLiteral(" mm"));
  harden_form_field(spin_lab_square_len_);
  form->addRow(QStringLiteral("方格边长"), spin_lab_square_len_);

  spin_lab_marker_len_ = new QDoubleSpinBox(form_host);
  spin_lab_marker_len_->setDecimals(2);
  spin_lab_marker_len_->setRange(1.0, 1000.0);
  spin_lab_marker_len_->setSingleStep(0.5);
  spin_lab_marker_len_->setValue(18.0);
  spin_lab_marker_len_->setSuffix(QStringLiteral(" mm"));
  harden_form_field(spin_lab_marker_len_);
  form->addRow(QStringLiteral("码边长"), spin_lab_marker_len_);

  connect(
      spin_lab_squares_x_, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [this](int) { refresh_status_task(); });
  connect(
      spin_lab_squares_y_, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [this](int) { refresh_status_task(); });

  connect(
      combo_lab_target_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
      [this](int) {
        sync_lab_target_defaults();
        refresh_status_task();
      });
  return form_host;
}

/// \brief 图像源块：ROS 仅提示；离线才显示目录与翻页
QWidget *MainWindow::build_lab_source_block(QWidget *parent) {
  auto *dir_body = new QWidget(parent);
  auto *dir_layout = new QVBoxLayout(dir_body);
  dir_layout->setContentsMargins(0, 0, 0, 0);
  dir_layout->setSpacing(8);

  lab_offline_host_ = new QWidget(dir_body);
  auto *off_lay = new QVBoxLayout(lab_offline_host_);
  off_lay->setContentsMargins(0, 0, 0, 0);
  off_lay->setSpacing(6);
  edit_lab_image_dir_ = new QLineEdit(lab_offline_host_);
  edit_lab_image_dir_->setPlaceholderText(QStringLiteral("离线图片目录…"));
  harden_form_field(edit_lab_image_dir_);
  auto *dir_row = new QHBoxLayout;
  auto *btn_browse = new QPushButton(QStringLiteral("浏览…"), lab_offline_host_);
  connect(btn_browse, &QPushButton::clicked, this, &MainWindow::on_lab_browse_images);
  dir_row->addWidget(edit_lab_image_dir_, 1);
  dir_row->addWidget(btn_browse);
  off_lay->addLayout(dir_row);
  dir_layout->addWidget(lab_offline_host_);

  lab_camera_info_host_ = new QWidget(dir_body);
  auto *info_lay = new QVBoxLayout(lab_camera_info_host_);
  info_lay->setContentsMargins(0, 0, 0, 0);
  info_lay->setSpacing(6);
  info_lay->addWidget(make_label(
      QStringLiteral("内参话题 (camera_info)"), QStringLiteral("Muted"),
      lab_camera_info_host_));
  auto *info_row = new QHBoxLayout;
  combo_lab_camera_info_ = new QComboBox(lab_camera_info_host_);
  combo_lab_camera_info_->setEditable(true);
  combo_lab_camera_info_->setInsertPolicy(QComboBox::NoInsert);
  combo_lab_camera_info_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  if (combo_lab_camera_info_->lineEdit() != nullptr) {
    combo_lab_camera_info_->lineEdit()->setPlaceholderText(
        QStringLiteral("可选，例如 /camera/camera_info"));
  }
  harden_form_field(combo_lab_camera_info_);
  // 沿用数据源页已填的 CameraInfo
  if (combo_camera_info_topic_ != nullptr &&
      !combo_camera_info_topic_->currentText().trimmed().isEmpty()) {
    combo_lab_camera_info_->setEditText(
        combo_camera_info_topic_->currentText().trimmed());
  }
  btn_lab_refresh_camera_info_ =
      new QPushButton(QStringLiteral("刷新"), lab_camera_info_host_);
  btn_lab_refresh_camera_info_->setObjectName(QStringLiteral("GhostButton"));
  connect(
      combo_lab_camera_info_, &QComboBox::currentTextChanged, this,
      &MainWindow::on_lab_camera_info_changed);
  connect(
      btn_lab_refresh_camera_info_, &QPushButton::clicked, this,
      &MainWindow::on_lab_refresh_camera_info);
  info_row->addWidget(combo_lab_camera_info_, 1);
  info_row->addWidget(btn_lab_refresh_camera_info_);
  info_lay->addLayout(info_row);
  dir_layout->addWidget(lab_camera_info_host_);

  auto *btn_to_source = new QPushButton(QStringLiteral("打开数据源设置"), dir_body);
  btn_to_source->setObjectName(QStringLiteral("GhostButton"));
  connect(btn_to_source, &QPushButton::clicked, this, [this]() {
    go_to(PageId::DataSource);
  });
  dir_layout->addWidget(btn_to_source);

  lab_nav_host_ = new QWidget(dir_body);
  auto *nav_row = new QHBoxLayout(lab_nav_host_);
  nav_row->setContentsMargins(0, 0, 0, 0);
  btn_lab_prev_ = new QPushButton(QStringLiteral("上一张"), lab_nav_host_);
  btn_lab_next_ = new QPushButton(QStringLiteral("下一张"), lab_nav_host_);
  connect(btn_lab_prev_, &QPushButton::clicked, this, [this]() {
    if (session_ == nullptr || session_->image_paths().isEmpty()) {
      return;
    }
    const int i = session_->current_index();
    if (i > 0) {
      session_->set_current_index(i - 1);
      refresh_detect_lab_view(false);
    }
  });
  connect(btn_lab_next_, &QPushButton::clicked, this, [this]() {
    if (session_ == nullptr || session_->image_paths().isEmpty()) {
      return;
    }
    const int i = session_->current_index();
    if (i + 1 < session_->image_paths().size()) {
      session_->set_current_index(i + 1);
      refresh_detect_lab_view(false);
    }
  });
  nav_row->addWidget(btn_lab_prev_);
  nav_row->addWidget(btn_lab_next_);
  dir_layout->addWidget(lab_nav_host_);
  return dir_body;
}

QWidget *MainWindow::build_lab_identify_side() {
  auto *page = new QWidget;
  auto *lay = new QVBoxLayout(page);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(10);

  auto *identify_host = new QWidget(page);
  auto *identify_lay = new QVBoxLayout(identify_host);
  identify_lay->setContentsMargins(0, 0, 0, 0);
  identify_lay->setSpacing(8);
  auto *identify_form = new QFormLayout;
  combo_lab_dict_hint_ = new QComboBox(identify_host);
  combo_lab_dict_hint_->addItem(QStringLiteral("（自动）"));
  combo_lab_dict_hint_->addItems(
      {QStringLiteral("DICT_4X4_50"), QStringLiteral("DICT_4X4_100"),
       QStringLiteral("DICT_4X4_250"), QStringLiteral("DICT_5X5_100"),
       QStringLiteral("DICT_5X5_250"), QStringLiteral("DICT_6X6_250"),
       QStringLiteral("DICT_6X6_1000"), QStringLiteral("DICT_APRILTAG_36h11")});
  harden_form_field(combo_lab_dict_hint_);
  identify_form->addRow(QStringLiteral("字典提示"), combo_lab_dict_hint_);
  spin_lab_dict_scan_ = new QSpinBox(identify_host);
  spin_lab_dict_scan_->setRange(1, 12);
  spin_lab_dict_scan_->setValue(8);
  harden_form_field(spin_lab_dict_scan_);
  identify_form->addRow(QStringLiteral("字典短扫"), spin_lab_dict_scan_);
  identify_lay->addLayout(identify_form);

  lab_candidate_host_ = new QWidget(identify_host);
  auto *cand_grid = new QGridLayout(lab_candidate_host_);
  cand_grid->setContentsMargins(0, 0, 0, 0);
  cand_grid->setHorizontalSpacing(8);
  cand_grid->setVerticalSpacing(4);
  auto add_cand = [&](QCheckBox *&slot, const QString &text, int row, int col) {
    slot = new QCheckBox(text, lab_candidate_host_);
    slot->setChecked(true);
    cand_grid->addWidget(slot, row, col);
  };
  add_cand(chk_lab_cand_chessboard_, QStringLiteral("chessboard"), 0, 0);
  add_cand(chk_lab_cand_charuco_, QStringLiteral("charuco"), 0, 1);
  add_cand(chk_lab_cand_aruco_, QStringLiteral("aruco"), 1, 0);
  add_cand(chk_lab_cand_aruco_grid_, QStringLiteral("aruco_grid"), 1, 1);
  add_cand(chk_lab_cand_aprilgrid_, QStringLiteral("aprilgrid"), 2, 0);
  add_cand(chk_lab_cand_circles_sym_, QStringLiteral("circles_sym"), 2, 1);
  add_cand(chk_lab_cand_circles_asym_, QStringLiteral("circles_asym"), 3, 0);
  add_cand(chk_lab_cand_tri_chess_, QStringLiteral("trihedral_chess"), 3, 1);
  add_cand(chk_lab_cand_tri_charuco_, QStringLiteral("trihedral_charuco"), 4, 0);
  identify_lay->addWidget(
      make_label(QStringLiteral("候选类型"), QStringLiteral("Muted"), identify_host));
  identify_lay->addWidget(lab_candidate_host_);
  lab_identify_list_ = new QListWidget(identify_host);
  lab_identify_list_->setMinimumHeight(160);
  lab_identify_list_->setAlternatingRowColors(true);
  identify_lay->addWidget(lab_identify_list_, 1);
  lab_identify_panel_ = make_panel(QStringLiteral("识别设置"), identify_host);
  lay->addWidget(lab_identify_panel_, 1);

  btn_lab_identify_ = new QPushButton(QStringLiteral("识别类型"), page);
  btn_lab_identify_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_lab_export_json_ = new QPushButton(QStringLiteral("导出识别 JSON"), page);
  btn_lab_export_json_->setObjectName(QStringLiteral("CompactButton"));
  connect(btn_lab_identify_, &QPushButton::clicked, this, &MainWindow::on_lab_identify);
  connect(
      btn_lab_export_json_, &QPushButton::clicked, this,
      &MainWindow::on_lab_export_identify_json);
  lay->addWidget(btn_lab_identify_);
  lay->addWidget(btn_lab_export_json_);
  return page;
}

QWidget *MainWindow::build_lab_partial_side() {
  // 局部/完整共用侧栏「采集检测 + 重投影」动作条；此处仅占位
  auto *page = new QWidget;
  page->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  page->setFixedHeight(0);
  return page;
}

QWidget *MainWindow::build_lab_full_side() {
  auto *page = new QWidget;
  page->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  page->setFixedHeight(0);
  return page;
}

/// \brief 检测调试容器：共享预览（实时/冻结）+ 三模式独立右侧页
QWidget *MainWindow::build_detect_lab_page() {
  auto *page = new QWidget;
  auto *root = new QHBoxLayout(page);
  root->setContentsMargins(16, 12, 16, 8);
  root->setSpacing(14);

  auto *preview_col = new QWidget(page);
  auto *preview_col_lay = new QVBoxLayout(preview_col);
  preview_col_lay->setContentsMargins(0, 0, 0, 0);
  preview_col_lay->setSpacing(0);

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
  lab_preview_title_label_ =
      make_label(QStringLiteral("检测预览"), QStringLiteral("SectionTitle"), preview_header);
  preview_header_lay->addWidget(lab_preview_title_label_);
  preview_header_lay->addStretch(1);
  btn_lab_preview_live_ = new QPushButton(QStringLiteral("实时"), preview_header);
  btn_lab_preview_live_->setObjectName(QStringLiteral("ModeChip"));
  btn_lab_preview_live_->setCheckable(true);
  btn_lab_preview_live_->setChecked(true);
  btn_lab_preview_live_->setToolTip(
      QStringLiteral("持续刷新话题图像并叠加检测；空格可切换"));
  btn_lab_preview_freeze_ = new QPushButton(QStringLiteral("冻结"), preview_header);
  btn_lab_preview_freeze_->setObjectName(QStringLiteral("ModeChip"));
  btn_lab_preview_freeze_->setCheckable(true);
  btn_lab_preview_freeze_->setToolTip(
      QStringLiteral("暂停画面与检测叠加，便于细看效果；可再点「采集检测」"));
  auto *lab_preview_mode_group = new QButtonGroup(preview_panel);
  lab_preview_mode_group->setExclusive(true);
  lab_preview_mode_group->addButton(btn_lab_preview_live_);
  lab_preview_mode_group->addButton(btn_lab_preview_freeze_);
  connect(btn_lab_preview_live_, &QPushButton::clicked, this, [this]() {
    set_preview_live(true);
  });
  connect(btn_lab_preview_freeze_, &QPushButton::clicked, this, [this]() {
    set_preview_live(false);
  });
  preview_header_lay->addWidget(btn_lab_preview_live_);
  preview_header_lay->addWidget(btn_lab_preview_freeze_);
  preview_panel_lay->addWidget(preview_header);

  auto *preview_body = new QWidget(preview_panel);
  auto *preview_layout = new QVBoxLayout(preview_body);
  preview_layout->setContentsMargins(8, 8, 8, 8);
  preview_layout->setSpacing(6);
  lab_path_label_ =
      make_label(QStringLiteral("未加载图片"), QStringLiteral("Muted"), preview_body);
  lab_preview_ = new ImageViewWidget(preview_body);
  lab_preview_->set_placeholder(QStringLiteral("配置图像源后显示预览 / 检测叠加"));
  lab_preview_->setMinimumSize(520, 380);
  preview_layout->addWidget(lab_path_label_);
  preview_layout->addWidget(lab_preview_, 1);
  preview_panel_lay->addWidget(preview_body, 1);
  preview_col_lay->addWidget(preview_panel, 1);
  root->addWidget(preview_col, 1);

  auto *side_col = new QWidget(page);
  auto *side_col_lay = new QVBoxLayout(side_col);
  side_col_lay->setContentsMargins(0, 0, 0, 0);
  side_col_lay->setSpacing(8);
  side_col->setMinimumWidth(340);
  side_col->setMaximumWidth(460);

  auto *side_scroll = new QScrollArea(side_col);
  side_scroll->setWidgetResizable(true);
  side_scroll->setFrameShape(QFrame::NoFrame);
  side_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  auto *side = new QWidget;
  auto *side_layout = new QVBoxLayout(side);
  side_layout->setContentsMargins(8, 4, 8, 8);
  side_layout->setSpacing(10);

  lab_title_label_ =
      make_label(QStringLiteral("局部特征检测"), QStringLiteral("PageTitle"), side);
  side_layout->addWidget(lab_title_label_);

  side_layout->addWidget(
      make_panel(QStringLiteral("图像源"), build_lab_source_block(side)));

  lab_board_form_panel_ =
      make_panel(QStringLiteral("靶标参数"), build_lab_board_form(side));
  side_layout->addWidget(lab_board_form_panel_);

  lab_mode_stack_ = new QStackedWidget(side);
  lab_mode_stack_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  lab_mode_stack_->addWidget(build_lab_identify_side());  // 0
  lab_mode_stack_->addWidget(build_lab_partial_side());   // 1
  lab_mode_stack_->addWidget(build_lab_full_side());      // 2
  side_layout->addWidget(lab_mode_stack_, 0);

  lab_detect_action_host_ = new QWidget(side);
  auto *detect_lay = new QVBoxLayout(lab_detect_action_host_);
  detect_lay->setContentsMargins(0, 0, 0, 0);
  detect_lay->setSpacing(0);
  btn_lab_detect_ = new QPushButton(QStringLiteral("采集检测"), lab_detect_action_host_);
  btn_lab_detect_->setObjectName(QStringLiteral("PrimaryButton"));
  connect(btn_lab_detect_, &QPushButton::clicked, this, [this]() {
    on_lab_detect(false);
  });
  detect_lay->addWidget(btn_lab_detect_);
  side_layout->addWidget(lab_detect_action_host_);

  lab_stats_ = make_label(QStringLiteral("等待操作"), QStringLiteral("Muted"), side);
  lab_stats_->setWordWrap(true);
  side_layout->addWidget(lab_stats_);

  auto *btn_back = new QPushButton(QStringLiteral("返回首页"), side);
  connect(btn_back, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  side_layout->addWidget(btn_back);
  side_layout->addStretch(1);

  side_scroll->setWidget(side);
  side_col_lay->addWidget(side_scroll, 1);
  root->addWidget(side_col, 0);

  sync_lab_target_defaults();
  refresh_lab_mode_ui();
  return page;
}

}  // namespace gui
}  // namespace hs_calib
