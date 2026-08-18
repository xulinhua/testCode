#include "hs_calib_suite/gui/window/main_window.hpp"

#include "main_window_helpers.hpp"

#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <map>
#include <string>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpinBox>
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

bool lab_target_needs_marker_len(const QString &target) {
  return lab_target_needs_dictionary(target);
}

bool lab_target_is_trihedral(const QString &target) {
  return target.startsWith(QStringLiteral("trihedral_"));
}

void harden_form_field(QWidget *w) {
  if (w == nullptr) {
    return;
  }
  w->setMinimumHeight(36);
  w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

}  // namespace

/// \brief 当前是否为「完整标定板」检测台模式
bool MainWindow::is_detect_lab_full_mode() const {
  return selected_calibrator_id_ == QStringLiteral("detect_lab_full");
}

/// \brief 按局部/完整模式刷新标题与按钮文案
void MainWindow::refresh_lab_mode_ui() {
  const bool full = is_detect_lab_full_mode();
  if (lab_title_label_ != nullptr) {
    lab_title_label_->setText(
        full ? QStringLiteral("完整标定板检测")
             : QStringLiteral("局部特征检测台"));
  }
  if (lab_subtitle_label_ != nullptr) {
    lab_subtitle_label_->setText(
        full ? QStringLiteral(
                   "只认整面网格；拍摄不全 / 缺角的帧直接判失败，适合验收标定采集质量。")
             : QStringLiteral(
                   "针对棋盘 / ChArUco / Tag 等局部可见内容尽量检出；Thorough 更全，"
                   "Fast 限时。不走采集求解流程。"));
  }
  if (btn_lab_detect_ != nullptr) {
    btn_lab_detect_->setText(
        full ? QStringLiteral("完整检测") : QStringLiteral("Thorough 尽量检出"));
  }
  if (btn_lab_detect_fast_ != nullptr) {
    btn_lab_detect_fast_->setText(
        full ? QStringLiteral("快速完整检测") : QStringLiteral("Fast 限时"));
  }
  if (lab_stats_ != nullptr && session_ != nullptr &&
      !session_->has_current_detection() && !session_->detect_busy()) {
    lab_stats_->setText(
        full ? QStringLiteral("等待检测\n完整模式：残缺板将判为失败")
             : QStringLiteral("等待检测\n局部模式：可见子网格也会尽量标出"));
  }
}

/// \brief 把检测台参数写入会话（标定器 / 靶标 / 可视化 / 完整度）
void MainWindow::apply_lab_params_to_session() {
  if (session_ == nullptr || combo_lab_target_ == nullptr) {
    return;
  }
  const QString target = combo_lab_target_->currentText();
  const bool full = is_detect_lab_full_mode();
  session_->set_calibrator_id(
      lab_target_is_trihedral(target) ? QStringLiteral("trihedral_oneshot")
                                      : QStringLiteral("cam_intrinsics"));
  session_->set_board_params(
      spin_lab_squares_x_ != nullptr ? spin_lab_squares_x_->value() : 8,
      spin_lab_squares_y_ != nullptr ? spin_lab_squares_y_->value() : 8,
      spin_lab_square_len_ != nullptr ? spin_lab_square_len_->value() : 0.025);

  std::map<std::string, std::string> opts = session_->solve_options();
  opts["target"] = target.toStdString();
  opts["detect_completeness"] = full ? "full" : "partial";
  if (combo_lab_dictionary_ != nullptr) {
    opts["dictionary"] = combo_lab_dictionary_->currentText().toStdString();
  }
  if (spin_lab_marker_len_ != nullptr) {
    opts["marker_length"] = std::to_string(spin_lab_marker_len_->value());
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
  const QString target = combo_lab_target_->currentText();
  const bool need_dict = lab_target_needs_dictionary(target);
  const bool need_marker = lab_target_needs_marker_len(target);
  if (combo_lab_dictionary_ != nullptr) {
    combo_lab_dictionary_->setEnabled(need_dict);
  }
  if (spin_lab_marker_len_ != nullptr) {
    spin_lab_marker_len_->setEnabled(need_marker);
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
      spin_lab_marker_len_->setValue(0.10);
    }
  } else if (target == QStringLiteral("aruco_grid")) {
    spin_lab_squares_x_->setValue(5);
    spin_lab_squares_y_->setValue(7);
  } else if (target == QStringLiteral("charuco")) {
    spin_lab_squares_x_->setValue(5);
    spin_lab_squares_y_->setValue(7);
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
      lab_stats_->setText(QStringLiteral("检测中…"));
    } else if (session_->has_current_detection()) {
      QString s = QStringLiteral("点数 %1 · conf %2%")
                      .arg(session_->last_point_count())
                      .arg(qRound(session_->last_confidence() * 100.0));
      if (session_->last_aruco_reproj_px() >= 0.0) {
        s += QStringLiteral(" · reproj %1 px")
                 .arg(session_->last_aruco_reproj_px(), 0, 'f', 1);
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
      if (combo_lab_target_ != nullptr &&
          (combo_lab_target_->currentText() == QStringLiteral("aruco") ||
           combo_lab_target_->currentText() == QStringLiteral("aruco_grid") ||
           combo_lab_target_->currentText() == QStringLiteral("charuco") ||
           combo_lab_target_->currentText().contains(QStringLiteral("charuco")))) {
        const int n_markers = session_->last_point_count() / 4;
        if (combo_lab_target_->currentText() == QStringLiteral("aruco") &&
            n_markers > 0) {
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
  if (btn_lab_detect_ != nullptr) {
    btn_lab_detect_->setEnabled(!busy);
  }
  if (btn_lab_detect_fast_ != nullptr) {
    btn_lab_detect_fast_->setEnabled(!busy);
  }
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

/// \brief 运行检测台 Thorough / Fast（完整或局部由模式决定）
void MainWindow::on_lab_detect(bool fast) {
  if (session_ == nullptr) {
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
  if (btn_lab_detect_ != nullptr) {
    btn_lab_detect_->setEnabled(false);
  }
  if (btn_lab_detect_fast_ != nullptr) {
    btn_lab_detect_fast_->setEnabled(false);
  }
  if (lab_stats_ != nullptr) {
    const bool full = is_detect_lab_full_mode();
    lab_stats_->setText(
        fast ? (full ? QStringLiteral("快速完整检测中…")
                     : QStringLiteral("Fast 限时检测中…"))
             : (full ? QStringLiteral("完整检测中…")
                     : QStringLiteral("Thorough 尽量检出中…")));
  }
  session_->request_detect(fast);
}

/// \brief 构建局部 / 完整特征检测调试页（右侧可滚动，避免控件被压扁）
QWidget *MainWindow::build_detect_lab_page() {
  auto *page = new QWidget;
  auto *root = new QHBoxLayout(page);
  root->setContentsMargins(16, 12, 16, 8);
  root->setSpacing(12);

  // —— 左侧预览 ——
  auto *preview_body = new QWidget;
  auto *preview_layout = new QVBoxLayout(preview_body);
  preview_layout->setContentsMargins(0, 0, 0, 0);
  preview_layout->setSpacing(6);
  lab_path_label_ = make_label(QStringLiteral("未加载图片"), QStringLiteral("Muted"), preview_body);
  lab_preview_ = new ImageViewWidget(preview_body);
  lab_preview_->set_placeholder(QStringLiteral("加载目录后显示原图 / 检测叠加"));
  lab_preview_->setMinimumSize(480, 360);
  preview_layout->addWidget(lab_path_label_);
  preview_layout->addWidget(lab_preview_, 1);
  root->addWidget(make_panel(QStringLiteral("检测预览"), preview_body), 1);

  // —— 右侧参数（滚动，防止窗口变矮时 SpinBox 叠在一起）——
  auto *side = new QWidget;
  auto *side_layout = new QVBoxLayout(side);
  side_layout->setContentsMargins(8, 4, 8, 8);
  side_layout->setSpacing(10);
  side->setMinimumWidth(300);

  lab_title_label_ =
      make_label(QStringLiteral("局部特征检测台"), QStringLiteral("PageTitle"), side);
  lab_subtitle_label_ = make_label(
      QStringLiteral(
          "针对棋盘 / ChArUco / Tag 等局部可见内容尽量检出；Thorough 更全，Fast 限时。"),
      QStringLiteral("PageSubtitle"), side);
  lab_subtitle_label_->setWordWrap(true);
  side_layout->addWidget(lab_title_label_);
  side_layout->addWidget(lab_subtitle_label_);

  auto *form_host = new QWidget(side);
  form_host->setObjectName(QStringLiteral("DetectLabForm"));
  auto *form = new QFormLayout(form_host);
  form->setContentsMargins(0, 4, 0, 4);
  form->setHorizontalSpacing(12);
  form->setVerticalSpacing(10);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  form->setRowWrapPolicy(QFormLayout::DontWrapRows);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);

  combo_lab_target_ = new QComboBox(form_host);
  combo_lab_target_->addItems(
      {QStringLiteral("chessboard"), QStringLiteral("charuco"),
       QStringLiteral("aruco"), QStringLiteral("aruco_grid"),
       QStringLiteral("circles_symmetric"), QStringLiteral("circles_asymmetric"),
       QStringLiteral("trihedral_chess"), QStringLiteral("trihedral_charuco")});
  harden_form_field(combo_lab_target_);
  form->addRow(QStringLiteral("target"), combo_lab_target_);

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
  form->addRow(QStringLiteral("dictionary"), combo_lab_dictionary_);

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
  spin_lab_square_len_->setDecimals(4);
  spin_lab_square_len_->setRange(0.001, 1.0);
  spin_lab_square_len_->setSingleStep(0.001);
  spin_lab_square_len_->setValue(0.025);
  harden_form_field(spin_lab_square_len_);
  form->addRow(QStringLiteral("square_length"), spin_lab_square_len_);

  spin_lab_marker_len_ = new QDoubleSpinBox(form_host);
  spin_lab_marker_len_->setDecimals(4);
  spin_lab_marker_len_->setRange(0.001, 1.0);
  spin_lab_marker_len_->setSingleStep(0.001);
  spin_lab_marker_len_->setValue(0.018);
  harden_form_field(spin_lab_marker_len_);
  form->addRow(QStringLiteral("marker_length"), spin_lab_marker_len_);

  side_layout->addWidget(make_panel(QStringLiteral("靶标参数"), form_host));

  auto *dir_body = new QWidget(side);
  auto *dir_layout = new QVBoxLayout(dir_body);
  dir_layout->setContentsMargins(0, 0, 0, 0);
  dir_layout->setSpacing(6);
  edit_lab_image_dir_ = new QLineEdit(dir_body);
  edit_lab_image_dir_->setPlaceholderText(QStringLiteral("离线图片目录…"));
  harden_form_field(edit_lab_image_dir_);
  auto *dir_row = new QHBoxLayout;
  auto *btn_browse = new QPushButton(QStringLiteral("浏览…"), dir_body);
  connect(btn_browse, &QPushButton::clicked, this, &MainWindow::on_lab_browse_images);
  dir_row->addWidget(edit_lab_image_dir_, 1);
  dir_row->addWidget(btn_browse);
  dir_layout->addLayout(dir_row);

  auto *nav_row = new QHBoxLayout;
  btn_lab_prev_ = new QPushButton(QStringLiteral("上一张"), dir_body);
  btn_lab_next_ = new QPushButton(QStringLiteral("下一张"), dir_body);
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
  dir_layout->addLayout(nav_row);
  side_layout->addWidget(make_panel(QStringLiteral("图像源"), dir_body));

  auto *run_body = new QWidget(side);
  auto *run_layout = new QVBoxLayout(run_body);
  run_layout->setContentsMargins(0, 0, 0, 0);
  run_layout->setSpacing(6);
  btn_lab_detect_ = new QPushButton(QStringLiteral("Thorough 尽量检出"), run_body);
  btn_lab_detect_->setObjectName(QStringLiteral("PrimaryButton"));
  btn_lab_detect_fast_ = new QPushButton(QStringLiteral("Fast 限时"), run_body);
  connect(btn_lab_detect_, &QPushButton::clicked, this, [this]() { on_lab_detect(false); });
  connect(btn_lab_detect_fast_, &QPushButton::clicked, this, [this]() {
    on_lab_detect(true);
  });
  run_layout->addWidget(btn_lab_detect_);
  run_layout->addWidget(btn_lab_detect_fast_);
  lab_stats_ = make_label(
      QStringLiteral("等待检测\n局部模式：可见子网格也会尽量标出"),
      QStringLiteral("Muted"), run_body);
  lab_stats_->setWordWrap(true);
  run_layout->addWidget(lab_stats_);
  side_layout->addWidget(make_panel(QStringLiteral("运行"), run_body));

  auto *btn_back = new QPushButton(QStringLiteral("返回首页"), side);
  connect(btn_back, &QPushButton::clicked, this, [this]() { go_to(PageId::Home); });
  side_layout->addWidget(btn_back);
  side_layout->addStretch(1);

  auto *side_scroll = new QScrollArea(page);
  side_scroll->setWidgetResizable(true);
  side_scroll->setFrameShape(QFrame::NoFrame);
  side_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  side_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  side_scroll->setMinimumWidth(320);
  side_scroll->setMaximumWidth(440);
  side_scroll->setWidget(side);
  root->addWidget(side_scroll, 0);

  connect(
      combo_lab_target_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
      [this](int) { sync_lab_target_defaults(); });
  sync_lab_target_defaults();
  refresh_lab_mode_ui();
  return page;
}

}  // namespace gui
}  // namespace hs_calib
