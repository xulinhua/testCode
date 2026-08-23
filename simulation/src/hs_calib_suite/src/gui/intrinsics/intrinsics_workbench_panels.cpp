#include "hs_calib_suite/gui/intrinsics/intrinsics_workbench_panels.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_session_state.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_dialog_ui.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

namespace hs_calib {
namespace gui {
namespace {

void add_group_title(QVBoxLayout *lay, const QString &title, QWidget *owner) {
  auto *l = new QLabel(title, owner);
  l->setObjectName(QStringLiteral("SectionTitle"));
  lay->addWidget(l);
}

}  // namespace

IntrinsicsMetricsStrip::IntrinsicsMetricsStrip(QWidget *parent) : QWidget(parent) {
  auto *row = new QHBoxLayout(this);
  row->setContentsMargins(0, 4, 0, 4);
  row->setSpacing(8);
  lbl_summary_ = new QLabel(QStringLiteral("检测：—"), this);
  lbl_summary_->setObjectName(QStringLiteral("Muted"));
  lbl_summary_->setWordWrap(true);
  row->addWidget(lbl_summary_, 1);
  btn_details_ = new QPushButton(QStringLiteral("检测详情…"), this);
  btn_details_->setObjectName(QStringLiteral("CompactButton"));
  connect(btn_details_, &QPushButton::clicked, this,
          &IntrinsicsMetricsStrip::details_requested);
  row->addWidget(btn_details_, 0);
}

void IntrinsicsMetricsStrip::refresh(const SessionController *session) {
  if (session == nullptr || !session->is_intrinsics()) {
    lbl_summary_->setText(QStringLiteral("检测：—"));
    return;
  }
  if (session->uses_stereo_dual_session()) {
  QString line;
  if (session->stereo_left_has_detection() && session->stereo_right_has_detection()) {
    line = QStringLiteral("L/R 已检出 · Δt %1ms")
               .arg(session->last_stereo_sync_delta_ms());
  } else if (session->stereo_left_has_detection() || session->stereo_right_has_detection()) {
    line = QStringLiteral("L:%1 R:%2")
               .arg(session->stereo_left_has_detection() ? QStringLiteral("✓")
                                                         : QStringLiteral("—"))
               .arg(session->stereo_right_has_detection() ? QStringLiteral("✓")
                                                          : QStringLiteral("—"));
  } else {
    line = QStringLiteral("检测：未检出");
  }
  if (session->uses_tier4_intrinsics()) {
    const auto &lcol = session->intrinsics_state_for_side(QStringLiteral("left")).collector();
    const auto &rcol = session->intrinsics_state_for_side(QStringLiteral("right")).collector();
    line += QStringLiteral(" · 训练 L%1/R%2")
                .arg(lcol.training_count())
                .arg(rcol.training_count());
  }
  lbl_summary_->setText(line);
  return;
  }
  if (!session->has_current_detection()) {
    lbl_summary_->setText(QStringLiteral("检测：未检出"));
    return;
  }
  const auto &m = session->last_board_metrics();
  if (!m.detected) {
    lbl_summary_->setText(
        QStringLiteral("检测：是 · 置信 %1%")
            .arg(qRound(session->last_confidence() * 100.0)));
    return;
  }
  QString line = QStringLiteral("检测：是 · 倾角 %1° · 面积 %2% · skew %3")
                     .arg(m.rough_tilt_deg, 0, 'f', 1)
                     .arg(m.relative_area_percent, 0, 'f', 0)
                     .arg(m.normalized_skew, 0, 'f', 2);
  if (m.has_reprojection) {
    line += QStringLiteral(" · RMS %1 px").arg(m.reproj_rms_px, 0, 'f', 2);
  }
  lbl_summary_->setText(line);
}

IntrinsicsDetectionDetailsDialog::IntrinsicsDetectionDetailsDialog(QWidget *parent)
    : QDialog(parent) {
  auto chrome = setup_intrinsics_dialog(
      this,
      QStringLiteral("检测详情"),
      QStringLiteral("当前帧几何质量与单帧重投影（Tier4 Detection / Single-shot）"),
      540);

  auto *banner = new QFrame(chrome.content);
  banner->setObjectName(QStringLiteral("Panel"));
  auto *banner_lay = new QHBoxLayout(banner);
  banner_lay->setContentsMargins(14, 10, 14, 10);
  lbl_detected_ = make_value_label(QStringLiteral("—"), banner);
  banner_lay->addWidget(make_field_label(QStringLiteral("检测状态"), banner));
  banner_lay->addWidget(lbl_detected_, 1);
  chrome.content_layout->addWidget(banner);

  {
    auto *g = make_param_group(QStringLiteral("Detection results"), chrome.content);
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Rough tilt"), &lbl_tilt_, g);
    add_metric_row(f, QStringLiteral("Rough angles"), &lbl_angles_, g);
    add_metric_row(f, QStringLiteral("Rough position"), &lbl_position_, g);
    add_metric_row(f, QStringLiteral("Skew"), &lbl_skew_, g);
    add_metric_row(f, QStringLiteral("Relative area"), &lbl_area_, g);
    add_metric_row(f, QStringLiteral("Linear error"), &lbl_linear_, g);
    add_metric_row(f, QStringLiteral("Aspect ratio"), &lbl_aspect_, g);
    chrome.content_layout->addWidget(g);
  }

  {
    auto *g = make_param_group(QStringLiteral("Single-shot reprojection"), chrome.content);
    reproj_section_ = g;
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Max"), &lbl_reproj_max_, g);
    add_metric_row(f, QStringLiteral("Avg"), &lbl_reproj_avg_, g);
    add_metric_row(f, QStringLiteral("RMS"), &lbl_reproj_rms_, g);
    chrome.content_layout->addWidget(g);
  }
  chrome.content_layout->addStretch(1);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("关闭"), [this]() { close(); });
}

void IntrinsicsDetectionDetailsDialog::refresh(const SessionController *session) {
  if (session == nullptr || !session->is_intrinsics()) {
    lbl_detected_->setText(QStringLiteral("无内参会话"));
    return;
  }
  const auto &m = session->last_board_metrics();
  lbl_detected_->setText(m.detected ? QStringLiteral("已检出") : QStringLiteral("未检出"));
  if (!m.detected) {
    lbl_tilt_->setText(QStringLiteral("—"));
    reproj_section_->setVisible(false);
    return;
  }
  reproj_section_->setVisible(true);
  lbl_tilt_->setText(QStringLiteral("%1°").arg(m.rough_tilt_deg, 0, 'f', 1));
  lbl_angles_->setText(
      QStringLiteral("x = %1°\ny = %2°")
          .arg(m.rough_angle_x_deg, 0, 'f', 1)
          .arg(m.rough_angle_y_deg, 0, 'f', 1));
  lbl_position_->setText(
      QStringLiteral("x = %1 m\ny = %2 m\nz = %3 m")
          .arg(m.rough_position_x_m, 0, 'f', 3)
          .arg(m.rough_position_y_m, 0, 'f', 3)
          .arg(m.rough_position_z_m, 0, 'f', 3));
  lbl_angles_->setMinimumWidth(220);
  lbl_position_->setMinimumWidth(220);
  lbl_skew_->setText(QString::number(m.normalized_skew, 'f', 3));
  lbl_area_->setText(QStringLiteral("%1 %").arg(m.relative_area_percent, 0, 'f', 1));
  lbl_linear_->setText(QStringLiteral("row %1 px  ·  col %2 px")
                           .arg(m.linear_error_rows_rms_px, 0, 'f', 2)
                           .arg(m.linear_error_cols_rms_px, 0, 'f', 2));
  lbl_aspect_->setText(QString::number(m.aspect_ratio, 'f', 3));
  if (m.has_reprojection) {
    lbl_reproj_max_->setText(QStringLiteral("%1 px  (%2%)")
                                 .arg(m.reproj_max_px, 0, 'f', 2)
                                 .arg(m.reproj_max_relative_percent, 0, 'f', 1));
    lbl_reproj_avg_->setText(QStringLiteral("%1 px  (%2%)")
                                 .arg(m.reproj_avg_px, 0, 'f', 2)
                                 .arg(m.reproj_avg_relative_percent, 0, 'f', 1));
    lbl_reproj_rms_->setText(QStringLiteral("%1 px  (%2%)")
                                 .arg(m.reproj_rms_px, 0, 'f', 2)
                                 .arg(m.reproj_rms_relative_percent, 0, 'f', 1));
  } else {
    lbl_reproj_max_->setText(QStringLiteral("等待 partial calib"));
    lbl_reproj_avg_->setText(QStringLiteral("—"));
    lbl_reproj_rms_->setText(QStringLiteral("—"));
  }
}

IntrinsicsVizOptionsDialog::IntrinsicsVizOptionsDialog(QWidget *parent) : QDialog(parent) {
  auto chrome = setup_intrinsics_dialog(
      this,
      QStringLiteral("Visualization options"),
      QStringLiteral("Tier4 预览叠加：检测、历史像点、占用/线性度热力图与指示条"),
      520);

  {
    auto *g = make_param_group(QStringLiteral("Draw overlays"), chrome.content);
    auto *f = new_param_form(g);
    chk_draw_detection_ = new QCheckBox(QStringLiteral("Draw detection"), g);
    chk_draw_detection_->setToolTip(
        QStringLiteral("叠加当前帧格点与检测连线（关闭时仅显示原图底图）"));
    chk_draw_train_pts_ = new QCheckBox(QStringLiteral("Draw training points"), g);
    chk_draw_train_pts_->setToolTip(QStringLiteral("叠加全部训练集像点"));
    chk_draw_eval_pts_ = new QCheckBox(QStringLiteral("Draw evaluation points"), g);
    chk_draw_eval_pts_->setToolTip(QStringLiteral("叠加评估集像点"));
    chk_draw_train_occ_ = new QCheckBox(QStringLiteral("Draw training occupancy"), g);
    chk_draw_train_occ_->setToolTip(QStringLiteral("训练集像面格占用热力图"));
    chk_draw_eval_occ_ = new QCheckBox(QStringLiteral("Draw evaluation occupancy"), g);
    chk_draw_eval_occ_->setToolTip(QStringLiteral("评估集像面格占用热力图"));
    chk_draw_linearity_ = new QCheckBox(QStringLiteral("Draw linearity error"), g);
    chk_draw_linearity_->setToolTip(QStringLiteral("格点线性度误差累积热力图"));
    chk_draw_indicators_ = new QCheckBox(QStringLiteral("Draw indicators"), g);
    chk_draw_indicators_->setToolTip(
        QStringLiteral("左侧 skew / area / RMS 指示条（与 Drawings 视图模式一致）"));
    f->addRow(chk_draw_detection_);
    f->addRow(chk_draw_train_pts_);
    f->addRow(chk_draw_eval_pts_);
    f->addRow(chk_draw_train_occ_);
    f->addRow(chk_draw_eval_occ_);
    f->addRow(chk_draw_linearity_);
    f->addRow(chk_draw_indicators_);
    chrome.content_layout->addWidget(g);
  }

  {
    auto *g = make_param_group(QStringLiteral("Opacity"), chrome.content);
    auto *f = new_param_form(g);
    spin_drawings_alpha_ = new QDoubleSpinBox(g);
    spin_drawings_alpha_->setRange(0.05, 1.0);
    spin_drawings_alpha_->setSingleStep(0.05);
    spin_drawings_alpha_->setDecimals(2);
    spin_drawings_alpha_->setToolTip(QStringLiteral("热力图与历史像点叠加透明度"));
    spin_indicators_alpha_ = new QDoubleSpinBox(g);
    spin_indicators_alpha_->setRange(0.05, 1.0);
    spin_indicators_alpha_->setSingleStep(0.05);
    spin_indicators_alpha_->setDecimals(2);
    spin_indicators_alpha_->setToolTip(QStringLiteral("指示条叠加透明度"));
    f->addRow(QStringLiteral("Drawings alpha"), spin_drawings_alpha_);
    f->addRow(QStringLiteral("Indicators alpha"), spin_indicators_alpha_);
    chrome.content_layout->addWidget(g);
  }

  btn_clear_linearity_ =
      new QPushButton(QStringLiteral("Clear heatmap linearity"), chrome.content);
  btn_clear_linearity_->setObjectName(QStringLiteral("CompactButton"));
  btn_clear_linearity_->setToolTip(QStringLiteral("重置线性度误差累积热力图"));
  chrome.content_layout->addWidget(btn_clear_linearity_);
  chrome.content_layout->addStretch(1);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("关闭"), [this]() { close(); });
  wire_live_apply();
}

void IntrinsicsVizOptionsDialog::bind_session(SessionController *session) {
  session_ = session;
  load_from_session();
}

void IntrinsicsVizOptionsDialog::load_from_session() {
  if (session_ == nullptr) {
    return;
  }
  const IntrinsicsVizOptions opts = session_->intrinsics_viz_options();
  const QSignalBlocker b1(chk_draw_detection_);
  const QSignalBlocker b2(chk_draw_train_pts_);
  const QSignalBlocker b3(chk_draw_eval_pts_);
  const QSignalBlocker b4(chk_draw_train_occ_);
  const QSignalBlocker b5(chk_draw_eval_occ_);
  const QSignalBlocker b6(chk_draw_linearity_);
  const QSignalBlocker b7(chk_draw_indicators_);
  const QSignalBlocker b8(spin_drawings_alpha_);
  const QSignalBlocker b9(spin_indicators_alpha_);
  chk_draw_detection_->setChecked(opts.draw_detection);
  chk_draw_train_pts_->setChecked(opts.draw_training_points);
  chk_draw_eval_pts_->setChecked(opts.draw_evaluation_points);
  chk_draw_train_occ_->setChecked(opts.draw_training_occupancy);
  chk_draw_eval_occ_->setChecked(opts.draw_evaluation_occupancy);
  chk_draw_linearity_->setChecked(opts.draw_linearity_error);
  chk_draw_indicators_->setChecked(opts.draw_indicators);
  spin_drawings_alpha_->setValue(opts.drawings_alpha);
  spin_indicators_alpha_->setValue(opts.indicators_alpha);
}

void IntrinsicsVizOptionsDialog::apply_to_session() {
  if (session_ == nullptr) {
    return;
  }
  IntrinsicsVizOptions opts = session_->intrinsics_viz_options();
  opts.draw_detection = chk_draw_detection_->isChecked();
  opts.draw_training_points = chk_draw_train_pts_->isChecked();
  opts.draw_evaluation_points = chk_draw_eval_pts_->isChecked();
  opts.draw_training_occupancy = chk_draw_train_occ_->isChecked();
  opts.draw_evaluation_occupancy = chk_draw_eval_occ_->isChecked();
  opts.draw_linearity_error = chk_draw_linearity_->isChecked();
  opts.draw_indicators = chk_draw_indicators_->isChecked();
  opts.drawings_alpha = static_cast<float>(spin_drawings_alpha_->value());
  opts.indicators_alpha = static_cast<float>(spin_indicators_alpha_->value());
  session_->set_intrinsics_viz_options(opts);
  const int radius = session_->viz_marker_radius();
  session_->set_viz_options(
      opts.draw_detection, opts.draw_detection, opts.draw_detection, radius,
      opts.draw_detection);
  emit options_applied();
}

void IntrinsicsVizOptionsDialog::wire_live_apply() {
  const auto apply = [this]() { apply_to_session(); };
  connect(chk_draw_detection_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_train_pts_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_eval_pts_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_train_occ_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_eval_occ_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_linearity_, &QCheckBox::toggled, this, apply);
  connect(chk_draw_indicators_, &QCheckBox::toggled, this, apply);
  connect(
      spin_drawings_alpha_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      apply);
  connect(
      spin_indicators_alpha_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      apply);
  connect(btn_clear_linearity_, &QPushButton::clicked, this, [this, apply]() {
    if (session_ != nullptr) {
      session_->clear_intrinsics_linearity_heatmap();
    }
    apply();
  });
}

IntrinsicsCalibrationStatusDialog::IntrinsicsCalibrationStatusDialog(QWidget *parent)
    : QDialog(parent) {
  auto chrome = setup_intrinsics_dialog(
      this,
      QStringLiteral("标定状态"),
      QStringLiteral("训练/评估样本数、inlier 与重投影 RMS"),
      480);

  {
    auto *g = make_param_group(QStringLiteral("Calibration control"), chrome.content);
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Status"), &lbl_status_, g);
    add_metric_row(f, QStringLiteral("Calibration time"), &lbl_time_, g);
    chrome.content_layout->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("Training"), chrome.content);
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Samples"), &lbl_train_, g);
    add_metric_row(f, QStringLiteral("Pre inliers"), &lbl_pre_, g);
    add_metric_row(f, QStringLiteral("Post inliers"), &lbl_post_, g);
    add_metric_row(f, QStringLiteral("RMS (all)"), &lbl_train_rms_, g);
    add_metric_row(f, QStringLiteral("RMS (inlier)"), &lbl_train_rms_inlier_, g);
    add_metric_row(f, QStringLiteral("Occupancy"), &lbl_train_occ_, g);
    chrome.content_layout->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("Evaluation"), chrome.content);
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Samples"), &lbl_eval_, g);
    add_metric_row(f, QStringLiteral("Post inliers"), &lbl_eval_inliers_, g);
    add_metric_row(f, QStringLiteral("RMS (all)"), &lbl_eval_rms_, g);
    add_metric_row(f, QStringLiteral("RMS (inlier)"), &lbl_eval_rms_inlier_, g);
    add_metric_row(f, QStringLiteral("Occupancy"), &lbl_eval_occ_, g);
    chrome.content_layout->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("Last frame"), chrome.content);
    auto *f = new_param_form(g);
    add_metric_row(f, QStringLiteral("Detection"), &lbl_last_det_, g);
    chrome.content_layout->addWidget(g);
  }
  chrome.content_layout->addStretch(1);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("关闭"), [this]() { close(); });
}

void IntrinsicsCalibrationStatusDialog::refresh(const SessionController *session) {
  if (session == nullptr || !session->is_intrinsics()) {
    lbl_status_->setText(QStringLiteral("—"));
    return;
  }
  if (session->uses_stereo_dual_session()) {
    const auto &lst = session->intrinsics_state_for_side(QStringLiteral("left")).stats();
    const auto &rst = session->intrinsics_state_for_side(QStringLiteral("right")).stats();
    const auto &lcol = session->intrinsics_state_for_side(QStringLiteral("left")).collector();
    const auto &rcol = session->intrinsics_state_for_side(QStringLiteral("right")).collector();
    lbl_status_->setText(
        (lst.calibration_time_sec > 0 || rst.calibration_time_sec > 0)
            ? QStringLiteral("done")
            : QStringLiteral("idle"));
    lbl_time_->setText(
        lst.calibration_time_sec > 0
            ? QStringLiteral("L %1s / R %2s")
                  .arg(lst.calibration_time_sec, 0, 'f', 2)
                  .arg(rst.calibration_time_sec, 0, 'f', 2)
            : QStringLiteral("—"));
    lbl_train_->setText(
        QStringLiteral("L %1 / R %2")
            .arg(lcol.training_count())
            .arg(rcol.training_count()));
    lbl_eval_->setText(
        QStringLiteral("L %1 / R %2")
            .arg(lcol.evaluation_count())
            .arg(rcol.evaluation_count()));
    lbl_train_rms_->setText(
        QStringLiteral("L %1 / R %2")
            .arg(lst.training_rms_all >= 0 ? QString::number(lst.training_rms_all, 'f', 3)
                                            : QStringLiteral("—"))
            .arg(rst.training_rms_all >= 0 ? QString::number(rst.training_rms_all, 'f', 3)
                                           : QStringLiteral("—")));
    lbl_last_det_->setText(
        QStringLiteral("成对 %1 · Δt %2ms")
            .arg(session->stereo_pair_count())
            .arg(session->last_stereo_sync_delta_ms()));
    return;
  }
  const auto &st = session->intrinsics_state().stats();
  const auto &col = session->intrinsics_state().collector();
  lbl_status_->setText(st.calibration_time_sec > 0 ? QStringLiteral("done")
                                                   : QStringLiteral("idle"));
  lbl_time_->setText(st.calibration_time_sec > 0
                         ? QStringLiteral("%1 s").arg(st.calibration_time_sec, 0, 'f', 2)
                         : QStringLiteral("—"));
  lbl_train_->setText(QString::number(col.training_count()));
  lbl_pre_->setText(st.pre_rejection_inliers > 0
                        ? QString::number(st.pre_rejection_inliers)
                        : QStringLiteral("—"));
  lbl_post_->setText(st.post_rejection_inliers > 0
                         ? QString::number(st.post_rejection_inliers)
                         : QStringLiteral("—"));
  lbl_train_rms_->setText(st.training_rms_all >= 0
                              ? QStringLiteral("%1 px").arg(st.training_rms_all, 0, 'f', 3)
                              : QStringLiteral("—"));
  lbl_train_rms_inlier_->setText(st.training_rms_inlier >= 0
                                     ? QStringLiteral("%1 px")
                                           .arg(st.training_rms_inlier, 0, 'f', 3)
                                     : QStringLiteral("—"));
  lbl_train_occ_->setText(QStringLiteral("%1 %")
                              .arg(col.training_occupancy_percent(), 0, 'f', 0));
  lbl_eval_->setText(QString::number(col.evaluation_count()));
  lbl_eval_inliers_->setText(st.eval_post_rejection_inliers > 0
                                 ? QString::number(st.eval_post_rejection_inliers)
                                 : QStringLiteral("—"));
  lbl_eval_rms_->setText(st.evaluation_rms_all >= 0
                             ? QStringLiteral("%1 px").arg(st.evaluation_rms_all, 0, 'f', 3)
                             : QStringLiteral("—"));
  lbl_eval_rms_inlier_->setText(st.evaluation_rms_inlier >= 0
                                      ? QStringLiteral("%1 px")
                                            .arg(st.evaluation_rms_inlier, 0, 'f', 3)
                                      : QStringLiteral("—"));
  lbl_eval_occ_->setText(QStringLiteral("%1 %")
                             .arg(col.evaluation_occupancy_percent(), 0, 'f', 0));
  const auto &m = session->last_board_metrics();
  lbl_last_det_->setText(
      m.detected ? QStringLiteral("skew %1 · area %2 %")
                       .arg(m.normalized_skew, 0, 'f', 2)
                       .arg(m.relative_area_percent, 0, 'f', 0)
                 : QStringLiteral("—"));
}

IntrinsicsControlRail::IntrinsicsControlRail(QWidget *parent) : QWidget(parent) {
  setMinimumWidth(280);
  setMaximumWidth(400);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);

  auto *scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto *body = new QWidget(scroll);
  auto *lay = new QVBoxLayout(body);
  lay->setContentsMargins(4, 0, 4, 0);
  lay->setSpacing(10);

  add_group_title(lay, QStringLiteral("Solver selection"), body);
  combo_solver_ = new QComboBox(body);
  combo_solver_->setMinimumHeight(34);
  combo_solver_->addItem(QStringLiteral("OpenCV"), QStringLiteral("opencv"));
  combo_solver_->addItem(QStringLiteral("Ceres"), QStringLiteral("ceres"));
  lay->addWidget(combo_solver_);

  add_group_title(lay, QStringLiteral("Calibration control"), body);
  auto *btn_cal_params = new QPushButton(QStringLiteral("标定参数…"), body);
  btn_cal_params->setObjectName(QStringLiteral("CompactButton"));
  btn_cal_params->setMinimumHeight(34);
  auto *btn_status = new QPushButton(QStringLiteral("标定状态…"), body);
  btn_status->setObjectName(QStringLiteral("CompactButton"));
  btn_status->setMinimumHeight(34);
  lay->addWidget(btn_cal_params);
  lay->addWidget(btn_status);

  lbl_compact_status_ = new QLabel(body);
  lbl_compact_status_->setObjectName(QStringLiteral("Muted"));
  lbl_compact_status_->setWordWrap(true);
  lay->addWidget(lbl_compact_status_);

  add_group_title(lay, QStringLiteral("Detection options"), body);
  auto *btn_det_params = new QPushButton(QStringLiteral("检测器参数…"), body);
  btn_det_params->setObjectName(QStringLiteral("CompactButton"));
  btn_det_params->setMinimumHeight(34);
  lay->addWidget(btn_det_params);

  add_group_title(lay, QStringLiteral("Data collection"), body);
  lbl_compact_collect_ = new QLabel(body);
  lbl_compact_collect_->setObjectName(QStringLiteral("Muted"));
  lbl_compact_collect_->setWordWrap(true);
  lay->addWidget(lbl_compact_collect_);
  auto *btn_stats = new QPushButton(QStringLiteral("采集统计…"), body);
  btn_stats->setObjectName(QStringLiteral("CompactButton"));
  btn_stats->setMinimumHeight(34);
  auto *btn_col_params = new QPushButton(QStringLiteral("采集参数…"), body);
  btn_col_params->setObjectName(QStringLiteral("CompactButton"));
  btn_col_params->setMinimumHeight(34);
  lay->addWidget(btn_stats);
  lay->addWidget(btn_col_params);
  lay->addStretch(1);

  scroll->setWidget(body);
  outer->addWidget(scroll);

  connect(btn_cal_params, &QPushButton::clicked, this,
          &IntrinsicsControlRail::calibration_params_requested);
  connect(btn_det_params, &QPushButton::clicked, this,
          &IntrinsicsControlRail::detector_params_requested);
  connect(btn_col_params, &QPushButton::clicked, this,
          &IntrinsicsControlRail::collector_params_requested);
  connect(btn_stats, &QPushButton::clicked, this,
          &IntrinsicsControlRail::statistics_requested);
  connect(btn_status, &QPushButton::clicked, this,
          &IntrinsicsControlRail::status_details_requested);
}

void IntrinsicsControlRail::sync_from_session() {
  if (session_ == nullptr || combo_solver_ == nullptr) {
    return;
  }
  const int sidx = combo_solver_->findData(
      QString::fromStdString(session_->intrinsics_solver_kind()));
  if (sidx >= 0) {
    QSignalBlocker block(combo_solver_);
    combo_solver_->setCurrentIndex(sidx);
  }
}

void IntrinsicsControlRail::set_session(SessionController *session) {
  session_ = session;
  sync_from_session();
}

void IntrinsicsControlRail::refresh() {
  if (session_ == nullptr || !session_->uses_tier4_intrinsics()) {
    return;
  }
  if (session_->uses_stereo_dual_session()) {
    const auto &lst = session_->intrinsics_state_for_side(QStringLiteral("left")).stats();
    const auto &rst = session_->intrinsics_state_for_side(QStringLiteral("right")).stats();
    const auto &lcol = session_->intrinsics_state_for_side(QStringLiteral("left")).collector();
    const auto &rcol = session_->intrinsics_state_for_side(QStringLiteral("right")).collector();
    lbl_compact_status_->setText(
        QStringLiteral("左 训%1 评%2 · 右 训%3 评%4")
            .arg(lcol.training_count())
            .arg(lcol.evaluation_count())
            .arg(rcol.training_count())
            .arg(rcol.evaluation_count()));
    const QString lrms =
        lst.training_rms_all >= 0
            ? QStringLiteral("%1").arg(lst.training_rms_all, 0, 'f', 2)
            : QStringLiteral("—");
    const QString rrms =
        rst.training_rms_all >= 0
            ? QStringLiteral("%1").arg(rst.training_rms_all, 0, 'f', 2)
            : QStringLiteral("—");
    lbl_compact_collect_->setText(
        QStringLiteral("RMS L %1 / R %2 px · 占用 L%3%/R%4%")
            .arg(lrms, rrms)
            .arg(lcol.training_occupancy_percent(), 0, 'f', 0)
            .arg(rcol.training_occupancy_percent(), 0, 'f', 0));
    return;
  }
  const auto &st = session_->intrinsics_state().stats();
  const auto &col = session_->intrinsics_state().collector();
  lbl_compact_status_->setText(
      QStringLiteral("训练 %1 · 评估 %2 · RMS %3")
          .arg(col.training_count())
          .arg(col.evaluation_count())
          .arg(st.training_rms_all >= 0
                   ? QStringLiteral("%1 px").arg(st.training_rms_all, 0, 'f', 2)
                   : QStringLiteral("—")));
  lbl_compact_collect_->setText(
      QStringLiteral("占用 训练 %1% · 评估 %2%")
          .arg(col.training_occupancy_percent(), 0, 'f', 0)
          .arg(col.evaluation_occupancy_percent(), 0, 'f', 0));
}

}  // namespace gui
}  // namespace hs_calib
