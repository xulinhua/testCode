#include "hs_calib_suite/gui/intrinsics/intrinsics_parameter_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_collector_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_matplotlib_plotter.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_async_plot_controller.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_session.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_dialog_ui.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_param_tooltips.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"

namespace hs_calib {
namespace gui {
namespace {

void add_form_row(QFormLayout *form, const QString &label, QWidget *w) {
  if (label.isEmpty()) {
    if (auto *chk = qobject_cast<QCheckBox *>(w)) {
      apply_intrinsics_param_tooltip(chk->text(), chk);
    }
    form->addRow(w);
    return;
  }
  auto *field_label = make_field_label(label, form->parentWidget());
  apply_intrinsics_param_tooltip(label, field_label);
  apply_intrinsics_param_tooltip(label, w);
  form->addRow(field_label, w);
}

QString subtitle_for_kind(IntrinsicsParameterDialog::Kind kind) {
  switch (kind) {
    case IntrinsicsParameterDialog::Kind::Calibration:
      return QStringLiteral("标定流水线参数（RANSAC / 子采样 / 畸变 / Ceres / OpenCV）");
    case IntrinsicsParameterDialog::Kind::Collector:
      return QStringLiteral("采集过滤与训练/评估分流（Tier4 §10）");
    case IntrinsicsParameterDialog::Kind::Detector:
      return QStringLiteral("按靶标类型配置检测器");
  }
  return {};
}

QString title_for_kind(IntrinsicsParameterDialog::Kind kind) {
  switch (kind) {
    case IntrinsicsParameterDialog::Kind::Calibration:
      return QStringLiteral("标定参数");
    case IntrinsicsParameterDialog::Kind::Collector:
      return QStringLiteral("采集参数");
    case IntrinsicsParameterDialog::Kind::Detector:
      return QStringLiteral("检测器参数");
  }
  return QStringLiteral("参数");
}

}  // namespace

IntrinsicsParameterDialog::IntrinsicsParameterDialog(Kind kind, QWidget *parent)
    : QDialog(parent), kind_(kind) {
  auto chrome = setup_intrinsics_dialog(
      this, title_for_kind(kind_), subtitle_for_kind(kind_), 520);
  scroll_ = chrome.scroll;

  switch (kind_) {
    case Kind::Calibration:
      build_calibration_ui(chrome.content);
      break;
    case Kind::Collector:
      build_collector_ui(chrome.content);
      break;
    case Kind::Detector:
      build_detector_ui(chrome.content);
      break;
  }
  chrome.content_layout->addStretch(1);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("应用"), [this]() { apply_to_session(); });

  if (kind_ == Kind::Calibration && combo_profile_ != nullptr) {
    apply_intrinsics_param_tooltip(QStringLiteral("intrinsics_profile"), combo_profile_);
    apply_intrinsics_param_tooltip(QStringLiteral("intrinsics_solver"), combo_solver_);
    connect(
        combo_profile_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int) {
          if (session_ == nullptr || combo_profile_ == nullptr) {
            return;
          }
          auto opts = session_->solve_options();
          core::apply_tier4_profile_bundle(
              combo_profile_->currentData().toString().toStdString(), &opts);
          session_->set_solve_options(opts);
          load_from_session();
        });
  }
}

void IntrinsicsParameterDialog::build_calibration_ui(QWidget *host) {
  auto *lay = qobject_cast<QVBoxLayout *>(host->layout());
  if (lay == nullptr) {
    lay = new QVBoxLayout(host);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(14);
  }
  {
    auto *g = make_param_group(QStringLiteral("预设 / 求解器"), host);
    auto *f = new_param_form(g);
    combo_profile_ = new QComboBox(g);
    combo_profile_->addItem(QStringLiteral("General"), QStringLiteral("general"));
    combo_profile_->addItem(QStringLiteral("C1"), QStringLiteral("c1"));
    combo_profile_->addItem(QStringLiteral("Ceres"), QStringLiteral("ceres"));
    combo_profile_->addItem(QStringLiteral("C2"), QStringLiteral("c2"));
    combo_solver_ = new QComboBox(g);
    combo_solver_->addItem(QStringLiteral("OpenCV"), QStringLiteral("opencv"));
    combo_solver_->addItem(QStringLiteral("Ceres"), QStringLiteral("ceres"));
    add_form_row(f, QStringLiteral("intrinsics_profile"), combo_profile_);
    add_form_row(f, QStringLiteral("intrinsics_solver"), combo_solver_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("RANSAC 预剔除"), host);
    auto *f = new_param_form(g);
    chk_ransac_ = new QCheckBox(QStringLiteral("use_ransac_pre_rejection"), g);
    spin_pre_iter_ = new QSpinBox(g);
    spin_pre_iter_->setRange(1, 500);
    spin_pre_min_hyp_ = new QSpinBox(g);
    spin_pre_min_hyp_->setRange(3, 20);
    spin_pre_max_rms_ = new QDoubleSpinBox(g);
    spin_pre_max_rms_->setRange(0.05, 5.0);
    spin_pre_max_rms_->setSingleStep(0.05);
    add_form_row(f, QString(), chk_ransac_);
    add_form_row(f, QStringLiteral("pre_rejection_iterations"), spin_pre_iter_);
    add_form_row(f, QStringLiteral("pre_rejection_min_hypotheses"), spin_pre_min_hyp_);
    add_form_row(f, QStringLiteral("pre_rejection_max_rms_error"), spin_pre_max_rms_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("熵子采样"), host);
    auto *f = new_param_form(g);
    spin_max_cal_samples_ = new QSpinBox(g);
    spin_max_cal_samples_->setRange(10, 500);
    chk_entropy_ = new QCheckBox(QStringLiteral("use_entropy_maximization_subsampling"), g);
    spin_sub_cells_ = new QSpinBox(g);
    spin_sub_cells_->setRange(4, 32);
    spin_sub_tilt_res_ = new QDoubleSpinBox(g);
    spin_sub_tilt_res_->setRange(5.0, 45.0);
    spin_sub_max_tilt_ = new QDoubleSpinBox(g);
    spin_sub_max_tilt_->setRange(10.0, 80.0);
    add_form_row(f, QStringLiteral("max_calibration_samples"), spin_max_cal_samples_);
    add_form_row(f, QString(), chk_entropy_);
    add_form_row(f, QStringLiteral("subsampling_pixel_cells"), spin_sub_cells_);
    add_form_row(f, QStringLiteral("subsampling_tilt_resolution"), spin_sub_tilt_res_);
    add_form_row(f, QStringLiteral("subsampling_max_tilt_deg"), spin_sub_max_tilt_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("后剔除"), host);
    auto *f = new_param_form(g);
    chk_post_ = new QCheckBox(QStringLiteral("use_post_rejection"), g);
    spin_post_rms_ = new QDoubleSpinBox(g);
    spin_post_rms_->setRange(0.05, 5.0);
    spin_post_rms_->setSingleStep(0.05);
    add_form_row(f, QString(), chk_post_);
    add_form_row(f, QStringLiteral("post_rejection_max_rms_error"), spin_post_rms_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("统计可视化"), host);
    auto *f = new_param_form(g);
    chk_plot_data_ = new QCheckBox(QStringLiteral("plot_calibration_data_statistics"), g);
    chk_plot_results_ = new QCheckBox(QStringLiteral("plot_calibration_results_statistics"), g);
    spin_viz_cells_ = new QSpinBox(g);
    spin_viz_cells_->setRange(4, 32);
    spin_viz_tilt_res_ = new QDoubleSpinBox(g);
    spin_viz_max_tilt_ = new QDoubleSpinBox(g);
    spin_viz_z_ = new QSpinBox(g);
    spin_viz_z_->setRange(4, 32);
    add_form_row(f, QString(), chk_plot_data_);
    add_form_row(f, QString(), chk_plot_results_);
    add_form_row(f, QStringLiteral("viz_pixel_cells"), spin_viz_cells_);
    add_form_row(f, QStringLiteral("viz_tilt_resolution"), spin_viz_tilt_res_);
    add_form_row(f, QStringLiteral("viz_max_tilt_deg"), spin_viz_max_tilt_);
    add_form_row(f, QStringLiteral("viz_z_cells"), spin_viz_z_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("畸变模型"), host);
    auto *f = new_param_form(g);
    spin_radial_ = new QSpinBox(g);
    spin_radial_->setRange(0, 3);
    spin_rational_ = new QSpinBox(g);
    spin_rational_->setRange(0, 3);
    chk_tangential_ = new QCheckBox(QStringLiteral("use_tangential_distortion"), g);
    add_form_row(f, QStringLiteral("radial_distortion_coefficients"), spin_radial_);
    add_form_row(f, QStringLiteral("rational_distortion_coefficients"), spin_rational_);
    add_form_row(f, QString(), chk_tangential_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("Ceres"), host);
    auto *f = new_param_form(g);
    spin_pre_cal_num_ = new QSpinBox(g);
    spin_pre_cal_num_->setRange(5, 200);
    spin_coeff_reg_ = new QDoubleSpinBox(g);
    spin_coeff_reg_->setRange(0.0, 2.0);
    spin_coeff_reg_->setSingleStep(0.05);
    spin_fov_reg_ = new QDoubleSpinBox(g);
    spin_fov_reg_->setRange(0.0, 2.0);
    spin_fov_reg_->setSingleStep(0.05);
    add_form_row(f, QStringLiteral("pre_calibration_num_samples"), spin_pre_cal_num_);
    add_form_row(f, QStringLiteral("coeffs_regularization_weight"), spin_coeff_reg_);
    add_form_row(f, QStringLiteral("fov_regularization_weight"), spin_fov_reg_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("OpenCV"), host);
    auto *f = new_param_form(g);
    chk_prism_ = new QCheckBox(QStringLiteral("enable_prism_model"), g);
    chk_fix_pp_ = new QCheckBox(QStringLiteral("fix_principal_point"), g);
    chk_fix_aspect_ = new QCheckBox(QStringLiteral("fix_aspect_ratio"), g);
    chk_lu_ = new QCheckBox(QStringLiteral("use_lu_decomposition"), g);
    chk_qr_ = new QCheckBox(QStringLiteral("use_qr_decomposition"), g);
    add_form_row(f, QString(), chk_prism_);
    add_form_row(f, QString(), chk_fix_pp_);
    add_form_row(f, QString(), chk_fix_aspect_);
    add_form_row(f, QString(), chk_lu_);
    add_form_row(f, QString(), chk_qr_);
    lay->addWidget(g);
  }
  {
    auto *g = make_param_group(QStringLiteral("采集重投影过滤"), host);
    auto *f = new_param_form(g);
    chk_filter_reproj_cal_ = new QCheckBox(QStringLiteral("filter_by_reprojection_error"), g);
    spin_capture_max_ = new QDoubleSpinBox(g);
    spin_capture_rms_ = new QDoubleSpinBox(g);
    add_form_row(f, QString(), chk_filter_reproj_cal_);
    add_form_row(f, QStringLiteral("max_allowed_max_reprojection_error"), spin_capture_max_);
    add_form_row(f, QStringLiteral("max_allowed_rms_reprojection_error"), spin_capture_rms_);
    lay->addWidget(g);
  }
}

void IntrinsicsParameterDialog::build_collector_ui(QWidget *host) {
  auto *lay = qobject_cast<QVBoxLayout *>(host->layout());
  if (lay == nullptr) {
    lay = new QVBoxLayout(host);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(14);
  }
  auto *g = make_param_group(QStringLiteral("Data collection §10"), host);
  auto *f = new_param_form(g);
  spin_max_samples_ = new QSpinBox(g);
  spin_max_samples_->setRange(10, 2000);
  spin_decorrelate_ = new QSpinBox(g);
  spin_decorrelate_->setRange(1, 20);
  spin_max_tilt_ = new QDoubleSpinBox(g);
  chk_filter_speed_ = new QCheckBox(QStringLiteral("filter_by_speed"), g);
  spin_max_pixel_speed_ = new QDoubleSpinBox(g);
  spin_max_speed_ = new QDoubleSpinBox(g);
  spin_max_speed_->setDecimals(3);
  chk_filter_reproj_ = new QCheckBox(QStringLiteral("filter_by_reprojection_error"), g);
  spin_reproj_max_ = new QDoubleSpinBox(g);
  spin_reproj_rms_ = new QDoubleSpinBox(g);
  chk_filter_2d_ = new QCheckBox(QStringLiteral("filter_by_2d_redundancy"), g);
  spin_min_center_ = new QDoubleSpinBox(g);
  spin_min_skew_ = new QDoubleSpinBox(g);
  spin_min_size_ = new QDoubleSpinBox(g);
  chk_filter_3d_ = new QCheckBox(QStringLiteral("filter_by_3d_redundancy"), g);
  spin_min_3d_center_ = new QDoubleSpinBox(g);
  spin_min_tilt_diff_ = new QDoubleSpinBox(g);
  spin_heatmap_cells_ = new QSpinBox(g);
  spin_rot_angle_res_ = new QSpinBox(g);
  spin_hist_2d_ = new QSpinBox(g);
  spin_hist_3d_ = new QSpinBox(g);
  chk_skip_no_det_ = new QCheckBox(QStringLiteral("skip_frames_when_not_detection"), g);
  spin_max_fast_ = new QSpinBox(g);
  add_form_row(f, QStringLiteral("collector_max_samples"), spin_max_samples_);
  add_form_row(f, QStringLiteral("decorrelate_eval_samples"), spin_decorrelate_);
  add_form_row(f, QStringLiteral("collector_max_tilt"), spin_max_tilt_);
  add_form_row(f, QString(), chk_filter_speed_);
  add_form_row(f, QStringLiteral("collector_max_pixel_speed"), spin_max_pixel_speed_);
  add_form_row(f, QStringLiteral("collector_max_speed"), spin_max_speed_);
  add_form_row(f, QString(), chk_filter_reproj_);
  add_form_row(f, QStringLiteral("max_allowed_max_reprojection_error"), spin_reproj_max_);
  add_form_row(f, QStringLiteral("max_allowed_rms_reprojection_error"), spin_reproj_rms_);
  add_form_row(f, QString(), chk_filter_2d_);
  add_form_row(f, QStringLiteral("collector_min_center_diff"), spin_min_center_);
  add_form_row(f, QStringLiteral("collector_min_skew_diff"), spin_min_skew_);
  add_form_row(f, QStringLiteral("collector_min_size_diff"), spin_min_size_);
  add_form_row(f, QString(), chk_filter_3d_);
  add_form_row(f, QStringLiteral("collector_min_3d_center_m"), spin_min_3d_center_);
  add_form_row(f, QStringLiteral("collector_min_tilt_diff_deg"), spin_min_tilt_diff_);
  add_form_row(f, QStringLiteral("collector_heatmap_cells"), spin_heatmap_cells_);
  add_form_row(f, QStringLiteral("rotation_heatmap_angle_res"), spin_rot_angle_res_);
  add_form_row(f, QStringLiteral("collector_point_2d_hist_bins"), spin_hist_2d_);
  add_form_row(f, QStringLiteral("collector_point_3d_hist_bins"), spin_hist_3d_);
  add_form_row(f, QString(), chk_skip_no_det_);
  add_form_row(f, QStringLiteral("max_fast_calibration_samples"), spin_max_fast_);
  lay->addWidget(g);
}

void IntrinsicsParameterDialog::build_detector_ui(QWidget *host) {
  auto *lay = qobject_cast<QVBoxLayout *>(host->layout());
  if (lay == nullptr) {
    lay = new QVBoxLayout(host);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(14);
  }
  lab_detector_target_ = new QLabel(host);
  lab_detector_target_->setObjectName(QStringLiteral("PageSubtitle"));
  lab_detector_target_->setToolTip(
      QStringLiteral("当前靶标类型由启动配置中的 target 决定；下方显示对应检测器参数页。"));
  lay->addWidget(lab_detector_target_);
  detector_stack_ = new QStackedWidget(host);
  detector_stack_->setObjectName(QStringLiteral("IntrinsicsDetectorStack"));

  {
    auto *page = new QWidget(detector_stack_);
    auto *page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    auto *g = make_param_group(QStringLiteral("Chessboard"), page);
    auto *f = new_param_form(g);
    chk_cb_adaptive_ = new QCheckBox(QStringLiteral("cb_adaptive"), page);
    chk_cb_normalize_ = new QCheckBox(QStringLiteral("cb_normalize"), page);
    chk_cb_fast_ = new QCheckBox(QStringLiteral("cb_fast_check"), page);
    chk_cb_resized_ = new QCheckBox(QStringLiteral("cb_resized_detection"), page);
    spin_cb_res_max_ = new QSpinBox(page);
    spin_cb_res_max_->setRange(500, 3000);
    chk_cb_subpix_ = new QCheckBox(QStringLiteral("cb_sub_pixel_refinement"), page);
    spin_cb_max_lost_ = new QSpinBox(page);
    spin_cb_max_lost_->setRange(0, 15);
    spin_cb_padding_ = new QSpinBox(page);
    spin_cb_padding_->setRange(10, 500);
    add_form_row(f, QString(), chk_cb_adaptive_);
    add_form_row(f, QString(), chk_cb_normalize_);
    add_form_row(f, QString(), chk_cb_fast_);
    add_form_row(f, QString(), chk_cb_resized_);
    add_form_row(f, QStringLiteral("cb_resized_max_resolution"), spin_cb_res_max_);
    add_form_row(f, QString(), chk_cb_subpix_);
    add_form_row(f, QStringLiteral("cb_max_lost_frames"), spin_cb_max_lost_);
    add_form_row(f, QStringLiteral("cb_padding"), spin_cb_padding_);
    page_lay->addWidget(g);
    page_lay->addStretch(1);
    detector_stack_->addWidget(page);
  }
  {
    auto *page = new QWidget(detector_stack_);
    auto *page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    auto *g = make_param_group(QStringLiteral("Dot board"), page);
    auto *f = new_param_form(g);
    chk_dot_sym_ = new QCheckBox(QStringLiteral("dot_symmetric_grid"), page);
    chk_dot_cluster_ = new QCheckBox(QStringLiteral("dot_clustering"), page);
    chk_dot_filter_area_ = new QCheckBox(QStringLiteral("dot_filter_by_area"), page);
    spin_dot_min_area_ = new QDoubleSpinBox(page);
    spin_dot_max_area_ = new QDoubleSpinBox(page);
    spin_dot_min_dist_ = new QDoubleSpinBox(page);
    chk_dot_resized_ = new QCheckBox(QStringLiteral("dot_resized_detection"), page);
    spin_dot_res_max_ = new QSpinBox(page);
    add_form_row(f, QString(), chk_dot_sym_);
    add_form_row(f, QString(), chk_dot_cluster_);
    add_form_row(f, QString(), chk_dot_filter_area_);
    add_form_row(f, QStringLiteral("dot_min_area_percentage"), spin_dot_min_area_);
    add_form_row(f, QStringLiteral("dot_max_area_percentage"), spin_dot_max_area_);
    add_form_row(f, QStringLiteral("dot_min_dist_between_blobs_percentage"), spin_dot_min_dist_);
    add_form_row(f, QString(), chk_dot_resized_);
    add_form_row(f, QStringLiteral("dot_resized_max_resolution"), spin_dot_res_max_);
    page_lay->addWidget(g);
    page_lay->addStretch(1);
    detector_stack_->addWidget(page);
  }
  {
    auto *page = new QWidget(detector_stack_);
    auto *page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    auto *g = make_param_group(QStringLiteral("AprilTag grid"), page);
    auto *f = new_param_form(g);
    spin_april_threads_ = new QSpinBox(page);
    spin_april_decimate_ = new QSpinBox(page);
    spin_april_sigma_ = new QDoubleSpinBox(page);
    chk_april_refine_ = new QCheckBox(QStringLiteral("april_refine_edges"), page);
    spin_april_sharpen_ = new QDoubleSpinBox(page);
    chk_april_debug_ = new QCheckBox(QStringLiteral("april_debug"), page);
    spin_april_hamming_ = new QSpinBox(page);
    spin_april_margin_ = new QDoubleSpinBox(page);
    spin_april_ratio_ = new QDoubleSpinBox(page);
    add_form_row(f, QStringLiteral("april_nthreads"), spin_april_threads_);
    add_form_row(f, QStringLiteral("april_quad_decimate"), spin_april_decimate_);
    add_form_row(f, QStringLiteral("april_quad_sigma"), spin_april_sigma_);
    add_form_row(f, QString(), chk_april_refine_);
    add_form_row(f, QStringLiteral("april_decode_sharpening"), spin_april_sharpen_);
    add_form_row(f, QString(), chk_april_debug_);
    add_form_row(f, QStringLiteral("april_max_hamming_error"), spin_april_hamming_);
    add_form_row(f, QStringLiteral("april_min_margin"), spin_april_margin_);
    add_form_row(f, QStringLiteral("april_min_detection_ratio"), spin_april_ratio_);
    page_lay->addWidget(g);
    page_lay->addStretch(1);
    detector_stack_->addWidget(page);
  }
  {
    auto *page = new QWidget(detector_stack_);
    auto *page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    auto *g = make_param_group(QStringLiteral("ChArUco"), page);
    auto *f = new_param_form(g);
    spin_charuco_win_min_ = new QSpinBox(page);
    spin_charuco_win_max_ = new QSpinBox(page);
    spin_charuco_marker_len_ = new QDoubleSpinBox(page);
    spin_charuco_marker_len_->setDecimals(4);
    add_form_row(f, QStringLiteral("charuco_adaptive_win_min"), spin_charuco_win_min_);
    add_form_row(f, QStringLiteral("charuco_adaptive_win_max"), spin_charuco_win_max_);
    add_form_row(f, QStringLiteral("charuco_marker_length"), spin_charuco_marker_len_);
    page_lay->addWidget(g);
    page_lay->addStretch(1);
    detector_stack_->addWidget(page);
  }
  lay->addWidget(detector_stack_);
}

void IntrinsicsParameterDialog::refresh_detector_page() {
  if (session_ == nullptr || detector_stack_ == nullptr) {
    return;
  }
  const std::string target =
      core::detector_target_from_config(session_->solve_options());
  lab_detector_target_->setText(
      QStringLiteral("target: %1").arg(QString::fromStdString(target)));
  int page = 0;
  if (target.find("circle") != std::string::npos) {
    page = 1;
  } else if (target == "aprilgrid") {
    page = 2;
  } else if (target == "charuco") {
    page = 3;
  }
  detector_stack_->setCurrentIndex(page);
}

void IntrinsicsParameterDialog::bind_session(SessionController *session) {
  session_ = session;
  load_from_session();
}

void IntrinsicsParameterDialog::load_from_session() {
  if (session_ == nullptr) {
    return;
  }
  const auto opts = session_->solve_options();
  if (kind_ == Kind::Calibration) {
    const auto profile = core::profile_from_config_map(opts);
    const auto extras = core::calibration_extras_from_config(opts);
    const int pidx = combo_profile_->findData(QString::fromStdString(profile.id));
    if (pidx >= 0) {
      combo_profile_->setCurrentIndex(pidx);
    }
    const QString solver =
        profile.solver == core::IntrinsicsSolverKind::Ceres ? QStringLiteral("ceres")
                                                          : QStringLiteral("opencv");
    const int sidx = combo_solver_->findData(solver);
    if (sidx >= 0) {
      combo_solver_->setCurrentIndex(sidx);
    }
    chk_ransac_->setChecked(profile.use_ransac_pre_rejection);
    spin_pre_iter_->setValue(profile.pre_rejection_iterations);
    spin_pre_min_hyp_->setValue(profile.pre_rejection_min_hypotheses);
    spin_pre_max_rms_->setValue(profile.pre_rejection_max_rms_error);
    spin_max_cal_samples_->setValue(profile.max_calibration_samples);
    chk_entropy_->setChecked(profile.use_entropy_subsampling);
    spin_sub_cells_->setValue(profile.subsampling_pixel_cells);
    spin_sub_tilt_res_->setValue(profile.subsampling_tilt_resolution_deg);
    spin_sub_max_tilt_->setValue(profile.subsampling_max_tilt_deg);
    chk_post_->setChecked(profile.use_post_rejection);
    spin_post_rms_->setValue(profile.post_rejection_max_rms_error);
    chk_plot_data_->setChecked(extras.plot_calibration_data_statistics);
    chk_plot_results_->setChecked(extras.plot_calibration_results_statistics);
    spin_viz_cells_->setValue(extras.viz_pixel_cells);
    spin_viz_tilt_res_->setValue(extras.viz_tilt_resolution_deg);
    spin_viz_max_tilt_->setValue(extras.viz_max_tilt_deg);
    spin_viz_z_->setValue(extras.viz_z_cells);
    spin_radial_->setValue(profile.radial_coeffs);
    spin_rational_->setValue(profile.rational_coeffs);
    chk_tangential_->setChecked(profile.use_tangential);
    spin_pre_cal_num_->setValue(extras.pre_calibration_num_samples);
    spin_coeff_reg_->setValue(profile.coeffs_regularization_weight);
    spin_fov_reg_->setValue(profile.fov_regularization_weight);
    chk_prism_->setChecked(extras.enable_prism_model);
    chk_fix_pp_->setChecked(extras.fix_principal_point);
    chk_fix_aspect_->setChecked(extras.fix_aspect_ratio);
    chk_lu_->setChecked(extras.use_lu_decomposition);
    chk_qr_->setChecked(extras.use_qr_decomposition);
    chk_filter_reproj_cal_->setChecked(profile.filter_capture_by_reproj);
    spin_capture_max_->setValue(profile.capture_max_reproj_error);
    spin_capture_rms_->setValue(profile.capture_max_rms_reproj_error);
  } else if (kind_ == Kind::Collector) {
    const auto profile = core::profile_from_config_map(opts);
    const auto p = core::collector_params_from_config(opts, profile);
    spin_max_samples_->setValue(p.max_samples);
    spin_decorrelate_->setValue(p.decorrelate_eval_samples);
    spin_max_tilt_->setValue(p.max_allowed_tilt_deg);
    chk_filter_speed_->setChecked(p.filter_by_speed);
    spin_max_pixel_speed_->setValue(p.max_allowed_pixel_speed);
    spin_max_speed_->setValue(p.max_allowed_speed);
    chk_filter_reproj_->setChecked(p.filter_by_reprojection_error);
    spin_reproj_max_->setValue(p.max_allowed_max_reprojection_error);
    spin_reproj_rms_->setValue(p.max_allowed_rms_reprojection_error);
    chk_filter_2d_->setChecked(p.filter_by_2d_redundancy);
    spin_min_center_->setValue(p.min_normalized_2d_center_difference);
    spin_min_skew_->setValue(p.min_normalized_skew_difference);
    spin_min_size_->setValue(p.min_normalized_2d_size_difference);
    chk_filter_3d_->setChecked(p.filter_by_3d_redundancy);
    spin_min_3d_center_->setValue(p.min_3d_center_difference_m);
    spin_min_tilt_diff_->setValue(p.min_tilt_difference_deg);
    spin_heatmap_cells_->setValue(p.heatmap_cells);
    spin_rot_angle_res_->setValue(p.rotation_heatmap_angle_res);
    spin_hist_2d_->setValue(p.point_2d_hist_bins);
    spin_hist_3d_->setValue(p.point_3d_hist_bins);
    chk_skip_no_det_->setChecked(p.skip_frames_when_not_detection);
    spin_max_fast_->setValue(p.max_fast_calibration_samples);
  } else if (kind_ == Kind::Detector) {
    refresh_detector_page();
    const auto chess = core::chess_detector_from_config(opts);
    chk_cb_adaptive_->setChecked(chess.adaptive_thresh);
    chk_cb_normalize_->setChecked(chess.normalize_image);
    chk_cb_fast_->setChecked(chess.fast_check);
    chk_cb_resized_->setChecked(chess.resized_detection);
    spin_cb_res_max_->setValue(chess.resized_max_resolution);
    chk_cb_subpix_->setChecked(chess.sub_pixel_refinement);
    spin_cb_max_lost_->setValue(chess.max_lost_frames);
    spin_cb_padding_->setValue(chess.padding);
    const auto dot = core::dot_detector_from_config(opts);
    chk_dot_sym_->setChecked(dot.symmetric_grid);
    chk_dot_cluster_->setChecked(dot.clustering);
    chk_dot_filter_area_->setChecked(dot.filter_by_area);
    spin_dot_min_area_->setValue(dot.min_area_percentage);
    spin_dot_max_area_->setValue(dot.max_area_percentage);
    spin_dot_min_dist_->setValue(dot.min_dist_between_blobs_percentage);
    chk_dot_resized_->setChecked(dot.resized_detection);
    spin_dot_res_max_->setValue(dot.resized_max_resolution);
    const auto april = core::aprilgrid_detector_from_config(opts);
    spin_april_threads_->setValue(april.nthreads);
    spin_april_decimate_->setValue(april.quad_decimate);
    spin_april_sigma_->setValue(april.quad_sigma);
    chk_april_refine_->setChecked(april.refine_edges);
    spin_april_sharpen_->setValue(april.decode_sharpening);
    chk_april_debug_->setChecked(april.debug);
    spin_april_hamming_->setValue(april.max_hamming_error);
    spin_april_margin_->setValue(april.min_margin);
    spin_april_ratio_->setValue(april.min_detection_ratio);
    const auto ch = core::charuco_detector_from_config(opts);
    spin_charuco_win_min_->setValue(ch.adaptive_thresh_win_size_min);
    spin_charuco_win_max_->setValue(ch.adaptive_thresh_win_size_max);
    spin_charuco_marker_len_->setValue(ch.marker_length_m);
  }
}

void IntrinsicsParameterDialog::apply_to_session() {
  if (session_ == nullptr) {
    return;
  }
  auto opts = session_->solve_options();
  if (kind_ == Kind::Calibration) {
    core::IntrinsicsProfile profile =
        core::intrinsics_profile_from_id(combo_profile_->currentData().toString().toStdString());
    profile.solver = combo_solver_->currentData().toString() == "ceres"
                         ? core::IntrinsicsSolverKind::Ceres
                         : core::IntrinsicsSolverKind::OpenCV;
    profile.use_ransac_pre_rejection = chk_ransac_->isChecked();
    profile.pre_rejection_iterations = spin_pre_iter_->value();
    profile.pre_rejection_min_hypotheses = spin_pre_min_hyp_->value();
    profile.pre_rejection_max_rms_error = spin_pre_max_rms_->value();
    profile.max_calibration_samples = spin_max_cal_samples_->value();
    profile.use_entropy_subsampling = chk_entropy_->isChecked();
    profile.subsampling_pixel_cells = spin_sub_cells_->value();
    profile.subsampling_tilt_resolution_deg = spin_sub_tilt_res_->value();
    profile.subsampling_max_tilt_deg = spin_sub_max_tilt_->value();
    profile.use_post_rejection = chk_post_->isChecked();
    profile.post_rejection_max_rms_error = spin_post_rms_->value();
    profile.radial_coeffs = spin_radial_->value();
    profile.rational_coeffs = spin_rational_->value();
    profile.use_tangential = chk_tangential_->isChecked();
    profile.coeffs_regularization_weight = spin_coeff_reg_->value();
    profile.fov_regularization_weight = spin_fov_reg_->value();
    profile.filter_capture_by_reproj = chk_filter_reproj_cal_->isChecked();
    profile.capture_max_reproj_error = spin_capture_max_->value();
    profile.capture_max_rms_reproj_error = spin_capture_rms_->value();
    core::IntrinsicsCalibrationExtras extras;
    extras.plot_calibration_data_statistics = chk_plot_data_->isChecked();
    extras.plot_calibration_results_statistics = chk_plot_results_->isChecked();
    extras.viz_pixel_cells = spin_viz_cells_->value();
    extras.viz_tilt_resolution_deg = spin_viz_tilt_res_->value();
    extras.viz_max_tilt_deg = spin_viz_max_tilt_->value();
    extras.viz_z_cells = spin_viz_z_->value();
    extras.pre_calibration_num_samples = spin_pre_cal_num_->value();
    extras.enable_prism_model = chk_prism_->isChecked();
    extras.fix_principal_point = chk_fix_pp_->isChecked();
    extras.fix_aspect_ratio = chk_fix_aspect_->isChecked();
    extras.use_lu_decomposition = chk_lu_->isChecked();
    extras.use_qr_decomposition = chk_qr_->isChecked();
    core::apply_calibration_to_config(profile, extras, &opts);
  } else if (kind_ == Kind::Collector) {
    core::IntrinsicsCollectorParams p;
    p.max_samples = spin_max_samples_->value();
    p.decorrelate_eval_samples = spin_decorrelate_->value();
    p.max_allowed_tilt_deg = spin_max_tilt_->value();
    p.filter_by_speed = chk_filter_speed_->isChecked();
    p.max_allowed_pixel_speed = spin_max_pixel_speed_->value();
    p.max_allowed_speed = spin_max_speed_->value();
    p.filter_by_reprojection_error = chk_filter_reproj_->isChecked();
    p.max_allowed_max_reprojection_error = spin_reproj_max_->value();
    p.max_allowed_rms_reprojection_error = spin_reproj_rms_->value();
    p.filter_by_2d_redundancy = chk_filter_2d_->isChecked();
    p.min_normalized_2d_center_difference = spin_min_center_->value();
    p.min_normalized_skew_difference = spin_min_skew_->value();
    p.min_normalized_2d_size_difference = spin_min_size_->value();
    p.filter_by_3d_redundancy = chk_filter_3d_->isChecked();
    p.min_3d_center_difference_m = spin_min_3d_center_->value();
    p.min_tilt_difference_deg = spin_min_tilt_diff_->value();
    p.heatmap_cells = spin_heatmap_cells_->value();
    p.rotation_heatmap_angle_res = spin_rot_angle_res_->value();
    p.point_2d_hist_bins = spin_hist_2d_->value();
    p.point_3d_hist_bins = spin_hist_3d_->value();
    p.skip_frames_when_not_detection = chk_skip_no_det_->isChecked();
    p.max_fast_calibration_samples = spin_max_fast_->value();
    core::apply_collector_to_config(p, &opts);
  } else if (kind_ == Kind::Detector) {
    core::ChessDetectorParams chess;
    chess.adaptive_thresh = chk_cb_adaptive_->isChecked();
    chess.normalize_image = chk_cb_normalize_->isChecked();
    chess.fast_check = chk_cb_fast_->isChecked();
    chess.resized_detection = chk_cb_resized_->isChecked();
    chess.resized_max_resolution = spin_cb_res_max_->value();
    chess.sub_pixel_refinement = chk_cb_subpix_->isChecked();
    chess.max_lost_frames = spin_cb_max_lost_->value();
    chess.padding = spin_cb_padding_->value();
    core::apply_chess_detector_to_config(chess, &opts);
    core::DotDetectorParams dot;
    dot.symmetric_grid = chk_dot_sym_->isChecked();
    dot.clustering = chk_dot_cluster_->isChecked();
    dot.filter_by_area = chk_dot_filter_area_->isChecked();
    dot.min_area_percentage = spin_dot_min_area_->value();
    dot.max_area_percentage = spin_dot_max_area_->value();
    dot.min_dist_between_blobs_percentage = spin_dot_min_dist_->value();
    dot.resized_detection = chk_dot_resized_->isChecked();
    dot.resized_max_resolution = spin_dot_res_max_->value();
    core::apply_dot_detector_to_config(dot, &opts);
    core::AprilgridDetectorParams april;
    april.nthreads = spin_april_threads_->value();
    april.quad_decimate = spin_april_decimate_->value();
    april.quad_sigma = spin_april_sigma_->value();
    april.refine_edges = chk_april_refine_->isChecked();
    april.decode_sharpening = spin_april_sharpen_->value();
    april.debug = chk_april_debug_->isChecked();
    april.max_hamming_error = spin_april_hamming_->value();
    april.min_detection_ratio = spin_april_ratio_->value();
    april.min_margin = spin_april_margin_->value();
    core::apply_aprilgrid_detector_to_config(april, &opts);
    core::CharucoDetectorParams ch;
    ch.adaptive_thresh_win_size_min = spin_charuco_win_min_->value();
    ch.adaptive_thresh_win_size_max = spin_charuco_win_max_->value();
    ch.marker_length_m = spin_charuco_marker_len_->value();
    core::apply_charuco_detector_to_config(ch, &opts);
  }
  session_->set_solve_options(opts);
}

IntrinsicsStatsDialog::IntrinsicsStatsDialog(QWidget *parent) : QDialog(parent) {
  auto chrome = setup_intrinsics_dialog(
      this,
      QStringLiteral("Calibration data statistics"),
      QStringLiteral("Tier4 采集数据五阶段分布：像素/旋转热力图与 z 直方图"),
      920);

  auto *metrics_row = new QHBoxLayout;
  metrics_row->setSpacing(10);
  auto train = make_stat_tile(QStringLiteral("训练样本"), chrome.content);
  auto eval = make_stat_tile(QStringLiteral("评估样本"), chrome.content);
  auto heat = make_stat_tile(QStringLiteral("热力图格"), chrome.content);
  lbl_train_tile_ = train.value;
  lbl_eval_tile_ = eval.value;
  lbl_heatmap_tile_ = heat.value;
  metrics_row->addWidget(train.frame, 1);
  metrics_row->addWidget(eval.frame, 1);
  metrics_row->addWidget(heat.frame, 1);
  chrome.content_layout->addLayout(metrics_row);

  auto *backend_frame = new QFrame(chrome.content);
  backend_frame->setObjectName(QStringLiteral("Panel"));
  auto *backend_lay = new QHBoxLayout(backend_frame);
  backend_lay->setContentsMargins(14, 10, 14, 10);
  backend_lay->addWidget(make_field_label(QStringLiteral("统计后端"), backend_frame));
  lbl_backend_ = make_value_label(QStringLiteral("—"), backend_frame);
  lbl_backend_->setToolTip(QStringLiteral("采集统计图渲染后端：Qt 轻量图或 matplotlib 脚本。"));
  backend_lay->addWidget(lbl_backend_, 1);
  chrome.content_layout->addWidget(backend_frame);

  auto *plot_frame = new QFrame(chrome.content);
  plot_frame->setObjectName(QStringLiteral("Panel"));
  auto *plot_lay = new QVBoxLayout(plot_frame);
  plot_lay->setContentsMargins(12, 12, 12, 12);
  plot_lay->setSpacing(6);
  auto *plot_hint = new QLabel(
      QStringLiteral("滚轮缩放 · 拖拽平移 · 双击适应窗口"), plot_frame);
  plot_hint->setObjectName(QStringLiteral("Muted"));
  plot_view_ = new ImageViewWidget(plot_frame);
  plot_view_->setMinimumHeight(300);
  plot_view_->set_background_color(QColor(255, 255, 255));
  plot_view_->set_toolbar_style(ImageViewToolbarStyle::OverlayZoomSave);
  plot_view_->set_placeholder(QStringLiteral("暂无统计图"));
  plot_lay->addWidget(plot_hint, 0);
  plot_lay->addWidget(plot_view_, 1);
  chrome.content_layout->addWidget(plot_frame, 1);

  summary_label_ = new QLabel(chrome.content);
  summary_label_->setObjectName(QStringLiteral("Muted"));
  summary_label_->setWordWrap(true);
  chrome.content_layout->addWidget(summary_label_);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("关闭"), [this]() { close(); });

  plot_loader_ = new IntrinsicsAsyncPlotController(this);
  connect(
      plot_loader_, &IntrinsicsAsyncPlotController::plot_started, this,
      [this]() {
        lbl_backend_->setText(QStringLiteral("处理中…"));
        summary_label_->setText(
            QStringLiteral("正在后台处理（流水线求解 + matplotlib 绘图），请稍候…"));
        plot_view_->clear_image();
        plot_view_->set_placeholder(QStringLiteral("正在生成统计图，请稍候…"));
      });
  connect(
      plot_loader_, &IntrinsicsAsyncPlotController::plot_finished, this,
      [this](
          bool ok, const QImage &image, const QString &summary,
          const QString &backend_label, bool matplotlib_failed) {
        if (ok) {
          summary_label_->clear();
          lbl_backend_->setText(backend_label);
          plot_view_->set_image(image);
          plot_view_->fit_to_window();
          return;
        }
        if (matplotlib_failed && last_session_ != nullptr &&
            last_backend_ == "qt") {
          const auto &col = last_session_->intrinsics_state().collector();
          QPixmap pm = IntrinsicsStatsPlotter::render_qt_summary(col);
          if (!pm.isNull()) {
            lbl_backend_->setText(QStringLiteral("qt"));
            plot_view_->set_image(pm.toImage());
            plot_view_->fit_to_window();
            summary_label_->setText(
                summary.isEmpty()
                    ? QStringLiteral("matplotlib 不可用，已回退 Qt 简图")
                    : QStringLiteral("matplotlib 回退 Qt：%1").arg(summary));
            return;
          }
        }
        lbl_backend_->setText(QStringLiteral("—"));
        summary_label_->setText(
            summary.isEmpty() ? QStringLiteral("统计图生成失败") : summary);
        plot_view_->clear_image();
        plot_view_->set_placeholder(
            summary.isEmpty() ? QStringLiteral("暂无统计图") : summary);
      });
}

void IntrinsicsStatsDialog::refresh(
    const SessionController *session, const std::string &backend) {
  last_session_ = session;
  last_backend_ = backend;
  if (session == nullptr || !session->is_intrinsics()) {
    lbl_backend_->setText(QStringLiteral("—"));
    lbl_train_tile_->setText(QStringLiteral("—"));
    lbl_eval_tile_->setText(QStringLiteral("—"));
    lbl_heatmap_tile_->setText(QStringLiteral("—"));
    summary_label_->setText(QStringLiteral("无内参采集数据"));
    plot_view_->clear_image();
    plot_view_->set_placeholder(QStringLiteral("暂无统计图"));
    return;
  }
  const auto &col = session->intrinsics_state().collector();
  lbl_train_tile_->setText(QStringLiteral("%1 帧 · %2%")
                               .arg(col.training_count())
                               .arg(col.training_occupancy_percent(), 0, 'f', 1));
  lbl_eval_tile_->setText(QStringLiteral("%1 帧 · %2%")
                              .arg(col.evaluation_count())
                              .arg(col.evaluation_occupancy_percent(), 0, 'f', 1));
  lbl_heatmap_tile_->setText(QStringLiteral("%1 × %1").arg(col.params().heatmap_cells));
  summary_label_->clear();

  if (col.training_count() < 3) {
    lbl_backend_->setText(QStringLiteral("—"));
    summary_label_->setText(QStringLiteral("训练样本不足（至少 3 帧），无法生成统计图"));
    plot_view_->clear_image();
    plot_view_->set_placeholder(QStringLiteral("暂无统计图"));
    return;
  }

  plot_loader_->start(
      session, IntrinsicsPlotKind::CollectionDataStatistics, backend);
}

}  // namespace gui
}  // namespace hs_calib
