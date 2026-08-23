#pragma once

#include <map>
#include <string>

#include <QDialog>
#include <QLabel>
#include <QStackedWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QScrollArea;
class QSpinBox;

namespace hs_calib {
namespace gui {

class SessionController;
class ImageViewWidget;

/// \brief Tier4 风格非模态参数弹窗（标定 / 采集 / 检测，YAML 全字段）
class IntrinsicsParameterDialog : public QDialog {
  Q_OBJECT

public:
  enum class Kind { Calibration, Collector, Detector };

  explicit IntrinsicsParameterDialog(Kind kind, QWidget *parent = nullptr);

  void bind_session(SessionController *session);
  void load_from_session();
  void apply_to_session();

private:
  void build_calibration_ui(QWidget *host);
  void build_collector_ui(QWidget *host);
  void build_detector_ui(QWidget *host);
  void refresh_detector_page();

  Kind kind_;
  SessionController *session_ = nullptr;
  QScrollArea *scroll_ = nullptr;

  // —— 标定 §8 ——
  QComboBox *combo_profile_ = nullptr;
  QComboBox *combo_solver_ = nullptr;
  QCheckBox *chk_ransac_ = nullptr;
  QSpinBox *spin_pre_iter_ = nullptr;
  QSpinBox *spin_pre_min_hyp_ = nullptr;
  QDoubleSpinBox *spin_pre_max_rms_ = nullptr;
  QSpinBox *spin_max_cal_samples_ = nullptr;
  QCheckBox *chk_entropy_ = nullptr;
  QSpinBox *spin_sub_cells_ = nullptr;
  QDoubleSpinBox *spin_sub_tilt_res_ = nullptr;
  QDoubleSpinBox *spin_sub_max_tilt_ = nullptr;
  QCheckBox *chk_post_ = nullptr;
  QDoubleSpinBox *spin_post_rms_ = nullptr;
  QCheckBox *chk_plot_data_ = nullptr;
  QCheckBox *chk_plot_results_ = nullptr;
  QSpinBox *spin_viz_cells_ = nullptr;
  QDoubleSpinBox *spin_viz_tilt_res_ = nullptr;
  QDoubleSpinBox *spin_viz_max_tilt_ = nullptr;
  QSpinBox *spin_viz_z_ = nullptr;
  QSpinBox *spin_radial_ = nullptr;
  QSpinBox *spin_rational_ = nullptr;
  QCheckBox *chk_tangential_ = nullptr;
  QSpinBox *spin_pre_cal_num_ = nullptr;
  QDoubleSpinBox *spin_coeff_reg_ = nullptr;
  QDoubleSpinBox *spin_fov_reg_ = nullptr;
  QCheckBox *chk_prism_ = nullptr;
  QCheckBox *chk_fix_pp_ = nullptr;
  QCheckBox *chk_fix_aspect_ = nullptr;
  QCheckBox *chk_lu_ = nullptr;
  QCheckBox *chk_qr_ = nullptr;
  QCheckBox *chk_filter_reproj_cal_ = nullptr;
  QDoubleSpinBox *spin_capture_max_ = nullptr;
  QDoubleSpinBox *spin_capture_rms_ = nullptr;

  // —— 采集 §10 ——
  QSpinBox *spin_max_samples_ = nullptr;
  QSpinBox *spin_decorrelate_ = nullptr;
  QDoubleSpinBox *spin_max_tilt_ = nullptr;
  QCheckBox *chk_filter_speed_ = nullptr;
  QDoubleSpinBox *spin_max_pixel_speed_ = nullptr;
  QDoubleSpinBox *spin_max_speed_ = nullptr;
  QCheckBox *chk_filter_reproj_ = nullptr;
  QDoubleSpinBox *spin_reproj_max_ = nullptr;
  QDoubleSpinBox *spin_reproj_rms_ = nullptr;
  QCheckBox *chk_filter_2d_ = nullptr;
  QDoubleSpinBox *spin_min_center_ = nullptr;
  QDoubleSpinBox *spin_min_skew_ = nullptr;
  QDoubleSpinBox *spin_min_size_ = nullptr;
  QCheckBox *chk_filter_3d_ = nullptr;
  QDoubleSpinBox *spin_min_3d_center_ = nullptr;
  QDoubleSpinBox *spin_min_tilt_diff_ = nullptr;
  QSpinBox *spin_heatmap_cells_ = nullptr;
  QSpinBox *spin_rot_angle_res_ = nullptr;
  QSpinBox *spin_hist_2d_ = nullptr;
  QSpinBox *spin_hist_3d_ = nullptr;
  QCheckBox *chk_skip_no_det_ = nullptr;
  QSpinBox *spin_max_fast_ = nullptr;

  // —— 检测 §9 ——
  QLabel *lab_detector_target_ = nullptr;
  QStackedWidget *detector_stack_ = nullptr;
  // chess
  QCheckBox *chk_cb_adaptive_ = nullptr;
  QCheckBox *chk_cb_normalize_ = nullptr;
  QCheckBox *chk_cb_fast_ = nullptr;
  QCheckBox *chk_cb_resized_ = nullptr;
  QSpinBox *spin_cb_res_max_ = nullptr;
  QCheckBox *chk_cb_subpix_ = nullptr;
  QSpinBox *spin_cb_max_lost_ = nullptr;
  QSpinBox *spin_cb_padding_ = nullptr;
  // dot
  QCheckBox *chk_dot_sym_ = nullptr;
  QCheckBox *chk_dot_cluster_ = nullptr;
  QCheckBox *chk_dot_filter_area_ = nullptr;
  QDoubleSpinBox *spin_dot_min_area_ = nullptr;
  QDoubleSpinBox *spin_dot_max_area_ = nullptr;
  QDoubleSpinBox *spin_dot_min_dist_ = nullptr;
  QCheckBox *chk_dot_resized_ = nullptr;
  QSpinBox *spin_dot_res_max_ = nullptr;
  // april
  QSpinBox *spin_april_threads_ = nullptr;
  QSpinBox *spin_april_decimate_ = nullptr;
  QDoubleSpinBox *spin_april_sigma_ = nullptr;
  QCheckBox *chk_april_refine_ = nullptr;
  QDoubleSpinBox *spin_april_sharpen_ = nullptr;
  QCheckBox *chk_april_debug_ = nullptr;
  QSpinBox *spin_april_hamming_ = nullptr;
  QDoubleSpinBox *spin_april_margin_ = nullptr;
  QDoubleSpinBox *spin_april_ratio_ = nullptr;
  // charuco
  QSpinBox *spin_charuco_win_min_ = nullptr;
  QSpinBox *spin_charuco_win_max_ = nullptr;
  QDoubleSpinBox *spin_charuco_marker_len_ = nullptr;
};

/// \brief 采集统计（Qt 轻量图 / matplotlib 嵌入）
class IntrinsicsStatsDialog : public QDialog {
  Q_OBJECT

public:
  explicit IntrinsicsStatsDialog(QWidget *parent = nullptr);
  void refresh(const SessionController *session, const std::string &backend);

private:
  QLabel *lbl_backend_ = nullptr;
  QLabel *lbl_train_tile_ = nullptr;
  QLabel *lbl_eval_tile_ = nullptr;
  QLabel *lbl_heatmap_tile_ = nullptr;
  QLabel *summary_label_ = nullptr;
  ImageViewWidget *plot_view_ = nullptr;
  class IntrinsicsAsyncPlotController *plot_loader_ = nullptr;
  const SessionController *last_session_ = nullptr;
  std::string last_backend_;
};

}  // namespace gui
}  // namespace hs_calib
