#pragma once

#include <memory>
#include <atomic>

#include <QMainWindow>
#include <QStackedWidget>
#include <QString>

#include "hs_calib_suite/gui/log/log_level.hpp"
#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/projects/project_catalog.hpp"
#include "hs_calib_suite/gui/projects/project_workspace.hpp"
#include "hs_calib_suite/gui/bridges/bag_image_loader.hpp"
#include "hs_calib_suite/gui/bridges/ros_bag_frame_reader.hpp"
#include "hs_calib_suite/core/review/review_diagnostics.hpp"

class QAction;
class QActionGroup;
class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QEvent;
class QFrame;
class QGridLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QObject;
class QProgressDialog;
class QPushButton;
class QSpinBox;
class QSlider;
class QStackedWidget;
class QTabWidget;
class QTextEdit;
class QTimer;
class QToolButton;
class QWidget;

namespace hs_calib {
namespace gui {

class RosImageBridge;
class RosStereoImageBridge;
class RosExecutorHub;
class TfPoseBridge;
class LauncherConfigPanel;
class ImageViewWidget;
class ResidualBarWidget;
class CoverageMapWidget;
class IntrinsicsControlRail;
class IntrinsicsMetricsStrip;
class IntrinsicsParameterDialog;
class IntrinsicsStatsDialog;
class IntrinsicsCalibrationBarsDialog;
class IntrinsicsCalibrationRmsDialog;
class IntrinsicsDetectionDetailsDialog;
class IntrinsicsVizOptionsDialog;
class IntrinsicsCalibrationStatusDialog;

/// \brief 标定管理主窗口（单目内参 / 直角三面 / 手眼）
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /// \brief 构造主窗口并组装会话/桥接/页面
  explicit MainWindow(QWidget *parent = nullptr);
  /// \brief 析构：停转 ROS 泵并退订图像
  ~MainWindow() override;

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

private:
  enum class PageId {
    Home = 0,         ///< 1 选择任务
    DataSource = 1,   ///< 2 数据源设置
    Setup = 2,        ///< 3 标定设置
    Workbench = 3,    ///< 4 采集求解
    StereoRectify = 4, ///< 5 校正验证（仅双目内参）
    Review = 5,       ///< 5/6 复核导出
    DetectLab = 6,    ///< 特征检测台（不计入步骤条）
  };

  // —— 窗口壳：菜单 / 工具栏 / 主题 / 导航 ——
  void setup_menu_bar();
  void show_about_dialog();
  void setup_mode_corner();
  void setup_status_bar();
  void refresh_ready_indicator(bool ready, const QString &detail_tip);
  void refresh_online_indicator();
  void setup_central_widget();
  void apply_theme();
  void set_theme(ThemeId id);
  void refresh_toolbar_icons();

  void go_to(PageId page);
  void update_step_rail(PageId page);
  void update_status_bar(PageId page);
  bool uses_stereo_rectify_flow() const;
  PageId page_id_for_step_index(int step) const;
  int step_index_for_page(PageId page) const;
  void refresh_status_task();
  QString current_task_status_text() const;
  void set_online_mode(bool online);
  void append_log(LogLevel level, const QString &line);
  void select_calib_tile(QFrame *tile);
  void refresh_project_list();
  void on_project_selection_changed();
  void on_new_project();
  void on_import_project();
  void on_import_project_images();
  void on_configure_project();
  void on_delete_project();
  void on_open_project_dir();
  void on_open_projects_root_dir();
  void on_project_context_menu(const QPoint &pos);
  bool ensure_project_workspace_open(QString *error_out = nullptr);
  void apply_selected_project_to_setup();
  void update_home_selection_label();
  QString selected_project_display_name() const;

  // —— 设置页 / 源 / 手眼 ——
  void refresh_setup_readiness();
  void refresh_setup_source_ui();
  void update_workbench_mode_actions();
  void set_workbench_path_text(const QString &text);
  void refresh_handeye_ui();
  void refresh_topic_list();
  void apply_refreshed_topics(const QStringList &image_topics,
                              const QStringList &info_topics);
  void sync_ros_image_subscription();
  void schedule_ros_image_subscription();
  void sync_ros_stereo_subscription();
  void sync_pending_ros_topics();
  bool needs_ros_image_subscription() const;
  void start_ros_image_pipeline();
  void stop_ros_image_pipeline();
  void clear_workbench_live_previews();
  void refresh_task_flow_chrome();
  void on_source_mode_changed(int index);
  void on_topic_changed(const QString &topic);
  void on_camera_info_topic_changed(const QString &topic);
  void on_intrinsics_source_changed(int mode);
  void on_ros_frame();
  void on_stereo_ros_frames();
  void run_stereo_live_preview_tick(bool allow_auto_capture);
  void apply_stereo_raw_previews();
  void on_capture_paired_observation();
  void sync_detect_intrinsics_from_sources();
  void apply_intrinsics_source_subscription();

  // —— 工作台：预览 / 检测 / 采集 / 求解 ——
  void refresh_workbench_view(bool update_preview = true);
  void show_intrinsics_parameter_dialog(int kind);
  void show_intrinsics_stats_dialog();
  void show_intrinsics_detection_details_dialog();
  void show_intrinsics_calibration_status_dialog();
  void show_intrinsics_calibration_bars_dialog();
  void show_intrinsics_calibration_rms_dialog();
  void show_intrinsics_tier4_statistics_dialogs();
  void show_intrinsics_viz_options_dialog();
  void refresh_intrinsics_workbench_ui();
  void update_workbench_layout_for_task();
  void refresh_review_view();
  void refresh_stereo_rectify_view();
  void schedule_review_diagnostics_async();
  void apply_review_diagnostics(const core::ReviewDiagnostics &diag);
  void show_preview_image(const QImage &img);
  void set_preview_live(bool live);
  void update_preview_mode_ui();
  void run_live_preview_tick(bool allow_auto_capture);
  void on_async_detect_started();
  void on_async_detect_finished(bool ok, const QString &err);
  void on_async_solve_started();
  void on_async_solve_progress(int percent, const QString &message);
  void on_async_solve_finished(bool ok, const QString &err);
  void update_solve_action_enabled();
  void update_detect_status_ui();
  void on_browse_image_dir();
  void on_browse_bag();
  void on_load_bag();
  void refresh_bag_topic_list();
  void apply_refreshed_bag_topics(const QList<BagTopicInfo> &topics, const QString &err);
  void apply_bag_load_result(
      int loaded, const QString &err, const QString &topic, RosBagFrameReader reader);
  void apply_stereo_bag_load_result(
      int loaded,
      const QString &err,
      const QString &left_topic,
      const QString &right_topic,
      RosBagStereoFrameReader reader);
  void apply_image_dir_scan(const QString &dir, const QStringList &paths);
  void schedule_workbench_view_refresh(bool update_preview);
  void schedule_workbench_preview_load();
  void on_browse_intrinsics_yaml();
  void on_browse_camera_yaml();
  void on_browse_left_camera_yaml();
  void on_browse_right_camera_yaml();
  void on_browse_pose_csv();
  void on_start_session();
  void on_detect_and_preview();
  void on_capture_observation();
  void on_solve();
  void on_export_yaml();
  QString compute_workbench_solve_fingerprint() const;
  void maybe_clear_observations_on_workbench_enter();
  bool ensure_implemented_calibrator(QString *error_out = nullptr) const;
  void sync_session_from_setup_ui();
  void sync_workbench_viz_from_session();
  void apply_workbench_viz_to_session();
  void refresh_workbench_preview_viz();
  void on_reload_default_board_config();
  void apply_board_config_from_package();
  bool apply_yaml_config_file(const QString &path, QString *error_out = nullptr);
  void load_calibrator_default_config();

  // —— 首页标定器瓷砖 ——
  void refresh_home_calibrator_grid();
  void on_home_category_changed(int category);
  void on_home_product_line_changed(int line);
  void refresh_home_category_chips();

  // —— 检测调试台 ——
  DetectLabMode detect_lab_mode() const;
  bool is_detect_lab_mode() const;
  bool is_detect_lab_full_mode() const;
  bool is_detect_lab_identify_mode() const;
  void apply_lab_params_to_session();
  void refresh_detect_lab_view(bool prefer_preview = true);
  void refresh_lab_mode_ui();
  void refresh_lab_source_ui();
  void apply_lab_camera_info_subscription();
  void on_lab_camera_info_changed(const QString &topic);
  void on_lab_refresh_camera_info();
  void on_lab_browse_images();
  QWidget *build_lab_identify_side();
  QWidget *build_lab_partial_side();
  QWidget *build_lab_full_side();
  QWidget *build_lab_board_form(QWidget *parent);
  QWidget *build_lab_source_block(QWidget *parent);
  void on_lab_detect(bool fast);
  void on_lab_identify();
  void on_lab_export_identify_json();
  void on_async_identify_started();
  void on_async_identify_finished(bool ok, const QString &err);
  void sync_lab_target_defaults();
  void populate_lab_identify_results();
  void run_lab_live_preview_tick();
  core::BoardTypeIdentifyOptions collect_lab_identify_options() const;

  // —— 页面工厂 / 小组件 ——
  QWidget *build_home_page();
  QWidget *build_data_source_page();
  QWidget *build_setup_page();
  QWidget *build_workbench_page();
  QWidget *build_stereo_rectify_page();
  QWidget *build_review_page();
  QWidget *build_detect_lab_page();
  void wire_launcher_panel_once();

  QWidget *make_panel(const QString &title, QWidget *body);
  QFrame *make_metric_card(const QString &name, const QString &value);
  QFrame *make_compact_metric_card(const QString &name, const QString &value);
  QFrame *make_calib_tile(
      const QString &title, const QString &id, bool implemented,
      const QString &subtitle = QString(), bool gallery = false);

  QStackedWidget *stack_ = nullptr;
  QTextEdit *log_ = nullptr;
  QWidget *log_panel_host_ = nullptr;
  QLabel *step_labels_[6] = {};
  QLabel *step_arrows_[5] = {};
  bool launcher_wired_ = false;
  QFrame *selected_tile_ = nullptr;
  QLabel *home_selection_ = nullptr;
  QString selected_calibrator_id_;
  QString selected_project_id_ = QStringLiteral("default_robot");
  ProjectCatalog project_catalog_;
  ProjectWorkspace project_workspace_;
  QListWidget *project_list_ = nullptr;
  int home_product_line_ = 0;  // 0工程标定 1检测调试
  int home_category_ = 0;      // 工程标定：0内参 1手眼 2外参 3多传感器
  QWidget *home_tile_host_ = nullptr;
  QGridLayout *home_tile_grid_ = nullptr;
  QButtonGroup *home_product_group_ = nullptr;
  QButtonGroup *home_category_group_ = nullptr;
  QWidget *home_category_row_host_ = nullptr;
  QWidget *home_task_split_ = nullptr;  ///< 分类栏 + 任务列表/画廊
  QPushButton *btn_home_next_ = nullptr;
  LauncherConfigPanel *launcher_panel_ = nullptr;
  std::unique_ptr<SessionController> session_;
  std::unique_ptr<RosImageBridge> ros_bridge_;
  std::unique_ptr<RosStereoImageBridge> ros_stereo_bridge_;
  std::unique_ptr<RosExecutorHub> ros_executor_hub_;
  std::unique_ptr<TfPoseBridge> tf_bridge_;
  QTimer *ros_stereo_sub_debounce_ = nullptr;
  QTimer *workbench_refresh_timer_ = nullptr;
  bool workbench_refresh_update_preview_ = false;

  // ===== Setup =====
  QComboBox *combo_source_mode_ = nullptr;
  QComboBox *combo_target_type_ = nullptr;
  QComboBox *combo_camera_model_ = nullptr;
  QWidget *offline_row_ = nullptr;
  QWidget *topic_row_ = nullptr;
  QWidget *handeye_block_ = nullptr;
  QComboBox *combo_image_topic_ = nullptr;
  QComboBox *combo_camera_info_topic_ = nullptr;
  QComboBox *combo_pose_source_ = nullptr;
  QComboBox *combo_handeye_method_ = nullptr;
  QPushButton *btn_refresh_topics_ = nullptr;
  QSpinBox *spin_squares_x_ = nullptr;
  QSpinBox *spin_squares_y_ = nullptr;
  QSpinBox *spin_min_views_ = nullptr;
  QDoubleSpinBox *spin_square_length_ = nullptr;
  QDoubleSpinBox *spin_marker_length_ = nullptr;
  QLineEdit *edit_image_dir_ = nullptr;
  QLineEdit *edit_camera_yaml_ = nullptr;
  QLineEdit *edit_pose_csv_ = nullptr;
  QLineEdit *edit_base_frame_ = nullptr;
  QLineEdit *edit_gripper_frame_ = nullptr;
  QLineEdit *edit_config_path_ = nullptr;
  QPushButton *btn_start_session_ = nullptr;
  QPushButton *btn_datasource_next_ = nullptr;
  QToolButton *btn_ready_indicator_ = nullptr;
  QToolButton *btn_online_indicator_ = nullptr;
  bool ready_indicator_ok_ = false;
  QString ready_indicator_tip_;

  // ===== Workbench =====
  ImageViewWidget *preview_view_ = nullptr;
  QWidget *mono_preview_host_ = nullptr;
  QWidget *stereo_preview_host_ = nullptr;
  ImageViewWidget *stereo_preview_left_ = nullptr;
  ImageViewWidget *stereo_preview_right_ = nullptr;
  QLabel *stereo_sync_label_ = nullptr;
  QLabel *preview_title_label_ = nullptr;
  QPushButton *btn_preview_live_ = nullptr;
  QPushButton *btn_preview_freeze_ = nullptr;
  QPushButton *btn_preview_zoom_in_ = nullptr;
  QPushButton *btn_preview_zoom_out_ = nullptr;
  QPushButton *btn_preview_fit_ = nullptr;
  QPushButton *btn_preview_one_to_one_ = nullptr;
  QPushButton *btn_preview_save_ = nullptr;
  bool preview_live_ = true;
  QListWidget *obs_list_ = nullptr;
  QListWidget *obs_eval_list_ = nullptr;
  QTabWidget *obs_tabs_ = nullptr;
  QWidget *workbench_default_right_ = nullptr;
  IntrinsicsControlRail *intrinsics_control_rail_ = nullptr;
  IntrinsicsMetricsStrip *intrinsics_metrics_strip_ = nullptr;
  IntrinsicsParameterDialog *intrinsics_calib_params_dlg_ = nullptr;
  IntrinsicsParameterDialog *intrinsics_collector_params_dlg_ = nullptr;
  IntrinsicsParameterDialog *intrinsics_detector_params_dlg_ = nullptr;
  IntrinsicsStatsDialog *intrinsics_stats_dlg_ = nullptr;
  IntrinsicsCalibrationBarsDialog *intrinsics_calibration_bars_dlg_ = nullptr;
  IntrinsicsCalibrationRmsDialog *intrinsics_calibration_rms_dlg_ = nullptr;
  IntrinsicsDetectionDetailsDialog *intrinsics_detection_details_dlg_ = nullptr;
  IntrinsicsCalibrationStatusDialog *intrinsics_calibration_status_dlg_ = nullptr;
  IntrinsicsVizOptionsDialog *intrinsics_viz_options_dlg_ = nullptr;
  std::string stats_backend_ = "qt";
  QLabel *metric_frames_ = nullptr;
  QLabel *metric_detect_ = nullptr;
  QLabel *metric_reproj_ = nullptr;
  QLabel *metric_coverage_ = nullptr;
  QLabel *workbench_path_label_ = nullptr;
  QWidget *workbench_header_left_ = nullptr;
  QString workbench_path_full_;
  QPushButton *btn_prev_ = nullptr;
  QPushButton *btn_next_ = nullptr;
  QPushButton *btn_detect_ = nullptr;
  QPushButton *btn_capture_wb_ = nullptr;
  QPushButton *btn_solve_wb_ = nullptr;
  QProgressDialog *solve_progress_dlg_ = nullptr;
  QString workbench_solve_fingerprint_;
  QCheckBox *chk_auto_capture_ = nullptr;
  QCheckBox *chk_viz_corners_wb_ = nullptr;
  QCheckBox *chk_viz_hull_wb_ = nullptr;
  QCheckBox *chk_viz_conf_wb_ = nullptr;
  QCheckBox *chk_viz_aruco_wb_ = nullptr;
  QWidget *viz_classic_row_ = nullptr;
  QComboBox *combo_intrinsics_view_mode_ = nullptr;
  QWidget *viz_tier4_row_ = nullptr;
  QPushButton *btn_tier4_viz_options_wb_ = nullptr;
  QSlider *intrinsics_sample_slider_ = nullptr;
  QLabel *intrinsics_sample_slider_label_ = nullptr;
  QSpinBox *spin_viz_marker_wb_ = nullptr;
  qint64 last_auto_capture_ms_ = 0;
  qint64 last_live_detect_ms_ = 0;
  qint64 last_live_raw_preview_ms_ = 0;
  qint64 last_stereo_raw_preview_ms_ = 0;
  qint64 last_stereo_sync_label_ms_ = 0;
  qint64 last_stereo_detect_ui_ms_ = 0;
  qint64 last_intrinsics_ui_refresh_ms_ = 0;
  qint64 last_stereo_preview_apply_ms_ = 0;
  qint64 last_readiness_refresh_ms_ = 0;
  QString workbench_layout_task_token_;
  uint64_t review_diag_epoch_ = 0;
  bool pending_detect_log_ = false;
  bool pending_capture_after_detect_ = false;
  bool allow_auto_on_detect_finish_ = false;

  // ===== Review =====
  QTextEdit *review_text_ = nullptr;
  QLabel *review_rmse_ = nullptr;
  QLabel *review_views_ = nullptr;
  QLabel *review_size_ = nullptr;
  QListWidget *review_obs_list_ = nullptr;
  ResidualBarWidget *review_residual_bars_ = nullptr;
  CoverageMapWidget *review_coverage_map_ = nullptr;
  QLabel *review_diag_label_ = nullptr;
  int review_selected_view_ = -1;
  void on_review_obs_clicked(QListWidgetItem *item);
  void apply_review_view_filter();
  void on_review_bar_clicked(int view_index);

  // ===== Detect Lab =====
  ImageViewWidget *lab_preview_ = nullptr;
  QLabel *lab_preview_title_label_ = nullptr;
  QPushButton *btn_lab_preview_live_ = nullptr;
  QPushButton *btn_lab_preview_freeze_ = nullptr;
  QLabel *lab_title_label_ = nullptr;
  QWidget *lab_board_form_panel_ = nullptr;
  QWidget *lab_identify_panel_ = nullptr;
  QWidget *lab_offline_host_ = nullptr;
  QWidget *lab_camera_info_host_ = nullptr;
  QWidget *lab_nav_host_ = nullptr;
  QStackedWidget *lab_mode_stack_ = nullptr;
  QComboBox *combo_lab_target_ = nullptr;
  QComboBox *combo_lab_dictionary_ = nullptr;
  QComboBox *combo_lab_dict_hint_ = nullptr;
  QComboBox *combo_lab_camera_info_ = nullptr;
  QPushButton *btn_lab_refresh_camera_info_ = nullptr;
  QSpinBox *spin_lab_dict_scan_ = nullptr;
  QWidget *lab_candidate_host_ = nullptr;
  QCheckBox *chk_lab_cand_chessboard_ = nullptr;
  QCheckBox *chk_lab_cand_charuco_ = nullptr;
  QCheckBox *chk_lab_cand_aruco_ = nullptr;
  QCheckBox *chk_lab_cand_aruco_grid_ = nullptr;
  QCheckBox *chk_lab_cand_aprilgrid_ = nullptr;
  QCheckBox *chk_lab_cand_circles_sym_ = nullptr;
  QCheckBox *chk_lab_cand_circles_asym_ = nullptr;
  QCheckBox *chk_lab_cand_tri_chess_ = nullptr;
  QCheckBox *chk_lab_cand_tri_charuco_ = nullptr;
  QSpinBox *spin_lab_squares_x_ = nullptr;
  QSpinBox *spin_lab_squares_y_ = nullptr;
  QDoubleSpinBox *spin_lab_square_len_ = nullptr;
  QDoubleSpinBox *spin_lab_marker_len_ = nullptr;
  QLineEdit *edit_lab_image_dir_ = nullptr;
  QLabel *lab_path_label_ = nullptr;
  QLabel *lab_stats_ = nullptr;
  QWidget *lab_detect_action_host_ = nullptr;
  QListWidget *lab_identify_list_ = nullptr;
  QPushButton *btn_lab_prev_ = nullptr;
  QPushButton *btn_lab_next_ = nullptr;
  QPushButton *btn_lab_identify_ = nullptr;
  QPushButton *btn_lab_detect_ = nullptr;
  QPushButton *btn_lab_export_json_ = nullptr;
  bool lab_pending_log_ = false;
  bool lab_pending_identify_log_ = false;

  QAction *act_home_ = nullptr;
  QAction *act_data_source_ = nullptr;
  QAction *act_setup_ = nullptr;
  QAction *act_workbench_ = nullptr;
  QAction *act_review_ = nullptr;
  QAction *act_online_ = nullptr;
  QAction *act_offline_ = nullptr;
  QAction *act_capture_ = nullptr;
  QAction *act_solve_ = nullptr;
  QAction *act_export_ = nullptr;
  QAction *act_theme_dark_ = nullptr;
  QAction *act_theme_light_ = nullptr;
  QAction *act_theme_blue_ = nullptr;
  QActionGroup *theme_group_ = nullptr;

  QLabel *status_mode_ = nullptr;
  QLabel *status_page_ = nullptr;
  QLabel *status_hint_ = nullptr;
  QLabel *status_task_ = nullptr;
  QLabel *flow_workbench_title_ = nullptr;
  QLabel *flow_review_title_ = nullptr;
  QLabel *flow_rectify_title_ = nullptr;
  ImageViewWidget *rectify_preview_left_ = nullptr;
  ImageViewWidget *rectify_preview_right_ = nullptr;
  QLabel *rectify_path_label_ = nullptr;
  QLabel *rectify_metric_baseline_ = nullptr;
  QLabel *rectify_metric_rms_ = nullptr;
  QLabel *rectify_metric_brightness_ = nullptr;
  QLabel *rectify_hint_label_ = nullptr;
  QSlider *rectify_pair_slider_ = nullptr;
  QLabel *rectify_pair_slider_label_ = nullptr;

  ThemeId theme_id_ = ThemeId::Dark;
  bool online_mode_ = false;
  std::atomic<bool> topic_refresh_busy_{false};
  std::atomic<bool> bag_topic_refresh_busy_{false};
  std::atomic<bool> bag_load_busy_{false};
  std::atomic<bool> image_dir_load_busy_{false};
};

}  // namespace gui
}  // namespace hs_calib
