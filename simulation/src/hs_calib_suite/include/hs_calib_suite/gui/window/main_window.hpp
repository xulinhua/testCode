#pragma once

#include <memory>

#include <QMainWindow>
#include <QStackedWidget>
#include <QString>

#include "hs_calib_suite/gui/log/log_level.hpp"
#include "hs_calib_suite/gui/theme/app_style.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/projects/project_catalog.hpp"
#include "hs_calib_suite/gui/projects/project_workspace.hpp"

class QAction;
class QActionGroup;
class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QGridLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QTimer;
class QWidget;

namespace hs_calib {
namespace gui {

class RosImageBridge;
class TfPoseBridge;
class LauncherConfigPanel;
class ImageViewWidget;
class ResidualBarWidget;
class CoverageMapWidget;

/// \brief 标定管理主窗口（单目内参 / 直角三面 / 手眼）
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /// \brief 构造主窗口并组装会话/桥接/页面
  explicit MainWindow(QWidget *parent = nullptr);
  /// \brief 析构：停转 ROS 泵并退订图像
  ~MainWindow() override;

private:
  enum class PageId {
    Home = 0,
    Setup = 1,
    Workbench = 2,
    Review = 3,
    DetectLab = 4,  ///< 局部角点 / ArUco / 圆点检测调试台
  };

  // —— 窗口壳：菜单 / 工具栏 / 主题 / 导航 ——
  void setup_menu_bar();
  void setup_tool_bar();
  void setup_status_bar();
  void setup_central_widget();
  void apply_theme();
  void set_theme(ThemeId id);
  void refresh_toolbar_icons();

  void go_to(PageId page);
  void update_step_rail(PageId page);
  void update_status_bar(PageId page);
  void set_online_mode(bool online);
  void append_log(LogLevel level, const QString &line);
  void select_calib_tile(QFrame *tile);
  void refresh_project_list();
  void on_project_selection_changed();
  void on_new_project();
  void on_import_project_images();
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
  void on_source_mode_changed(int index);
  void on_topic_changed(const QString &topic);
  void on_camera_info_topic_changed(const QString &topic);
  void on_ros_frame();
  void sync_detect_intrinsics_from_sources();

  // —— 工作台：预览 / 检测 / 采集 / 求解 ——
  void refresh_workbench_view(bool update_preview = true);
  void refresh_review_view();
  void show_preview_image(const QImage &img);
  void set_preview_live(bool live);
  void update_preview_mode_ui();
  void run_live_preview_tick(bool allow_auto_capture);
  void on_async_detect_started();
  void on_async_detect_finished(bool ok, const QString &err);
  void update_detect_status_ui();
  void on_browse_image_dir();
  void on_browse_camera_yaml();
  void on_browse_left_camera_yaml();
  void on_browse_right_camera_yaml();
  void on_browse_pose_csv();
  void on_start_session();
  void on_detect_and_preview();
  void on_capture_observation();
  void on_solve();
  void on_export_yaml();
  bool ensure_implemented_calibrator(QString *error_out = nullptr) const;
  void sync_session_from_setup_ui();
  void sync_workbench_viz_from_session();
  void apply_workbench_viz_to_session();
  void on_reload_default_board_config();
  void apply_board_config_from_package();

  // —— 首页标定器瓷砖 ——
  void refresh_home_calibrator_grid();
  void on_home_category_changed(int category);

  // —— 检测调试台 ——
  void apply_lab_params_to_session();
  void refresh_detect_lab_view(bool prefer_preview = true);
  void refresh_lab_mode_ui();
  bool is_detect_lab_full_mode() const;
  void on_lab_browse_images();
  void on_lab_detect(bool fast);
  void sync_lab_target_defaults();

  // —— 页面工厂 / 小组件 ——
  QWidget *build_home_page();
  QWidget *build_setup_page();
  QWidget *build_workbench_page();
  QWidget *build_review_page();
  QWidget *build_detect_lab_page();

  QWidget *make_panel(const QString &title, QWidget *body);
  QFrame *make_metric_card(const QString &name, const QString &value);
  QFrame *make_compact_metric_card(const QString &name, const QString &value);
  QFrame *make_calib_tile(
      const QString &title, const QString &id, bool implemented);

  QStackedWidget *stack_ = nullptr;
  QTextEdit *log_ = nullptr;
  QLabel *step_labels_[4] = {};
  QFrame *selected_tile_ = nullptr;
  QLabel *home_selection_ = nullptr;
  QString selected_calibrator_id_;
  QString selected_project_id_ = QStringLiteral("default_robot");
  ProjectCatalog project_catalog_;
  ProjectWorkspace project_workspace_;
  QListWidget *project_list_ = nullptr;
  int home_category_ = 0;  // 0内参 1手眼 2外参 3多传感器 4检测调试
  QWidget *home_tile_host_ = nullptr;
  QGridLayout *home_tile_grid_ = nullptr;
  QButtonGroup *home_category_group_ = nullptr;
  QPushButton *btn_home_next_ = nullptr;
  LauncherConfigPanel *launcher_panel_ = nullptr;
  std::unique_ptr<SessionController> session_;
  std::unique_ptr<RosImageBridge> ros_bridge_;
  std::unique_ptr<TfPoseBridge> tf_bridge_;
  QTimer *ros_spin_timer_ = nullptr;

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
  QLabel *setup_ready_label_ = nullptr;
  QPushButton *btn_start_session_ = nullptr;

  // ===== Workbench =====
  ImageViewWidget *preview_view_ = nullptr;
  QLabel *preview_title_label_ = nullptr;
  QPushButton *btn_preview_live_ = nullptr;
  QPushButton *btn_preview_freeze_ = nullptr;
  bool preview_live_ = true;
  QListWidget *obs_list_ = nullptr;
  QLabel *metric_frames_ = nullptr;
  QLabel *metric_detect_ = nullptr;
  QLabel *metric_reproj_ = nullptr;
  QLabel *metric_coverage_ = nullptr;
  QLabel *workbench_path_label_ = nullptr;
  QPushButton *btn_prev_ = nullptr;
  QPushButton *btn_next_ = nullptr;
  QPushButton *btn_detect_ = nullptr;
  QPushButton *btn_capture_wb_ = nullptr;
  QPushButton *btn_solve_wb_ = nullptr;
  QCheckBox *chk_auto_capture_ = nullptr;
  QCheckBox *chk_viz_corners_wb_ = nullptr;
  QCheckBox *chk_viz_hull_wb_ = nullptr;
  QCheckBox *chk_viz_conf_wb_ = nullptr;
  QCheckBox *chk_viz_aruco_wb_ = nullptr;
  QSpinBox *spin_viz_marker_wb_ = nullptr;
  qint64 last_auto_capture_ms_ = 0;
  qint64 last_live_detect_ms_ = 0;
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
  void on_review_obs_selected();
  void on_review_bar_clicked(int view_index);

  // ===== Detect Lab =====
  ImageViewWidget *lab_preview_ = nullptr;
  QLabel *lab_title_label_ = nullptr;
  QLabel *lab_subtitle_label_ = nullptr;
  QComboBox *combo_lab_target_ = nullptr;
  QComboBox *combo_lab_dictionary_ = nullptr;
  QSpinBox *spin_lab_squares_x_ = nullptr;
  QSpinBox *spin_lab_squares_y_ = nullptr;
  QDoubleSpinBox *spin_lab_square_len_ = nullptr;
  QDoubleSpinBox *spin_lab_marker_len_ = nullptr;
  QLineEdit *edit_lab_image_dir_ = nullptr;
  QLabel *lab_path_label_ = nullptr;
  QLabel *lab_stats_ = nullptr;
  QPushButton *btn_lab_prev_ = nullptr;
  QPushButton *btn_lab_next_ = nullptr;
  QPushButton *btn_lab_detect_ = nullptr;
  QPushButton *btn_lab_detect_fast_ = nullptr;
  bool lab_pending_log_ = false;

  QAction *act_home_ = nullptr;
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

  ThemeId theme_id_ = ThemeId::Dark;
  bool online_mode_ = false;
};

}  // namespace gui
}  // namespace hs_calib
