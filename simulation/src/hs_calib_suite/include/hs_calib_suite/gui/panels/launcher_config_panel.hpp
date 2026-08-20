#pragma once

#include <map>
#include <string>

#include <QString>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QVBoxLayout;

namespace hs_calib {
namespace gui {

class SessionController;

/// \brief 配置面板：数据源页 + 标定设置页（按标定类型组合显隐）
class LauncherConfigPanel : public QWidget {
  Q_OBJECT

public:
  explicit LauncherConfigPanel(QWidget *parent = nullptr);

  void set_calibrator_id(const QString &id);
  QString calibrator_id() const { return calibrator_id_; }

  void apply_to_session(SessionController *session) const;
  std::map<std::string, std::string> to_config_map() const;

  QComboBox *combo_source_mode() const { return combo_source_mode_; }
  QComboBox *combo_image_topic() const { return combo_image_topic_; }
  QComboBox *combo_camera_info_topic() const { return combo_camera_info_topic_; }
  QLineEdit *edit_image_dir() const { return edit_image_dir_; }
  QWidget *offline_row() const { return offline_row_; }
  QWidget *topic_row() const { return topic_row_; }
  QLineEdit *edit_bag_path() const { return edit_bag_path_; }
  QComboBox *combo_bag_topic() const { return combo_bag_topic_; }
  int bag_max_frames() const;
  void refresh_source_mode_rows();
  void refresh_intrinsics_source_rows();
  int intrinsics_source_mode() const;
  QLineEdit *edit_intrinsics_yaml() const { return edit_intrinsics_yaml_; }
  QComboBox *combo_intrinsics_source() const { return combo_intrinsics_source_; }
  QPushButton *btn_refresh_topics() const { return btn_refresh_topics_; }
  QPushButton *btn_load_bag() const { return btn_load_bag_; }
  QWidget *handeye_block() const { return handeye_block_; }
  QWidget *handeye_group() const { return handeye_group_; }

  QLineEdit *edit_camera_yaml() const { return edit_camera_yaml_; }
  QLineEdit *edit_left_camera_yaml() const { return edit_left_camera_yaml_; }
  QLineEdit *edit_right_camera_yaml() const { return edit_right_camera_yaml_; }
  QWidget *stereo_extrinsics_block() const { return stereo_extrinsics_block_; }
  QLineEdit *edit_pose_csv() const { return edit_pose_csv_; }
  QComboBox *combo_pose_source() const { return combo_pose_source_; }
  QLineEdit *edit_base_frame() const { return edit_base_frame_; }
  QLineEdit *edit_gripper_frame() const { return edit_gripper_frame_; }
  QComboBox *combo_handeye_method() const { return combo_handeye_method_; }
  QLineEdit *edit_config_path() const { return edit_config_path_; }
  QWidget *data_source_panel() const { return data_source_panel_; }
  QComboBox *combo_target_type() const { return combo_target_type_; }
  QString target_type_id() const;
  QComboBox *combo_camera_model() const { return combo_camera_model_; }
  QComboBox *combo_stereo_side() const { return combo_stereo_side_; }
  QSpinBox *spin_min_views() const { return spin_min_views_; }

  int squares_x() const;
  int squares_y() const;
  QSpinBox *spin_squares_x() const { return spin_squares_x_; }
  QSpinBox *spin_squares_y() const { return spin_squares_y_; }
  double square_length() const;
  double marker_length() const;
  int min_views() const;
  double min_confidence() const;
  double min_diversity() const;
  int auto_cooldown_ms() const;
  bool auto_capture_default() const;

  /// \brief 显示设置已移至工作台预览；保留接口返回默认值
  bool viz_draw_corners() const;
  bool viz_draw_hull() const;
  bool viz_show_conf() const;
  bool viz_draw_aruco() const;
  int viz_marker_radius() const;

  void set_board_params(int sx, int sy, double square_m, double marker_m = 0.018);
  void set_config_path(const QString &path);
  void set_image_dir(const QString &path);
  void apply_project_frames(
      const QString &parent_frame,
      const QString &child_frame,
      const QString &base_frame,
      const QString &gripper_frame,
      const QString &image_frame = QString(),
      const QString &camera_link_frame = QString());
  QString export_dir_hint() const;
  QLineEdit *edit_export_path() const { return edit_export_path_; }
  QLineEdit *edit_parent_frame() const { return edit_parent_frame_; }
  QLineEdit *edit_child_frame() const { return edit_child_frame_; }

signals:
  void source_mode_changed(int index);
  void intrinsics_source_changed(int mode);
  void image_topic_changed(const QString &topic);
  void camera_info_topic_changed(const QString &topic);
  void refresh_topics_clicked();
  void browse_image_dir_clicked();
  void browse_bag_clicked();
  void load_bag_clicked();
  void browse_intrinsics_yaml_clicked();
  void browse_camera_yaml_clicked();
  void browse_left_camera_yaml_clicked();
  void browse_right_camera_yaml_clicked();
  void browse_pose_csv_clicked();
  void reload_yaml_clicked();
  void pose_source_changed(int index);

private:
  QWidget *make_group(const QString &title, QWidget *body);
  QFormLayout *new_form(QWidget *host);
  void update_board_param_visibility();
  static void set_form_row_visible(QFormLayout *form, QWidget *field, bool visible);

  QString calibrator_id_ = QStringLiteral("cam_intrinsics");
  QWidget *data_source_panel_ = nullptr;

  QFormLayout *form_ros_ = nullptr;
  QComboBox *combo_source_mode_ = nullptr;
  QComboBox *combo_image_topic_ = nullptr;
  QCheckBox *chk_use_rectified_ = nullptr;
  QLineEdit *edit_image_frame_ = nullptr;
  QLineEdit *edit_camera_link_frame_ = nullptr;
  QWidget *offline_row_ = nullptr;
  QWidget *topic_row_ = nullptr;
  QWidget *bag_path_row_ = nullptr;
  QWidget *bag_topic_row_ = nullptr;
  QPushButton *btn_refresh_topics_ = nullptr;
  QPushButton *btn_load_bag_ = nullptr;
  QLineEdit *edit_image_dir_ = nullptr;
  QLineEdit *edit_bag_path_ = nullptr;
  QComboBox *combo_bag_topic_ = nullptr;
  QSpinBox *spin_bag_max_frames_ = nullptr;
  QLineEdit *edit_config_path_ = nullptr;

  QWidget *intrinsics_source_block_ = nullptr;
  QFormLayout *form_intrinsics_ = nullptr;
  QComboBox *combo_intrinsics_source_ = nullptr;
  QComboBox *combo_camera_info_topic_ = nullptr;
  QWidget *camera_info_row_ = nullptr;
  QWidget *intrinsics_yaml_row_ = nullptr;
  QPushButton *btn_refresh_camera_info_ = nullptr;
  QLineEdit *edit_intrinsics_yaml_ = nullptr;

  QWidget *tf_source_block_ = nullptr;
  QWidget *stereo_extrinsics_group_ = nullptr;
  QWidget *stereo_extrinsics_block_ = nullptr;
  QLineEdit *edit_left_camera_yaml_ = nullptr;
  QLineEdit *edit_right_camera_yaml_ = nullptr;

  QWidget *board_params_block_ = nullptr;
  QFormLayout *form_target_ = nullptr;
  QComboBox *combo_target_type_ = nullptr;
  QComboBox *combo_dictionary_ = nullptr;
  QWidget *squares_row_ = nullptr;
  QWidget *area_row_ = nullptr;
  QLabel *lab_squares_x_ = nullptr;
  QLabel *lab_squares_y_ = nullptr;
  QSpinBox *spin_squares_x_ = nullptr;
  QSpinBox *spin_squares_y_ = nullptr;
  QDoubleSpinBox *spin_square_length_ = nullptr;
  QDoubleSpinBox *spin_marker_length_ = nullptr;
  QDoubleSpinBox *spin_min_board_area_ = nullptr;
  QDoubleSpinBox *spin_max_board_area_ = nullptr;
  QDoubleSpinBox *spin_max_tag_distance_ = nullptr;
  QCheckBox *chk_cb_adaptive_ = nullptr;
  QCheckBox *chk_cb_normalize_ = nullptr;
  QCheckBox *chk_cb_filter_quads_ = nullptr;
  QCheckBox *chk_cb_fast_check_ = nullptr;
  QSpinBox *spin_subpix_win_ = nullptr;
  QWidget *chess_flags_row_ = nullptr;

  QWidget *capture_criteria_block_ = nullptr;
  QFormLayout *form_detect_ = nullptr;
  QSpinBox *spin_min_views_ = nullptr;
  QDoubleSpinBox *spin_min_confidence_ = nullptr;
  QDoubleSpinBox *spin_min_diversity_ = nullptr;
  QSpinBox *spin_auto_cooldown_ms_ = nullptr;
  QCheckBox *chk_auto_capture_default_ = nullptr;

  QWidget *solver_intrinsics_block_ = nullptr;
  QFormLayout *form_solver_ = nullptr;
  QWidget *solver_flags_row1_ = nullptr;
  QWidget *solver_flags_row2_ = nullptr;
  QComboBox *combo_camera_model_ = nullptr;
  QCheckBox *chk_fix_principal_ = nullptr;
  QCheckBox *chk_fix_aspect_ = nullptr;
  QCheckBox *chk_zero_tangent_ = nullptr;
  QCheckBox *chk_fix_k1_ = nullptr;
  QCheckBox *chk_fix_k2_ = nullptr;
  QCheckBox *chk_fix_k3_ = nullptr;
  QCheckBox *chk_rational_model_ = nullptr;
  QCheckBox *chk_thin_prism_ = nullptr;
  QCheckBox *chk_use_intrinsic_guess_ = nullptr;
  QComboBox *combo_stereo_side_ = nullptr;
  QComboBox *combo_handeye_method_ = nullptr;
  QLineEdit *edit_export_path_ = nullptr;

  QWidget *export_tf_row_ = nullptr;
  QLineEdit *edit_parent_frame_ = nullptr;
  QLineEdit *edit_child_frame_ = nullptr;

  QWidget *handeye_group_ = nullptr;
  QWidget *handeye_block_ = nullptr;
  QLineEdit *edit_camera_yaml_ = nullptr;
  QComboBox *combo_pose_source_ = nullptr;
  QLineEdit *edit_pose_csv_ = nullptr;
  QLineEdit *edit_base_frame_ = nullptr;
  QLineEdit *edit_gripper_frame_ = nullptr;
};

}  // namespace gui
}  // namespace hs_calib
