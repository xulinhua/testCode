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

/// \brief Launcher 配置面板（Tier4 风格：按组密集展示标定器参数）
class LauncherConfigPanel : public QWidget {
  Q_OBJECT

public:
  /// \brief 构建分组配置面板
  explicit LauncherConfigPanel(QWidget *parent = nullptr);

  /// \brief 切换标定器并调整手眼块/默认靶标
  void set_calibrator_id(const QString &id);
  /// \brief 当前标定器 ID
  QString calibrator_id() const { return calibrator_id_; }

  /// \brief 将面板参数写入 SessionController
  void apply_to_session(SessionController *session) const;
  /// \brief 导出求解用配置字典
  std::map<std::string, std::string> to_config_map() const;

  // 供 MainWindow 桥接的话题 / IO 访问器
  /// \brief 图像源模式下拉
  QComboBox *combo_source_mode() const { return combo_source_mode_; }
  /// \brief 图像话题下拉
  QComboBox *combo_image_topic() const { return combo_image_topic_; }
  /// \brief CameraInfo（内参）话题下拉
  QComboBox *combo_camera_info_topic() const { return combo_camera_info_topic_; }
  /// \brief 离线图片目录编辑框
  QLineEdit *edit_image_dir() const { return edit_image_dir_; }
  /// \brief 离线源行容器
  QWidget *offline_row() const { return offline_row_; }
  /// \brief ROS 话题行容器
  QWidget *topic_row() const { return topic_row_; }
  /// \brief 按离线/ROS 显隐对应表单行（含左侧标签）
  void refresh_source_mode_rows();
  /// \brief 刷新话题按钮
  QPushButton *btn_refresh_topics() const { return btn_refresh_topics_; }
  /// \brief 手眼参数块
  QWidget *handeye_block() const { return handeye_block_; }

  /// \brief 相机内参 YAML 编辑框
  QLineEdit *edit_camera_yaml() const { return edit_camera_yaml_; }
  /// \brief 双目外参：左/右内参 YAML
  QLineEdit *edit_left_camera_yaml() const { return edit_left_camera_yaml_; }
  QLineEdit *edit_right_camera_yaml() const { return edit_right_camera_yaml_; }
  /// \brief 双目外参配置块
  QWidget *stereo_extrinsics_block() const { return stereo_extrinsics_block_; }
  /// \brief 位姿 CSV 编辑框
  QLineEdit *edit_pose_csv() const { return edit_pose_csv_; }
  /// \brief 位姿源下拉
  QComboBox *combo_pose_source() const { return combo_pose_source_; }
  /// \brief base 坐标系编辑框
  QLineEdit *edit_base_frame() const { return edit_base_frame_; }
  /// \brief gripper 坐标系编辑框
  QLineEdit *edit_gripper_frame() const { return edit_gripper_frame_; }
  /// \brief 手眼方法下拉
  QComboBox *combo_handeye_method() const { return combo_handeye_method_; }
  /// \brief 配置路径编辑框
  QLineEdit *edit_config_path() const { return edit_config_path_; }
  /// \brief 靶标类型下拉
  QComboBox *combo_target_type() const { return combo_target_type_; }
  /// \brief 相机模型下拉
  QComboBox *combo_camera_model() const { return combo_camera_model_; }
  /// \brief 双目当前采集侧（left/right）
  QComboBox *combo_stereo_side() const { return combo_stereo_side_; }
  /// \brief 最少视角数微调框
  QSpinBox *spin_min_views() const { return spin_min_views_; }

  /// \brief 方格/内角点 X 数
  int squares_x() const;
  /// \brief 方格/内角点 Y 数
  int squares_y() const;
  /// \brief 方格边长（米）
  double square_length() const;
  /// \brief 码边长（米）
  double marker_length() const;
  /// \brief 最少采集视角数
  int min_views() const;
  /// \brief 自动采集最低置信度
  double min_confidence() const;
  /// \brief 自动采集最小视角差异
  double min_diversity() const;
  /// \brief 自动采集冷却毫秒
  int auto_cooldown_ms() const;
  /// \brief 是否默认开启自动采集
  bool auto_capture_default() const;

  // 可视化选项（工作台同步使用）
  /// \brief 是否画角点
  bool viz_draw_corners() const;
  /// \brief 是否画凸包
  bool viz_draw_hull() const;
  /// \brief 是否显示置信度
  bool viz_show_conf() const;
  /// \brief 是否叠加 ArUco
  bool viz_draw_aruco() const;
  /// \brief 角点/码绘制半径
  int viz_marker_radius() const;

  /// \brief 写入板参数到控件
  void set_board_params(int sx, int sy, double square_m, double marker_m = 0.018);
  /// \brief 设置配置路径编辑框
  void set_config_path(const QString &path);
  /// \brief 设置离线图片目录
  void set_image_dir(const QString &path);
  /// \brief 应用项目默认坐标系（parent/child/base/gripper/image/camera_link）
  void apply_project_frames(
      const QString &parent_frame,
      const QString &child_frame,
      const QString &base_frame,
      const QString &gripper_frame,
      const QString &image_frame = QString(),
      const QString &camera_link_frame = QString());
  /// \brief 导出目录提示路径
  QString export_dir_hint() const;
  /// \brief 导出目录编辑框
  QLineEdit *edit_export_path() const { return edit_export_path_; }

  /// \brief parent / child 坐标系编辑框
  QLineEdit *edit_parent_frame() const { return edit_parent_frame_; }
  QLineEdit *edit_child_frame() const { return edit_child_frame_; }

signals:
  /// \brief 图像源模式变更
  void source_mode_changed(int index);
  /// \brief 图像话题变更
  void image_topic_changed(const QString &topic);
  /// \brief CameraInfo 话题变更
  void camera_info_topic_changed(const QString &topic);
  /// \brief 点击刷新话题
  void refresh_topics_clicked();
  /// \brief 点击浏览图片目录
  void browse_image_dir_clicked();
  /// \brief 点击浏览相机 YAML
  void browse_camera_yaml_clicked();
  /// \brief 点击浏览左目内参 YAML（双目外参）
  void browse_left_camera_yaml_clicked();
  /// \brief 点击浏览右目内参 YAML（双目外参）
  void browse_right_camera_yaml_clicked();
  /// \brief 点击浏览位姿 CSV
  void browse_pose_csv_clicked();
  /// \brief 点击重载 YAML 配置
  void reload_yaml_clicked();
  /// \brief 位姿源变更
  void pose_source_changed(int index);

private:
  QWidget *make_group(const QString &title, QWidget *body);
  QFormLayout *new_form(QWidget *host);
  /// \brief 按当前靶标类型显隐板参数与检测选项，并更新标签文案
  void update_board_param_visibility();
  static void set_form_row_visible(QFormLayout *form, QWidget *field, bool visible);

  QString calibrator_id_ = QStringLiteral("cam_intrinsics");

  // ROS / 输入输出
  QFormLayout *form_ros_ = nullptr;
  QComboBox *combo_source_mode_ = nullptr;
  QComboBox *combo_image_topic_ = nullptr;
  QComboBox *combo_camera_info_topic_ = nullptr;
  QCheckBox *chk_use_rectified_ = nullptr;
  QLineEdit *edit_image_frame_ = nullptr;
  QLineEdit *edit_camera_link_frame_ = nullptr;
  QWidget *offline_row_ = nullptr;
  QWidget *topic_row_ = nullptr;
  QWidget *camera_info_row_ = nullptr;
  QPushButton *btn_refresh_topics_ = nullptr;
  QLineEdit *edit_image_dir_ = nullptr;
  QLineEdit *edit_config_path_ = nullptr;

  // 靶标
  QFormLayout *form_target_ = nullptr;
  QComboBox *combo_target_type_ = nullptr;
  QComboBox *combo_dictionary_ = nullptr;
  QWidget *squares_row_ = nullptr;
  QWidget *lengths_row_ = nullptr;
  QWidget *area_row_ = nullptr;
  QLabel *lab_squares_x_ = nullptr;
  QLabel *lab_squares_y_ = nullptr;
  QLabel *lab_square_length_ = nullptr;
  QLabel *lab_marker_length_ = nullptr;
  QSpinBox *spin_squares_x_ = nullptr;
  QSpinBox *spin_squares_y_ = nullptr;
  QDoubleSpinBox *spin_square_length_ = nullptr;
  QDoubleSpinBox *spin_marker_length_ = nullptr;
  QDoubleSpinBox *spin_min_board_area_ = nullptr;
  QDoubleSpinBox *spin_max_board_area_ = nullptr;
  QDoubleSpinBox *spin_max_tag_distance_ = nullptr;

  // 检测 / 采集（对齐 Tier4：对数、距离、滤波）
  QFormLayout *form_detect_ = nullptr;
  QComboBox *combo_stereo_side_ = nullptr;
  QSpinBox *spin_min_views_ = nullptr;
  QDoubleSpinBox *spin_min_confidence_ = nullptr;
  QDoubleSpinBox *spin_min_diversity_ = nullptr;
  QSpinBox *spin_auto_cooldown_ms_ = nullptr;
  QCheckBox *chk_auto_capture_default_ = nullptr;
  QCheckBox *chk_cb_adaptive_ = nullptr;
  QCheckBox *chk_cb_normalize_ = nullptr;
  QCheckBox *chk_cb_filter_quads_ = nullptr;
  QCheckBox *chk_cb_fast_check_ = nullptr;
  QSpinBox *spin_subpix_win_ = nullptr;
  QWidget *chess_flags_row_ = nullptr;

  // 求解器
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

  // 坐标系 / 导出
  QLineEdit *edit_parent_frame_ = nullptr;
  QLineEdit *edit_child_frame_ = nullptr;
  QLineEdit *edit_export_path_ = nullptr;

  // 手眼
  QWidget *handeye_block_ = nullptr;
  QLineEdit *edit_camera_yaml_ = nullptr;
  QComboBox *combo_pose_source_ = nullptr;
  QLineEdit *edit_pose_csv_ = nullptr;
  QLineEdit *edit_base_frame_ = nullptr;
  QLineEdit *edit_gripper_frame_ = nullptr;
  QComboBox *combo_handeye_method_ = nullptr;

  // 双目外参
  QWidget *stereo_extrinsics_block_ = nullptr;
  QLineEdit *edit_left_camera_yaml_ = nullptr;
  QLineEdit *edit_right_camera_yaml_ = nullptr;

  // 可视化默认项
  QCheckBox *chk_viz_corners_ = nullptr;
  QCheckBox *chk_viz_hull_ = nullptr;
  QCheckBox *chk_viz_conf_ = nullptr;
  QCheckBox *chk_viz_aruco_ = nullptr;
  QSpinBox *spin_viz_marker_radius_ = nullptr;
};

}  // namespace gui
}  // namespace hs_calib
