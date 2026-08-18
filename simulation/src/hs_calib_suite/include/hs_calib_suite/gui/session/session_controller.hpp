#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QImage>
#include <QObject>
#include <QString>
#include <QStringList>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/types/types.hpp"
#include "hs_calib_suite/gui/data/pose_csv_store.hpp"

namespace hs_calib {
namespace gui {

class TfPoseBridge;

enum class SourceMode {
  Offline = 0,
  RosTopic = 1,
};

enum class PoseSource {
  None = 0,  ///< 单目内参不需要
  Csv = 1,
  Tf = 2,
};

/// \brief 会话编排：单目内参 / 直角三面 / 眼在手上 / 眼在手外
class SessionController : public QObject {
  Q_OBJECT

public:
  /// \brief 构造会话控制器
  explicit SessionController(QObject *parent = nullptr);
  /// \brief 析构：取消后台检测并 join 线程
  ~SessionController() override;

  /// \brief 设置标定器 ID（并按类型调整位姿源等）
  void set_calibrator_id(const QString &id);
  /// \brief 当前标定器 ID
  QString calibrator_id() const { return calibrator_id_; }
  /// \brief 是否为手眼标定器
  bool is_handeye() const;
  /// \brief 是否为单目内参标定器
  bool is_intrinsics() const;
  /// \brief 是否为双目各自内参
  bool is_stereo_intrinsics() const;
  /// \brief 是否为双目相对外参
  bool is_stereo_extrinsics() const;
  /// \brief 是否为需左右侧标记的双目流程（内参分侧 / 外参成对）
  bool is_stereo_side_tagged() const;
  /// \brief 是否为直角三面标定器
  bool is_trihedral() const;

  /// \brief 设置图像源模式（离线/ROS）
  void set_source_mode(SourceMode mode);
  /// \brief 当前图像源模式
  SourceMode source_mode() const { return source_mode_; }

  /// \brief 设置手眼位姿源
  void set_pose_source(PoseSource src);
  /// \brief 当前手眼位姿源
  PoseSource pose_source() const { return pose_source_; }

  /// \brief 设置靶标网格参数
  void set_board_params(int squares_x, int squares_y, double square_length_m);
  /// \brief 方格/内角点 X 数
  int squares_x() const { return squares_x_; }
  /// \brief 方格/内角点 Y 数
  int squares_y() const { return squares_y_; }
  /// \brief 方格边长（米）
  double square_length_m() const { return square_length_m_; }

  /// \brief 设置自动采集阈值与冷却
  void set_capture_options(
      int min_views, double min_confidence, double min_diversity, int cooldown_ms);
  /// \brief 最少采集视角数
  int min_views() const { return min_views_; }
  /// \brief 自动采集最低置信度
  double min_confidence() const { return min_confidence_; }
  /// \brief 自动采集最小视角差异
  double min_diversity() const { return min_diversity_; }
  /// \brief 自动采集冷却毫秒
  int auto_cooldown_ms() const { return auto_cooldown_ms_; }

  /// \brief 设置求解器键值选项
  void set_solve_options(const std::map<std::string, std::string> &opts);
  /// \brief 当前求解选项
  const std::map<std::string, std::string> &solve_options() const { return solve_options_; }

  /// \brief 设置检测预览叠加开关
  void set_viz_options(
      bool corners, bool hull, bool conf, int marker_radius, bool aruco = true);
  /// \brief 是否画角点
  bool viz_corners() const { return viz_corners_; }
  /// \brief 是否画凸包
  bool viz_hull() const { return viz_hull_; }
  /// \brief 是否显示置信度
  bool viz_conf() const { return viz_conf_; }
  /// \brief 是否叠加 ArUco
  bool viz_aruco() const { return viz_aruco_; }
  /// \brief 角点/码绘制半径
  int viz_marker_radius() const { return viz_marker_radius_; }

  /// \brief 设置检测用内参（CameraInfo / YAML）；空 Mat 则用 guess_K
  void set_detect_intrinsics(
      const cv::Mat &camera_matrix,
      const cv::Mat &dist_coeffs,
      const std::string &model = "brown_conrady",
      double xi = 0.0);
  /// \brief 是否已有检测用内参
  bool has_detect_intrinsics() const { return !detect_K_.empty(); }
  /// \brief 当前检测用畸变模型
  std::string detect_camera_model() const { return detect_model_; }

  /// \brief 设置手眼 TF 坐标系
  void set_handeye_frames(const QString &base, const QString &gripper);
  /// \brief 设置手眼求解方法名
  void set_handeye_method(const QString &method);
  /// \brief 设置手眼用相机内参 YAML
  void set_camera_yaml(const QString &path);
  /// \brief 相机内参 YAML 路径
  QString camera_yaml() const { return camera_yaml_; }
  /// \brief 是否已指定相机 YAML
  bool has_camera_yaml() const { return !camera_yaml_.isEmpty(); }

  /// \brief 绑定 TF 位姿桥
  void set_tf_bridge(TfPoseBridge *bridge);

  /// \brief 扫描目录加载离线图片
  int load_image_dir(const QString &dir_path);
  /// \brief 加载离线位姿 CSV
  bool load_pose_csv(const QString &path, QString *error_out = nullptr);
  /// \brief CSV 位姿条数
  int pose_csv_count() const { return pose_csv_.size(); }

  /// \brief 写入在线最新 BGR 帧
  void set_live_bgr(const cv::Mat &bgr);
  /// \brief 是否已有在线帧
  bool has_live_frame() const { return !live_bgr_.empty(); }

  /// \brief 离线图片路径列表
  const QStringList &image_paths() const { return image_paths_; }
  /// \brief 当前离线图片索引
  int current_index() const { return current_index_; }
  /// \brief 切换当前离线图片索引
  void set_current_index(int index);
  /// \brief 当前帧路径或 ros:// 标识
  QString current_path() const;

  /// \brief 同步检测当前帧（离线/测试）；成功时 preview 含角点叠加
  bool detect_current(
      QImage *preview_out, QString *error_out = nullptr, bool fast = false);

  /// \brief 后台检测：立即返回，结果经 detect_started / detect_finished 回调
  /// \param fast 实时预览用快速路径；手动「检测」用 false
  void request_detect(bool fast = false);
  /// \brief 后台检测是否忙
  bool detect_busy() const { return detect_busy_.load(); }
  /// \brief 最近一次检出的面数（三面靶）
  int last_faces_found() const { return last_faces_found_; }
  /// \brief 最近一次有效检测的图像点数（失败为 0）
  int last_point_count() const;

  /// \brief 当前帧是否有有效检测
  bool has_current_detection() const { return has_detection_; }
  /// \brief 最近检测置信度
  double last_confidence() const { return last_confidence_; }
  /// \brief 最近 ArUco 轴位姿平均重投影误差（px）；无有效值时 < 0
  double last_aruco_reproj_px() const { return last_aruco_reproj_px_; }
  /// \brief 最近检测叠加预览
  QImage last_preview() const { return last_preview_; }

  /// \brief 高置信且与已采集姿态足够不同时自动入库（在线采集）
  bool try_auto_capture(
      double min_confidence, double min_diversity, QString *error_out = nullptr);

  /// \brief 将当前检测加入观测批次
  bool add_current_observation(QString *error_out = nullptr);
  /// \brief 删除指定行观测
  void remove_observation(int row);
  /// \brief 清空全部观测
  void clear_observations();

  /// \brief 当前观测批次
  const core::ObservationBatch &batch() const { return batch_; }
  /// \brief 观测条数
  int observation_count() const { return static_cast<int>(batch_.items.size()); }

  /// \brief 调用注册表标定器求解
  bool solve(QString *error_out = nullptr);
  /// \brief 最近一次标定结果
  const core::CalibrationResult &last_result() const { return last_result_; }
  /// \brief 是否有成功结果
  bool has_result() const { return last_result_.success; }

  /// \brief 导出内参或手眼外参 YAML
  bool export_yaml(const QString &path, QString *error_out = nullptr) const;
  /// \brief 导出结果文件夹：内参/外参 YAML、会话配置、采集原图与检测叠加图
  bool export_bundle(const QString &dir_path, QString *error_out = nullptr) const;

  /// \brief 组装求解配置字典
  std::map<std::string, std::string> solve_config_map() const;
  /// \brief 当前帧转为 QImage
  QImage load_current_qimage() const;
  /// \brief 记录 ROS 图像话题名（展示用）
  void set_ros_topic_name(const QString &topic);

  /// \brief 结果父坐标系名
  QString result_parent_frame() const;
  /// \brief 结果子坐标系名
  QString result_child_frame() const;

signals:
  /// \brief 离线图片列表变化
  void images_changed();
  /// \brief 当前帧变化
  void current_changed();
  /// \brief 观测列表变化
  void observations_changed();
  /// \brief 标定结果变化
  void result_changed();
  /// \brief 后台检测开始
  void detect_started();
  /// \brief 后台检测结束
  void detect_finished(bool ok, const QString &error);

private:
  cv::Mat current_bgr() const;
  bool attach_pose_to_observation(core::Observation *obs, QString *error_out);
  void start_detect_job(bool fast);

  QString calibrator_id_ = QStringLiteral("cam_intrinsics");
  SourceMode source_mode_ = SourceMode::Offline;
  PoseSource pose_source_ = PoseSource::None;

  int squares_x_ = 9;
  int squares_y_ = 6;
  double square_length_m_ = 0.025;

  int min_views_ = 12;
  double min_confidence_ = 0.55;
  double min_diversity_ = 0.12;
  int auto_cooldown_ms_ = 900;
  std::map<std::string, std::string> solve_options_;
  bool viz_corners_ = true;
  bool viz_hull_ = true;
  bool viz_conf_ = true;
  bool viz_aruco_ = true;
  int viz_marker_radius_ = 4;

  QString camera_yaml_;
  cv::Mat detect_K_;   ///< 检测叠加坐标系用（CameraInfo / YAML）
  cv::Mat detect_D_;
  std::string detect_model_ = "brown_conrady";
  double detect_xi_ = 0.0;
  QString base_frame_ = QStringLiteral("base");
  QString gripper_frame_ = QStringLiteral("tool0");
  QString handeye_method_ = QStringLiteral("tsai");

  QStringList image_paths_;
  int current_index_ = -1;

  cv::Mat live_bgr_;
  QString ros_topic_name_;
  int live_seq_ = 0;

  bool has_detection_ = false;
  core::Correspondence current_corr_;
  int detect_width_ = 0;
  int detect_height_ = 0;
  double last_confidence_ = 0.0;
  double last_aruco_reproj_px_ = -1.0;
  int last_faces_found_ = 0;
  QImage last_preview_;

  std::atomic<bool> detect_busy_{false};
  std::atomic<uint64_t> detect_epoch_{0};
  std::thread detect_thread_;
  bool pending_detect_ = false;
  bool pending_fast_ = true;

  struct ViewFingerprint {
    double area_ratio = 0.0;
    double cx = 0.5;
    double cy = 0.5;
    double tilt_deg = 0.0;
  };
  ViewFingerprint last_fp_{};
  std::vector<ViewFingerprint> captured_fps_;

  PoseCsvStore pose_csv_;
  TfPoseBridge *tf_bridge_ = nullptr;

  core::ObservationBatch batch_;
  core::CalibrationResult last_result_;

  struct CapturedView {
    cv::Mat bgr;      ///< 采集时原图
    QImage overlay;   ///< 检测叠加预览
  };
  std::vector<CapturedView> captured_views_;

  ViewFingerprint fingerprint_from_corners(
      const std::vector<cv::Point2f> &corners, int width, int height) const;
  double confidence_from_corners(
      const std::vector<cv::Point2f> &corners, int width, int height) const;
  bool is_diverse_enough(const ViewFingerprint &fp, double min_diversity) const;
};
}  // namespace gui
}  // namespace hs_calib
