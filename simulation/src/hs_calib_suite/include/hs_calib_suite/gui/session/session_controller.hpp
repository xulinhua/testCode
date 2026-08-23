#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QImage>
#include <QObject>
#include <QString>
#include <QStringList>

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/detectors/board_type_identifier.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_capture_filter.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_collector_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_session_state.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"
#include "hs_calib_suite/core/types/types.hpp"
#include "hs_calib_suite/gui/bridges/ros_bag_frame_reader.hpp"
#include "hs_calib_suite/gui/bridges/ros_bag_stereo_frame_reader.hpp"
#include "hs_calib_suite/gui/data/pose_csv_store.hpp"

namespace hs_calib {
namespace gui {

class TfPoseBridge;

/// \brief 成对采集记录（左右各一条观测，共享 pair_id）
struct StereoPairRecord {
  int pair_id = 0;
  std::string left_source_path;
  std::string right_source_path;
  std::string left_image_path;   ///< 离线/缓存原图（校正预览用）
  std::string right_image_path;
  int64_t timestamp_delta_ms = 0;
};

enum class SourceMode {
  Offline = 0,   ///< 离线图片目录
  RosTopic = 1,  ///< 在线 ROS 图像话题
  RosBag = 2,    ///< rosbag2 回放导出
};

enum class PoseSource {
  None = 0,  ///< 单目内参不需要
  Csv = 1,
  Tf = 2,
};

/// \brief 检测调试子模式（与首页磁贴 ID 对应；正式标定为 None）
enum class DetectLabMode {
  None = 0,
  Identify,  ///< detect_lab_identify
  Partial,   ///< detect_lab
  Full,      ///< detect_lab_full
};

/// \brief 由任务 ID 解析调试模式
inline DetectLabMode detect_lab_mode_from_task_id(const QString &id) {
  if (id == QStringLiteral("detect_lab_identify")) {
    return DetectLabMode::Identify;
  }
  if (id == QStringLiteral("detect_lab_full")) {
    return DetectLabMode::Full;
  }
  if (id == QStringLiteral("detect_lab")) {
    return DetectLabMode::Partial;
  }
  return DetectLabMode::None;
}

inline QString detect_lab_task_id(DetectLabMode mode) {
  switch (mode) {
    case DetectLabMode::Identify:
      return QStringLiteral("detect_lab_identify");
    case DetectLabMode::Partial:
      return QStringLiteral("detect_lab");
    case DetectLabMode::Full:
      return QStringLiteral("detect_lab_full");
    case DetectLabMode::None:
    default:
      return {};
  }
}

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
  /// \brief 是否启用 Tier4 内参流水线（训练/评估分流、RANSAC 等）
  bool uses_tier4_intrinsics() const;
  /// \brief 设置内参 profile（classic 或 general/c1/ceres/c2）
  void set_intrinsics_profile_id(const std::string &profile_id);
  std::string intrinsics_profile_id() const;
  /// \brief 是否为双目各自内参
  bool is_stereo_intrinsics() const;
  /// \brief 是否为双目相对外参
  bool is_stereo_extrinsics() const;
  /// \brief 是否为需左右侧标记的双目流程（内参分侧 / 外参成对）
  bool is_stereo_side_tagged() const;
  /// \brief 双目内参：独立左右会话与成对采集
  bool uses_stereo_dual_session() const;
  /// \brief 采集模式：paired / left / right（补采）
  QString stereo_capture_mode() const;
  /// \brief 成对记录
  const std::vector<StereoPairRecord> &stereo_pairs() const { return stereo_pairs_; }
  int stereo_pair_count() const { return static_cast<int>(stereo_pairs_.size()); }
  int stereo_rectify_pair_count() const;
  int stereo_left_sample_count() const;
  int stereo_right_sample_count() const;
  int64_t last_stereo_sync_delta_ms() const { return last_stereo_sync_delta_ms_; }
  /// \brief 分侧 Tier4 状态
  core::IntrinsicsSessionState &intrinsics_state_for_side(const QString &side);
  const core::IntrinsicsSessionState &intrinsics_state_for_side(const QString &side) const;
  /// \brief 写入在线立体帧（带同步 Δt）
  void set_live_stereo_bgr(
      const cv::Mat &left, const cv::Mat &right, int64_t sync_delta_ms);
  void set_live_stereo_bgr(
      cv::Mat &&left, cv::Mat &&right, int64_t sync_delta_ms);
  /// \brief 成对采集（主流程）
  bool capture_paired_observation(QString *error_out = nullptr);
  /// \brief 扫描 left/right 子目录或 *_L/*_R 命名
  int load_stereo_image_dir(
      const QString &root_or_left_dir, const QString &right_dir = QString());
  /// \brief Bag 双话题时间戳配对加载
  int load_stereo_rosbag(
      const QString &bag_uri,
      const QString &left_topic,
      const QString &right_topic,
      int max_pairs = 500,
      QString *error_out = nullptr);
  /// \brief 应用后台解码完成的立体 bag 帧（主线程）
  int apply_loaded_stereo_bag(
      RosBagStereoFrameReader reader,
      const QString &left_topic,
      const QString &right_topic);
  const QStringList &stereo_left_image_paths() const { return stereo_left_paths_; }
  const QStringList &stereo_right_image_paths() const { return stereo_right_paths_; }
  int stereo_pair_index() const { return stereo_pair_index_; }
  void set_stereo_pair_index(int index);
  /// \brief 左右目检测预览
  QImage last_stereo_left_preview() const;
  QImage last_stereo_right_preview() const;
  bool stereo_left_has_detection() const;
  bool stereo_right_has_detection() const;
  /// \brief 后台检测左右目当前帧
  void request_stereo_detect(bool fast = false);
  bool stereo_detect_busy() const { return stereo_detect_busy_.load(); }
  /// \brief 成对自动采集（diversity + Δt）
  bool try_auto_capture_paired(
      double min_confidence, double min_diversity, QString *error_out = nullptr);
  /// \brief 是否已有立体校正参数
  bool has_stereo_rectified() const;
  bool has_stereo_rectify_maps() const { return has_stereo_rectify_maps_; }
  /// \brief 标定成功后补建立体校正（兼容旧会话）
  bool ensure_stereo_rectification();
  void backfill_stereo_pair_image_paths();
  bool load_stereo_pair_bgr(int index, cv::Mat *left, cv::Mat *right) const;
  /// \brief 校正后左右预览（含极线）；需先标定成功
  bool stereo_rectified_preview(QImage *left_out, QImage *right_out) const;
  int stereo_loaded_pair_count() const;
  QString stereo_pair_brightness_hint() const;
  /// \brief 是否为直角三面标定器
  bool is_trihedral() const;

  /// \brief 设置检测调试子模式（正式标定为 None）
  void set_detect_lab_mode(DetectLabMode mode);
  /// \brief 当前检测调试子模式
  DetectLabMode detect_lab_mode() const { return detect_lab_mode_; }
  /// \brief 是否处于任一检测调试模式
  bool is_detect_lab() const { return detect_lab_mode_ != DetectLabMode::None; }
  /// \brief 由任务 ID 同步调试模式（正式 ID 则清为 None）
  void sync_detect_lab_mode_from_task_id(const QString &task_id);

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
  /// \brief 清空检测用内参（回退 guess_K）
  void clear_detect_intrinsics();
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
  /// \brief 应用已扫描的离线图片路径（主线程；扫描在后台完成）
  int apply_image_paths(const QString &dir_path, const QStringList &paths);
  /// \brief 直接加载 rosbag 图像话题到内存（不导出 PNG）
  int load_rosbag(
      const QString &bag_uri,
      const QString &topic,
      int max_frames = 500,
      QString *error_out = nullptr);
  /// \brief 应用后台解码完成的 bag 帧（主线程）
  int apply_loaded_bag(RosBagFrameReader reader, const QString &topic);
  /// \brief 加载离线位姿 CSV
  bool load_pose_csv(const QString &path, QString *error_out = nullptr);
  /// \brief CSV 位姿条数
  int pose_csv_count() const { return pose_csv_.size(); }

  /// \brief 写入在线最新 BGR 帧
  void set_live_bgr(const cv::Mat &bgr);
  void set_live_bgr(cv::Mat &&bgr);
  /// \brief 后台将 live BGR 转为 QImage（不阻塞界面线程）
  void schedule_live_preview_update();
  /// \brief 后台将立体 live BGR 转为 QImage（不阻塞界面线程）
  void schedule_stereo_live_preview_update();
  /// \brief 后台加载当前离线帧并转 QImage
  void schedule_offline_preview_update();
  /// \brief 最近一次异步生成的裸图预览
  QImage cached_live_preview_qimage() const;
  QImage cached_stereo_live_preview_left() const;
  QImage cached_stereo_live_preview_right() const;
  QImage cached_offline_preview_qimage() const;
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
  /// \brief 实时预览检测间隔（ms）；无检出且开启跳帧时降频，与 Tier4 一致
  int live_detect_interval_ms() const;
  /// \brief 后台标定板类型识别（与 request_detect 互斥；忽略板尺寸）
  void request_identify(const core::BoardTypeIdentifyOptions &options = {});
  /// \brief 取消排队中的检测/识别，并作废进行中的任务（冻结预览时调用）
  void cancel_pending_detect();
  /// \brief 离开采集页时清空在线帧与预览缓存
  void clear_live_ros_frames();
  /// \brief 后台检测或识别是否忙
  bool detect_busy() const { return detect_busy_.load(); }
  /// \brief 最近一次类型识别结果（按 score 降序）
  const std::vector<core::BoardTypeHypothesis> &last_identify_ranked() const {
    return last_identify_ranked_;
  }
  /// \brief 最近一次识别摘要文案
  QString last_identify_message() const { return last_identify_message_; }
  /// \brief 导出最近一次识别结果为 JSON 报告
  bool export_identify_json(const QString &path, QString *error_out = nullptr) const;
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
  /// \brief 内参：按训练/评估集删除
  void remove_intrinsics_sample(core::IntrinsicsSampleSplit split, int row);
  /// \brief 清空全部观测
  void clear_observations();
  /// \brief 是否存在可清空的观测（与列表展示一致）
  bool has_observations() const;
  /// \brief 工作台列表用的训练集观测
  core::ObservationBatch training_observations() const;

  /// \brief 当前观测批次（内参任务为训练集）
  const core::ObservationBatch &batch() const { return batch_; }
  /// \brief 内参评估集
  core::ObservationBatch evaluation_batch() const;
  /// \brief 观测条数（训练集）
  int observation_count() const;
  /// \brief 是否满足求解/标定按钮启用条件
  bool can_solve() const;
  /// \brief 离线模式：后台检测并入库全部已加载图片（无观测时）
  void request_offline_batch_ingest();
  bool offline_ingest_busy() const { return offline_ingest_busy_.load(); }
  /// \brief 训练/评估集条数
  int training_sample_count() const;
  int evaluation_sample_count() const;

  /// \brief Tier4 内参会话状态（仅内参任务有效）
  core::IntrinsicsSessionState &intrinsics_state();
  const core::IntrinsicsSessionState &intrinsics_state() const;
  /// \brief 最近一帧检测指标
  const core::BoardFrameMetrics &last_board_metrics() const {
    return last_board_metrics_;
  }
  /// \brief 覆盖求解器（opencv / ceres），写入 solve_options
  void set_intrinsics_solver_kind(const std::string &solver);
  std::string intrinsics_solver_kind() const;

  /// \brief 评估已标定模型（不重新优化）
  bool evaluate_calibration(QString *error_out = nullptr);

  void set_intrinsics_image_view_mode(IntrinsicsImageViewMode mode);
  IntrinsicsImageViewMode intrinsics_image_view_mode() const;
  void set_intrinsics_viz_options(const IntrinsicsVizOptions &options);
  IntrinsicsVizOptions intrinsics_viz_options() const;
  void clear_intrinsics_linearity_heatmap();
  void set_intrinsics_browse_sample(int index, bool evaluation_set);
  int intrinsics_browse_sample_index() const;
  bool intrinsics_browse_is_evaluation() const;
  void clear_intrinsics_browse();
  QImage decorate_intrinsics_preview(const QImage &preview, bool lightweight = false) const;
  bool intrinsics_browse_preview(QImage *out) const;

  /// \brief 调用注册表标定器求解（同步，供后台线程调用）
  bool solve(QString *error_out = nullptr);
  /// \brief 后台标定：立即返回，结果经 solve_started / solve_progress / solve_finished
  void request_solve();
  /// \brief 后台标定是否忙
  bool solve_busy() const { return solve_busy_.load(); }
  /// \brief 最近一次标定结果
  const core::CalibrationResult &last_result() const { return last_result_; }
  /// \brief 是否有成功结果
  bool has_result() const { return last_result_.success; }

  /// \brief 导出内参或手眼外参 YAML
  bool export_yaml(const QString &path, QString *error_out = nullptr) const;
  /// \brief 导出结果文件夹：内参/外参 YAML、标定设置、采集原图与检测叠加图
  bool export_bundle(const QString &dir_path, QString *error_out = nullptr) const;

  /// \brief 组装求解配置字典
  std::map<std::string, std::string> solve_config_map() const;
  /// \brief 当前帧转为 QImage
  QImage load_current_qimage() const;
  /// \brief 记录 ROS 图像话题名（展示用）
  void set_ros_topic_name(const QString &topic);
  /// \brief 仅更新双目 ROS 话题名（不触发 configure 引擎）
  void set_stereo_ros_topics(const QString &left_topic, const QString &right_topic);

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
  /// \brief 内参采集/指标变化
  void intrinsics_state_changed();
  /// \brief 后台检测开始
  void detect_started();
  /// \brief 后台检测结束
  void detect_finished(bool ok, const QString &error);
  /// \brief 双目左右检测完成
  void stereo_detect_finished(bool ok, const QString &error);
  /// \brief 异步裸图预览就绪（无检出时刷新画面）
  void live_preview_updated();
  void stereo_live_preview_updated();
  void offline_preview_updated();
  /// \brief 离线批量入库开始（total=待处理帧/对数）
  void offline_ingest_started(int total);
  /// \brief 离线批量入库结束（added=成功入库，skipped=检测失败或重复）
  void offline_ingest_finished(int added, int skipped);
  /// \brief 后台类型识别开始
  void identify_started();
  /// \brief 后台类型识别结束
  void identify_finished(bool ok, const QString &error);
  /// \brief 后台标定开始
  void solve_started();
  /// \brief 后台标定进度（0–100）
  void solve_progress(int percent, const QString &message);
  /// \brief 后台标定结束
  void solve_finished(bool ok, const QString &error);

private:
  bool validate_solve_preconditions(QString *error_out) const;
  void start_solve_job();
  cv::Mat current_bgr() const;
  bool attach_pose_to_observation(core::Observation *obs, QString *error_out);
  void start_detect_job(bool fast);
  void start_identify_job(const core::BoardTypeIdentifyOptions &options);
  void clear_loaded_source_data();

  QString calibrator_id_ = QStringLiteral("cam_intrinsics");
  DetectLabMode detect_lab_mode_ = DetectLabMode::None;
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
  mutable std::mutex live_preview_mutex_;
  QImage cached_live_preview_qimage_;
  std::atomic<bool> live_preview_busy_{false};
  mutable std::mutex stereo_live_preview_mutex_;
  QImage cached_stereo_live_left_;
  QImage cached_stereo_live_right_;
  std::atomic<bool> stereo_live_preview_busy_{false};
  mutable std::mutex offline_preview_mutex_;
  QImage cached_offline_preview_qimage_;
  std::atomic<bool> offline_preview_busy_{false};
  int offline_preview_index_ = -1;
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
  std::vector<core::BoardTypeHypothesis> last_identify_ranked_;
  QString last_identify_message_;

  std::atomic<bool> detect_busy_{false};
  std::atomic<uint64_t> detect_epoch_{0};
  std::thread detect_thread_;
  bool pending_detect_ = false;
  bool pending_fast_ = true;

  std::atomic<bool> solve_busy_{false};
  std::thread solve_thread_;

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
    cv::Mat bgr;           ///< 采集时原图（内存，小数据集回退）
    QString image_path;    ///< 采集时原图磁盘缓存（在线 ROS 等）
    QImage overlay;        ///< 检测叠加预览
  };
  std::vector<CapturedView> captured_views_;

  QString capture_cache_dir_;
  QString ensure_capture_cache_dir();
  void clear_capture_cache();
  QString save_capture_original(const cv::Mat &bgr, int index);
  void fill_capture_view(CapturedView *view);
  static QString resolve_obs_source_path(const QString &source);

  ViewFingerprint fingerprint_from_corners(
      const std::vector<cv::Point2f> &corners, int width, int height) const;
  double confidence_from_corners(
      const std::vector<cv::Point2f> &corners, int width, int height) const;
  bool is_diverse_enough(const ViewFingerprint &fp, double min_diversity) const;
  void refresh_provisional_intrinsics();
  void request_provisional_intrinsics_refresh(bool force = false);
  bool uses_intrinsics_capture_filter() const;
  void configure_intrinsics_engine();
  void sync_batch_from_intrinsics();
  void update_board_metrics_after_detect();
  bool tier4_preview_needs_overlay() const;
  double compute_pixel_speed() const;
  bool detect_on_bgr(const cv::Mat &bgr, bool fast, QImage *preview_out, QString *error_out);
  bool add_side_observation(
      const std::string &side,
      const core::Correspondence &corr,
      int width,
      int height,
      const QString &path,
      QString *error_out);
  bool add_observation_from_detect(
      const QString &path,
      const core::Correspondence &corr,
      int width,
      int height,
      const QImage &overlay,
      QString *error_out);
  void configure_stereo_intrinsics_states();
  bool solve_stereo_intrinsics(QString *error_out);
  void merge_stereo_calib_results(
      const core::CalibrationResult &left_r,
      const core::CalibrationResult &right_r,
      core::CalibrationResult *out);
  bool append_stereo_rectified_meta(core::CalibrationResult *result);
  void rebuild_stereo_rectify_maps();
  RosBagStereoFrameReader stereo_bag_reader_;

  cv::Mat stereo_map1_x_;
  cv::Mat stereo_map1_y_;
  cv::Mat stereo_map2_x_;
  cv::Mat stereo_map2_y_;
  cv::Size stereo_rect_size_;
  bool has_stereo_rectify_maps_ = false;

  core::IntrinsicsSessionState intrinsics_state_;
  core::IntrinsicsSessionState intrinsics_left_state_;
  core::IntrinsicsSessionState intrinsics_right_state_;
  std::vector<StereoPairRecord> stereo_pairs_;
  int next_stereo_pair_id_ = 1;
  cv::Mat live_left_bgr_;
  cv::Mat live_right_bgr_;
  int64_t last_stereo_sync_delta_ms_ = -1;
  QStringList stereo_left_paths_;
  QStringList stereo_right_paths_;
  int stereo_pair_index_ = -1;
  struct StereoSideDetect {
    bool has = false;
    core::Correspondence corr;
    int width = 0;
    int height = 0;
    double confidence = 0.0;
    QImage preview;
    cv::Mat bgr;
    core::BoardFrameFingerprint fp{};
  };
  StereoSideDetect stereo_left_detect_;
  StereoSideDetect stereo_right_detect_;
  std::atomic<bool> stereo_detect_busy_{false};
  std::atomic<uint64_t> stereo_detect_epoch_{0};
  std::atomic<bool> offline_ingest_busy_{false};
  core::BoardFrameMetrics last_board_metrics_;
  cv::Point2f last_frame_centroid_{0.5f, 0.5f};
  bool has_last_frame_centroid_ = false;

  IntrinsicsImageViewMode intrinsics_view_mode_ = IntrinsicsImageViewMode::Source;
  IntrinsicsVizOptions intrinsics_viz_options_;
  std::vector<float> intrinsics_linearity_grid_;
  int intrinsics_browse_index_ = -1;
  bool intrinsics_browse_eval_ = false;
  cv::Rect chess_track_roi_;
  int chess_lost_frames_ = 0;
  bool has_chess_track_roi_ = false;

  core::ProvisionalIntrinsics provisional_intrinsics_;
  std::atomic<bool> provisional_refresh_busy_{false};
  std::atomic<bool> provisional_refresh_dirty_{false};
  int last_provisional_sample_count_ = 0;
  qint64 last_provisional_refresh_ms_ = 0;
  RosBagFrameReader bag_reader_;
};
}  // namespace gui
}  // namespace hs_calib
