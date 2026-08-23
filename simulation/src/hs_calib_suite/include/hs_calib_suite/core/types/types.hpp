#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hs_calib {
namespace core {

/// \brief 算法层图像帧视图（不拥有像素；与 cv::Mat 桥接见 cv_bridge_local）
struct ImageFrame {
  int width = 0;
  int height = 0;
  int channels = 0;
  size_t step = 0;  ///< 行字节数；0 表示按 width*channels 紧凑行
  const uint8_t *data = nullptr;  ///< 像素指针（调用方保活）
  std::string encoding;           ///< 如 bgr8 / mono8
};

/// \brief 二维像点与三维物点的对应
struct Correspondence {
  Eigen::MatrixXd image_points;   ///< Nx2 像素坐标
  Eigen::MatrixXd object_points;  ///< Nx3 物点（靶标坐标系）
  std::vector<int> ids;           ///< 特征 ID（角点 / Tag）
};

/// \brief 单帧观测
struct Observation {
  double timestamp_sec = 0.0;               ///< 时间戳（秒）
  std::string frame_id;                     ///< 传感器坐标系
  std::string source_path;                  ///< 离线图片路径（可选）
  int image_width = 0;                      ///< 图像宽
  int image_height = 0;                     ///< 图像高
  std::vector<Correspondence> correspondences;
  /// \brief 采集时 PnP 板位姿（内参流水线写入，可选）
  bool has_board_pose = false;
  Eigen::Vector3d board_rvec = Eigen::Vector3d::Zero();
  Eigen::Vector3d board_tvec = Eigen::Vector3d::Zero();
  double board_reproj_rms = -1.0;
  double board_reproj_max = -1.0;
  double board_center_x_norm = 0.5;
  double board_center_y_norm = 0.5;
  double board_tilt_deg = 0.0;
  /// \brief 是否含机械臂位姿（手眼用）
  bool has_base_gripper = false;
  /// \brief T_base_gripper：把 gripper 系点变到 base（OpenCV gripper2base）
  Eigen::Matrix4d T_base_gripper = Eigen::Matrix4d::Identity();
};

/// \brief 一次标定会话的观测集合
struct ObservationBatch {
  std::vector<Observation> items;  ///< 观测列表
  std::string notes;               ///< 备注
};

/// \brief 算法层标定结果（不含 ROS 类型）
struct CalibrationResult {
  bool success = false;                       ///< 是否成功
  float score = 0.f;                          ///< 质量分数（语义由标定器定义）
  std::string message;                        ///< 人类可读说明
  /// parent_frame -> child_frame -> 4x4 变换
  std::map<std::string, std::map<std::string, Eigen::Matrix4d>> transforms;
  std::map<std::string, double> metrics;      ///< 误差等指标
  std::map<std::string, std::string> intrinsics_meta;  ///< 内参元数据
};

/// \brief 标定器描述信息（供 UI / 服务查询）
struct CalibratorInfo {
  std::string calibrator_id;                  ///< 唯一 ID
  std::string display_name;                   ///< 显示名
  std::string category;                       ///< 分类（内参/外参/手眼等）
  std::vector<std::string> required_frames;   ///< 所需坐标系
  std::vector<std::string> supported_targets; ///< 支持靶标类型
};

}  // namespace core
}  // namespace hs_calib
