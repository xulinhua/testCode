#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hs_calib {
namespace gui {

/// \brief GUI 内嵌 ROS 图像桥：列话题、订阅、缓存最新 BGR 帧与 CameraInfo 内参
class RosImageBridge : public QObject {
  Q_OBJECT

public:
  /// \brief 构造：创建图像桥节点（需 rclcpp 就绪）
  explicit RosImageBridge(QObject *parent = nullptr);
  /// \brief 析构：退订并释放节点
  ~RosImageBridge() override;

  /// \brief 是否已初始化 rclcpp 节点
  bool is_ready() const { return node_ != nullptr && rclcpp::ok(); }

  /// \brief 列出 sensor_msgs/Image 话题（排除名称含 depth/disparity）
  QStringList list_image_topics() const;

  /// \brief 列出 sensor_msgs/CameraInfo 话题
  QStringList list_camera_info_topics() const;

  /// \brief 订阅图像话题（keep_last 1）
  void subscribe(const QString &topic);

  /// \brief 订阅 CameraInfo（用于画坐标系 / PnP）
  void subscribe_camera_info(const QString &topic);

  /// \brief 取消图像订阅
  void unsubscribe();

  /// \brief 取消 CameraInfo 订阅
  void unsubscribe_camera_info();

  /// \brief 当前已订阅图像话题名
  QString subscribed_topic() const { return subscribed_topic_; }

  /// \brief 当前已订阅 CameraInfo 话题名
  QString subscribed_camera_info_topic() const { return camera_info_topic_; }

  /// \brief 取最新一帧 BGR 拷贝；无帧返回空 Mat
  cv::Mat latest_bgr() const;

  /// \brief 取走最新 BGR 帧（桥内缓存置空，由调用方持有；避免重复 clone）
  cv::Mat take_latest_bgr();

  /// \brief 是否已有缓存帧
  bool has_frame() const;

  /// \brief 是否已收到有效 CameraInfo（含 K）
  bool has_camera_info() const;

  /// \brief 最新 3×3 相机矩阵（CV_64F）；无则空
  cv::Mat camera_matrix() const;

  /// \brief 最新畸变系数（CV_64F 行向量）；可能为空
  cv::Mat dist_coeffs() const;

  /// \brief CameraInfo.distortion_model（如 plumb_bob / equidistant）
  QString distortion_model() const;

  /// \brief 泵一次 ROS 回调（由 QTimer 调用）
  void spin_some();

signals:
  /// \brief 收到新帧（仅通知；取图用 latest_bgr）
  void frame_received();
  /// \brief 收到 / 更新 CameraInfo
  void camera_info_received();

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  QString subscribed_topic_;
  QString camera_info_topic_;

  mutable std::mutex mutex_;
  cv::Mat latest_bgr_;
  bool has_frame_ = false;
  bool logged_first_frame_ = false;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  QString distortion_model_;
  bool has_camera_info_ = false;
  bool logged_first_info_ = false;
};

}  // namespace gui
}  // namespace hs_calib
