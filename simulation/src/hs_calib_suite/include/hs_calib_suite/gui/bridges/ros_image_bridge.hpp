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
#include <sensor_msgs/msg/image.hpp>

namespace hs_calib {
namespace gui {

/// \brief GUI 内嵌 ROS 图像桥：列话题、订阅、缓存最新 BGR 帧
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

  /// \brief 订阅图像话题（keep_last 1）
  void subscribe(const QString &topic);

  /// \brief 取消订阅
  void unsubscribe();

  /// \brief 当前已订阅话题名
  QString subscribed_topic() const { return subscribed_topic_; }

  /// \brief 取最新一帧 BGR 拷贝；无帧返回空 Mat
  cv::Mat latest_bgr() const;

  /// \brief 是否已有缓存帧
  bool has_frame() const;

  /// \brief 泵一次 ROS 回调（由 QTimer 调用）
  void spin_some();

signals:
  /// \brief 收到新帧（仅通知；取图用 latest_bgr）
  void frame_received();

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  QString subscribed_topic_;

  mutable std::mutex mutex_;
  cv::Mat latest_bgr_;
  bool has_frame_ = false;
  bool logged_first_frame_ = false;
};

}  // namespace gui
}  // namespace hs_calib
