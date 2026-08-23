#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

#include <QObject>
#include <QString>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hs_calib {
namespace gui {

/// \brief 时间戳匹配后的立体帧对（BGR）
struct StereoMatchedFrames {
  cv::Mat left_bgr;
  cv::Mat right_bgr;
  int64_t stamp_ns = 0;
  int64_t delta_ms = 0;
  bool valid() const { return !left_bgr.empty() && !right_bgr.empty(); }
};

/// \brief 双 ROS 图像话题桥：环形缓冲 + 时间戳最近邻配对（参考 LuminaCalib）
class RosStereoImageBridge : public QObject {
  Q_OBJECT

public:
  explicit RosStereoImageBridge(QObject *parent = nullptr);
  ~RosStereoImageBridge() override;

  bool is_ready() const { return node_ != nullptr && rclcpp::ok(); }

  /// \brief 订阅左右图像话题；\a max_match_delta_ms 为可接受的最大时间差
  void subscribe(
      const QString &left_topic,
      const QString &right_topic,
      int max_match_delta_ms = 30);

  void unsubscribe();

  QString left_topic() const { return left_topic_; }
  QString right_topic() const { return right_topic_; }
  int max_match_delta_ms() const { return max_match_delta_ms_; }

  cv::Mat latest_left_bgr() const;
  cv::Mat latest_right_bgr() const;
  /// \brief 取走最新帧（移动语义，避免 UI 线程额外 clone）
  cv::Mat take_latest_left_bgr();
  cv::Mat take_latest_right_bgr();
  bool has_left_frame() const;
  bool has_right_frame() const;

  /// \brief 当前最佳匹配对（拷贝，不消费缓冲）
  StereoMatchedFrames peek_matched_pair() const;

  /// \brief 取走当前最佳匹配对并清除已配对帧
  StereoMatchedFrames take_matched_pair();

  /// \brief 最近一次成功配对的 Δt（ms）；无配对时为 -1
  int64_t last_match_delta_ms() const;

  /// \brief 是否已有 Δt 在阈值内的有效配对
  bool has_synced_pair() const;

  void spin_some();

  rclcpp::Node::SharedPtr ros_node() const { return node_; }

signals:
  void stereo_frames_updated();
  void stereo_pair_matched(int64_t delta_ms);

private:
  struct StampedBgr {
    int64_t stamp_ns = 0;
    cv::Mat bgr;
  };

  void on_left_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_right_image(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat decode_bgr(const sensor_msgs::msg::Image::SharedPtr &msg) const;
  void push_buffer(std::deque<StampedBgr> *buf, StampedBgr frame);
  StereoMatchedFrames find_best_pair_locked() const;
  void notify_ui_throttled();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_right_;
  QString left_topic_;
  QString right_topic_;
  int max_match_delta_ms_ = 30;

  mutable std::mutex mutex_;
  std::deque<StampedBgr> left_buf_;
  std::deque<StampedBgr> right_buf_;
  cv::Mat latest_left_bgr_;
  cv::Mat latest_right_bgr_;
  bool has_left_ = false;
  bool has_right_ = false;
  StereoMatchedFrames last_matched_;
  int64_t last_match_delta_ms_ = -1;
  std::atomic<int64_t> last_ui_notify_ns_{0};
};

}  // namespace gui
}  // namespace hs_calib
