#include "hs_calib_suite/gui/bridges/ros_stereo_image_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <QMetaObject>

#include <QMetaObject>
#include <opencv2/imgproc.hpp>

namespace hs_calib {
namespace gui {

namespace {

int64_t stamp_ns(const sensor_msgs::msg::Image &msg) {
  return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
         static_cast<int64_t>(msg.header.stamp.nanosec);
}

constexpr size_t kMaxBufferFrames = 32;
constexpr int64_t kUiNotifyIntervalNs = 0;

}  // namespace

RosStereoImageBridge::RosStereoImageBridge(QObject *parent) : QObject(parent) {
  if (!rclcpp::ok()) {
    return;
  }
  node_ = std::make_shared<rclcpp::Node>("hs_calib_gui_stereo_image_bridge");
}

RosStereoImageBridge::~RosStereoImageBridge() {
  unsubscribe();
  node_.reset();
}

void RosStereoImageBridge::subscribe(
    const QString &left_topic, const QString &right_topic, int max_match_delta_ms) {
  unsubscribe();
  left_topic_ = left_topic;
  right_topic_ = right_topic;
  max_match_delta_ms_ = std::max(1, max_match_delta_ms);
  if (!is_ready() || left_topic.isEmpty() || right_topic.isEmpty()) {
    return;
  }
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  sub_left_ = node_->create_subscription<sensor_msgs::msg::Image>(
      left_topic.toStdString(), qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_left_image(msg); });
  sub_right_ = node_->create_subscription<sensor_msgs::msg::Image>(
      right_topic.toStdString(), qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_right_image(msg); });
  RCLCPP_INFO(
      node_->get_logger(), "stereo subscribe L=%s R=%s max_dt=%dms",
      left_topic.toStdString().c_str(), right_topic.toStdString().c_str(),
      max_match_delta_ms_);
}

void RosStereoImageBridge::unsubscribe() {
  sub_left_.reset();
  sub_right_.reset();
  left_topic_.clear();
  right_topic_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  left_buf_.clear();
  right_buf_.clear();
  latest_left_bgr_.release();
  latest_right_bgr_.release();
  has_left_ = false;
  has_right_ = false;
  last_matched_ = {};
  last_match_delta_ms_ = -1;
}

cv::Mat RosStereoImageBridge::latest_left_bgr() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_left_bgr_.empty() ? cv::Mat() : latest_left_bgr_.clone();
}

cv::Mat RosStereoImageBridge::latest_right_bgr() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_right_bgr_.empty() ? cv::Mat() : latest_right_bgr_.clone();
}

cv::Mat RosStereoImageBridge::take_latest_left_bgr() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::move(latest_left_bgr_);
}

cv::Mat RosStereoImageBridge::take_latest_right_bgr() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::move(latest_right_bgr_);
}

bool RosStereoImageBridge::has_left_frame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_left_ && !latest_left_bgr_.empty();
}

bool RosStereoImageBridge::has_right_frame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_right_ && !latest_right_bgr_.empty();
}

int64_t RosStereoImageBridge::last_match_delta_ms() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_match_delta_ms_;
}

bool RosStereoImageBridge::has_synced_pair() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_matched_.valid() && last_match_delta_ms_ >= 0 &&
         last_match_delta_ms_ <= max_match_delta_ms_;
}

StereoMatchedFrames RosStereoImageBridge::peek_matched_pair() const {
  std::lock_guard<std::mutex> lock(mutex_);
  StereoMatchedFrames out = last_matched_;
  if (!out.left_bgr.empty()) {
    out.left_bgr = out.left_bgr.clone();
  }
  if (!out.right_bgr.empty()) {
    out.right_bgr = out.right_bgr.clone();
  }
  return out;
}

StereoMatchedFrames RosStereoImageBridge::take_matched_pair() {
  std::lock_guard<std::mutex> lock(mutex_);
  StereoMatchedFrames out = std::move(last_matched_);
  last_matched_ = {};
  return out;
}

void RosStereoImageBridge::spin_some() {
  // 回调由 RosExecutorHub 后台 MultiThreadedExecutor 驱动
}

void RosStereoImageBridge::notify_ui_throttled() {
  if (kUiNotifyIntervalNs <= 0) {
    QMetaObject::invokeMethod(
        this, [this]() { emit stereo_frames_updated(); }, Qt::QueuedConnection);
    return;
  }
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  int64_t prev = last_ui_notify_ns_.load();
  while (now - prev >= kUiNotifyIntervalNs) {
    if (last_ui_notify_ns_.compare_exchange_weak(prev, now)) {
      QMetaObject::invokeMethod(
          this, [this]() { emit stereo_frames_updated(); }, Qt::QueuedConnection);
      break;
    }
  }
}

cv::Mat RosStereoImageBridge::decode_bgr(
    const sensor_msgs::msg::Image::SharedPtr &msg) const {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception &) {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  cv::Mat bgr;
  if (cv_ptr->image.channels() == 1) {
    cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_GRAY2BGR);
  } else if (cv_ptr->encoding == "rgb8") {
    cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
  } else {
    bgr = cv_ptr->image.clone();
  }
  return bgr;
}

void RosStereoImageBridge::push_buffer(std::deque<StampedBgr> *buf, StampedBgr frame) {
  buf->push_back(std::move(frame));
  while (buf->size() > kMaxBufferFrames) {
    buf->pop_front();
  }
}

StereoMatchedFrames RosStereoImageBridge::find_best_pair_locked() const {
  StereoMatchedFrames best;
  if (left_buf_.empty() || right_buf_.empty()) {
    return best;
  }
  const int64_t max_delta_ns =
      static_cast<int64_t>(max_match_delta_ms_) * 1000000LL;
  int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
  const StampedBgr *best_l = nullptr;
  const StampedBgr *best_r = nullptr;

  for (const auto &l : left_buf_) {
    for (const auto &r : right_buf_) {
      const int64_t d = std::llabs(l.stamp_ns - r.stamp_ns);
      if (d <= max_delta_ns && d < best_delta_ns) {
        best_delta_ns = d;
        best_l = &l;
        best_r = &r;
      }
    }
  }
  if (best_l == nullptr || best_r == nullptr) {
    return best;
  }
  best.left_bgr = best_l->bgr.clone();
  best.right_bgr = best_r->bgr.clone();
  best.stamp_ns = (best_l->stamp_ns + best_r->stamp_ns) / 2;
  best.delta_ms = best_delta_ns / 1000000LL;
  return best;
}

void RosStereoImageBridge::on_left_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (!msg) {
    return;
  }
  try {
    StampedBgr frame;
    frame.stamp_ns = stamp_ns(*msg);
    frame.bgr = decode_bgr(msg);
    int64_t delta_ms = -1;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_left_bgr_ = frame.bgr;
      has_left_ = true;
      push_buffer(&left_buf_, {frame.stamp_ns, frame.bgr.clone()});
      const auto pair = find_best_pair_locked();
      if (pair.valid()) {
        last_matched_ = pair;
        last_match_delta_ms_ = pair.delta_ms;
        matched = true;
        delta_ms = pair.delta_ms;
      }
    }
    notify_ui_throttled();
    if (matched) {
      QMetaObject::invokeMethod(
          this,
          [this, delta_ms]() { emit stereo_pair_matched(delta_ms); },
          Qt::QueuedConnection);
    }
  } catch (const std::exception &) {
    // ignore decode errors
  }
}

void RosStereoImageBridge::on_right_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (!msg) {
    return;
  }
  try {
    StampedBgr frame;
    frame.stamp_ns = stamp_ns(*msg);
    frame.bgr = decode_bgr(msg);
    int64_t delta_ms = -1;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_right_bgr_ = frame.bgr;
      has_right_ = true;
      push_buffer(&right_buf_, {frame.stamp_ns, frame.bgr.clone()});
      const auto pair = find_best_pair_locked();
      if (pair.valid()) {
        last_matched_ = pair;
        last_match_delta_ms_ = pair.delta_ms;
        matched = true;
        delta_ms = pair.delta_ms;
      }
    }
    notify_ui_throttled();
    if (matched) {
      QMetaObject::invokeMethod(
          this,
          [this, delta_ms]() { emit stereo_pair_matched(delta_ms); },
          Qt::QueuedConnection);
    }
  } catch (const std::exception &) {
    // ignore
  }
}

}  // namespace gui
}  // namespace hs_calib
