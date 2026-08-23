#include "hs_calib_suite/gui/bridges/ros_bag_stereo_frame_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include <QFileInfo>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hs_calib {
namespace gui {
namespace {

constexpr int kHardMaxPairs = 2000;

bool is_image_type(const std::string &type) {
  return type == "sensor_msgs/msg/Image" || type == "sensor_msgs/Image" ||
         type == "sensor_msgs/msg/CompressedImage" ||
         type == "sensor_msgs/CompressedImage";
}

bool is_compressed_type(const std::string &type) {
  return type == "sensor_msgs/msg/CompressedImage" ||
         type == "sensor_msgs/CompressedImage";
}

std::unique_ptr<rosbag2_cpp::readers::SequentialReader> open_reader(
    const QString &bag_uri, QString *error_out) {
  QString uri = bag_uri.trimmed();
  QFileInfo fi(uri);
  if (fi.isFile()) {
    uri = fi.absolutePath();
  } else {
    uri = fi.absoluteFilePath();
  }
  if (uri.isEmpty() || !QFileInfo::exists(uri)) {
    if (error_out) {
      *error_out = QStringLiteral("bag 路径不存在：%1").arg(bag_uri);
    }
    return nullptr;
  }
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri.toStdString();
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  try {
    reader->open(storage_options, converter_options);
  } catch (const std::exception &e) {
    if (error_out) {
      *error_out =
          QStringLiteral("打开 bag 失败：%1").arg(QString::fromStdString(e.what()));
    }
    return nullptr;
  }
  return reader;
}

cv::Mat to_bgr(const cv::Mat &src, const std::string &encoding) {
  if (src.empty()) {
    return {};
  }
  if (src.channels() == 1) {
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  if (encoding == "rgb8" || encoding == "RGB8") {
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  return src.clone();
}

cv::Mat decode_image_msg(const sensor_msgs::msg::Image &msg) {
  try {
    const std::string enc = msg.encoding.empty() ? "bgr8" : msg.encoding;
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, enc);
    return to_bgr(cv_ptr->image, cv_ptr->encoding);
  } catch (...) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg);
      return to_bgr(cv_ptr->image, cv_ptr->encoding);
    } catch (...) {
      return {};
    }
  }
}

cv::Mat decode_compressed_msg(const sensor_msgs::msg::CompressedImage &msg) {
  if (msg.data.empty()) {
    return {};
  }
  const std::vector<uint8_t> buf(msg.data.begin(), msg.data.end());
  return cv::imdecode(buf, cv::IMREAD_COLOR);
}

rclcpp::SerializedMessage copy_serialized(
    const rosbag2_storage::SerializedBagMessage &bag_msg) {
  rclcpp::SerializedMessage out;
  out.reserve(bag_msg.serialized_data->buffer_length);
  auto &rcl = out.get_rcl_serialized_message();
  std::memcpy(
      rcl.buffer, bag_msg.serialized_data->buffer,
      bag_msg.serialized_data->buffer_length);
  rcl.buffer_length = bag_msg.serialized_data->buffer_length;
  return out;
}

int64_t stamp_ns(const sensor_msgs::msg::Image &msg) {
  return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
         static_cast<int64_t>(msg.header.stamp.nanosec);
}

int64_t stamp_ns_cmp(const sensor_msgs::msg::CompressedImage &msg) {
  return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
         static_cast<int64_t>(msg.header.stamp.nanosec);
}

struct StampedBgr {
  int64_t stamp_ns = 0;
  cv::Mat bgr;
};

}  // namespace

void RosBagStereoFrameReader::clear() {
  left_topic_.clear();
  right_topic_.clear();
  pairs_.clear();
}

const StereoBagPair &RosBagStereoFrameReader::pair(int index) const {
  static const StereoBagPair kEmpty;
  if (index < 0 || index >= static_cast<int>(pairs_.size())) {
    return kEmpty;
  }
  return pairs_[static_cast<size_t>(index)];
}

QString RosBagStereoFrameReader::left_label(int index) const {
  return QStringLiteral("bag://%1#%2").arg(left_topic_).arg(index + 1);
}

QString RosBagStereoFrameReader::right_label(int index) const {
  return QStringLiteral("bag://%1#%2").arg(right_topic_).arg(index + 1);
}

int RosBagStereoFrameReader::open(
    const QString &bag_uri,
    const QString &left_topic,
    const QString &right_topic,
    int max_pairs,
    int max_match_delta_ms,
    QString *error_out) {
  clear();
  left_topic_ = left_topic.trimmed();
  right_topic_ = right_topic.trimmed();
  if (left_topic_.isEmpty() || right_topic_.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("未指定左右图像话题");
    }
    return 0;
  }

  auto reader = open_reader(bag_uri, error_out);
  if (!reader) {
    return 0;
  }

  std::unordered_map<std::string, std::string> topic_types;
  try {
    for (const auto &t : reader->get_all_topics_and_types()) {
      topic_types[t.name] = t.type;
    }
  } catch (...) {
  }

  const auto type_l = topic_types.find(left_topic_.toStdString());
  const auto type_r = topic_types.find(right_topic_.toStdString());
  if (type_l == topic_types.end() || !is_image_type(type_l->second) ||
      type_r == topic_types.end() || !is_image_type(type_r->second)) {
    if (error_out) {
      *error_out = QStringLiteral("左右话题须为 Image/CompressedImage");
    }
    return 0;
  }

  const bool cmp_l = is_compressed_type(type_l->second);
  const bool cmp_r = is_compressed_type(type_r->second);
  const int limit =
      max_pairs <= 0 ? kHardMaxPairs : std::min(max_pairs, kHardMaxPairs);
  const int64_t max_delta_ns =
      static_cast<int64_t>(std::max(1, max_match_delta_ms)) * 1000000LL;
  const std::string want_l = left_topic_.toStdString();
  const std::string want_r = right_topic_.toStdString();

  rclcpp::Serialization<sensor_msgs::msg::Image> ser_img;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser_cmp;

  std::deque<StampedBgr> left_buf;
  std::deque<StampedBgr> right_buf;

  auto try_match = [&]() {
    if (left_buf.empty() || right_buf.empty() ||
        static_cast<int>(pairs_.size()) >= limit) {
      return;
    }
    int64_t best_d = max_delta_ns + 1;
    size_t best_li = 0;
    size_t best_ri = 0;
    for (size_t li = 0; li < left_buf.size(); ++li) {
      for (size_t ri = 0; ri < right_buf.size(); ++ri) {
        const int64_t d = std::llabs(left_buf[li].stamp_ns - right_buf[ri].stamp_ns);
        if (d <= max_delta_ns && d < best_d) {
          best_d = d;
          best_li = li;
          best_ri = ri;
        }
      }
    }
    if (best_d > max_delta_ns) {
      return;
    }
    StereoBagPair p;
    p.left_bgr = left_buf[best_li].bgr.clone();
    p.right_bgr = right_buf[best_ri].bgr.clone();
    p.delta_ms = best_d / 1000000LL;
    pairs_.push_back(std::move(p));
    left_buf.erase(left_buf.begin() + static_cast<std::ptrdiff_t>(best_li));
    right_buf.erase(right_buf.begin() + static_cast<std::ptrdiff_t>(best_ri));
  };

  auto push_buf = [](std::deque<StampedBgr> *buf, StampedBgr f) {
    buf->push_back(std::move(f));
    while (buf->size() > 64) {
      buf->pop_front();
    }
  };

  try {
    while (reader->has_next() && static_cast<int>(pairs_.size()) < limit) {
      auto bag_msg = reader->read_next();
      if (!bag_msg) {
        continue;
      }
      const std::string &tn = bag_msg->topic_name;
      const bool is_left = (tn == want_l);
      const bool is_right = (tn == want_r);
      if (!is_left && !is_right) {
        continue;
      }
      cv::Mat bgr;
      int64_t t_ns = 0;
      auto serialized = copy_serialized(*bag_msg);
      try {
        if (is_left) {
          if (cmp_l) {
            sensor_msgs::msg::CompressedImage cmp;
            ser_cmp.deserialize_message(&serialized, &cmp);
            t_ns = stamp_ns_cmp(cmp);
            bgr = decode_compressed_msg(cmp);
          } else {
            sensor_msgs::msg::Image img;
            ser_img.deserialize_message(&serialized, &img);
            t_ns = stamp_ns(img);
            bgr = decode_image_msg(img);
          }
        } else {
          if (cmp_r) {
            sensor_msgs::msg::CompressedImage cmp;
            ser_cmp.deserialize_message(&serialized, &cmp);
            t_ns = stamp_ns_cmp(cmp);
            bgr = decode_compressed_msg(cmp);
          } else {
            sensor_msgs::msg::Image img;
            ser_img.deserialize_message(&serialized, &img);
            t_ns = stamp_ns(img);
            bgr = decode_image_msg(img);
          }
        }
      } catch (...) {
        continue;
      }
      if (bgr.empty()) {
        continue;
      }
      StampedBgr frame;
      frame.stamp_ns = t_ns;
      frame.bgr = std::move(bgr);
      if (is_left) {
        push_buf(&left_buf, std::move(frame));
      } else {
        push_buf(&right_buf, std::move(frame));
      }
      try_match();
    }
    while (!left_buf.empty() && !right_buf.empty() &&
           static_cast<int>(pairs_.size()) < limit) {
      try_match();
      if (left_buf.size() > right_buf.size()) {
        left_buf.pop_front();
      } else {
        right_buf.pop_front();
      }
    }
  } catch (const std::exception &e) {
    if (error_out) {
      *error_out = QStringLiteral("解码立体 bag 失败：%1")
                       .arg(QString::fromStdString(e.what()));
    }
  }

  if (pairs_.empty() && error_out && error_out->isEmpty()) {
    *error_out = QStringLiteral("未匹配到任何立体帧对（检查话题与 Δt 阈值）");
  }
  return static_cast<int>(pairs_.size());
}

}  // namespace gui
}  // namespace hs_calib
