#include "hs_calib_suite/gui/bridges/ros_bag_frame_reader.hpp"

#include <algorithm>
#include <cstring>
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

constexpr int kHardMaxFrames = 2000;

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
  if (src.channels() == 4) {
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
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

}  // namespace

void RosBagFrameReader::clear() {
  topic_.clear();
  frames_.clear();
}

int RosBagFrameReader::open(
    const QString &bag_uri,
    const QString &topic,
    int max_frames,
    QString *error_out) {
  clear();
  topic_ = topic.trimmed();
  if (topic_.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("未指定图像话题");
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

  const auto type_it = topic_types.find(topic_.toStdString());
  if (type_it == topic_types.end() || !is_image_type(type_it->second)) {
    if (error_out) {
      *error_out = QStringLiteral("话题不是图像类型：%1").arg(topic_);
    }
    return 0;
  }
  const bool compressed = is_compressed_type(type_it->second);
  const int limit =
      max_frames <= 0 ? kHardMaxFrames : std::min(max_frames, kHardMaxFrames);
  const std::string want = topic_.toStdString();

  rclcpp::Serialization<sensor_msgs::msg::Image> ser_img;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser_cmp;

  try {
    while (reader->has_next() &&
           static_cast<int>(frames_.size()) < limit) {
      auto bag_msg = reader->read_next();
      if (!bag_msg || bag_msg->topic_name != want) {
        continue;
      }
      cv::Mat bgr;
      auto serialized = copy_serialized(*bag_msg);
      try {
        if (compressed) {
          sensor_msgs::msg::CompressedImage cmp;
          ser_cmp.deserialize_message(&serialized, &cmp);
          bgr = decode_compressed_msg(cmp);
        } else {
          sensor_msgs::msg::Image img;
          ser_img.deserialize_message(&serialized, &img);
          bgr = decode_image_msg(img);
        }
      } catch (...) {
        continue;
      }
      if (bgr.empty()) {
        continue;
      }
      frames_.push_back(std::move(bgr));
    }
  } catch (const std::exception &e) {
    if (error_out) {
      *error_out = QStringLiteral("解码 bag 失败：%1")
                       .arg(QString::fromStdString(e.what()));
    }
  }

  if (frames_.empty() && error_out && error_out->isEmpty()) {
    *error_out = QStringLiteral("话题 %1 未解码到任何图像帧").arg(topic_);
  }
  return static_cast<int>(frames_.size());
}

const cv::Mat &RosBagFrameReader::frame(int index) const {
  static const cv::Mat kEmpty;
  if (index < 0 || index >= static_cast<int>(frames_.size())) {
    return kEmpty;
  }
  return frames_[static_cast<size_t>(index)];
}

QString RosBagFrameReader::frame_label(int index) const {
  return QStringLiteral("bag://%1#%2").arg(topic_).arg(index + 1);
}

}  // namespace gui
}  // namespace hs_calib
