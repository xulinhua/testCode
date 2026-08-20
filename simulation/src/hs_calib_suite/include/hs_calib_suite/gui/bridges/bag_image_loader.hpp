#pragma once

#include <QList>
#include <QString>
#include <QStringList>

namespace hs_calib {
namespace gui {

/// \brief rosbag2 图像话题摘要
struct BagTopicInfo {
  QString name;
  QString type;  ///< sensor_msgs/msg/Image 或 CompressedImage
  int64_t message_count = 0;
};

/// \brief 从 rosbag2 列出图像话题，并导出为离线 PNG 序列
class BagImageLoader {
public:
  /// \brief 列出 bag 中的 Image / CompressedImage 话题
  static QList<BagTopicInfo> list_image_topics(
      const QString &bag_uri, QString *error_out = nullptr);

  /// \brief 将指定话题图像导出到 output_dir（PNG），返回写出张数
  /// \param max_frames ≤0 表示不限制（仍有安全上限）
  static int extract_images(
      const QString &bag_uri, const QString &topic, const QString &output_dir,
      int max_frames = 500, QString *error_out = nullptr);

  /// \brief 为 bag+topic 生成稳定的缓存目录路径
  static QString default_cache_dir(const QString &bag_uri, const QString &topic);
};

}  // namespace gui
}  // namespace hs_calib
