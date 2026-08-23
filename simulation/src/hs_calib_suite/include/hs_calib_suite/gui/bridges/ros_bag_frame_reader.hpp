#pragma once

#include <vector>

#include <QString>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace gui {

/// \brief 直接解码 rosbag2 图像话题（不导出 PNG）
class RosBagFrameReader {
public:
  RosBagFrameReader() = default;
  RosBagFrameReader(RosBagFrameReader &&) = default;
  RosBagFrameReader &operator=(RosBagFrameReader &&) = default;
  RosBagFrameReader(const RosBagFrameReader &) = delete;
  RosBagFrameReader &operator=(const RosBagFrameReader &) = delete;

  /// \brief 打开 bag 并解码指定话题帧到内存
  /// \return 解码帧数；失败返回 0
  int open(
      const QString &bag_uri,
      const QString &topic,
      int max_frames = 500,
      QString *error_out = nullptr);

  int size() const { return static_cast<int>(frames_.size()); }
  bool empty() const { return frames_.empty(); }

  const cv::Mat &frame(int index) const;
  QString frame_label(int index) const;

  void clear();

private:
  QString topic_;
  std::vector<cv::Mat> frames_;
};

}  // namespace gui
}  // namespace hs_calib
