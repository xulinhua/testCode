#pragma once

#include <cstdint>
#include <vector>

#include <QString>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace gui {

/// \brief Bag 解码得到的立体帧对
struct StereoBagPair {
  cv::Mat left_bgr;
  cv::Mat right_bgr;
  int64_t delta_ms = 0;
};

/// \brief 双话题 rosbag2 时间戳配对解码
class RosBagStereoFrameReader {
public:
  /// \return 配对帧数
  int open(
      const QString &bag_uri,
      const QString &left_topic,
      const QString &right_topic,
      int max_pairs = 500,
      int max_match_delta_ms = 30,
      QString *error_out = nullptr);

  int size() const { return static_cast<int>(pairs_.size()); }
  bool empty() const { return pairs_.empty(); }
  const StereoBagPair &pair(int index) const;
  QString left_label(int index) const;
  QString right_label(int index) const;
  void clear();

private:
  QString left_topic_;
  QString right_topic_;
  std::vector<StereoBagPair> pairs_;
};

}  // namespace gui
}  // namespace hs_calib
