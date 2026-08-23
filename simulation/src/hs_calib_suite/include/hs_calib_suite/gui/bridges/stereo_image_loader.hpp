#pragma once

#include <algorithm>

#include <QString>
#include <QStringList>

namespace hs_calib {
namespace gui {

/// \brief 双目离线目录扫描结果
struct StereoImageDirScan {
  QString left_dir;
  QString right_dir;
  QStringList left_paths;
  QStringList right_paths;
  QString message;
  bool valid() const { return !left_paths.isEmpty() && !right_paths.isEmpty(); }
  int pair_count() const {
    return std::min(left_paths.size(), right_paths.size());
  }
};

/// \brief 扫描 left/ + right/ 子目录，或同目录 *_L / *_R 命名
class StereoImageLoader {
public:
  /// \brief 扫描根目录（含 left/ right/ 子目录）或左右独立路径
  static StereoImageDirScan scan(
      const QString &root_or_left_dir, const QString &right_dir = QString());
};

}  // namespace gui
}  // namespace hs_calib
