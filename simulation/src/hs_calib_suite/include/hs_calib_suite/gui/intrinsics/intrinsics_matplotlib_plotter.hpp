#pragma once

#include <string>

#include <QPixmap>

namespace hs_calib {
namespace core {
class IntrinsicsDataCollector;
}  // namespace core

namespace gui {

/// \brief 采集统计图：Qt 轻量渲染或调用 matplotlib 生成 PNG
class IntrinsicsStatsPlotter {
public:
  /// \brief Qt 后端：质心散点 + 倾角直方图
  static QPixmap render_qt_summary(const core::IntrinsicsDataCollector &collector);

  /// \brief matplotlib 后端：写入 JSON 并调用 python3 脚本
  static QPixmap render_matplotlib(
      const core::IntrinsicsDataCollector &collector, std::string *error_out);

  /// \brief 导出采集指纹 JSON（供脚本或调试）
  static bool export_collector_json(
      const core::IntrinsicsDataCollector &collector, const std::string &path);
};

}  // namespace gui
}  // namespace hs_calib
