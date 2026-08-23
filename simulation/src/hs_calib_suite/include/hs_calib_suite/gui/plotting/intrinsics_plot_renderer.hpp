#pragma once

#include <string>

#include <QPixmap>

namespace hs_calib {
namespace core {
struct IntrinsicsPlotInput;
struct IntrinsicsPipelineStageViews;
}  // namespace core

namespace gui {

/// \brief Tier4 统计图类型
enum class IntrinsicsPlotKind {
  CollectionDataStatistics,
  CalibrationBars,
  CalibrationRmsHeatmaps,
};

/// \brief 统一 matplotlib 渲染后端（调用 share/scripts 下 Python 脚本）
class IntrinsicsPlotRenderer {
public:
  static QPixmap render(
      IntrinsicsPlotKind kind,
      const core::IntrinsicsPlotInput &input,
      const core::IntrinsicsPipelineStageViews &stages,
      std::string *error_out);

  static const char *kind_title(IntrinsicsPlotKind kind);
};

}  // namespace gui
}  // namespace hs_calib
