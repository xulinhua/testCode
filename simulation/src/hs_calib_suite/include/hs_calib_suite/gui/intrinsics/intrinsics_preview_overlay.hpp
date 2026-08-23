#pragma once

#include <vector>

#include <QImage>
#include <opencv2/core.hpp>

#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {
struct BoardFrameMetrics;
class IntrinsicsDataCollector;
}  // namespace core

namespace gui {

/// \brief Tier4 ImageViewMode（§11）
enum class IntrinsicsImageViewMode {
  Source = 0,
  SourceRectified,
  UndistortionAlpha,
  Drawings,
  Indicators,
};

/// \brief Tier4 Visualization options（§13）
struct IntrinsicsVizOptions {
  bool draw_detection = true;
  bool draw_training_points = false;
  bool draw_evaluation_points = false;
  bool draw_training_occupancy = false;
  bool draw_evaluation_occupancy = false;
  bool draw_linearity_error = false;
  bool draw_indicators = true;
  float drawings_alpha = 1.0f;
  float indicators_alpha = 1.0f;
};

const char *intrinsics_image_view_mode_label(IntrinsicsImageViewMode mode);

/// \brief QImage → OpenCV BGR（正确处理 RGB888，避免按 QRgb 误读越界）
cv::Mat qimage_to_cv_bgr(const QImage &image);

/// \brief OpenCV BGR → QImage（RGB888 深拷贝）
QImage cv_bgr_to_qimage(const cv::Mat &bgr);

/// \brief 在预览上叠加热力图 / 历史点 / 指示条等（内参 Tier4）
void apply_intrinsics_preview_overlay(
    cv::Mat *bgr,
    IntrinsicsImageViewMode mode,
    const IntrinsicsVizOptions &viz,
    const core::IntrinsicsDataCollector &collector,
    const core::BoardFrameMetrics &metrics,
    const std::vector<float> &linearity_grid,
    int viz_pixel_cells);

/// \brief 将当前帧角点线性度误差累积到像面格热力图
void accumulate_linearity_heatmap(
    const core::Correspondence &corr,
    int image_width,
    int image_height,
    int cells,
    std::vector<float> *grid);

QImage apply_intrinsics_view_mode(
    const QImage &source,
    IntrinsicsImageViewMode mode,
    const cv::Mat &K,
    const cv::Mat &D,
    double undistort_alpha = 0.0);

}  // namespace gui
}  // namespace hs_calib
