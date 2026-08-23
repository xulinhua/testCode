#include "hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp"

#include <algorithm>
#include <cmath>

#include <QImage>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace gui {
namespace {

QImage mat_bgr_to_qimage(const cv::Mat &bgr) {
  if (bgr.empty()) {
    return {};
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(
             rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
             QImage::Format_RGB888)
      .copy();
}

double point_line_distance(const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  return std::fabs((b.x - a.x) * (a.y - p.y) - (a.x - p.x) * (b.y - p.y)) / len;
}

void draw_heatmap_grid(
    cv::Mat *bgr,
    const std::vector<int> &grid,
    int cells,
    double alpha,
    const cv::Scalar &tint) {
  if (bgr == nullptr || bgr->empty() || cells < 2 ||
      static_cast<int>(grid.size()) != cells * cells) {
    return;
  }
  const int w = bgr->cols;
  const int h = bgr->rows;
  int max_count = 1;
  for (int v : grid) {
    max_count = std::max(max_count, v);
  }
  cv::Mat overlay = bgr->clone();
  for (int gy = 0; gy < cells; ++gy) {
    for (int gx = 0; gx < cells; ++gx) {
      const int c = grid[static_cast<size_t>(gy * cells + gx)];
      if (c <= 0) {
        continue;
      }
      const double t = static_cast<double>(c) / static_cast<double>(max_count);
      const int x0 = gx * w / cells;
      const int y0 = gy * h / cells;
      const int x1 = (gx + 1) * w / cells;
      const int y1 = (gy + 1) * h / cells;
      const cv::Scalar col(
          tint[0] * (0.2 + 0.8 * t), tint[1] * (0.2 + 0.8 * t),
          tint[2] * (0.2 + 0.8 * t));
      cv::rectangle(overlay, cv::Rect(x0, y0, x1 - x0, y1 - y0), col, cv::FILLED);
    }
  }
  cv::addWeighted(overlay, alpha, *bgr, 1.0 - alpha, 0.0, *bgr);
}

void draw_heatmap_float(
    cv::Mat *bgr,
    const std::vector<float> &grid,
    int cells,
    double alpha) {
  if (bgr == nullptr || bgr->empty() || cells < 2 ||
      static_cast<int>(grid.size()) != cells * cells) {
    return;
  }
  const int w = bgr->cols;
  const int h = bgr->rows;
  float max_v = 1e-3f;
  for (float v : grid) {
    max_v = std::max(max_v, v);
  }
  cv::Mat overlay = bgr->clone();
  for (int gy = 0; gy < cells; ++gy) {
    for (int gx = 0; gx < cells; ++gx) {
      const float v = grid[static_cast<size_t>(gy * cells + gx)];
      if (v <= 0.f) {
        continue;
      }
      const double t = std::min(1.0, static_cast<double>(v / max_v));
      const int x0 = gx * w / cells;
      const int y0 = gy * h / cells;
      const int x1 = (gx + 1) * w / cells;
      const int y1 = (gy + 1) * h / cells;
      const cv::Scalar col(
          30 + static_cast<int>(80 * t), 40 + static_cast<int>(120 * (1.0 - t)),
          200 - static_cast<int>(120 * t));
      cv::rectangle(overlay, cv::Rect(x0, y0, x1 - x0, y1 - y0), col, cv::FILLED);
    }
  }
  cv::addWeighted(overlay, alpha, *bgr, 1.0 - alpha, 0.0, *bgr);
}

void draw_collected_points(
    cv::Mat *bgr,
    const std::vector<core::CollectedIntrinsicsSample> &samples,
    const cv::Scalar &color,
    double alpha) {
  if (bgr == nullptr || bgr->empty() || samples.empty()) {
    return;
  }
  cv::Mat overlay = bgr->clone();
  const int radius = 4;
  for (const auto &s : samples) {
    if (s.observation.correspondences.empty()) {
      continue;
    }
    const auto &corr = s.observation.correspondences.front();
    for (int r = 0; r < corr.image_points.rows(); ++r) {
      const cv::Point2f pt(
          static_cast<float>(corr.image_points(r, 0)),
          static_cast<float>(corr.image_points(r, 1)));
      cv::circle(overlay, pt, radius + 1, cv::Scalar(8, 8, 8), -1, cv::LINE_AA);
      cv::circle(overlay, pt, radius, color, -1, cv::LINE_AA);
    }
  }
  cv::addWeighted(overlay, alpha, *bgr, 1.0 - alpha, 0.0, *bgr);
}

void draw_indicators(
    cv::Mat *bgr, const core::BoardFrameMetrics &metrics, int bar_w, double alpha) {
  if (bgr == nullptr || bgr->empty() || !metrics.detected) {
    return;
  }
  cv::Mat overlay = bgr->clone();
  const int x0 = 12;
  int y = 48;
  const auto bar = [&](const char *label, double value01) {
    value01 = std::clamp(value01, 0.0, 1.0);
    cv::putText(
        overlay, label, cv::Point(x0, y), cv::FONT_HERSHEY_SIMPLEX, 0.42,
        cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    y += 6;
    cv::rectangle(
        overlay, cv::Rect(x0, y, bar_w, 8), cv::Scalar(60, 60, 60), cv::FILLED);
    cv::rectangle(
        overlay, cv::Rect(x0, y, static_cast<int>(bar_w * value01), 8),
        cv::Scalar(80, 200, 120), cv::FILLED);
    y += 18;
  };
  bar("skew", metrics.normalized_skew);
  bar("area", metrics.relative_area_percent / 100.0);
  if (metrics.has_reprojection) {
    bar("rms", std::min(1.0, metrics.reproj_rms_px / 2.0));
  }
  cv::addWeighted(overlay, alpha, *bgr, 1.0 - alpha, 0.0, *bgr);
}

double point_line_distance_pts(
    const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b) {
  return point_line_distance(p, a, b);
}

}  // namespace

void accumulate_linearity_heatmap(
    const core::Correspondence &corr,
    int image_width,
    int image_height,
    int cells,
    std::vector<float> *grid) {
  if (grid == nullptr || cells < 2 || image_width <= 0 || image_height <= 0 ||
      corr.image_points.rows() < 4) {
    return;
  }
  const size_t ncells = static_cast<size_t>(cells * cells);
  if (grid->size() != ncells) {
    grid->assign(ncells, 0.f);
  }
  std::vector<cv::Point2f> pts;
  pts.reserve(static_cast<size_t>(corr.image_points.rows()));
  for (int r = 0; r < corr.image_points.rows(); ++r) {
    pts.emplace_back(
        static_cast<float>(corr.image_points(r, 0)),
        static_cast<float>(corr.image_points(r, 1)));
  }
  std::sort(pts.begin(), pts.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
    if (std::abs(a.y - b.y) < 8.f) {
      return a.x < b.x;
    }
    return a.y < b.y;
  });
  for (const auto &p : pts) {
    double err = 0.0;
    std::vector<cv::Point2f> row_pts;
    for (const auto &q : pts) {
      if (std::abs(q.y - p.y) < 12.f) {
        row_pts.push_back(q);
      }
    }
    if (row_pts.size() >= 2) {
      std::sort(row_pts.begin(), row_pts.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.x < b.x;
      });
      err = std::max(
          err, point_line_distance_pts(p, row_pts.front(), row_pts.back()));
    }
    std::vector<cv::Point2f> col_pts;
    for (const auto &q : pts) {
      if (std::abs(q.x - p.x) < 12.f) {
        col_pts.push_back(q);
      }
    }
    if (col_pts.size() >= 2) {
      std::sort(col_pts.begin(), col_pts.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.y < b.y;
      });
      err = std::max(
          err, point_line_distance_pts(p, col_pts.front(), col_pts.back()));
    }
    if (err <= 1e-6) {
      continue;
    }
    const int ix = std::min(
        cells - 1, std::max(0, static_cast<int>(p.x / image_width * cells)));
    const int iy = std::min(
        cells - 1, std::max(0, static_cast<int>(p.y / image_height * cells)));
    float &cell = (*grid)[static_cast<size_t>(iy * cells + ix)];
    cell = std::max(cell, static_cast<float>(err));
  }
}

const char *intrinsics_image_view_mode_label(IntrinsicsImageViewMode mode) {
  switch (mode) {
    case IntrinsicsImageViewMode::Source:
      return "Source";
    case IntrinsicsImageViewMode::SourceRectified:
      return "Source rectified";
    case IntrinsicsImageViewMode::UndistortionAlpha:
      return "Undistortion alpha";
    case IntrinsicsImageViewMode::Drawings:
      return "Drawings";
    case IntrinsicsImageViewMode::Indicators:
      return "Indicators";
  }
  return "Source";
}

cv::Mat qimage_to_cv_bgr(const QImage &image) {
  if (image.isNull()) {
    return {};
  }
  QImage rgb = image;
  if (rgb.format() != QImage::Format_RGB888) {
    rgb = rgb.convertToFormat(QImage::Format_RGB888);
  }
  if (rgb.isNull()) {
    return {};
  }
  cv::Mat bgr(rgb.height(), rgb.width(), CV_8UC3);
  for (int y = 0; y < rgb.height(); ++y) {
    const uchar *src = rgb.constScanLine(y);
    uchar *dst = bgr.ptr(y);
    for (int x = 0; x < rgb.width(); ++x) {
      dst[x * 3 + 0] = src[x * 3 + 2];
      dst[x * 3 + 1] = src[x * 3 + 1];
      dst[x * 3 + 2] = src[x * 3 + 0];
    }
  }
  return bgr;
}

QImage cv_bgr_to_qimage(const cv::Mat &bgr) {
  return mat_bgr_to_qimage(bgr);
}

void apply_intrinsics_preview_overlay(
    cv::Mat *bgr,
    IntrinsicsImageViewMode mode,
    const IntrinsicsVizOptions &viz,
    const core::IntrinsicsDataCollector &collector,
    const core::BoardFrameMetrics &metrics,
    const std::vector<float> &linearity_grid,
    int viz_pixel_cells) {
  if (bgr == nullptr || bgr->empty()) {
    return;
  }
  const int cells = std::max(
      2, std::max(collector.params().heatmap_cells, viz_pixel_cells));
  const double draw_alpha = std::clamp(static_cast<double>(viz.drawings_alpha), 0.05, 1.0);
  const double ind_alpha = std::clamp(static_cast<double>(viz.indicators_alpha), 0.05, 1.0);

  if (viz.draw_training_occupancy) {
    const auto &grid = collector.training_grid();
    if (static_cast<int>(grid.size()) == cells * cells) {
      draw_heatmap_grid(bgr, grid, cells, 0.35 * draw_alpha, cv::Scalar(30, 80, 40));
    }
  }
  if (viz.draw_evaluation_occupancy) {
    const auto &grid = collector.evaluation_grid();
    if (static_cast<int>(grid.size()) == cells * cells) {
      draw_heatmap_grid(bgr, grid, cells, 0.35 * draw_alpha, cv::Scalar(40, 60, 200));
    }
  }
  if (viz.draw_linearity_error && static_cast<int>(linearity_grid.size()) == cells * cells) {
    draw_heatmap_float(bgr, linearity_grid, cells, 0.4 * draw_alpha);
  }
  if (viz.draw_training_points) {
    draw_collected_points(
        bgr, collector.training(), cv::Scalar(70, 255, 120), 0.85 * draw_alpha);
  }
  if (viz.draw_evaluation_points) {
    draw_collected_points(
        bgr, collector.evaluation(), cv::Scalar(60, 140, 255), 0.85 * draw_alpha);
  }
  if (viz.draw_indicators || mode == IntrinsicsImageViewMode::Indicators ||
      mode == IntrinsicsImageViewMode::Drawings) {
    draw_indicators(bgr, metrics, 120, ind_alpha);
  }
}

QImage apply_intrinsics_view_mode(
    const QImage &source,
    IntrinsicsImageViewMode mode,
    const cv::Mat &K,
    const cv::Mat &D,
    double undistort_alpha) {
  if (source.isNull()) {
    return source;
  }
  cv::Mat bgr = qimage_to_cv_bgr(source);
  if (bgr.empty()) {
    return source;
  }
  if (mode == IntrinsicsImageViewMode::SourceRectified ||
      mode == IntrinsicsImageViewMode::UndistortionAlpha) {
    if (!K.empty() && K.rows == 3 && K.cols == 3) {
      cv::Mat map1, map2;
      cv::Mat newK = K.clone();
      if (mode == IntrinsicsImageViewMode::UndistortionAlpha) {
        newK = cv::getOptimalNewCameraMatrix(
            K, D, bgr.size(), undistort_alpha, bgr.size());
      }
      cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, bgr.size(), CV_16SC2, map1, map2);
      cv::Mat rect;
      cv::remap(bgr, rect, map1, map2, cv::INTER_LINEAR);
      bgr = rect;
    }
  }
  return mat_bgr_to_qimage(bgr);
}

}  // namespace gui
}  // namespace hs_calib
