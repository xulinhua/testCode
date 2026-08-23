#include "hs_calib_suite/core/detectors/circle_grid_detect_impl.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {
namespace {

cv::Ptr<cv::SimpleBlobDetector> make_circle_blob_detector(
    const CircleGridTarget &target, const DotDetectorParams &dot) {
  cv::SimpleBlobDetector::Params p;
  p.filterByColor = true;
  p.blobColor = 0;
  p.filterByCircularity = true;
  p.minCircularity = 0.65f;
  p.filterByConvexity = true;
  p.minConvexity = 0.8f;
  p.filterByInertia = true;
  p.minInertiaRatio = 0.3f;
  p.filterByArea = dot.filter_by_area;
  const double img_area = 1.0e6;
  p.minArea = static_cast<float>(dot.min_area_percentage * 0.01 * img_area);
  p.maxArea = static_cast<float>(dot.max_area_percentage * 0.01 * img_area);
  p.minDistBetweenBlobs =
      static_cast<float>(dot.min_dist_between_blobs_percentage);
  p.minArea = std::max(25.0f, p.minArea);
  p.maxArea = std::max(p.minArea + 1.0f, p.maxArea);

  const double spacing = std::max(1e-6, target.center_distance_m());
  const double diam = target.circle_diameter_m();
  if (diam > 1e-6) {
    const double rel = diam / spacing;
    if (rel > 0.15 && rel < 1.8) {
      p.minCircularity = 0.7f;
    }
  }
  return cv::SimpleBlobDetector::create(p);
}

/// \brief OpenCV 官方 calcBoardCornerPositions 的单位物点（squareSize=1）
std::vector<cv::Point2f> unit_object_points(cv::Size pattern, bool asymmetric) {
  std::vector<cv::Point2f> pts;
  pts.reserve(static_cast<size_t>(pattern.area()));
  for (int i = 0; i < pattern.height; ++i) {
    for (int j = 0; j < pattern.width; ++j) {
      if (asymmetric) {
        pts.emplace_back(
            static_cast<float>(2 * j + (i % 2)), static_cast<float>(i));
      } else {
        pts.emplace_back(static_cast<float>(j), static_cast<float>(i));
      }
    }
  }
  return pts;
}

double homography_rmse(
    const std::vector<cv::Point2f> &obj, const std::vector<cv::Point2f> &img) {
  if (obj.size() != img.size() || obj.size() < 4) {
    return 1e9;
  }
  const cv::Mat H = cv::findHomography(obj, img, 0);
  if (H.empty()) {
    return 1e9;
  }
  std::vector<cv::Point2f> proj;
  cv::perspectiveTransform(obj, proj, H);
  double sse = 0.0;
  for (size_t i = 0; i < img.size(); ++i) {
    const double dx = static_cast<double>(proj[i].x - img[i].x);
    const double dy = static_cast<double>(proj[i].y - img[i].y);
    sse += dx * dx + dy * dy;
  }
  return std::sqrt(sse / static_cast<double>(img.size()));
}

/// \brief 按 y 聚类成 rows 行，行内按 x 排序（对齐 OpenCV 物点行主序）
bool cluster_rows_sorted(
    const std::vector<cv::Point2f> &centers, int rows, int cols,
    bool flip_rows, bool flip_cols, std::vector<cv::Point2f> *ordered) {
  if (static_cast<int>(centers.size()) != rows * cols || ordered == nullptr) {
    return false;
  }
  std::vector<int> idx(centers.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int a, int b) {
    return centers[static_cast<size_t>(a)].y < centers[static_cast<size_t>(b)].y;
  });

  const float y0 = centers[static_cast<size_t>(idx.front())].y;
  const float y1 = centers[static_cast<size_t>(idx.back())].y;
  const float span = std::max(1.0f, y1 - y0);
  const float thr = span / static_cast<float>(std::max(1, rows - 1)) * 0.45f;

  std::vector<std::vector<int>> clusters;
  for (int id : idx) {
    const float y = centers[static_cast<size_t>(id)].y;
    if (clusters.empty() ||
        std::abs(y - centers[static_cast<size_t>(clusters.back().back())].y) > thr) {
      clusters.emplace_back();
    }
    clusters.back().push_back(id);
  }
  if (static_cast<int>(clusters.size()) != rows) {
    // 等分兜底：按 y 排序后切成 rows 段
    clusters.assign(static_cast<size_t>(rows), {});
    for (int k = 0; k < static_cast<int>(idx.size()); ++k) {
      const int r = std::min(rows - 1, k / cols);
      clusters[static_cast<size_t>(r)].push_back(idx[static_cast<size_t>(k)]);
    }
  }
  if (static_cast<int>(clusters.size()) != rows) {
    return false;
  }
  for (auto &cl : clusters) {
    if (static_cast<int>(cl.size()) != cols) {
      return false;
    }
    std::sort(cl.begin(), cl.end(), [&](int a, int b) {
      return centers[static_cast<size_t>(a)].x < centers[static_cast<size_t>(b)].x;
    });
    if (flip_cols) {
      std::reverse(cl.begin(), cl.end());
    }
  }
  if (flip_rows) {
    std::reverse(clusters.begin(), clusters.end());
  }

  ordered->clear();
  ordered->reserve(centers.size());
  for (const auto &cl : clusters) {
    for (int id : cl) {
      ordered->push_back(centers[static_cast<size_t>(id)]);
    }
  }
  return static_cast<int>(ordered->size()) == rows * cols;
}

/// \brief findCirclesGrid 点序不稳定（尤其 3×6）；重排到官方物点顺序
bool align_centers_to_object_order(
    std::vector<cv::Point2f> *centers, cv::Size pattern, bool asymmetric) {
  if (centers == nullptr ||
      static_cast<int>(centers->size()) != pattern.area()) {
    return false;
  }
  const auto obj = unit_object_points(pattern, asymmetric);
  const double raw_err = homography_rmse(obj, *centers);
  // 已对齐则不动（4×11 等常见尺寸通常 raw_err≈0）
  if (raw_err < 2.0) {
    return true;
  }

  double best_err = raw_err;
  std::vector<cv::Point2f> best = *centers;
  for (bool flip_rows : {false, true}) {
    for (bool flip_cols : {false, true}) {
      std::vector<cv::Point2f> ordered;
      if (!cluster_rows_sorted(
              *centers, pattern.height, pattern.width, flip_rows, flip_cols,
              &ordered)) {
        continue;
      }
      const double err = homography_rmse(obj, ordered);
      if (err < best_err) {
        best_err = err;
        best = std::move(ordered);
      }
    }
  }
  if (best_err > 15.0 && best_err >= raw_err * 0.9) {
    // 对齐失败，保留原序（总比乱改好）
    return false;
  }
  *centers = std::move(best);
  return best_err < raw_err;
}

bool try_find(
    const cv::Mat &gray, cv::Size pattern, int flags,
    const cv::Ptr<cv::Feature2D> &blob, std::vector<cv::Point2f> *centers) {
  centers->clear();
  if (gray.empty() || pattern.width < 2 || pattern.height < 2) {
    return false;
  }
  const bool ok = blob.empty()
      ? cv::findCirclesGrid(gray, pattern, *centers, flags)
      : cv::findCirclesGrid(gray, pattern, *centers, flags, blob);
  return ok && static_cast<int>(centers->size()) == pattern.area();
}

bool find_circles_like_opencv(
    const cv::Mat &gray, const CircleGridTarget &target,
    const DotDetectorParams &dot_params,
    std::vector<cv::Point2f> *centers) {
  const bool asymmetric = target.pattern() == CircleGridPattern::Asymmetric;
  const cv::Size pattern(target.circles_x(), target.circles_y());
  const auto blob = make_circle_blob_detector(target, dot_params);
  const int base = asymmetric ? cv::CALIB_CB_ASYMMETRIC_GRID
                              : cv::CALIB_CB_SYMMETRIC_GRID;
  std::vector<int> attempts;
  attempts.push_back(base);
  if (dot_params.clustering) {
    attempts.push_back(base | cv::CALIB_CB_CLUSTERING);
  }

  auto run = [&](const cv::Mat &img) -> bool {
    for (int flags : attempts) {
      if (try_find(img, pattern, flags, blob, centers)) {
        return true;
      }
      if (try_find(img, pattern, flags, {}, centers)) {
        return true;
      }
    }
    return false;
  };

  cv::Mat work = gray;
  if (dot_params.resized_detection) {
    const int max_dim = std::max(gray.cols, gray.rows);
    if (max_dim > dot_params.resized_max_resolution) {
      const double scale =
          static_cast<double>(dot_params.resized_max_resolution) / max_dim;
      cv::resize(gray, work, cv::Size(), scale, scale, cv::INTER_AREA);
    }
  }
  if (run(work)) {
    if (work.data != gray.data && centers != nullptr) {
      const float inv = static_cast<float>(gray.cols) / static_cast<float>(work.cols);
      for (auto &p : *centers) {
        p *= inv;
      }
    }
    return true;
  }
  cv::Mat eq;
  cv::createCLAHE(2.0, cv::Size(8, 8))->apply(gray, eq);
  if (run(eq)) {
    return true;
  }
  cv::Mat inv;
  cv::bitwise_not(gray, inv);
  return run(inv);
}

}  // namespace

std::vector<Correspondence> detect_circle_grid_impl(
    const ImageFrame &frame, const CircleGridTarget &target,
    const DotDetectorParams &dot_params) {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }
  const cv::Mat gray = to_gray(mat);
  std::vector<cv::Point2f> centers;
  if (!find_circles_like_opencv(gray, target, dot_params, &centers)) {
    return out;
  }

  const bool asymmetric = target.pattern() == CircleGridPattern::Asymmetric;
  const cv::Size pattern(target.circles_x(), target.circles_y());
  align_centers_to_object_order(&centers, pattern, asymmetric);

  Correspondence c;
  const int n = static_cast<int>(centers.size());
  c.image_points.resize(n, 2);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = centers[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = centers[static_cast<size_t>(i)].y;
    c.ids[static_cast<size_t>(i)] = i;
  }
  c.object_points = target.all_object_points();
  if (c.object_points.rows() != n) {
    return out;
  }
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
