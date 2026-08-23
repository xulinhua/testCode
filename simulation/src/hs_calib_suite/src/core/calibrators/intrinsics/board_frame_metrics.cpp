#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

namespace hs_calib {
namespace core {
namespace {

std::vector<cv::Point2f> to_image_points(const Correspondence &corr) {
  std::vector<cv::Point2f> pts;
  pts.reserve(static_cast<size_t>(corr.image_points.rows()));
  for (int i = 0; i < corr.image_points.rows(); ++i) {
    pts.emplace_back(
        static_cast<float>(corr.image_points(i, 0)),
        static_cast<float>(corr.image_points(i, 1)));
  }
  return pts;
}

std::vector<cv::Point3f> to_object_points(const Correspondence &corr) {
  std::vector<cv::Point3f> pts;
  pts.reserve(static_cast<size_t>(corr.image_points.rows()));
  for (int i = 0; i < corr.image_points.rows(); ++i) {
    pts.emplace_back(
        static_cast<float>(corr.object_points(i, 0)),
        static_cast<float>(corr.object_points(i, 1)),
        static_cast<float>(corr.object_points(i, 2)));
  }
  return pts;
}

double point_line_distance(
    const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b) {
  const cv::Point2f ab = b - a;
  const double denom = std::hypot(ab.x, ab.y);
  if (denom < 1e-6) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  const double cross = std::fabs((p.x - a.x) * ab.y - (p.y - a.y) * ab.x);
  return cross / denom;
}

double interior_angle_deg(
    const cv::Point2f &prev, const cv::Point2f &cur, const cv::Point2f &next) {
  const cv::Point2f v1 = prev - cur;
  const cv::Point2f v2 = next - cur;
  const double n1 = std::hypot(v1.x, v1.y);
  const double n2 = std::hypot(v2.x, v2.y);
  if (n1 < 1e-6 || n2 < 1e-6) {
    return 90.0;
  }
  double cos_a = (v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
  cos_a = std::clamp(cos_a, -1.0, 1.0);
  return std::acos(cos_a) * 180.0 / CV_PI;
}

double compute_normalized_skew(const std::vector<cv::Point2f> &pts) {
  if (pts.size() < 4) {
    return 0.0;
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(pts, hull);
  if (hull.size() < 4) {
    return 0.0;
  }
  double max_dev = 0.0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const cv::Point2f prev = hull[(i + hull.size() - 1) % hull.size()];
    const cv::Point2f cur = hull[i];
    const cv::Point2f next = hull[(i + 1) % hull.size()];
    const double ang = interior_angle_deg(prev, cur, next);
    max_dev = std::max(max_dev, std::fabs(ang - 90.0));
  }
  return std::min(1.0, max_dev / 45.0);
}

double linear_rms_along_axis(
    const std::vector<cv::Point2f> &pts, bool horizontal) {
  if (pts.size() < 3) {
    return 0.0;
  }
  std::vector<double> residuals;
  residuals.reserve(pts.size());
  if (horizontal) {
    std::vector<cv::Point2f> sorted = pts;
    std::sort(sorted.begin(), sorted.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
      return a.x < b.x;
    });
    const cv::Point2f a = sorted.front();
    const cv::Point2f b = sorted.back();
    for (const auto &p : pts) {
      residuals.push_back(point_line_distance(p, a, b));
    }
  } else {
    std::vector<cv::Point2f> sorted = pts;
    std::sort(sorted.begin(), sorted.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
      return a.y < b.y;
    });
    const cv::Point2f a = sorted.front();
    const cv::Point2f b = sorted.back();
    for (const auto &p : pts) {
      residuals.push_back(point_line_distance(p, a, b));
    }
  }
  const double mean =
      std::accumulate(residuals.begin(), residuals.end(), 0.0) /
      static_cast<double>(residuals.size());
  double sq = 0.0;
  for (double r : residuals) {
    const double d = r - mean;
    sq += d * d;
  }
  return std::sqrt(sq / static_cast<double>(residuals.size()));
}

void fill_geometry_metrics(
    BoardFrameMetrics *out,
    const std::vector<cv::Point2f> &img_pts,
    int image_width,
    int image_height) {
  if (out == nullptr || img_pts.empty() || image_width <= 0 || image_height <= 0) {
    return;
  }
  cv::Rect bounds = cv::boundingRect(img_pts);
  out->area_ratio =
      static_cast<double>(bounds.area()) /
      static_cast<double>(image_width * image_height);
  out->relative_area_percent = out->area_ratio * 100.0;
  out->normalized_skew = compute_normalized_skew(img_pts);
  out->linear_error_rows_rms_px = linear_rms_along_axis(img_pts, true);
  out->linear_error_cols_rms_px = linear_rms_along_axis(img_pts, false);

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto &p : img_pts) {
    sum_x += p.x;
    sum_y += p.y;
  }
  out->centroid_x_norm = sum_x / img_pts.size() / image_width;
  out->centroid_y_norm = sum_y / img_pts.size() / image_height;

  if (img_pts.size() >= 2) {
    const double w = static_cast<double>(bounds.width);
    const double h = static_cast<double>(bounds.height);
    out->aspect_ratio = (h > 1e-3) ? w / h : 0.0;
    if (out->normalized_skew > 0.35) {
      out->aspect_ratio = 0.0;
    }
  }
}

void fill_pose_metrics(
    BoardFrameMetrics *out,
    const cv::Mat &rvec,
    const cv::Mat &tvec) {
  if (out == nullptr || rvec.empty() || tvec.empty()) {
    return;
  }
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  const cv::Vec3d z_board(0.0, 0.0, 1.0);
  cv::Mat z_cam = R * cv::Mat(z_board);
  const double nz = std::hypot(z_cam.at<double>(0), z_cam.at<double>(1));
  out->rough_tilt_deg =
      std::atan2(nz, std::fabs(z_cam.at<double>(2))) * 180.0 / CV_PI;
  out->tilt_deg = out->rough_tilt_deg;

  const double rx = rvec.at<double>(0, 0);
  const double ry = rvec.at<double>(1, 0);
  out->rough_angle_x_deg = rx * 180.0 / CV_PI;
  out->rough_angle_y_deg = ry * 180.0 / CV_PI;

  out->rough_position_x_m = tvec.at<double>(0, 0);
  out->rough_position_y_m = tvec.at<double>(1, 0);
  out->rough_position_z_m = tvec.at<double>(2, 0);
}

void fill_reprojection_metrics(
    BoardFrameMetrics *out,
    const IntrinsicsView &view,
    const cv::Mat &K,
    const cv::Mat &D,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    double cell_size_m) {
  if (out == nullptr) {
    return;
  }
  const auto stats = compute_reprojection_stats(view, K, D, rvec, tvec);
  out->has_reprojection = true;
  out->reproj_max_px = stats.max;
  out->reproj_rms_px = stats.rms;
  double sum = 0.0;
  std::vector<cv::Point2f> projected;
  cv::projectPoints(view.object_points, rvec, tvec, K, D, projected);
  for (size_t i = 0; i < projected.size(); ++i) {
    const double dx = projected[i].x - view.image_points[i].x;
    const double dy = projected[i].y - view.image_points[i].y;
    sum += std::hypot(dx, dy);
  }
  out->reproj_avg_px =
      view.image_points.empty() ? 0.0 : sum / view.image_points.size();

  if (cell_size_m > 1e-9 && !view.object_points.empty()) {
    double cell_px = 0.0;
    int pairs = 0;
    for (size_t i = 1; i < view.object_points.size(); ++i) {
      const auto &a = view.object_points[i - 1];
      const auto &b = view.object_points[i];
      const double obj_d = cv::norm(a - b);
      if (obj_d < 1e-6) {
        continue;
      }
      const double img_d = cv::norm(view.image_points[i] - view.image_points[i - 1]);
      cell_px += img_d * cell_size_m / obj_d;
      ++pairs;
    }
    if (pairs > 0) {
      cell_px /= pairs;
      if (cell_px > 1e-6) {
        out->reproj_max_relative_percent = stats.max / cell_px * 100.0;
        out->reproj_avg_relative_percent = out->reproj_avg_px / cell_px * 100.0;
        out->reproj_rms_relative_percent = stats.rms / cell_px * 100.0;
      }
    }
  }
}

}  // namespace

BoardFrameFingerprint fingerprint_from_correspondence(
    const Correspondence &corr,
    int image_width,
    int image_height) {
  BoardFrameFingerprint fp;
  if (corr.image_points.rows() < 4 || image_width <= 0 || image_height <= 0) {
    return fp;
  }
  const auto img_pts = to_image_points(corr);
  BoardFrameMetrics m;
  fill_geometry_metrics(&m, img_pts, image_width, image_height);
  fp.centroid_x = m.centroid_x_norm;
  fp.centroid_y = m.centroid_y_norm;
  fp.normalized_skew = m.normalized_skew;
  fp.normalized_size = m.area_ratio;
  fp.tilt_deg = m.tilt_deg;
  return fp;
}

BoardFrameMetrics compute_board_frame_metrics(
    const Correspondence &corr,
    int image_width,
    int image_height,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    double cell_size_m) {
  BoardFrameMetrics out;
  if (corr.image_points.rows() < 4) {
    return out;
  }
  out.detected = true;
  const auto img_pts = to_image_points(corr);
  fill_geometry_metrics(&out, img_pts, image_width, image_height);

  IntrinsicsView view;
  view.object_points = to_object_points(corr);
  view.image_points = img_pts;
  fill_view_fingerprint(&view, image_width, image_height);
  out.centroid_x_norm = view.centroid_x;
  out.centroid_y_norm = view.centroid_y;
  out.tilt_deg = view.tilt_deg;
  out.area_ratio = view.area_ratio;

  cv::Mat K = camera_matrix.empty()
      ? make_initial_camera_matrix(image_width, image_height)
      : camera_matrix;
  cv::Mat D = dist_coeffs.empty() ? cv::Mat::zeros(5, 1, CV_64F) : dist_coeffs;
  cv::Mat rv, tv;
  if (solve_board_pose(view, K, D, &rv, &tv)) {
    fill_pose_metrics(&out, rv, tv);
    fill_reprojection_metrics(&out, view, K, D, rv, tv, cell_size_m);
  }
  return out;
}

BoardFrameMetrics compute_board_frame_metrics(
    const IntrinsicsView &view,
    int image_width,
    int image_height,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    double cell_size_m) {
  Correspondence corr;
  corr.image_points.resize(view.image_points.size(), 2);
  corr.object_points.resize(view.object_points.size(), 3);
  for (size_t i = 0; i < view.image_points.size(); ++i) {
    corr.image_points(static_cast<int>(i), 0) = view.image_points[i].x;
    corr.image_points(static_cast<int>(i), 1) = view.image_points[i].y;
    corr.object_points(static_cast<int>(i), 0) = view.object_points[i].x;
    corr.object_points(static_cast<int>(i), 1) = view.object_points[i].y;
    corr.object_points(static_cast<int>(i), 2) = view.object_points[i].z;
  }
  return compute_board_frame_metrics(
      corr, image_width, image_height, camera_matrix, dist_coeffs, cell_size_m);
}

}  // namespace core
}  // namespace hs_calib
