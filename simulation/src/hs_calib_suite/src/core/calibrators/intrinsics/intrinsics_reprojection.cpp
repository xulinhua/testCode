#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace hs_calib {
namespace core {

void fill_view_fingerprint(IntrinsicsView *view, int image_width, int image_height) {
  if (view == nullptr || view->image_points.empty() || image_width <= 0 ||
      image_height <= 0) {
    return;
  }
  const cv::Rect box = cv::boundingRect(view->image_points);
  view->area_ratio =
      static_cast<double>(box.area()) /
      (static_cast<double>(image_width) * image_height);
  view->centroid_x =
      (box.x + box.width * 0.5) / static_cast<double>(image_width);
  view->centroid_y =
      (box.y + box.height * 0.5) / static_cast<double>(image_height);
  view->tilt_deg = std::atan2(
                       static_cast<double>(box.height),
                       static_cast<double>(std::max(1, box.width))) *
                   180.0 / CV_PI;
}

bool solve_board_pose(
    const IntrinsicsView &view,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    cv::Mat *rvec,
    cv::Mat *tvec) {
  if (rvec == nullptr || tvec == nullptr || view.object_points.size() < 4 ||
      view.object_points.size() != view.image_points.size()) {
    return false;
  }
  return cv::solvePnP(
      view.object_points, view.image_points, camera_matrix, dist_coeffs, *rvec,
      *tvec);
}

ReprojectionStats compute_reprojection_stats(
    const IntrinsicsView &view,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    const cv::Mat &rvec,
    const cv::Mat &tvec) {
  ReprojectionStats stats;
  if (view.object_points.size() < 4) {
    return stats;
  }
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
      view.object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
  double sum_sq = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const double dx = projected[i].x - view.image_points[i].x;
    const double dy = projected[i].y - view.image_points[i].y;
    const double err = std::sqrt(dx * dx + dy * dy);
    stats.max = std::max(stats.max, err);
    sum_sq += dx * dx + dy * dy;
  }
  stats.rms = std::sqrt(sum_sq / static_cast<double>(projected.size()));
  return stats;
}

void update_view_poses_and_errors(
    std::vector<IntrinsicsView> *views,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs) {
  if (views == nullptr) {
    return;
  }
  for (auto &view : *views) {
    cv::Mat rv, tv;
    if (!solve_board_pose(view, camera_matrix, dist_coeffs, &rv, &tv)) {
      continue;
    }
    view.rvec = rv;
    view.tvec = tv;
    const auto stats =
        compute_reprojection_stats(view, camera_matrix, dist_coeffs, rv, tv);
    view.reproj_rms = stats.rms;
    view.reproj_max = stats.max;
  }
}

cv::Mat make_initial_camera_matrix(int width, int height) {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  const double f = static_cast<double>(std::max(width, height));
  K.at<double>(0, 0) = f;
  K.at<double>(1, 1) = f;
  K.at<double>(0, 2) = width * 0.5;
  K.at<double>(1, 2) = height * 0.5;
  return K;
}

cv::Mat make_initial_dist_coeffs(int rational_coeffs) {
  const int n = rational_coeffs > 0 ? 8 : 5;
  return cv::Mat::zeros(n, 1, CV_64F);
}

}  // namespace core
}  // namespace hs_calib
