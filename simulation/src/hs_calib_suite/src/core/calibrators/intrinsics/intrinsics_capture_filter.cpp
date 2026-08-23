#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_capture_filter.hpp"

#include <vector>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_pipeline.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

namespace hs_calib {
namespace core {
namespace {

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

}  // namespace

void update_provisional_intrinsics(
    const ObservationBatch &batch,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    ProvisionalIntrinsics *out,
    int max_fast_samples) {
  if (out == nullptr || image_width <= 0 || image_height <= 0) {
    return;
  }
  ObservationBatch subset;
  if (max_fast_samples > 0 &&
      static_cast<int>(batch.items.size()) > max_fast_samples) {
    const int skip =
        static_cast<int>(batch.items.size()) - max_fast_samples;
    subset.items.assign(
        batch.items.begin() + skip, batch.items.end());
  } else {
    subset = batch;
  }
  const auto views = build_intrinsics_views(subset);
  if (views.size() < 6) {
    out->camera_matrix = make_initial_camera_matrix(image_width, image_height);
    out->dist_coeffs = make_initial_dist_coeffs(profile.rational_coeffs);
    out->valid = false;
    return;
  }
  const auto solved = run_intrinsics_pipeline(
      views, image_width, image_height, profile, {});
  if (!solved.ok) {
    out->valid = false;
    return;
  }
  out->camera_matrix = solved.camera_matrix;
  out->dist_coeffs = solved.dist_coeffs;
  out->valid = true;
}

bool capture_passes_reprojection_filter(
    const Correspondence &corr,
    int image_width,
    int image_height,
    const ProvisionalIntrinsics &model,
    const IntrinsicsProfile &profile,
    double *rms_out,
    double *max_out) {
  if (!profile.filter_capture_by_reproj) {
    return true;
  }
  if (corr.image_points.rows() < 6) {
    return false;
  }
  cv::Mat K = model.valid
      ? model.camera_matrix
      : make_initial_camera_matrix(image_width, image_height);
  cv::Mat D = model.valid
      ? model.dist_coeffs
      : make_initial_dist_coeffs(profile.rational_coeffs);

  IntrinsicsView view;
  view.object_points = to_object_points(corr);
  view.image_points = to_image_points(corr);
  cv::Mat rv, tv;
  if (!solve_board_pose(view, K, D, &rv, &tv)) {
    return false;
  }
  const auto stats = compute_reprojection_stats(view, K, D, rv, tv);
  if (rms_out != nullptr) {
    *rms_out = stats.rms;
  }
  if (max_out != nullptr) {
    *max_out = stats.max;
  }
  return stats.max < profile.capture_max_reproj_error &&
         stats.rms < profile.capture_max_rms_reproj_error;
}

}  // namespace core
}  // namespace hs_calib
