#pragma once

#include <opencv2/core.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_view.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief Tier4 Detection results + Single-shot 指标（§6–7）
struct BoardFrameMetrics {
  bool detected = false;

  double rough_tilt_deg = 0.0;
  double rough_angle_x_deg = 0.0;
  double rough_angle_y_deg = 0.0;
  double rough_position_x_m = 0.0;
  double rough_position_y_m = 0.0;
  double rough_position_z_m = 0.0;

  double normalized_skew = 0.0;
  double relative_area_percent = 0.0;
  double linear_error_rows_rms_px = 0.0;
  double linear_error_cols_rms_px = 0.0;
  double aspect_ratio = 0.0;

  bool has_reprojection = false;
  double reproj_max_px = 0.0;
  double reproj_avg_px = 0.0;
  double reproj_rms_px = 0.0;
  double reproj_max_relative_percent = 0.0;
  double reproj_avg_relative_percent = 0.0;
  double reproj_rms_relative_percent = 0.0;

  double centroid_x_norm = 0.5;
  double centroid_y_norm = 0.5;
  double area_ratio = 0.0;
  double tilt_deg = 0.0;
};

/// \brief 2D 指纹（采集冗余判定）
struct BoardFrameFingerprint {
  double centroid_x = 0.5;
  double centroid_y = 0.5;
  double normalized_skew = 0.0;
  double normalized_size = 0.0;
  double tilt_deg = 0.0;
  double rough_angle_x_deg = 0.0;
  double rough_angle_y_deg = 0.0;
  cv::Vec3d position_m{0.0, 0.0, 0.0};
  bool has_pose = false;
};

BoardFrameFingerprint fingerprint_from_correspondence(
    const Correspondence &corr, int image_width, int image_height);

BoardFrameMetrics compute_board_frame_metrics(
    const Correspondence &corr,
    int image_width,
    int image_height,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    double cell_size_m = 0.025);

BoardFrameMetrics compute_board_frame_metrics(
    const IntrinsicsView &view,
    int image_width,
    int image_height,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    double cell_size_m = 0.025);

}  // namespace core
}  // namespace hs_calib
