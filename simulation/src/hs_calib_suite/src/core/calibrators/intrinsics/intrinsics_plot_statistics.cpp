#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_plot_statistics.hpp"

#include <fstream>
#include <sstream>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"

namespace hs_calib {
namespace core {
namespace {

struct FramePose {
  double angle_x_deg = 0.0;
  double angle_y_deg = 0.0;
  double z_m = 0.0;
  bool valid = false;
};

FramePose pose_from_observation(const Observation &obs) {
  FramePose out;
  if (!obs.has_board_pose) {
    return out;
  }
  out.angle_x_deg = obs.board_rvec.x() * 180.0 / CV_PI;
  out.angle_y_deg = obs.board_rvec.y() * 180.0 / CV_PI;
  out.z_m = obs.board_tvec.z();
  out.valid = std::isfinite(out.z_m);
  return out;
}

FramePose pose_from_view(
    const IntrinsicsView &view, const cv::Mat &K, const cv::Mat &D) {
  FramePose out;
  if (K.empty() || D.empty()) {
    return out;
  }
  cv::Mat rvec, tvec;
  if (!solve_board_pose(view, K, D, &rvec, &tvec)) {
    return out;
  }
  out.angle_x_deg = rvec.at<double>(0, 0) * 180.0 / CV_PI;
  out.angle_y_deg = rvec.at<double>(1, 0) * 180.0 / CV_PI;
  out.z_m = tvec.at<double>(2, 0);
  out.valid = true;
  return out;
}

void append_frames_json(
    std::ostringstream *oss,
    const std::vector<IntrinsicsView> &views,
    const ObservationBatch &batch,
    const cv::Mat &K,
    const cv::Mat &D) {
  *oss << "[";
  bool first = true;
  for (const auto &view : views) {
    if (!first) {
      *oss << ",";
    }
    first = false;
    FramePose pose;
    if (view.source_index < batch.items.size()) {
      pose = pose_from_observation(batch.items[view.source_index]);
    }
    if (!pose.valid) {
      pose = pose_from_view(view, K, D);
    }
    *oss << "{";
    *oss << "\"angle_x\":" << pose.angle_x_deg << ",";
    *oss << "\"angle_y\":" << pose.angle_y_deg << ",";
    *oss << "\"z\":" << pose.z_m << ",";
    *oss << "\"has_z\":" << (pose.valid ? "true" : "false");
    *oss << "}";
  }
  *oss << "]";
}

void append_stage_collection(
    std::ostringstream *oss,
    const char *name,
    const std::vector<IntrinsicsView> &views,
    const ObservationBatch &batch,
    const cv::Mat &K,
    const cv::Mat &D) {
  *oss << "{\"name\":\"" << name << "\",\"pixel_points\":[";
  bool first_pt = true;
  for (const auto &view : views) {
    for (const auto &pt : view.image_points) {
      if (!first_pt) {
        *oss << ",";
      }
      first_pt = false;
      *oss << "[" << pt.x << "," << pt.y << "]";
    }
  }
  *oss << "],\"frames\":";
  append_frames_json(oss, views, batch, K, D);
  *oss << "}";
}

double view_rms(
    const IntrinsicsView &view, const cv::Mat &K, const cv::Mat &D) {
  if (K.empty() || D.empty()) {
    return -1.0;
  }
  cv::Mat rvec, tvec;
  if (!solve_board_pose(view, K, D, &rvec, &tvec)) {
    return -1.0;
  }
  return compute_reprojection_stats(view, K, D, rvec, tvec).rms;
}

void append_bar_set(
    std::ostringstream *oss,
    const char *name,
    const std::vector<IntrinsicsView> &views,
    const cv::Mat &cal_K,
    const cv::Mat &cal_D,
    int image_width,
    int image_height,
    const IntrinsicsProfile &profile,
    const std::map<std::string, std::string> &solve_config) {
  *oss << "\"" << name << "\":[";
  bool first = true;
  for (const auto &view : views) {
    const double cal = view_rms(view, cal_K, cal_D);
    const double single = compute_single_shot_view_rms(
        view, image_width, image_height, profile, solve_config);
    if (cal < 0.0 || single < 0.0) {
      continue;
    }
    if (!first) {
      *oss << ",";
    }
    first = false;
    *oss << "{"
         << "\"index\":" << view.source_index << ","
         << "\"calibrated_rms\":" << cal << ","
         << "\"singleshot_rms\":" << single
         << "}";
  }
  *oss << "]";
}

void append_residual_points(
    std::ostringstream *oss,
    const std::vector<IntrinsicsView> &views,
    const cv::Mat &K,
    const cv::Mat &D) {
  *oss << "[";
  bool first = true;
  for (const auto &view : views) {
    if (K.empty() || D.empty()) {
      continue;
    }
    cv::Mat rvec, tvec;
    if (!solve_board_pose(view, K, D, &rvec, &tvec)) {
      continue;
    }
    const double angle_x = rvec.at<double>(0, 0) * 180.0 / CV_PI;
    const double angle_y = rvec.at<double>(1, 0) * 180.0 / CV_PI;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(view.object_points, rvec, tvec, K, D, projected);
    for (size_t i = 0; i < projected.size(); ++i) {
      const double dx = projected[i].x - view.image_points[i].x;
      const double dy = projected[i].y - view.image_points[i].y;
      const double err = std::hypot(dx, dy);
      if (!first) {
        *oss << ",";
      }
      first = false;
      *oss << "{"
           << "\"u\":" << view.image_points[i].x << ","
           << "\"v\":" << view.image_points[i].y << ","
           << "\"angle_x\":" << angle_x << ","
           << "\"angle_y\":" << angle_y << ","
           << "\"err\":" << err
           << "}";
    }
  }
  *oss << "]";
}

bool write_text_file(const std::string &path, const std::string &body) {
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << body;
  return static_cast<bool>(out);
}

cv::Mat pick_pose_model(const IntrinsicsPlotInput &input) {
  if (input.has_calibrated) {
    return input.calibrated_K;
  }
  if (input.has_singleshot) {
    return input.singleshot_K;
  }
  return {};
}

cv::Mat pick_pose_dist(const IntrinsicsPlotInput &input) {
  if (input.has_calibrated) {
    return input.calibrated_D;
  }
  if (input.has_singleshot) {
    return input.singleshot_D;
  }
  return {};
}

}  // namespace

bool build_plot_pipeline_stages(
    const IntrinsicsPlotInput &input,
    IntrinsicsPipelineStageViews *stages,
    std::string *error_out) {
  if (stages == nullptr) {
    if (error_out) {
      *error_out = "输出为空";
    }
    return false;
  }
  if (!input.has_owned_batches && input.collector == nullptr) {
    if (error_out) {
      *error_out = "输入无效";
    }
    return false;
  }
  return compute_intrinsics_pipeline_stage_views(
      input.training_batch(), input.evaluation_batch(),
      input.image_width, input.image_height, input.profile, input.solve_config,
      stages, error_out);
}

bool export_collection_statistics_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out) {
  const cv::Mat K = pick_pose_model(input);
  const cv::Mat D = pick_pose_dist(input);
  const ObservationBatch training_batch = input.training_batch();
  const ObservationBatch evaluation_batch = input.evaluation_batch();
  std::ostringstream oss;
  oss << "{";
  oss << "\"image_width\":" << input.image_width << ",";
  oss << "\"image_height\":" << input.image_height << ",";
  oss << "\"heatmap_cells\":" << input.collector_params.heatmap_cells << ",";
  oss << "\"rotation_angle_res\":"
      << input.extras.viz_tilt_resolution_deg << ",";
  oss << "\"max_tilt_deg\":" << input.extras.viz_max_tilt_deg << ",";
  oss << "\"z_bins\":" << input.extras.viz_z_cells << ",";
  oss << "\"stages\":[";
  append_stage_collection(&oss, "Training", stages.training, training_batch, K, D);
  oss << ",";
  append_stage_collection(
      &oss, "Pre rejection inliers", stages.pre_rejection_inliers, training_batch,
      K, D);
  oss << ",";
  append_stage_collection(
      &oss, "Subsampled", stages.subsampled, training_batch, K, D);
  oss << ",";
  append_stage_collection(
      &oss, "Post rejection inliers", stages.post_rejection_inliers,
      training_batch, K, D);
  oss << ",";
  append_stage_collection(
      &oss, "Evaluation", stages.evaluation, evaluation_batch, K, D);
  oss << "]}";

  if (!write_text_file(path, oss.str())) {
    if (error_out) {
      *error_out = "写入 JSON 失败";
    }
    return false;
  }
  return true;
}

bool export_calibration_bars_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out) {
  if (!input.has_calibrated) {
    if (error_out) {
      *error_out = "需要标定模型";
    }
    return false;
  }
  if (input.image_width <= 0 || input.image_height <= 0) {
    if (error_out) {
      *error_out = "图像尺寸无效";
    }
    return false;
  }
  std::ostringstream oss;
  oss << "{";
  append_bar_set(
      &oss, "training", stages.training, input.calibrated_K, input.calibrated_D,
      input.image_width, input.image_height, input.profile, input.solve_config);
  oss << ",";
  append_bar_set(
      &oss, "inliers", stages.post_rejection_inliers, input.calibrated_K,
      input.calibrated_D, input.image_width, input.image_height, input.profile,
      input.solve_config);
  oss << ",";
  append_bar_set(
      &oss, "evaluation", stages.evaluation, input.calibrated_K,
      input.calibrated_D, input.image_width, input.image_height, input.profile,
      input.solve_config);
  oss << "}";

  if (!write_text_file(path, oss.str())) {
    if (error_out) {
      *error_out = "写入 JSON 失败";
    }
    return false;
  }
  return true;
}

bool export_calibration_rms_json(
    const IntrinsicsPlotInput &input,
    const IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out) {
  if (!input.has_calibrated) {
    if (error_out) {
      *error_out = "需要标定模型";
    }
    return false;
  }
  std::ostringstream oss;
  oss << "{";
  oss << "\"image_width\":" << input.image_width << ",";
  oss << "\"image_height\":" << input.image_height << ",";
  oss << "\"viz_pixel_cells\":" << input.extras.viz_pixel_cells << ",";
  oss << "\"viz_tilt_resolution_deg\":" << input.extras.viz_tilt_resolution_deg
      << ",";
  oss << "\"viz_max_tilt_deg\":" << input.extras.viz_max_tilt_deg << ",";
  oss << "\"sets\":[";
  oss << "{\"name\":\"Training\",\"points\":";
  append_residual_points(
      &oss, stages.training, input.calibrated_K, input.calibrated_D);
  oss << "},";
  oss << "{\"name\":\"Inliers\",\"points\":";
  append_residual_points(
      &oss, stages.post_rejection_inliers, input.calibrated_K,
      input.calibrated_D);
  oss << "},";
  oss << "{\"name\":\"Evaluation\",\"points\":";
  append_residual_points(
      &oss, stages.evaluation, input.calibrated_K, input.calibrated_D);
  oss << "}";
  oss << "]}";

  if (!write_text_file(path, oss.str())) {
    if (error_out) {
      *error_out = "写入 JSON 失败";
    }
    return false;
  }
  return true;
}

}  // namespace core
}  // namespace hs_calib
