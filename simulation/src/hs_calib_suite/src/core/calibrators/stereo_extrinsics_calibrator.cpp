#include "hs_calib_suite/core/calibrators/stereo_extrinsics_calibrator.hpp"

#include <algorithm>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"

namespace hs_calib {
namespace core {
namespace {

std::string opt_str(
    const std::map<std::string, std::string> &config, const char *key,
    const char *def) {
  const auto it = config.find(key);
  return it == config.end() ? std::string(def) : it->second;
}

ObservationBatch filter_side(const ObservationBatch &in, const std::string &side) {
  ObservationBatch out;
  out.notes = in.notes;
  for (const auto &obs : in.items) {
    std::string tag = obs.frame_id;
    if (tag != "left" && tag != "right") {
      if (obs.source_path.rfind("left:", 0) == 0) {
        tag = "left";
      } else if (obs.source_path.rfind("right:", 0) == 0) {
        tag = "right";
      }
    }
    if (tag == side) {
      out.items.push_back(obs);
    }
  }
  return out;
}

bool load_side_intrinsics(
    const std::map<std::string, std::string> &config, const char *primary_key,
    const char *fallback_key, CameraIntrinsics *intr) {
  for (const char *key : {primary_key, fallback_key}) {
    const auto it = config.find(key);
    if (it != config.end() && !it->second.empty()) {
      return load_camera_yaml(it->second, intr);
    }
  }
  return false;
}

std::string mat_row_csv(const cv::Mat &m) {
  std::ostringstream oss;
  oss.precision(12);
  for (int r = 0; r < m.rows; ++r) {
    for (int c = 0; c < m.cols; ++c) {
      if (r || c) {
        oss << ",";
      }
      oss << m.at<double>(r, c);
    }
  }
  return oss.str();
}

}  // namespace

CalibratorInfo StereoExtrinsicsCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "stereo_extrinsics";
  info.display_name = "双目相对外参";
  info.category = "extrinsics";
  info.required_frames = {"left", "right"};
  info.supported_targets = {
      "chessboard", "charuco", "aruco_grid", "circles_symmetric", "circles_asymmetric"};
  return info;
}

CalibrationResult StereoExtrinsicsCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;

  CameraIntrinsics left_intr;
  CameraIntrinsics right_intr;
  // left_camera_yaml / right_camera_yaml；兼容 camera_yaml 作左目
  if (!load_side_intrinsics(config, "left_camera_yaml", "camera_yaml", &left_intr) ||
      !left_intr.valid) {
    result.success = false;
    result.message =
        "需要左目内参 YAML（left_camera_yaml 或 camera_yaml）：" +
        (left_intr.message.empty() ? std::string("missing") : left_intr.message);
    return result;
  }
  if (!load_side_intrinsics(config, "right_camera_yaml", "camera_yaml_right", &right_intr) ||
      !right_intr.valid) {
    result.success = false;
    result.message =
        "需要右目内参 YAML（right_camera_yaml）：" +
        (right_intr.message.empty() ? std::string("missing") : right_intr.message);
    return result;
  }

  const ObservationBatch left = filter_side(observations, "left");
  const ObservationBatch right = filter_side(observations, "right");
  if (left.items.empty() || right.items.empty()) {
    result.success = false;
    result.message =
        "左右观测不足。请用 stereo_side 分别采集左/右目（建议成对、数量尽量一致）。";
    return result;
  }

  const size_t n_pairs = std::min(left.items.size(), right.items.size());
  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_l;
  std::vector<std::vector<cv::Point2f>> img_r;
  obj_pts.reserve(n_pairs);
  img_l.reserve(n_pairs);
  img_r.reserve(n_pairs);

  int skipped = 0;
  int image_w = left_intr.image_width > 0 ? left_intr.image_width : 0;
  int image_h = left_intr.image_height > 0 ? left_intr.image_height : 0;

  for (size_t i = 0; i < n_pairs; ++i) {
    const auto &ol = left.items[i];
    const auto &orr = right.items[i];
    if (ol.correspondences.empty() || orr.correspondences.empty()) {
      ++skipped;
      continue;
    }
    const auto &cl = ol.correspondences.front();
    const auto &cr = orr.correspondences.front();
    const int nl = cl.image_points.rows();
    const int nr = cr.image_points.rows();
    const int no = cl.object_points.rows();
    if (nl < 6 || nr < 6 || no < 6 || nl != nr || nl != no) {
      ++skipped;
      continue;
    }
    if (cr.object_points.rows() != no) {
      ++skipped;
      continue;
    }

    std::vector<cv::Point3f> obj(static_cast<size_t>(no));
    std::vector<cv::Point2f> pl(static_cast<size_t>(nl));
    std::vector<cv::Point2f> pr(static_cast<size_t>(nr));
    for (int k = 0; k < no; ++k) {
      obj[static_cast<size_t>(k)] = cv::Point3f(
          static_cast<float>(cl.object_points(k, 0)),
          static_cast<float>(cl.object_points(k, 1)),
          static_cast<float>(cl.object_points(k, 2)));
      pl[static_cast<size_t>(k)] = cv::Point2f(
          static_cast<float>(cl.image_points(k, 0)),
          static_cast<float>(cl.image_points(k, 1)));
      pr[static_cast<size_t>(k)] = cv::Point2f(
          static_cast<float>(cr.image_points(k, 0)),
          static_cast<float>(cr.image_points(k, 1)));
    }
    obj_pts.push_back(std::move(obj));
    img_l.push_back(std::move(pl));
    img_r.push_back(std::move(pr));

    if (image_w <= 0 && ol.image_width > 0) {
      image_w = ol.image_width;
      image_h = ol.image_height;
    }
  }

  if (obj_pts.size() < 3) {
    result.success = false;
    result.message =
        "有效左右配对不足 3 组（点数须一致）。已跳过 " + std::to_string(skipped) +
        " 组；左 " + std::to_string(left.items.size()) + " / 右 " +
        std::to_string(right.items.size());
    return result;
  }
  if (image_w <= 0 || image_h <= 0) {
    image_w = left_intr.image_width > 0 ? left_intr.image_width : 1280;
    image_h = left_intr.image_height > 0 ? left_intr.image_height : 720;
  }

  cv::Mat K1 = left_intr.K.clone();
  cv::Mat D1 = left_intr.D.clone();
  cv::Mat K2 = right_intr.K.clone();
  cv::Mat D2 = right_intr.D.clone();
  if (D1.empty()) {
    D1 = cv::Mat::zeros(5, 1, CV_64F);
  }
  if (D2.empty()) {
    D2 = cv::Mat::zeros(5, 1, CV_64F);
  }

  cv::Mat R, T, E, F;
  const int flags = cv::CALIB_FIX_INTRINSIC;
  double rms = 0.0;
  try {
    rms = cv::stereoCalibrate(
        obj_pts, img_l, img_r, K1, D1, K2, D2, cv::Size(image_w, image_h), R, T, E, F,
        flags, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
  } catch (const cv::Exception &ex) {
    result.success = false;
    result.message = std::string("stereoCalibrate 异常：") + ex.what();
    return result;
  }

  cv::Mat R1, R2, P1, P2, Q;
  try {
    cv::stereoRectify(
        K1, D1, K2, D2, cv::Size(image_w, image_h), R, T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, -1, cv::Size(image_w, image_h));
  } catch (const cv::Exception &ex) {
    result.success = false;
    result.message = std::string("stereoRectify 异常：") + ex.what();
    return result;
  }

  std::string parent = opt_str(config, "parent_frame", "left");
  std::string child = opt_str(config, "child_frame", "right");
  if (parent.empty()) {
    parent = "left";
  }
  if (child.empty()) {
    child = "right";
  }

  // OpenCV：X_left = R * X_right + T  → 存为 T_parent_child，使 p_left = T * p_right
  Eigen::Matrix4d T_lr = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_lr(r, c) = R.at<double>(r, c);
    }
    T_lr(r, 3) = T.at<double>(r, 0);
  }
  result.transforms[parent][child] = T_lr;

  result.intrinsics_meta["parent_frame"] = parent;
  result.intrinsics_meta["child_frame"] = child;
  result.intrinsics_meta["mode"] = "stereo_extrinsics";
  result.intrinsics_meta["image_width"] = std::to_string(image_w);
  result.intrinsics_meta["image_height"] = std::to_string(image_h);
  result.intrinsics_meta["R"] = mat_row_csv(R);
  result.intrinsics_meta["T"] = mat_row_csv(T);
  result.intrinsics_meta["R1"] = mat_row_csv(R1);
  result.intrinsics_meta["R2"] = mat_row_csv(R2);
  result.intrinsics_meta["P1"] = mat_row_csv(P1);
  result.intrinsics_meta["P2"] = mat_row_csv(P2);
  result.intrinsics_meta["Q"] = mat_row_csv(Q);

  result.metrics["stereo_rms"] = rms;
  result.metrics["num_pairs"] = static_cast<double>(obj_pts.size());
  result.metrics["num_views_left"] = static_cast<double>(left.items.size());
  result.metrics["num_views_right"] = static_cast<double>(right.items.size());
  result.metrics["skipped_pairs"] = static_cast<double>(skipped);

  const double baseline = cv::norm(T);
  result.metrics["baseline_m"] = baseline;
  result.score = static_cast<float>(std::max(0.0, 1.0 - rms / 2.0));

  std::ostringstream msg;
  msg << "stereo_extrinsics ok: pairs=" << obj_pts.size() << " rms=" << rms
      << " baseline=" << baseline << " m";
  if (skipped > 0) {
    msg << " (skipped " << skipped << ")";
  }
  result.message = msg.str();
  result.success = true;
  return result;
}

HS_CALIB_REGISTER("stereo_extrinsics", StereoExtrinsicsCalibrator);

}  // namespace core
}  // namespace hs_calib
