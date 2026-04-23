#include "calib_sim_isaac/calib_node.hpp"
#include "calib_sim_isaac/calib_node_aruco.hpp"

#include <opencv2/imgproc.hpp>

#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

namespace calib_sim_isaac
{
namespace
{
constexpr int kArucoNoInputFlip = -1000;

void mapArucoCornersFromFlippedToOriginal(
  std::vector<std::vector<cv::Point2f>> & corners, int flip_code, int img_w, int img_h)
{
  if (flip_code == kArucoNoInputFlip || img_w <= 0 || img_h <= 0) {
    return;
  }
  const float wf = static_cast<float>(img_w - 1);
  const float hf = static_cast<float>(img_h - 1);
  for (auto & marker : corners) {
    for (auto & p : marker) {
      if (flip_code == 1) {
        p.x = wf - p.x;
      } else if (flip_code == 0) {
        p.y = hf - p.y;
      } else if (flip_code == -1) {
        p.x = wf - p.x;
        p.y = hf - p.y;
      }
    }
  }
}

cv::Ptr<cv::aruco::DetectorParameters> makeMainDetectorParams(bool eye_in_hand)
{
  auto detector_params = cv::makePtr<cv::aruco::DetectorParameters>();
  detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params->cornerRefinementWinSize = 9;
  detector_params->cornerRefinementMaxIterations = 50;
  detector_params->cornerRefinementMinAccuracy = 0.005;
  detector_params->adaptiveThreshWinSizeMin = 7;
  detector_params->adaptiveThreshWinSizeMax = 31;
  detector_params->adaptiveThreshWinSizeStep = 4;
  detector_params->adaptiveThreshConstant = 7.0;
  detector_params->minMarkerDistanceRate = eye_in_hand ? 0.03 : 0.01;
  detector_params->polygonalApproxAccuracyRate = 0.03;
  detector_params->minMarkerPerimeterRate = 0.02;
  detector_params->maxMarkerPerimeterRate = 5.0;
  return detector_params;
}

cv::Point2f markerCenter(const std::vector<cv::Point2f> & c)
{
  cv::Point2f ctr(0.0F, 0.0F);
  if (c.empty()) {
    return ctr;
  }
  for (const auto & p : c) {
    ctr.x += p.x;
    ctr.y += p.y;
  }
  ctr.x /= static_cast<float>(c.size());
  ctr.y /= static_cast<float>(c.size());
  return ctr;
}

double markerArea(const std::vector<cv::Point2f> & c)
{
  return std::abs(cv::contourArea(c));
}

int selectTargetIndex(
  const std::vector<std::vector<cv::Point2f>> & cands, const std::vector<int> & ids_in,
  int marker_id, bool has_prev_target_marker_center, const cv::Point2f & prev_target_marker_center,
  const char * stage, const std::function<void(const std::string &)> & log_cb)
{
  std::vector<int> target_indices;
  for (std::size_t i = 0; i < ids_in.size(); ++i) {
    if (ids_in[i] == marker_id && i < cands.size() && !cands[i].empty()) {
      target_indices.push_back(static_cast<int>(i));
    }
  }
  if (target_indices.empty()) {
    return -1;
  }
  if (target_indices.size() == 1U) {
    return target_indices.front();
  }
  int best = target_indices.front();
  if (has_prev_target_marker_center) {
    double best_dist = std::numeric_limits<double>::max();
    double best_area = -1.0;
    for (int idx : target_indices) {
      const cv::Point2f ctr = markerCenter(cands[static_cast<std::size_t>(idx)]);
      const double dist = cv::norm(ctr - prev_target_marker_center);
      const double area = markerArea(cands[static_cast<std::size_t>(idx)]);
      if (dist < best_dist - 1e-6 || (std::abs(dist - best_dist) <= 1e-6 && area > best_area)) {
        best_dist = dist;
        best_area = area;
        best = idx;
      }
    }
  } else {
    double best_area = -1.0;
    for (int idx : target_indices) {
      const double area = markerArea(cands[static_cast<std::size_t>(idx)]);
      if (area > best_area) {
        best_area = area;
        best = idx;
      }
    }
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "detect_diag multi_target_id_select stage=" << stage
      << " id=" << marker_id
      << " candidate_count=" << target_indices.size()
      << " selected_index=" << best
      << " selected_area=" << markerArea(cands[static_cast<std::size_t>(best)]);
  if (has_prev_target_marker_center) {
    const cv::Point2f ctr = markerCenter(cands[static_cast<std::size_t>(best)]);
    oss << " dist_to_prev_px=" << cv::norm(ctr - prev_target_marker_center);
  }
  log_cb(oss.str());
  return best;
}

bool runMainDetectionPasses(
  const cv::Mat & proc_bgr,
  int img_w,
  int img_h,
  const cv::Ptr<cv::aruco::Dictionary> & dict_ptr,
  const cv::Ptr<cv::aruco::DetectorParameters> & detector_params,
  double detect_scale,
  int marker_id,
  bool has_prev_target_marker_center,
  const cv::Point2f & prev_target_marker_center,
  const std::function<void(const std::string &)> & log_cb,
  std::vector<std::vector<cv::Point2f>> & corners,
  std::vector<int> & ids,
  int & found_index)
{
  struct FlipTry
  {
    int code;
    const char * name;
  };
  const FlipTry flip_tries[] = {
    {kArucoNoInputFlip, "none"},
    {1, "horizontal"},
    {0, "vertical"},
    {-1, "both"},
  };

  for (const auto & ft : flip_tries) {
    cv::Mat bgr_in;
    if (ft.code == kArucoNoInputFlip) {
      bgr_in = proc_bgr;
    } else {
      cv::flip(proc_bgr, bgr_in, ft.code);
    }

    cv::Mat gray;
    cv::cvtColor(bgr_in, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_eq;
    {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(gray, gray_eq);
    }
    cv::Mat blur_eq;
    cv::GaussianBlur(gray_eq, blur_eq, cv::Size(0, 0), 1.2);
    cv::Mat gray_sharp;
    cv::addWeighted(gray_eq, 1.6, blur_eq, -0.6, 0.0, gray_sharp);

    cv::Mat gray_detect;
    cv::resize(gray_sharp, gray_detect, cv::Size(), detect_scale, detect_scale, cv::INTER_CUBIC);

    auto detect_one_pass =
      [&](const cv::Mat & src, bool need_scale_back, double scale_factor,
        std::vector<std::vector<cv::Point2f>> & out_corners, std::vector<int> & out_ids) {
        cv::aruco::detectMarkers(src, dict_ptr, out_corners, out_ids, detector_params);
        if (need_scale_back) {
          for (auto & marker : out_corners) {
            for (auto & p : marker) {
              p.x = static_cast<float>(p.x / scale_factor);
              p.y = static_cast<float>(p.y / scale_factor);
            }
          }
        }
        for (auto & marker : out_corners) {
          cv::cornerSubPix(
            gray_sharp, marker, cv::Size(7, 7), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
        }
      };

    corners.clear();
    ids.clear();
    detect_one_pass(gray_detect, true, detect_scale, corners, ids);
    if (ids.empty()) {
      detect_one_pass(gray_eq, false, 1.0, corners, ids);
    }

    found_index = selectTargetIndex(
      corners, ids, marker_id, has_prev_target_marker_center, prev_target_marker_center, "main",
      log_cb);
    if (found_index >= 0) {
      if (ft.code != kArucoNoInputFlip) {
        mapArucoCornersFromFlippedToOriginal(corners, ft.code, img_w, img_h);
        log_cb(std::string("detect_diag aruco_mirror_fix input_") + ft.name);
      }
      return true;
    }
  }
  return false;
}

bool runEyeToHandRetryPasses(
  const cv::Mat & proc_bgr,
  const cv::Ptr<cv::aruco::Dictionary> & dict_ptr,
  int marker_id,
  bool has_prev_target_marker_center,
  const cv::Point2f & prev_target_marker_center,
  const std::function<void(const std::string &)> & log_cb,
  std::vector<std::vector<cv::Point2f>> & corners,
  std::vector<int> & ids,
  int & found_index)
{
  cv::Mat g0;
  cv::cvtColor(proc_bgr, g0, cv::COLOR_BGR2GRAY);
  cv::Mat g_eq;
  {
    cv::Ptr<cv::CLAHE> clahe2 = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe2->apply(g0, g_eq);
  }
  cv::Mat g_up2;
  cv::resize(g0, g_up2, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
  cv::Mat g_eq_up;
  cv::resize(g_eq, g_eq_up, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
  const auto make_loose_params = [](double min_perim_rate, double min_dist_rate) {
      auto pr = cv::makePtr<cv::aruco::DetectorParameters>();
      pr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      pr->cornerRefinementWinSize = 5;
      pr->adaptiveThreshWinSizeMin = 3;
      pr->adaptiveThreshWinSizeMax = 45;
      pr->adaptiveThreshWinSizeStep = 2;
      pr->adaptiveThreshConstant = 5.0;
      pr->minMarkerPerimeterRate = min_perim_rate;
      pr->minMarkerDistanceRate = min_dist_rate;
      pr->polygonalApproxAccuracyRate = 0.05;
      return pr;
    };
  const auto try_e2h = [&](const cv::Mat & gsrc, const cv::Ptr<cv::aruco::DetectorParameters> & pr,
                         bool halve_to_full_res, const char * tag) -> bool {
      std::vector<std::vector<cv::Point2f>> c_try;
      std::vector<int> id_try;
      cv::aruco::detectMarkers(gsrc, dict_ptr, c_try, id_try, pr);
      if (halve_to_full_res) {
        for (auto & m : c_try) {
          for (auto & q : m) {
            q.x = static_cast<float>(q.x * 0.5F);
            q.y = static_cast<float>(q.y * 0.5F);
          }
        }
      }
      for (auto & m : c_try) {
        cv::cornerSubPix(
          g0, m, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.01));
      }
      const int idx = selectTargetIndex(
        c_try, id_try, marker_id, has_prev_target_marker_center, prev_target_marker_center, tag,
        log_cb);
      if (idx < 0) {
        return false;
      }
      found_index = idx;
      corners = std::move(c_try);
      ids = std::move(id_try);
      log_cb(std::string("detect_diag eye_to_hand_retry: ok (") + tag + ")");
      return true;
    };
  const cv::Ptr<cv::aruco::DetectorParameters> p1 = make_loose_params(0.002, 0.0008);
  const cv::Ptr<cv::aruco::DetectorParameters> p2 = make_loose_params(0.001, 0.0005);
  if (try_e2h(g_eq, p1, false, "CLAHE+loose1")) {
    return true;
  }
  if (try_e2h(g0, p1, false, "gray+loose1")) {
    return true;
  }
  if (try_e2h(g_eq, p2, false, "CLAHE+loose2")) {
    return true;
  }
  if (try_e2h(g_up2, p1, true, "2x_gray+loose1")) {
    return true;
  }
  if (try_e2h(g_eq_up, p1, true, "2x_clahe+loose1")) {
    return true;
  }
  log_cb("detect_diag eye_to_hand_retry: all passes failed (target id not found)");
  return false;
}

bool publishMarkerPresenceDiagnostics(
  const cv::Mat & frame_bgr,
  const std::vector<int> & ids,
  int dict_id,
  int marker_id,
  std::string & fail_reason,
  const std::function<void(const std::string &)> & log_cb)
{
  if (ids.empty()) {
    std::ostringstream oss;
    oss << "detect_diag no_marker"
        << " image=" << frame_bgr.cols << "x" << frame_bgr.rows
        << " dict_id=" << dict_id
        << " target_id=" << marker_id;
    log_cb(oss.str());
    fail_reason = "no_marker";
    return false;
  }

  std::ostringstream oss;
  oss << "detect_diag marker_id_mismatch target_id=" << marker_id << " ids=[";
  for (std::size_t i = 0; i < ids.size(); ++i) {
    oss << ids[i];
    if (i + 1 < ids.size()) {
      oss << ",";
    }
  }
  oss << "]";
  log_cb(oss.str());
  fail_reason = "marker_id_mismatch";
  return false;
}

void drawSelectedMarkerOverlay(
  cv::Mat & annotated,
  const std::vector<std::vector<cv::Point2f>> & corners,
  int found_index,
  int marker_id)
{
  if (found_index < 0 || static_cast<std::size_t>(found_index) >= corners.size()) {
    return;
  }
  const auto & c = corners[static_cast<std::size_t>(found_index)];
  if (c.size() != 4U) {
    return;
  }
  cv::line(annotated, c[0], c[1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::line(annotated, c[1], c[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::line(annotated, c[2], c[3], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::line(annotated, c[3], c[0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::circle(annotated, c[0], 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
  cv::putText(
    annotated, std::to_string(marker_id), c[0] + cv::Point2f(2.0F, -6.0F),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

bool validateMarkerSizeAndReport(
  const cv::Mat & proc_bgr,
  const std::vector<int> & ids,
  int found_index,
  const std::vector<cv::Point2f> & marker_corners,
  double marker_bbox_ratio_min,
  double marker_bbox_ratio_max,
  std::string & fail_reason,
  const std::function<void(const std::string &)> & log_cb)
{
  const cv::Rect bbox = cv::boundingRect(marker_corners);
  const double marker_area_px = std::abs(cv::contourArea(marker_corners));
  double perimeter_px = 0.0;
  for (std::size_t i = 0; i < marker_corners.size(); ++i) {
    const cv::Point2f & p0 = marker_corners[i];
    const cv::Point2f & p1 = marker_corners[(i + 1U) % marker_corners.size()];
    perimeter_px += cv::norm(p0 - p1);
  }
  const double image_area = static_cast<double>(proc_bgr.cols) * static_cast<double>(proc_bgr.rows);
  const double marker_area_ratio = image_area > 0.0 ? (marker_area_px / image_area) : 0.0;

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "detect_diag marker_id=" << ids[found_index]
        << " perimeter_px=" << perimeter_px
        << " bbox=" << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height
        << " marker_area_ratio=" << marker_area_ratio;
    log_cb(oss.str());
  }

  if (marker_area_ratio < marker_bbox_ratio_min) {
    std::ostringstream gate_ss;
    gate_ss << std::fixed << std::setprecision(4)
            << "detect_diag marker_too_small gate marker_bbox_ratio_min=" << marker_bbox_ratio_min
            << " marker_area_ratio=" << marker_area_ratio;
    log_cb(gate_ss.str());
    fail_reason = "marker_too_small";
    return false;
  }
  if (marker_area_ratio > marker_bbox_ratio_max) {
    fail_reason = "marker_too_large";
    return false;
  }
  return true;
}

bool solvePnPAndValidateReproj(
  const std::vector<cv::Point2f> & marker_corners_for_pnp,
  double marker_length_m,
  const cv::Mat & proc_camera_matrix,
  const cv::Mat & proc_dist_coeffs,
  cv::Mat & annotated,
  cv::Mat & out_rvec,
  cv::Mat & out_tvec,
  double & out_reproj_err_px,
  std::string & out_fail_reason,
  const std::function<void(const std::string &)> & log_cb)
{
  const double h = marker_length_m * 0.5;
  std::vector<cv::Point3f> object_points_proj{
    cv::Point3f(-h, h, 0.0), cv::Point3f(h, h, 0.0),
    cv::Point3f(h, -h, 0.0), cv::Point3f(-h, -h, 0.0)
  };

  bool pnp_ok = cv::solvePnP(
    object_points_proj, marker_corners_for_pnp, proc_camera_matrix, proc_dist_coeffs, out_rvec, out_tvec,
    false, cv::SOLVEPNP_IPPE_SQUARE);
  if (!pnp_ok) {
    pnp_ok = cv::solvePnP(
      object_points_proj, marker_corners_for_pnp, proc_camera_matrix, proc_dist_coeffs, out_rvec, out_tvec,
      false, cv::SOLVEPNP_ITERATIVE);
  }
  if (!pnp_ok) {
    log_cb("detect_diag pose_estimation_failed");
    out_fail_reason = "pnp_failed";
    return false;
  }

  std::vector<cv::Point2f> reproj;
  cv::projectPoints(object_points_proj, out_rvec, out_tvec, proc_camera_matrix, proc_dist_coeffs, reproj);
  out_reproj_err_px = 0.0;
  if (reproj.size() == marker_corners_for_pnp.size()) {
    for (std::size_t i = 0; i < reproj.size(); ++i) {
      out_reproj_err_px += cv::norm(reproj[i] - marker_corners_for_pnp[i]);
    }
    out_reproj_err_px /= static_cast<double>(reproj.size());
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "detect_diag pnp_reproj_err_px=" << out_reproj_err_px;
    log_cb(oss.str());
  }

  constexpr double kReprojWarnPx = 6.0;
  constexpr double kReprojHardRejectPx = 14.0;
  if (out_reproj_err_px > kReprojWarnPx) {
    std::ostringstream warn_ss;
    warn_ss << std::fixed << std::setprecision(3)
            << "detect_warn pnp_reproj_large_px=" << out_reproj_err_px;
    log_cb(warn_ss.str());
    cv::putText(
      annotated, "WARN: pnp_reproj_large " + std::to_string(out_reproj_err_px) + "px",
      cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
  }
  if (out_reproj_err_px > kReprojHardRejectPx) {
    out_fail_reason = "pnp_reproj_too_large_hard";
    return false;
  }

  cv::drawFrameAxes(
    annotated, proc_camera_matrix, proc_dist_coeffs, out_rvec, out_tvec,
    static_cast<float>(marker_length_m * 0.6), 2);

  constexpr double kProjectedBorderDrawMaxReprojPx = 2.5;
  if (reproj.size() == 4U && out_reproj_err_px <= kProjectedBorderDrawMaxReprojPx) {
    cv::line(annotated, reproj[0], reproj[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, reproj[1], reproj[2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, reproj[2], reproj[3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, reproj[3], reproj[0], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  } else if (reproj.size() == 4U) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "detect_diag projected_border_suppressed reproj_px=" << out_reproj_err_px
        << " threshold_px=" << kProjectedBorderDrawMaxReprojPx;
    log_cb(oss.str());
  }
  return true;
}

bool prepareDetectionFrameAndLogCameraModel(
  const cv::Mat & frame_bgr,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs,
  cv::Mat & proc_bgr,
  cv::Mat & proc_camera_matrix,
  cv::Mat & proc_dist_coeffs,
  bool & used_undistort,
  std::string & fail_reason,
  const std::function<void(const std::string &)> & log_cb)
{
  if (frame_bgr.cols <= 0 || frame_bgr.rows <= 0) {
    fail_reason = "no_image";
    return false;
  }

  proc_bgr = frame_bgr;
  proc_camera_matrix = camera_matrix.clone();
  proc_dist_coeffs = dist_coeffs.clone();
  used_undistort = false;

  const bool camera_ready =
    (camera_matrix.rows == 3 && camera_matrix.cols == 3 && !camera_matrix.empty());
  if (camera_ready && !dist_coeffs.empty()) {
    cv::Mat dist_abs;
    cv::absdiff(dist_coeffs, cv::Scalar::all(0), dist_abs);
    const bool has_distortion = cv::countNonZero(dist_abs.reshape(1) > 1e-12) > 0;
    if (has_distortion) {
      cv::undistort(frame_bgr, proc_bgr, camera_matrix, dist_coeffs);
      proc_dist_coeffs = cv::Mat::zeros(dist_coeffs.size(), dist_coeffs.type());
      used_undistort = true;
    }
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "detect_diag camera_model"
      << " raw=" << frame_bgr.cols << "x" << frame_bgr.rows
      << " proc=" << proc_bgr.cols << "x" << proc_bgr.rows
      << " fx=" << proc_camera_matrix.at<double>(0, 0)
      << " fy=" << proc_camera_matrix.at<double>(1, 1)
      << " cx=" << proc_camera_matrix.at<double>(0, 2)
      << " cy=" << proc_camera_matrix.at<double>(1, 2)
      << " undistort=" << (used_undistort ? "true" : "false");
  log_cb(oss.str());
  return true;
}
}  // namespace

double EstimateReprojMmAtTargetDepth(
  double reproj_px, const cv::Mat & t_target_to_cam, const cv::Mat & camera_matrix)
{
  if (reproj_px <= 0.0 || t_target_to_cam.rows < 3 || t_target_to_cam.cols < 1 ||
    camera_matrix.rows < 3 || camera_matrix.cols < 3)
  {
    return 0.0;
  }
  const double z_m = std::abs(t_target_to_cam.at<double>(2, 0));
  const double fx = camera_matrix.at<double>(0, 0);
  const double fy = camera_matrix.at<double>(1, 1);
  if (z_m <= 1e-9 || fx <= 1e-9 || fy <= 1e-9) {
    return 0.0;
  }
  const double f_avg = 0.5 * (fx + fy);
  return reproj_px * (z_m / f_avg) * 1000.0;
}

bool CalibNode::detectTargetPoseInCamera(
  const cv::Mat & frame_bgr, cv::Mat & r_target_to_cam, cv::Mat & t_target_to_cam,
  double & out_mean_corner_reproj_px,
  std::string & fail_reason, cv::Mat & annotated, std::vector<int> & detected_ids)
{
  out_mean_corner_reproj_px = 0.0;
  const auto log_cb = [this](const std::string & s) { publishLog(s); };
  cv::Mat proc_bgr;
  cv::Mat proc_camera_matrix;
  cv::Mat proc_dist_coeffs;
  bool used_undistort = false;
  if (!prepareDetectionFrameAndLogCameraModel(
      frame_bgr, camera_matrix_, dist_coeffs_, proc_bgr, proc_camera_matrix, proc_dist_coeffs,
      used_undistort, fail_reason, log_cb))
  {
    return false;
  }
  annotated = proc_bgr.clone();
  const int img_w = frame_bgr.cols;
  const int img_h = frame_bgr.rows;

  const auto dict_ptr = cv::makePtr<cv::aruco::Dictionary>(aruco_dict_);
  auto detector_params = makeMainDetectorParams(eye_in_hand_);

  const double detect_scale = 2.0;

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  int found_index = -1;

  runMainDetectionPasses(
    proc_bgr, img_w, img_h, dict_ptr, detector_params, detect_scale, marker_id_,
    has_prev_target_marker_center_, prev_target_marker_center_, log_cb, corners, ids, found_index);

  if (found_index < 0 && !eye_in_hand_ && eye_to_hand_retry_enabled_) {
    runEyeToHandRetryPasses(
      proc_bgr, dict_ptr, marker_id_, has_prev_target_marker_center_, prev_target_marker_center_,
      log_cb, corners, ids, found_index);
  }
  if (found_index < 0 && !eye_in_hand_ && !eye_to_hand_retry_enabled_) {
    publishLog("detect_diag eye_to_hand_retry: disabled");
  }

  if (found_index >= 0 && ids.size() > 1U) {
    std::ostringstream multi_ss;
    multi_ss << "detect_diag multi_marker using_id=" << marker_id_ << " raw_ids=[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0U) {
        multi_ss << ",";
      }
      multi_ss << ids[i];
    }
    multi_ss << "] (pnp uses target id only)";
    publishLog(multi_ss.str());
  }

  detected_ids = ids;
  if (found_index < 0) {
    has_prev_target_marker_center_ = false;
    return publishMarkerPresenceDiagnostics(
      frame_bgr, ids, aruco_dict_id_, marker_id_, fail_reason, log_cb);
  }

  drawSelectedMarkerOverlay(annotated, corners, found_index, marker_id_);

  const auto & marker_corners = corners[found_index];
  prev_target_marker_center_ = markerCenter(marker_corners);
  has_prev_target_marker_center_ = true;
  if (!validateMarkerSizeAndReport(
      proc_bgr, ids, found_index, marker_corners, marker_bbox_ratio_min_, marker_bbox_ratio_max_,
      fail_reason, log_cb))
  {
    return false;
  }

  const auto & marker_corners_for_pnp = corners[found_index];
  cv::Mat rvec, tvec;
  double reproj_err_px = 0.0;
  if (!solvePnPAndValidateReproj(
      marker_corners_for_pnp, marker_length_m_, proc_camera_matrix, proc_dist_coeffs, annotated, rvec, tvec,
      reproj_err_px, fail_reason, log_cb))
  {
    return false;
  }
  cv::Rodrigues(rvec, r_target_to_cam);
  t_target_to_cam = tvec.clone();
  out_mean_corner_reproj_px = reproj_err_px;
  fail_reason = "ok";
  detected_ids.clear();
  detected_ids.push_back(marker_id_);
  return true;
}

}  // namespace calib_sim_isaac
