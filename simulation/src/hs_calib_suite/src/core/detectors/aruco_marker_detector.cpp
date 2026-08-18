#include "hs_calib_suite/core/detectors/aruco_marker_detector.hpp"

#include <mutex>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace hs_calib {
namespace core {

namespace {

std::mutex g_last_dict_mu;
std::string g_last_hit_dict;

cv::Ptr<cv::aruco::DetectorParameters> make_params() {
  auto p = cv::makePtr<cv::aruco::DetectorParameters>();
  p->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  p->cornerRefinementWinSize = 5;
  p->adaptiveThreshWinSizeMin = 5;
  p->adaptiveThreshWinSizeMax = 23;
  p->adaptiveThreshWinSizeStep = 6;
  p->minMarkerPerimeterRate = 0.02;
  p->maxMarkerPerimeterRate = 4.0;
  p->polygonalApproxAccuracyRate = 0.03;
  p->minMarkerDistanceRate = 0.01;
  return p;
}

/// \brief 标准实物板：灰度(+CLAHE) → detectMarkers，不做翻转/放大
bool detect_once(
    const cv::Mat &bgr,
    const cv::Ptr<cv::aruco::Dictionary> &dict,
    bool use_clahe,
    std::vector<std::vector<cv::Point2f>> *out_corners,
    std::vector<int> *out_ids) {
  out_corners->clear();
  out_ids->clear();
  if (bgr.empty() || !dict) {
    return false;
  }
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat src = gray;
  cv::Mat eq;
  if (use_clahe) {
    cv::createCLAHE(2.0, cv::Size(8, 8))->apply(gray, eq);
    src = eq;
  }
  auto params = make_params();
  cv::aruco::detectMarkers(src, dict, *out_corners, *out_ids, params);
  if (out_ids->empty()) {
    return false;
  }
  for (auto &marker : *out_corners) {
    if (marker.size() < 4) {
      continue;
    }
    cv::cornerSubPix(
        gray, marker, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
  }
  return true;
}

/// \brief 常见实物码字典（优先用户选择，再遍历）
const char *kFallbackDicts[] = {
    "DICT_6X6_1000", "DICT_6X6_250", "DICT_6X6_50", "DICT_5X5_100",
    "DICT_5X5_250",  "DICT_4X4_250", "DICT_4X4_50", "DICT_ARUCO_ORIGINAL",
};

Correspondence build_corr(
    const std::vector<std::vector<cv::Point2f>> &marker_corners,
    const std::vector<int> &marker_ids, double L) {
  Correspondence c;
  const int n_markers = static_cast<int>(marker_ids.size());
  const int n = n_markers * 4;
  c.image_points.resize(n, 2);
  c.object_points.resize(n, 3);
  c.ids.resize(static_cast<size_t>(n));
  const double local[4][2] = {{0.0, 0.0}, {L, 0.0}, {L, L}, {0.0, L}};
  for (int m = 0; m < n_markers; ++m) {
    const auto &corners = marker_corners[static_cast<size_t>(m)];
    if (corners.size() < 4) {
      continue;
    }
    const double ox =
        static_cast<double>(marker_ids[static_cast<size_t>(m)]) * (L + 0.01);
    for (int k = 0; k < 4; ++k) {
      const int row = m * 4 + k;
      c.image_points(row, 0) = corners[static_cast<size_t>(k)].x;
      c.image_points(row, 1) = corners[static_cast<size_t>(k)].y;
      c.object_points(row, 0) = ox + local[k][0];
      c.object_points(row, 1) = local[k][1];
      c.object_points(row, 2) = 0.0;
      c.ids[static_cast<size_t>(row)] =
          marker_ids[static_cast<size_t>(m)] * 10 + k;
    }
  }
  return c;
}

}  // namespace

ArucoMarkerDetector::ArucoMarkerDetector(
    const std::string &dictionary_name, double marker_length_m)
    : dictionary_name_(dictionary_name.empty() ? "DICT_6X6_1000" : dictionary_name),
      marker_length_m_(marker_length_m > 1e-6 ? marker_length_m : 0.05) {}

std::vector<Correspondence> ArucoMarkerDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr), false);
}

/// \brief Fast：当前(+缓存)字典；Thorough：再遍历若干常用字典。均无翻转/放大。
std::vector<Correspondence> ArucoMarkerDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers, bool fast) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }
  cv::Mat bgr;
  if (mat.channels() == 1) {
    cv::cvtColor(mat, bgr, cv::COLOR_GRAY2BGR);
  } else if (mat.channels() == 4) {
    cv::cvtColor(mat, bgr, cv::COLOR_BGRA2BGR);
  } else {
    bgr = mat;
  }

  std::vector<std::string> dict_order;
  dict_order.push_back(dictionary_name_);
  {
    std::lock_guard<std::mutex> lock(g_last_dict_mu);
    if (!g_last_hit_dict.empty() && g_last_hit_dict != dictionary_name_) {
      dict_order.push_back(g_last_hit_dict);
    }
  }
  if (!fast) {
    for (const char *name : kFallbackDicts) {
      bool seen = false;
      for (const auto &d : dict_order) {
        if (d == name) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        dict_order.emplace_back(name);
      }
    }
  }

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  std::string hit_dict;
  for (const std::string &dname : dict_order) {
    auto dict = make_aruco_dictionary(dname);
    if (detect_once(bgr, dict, false, &marker_corners, &marker_ids) ||
        detect_once(bgr, dict, true, &marker_corners, &marker_ids)) {
      hit_dict = dname;
      break;
    }
  }

  if (!hit_dict.empty()) {
    std::lock_guard<std::mutex> lock(g_last_dict_mu);
    g_last_hit_dict = hit_dict;
  }

  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
    markers->face_ids.assign(marker_ids.size(), -1);
    markers->dictionary_name = hit_dict;
  }
  if (marker_ids.empty()) {
    return out;
  }

  Correspondence c = build_corr(marker_corners, marker_ids, marker_length_m_);
  if (c.image_points.rows() < 4) {
    return out;
  }
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
