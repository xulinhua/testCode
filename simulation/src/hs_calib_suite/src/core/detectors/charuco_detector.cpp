#include "hs_calib_suite/core/detectors/charuco_detector.hpp"

#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {

/// \brief 绑定平面 ChArUco 靶标（含字典 / 板几何）
CharucoDetector::CharucoDetector(CharucoTarget target, CharucoDetectorParams params)
    : target_(std::move(target)), params_(params) {}

/// \brief DetectorBase 入口：忽略外部 target，使用成员板
std::vector<Correspondence> CharucoDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr));
}

namespace {

/// \brief 用指定 CharucoBoard 跑一轮检测
bool run_charuco(
    const cv::aruco::CharucoBoard &board, const cv::Mat &mat, const cv::Mat &K,
    const cv::Mat &D, std::vector<cv::Point2f> *charuco_corners,
    std::vector<int> *charuco_ids, std::vector<std::vector<cv::Point2f>> *marker_corners,
    std::vector<int> *marker_ids,
    const cv::aruco::DetectorParameters &det_params) {
  return charuco_detect_corners(
      board, mat, K, D, *charuco_corners, *charuco_ids, marker_corners, marker_ids,
      det_params);
}

}  // namespace

/// \brief 平面 ChArUco：检码 → 插值角点；失败时仍回传已检出的 ArUco
std::vector<Correspondence> CharucoDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty() || !target_.board()) {
    return out;
  }

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  const cv::Mat gray = to_gray(mat);
  const cv::Mat K = guess_K(mat.size());
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  cv::aruco::DetectorParameters det_params = make_aruco_detector_params();
  det_params.adaptiveThreshWinSizeMin = params_.adaptive_thresh_win_size_min;
  det_params.adaptiveThreshWinSizeMax = params_.adaptive_thresh_win_size_max;

  bool ok = charuco_detect_corners(
      *target_.board(), mat, K, D, charuco_corners, charuco_ids, &marker_corners,
      &marker_ids, det_params);

  // OpenCV≥4.6 新旧印刷布局：现代板失败时再试 legacy（多数下载的 PDF 是旧布局）
  cv::Ptr<cv::aruco::CharucoBoard> used_board = target_.board();
  if (!ok) {
    auto legacy = make_charuco_board(
        target_.squares_x(), target_.squares_y(),
        static_cast<float>(target_.square_length_m()),
        static_cast<float>(target_.marker_length_m()),
        make_aruco_dictionary(target_.dictionary()), true);
    std::vector<std::vector<cv::Point2f>> mc2;
    std::vector<int> mi2;
    std::vector<cv::Point2f> cc2;
    std::vector<int> ci2;
    if (run_charuco(
            *legacy, mat, K, D, &cc2, &ci2, &mc2, &mi2, det_params)) {
      ok = true;
      used_board = legacy;
      charuco_corners = std::move(cc2);
      charuco_ids = std::move(ci2);
      // 优先保留检出更多码的那次 marker 结果
      if (mc2.size() >= marker_corners.size()) {
        marker_corners = std::move(mc2);
        marker_ids = std::move(mi2);
      }
    } else if (mc2.size() > marker_corners.size()) {
      marker_corners = std::move(mc2);
      marker_ids = std::move(mi2);
    }
  }

  // 无论角点是否够，先回传 ArUco，便于 UI 提示「有码但网格参数不对」
  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
    markers->dictionary_name = target_.dictionary();
  }

  if (!ok || charuco_ids.size() < 4) {
    return out;
  }

  cv::Mat work = gray.clone();
  for (const auto &mc : marker_corners) {
    if (mc.size() < 4) {
      continue;
    }
    cv::Point2f c(0.f, 0.f);
    for (const auto &p : mc) {
      c += p;
    }
    c *= 1.f / static_cast<float>(mc.size());
    std::vector<cv::Point> poly;
    poly.reserve(mc.size());
    for (const auto &p : mc) {
      const cv::Point2f q = c + (p - c) * 0.90f;
      poly.emplace_back(cvRound(q.x), cvRound(q.y));
    }
    cv::fillConvexPoly(work, poly, 128);
  }
  refine_corners_subpix(work, &charuco_corners, 5);

  Correspondence c;
  const int n = static_cast<int>(charuco_ids.size());
  if (n < 4) {
    return out;
  }
  c.image_points.resize(n, 2);
  c.object_points.resize(n, 3);
  c.ids.resize(static_cast<size_t>(n));
  const auto obj_all = charuco_board_corners(*used_board);
  for (int i = 0; i < n; ++i) {
    const int id = charuco_ids[static_cast<size_t>(i)];
    c.image_points(i, 0) = charuco_corners[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = charuco_corners[static_cast<size_t>(i)].y;
    c.ids[static_cast<size_t>(i)] = id;
    if (id >= 0 && id < static_cast<int>(obj_all.size())) {
      c.object_points(i, 0) = obj_all[static_cast<size_t>(id)].x;
      c.object_points(i, 1) = obj_all[static_cast<size_t>(id)].y;
      c.object_points(i, 2) = obj_all[static_cast<size_t>(id)].z;
    }
  }
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
