#include "hs_calib_suite/core/detectors/charuco_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {

/// \brief 绑定平面 ChArUco 靶标（含字典 / 板几何）
CharucoDetector::CharucoDetector(CharucoTarget target) : target_(std::move(target)) {}

/// \brief DetectorBase 入口：忽略外部 target，使用成员板
std::vector<Correspondence> CharucoDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr));
}

/// \brief 平面 ChArUco：检码 → interpolateCornersCharuco → 亚像素 → 对应点
/// \param markers 若非空，写出原始 ArUco 角点/ID（供可视化）
std::vector<Correspondence> CharucoDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty() || !target_.board()) {
    return out;
  }

  // —— ArUco 检测 ——
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  auto params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  cv::aruco::detectMarkers(
      mat, target_.board()->dictionary, marker_corners, marker_ids, params);
  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
  }
  if (marker_ids.empty()) {
    return out;
  }

  // —— 灰度 + 内参初值（插值角点用）——
  const cv::Mat gray = to_gray(mat);
  const cv::Mat K = guess_K(mat.size());
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);

  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, mat, target_.board(), charuco_corners, charuco_ids, K, D);
  if (charuco_ids.size() < 4 || charuco_corners.size() != charuco_ids.size()) {
    return out;
  }

  // 码块区域涂灰后再亚像素，减轻码边缘对棋盘角的拉扯
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
  const auto &obj_all = target_.board()->chessboardCorners;
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
