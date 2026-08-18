#include "hs_calib_suite/core/detectors/aruco_grid_detector.hpp"

#include <opencv2/aruco.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace hs_calib {
namespace core {

/// \brief 绑定 ArUco 网格板几何与字典
ArucoGridDetector::ArucoGridDetector(ArucoGridTarget target)
    : target_(std::move(target)) {}

/// \brief DetectorBase 入口
std::vector<Correspondence> ArucoGridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr));
}

/// \brief ArUco 阵列：detectMarkers → getBoardObjectAndImagePoints → 对应点
/// \param markers 若非空，写出检出码角点/ID
std::vector<Correspondence> ArucoGridDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty() || !target_.board() || !target_.dictionary_ptr()) {
    return out;
  }

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  auto params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params->minMarkerPerimeterRate = 0.01;
  params->maxErroneousBitsInBorderRate = 0.4;
  cv::aruco::detectMarkers(mat, target_.dictionary_ptr(), marker_corners, marker_ids, params);
  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
  }
  if (marker_ids.empty()) {
    return out;
  }

  // 板物点与图像点对齐（每码四角）
  cv::Mat obj_mat, img_mat;
  cv::aruco::getBoardObjectAndImagePoints(
      target_.board(), marker_corners, marker_ids, obj_mat, img_mat);
  if (obj_mat.empty() || img_mat.empty() || obj_mat.rows != img_mat.rows || obj_mat.rows < 4) {
    return out;
  }
  cv::Mat obj_f, img_f;
  obj_mat.convertTo(obj_f, CV_32F);
  img_mat.convertTo(img_f, CV_32F);

  Correspondence c;
  const int n = obj_f.rows;
  c.image_points.resize(n, 2);
  c.object_points.resize(n, 3);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = img_f.at<float>(i, 0);
    c.image_points(i, 1) = img_f.at<float>(i, 1);
    c.object_points(i, 0) = obj_f.at<float>(i, 0);
    c.object_points(i, 1) = obj_f.at<float>(i, 1);
    c.object_points(i, 2) = obj_f.at<float>(i, 2);
    c.ids[static_cast<size_t>(i)] = i;
  }
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
