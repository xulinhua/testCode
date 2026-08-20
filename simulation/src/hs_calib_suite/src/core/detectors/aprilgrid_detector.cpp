#include "hs_calib_suite/core/detectors/aprilgrid_detector.hpp"

#include <array>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace hs_calib {
namespace core {

/// \brief 绑定 Kalibr Aprilgrid 几何
AprilgridDetector::AprilgridDetector(AprilgridTarget target)
    : target_(std::move(target)) {}

/// \brief DetectorBase 入口
std::vector<Correspondence> AprilgridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr));
}

/// \brief detectMarkers(DICT_APRILTAG_36h11) 并按 Kalibr 规则匹配物点
///
/// OpenCV AprilTag 角点顺序为 TL→TR→BR→BL，与 Kalibr pIdx 一致，勿再翻转。
std::vector<Correspondence> AprilgridDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }

  const cv::aruco::Dictionary dict = make_aruco_dictionary("DICT_APRILTAG_36h11");
  cv::aruco::DetectorParameters params = make_aruco_detector_params();
  params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  aruco_detect_markers(mat, dict, marker_corners, marker_ids, params);
  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
    markers->dictionary_name = "DICT_APRILTAG_36h11";
  }
  if (marker_ids.empty()) {
    return out;
  }

  const Eigen::MatrixXd obj_all = target_.all_object_points();
  const int max_tag_id = target_.num_tags();
  Correspondence corr;
  corr.image_points.resize(0, 2);
  corr.object_points.resize(0, 3);
  corr.ids.clear();

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    const int tag_id = marker_ids[i];
    if (tag_id < 0 || tag_id >= max_tag_id) {
      continue;
    }
    if (marker_corners[i].size() != 4) {
      continue;
    }
    std::array<int, 4> p_idx;
    try {
      p_idx = target_.corner_indices_for_tag(tag_id);
    } catch (...) {
      continue;
    }
    for (int j = 0; j < 4; ++j) {
      const int grid_idx = p_idx[static_cast<size_t>(j)];
      if (grid_idx < 0 || grid_idx >= obj_all.rows()) {
        continue;
      }
      // OpenCV 角点 j 直接对应 Kalibr pIdx[j]
      const cv::Point2f &pt = marker_corners[i][static_cast<size_t>(j)];
      const int row = static_cast<int>(corr.image_points.rows());
      corr.image_points.conservativeResize(row + 1, 2);
      corr.object_points.conservativeResize(row + 1, 3);
      corr.image_points(row, 0) = pt.x;
      corr.image_points(row, 1) = pt.y;
      corr.object_points.row(row) = obj_all.row(grid_idx);
      corr.ids.push_back(grid_idx);
    }
  }

  if (corr.image_points.rows() < 4) {
    return out;
  }
  out.push_back(std::move(corr));
  return out;
}

}  // namespace core
}  // namespace hs_calib
