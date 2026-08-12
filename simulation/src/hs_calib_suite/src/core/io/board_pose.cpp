#include "hs_calib_suite/core/io/board_pose.hpp"

#include <opencv2/calib3d.hpp>

namespace hs_calib {
namespace core {

/// \brief 对应点 + 内参 → solvePnP 估计 T_cam_target
bool estimate_board_pose(
    const Correspondence &corr,
    const cv::Mat &K,
    const cv::Mat &D,
    Eigen::Matrix4d *T_cam_target,
    std::string *error_out) {
  if (T_cam_target == nullptr) {
    return false;
  }
  if (corr.image_points.rows() < 4 ||
      corr.object_points.rows() != corr.image_points.rows()) {
    if (error_out) {
      *error_out = "correspondence too few";
    }
    return false;
  }
  if (K.empty() || K.rows != 3 || K.cols != 3) {
    if (error_out) {
      *error_out = "invalid K";
    }
    return false;
  }

  // —— 组装 OpenCV 点集 ——
  std::vector<cv::Point3f> obj;
  std::vector<cv::Point2f> img;
  obj.reserve(static_cast<size_t>(corr.image_points.rows()));
  img.reserve(static_cast<size_t>(corr.image_points.rows()));
  for (int i = 0; i < corr.image_points.rows(); ++i) {
    obj.emplace_back(
        static_cast<float>(corr.object_points(i, 0)),
        static_cast<float>(corr.object_points(i, 1)),
        static_cast<float>(corr.object_points(i, 2)));
    img.emplace_back(
        static_cast<float>(corr.image_points(i, 0)),
        static_cast<float>(corr.image_points(i, 1)));
  }

  cv::Mat rvec;
  cv::Mat tvec;
  const bool ok = cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  if (!ok) {
    if (error_out) {
      *error_out = "solvePnP failed";
    }
    return false;
  }
  // —— rvec/tvec → 4×4 齐次变换 ——
  // OpenCV: rvec/tvec map object(target) → camera: p_cam = R * p_target + t
  // That is T_cam_target.
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T(r, c) = R.at<double>(r, c);
    }
    T(r, 3) = tvec.at<double>(r, 0);
  }
  *T_cam_target = T;
  return true;
}

}  // namespace core
}  // namespace hs_calib
