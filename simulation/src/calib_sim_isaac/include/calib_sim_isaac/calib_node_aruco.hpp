#ifndef CALIB_SIM__CALIB_NODE_ARUCO_HPP_
#define CALIB_SIM__CALIB_NODE_ARUCO_HPP_

#include <opencv2/core.hpp>

namespace calib_sim_isaac
{

/// 将角点重投影误差从像素近似换算为目标深度处的毫米误差。
double EstimateReprojMmAtTargetDepth(
  double reproj_px, const cv::Mat & t_target_to_cam, const cv::Mat & camera_matrix);

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_NODE_ARUCO_HPP_
