#include "hs_calib_suite/core/targets/aruco_grid_target.hpp"

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"

namespace hs_calib {
namespace core {

/// \brief 构造 ArUco 阵列靶标并创建 OpenCV GridBoard
ArucoGridTarget::ArucoGridTarget(
    int markers_x, int markers_y, double marker_length_m, double marker_separation_m,
    const std::string &dictionary)
    : markers_x_(markers_x),
      markers_y_(markers_y),
      marker_length_m_(marker_length_m),
      marker_separation_m_(marker_separation_m),
      dictionary_(dictionary),
      dictionary_ptr_(make_aruco_dictionary(dictionary)),
      board_(cv::aruco::GridBoard::create(
          markers_x, markers_y, static_cast<float>(marker_length_m),
          static_cast<float>(marker_separation_m), dictionary_ptr_)) {}

/// \brief 返回靶标类型标识 "aruco_grid"
std::string ArucoGridTarget::target_id() const {
  return "aruco_grid";
}

/// \brief 按 ID 查询物点（标定路径由检测器 match 提供，此处返回空）
Eigen::MatrixXd ArucoGridTarget::object_points(const std::vector<int> &ids) const {
  (void)ids;
  // 标定路径使用检测器 match 结果；此处返回空表示「按 ID 查询未实现完整展平」
  return Eigen::MatrixXd(0, 3);
}

}  // namespace core
}  // namespace hs_calib
