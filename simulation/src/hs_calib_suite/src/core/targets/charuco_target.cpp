#include "hs_calib_suite/core/targets/charuco_target.hpp"

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"

namespace hs_calib {
namespace core {

/// \brief 构造 ChArUco 靶标并创建 OpenCV CharucoBoard
CharucoTarget::CharucoTarget(
    int squares_x, int squares_y, double square_length_m, double marker_length_m,
    const std::string &dictionary)
    : squares_x_(squares_x),
      squares_y_(squares_y),
      square_length_m_(square_length_m),
      marker_length_m_(marker_length_m),
      dictionary_(dictionary),
      board_(make_charuco_board(
          squares_x, squares_y, static_cast<float>(square_length_m),
          static_cast<float>(marker_length_m), make_aruco_dictionary(dictionary))) {}

/// \brief 返回靶标类型标识 "charuco"
std::string CharucoTarget::target_id() const {
  return "charuco";
}

/// \brief 从 CharucoBoard 读取角点物点，按 ID 子集或全部返回
Eigen::MatrixXd CharucoTarget::object_points(const std::vector<int> &ids) const {
  const auto corners = charuco_board_corners(*board_);
  if (ids.empty()) {
    Eigen::MatrixXd out(static_cast<int>(corners.size()), 3);
    for (size_t i = 0; i < corners.size(); ++i) {
      out(static_cast<int>(i), 0) = corners[i].x;
      out(static_cast<int>(i), 1) = corners[i].y;
      out(static_cast<int>(i), 2) = corners[i].z;
    }
    return out;
  }
  Eigen::MatrixXd out(static_cast<int>(ids.size()), 3);
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    const int id = ids[static_cast<size_t>(i)];
    if (id < 0 || id >= static_cast<int>(corners.size())) {
      return Eigen::MatrixXd(0, 3);
    }
    out.row(i) << corners[static_cast<size_t>(id)].x, corners[static_cast<size_t>(id)].y,
        corners[static_cast<size_t>(id)].z;
  }
  return out;
}

}  // namespace core
}  // namespace hs_calib
