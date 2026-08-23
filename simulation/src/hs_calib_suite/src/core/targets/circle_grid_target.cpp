#include "hs_calib_suite/core/targets/circle_grid_target.hpp"

namespace hs_calib {
namespace core {

CircleGridTarget::CircleGridTarget(
    int circles_x, int circles_y, double center_distance_m, CircleGridPattern pattern,
    double circle_diameter_m)
    : circles_x_(circles_x),
      circles_y_(circles_y),
      center_distance_m_(center_distance_m),
      circle_diameter_m_(circle_diameter_m),
      pattern_(pattern) {}

std::string CircleGridTarget::target_id() const {
  return pattern_ == CircleGridPattern::Asymmetric ? "circles_asymmetric"
                                                   : "circles_symmetric";
}

Eigen::MatrixXd CircleGridTarget::all_object_points() const {
  const int n = circles_x_ * circles_y_;
  Eigen::MatrixXd pts(n, 3);
  const double s = center_distance_m_;
  int k = 0;
  // 与 OpenCV tutorial calcBoardCornerPositions 逐点一致：行 i、列 j
  for (int i = 0; i < circles_y_; ++i) {
    for (int j = 0; j < circles_x_; ++j) {
      if (pattern_ == CircleGridPattern::Asymmetric) {
        pts(k, 0) = static_cast<double>(2 * j + (i % 2)) * s;
        pts(k, 1) = static_cast<double>(i) * s;
      } else {
        pts(k, 0) = static_cast<double>(j) * s;
        pts(k, 1) = static_cast<double>(i) * s;
      }
      pts(k, 2) = 0.0;
      ++k;
    }
  }
  return pts;
}

Eigen::MatrixXd CircleGridTarget::object_points(const std::vector<int> &ids) const {
  const Eigen::MatrixXd all = all_object_points();
  if (ids.empty()) {
    return all;
  }
  Eigen::MatrixXd out(static_cast<int>(ids.size()), 3);
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    const int id = ids[static_cast<size_t>(i)];
    if (id < 0 || id >= all.rows()) {
      out.row(i).setZero();
    } else {
      out.row(i) = all.row(id);
    }
  }
  return out;
}

}  // namespace core
}  // namespace hs_calib
