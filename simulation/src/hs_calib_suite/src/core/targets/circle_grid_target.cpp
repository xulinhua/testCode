#include "hs_calib_suite/core/targets/circle_grid_target.hpp"

namespace hs_calib {
namespace core {

/// \brief 构造圆点阵列靶标（对称或非对称布局）
CircleGridTarget::CircleGridTarget(
    int circles_x, int circles_y, double center_distance_m, CircleGridPattern pattern)
    : circles_x_(circles_x),
      circles_y_(circles_y),
      center_distance_m_(center_distance_m),
      pattern_(pattern) {}

/// \brief 返回靶标类型 ID（对称 / 非对称）
std::string CircleGridTarget::target_id() const {
  return pattern_ == CircleGridPattern::Asymmetric ? "circles_asymmetric"
                                                   : "circles_symmetric";
}

/// \brief 生成全部圆心物点（对称或 OpenCV 非对称交错布局）
Eigen::MatrixXd CircleGridTarget::all_object_points() const {
  const int n = circles_x_ * circles_y_;
  Eigen::MatrixXd pts(n, 3);
  int k = 0;
  for (int j = 0; j < circles_y_; ++j) {
    for (int i = 0; i < circles_x_; ++i) {
      if (pattern_ == CircleGridPattern::Asymmetric) {
        // OpenCV 非对称圆点：交错行偏移半格
        pts(k, 0) = static_cast<double>(2 * i + (j % 2)) * center_distance_m_;
        pts(k, 1) = static_cast<double>(j) * center_distance_m_;
      } else {
        pts(k, 0) = static_cast<double>(i) * center_distance_m_;
        pts(k, 1) = static_cast<double>(j) * center_distance_m_;
      }
      pts(k, 2) = 0.0;
      ++k;
    }
  }
  return pts;
}

/// \brief 按特征 ID 查询物点；空 ID 列表返回全部点
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
