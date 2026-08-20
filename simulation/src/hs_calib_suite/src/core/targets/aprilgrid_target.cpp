#include "hs_calib_suite/core/targets/aprilgrid_target.hpp"

#include <array>
#include <stdexcept>

namespace hs_calib {
namespace core {

/// \brief 构造 Kalibr Aprilgrid 几何模型
AprilgridTarget::AprilgridTarget(
    int tag_cols, int tag_rows, double tag_size_m, double tag_spacing)
    : tag_cols_(tag_cols),
      tag_rows_(tag_rows),
      tag_size_m_(tag_size_m),
      tag_spacing_(tag_spacing) {}

/// \brief 返回靶标类型标识 "aprilgrid"
std::string AprilgridTarget::target_id() const {
  return "aprilgrid";
}

/// \brief 生成与 Kalibr GridCalibrationTargetAprilgrid 一致的角点网格
Eigen::MatrixXd AprilgridTarget::all_object_points() const {
  const int cols = grid_cols();
  const int rows = grid_rows();
  Eigen::MatrixXd pts(rows * cols, 3);
  const double step = (1.0 + tag_spacing_) * tag_size_m_;
  int idx = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      pts(idx, 0) = (c / 2) * step + (c % 2) * tag_size_m_;
      pts(idx, 1) = (r / 2) * step + (r % 2) * tag_size_m_;
      pts(idx, 2) = 0.0;
      ++idx;
    }
  }
  return pts;
}

/// \brief Tag ID → 四角网格索引（Kalibr baseId / pIdx）
std::array<int, 4> AprilgridTarget::corner_indices_for_tag(int tag_id) const {
  if (tag_id < 0 || tag_id >= num_tags()) {
    throw std::out_of_range("aprilgrid tag_id out of range");
  }
  const int cols = grid_cols();
  const int tags_per_row = tag_cols_;
  const unsigned base_id =
      static_cast<unsigned>(tag_id / tags_per_row) * static_cast<unsigned>(cols) * 2u +
      static_cast<unsigned>(tag_id % tags_per_row) * 2u;
  return {
      static_cast<int>(base_id),
      static_cast<int>(base_id + 1),
      static_cast<int>(base_id + cols + 1),
      static_cast<int>(base_id + cols)};
}

/// \brief 按 ID 子集返回物点
Eigen::MatrixXd AprilgridTarget::object_points(const std::vector<int> &ids) const {
  const Eigen::MatrixXd all = all_object_points();
  if (ids.empty()) {
    return all;
  }
  Eigen::MatrixXd out(static_cast<int>(ids.size()), 3);
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    const int id = ids[static_cast<size_t>(i)];
    if (id < 0 || id >= all.rows()) {
      return Eigen::MatrixXd(0, 3);
    }
    out.row(i) = all.row(id);
  }
  return out;
}

}  // namespace core
}  // namespace hs_calib
