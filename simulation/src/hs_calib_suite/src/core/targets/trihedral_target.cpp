#include "hs_calib_suite/core/targets/trihedral_target.hpp"

#include <algorithm>
#include <cmath>

namespace hs_calib {
namespace core {

namespace {

/// \brief 生成 XY 面内角点物点（Z=0）
Eigen::MatrixXd make_face_xy(int sx, int sy, double s, double border) {
  const int n = sx * sy;
  Eigen::MatrixXd pts(n, 3);
  int k = 0;
  for (int j = 0; j < sy; ++j) {
    for (int i = 0; i < sx; ++i) {
      // 图案自 (border,border) 起铺外格；内角点在 (border+(i+1)s, ...)
      pts(k, 0) = border + static_cast<double>(i + 1) * s;
      pts(k, 1) = border + static_cast<double>(j + 1) * s;
      pts(k, 2) = 0.0;
      ++k;
    }
  }
  return pts;
}

/// \brief 生成 XZ 面内角点物点（Y=0）
Eigen::MatrixXd make_face_xz(int sx, int sy, double s, double border) {
  const int n = sx * sy;
  Eigen::MatrixXd pts(n, 3);
  int k = 0;
  for (int j = 0; j < sy; ++j) {
    for (int i = 0; i < sx; ++i) {
      pts(k, 0) = border + static_cast<double>(i + 1) * s;
      pts(k, 1) = 0.0;
      pts(k, 2) = border + static_cast<double>(j + 1) * s;
      ++k;
    }
  }
  return pts;
}

/// \brief 生成 YZ 面内角点物点（X=0）
Eigen::MatrixXd make_face_yz(int sx, int sy, double s, double border) {
  const int n = sx * sy;
  Eigen::MatrixXd pts(n, 3);
  int k = 0;
  for (int j = 0; j < sy; ++j) {
    for (int i = 0; i < sx; ++i) {
      pts(k, 0) = 0.0;
      pts(k, 1) = border + static_cast<double>(i + 1) * s;
      pts(k, 2) = border + static_cast<double>(j + 1) * s;
      ++k;
    }
  }
  return pts;
}

}  // namespace

/// \brief 构造三面靶并规范为正方形网格，随后生成三面物点
TrihedralTarget::TrihedralTarget(
    int squares_x, int squares_y, double square_length_m, double angle_deg,
    double border_m)
    : square_length_m_(square_length_m), angle_deg_(angle_deg) {
  const int n = std::max(3, std::max(squares_x, squares_y));
  squares_x_ = n;
  squares_y_ = n;
  border_m_ = (border_m >= 0.0) ? border_m : square_length_m_;
  build_faces();
}

/// \brief 为三正交面分别填充物点网格（XY / XZ / YZ）
void TrihedralTarget::build_faces() {
  faces_.clear();
  faces_.resize(3);
  (void)angle_deg_;
  const double b = border_m_;
  const double s = square_length_m_;

  faces_[0].face_id = 0;
  faces_[0].squares_x = squares_x_;
  faces_[0].squares_y = squares_y_;
  faces_[0].square_length_m = s;
  faces_[0].border_m = b;
  faces_[0].object_points = make_face_xy(squares_x_, squares_y_, s, b);

  faces_[1].face_id = 1;
  faces_[1].squares_x = squares_x_;
  faces_[1].squares_y = squares_y_;
  faces_[1].square_length_m = s;
  faces_[1].border_m = b;
  faces_[1].object_points = make_face_xz(squares_x_, squares_y_, s, b);

  faces_[2].face_id = 2;
  faces_[2].squares_x = squares_x_;
  faces_[2].squares_y = squares_y_;
  faces_[2].square_length_m = s;
  faces_[2].border_m = b;
  faces_[2].object_points = make_face_yz(squares_x_, squares_y_, s, b);
}

/// \brief 返回几何类型标识 "trihedral"（图案由配置 target 字段区分）
std::string TrihedralTarget::target_id() const {
  return "trihedral";
}

/// \brief 面内局部索引编码为全局特征 ID（面号×1000+局部索引）
int TrihedralTarget::point_id(int face_id, int local_index) {
  return face_id * 1000 + local_index;
}

/// \brief 从全局点 ID 解析面编号
int TrihedralTarget::face_id_of_point_id(int id) const {
  if (id < 0) {
    return -1;
  }
  return id / 1000;
}

/// \brief 拼接三面全部物点为单一 N×3 矩阵
Eigen::MatrixXd TrihedralTarget::all_object_points() const {
  int n = 0;
  for (const auto &f : faces_) {
    n += static_cast<int>(f.object_points.rows());
  }
  Eigen::MatrixXd out(n, 3);
  int row = 0;
  for (const auto &f : faces_) {
    out.block(row, 0, f.object_points.rows(), 3) = f.object_points;
    row += static_cast<int>(f.object_points.rows());
  }
  return out;
}

/// \brief 按全局特征 ID 查询物点；空 ID 返回全部点
Eigen::MatrixXd TrihedralTarget::object_points(const std::vector<int> &ids) const {
  if (ids.empty()) {
    return all_object_points();
  }
  Eigen::MatrixXd out(static_cast<int>(ids.size()), 3);
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    const int id = ids[static_cast<size_t>(i)];
    const int face = id / 1000;
    const int local = id % 1000;
    if (face < 0 || face >= static_cast<int>(faces_.size()) || local < 0 ||
        local >= faces_[static_cast<size_t>(face)].object_points.rows()) {
      out.row(i).setZero();
    } else {
      out.row(i) = faces_[static_cast<size_t>(face)].object_points.row(local);
    }
  }
  return out;
}

}  // namespace core
}  // namespace hs_calib
