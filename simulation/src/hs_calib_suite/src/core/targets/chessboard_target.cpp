#include "hs_calib_suite/core/targets/chessboard_target.hpp"

namespace hs_calib {
namespace core {

/// \brief 构造棋盘格靶标并保存网格尺寸与方格边长
ChessboardTarget::ChessboardTarget(int squares_x, int squares_y, double square_length_m)
    : squares_x_(squares_x),
      squares_y_(squares_y),
      square_length_m_(square_length_m) {}

/// \brief 返回靶标类型标识 "chessboard"
std::string ChessboardTarget::target_id() const {
  return "chessboard";
}

/// \brief 生成全部内角点物点（Z=0 平面网格）
Eigen::MatrixXd ChessboardTarget::all_object_points() const {
  const int n = squares_x_ * squares_y_;
  Eigen::MatrixXd pts(n, 3);
  int k = 0;
  // —— 按行优先遍历内角点 ——
  for (int y = 0; y < squares_y_; ++y) {
    for (int x = 0; x < squares_x_; ++x) {
      pts(k, 0) = static_cast<double>(x) * square_length_m_;
      pts(k, 1) = static_cast<double>(y) * square_length_m_;
      pts(k, 2) = 0.0;
      ++k;
    }
  }
  return pts;
}

/// \brief 按特征 ID 查询物点；空 ID 列表返回全部点
Eigen::MatrixXd ChessboardTarget::object_points(const std::vector<int> &ids) const {
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
