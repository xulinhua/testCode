#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 平面棋盘格靶标（内角点网格）
class ChessboardTarget : public TargetModelBase {
public:
  /// \param squares_x 内角点列数
  /// \param squares_y 内角点行数
  /// \param square_length_m 方格边长（米）
  ChessboardTarget(int squares_x, int squares_y, double square_length_m);

  /// \brief 返回靶标类型 ID "chessboard"
  std::string target_id() const override;
  /// \brief 按特征 ID 查询物点坐标
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  /// \brief 内角点列数
  int squares_x() const { return squares_x_; }
  /// \brief 内角点行数
  int squares_y() const { return squares_y_; }
  /// \brief 方格边长（米）
  double square_length_m() const { return square_length_m_; }

  /// \brief 全部内角点物点（Nx3）
  Eigen::MatrixXd all_object_points() const;

private:
  int squares_x_ = 9;
  int squares_y_ = 6;
  double square_length_m_ = 0.025;
};

}  // namespace core
}  // namespace hs_calib
