#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 圆点阵列靶标（对称 / 非对称）
enum class CircleGridPattern {
  Symmetric = 0,    ///< CALIB_CB_SYMMETRIC_GRID
  Asymmetric = 1,   ///< CALIB_CB_ASYMMETRIC_GRID
};

/// \brief 圆点阵列靶标（对称 / 非对称布局）
class CircleGridTarget : public TargetModelBase {
public:
  /// \param circles_x 列数
  /// \param circles_y 行数
  /// \param center_distance_m 相邻圆心间距（米）
  CircleGridTarget(
      int circles_x, int circles_y, double center_distance_m,
      CircleGridPattern pattern = CircleGridPattern::Symmetric);

  /// \brief 返回靶标类型 ID（对称或 asymmetric）
  std::string target_id() const override;
  /// \brief 按特征 ID 查询物点坐标
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  /// \brief 圆点列数
  int circles_x() const { return circles_x_; }
  /// \brief 圆点行数
  int circles_y() const { return circles_y_; }
  /// \brief 圆心间距（米）
  double center_distance_m() const { return center_distance_m_; }
  /// \brief 对称或非对称布局
  CircleGridPattern pattern() const { return pattern_; }

  /// \brief 全部圆心物点（Nx3）
  Eigen::MatrixXd all_object_points() const;

private:
  int circles_x_ = 9;
  int circles_y_ = 6;
  double center_distance_m_ = 0.025;
  CircleGridPattern pattern_ = CircleGridPattern::Symmetric;
};

}  // namespace core
}  // namespace hs_calib
