#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 圆点阵列靶标（对称 / 非对称）
enum class CircleGridPattern {
  Symmetric = 0,   ///< CALIB_CB_SYMMETRIC_GRID
  Asymmetric = 1,  ///< CALIB_CB_ASYMMETRIC_GRID
};

/// \brief 圆点阵列几何
///
/// 物点只用圆心，不依赖直径。直径仅用于检测时 Blob 面积约束。
///
/// 非对称（OpenCV 官方 calcBoardCornerPositions）：
///   X = (2*j + i%2) * squareSize
///   Y = i * squareSize
/// 其中 squareSize = 相邻行圆心距 = 同行相邻圆心距 / 2。
/// 若把「同行间距」误当 squareSize，物点会放大约 2×，RMS 可到几十像素。
class CircleGridTarget : public TargetModelBase {
public:
  /// \param circles_x 每行圆数（patternSize.width）
  /// \param circles_y 行数（patternSize.height）
  /// \param center_distance_m 对称：相邻圆心距；非对称：OpenCV squareSize
  ///        （= calib.io Diagonal Spacing / √2；行距；同行距的一半）
  /// \param circle_diameter_m 圆直径（米），≤0 表示未知，仅用默认 Blob 阈值
  CircleGridTarget(
      int circles_x, int circles_y, double center_distance_m,
      CircleGridPattern pattern = CircleGridPattern::Symmetric,
      double circle_diameter_m = 0.0);

  std::string target_id() const override;
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  int circles_x() const { return circles_x_; }
  int circles_y() const { return circles_y_; }
  double center_distance_m() const { return center_distance_m_; }
  double circle_diameter_m() const { return circle_diameter_m_; }
  CircleGridPattern pattern() const { return pattern_; }

  Eigen::MatrixXd all_object_points() const;

private:
  int circles_x_ = 9;
  int circles_y_ = 6;
  double center_distance_m_ = 0.025;
  double circle_diameter_m_ = 0.0;
  CircleGridPattern pattern_ = CircleGridPattern::Symmetric;
};

}  // namespace core
}  // namespace hs_calib
