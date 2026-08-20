#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief Kalibr 风格 Aprilgrid 靶标（AprilTag 36h11 阵列，平面 Z=0）
///
/// 参数与 Kalibr `aprilgrid.yaml` 一致：
/// - tag_cols / tag_rows：Tag 列数 / 行数
/// - tag_size：Tag 边到边尺寸 [m]
/// - tag_spacing：Tag 间空白与 tag_size 之比（非中心距比）
class AprilgridTarget : public TargetModelBase {
public:
  /// \param tag_cols X 方向 Tag 数（Kalibr tagCols）
  /// \param tag_rows Y 方向 Tag 数（Kalibr tagRows）
  /// \param tag_size_m Tag 边长 [m]
  /// \param tag_spacing Tag 间距 / tag_size（无量纲，典型 0.3）
  AprilgridTarget(
      int tag_cols, int tag_rows, double tag_size_m, double tag_spacing);

  /// \brief 返回靶标类型 ID "aprilgrid"
  std::string target_id() const override;

  /// \brief 按角点索引查询物点（检测器通常直接使用全网格）
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  int tag_cols() const { return tag_cols_; }
  int tag_rows() const { return tag_rows_; }
  double tag_size_m() const { return tag_size_m_; }
  double tag_spacing() const { return tag_spacing_; }

  /// \brief 角点网格列数（2 * tag_cols）
  int grid_cols() const { return tag_cols_ * 2; }
  /// \brief 角点网格行数（2 * tag_rows）
  int grid_rows() const { return tag_rows_ * 2; }
  /// \brief 角点总数
  int num_corners() const { return grid_cols() * grid_rows(); }
  /// \brief Tag 总数
  int num_tags() const { return tag_cols_ * tag_rows_; }

  /// \brief 生成 Kalibr 顺序的角点物点（row-major，长度 num_corners()×3）
  Eigen::MatrixXd all_object_points() const;

  /// \brief Tag ID → 四角在物点网格中的索引（Kalibr pIdx 顺序）
  std::array<int, 4> corner_indices_for_tag(int tag_id) const;

private:
  int tag_cols_ = 6;
  int tag_rows_ = 6;
  double tag_size_m_ = 0.088;
  double tag_spacing_ = 0.3;
};

}  // namespace core
}  // namespace hs_calib
