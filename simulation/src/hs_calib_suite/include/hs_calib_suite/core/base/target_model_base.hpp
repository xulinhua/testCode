#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace hs_calib {
namespace core {

/// \brief 靶标几何模型抽象基类
/// 具体实现：棋盘 / ChArUco / ArUco / 直角三面等。
class TargetModelBase {
public:
  virtual ~TargetModelBase() = default;

  /// \brief 靶标类型 ID（如 charuco / chessboard）
  virtual std::string target_id() const = 0;

  /// \brief 按特征 ID 返回三维物点（靶标坐标系，单位米）
  /// \param ids 特征索引；空表示全部角点
  virtual Eigen::MatrixXd object_points(const std::vector<int> &ids) const = 0;
};

}  // namespace core
}  // namespace hs_calib
