#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/aruco/charuco.hpp>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 平面 ChArUco 靶标（方格数 × 方格边长 + 标记边长）
///
/// \note squares_x/y 为 OpenCV CharucoBoard 的棋盘方格数（非内角点数）。
class CharucoTarget : public TargetModelBase {
public:
  CharucoTarget(
      int squares_x, int squares_y, double square_length_m, double marker_length_m,
      const std::string &dictionary = "DICT_4X4_50");

  /// \brief 返回靶标类型 ID "charuco"
  std::string target_id() const override;
  /// \brief 按特征 ID 查询 ChArUco 角点物点
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  /// \brief 棋盘方格列数
  int squares_x() const { return squares_x_; }
  /// \brief 棋盘方格行数
  int squares_y() const { return squares_y_; }
  /// \brief 方格边长（米）
  double square_length_m() const { return square_length_m_; }
  /// \brief 标记边长（米）
  double marker_length_m() const { return marker_length_m_; }
  /// \brief 字典名称
  const std::string &dictionary() const { return dictionary_; }
  /// \brief OpenCV CharucoBoard 实例
  cv::Ptr<cv::aruco::CharucoBoard> board() const { return board_; }

private:
  int squares_x_ = 5;
  int squares_y_ = 7;
  double square_length_m_ = 0.04;
  double marker_length_m_ = 0.03;
  std::string dictionary_ = "DICT_4X4_50";
  cv::Ptr<cv::aruco::CharucoBoard> board_;
};

}  // namespace core
}  // namespace hs_calib
