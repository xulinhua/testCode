#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 平面 ArUco / AprilTag 阵列靶标（GridBoard）
class ArucoGridTarget : public TargetModelBase {
public:
  /// \param dictionary ArUco 字典名称
  ArucoGridTarget(
      int markers_x, int markers_y, double marker_length_m, double marker_separation_m,
      const std::string &dictionary = "DICT_4X4_50");

  /// \brief 返回靶标类型 ID "aruco_grid"
  std::string target_id() const override;
  /// \brief 按 ID 查询物点（标定由检测器 match 提供）
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  /// \brief 标记列数
  int markers_x() const { return markers_x_; }
  /// \brief 标记行数
  int markers_y() const { return markers_y_; }
  /// \brief 标记边长（米）
  double marker_length_m() const { return marker_length_m_; }
  /// \brief 标记间距（米）
  double marker_separation_m() const { return marker_separation_m_; }
  /// \brief 字典名称
  const std::string &dictionary() const { return dictionary_name_; }
  /// \brief OpenCV GridBoard 实例
  cv::Ptr<cv::aruco::GridBoard> board() const { return board_; }
  /// \brief OpenCV 字典
  const cv::aruco::Dictionary &dictionary_ref() const { return dictionary_; }

private:
  int markers_x_ = 5;
  int markers_y_ = 7;
  double marker_length_m_ = 0.04;
  double marker_separation_m_ = 0.01;
  std::string dictionary_name_ = "DICT_4X4_50";
  cv::aruco::Dictionary dictionary_;
  cv::Ptr<cv::aruco::GridBoard> board_;
};

}  // namespace core
}  // namespace hs_calib
