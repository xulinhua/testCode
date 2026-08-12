#pragma once

#include <string>
#include <vector>

#include <opencv2/aruco.hpp>

namespace hs_calib {
namespace core {

/// \brief 一次 detectMarkers 结果（可视化 / 调试）
struct DetectedMarkers {
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  /// \brief 几何分面结果：-1 未归类，0/1/2 = XY/XZ/YZ（与码 ID 无关）
  std::vector<int> face_ids;
  bool empty() const { return ids.empty(); }
};

/// \brief 由字典名解析预定义 ArUco / AprilTag 字典（缺省 DICT_4X4_50；三面 ChArUco 用 DICT_4X4_250）
cv::Ptr<cv::aruco::Dictionary> make_aruco_dictionary(const std::string &name);

}  // namespace core
}  // namespace hs_calib
