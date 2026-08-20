#pragma once

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include <string>
#include <vector>

namespace hs_calib {
namespace core {

/// \brief 一次 detectMarkers 结果（可视化 / 调试）
struct DetectedMarkers {
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  /// \brief 几何分面结果：-1 未归类，0/1/2 = XY/XZ/YZ（与码 ID 无关）
  std::vector<int> face_ids;
  /// \brief 实际命中的字典名（单码多字典回退时填写）
  std::string dictionary_name;
  bool empty() const { return ids.empty(); }
};

/// \brief 默认 ArUco / AprilTag 检测参数（亚像素 + 宽松边界）
cv::aruco::DetectorParameters make_aruco_detector_params();

/// \brief 由字典名解析预定义 ArUco / AprilTag 字典
cv::aruco::Dictionary make_aruco_dictionary(const std::string &name);

/// \brief 检测平面标记
void aruco_detect_markers(
    const cv::Mat &image, const cv::aruco::Dictionary &dictionary,
    std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids,
    const cv::aruco::DetectorParameters &params = make_aruco_detector_params());

/// \brief 创建 GridBoard
cv::Ptr<cv::aruco::GridBoard> make_grid_board(
    int markers_x, int markers_y, float marker_length, float marker_separation,
    const cv::aruco::Dictionary &dictionary);

/// \brief 创建 CharucoBoard
/// \param legacy_pattern OpenCV≥4.6 新旧印刷布局；实物板常需 true
cv::Ptr<cv::aruco::CharucoBoard> make_charuco_board(
    int squares_x, int squares_y, float square_length, float marker_length,
    const cv::aruco::Dictionary &dictionary, bool legacy_pattern = false);

/// \brief GridBoard 物点与图像点匹配
void grid_board_match_points(
    const cv::aruco::GridBoard &board,
    const std::vector<std::vector<cv::Point2f>> &marker_corners,
    const std::vector<int> &marker_ids, cv::Mat &obj_mat, cv::Mat &img_mat);

/// \brief ChArUco 角点检测（含 marker 检测）
bool charuco_detect_corners(
    const cv::aruco::CharucoBoard &board, const cv::Mat &image,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    std::vector<cv::Point2f> &charuco_corners, std::vector<int> &charuco_ids,
    std::vector<std::vector<cv::Point2f>> *marker_corners_out = nullptr,
    std::vector<int> *marker_ids_out = nullptr,
    const cv::aruco::DetectorParameters &params = make_aruco_detector_params());

/// \brief ChArUco 角点插值（已知 marker 检测结果）
bool charuco_interpolate_from_markers(
    const cv::aruco::CharucoBoard &board, const cv::Mat &image,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    const std::vector<std::vector<cv::Point2f>> &marker_corners,
    const std::vector<int> &marker_ids, std::vector<cv::Point2f> &charuco_corners,
    std::vector<int> &charuco_ids, int min_markers = 2,
    const cv::aruco::DetectorParameters &params = make_aruco_detector_params());

/// \brief CharucoBoard 棋盘角点物坐标
std::vector<cv::Point3f> charuco_board_corners(const cv::aruco::CharucoBoard &board);

}  // namespace core
}  // namespace hs_calib
