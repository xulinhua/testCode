#include "hs_calib_suite/core/detectors/aruco_dict.hpp"

#include <algorithm>

namespace hs_calib {
namespace core {

/// \brief 默认检测参数
cv::aruco::DetectorParameters make_aruco_detector_params() {
  cv::aruco::DetectorParameters params;
  params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params.minMarkerPerimeterRate = 0.01;
  params.maxErroneousBitsInBorderRate = 0.4;
  return params;
}

/// \brief 将字符串字典名映射为 OpenCV 预定义字典
cv::aruco::Dictionary make_aruco_dictionary(const std::string &name) {
  int t = cv::aruco::DICT_4X4_50;
  if (name == "DICT_4X4_50") {
    t = cv::aruco::DICT_4X4_50;
  } else if (name == "DICT_4X4_100") {
    t = cv::aruco::DICT_4X4_100;
  } else if (name == "DICT_4X4_250") {
    t = cv::aruco::DICT_4X4_250;
  } else if (name == "DICT_4X4_1000") {
    t = cv::aruco::DICT_4X4_1000;
  } else if (name == "DICT_5X5_50") {
    t = cv::aruco::DICT_5X5_50;
  } else if (name == "DICT_5X5_100") {
    t = cv::aruco::DICT_5X5_100;
  } else if (name == "DICT_5X5_250") {
    t = cv::aruco::DICT_5X5_250;
  } else if (name == "DICT_5X5_1000") {
    t = cv::aruco::DICT_5X5_1000;
  } else if (name == "DICT_6X6_50") {
    t = cv::aruco::DICT_6X6_50;
  } else if (name == "DICT_6X6_100") {
    t = cv::aruco::DICT_6X6_100;
  } else if (name == "DICT_6X6_250") {
    t = cv::aruco::DICT_6X6_250;
  } else if (name == "DICT_6X6_1000") {
    t = cv::aruco::DICT_6X6_1000;
  } else if (name == "DICT_7X7_50") {
    t = cv::aruco::DICT_7X7_50;
  } else if (name == "DICT_7X7_100") {
    t = cv::aruco::DICT_7X7_100;
  } else if (name == "DICT_7X7_1000") {
    t = cv::aruco::DICT_7X7_1000;
  } else if (name == "DICT_APRILTAG_16h5") {
    t = cv::aruco::DICT_APRILTAG_16h5;
  } else if (name == "DICT_APRILTAG_25h9") {
    t = cv::aruco::DICT_APRILTAG_25h9;
  } else if (name == "DICT_APRILTAG_36h10") {
    t = cv::aruco::DICT_APRILTAG_36h10;
  } else if (name == "DICT_APRILTAG_36h11") {
    t = cv::aruco::DICT_APRILTAG_36h11;
  } else if (name == "DICT_ARUCO_ORIGINAL") {
    t = cv::aruco::DICT_ARUCO_ORIGINAL;
  }
  return cv::aruco::getPredefinedDictionary(t);
}

/// \brief 检测平面标记
void aruco_detect_markers(
    const cv::Mat &image, const cv::aruco::Dictionary &dictionary,
    std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids,
    const cv::aruco::DetectorParameters &params) {
  cv::aruco::ArucoDetector detector(dictionary, params);
  detector.detectMarkers(image, corners, ids);
}

/// \brief 创建 GridBoard
cv::Ptr<cv::aruco::GridBoard> make_grid_board(
    int markers_x, int markers_y, float marker_length, float marker_separation,
    const cv::aruco::Dictionary &dictionary) {
  return cv::makePtr<cv::aruco::GridBoard>(
      cv::Size(markers_x, markers_y), marker_length, marker_separation, dictionary);
}

/// \brief 创建 CharucoBoard（自动保证 marker < square）
cv::Ptr<cv::aruco::CharucoBoard> make_charuco_board(
    int squares_x, int squares_y, float square_length, float marker_length,
    const cv::aruco::Dictionary &dictionary, bool legacy_pattern) {
  float sq = std::max(1e-4f, square_length);
  float mk = marker_length;
  if (mk <= 0.f || mk >= sq) {
    mk = sq * 0.75f;
  }
  auto board = cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(std::max(2, squares_x), std::max(2, squares_y)), sq, mk, dictionary);
  board->setLegacyPattern(legacy_pattern);
  return board;
}

/// \brief GridBoard 物点与图像点匹配
void grid_board_match_points(
    const cv::aruco::GridBoard &board,
    const std::vector<std::vector<cv::Point2f>> &marker_corners,
    const std::vector<int> &marker_ids, cv::Mat &obj_mat, cv::Mat &img_mat) {
  board.matchImagePoints(marker_corners, marker_ids, obj_mat, img_mat);
}

/// \brief ChArUco 角点检测
bool charuco_detect_corners(
    const cv::aruco::CharucoBoard &board, const cv::Mat &image,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    std::vector<cv::Point2f> &charuco_corners, std::vector<int> &charuco_ids,
    std::vector<std::vector<cv::Point2f>> *marker_corners_out,
    std::vector<int> *marker_ids_out, const cv::aruco::DetectorParameters &params) {
  cv::aruco::CharucoParameters cparams;
  cparams.cameraMatrix = camera_matrix;
  cparams.distCoeffs = dist_coeffs;
  cparams.minMarkers = 1;  // 局部遮挡时仍尽量插值角点
  cv::aruco::CharucoDetector detector(board, cparams, params);
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  detector.detectBoard(image, charuco_corners, charuco_ids, marker_corners, marker_ids);
  if (marker_corners_out) {
    *marker_corners_out = marker_corners;
  }
  if (marker_ids_out) {
    *marker_ids_out = marker_ids;
  }
  return charuco_ids.size() >= 4 &&
         charuco_corners.size() == charuco_ids.size();
}

/// \brief ChArUco 角点插值（已知 marker 检测结果）
bool charuco_interpolate_from_markers(
    const cv::aruco::CharucoBoard &board, const cv::Mat &image,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    const std::vector<std::vector<cv::Point2f>> &marker_corners,
    const std::vector<int> &marker_ids, std::vector<cv::Point2f> &charuco_corners,
    std::vector<int> &charuco_ids, int min_markers,
    const cv::aruco::DetectorParameters &params) {
  cv::aruco::CharucoParameters cparams;
  cparams.cameraMatrix = camera_matrix;
  cparams.distCoeffs = dist_coeffs;
  cparams.minMarkers = std::max(1, min_markers);
  cv::aruco::CharucoDetector detector(board, cparams, params);
  std::vector<std::vector<cv::Point2f>> mc = marker_corners;
  std::vector<int> mi = marker_ids;
  detector.detectBoard(image, charuco_corners, charuco_ids, mc, mi);
  return charuco_ids.size() >= 4 &&
         charuco_corners.size() == charuco_ids.size();
}

/// \brief CharucoBoard 棋盘角点物坐标
std::vector<cv::Point3f> charuco_board_corners(const cv::aruco::CharucoBoard &board) {
  return board.getChessboardCorners();
}

}  // namespace core
}  // namespace hs_calib
