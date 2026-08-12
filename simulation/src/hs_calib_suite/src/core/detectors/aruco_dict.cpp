#include "hs_calib_suite/core/detectors/aruco_dict.hpp"

namespace hs_calib {
namespace core {

/// \brief 将字符串字典名映射为 OpenCV 预定义字典；未知名回退 DICT_4X4_50
cv::Ptr<cv::aruco::Dictionary> make_aruco_dictionary(const std::string &name) {
  int t = cv::aruco::DICT_4X4_50;
  if (name == "DICT_4X4_50") {
    t = cv::aruco::DICT_4X4_50;
  } else if (name == "DICT_4X4_100") {
    t = cv::aruco::DICT_4X4_100;
  } else if (name == "DICT_4X4_250") {
    t = cv::aruco::DICT_4X4_250;
  } else if (name == "DICT_5X5_50") {
    t = cv::aruco::DICT_5X5_50;
  } else if (name == "DICT_5X5_100") {
    t = cv::aruco::DICT_5X5_100;
  } else if (name == "DICT_5X5_250") {
    t = cv::aruco::DICT_5X5_250;
  } else if (name == "DICT_6X6_50") {
    t = cv::aruco::DICT_6X6_50;
  } else if (name == "DICT_6X6_100") {
    t = cv::aruco::DICT_6X6_100;
  } else if (name == "DICT_6X6_250") {
    t = cv::aruco::DICT_6X6_250;
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

}  // namespace core
}  // namespace hs_calib
