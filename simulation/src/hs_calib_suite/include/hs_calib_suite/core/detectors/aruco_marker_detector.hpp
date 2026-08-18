#pragma once

#include <string>
#include <vector>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/base/detector_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 任意单码 / 多码 ArUco 检测（不依赖 GridBoard 布局）
class ArucoMarkerDetector : public DetectorBase {
public:
  /// \param dictionary_name OpenCV 预定义字典名（如 DICT_6X6_1000）
  /// \param marker_length_m 单码边长（米），用于物点尺度
  ArucoMarkerDetector(
      const std::string &dictionary_name, double marker_length_m = 0.05);

  /// \brief DetectorBase 入口
  std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const override;

  /// \brief 检测任意 ArUco；fast=true 仅当前字典轻量路径（实时用）
  std::vector<Correspondence> detect(
      const ImageFrame &frame, DetectedMarkers *markers = nullptr,
      bool fast = false) const;

private:
  std::string dictionary_name_;
  double marker_length_m_ = 0.05;
};

}  // namespace core
}  // namespace hs_calib
