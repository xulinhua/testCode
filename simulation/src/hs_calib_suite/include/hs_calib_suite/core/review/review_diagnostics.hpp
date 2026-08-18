#pragma once

#include <string>
#include <vector>

#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 单帧重投影诊断
struct ViewResidual {
  int index = -1;           ///< 观测在 batch 中的下标
  std::string label;        ///< 列表显示名
  std::string side;         ///< left / right / 空
  double rms_px = -1.0;     ///< 该帧 RMS（像素）；失败为 -1
  int num_points = 0;       ///< 参与点数
  bool ok = false;          ///< PnP+投影是否成功
};

/// \brief 单点残差（覆盖/重投影图用）
struct ResidualPoint {
  float u = 0.f;            ///< 观测像素 x
  float v = 0.f;            ///< 观测像素 y
  float err_px = 0.f;       ///< |观测-重投影|
  int view_index = -1;      ///< 所属观测下标
};

/// \brief 复核诊断汇总
struct ReviewDiagnostics {
  bool valid = false;
  std::string message;
  int image_width = 0;
  int image_height = 0;
  double global_rms_px = -1.0;
  std::vector<ViewResidual> views;
  std::vector<ResidualPoint> points;
};

/// \brief 从结果 + 观测批量计算逐帧/逐点重投影残差（内参类标定）
/// \param meta_prefix 立体时用 "left_" / "right_"；单目传空
ReviewDiagnostics compute_review_diagnostics(
    const CalibrationResult &result,
    const ObservationBatch &batch,
    const std::string &meta_prefix = "");

/// \brief 立体：合并左右诊断（左右各自用前缀内参）
ReviewDiagnostics compute_stereo_review_diagnostics(
    const CalibrationResult &result, const ObservationBatch &batch);

}  // namespace core
}  // namespace hs_calib
