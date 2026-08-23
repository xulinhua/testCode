#pragma once

#include <map>
#include <string>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_collector_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"

namespace hs_calib {
namespace core {

/// \brief Tier4 标定参数扩展（§8，含可视化与 OpenCV/Ceres 专有项）
struct IntrinsicsCalibrationExtras {
  int pre_calibration_num_samples = 40;

  bool plot_calibration_data_statistics = true;
  bool plot_calibration_results_statistics = true;
  int viz_pixel_cells = 16;
  double viz_tilt_resolution_deg = 15.0;
  double viz_max_tilt_deg = 45.0;
  int viz_z_cells = 12;

  bool enable_prism_model = false;
  bool fix_principal_point = false;
  bool fix_aspect_ratio = false;
  bool use_lu_decomposition = false;
  bool use_qr_decomposition = false;
};

/// \brief 棋盘格检测器参数（§9.2）
struct ChessDetectorParams {
  bool adaptive_thresh = false;
  bool normalize_image = false;
  bool fast_check = false;
  bool resized_detection = false;
  int resized_max_resolution = 1000;
  bool sub_pixel_refinement = true;
  int max_lost_frames = 3;
  int padding = 120;
};

/// \brief 圆点板检测器参数（§9.1）
struct DotDetectorParams {
  bool symmetric_grid = true;
  bool clustering = true;
  bool filter_by_area = true;
  double min_area_percentage = 0.01;
  double max_area_percentage = 1.2;
  double min_dist_between_blobs_percentage = 1.0;
  bool resized_detection = true;
  int resized_max_resolution = 2000;
};

/// \brief AprilTag 阵列检测器参数（§9.3，映射本工程 aprilgrid）
struct AprilgridDetectorParams {
  int nthreads = 8;
  int quad_decimate = 1;
  double quad_sigma = 1.5;
  bool refine_edges = true;
  double decode_sharpening = 0.25;
  bool debug = false;
  int max_hamming_error = 0;
  double min_margin = 25.0;
  double min_detection_ratio = 0.2;
};

/// \brief ChArUco 扩展检测参数（本工程特有）
struct CharucoDetectorParams {
  int adaptive_thresh_win_size_min = 3;
  int adaptive_thresh_win_size_max = 23;
  double marker_length_m = 0.04;
};

IntrinsicsProfile profile_from_config_map(
    const std::map<std::string, std::string> &config);

IntrinsicsCalibrationExtras calibration_extras_from_config(
    const std::map<std::string, std::string> &config,
    const IntrinsicsCalibrationExtras &defaults = {});

void apply_calibration_to_config(
    const IntrinsicsProfile &profile,
    const IntrinsicsCalibrationExtras &extras,
    std::map<std::string, std::string> *config);

void apply_collector_to_config(
    const IntrinsicsCollectorParams &params,
    std::map<std::string, std::string> *config);

ChessDetectorParams chess_detector_from_config(
    const std::map<std::string, std::string> &config);
void apply_chess_detector_to_config(
    const ChessDetectorParams &p, std::map<std::string, std::string> *config);

DotDetectorParams dot_detector_from_config(
    const std::map<std::string, std::string> &config);
void apply_dot_detector_to_config(
    const DotDetectorParams &p, std::map<std::string, std::string> *config);

AprilgridDetectorParams aprilgrid_detector_from_config(
    const std::map<std::string, std::string> &config);
void apply_aprilgrid_detector_to_config(
    const AprilgridDetectorParams &p, std::map<std::string, std::string> *config);

CharucoDetectorParams charuco_detector_from_config(
    const std::map<std::string, std::string> &config);
void apply_charuco_detector_to_config(
    const CharucoDetectorParams &p, std::map<std::string, std::string> *config);

std::string detector_target_from_config(
    const std::map<std::string, std::string> &config);

/// \brief 写入 Tier4 General profile 缺省键（不覆盖已有用户配置；classic 模式不调用）
void merge_tier4_intrinsics_defaults(std::map<std::string, std::string> *config);

/// \brief Profile 切换：重写标定/采集/检测三套 Tier4 默认键（保留靶标几何等会话键）
void apply_tier4_profile_bundle(
    const std::string &profile_id, std::map<std::string, std::string> *config);

/// \brief 读取 bool 配置（主键或别名任一为 true 即 true）
bool config_flag_on(
    const std::map<std::string, std::string> &config,
    const char *primary_key,
    const char *alias_key = nullptr);

}  // namespace core
}  // namespace hs_calib
