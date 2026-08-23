#include "hs_calib_suite/gui/intrinsics/intrinsics_param_tooltips.hpp"

#include <QHash>
#include <QWidget>

namespace hs_calib {
namespace gui {
namespace {

const QHash<QString, QString> &tooltip_table() {
  static const QHash<QString, QString> kTable = {
      // —— 标定：预设 / 求解器 ——
      {QStringLiteral("intrinsics_profile"),
       QStringLiteral("内参求解预设（General / C1 / Ceres / C2），切换会批量重置标定、采集与检测默认参数。")},
      {QStringLiteral("intrinsics_solver"),
       QStringLiteral("标定求解后端：OpenCV 经典 Brown 模型，或 Ceres 非线性优化（可配合有理畸变）。")},

      // —— RANSAC 预剔除 ——
      {QStringLiteral("use_ransac_pre_rejection"),
       QStringLiteral("标定前用 RANSAC 随机子集估计内参，剔除重投影误差过大的离群帧。")},
      {QStringLiteral("pre_rejection_iterations"),
       QStringLiteral("RANSAC 随机采样迭代次数；越大越稳但耗时增加。")},
      {QStringLiteral("pre_rejection_min_hypotheses"),
       QStringLiteral("每次 RANSAC 假设使用的最少帧数（需 ≥3）。")},
      {QStringLiteral("pre_rejection_max_rms_error"),
       QStringLiteral("RANSAC 阶段单帧重投影 RMS 上限（像素）；超过视为外点。")},

      // —— 熵子采样 ——
      {QStringLiteral("max_calibration_samples"),
       QStringLiteral("参与最终标定的训练帧数上限；超出时启用子采样。")},
      {QStringLiteral("use_entropy_maximization_subsampling"),
       QStringLiteral("用熵最大化在像面格×倾角空间选帧，保证视角覆盖均匀（否则随机子采样）。")},
      {QStringLiteral("subsampling_pixel_cells"),
       QStringLiteral("子采样时像面划分格数（如 16×16），控制中心位置多样性。")},
      {QStringLiteral("subsampling_tilt_resolution"),
       QStringLiteral("子采样倾角分箱分辨率（度）。")},
      {QStringLiteral("subsampling_max_tilt_deg"),
       QStringLiteral("子采样倾角分箱上限（度）。")},

      // —— 后剔除 ——
      {QStringLiteral("use_post_rejection"),
       QStringLiteral("标定求解后按帧 RMS 再剔除外点，并用剩余帧重新标定。")},
      {QStringLiteral("post_rejection_max_rms_error"),
       QStringLiteral("后剔除单帧重投影 RMS 阈值（像素）。")},

      // —— 统计可视化 ——
      {QStringLiteral("plot_calibration_data_statistics"),
       QStringLiteral("标定完成后自动弹出采集/各阶段 inlier 分布统计图。")},
      {QStringLiteral("plot_calibration_results_statistics"),
       QStringLiteral("标定完成后自动弹出重投影误差等结果统计图。")},
      {QStringLiteral("viz_pixel_cells"),
       QStringLiteral("标定结果可视化：像面占用格分辨率（格数）。")},
      {QStringLiteral("viz_tilt_resolution"),
       QStringLiteral("标定结果可视化：倾角分箱分辨率（度）。")},
      {QStringLiteral("viz_max_tilt_deg"),
       QStringLiteral("标定结果可视化：倾角显示上限（度）。")},
      {QStringLiteral("viz_z_cells"),
       QStringLiteral("标定结果可视化：板深度（Z）方向分箱数。")},

      // —— 畸变模型 ——
      {QStringLiteral("radial_distortion_coefficients"),
       QStringLiteral("径向畸变系数个数（k1、k2、k3 等），0~3。")},
      {QStringLiteral("rational_distortion_coefficients"),
       QStringLiteral("有理畸变系数个数（k4~k6），Ceres/C2 常用 3。")},
      {QStringLiteral("use_tangential_distortion"),
       QStringLiteral("是否优化切向畸变系数 p1、p2。")},

      // —— Ceres ——
      {QStringLiteral("pre_calibration_num_samples"),
       QStringLiteral("Ceres 全量优化前，用 OpenCV 粗标定估计初值的帧数上限。")},
      {QStringLiteral("coeffs_regularization_weight"),
       QStringLiteral("Ceres 畸变系数 L2 正则权重，抑制过拟合。")},
      {QStringLiteral("fov_regularization_weight"),
       QStringLiteral("Ceres 视场/主点异常惩罚权重（C2 可启用）。")},

      // —— OpenCV ——
      {QStringLiteral("enable_prism_model"),
       QStringLiteral("启用薄棱镜畸变模型（OpenCV CALIB_THIN_PRISM）。")},
      {QStringLiteral("fix_principal_point"),
       QStringLiteral("固定主点 cx、cy 不优化。")},
      {QStringLiteral("fix_aspect_ratio"),
       QStringLiteral("固定 fx/fy 宽高比。")},
      {QStringLiteral("use_lu_decomposition"),
       QStringLiteral("OpenCV 标定内部使用 LU 分解（高级选项）。")},
      {QStringLiteral("use_qr_decomposition"),
       QStringLiteral("OpenCV 标定内部使用 QR 分解（高级选项）。")},

      // —— 采集重投影过滤（标定参数页） ——
      {QStringLiteral("filter_by_reprojection_error"),
       QStringLiteral("采集入库时按 partial 内参模型过滤重投影误差过大的帧。")},
      {QStringLiteral("max_allowed_max_reprojection_error"),
       QStringLiteral("采集允许的单点最大重投影误差（像素）。")},
      {QStringLiteral("max_allowed_rms_reprojection_error"),
       QStringLiteral("采集允许的单帧重投影 RMS 上限（像素）。")},

      // —— 采集参数 §10 ——
      {QStringLiteral("collector_max_samples"),
       QStringLiteral("训练集与评估集各自的最大样本数上限。")},
      {QStringLiteral("decorrelate_eval_samples"),
       QStringLiteral("评估集冗余判断时，仅与训练集最近 N 帧比较，避免评估帧高度相关。")},
      {QStringLiteral("collector_max_tilt"),
       QStringLiteral("允许入库的最大板倾角（度）；超过视为检测质量差而拒绝。")},
      {QStringLiteral("filter_by_speed"),
       QStringLiteral("是否过滤运动过快/模糊的帧（在线 ROS 话题有效；离线图片自动关闭）。")},
      {QStringLiteral("collector_max_pixel_speed"),
       QStringLiteral("相邻帧靶标中心平均像素位移上限；超过则拒绝入库。")},
      {QStringLiteral("collector_max_speed"),
       QStringLiteral("归一化运动速度辅助阈值（与像素速度配合）。")},
      {QStringLiteral("filter_by_2d_redundancy"),
       QStringLiteral("新帧须在 2D 姿态（中心/skew/面积）上与库中帧有足够差异，否则视为冗余。")},
      {QStringLiteral("collector_min_center_diff"),
       QStringLiteral("2D 冗余：归一化板中心位移最小差异。")},
      {QStringLiteral("collector_min_skew_diff"),
       QStringLiteral("2D 冗余：归一化透视 skew 最小差异。")},
      {QStringLiteral("collector_min_size_diff"),
       QStringLiteral("2D 冗余：归一化板面积/距离最小差异。")},
      {QStringLiteral("filter_by_3d_redundancy"),
       QStringLiteral("启用基于 PnP 3D 位姿的冗余判断（需 partial 内参）。")},
      {QStringLiteral("collector_min_3d_center_m"),
       QStringLiteral("3D 冗余：板中心三维距离最小差异（米）。")},
      {QStringLiteral("collector_min_tilt_diff_deg"),
       QStringLiteral("3D 冗余：板倾角 x/y 分量最小差异（度）。")},
      {QStringLiteral("collector_heatmap_cells"),
       QStringLiteral("训练/评估占用率热力图的像面格分辨率（N×N）。")},
      {QStringLiteral("rotation_heatmap_angle_res"),
       QStringLiteral("旋转空间热力图的角度分箱分辨率（度）。")},
      {QStringLiteral("collector_point_2d_hist_bins"),
       QStringLiteral("采集统计图中 2D 角点分布直方图 bin 数。")},
      {QStringLiteral("collector_point_3d_hist_bins"),
       QStringLiteral("采集统计图中 3D 物点分布直方图 bin 数。")},
      {QStringLiteral("skip_frames_when_not_detection"),
       QStringLiteral("在线预览未检出靶标时跳过检测，降低 CPU 负载。")},
      {QStringLiteral("max_fast_calibration_samples"),
       QStringLiteral("采集中 partial 快速标定使用的最大帧数，用于滚动更新临时内参。")},

      // —— 棋盘格检测 ——
      {QStringLiteral("cb_adaptive"),
       QStringLiteral("OpenCV CALIB_CB_ADAPTIVE_THRESH：自适应阈值，光照不均时更稳。")},
      {QStringLiteral("cb_normalize"),
       QStringLiteral("OpenCV CALIB_CB_NORMALIZE_IMAGE：检测前直方图归一化。")},
      {QStringLiteral("cb_fast_check"),
       QStringLiteral("OpenCV CALIB_CB_FAST_CHECK：快速预判是否有棋盘，减少无效帧耗时。")},
      {QStringLiteral("cb_resized_detection"),
       QStringLiteral("大图先缩放到 max_resolution 粗检，再在 ROI 内全分辨率精检。")},
      {QStringLiteral("cb_resized_max_resolution"),
       QStringLiteral("缩放粗检时长边像素上限；仅 resized_detection 开启时生效。")},
      {QStringLiteral("cb_sub_pixel_refinement"),
       QStringLiteral("对角点做 cornerSubPix 亚像素精化；内参标定强烈建议开启。")},
      {QStringLiteral("cb_max_lost_frames"),
       QStringLiteral("ROI 跟踪：连续多少帧未检出后放弃 ROI、回全图搜索。")},
      {QStringLiteral("cb_padding"),
       QStringLiteral("ROI 在检测框四周扩展的像素边距。")},

      // —— 圆点板 ——
      {QStringLiteral("dot_symmetric_grid"),
       QStringLiteral("true=对称圆阵；false=非对称圆阵（CALIB_CB_ASYMMETRIC_GRID）。")},
      {QStringLiteral("dot_clustering"),
       QStringLiteral("启用 CALIB_CB_CLUSTERING；边缘缺失时聚类补全圆点。")},
      {QStringLiteral("dot_filter_by_area"),
       QStringLiteral("SimpleBlobDetector 按面积过滤噪点 blob。")},
      {QStringLiteral("dot_min_area_percentage"),
       QStringLiteral("最小 blob 面积 = 该百分比 × 图像面积。")},
      {QStringLiteral("dot_max_area_percentage"),
       QStringLiteral("最大 blob 面积上限（相对图像面积的百分比）。")},
      {QStringLiteral("dot_min_dist_between_blobs_percentage"),
       QStringLiteral("blob 最小间距 = 该百分比 × max(宽,高)。")},
      {QStringLiteral("dot_resized_detection"),
       QStringLiteral("大图缩放粗检 + ROI 全分辨率精检。")},
      {QStringLiteral("dot_resized_max_resolution"),
       QStringLiteral("圆点缩放粗检长边像素上限。")},

      // —— AprilTag grid ——
      {QStringLiteral("april_nthreads"),
       QStringLiteral("AprilTag 检测线程数。")},
      {QStringLiteral("april_quad_decimate"),
       QStringLiteral("四边形 decimate 因子；>1 加快检测、略损精度。")},
      {QStringLiteral("april_quad_sigma"),
       QStringLiteral("检测前高斯平滑 sigma；摩尔纹场景可略增大。")},
      {QStringLiteral("april_refine_edges"),
       QStringLiteral("是否细化 tag 边缘以提高角点精度。")},
      {QStringLiteral("april_decode_sharpening"),
       QStringLiteral("解码锐化强度；模糊图像可适当增大。")},
      {QStringLiteral("april_debug"),
       QStringLiteral("开启 AprilTag 调试输出（开发用）。")},
      {QStringLiteral("april_max_hamming_error"),
       QStringLiteral("允许的最大 Hamming 纠错位数；0 表示不允许纠错。")},
      {QStringLiteral("april_min_margin"),
       QStringLiteral("解码置信度 margin 下限；过低易误检。")},
      {QStringLiteral("april_min_detection_ratio"),
       QStringLiteral("相对网格 tag 总数的最低检出比例；低于则整帧无效。")},

      // —— ChArUco ——
      {QStringLiteral("charuco_adaptive_win_min"),
       QStringLiteral("ArUco 自适应阈值窗口最小尺寸。")},
      {QStringLiteral("charuco_adaptive_win_max"),
       QStringLiteral("ArUco 自适应阈值窗口最大尺寸。")},
      {QStringLiteral("charuco_marker_length"),
       QStringLiteral("ChArUco 码边长（米）；须小于方格边长 square_length。")},
  };
  return kTable;
}

}  // namespace

QString intrinsics_param_tooltip(const QString &param_key) {
  return tooltip_table().value(param_key.trimmed());
}

void apply_intrinsics_param_tooltip(const QString &param_key, QWidget *widget) {
  if (widget == nullptr) {
    return;
  }
  const QString tip = intrinsics_param_tooltip(param_key);
  if (!tip.isEmpty()) {
    widget->setToolTip(tip);
  }
}

}  // namespace gui
}  // namespace hs_calib
