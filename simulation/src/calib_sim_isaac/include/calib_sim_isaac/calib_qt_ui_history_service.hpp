#ifndef CALIB_SIM__CALIB_QT_UI_HISTORY_SERVICE_HPP_
#define CALIB_SIM__CALIB_QT_UI_HISTORY_SERVICE_HPP_

#include <string>
#include <vector>

namespace calib_sim_isaac
{

/// 历史 run 的聚合数据：UI 只需要一次拿到所有展示所需内容。
struct RunHistoryData
{
  /// 是否成功找到并读取到标定结果 YAML。
  bool yaml_found{false};
  /// 实际读取到的 YAML 文件路径（eye_to_hand 优先，eye_in_hand 兜底）。
  std::string yaml_file;
  /// 过滤后的 YAML 文本（去除内部定位字段，适合直接显示）。
  std::string filtered_yaml_text;
  /// 结果叠加图路径列表（result_sample_*.png）。
  std::vector<std::string> result_images;
  /// 原始图路径列表（raw_sample_*.png）。
  std::vector<std::string> raw_images;
};

/// 从指定 run 目录加载历史标定数据（YAML + 样本图列表）。
RunHistoryData LoadRunHistoryData(const std::string & run_dir);

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_QT_UI_HISTORY_SERVICE_HPP_
