#ifndef CALIB_SIM__CALIB_QT_UI_DATA_UTILS_HPP_
#define CALIB_SIM__CALIB_QT_UI_DATA_UTILS_HPP_

#include <string>
#include <vector>

namespace calib_sim_isaac
{

/// 去掉字符串首尾空白字符（空串安全）。
std::string TrimString(const std::string & s);
/// 根据当前结果文本推断可扫描的历史目录根（含环境变量与相对路径兜底）。
std::vector<std::string> CollectScanRoots(const std::string & result_text_hint);
/// 从 result_text 中解析本次 run 目录；优先 calib_run_dir，其次 run_stamp 推导。
std::string ParseRunDirFromResultText(const std::string & result_text);
/// 在 run 目录中收集样本图（按前缀 + .png 过滤并排序）。
std::vector<std::string> CollectSampleImages(const std::string & run_dir, const std::string & prefix);
/// 列出所有历史 run 目录（calib_run_*，按时间倒序）。
std::vector<std::string> ListRunDirs(const std::string & result_text_hint);
/// 删除可扫描根目录下全部 calib_run_* 子目录，返回删除成功数量。
int RemoveAllRunDirs(const std::string & result_text_hint);
/// 获取路径最后一级文件/目录名（跨平台路径分隔符兼容）。
std::string Basename(const std::string & path_str);
/// 过滤结果 YAML 中仅用于内部定位/文件跳转的字段，便于 UI 展示核心内容。
std::string FilterResultYamlForDisplay(const std::string & yaml_text);

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_QT_UI_DATA_UTILS_HPP_
