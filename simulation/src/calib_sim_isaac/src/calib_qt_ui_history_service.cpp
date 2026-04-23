#include "calib_sim_isaac/calib_qt_ui_history_service.hpp"

#include "calib_sim_isaac/calib_qt_ui_data_utils.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace calib_sim_isaac
{
namespace fs = std::filesystem;

namespace
{
std::string TryReadTextFile(const std::string & path)
{
  // 统一封装文本读取，便于后续补充编码/异常处理策略。
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return "";
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}
}  // namespace

RunHistoryData LoadRunHistoryData(const std::string & run_dir)
{
  RunHistoryData out;
  if (run_dir.empty()) {
    return out;
  }

  // 先尝试眼在手外结果文件；不存在则回退眼在手上结果文件。
  std::string yaml_file = run_dir + "/calib_result_eye_to_hand.yaml";
  if (!fs::exists(yaml_file)) {
    const std::string fallback = run_dir + "/calib_result_eye_in_hand.yaml";
    if (fs::exists(fallback)) {
      yaml_file = fallback;
    }
  }

  if (fs::exists(yaml_file)) {
    const std::string yaml_text = TryReadTextFile(yaml_file);
    if (!yaml_text.empty()) {
      out.yaml_found = true;
      out.yaml_file = yaml_file;
      out.filtered_yaml_text = FilterResultYamlForDisplay(yaml_text);
    }
  }

  // 样本图统一在服务层收集，UI 侧只负责显示。
  out.result_images = CollectSampleImages(run_dir, "result_sample_");
  out.raw_images = CollectSampleImages(run_dir, "raw_sample_");
  return out;
}

}  // namespace calib_sim_isaac
