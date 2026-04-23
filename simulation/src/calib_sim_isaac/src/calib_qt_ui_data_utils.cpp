#include "calib_sim_isaac/calib_qt_ui_data_utils.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace calib_sim_isaac
{
namespace fs = std::filesystem;

std::string TrimString(const std::string & s)
{
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1U);
}

std::vector<std::string> CollectScanRoots(const std::string & result_text_hint)
{
  std::vector<std::string> roots;
  // 统一做路径归一化与去重，避免同一目录因相对/绝对路径重复扫描。
  auto try_add = [&](const fs::path & p) {
      if (p.empty()) {
        return;
      }
      std::error_code ec;
      fs::path abs_p = p;
      if (p.is_relative()) {
        abs_p = fs::current_path() / p;
      }
      abs_p = fs::weakly_canonical(abs_p, ec);
      if (ec || !fs::exists(abs_p) || !fs::is_directory(abs_p)) {
        return;
      }
      const std::string v = abs_p.string();
      if (std::find(roots.begin(), roots.end(), v) == roots.end()) {
        roots.push_back(v);
      }
    };

  // 默认输出目录（相对当前工作目录）。
  try_add(fs::path("calib_output_isaac"));
  {
    const fs::path cwd = fs::current_path();
    if (!cwd.empty()) {
      const fs::path parent = cwd.parent_path();
      if (!parent.empty() && parent != cwd) {
        try_add(parent / "calib_output_isaac");
      }
    }
  }
  // 允许通过环境变量显式指定输出根目录。
  if (const char * env = std::getenv("CALIB_SIM_ISAAC_OUTPUT_DIR")) {
    if (env[0] != '\0') {
      try_add(fs::path(env));
    }
  }

  // 若结果文本里包含 run_dir，补充其父目录，便于刷新历史列表。
  const std::string key_dir = "calib_run_dir:";
  const std::size_t pos_dir = result_text_hint.rfind(key_dir);
  if (pos_dir != std::string::npos) {
    std::string dir = TrimString(result_text_hint.substr(pos_dir + key_dir.size()));
    if (!dir.empty()) {
      std::error_code ec;
      fs::path run_path(dir);
      if (run_path.is_relative()) {
        run_path = fs::weakly_canonical(fs::current_path() / run_path, ec);
      } else {
        run_path = fs::weakly_canonical(run_path, ec);
      }
      if (!ec && !run_path.empty() && run_path.has_parent_path()) {
        try_add(run_path.parent_path());
      }
    }
  }
  return roots;
}

std::string ParseRunDirFromResultText(const std::string & result_text)
{
  // 优先使用结果中明确给出的 run 目录。
  {
    const std::string key = "calib_run_dir:";
    const std::size_t pos = result_text.rfind(key);
    if (pos != std::string::npos) {
      const std::string dir = TrimString(result_text.substr(pos + key.size()));
      if (!dir.empty()) {
        std::error_code ec;
        fs::path p(dir);
        if (p.is_relative()) {
          p = fs::weakly_canonical(fs::current_path() / p, ec);
        } else {
          p = fs::weakly_canonical(p, ec);
        }
        if (!ec) {
          return p.string();
        }
        return dir;
      }
    }
  }

  // 其次根据 run_stamp 在各扫描根中反查目录。
  {
    const std::string key = "calib_run_stamp:";
    const std::size_t pos = result_text.rfind(key);
    if (pos != std::string::npos) {
      const std::string stamp = TrimString(result_text.substr(pos + key.size()));
      if (!stamp.empty()) {
        const auto roots = CollectScanRoots(result_text);
        for (const auto & root : roots) {
          const fs::path candidate = fs::path(root) / ("calib_run_" + stamp);
          std::error_code ec;
          if (fs::exists(candidate, ec) && !ec) {
            return fs::weakly_canonical(candidate, ec).string();
          }
        }
      }
    }
  }

  const std::string key = "result_image_samples_dir:";
  const std::size_t pos = result_text.rfind(key);
  if (pos == std::string::npos) {
    return "";
  }
  return TrimString(result_text.substr(pos + key.size()));
}

std::vector<std::string> CollectSampleImages(const std::string & run_dir, const std::string & prefix)
{
  std::vector<std::string> files;
  if (run_dir.empty()) {
    return files;
  }
  std::error_code ec;
  if (!fs::exists(run_dir, ec) || ec) {
    return files;
  }
  for (const auto & entry : fs::directory_iterator(run_dir, ec)) {
    if (ec || !entry.is_regular_file()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0 && entry.path().extension() == ".png") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::vector<std::string> ListRunDirs(const std::string & result_text_hint)
{
  std::vector<std::string> run_dirs;
  const auto scan_roots = CollectScanRoots(result_text_hint);
  for (const auto & root : scan_roots) {
    std::error_code ec;
    for (const auto & entry : fs::directory_iterator(root, ec)) {
      if (ec || !entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind("calib_run_", 0) == 0) {
        run_dirs.push_back(entry.path().string());
      }
    }
  }
  std::sort(run_dirs.begin(), run_dirs.end(), std::greater<std::string>());
  return run_dirs;
}

int RemoveAllRunDirs(const std::string & result_text_hint)
{
  int dirs_removed = 0;
  const auto scan_roots = CollectScanRoots(result_text_hint);
  for (const auto & root : scan_roots) {
    std::error_code ec;
    for (const auto & entry : fs::directory_iterator(root, ec)) {
      if (ec || !entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind("calib_run_", 0) != 0) {
        continue;
      }
      std::error_code rm_ec;
      fs::remove_all(entry.path(), rm_ec);
      if (!rm_ec) {
        ++dirs_removed;
      }
    }
  }
  return dirs_removed;
}

std::string Basename(const std::string & path_str)
{
  return fs::path(path_str).filename().string();
}

std::string FilterResultYamlForDisplay(const std::string & yaml_text)
{
  std::string out;
  // 仅过滤“路径与内部索引”字段，保留标定核心结果与质量指标。
  std::size_t start = 0;
  while (start <= yaml_text.size()) {
    const std::size_t end = yaml_text.find('\n', start);
    const std::string line = yaml_text.substr(start, end == std::string::npos ? std::string::npos : (end - start));
    const bool skip =
      line.find("sample_manifest_file") != std::string::npos ||
      line.find("result_file:") != std::string::npos ||
      line.find("result_image_samples_dir:") != std::string::npos ||
      line.find("calib_result_file:") != std::string::npos ||
      line.find("calib_run_dir:") != std::string::npos;
    if (!skip) {
      out += line;
      out.push_back('\n');
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return out;
}

}  // namespace calib_sim_isaac
