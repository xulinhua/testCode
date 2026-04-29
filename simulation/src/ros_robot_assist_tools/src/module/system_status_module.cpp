#include "ros_robot_assist_tools/module/system_status_module.h"

#include <cstdio>
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <QFileInfo>
#include <QProcess>
#include <QRegExp>
#include <QStringList>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <yaml-cpp/yaml.h>

namespace ros_robot_assist_tools::ui
{
namespace
{
QString ShellQuote(const QString & raw) { QString escaped = raw; escaped.replace("'", "'\\''"); return "'" + escaped + "'"; }

class TopicRateSampler
{
public:
  TopicRateSampler()
  : last_tp_(std::chrono::steady_clock::now())
  {
    if (!rclcpp::ok()) {
      return;
    }
    node_ = rclcpp::Node::make_shared("ros_robot_assist_tools_topic_rate_sampler");
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      while (!stop_.load()) {
        exec_->spin_some(std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  ~TopicRateSampler()
  {
    stop_.store(true);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (exec_ && node_) {
      exec_->remove_node(node_);
    }
  }

  std::vector<TopicTypeRow> Snapshot()
  {
    std::vector<TopicTypeRow> rows;
    if (!node_) {
      return rows;
    }
    const auto names_and_types = node_->get_topic_names_and_types();
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto & kv : names_and_types) {
      const std::string & topic = kv.first;
      if (topic.empty() || topic.rfind("/_ros2cli", 0) == 0) {
        continue;
      }
      const std::string type = kv.second.empty() ? std::string{} : kv.second.front();
      TopicTypeRow row;
      row.topic = QString::fromStdString(topic);
      row.type = type.empty() ? "-" : QString::fromStdString(type);
      rows.push_back(row);
      if (subs_.find(topic) == subs_.end() && !type.empty()) {
        try {
          auto sub = node_->create_generic_subscription(
            topic, type, rclcpp::SensorDataQoS(),
            [this, topic](std::shared_ptr<rclcpp::SerializedMessage>) {
              std::lock_guard<std::mutex> g(mu_);
              counts_[topic] += 1ULL;
            });
          subs_[topic] = sub;
          counts_.emplace(topic, 0ULL);
          last_counts_.emplace(topic, 0ULL);
        } catch (...) {
        }
      }
    }
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_tp_).count();
    for (auto & row : rows) {
      const std::string key = row.topic.toStdString();
      const auto it_cur = counts_.find(key);
      const auto it_prev = last_counts_.find(key);
      if (it_cur == counts_.end() || it_prev == last_counts_.end() || dt <= 0.0001) {
        row.hz = "-";
        continue;
      }
      const uint64_t cur = it_cur->second;
      const uint64_t prev = it_prev->second;
      const double hz = static_cast<double>(cur - prev) / dt;
      row.hz = QString("%1 Hz").arg(hz, 0, 'f', 2);
      it_prev->second = cur;
    }
    last_tp_ = now;
    return rows;
  }

private:
  std::mutex mu_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::atomic<bool> stop_{false};
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;
  std::unordered_map<std::string, uint64_t> counts_;
  std::unordered_map<std::string, uint64_t> last_counts_;
  std::chrono::steady_clock::time_point last_tp_;
};
}

bool ReadCpuCounters(unsigned long long & idle_all, unsigned long long & total_all)
{
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) return false;
  std::string cpu_label;
  unsigned long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
  stat_file >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  if (cpu_label != "cpu") return false;
  idle_all = idle + iowait;
  total_all = idle_all + user + nice + system + irq + softirq + steal;
  return true;
}

bool ReadNetworkBytes(unsigned long long & rx_bytes, unsigned long long & tx_bytes)
{
  std::ifstream net_file("/proc/net/dev");
  if (!net_file.is_open()) return false;
  rx_bytes = 0; tx_bytes = 0; std::string line;
  while (std::getline(net_file, line)) {
    if (line.find(':') == std::string::npos) continue;
    std::istringstream iss(line); std::string iface; iss >> iface;
    if (iface.find(':') != std::string::npos) iface = iface.substr(0, iface.size() - 1);
    if (iface == "lo") continue;
    unsigned long long if_rx = 0, if_tx = 0, skip = 0;
    iss >> if_rx; for (int i = 0; i < 7; ++i) { iss >> skip; } iss >> if_tx;
    rx_bytes += if_rx; tx_bytes += if_tx;
  }
  return true;
}

bool ReadDiskBytes(unsigned long long & read_bytes, unsigned long long & write_bytes)
{
  std::ifstream disk_file("/proc/diskstats");
  if (!disk_file.is_open()) return false;
  read_bytes = 0; write_bytes = 0; std::string line;
  while (std::getline(disk_file, line)) {
    std::istringstream iss(line);
    int major = 0, minor = 0; std::string name;
    unsigned long long rc = 0, rm = 0, sr = 0, mr = 0, wc = 0, wm = 0, sw = 0, mw = 0;
    if (!(iss >> major >> minor >> name >> rc >> rm >> sr >> mr >> wc >> wm >> sw >> mw)) continue;
    if (name.rfind("loop", 0) == 0 || name.rfind("ram", 0) == 0) continue;
    read_bytes += sr * 512ULL; write_bytes += sw * 512ULL;
  }
  return true;
}

std::vector<ProcessRow> ReadTopProcessRows()
{
  std::vector<ProcessRow> rows;
  const long cpu_cores = sysconf(_SC_NPROCESSORS_ONLN);
  const double core_count = (cpu_cores > 0) ? static_cast<double>(cpu_cores) : 1.0;
  FILE * proc_pipe = popen("ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -n 11", "r");
  if (proc_pipe == nullptr) return rows;
  char line[256] = {0}; bool first_line = true;
  while (fgets(line, sizeof(line), proc_pipe) != nullptr) {
    QString row = QString::fromUtf8(line).trimmed();
    if (row.isEmpty()) continue;
    if (first_line) { first_line = false; continue; }
    QStringList cols = row.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (cols.size() < 4) continue;
    ProcessRow pr;
    pr.pid = cols[0];
    const double raw_cpu = cols[1].toDouble();
    const double normalized_cpu = raw_cpu / core_count;
    pr.cpu = QString::number(normalized_cpu, 'f', 1);
    pr.mem = cols[2];
    pr.command = cols.mid(3).join(" ");
    rows.push_back(pr);
  }
  pclose(proc_pipe);
  return rows;
}

std::vector<QString> ReadRos2NodeRows()
{
  std::vector<QString> rows;
  FILE * ros_pipe = popen("bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 node list 2>/dev/null'", "r");
  if (ros_pipe == nullptr) return rows;
  char line[256] = {0};
  while (fgets(line, sizeof(line), ros_pipe) != nullptr) {
    const QString node = QString::fromUtf8(line).trimmed();
    if (!node.isEmpty()) rows.push_back(node);
  }
  pclose(ros_pipe);
  return rows;
}

std::vector<NodeInfoRow> ReadRos2NodeInfoRows()
{
  const std::vector<QString> nodes = ReadRos2NodeRows();
  std::vector<NodeInfoRow> rows;
  if (nodes.empty()) return rows;

  FILE * proc_pipe = popen("ps -eo args --no-headers", "r");
  std::vector<QString> proc_args;
  if (proc_pipe != nullptr) {
    char line[2048] = {0};
    while (fgets(line, sizeof(line), proc_pipe) != nullptr) {
      const QString args = QString::fromUtf8(line).trimmed();
      if (!args.isEmpty()) proc_args.push_back(args);
    }
    pclose(proc_pipe);
  }

  auto infer_startup_file = [](const QString & args) -> QString {
    const QStringList tokens = args.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (tokens.isEmpty()) return "-";
    for (const auto & t : tokens) {
      if (t.endsWith(".launch.py") || t.endsWith(".py") || t.endsWith(".xml")) {
        return QFileInfo(t).fileName();
      }
    }
    if ((tokens[0] == "python" || tokens[0] == "python3") && tokens.size() > 1) {
      return QFileInfo(tokens[1]).fileName();
    }
    return QFileInfo(tokens[0]).fileName();
  };

  for (const auto & node_name : nodes) {
    QString startup = "-";
    const QString node_token = "__node:=" + node_name;
    for (const auto & args : proc_args) {
      if (args.contains(node_token)) {
        startup = infer_startup_file(args);
        break;
      }
    }
    rows.push_back({node_name, startup});
  }
  return rows;
}

std::vector<QString> ReadRos2SimpleList(const QString & subcommand)
{
  std::vector<QString> rows;
  const QString cmd = QString("bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && %1 2>/dev/null'").arg(subcommand);
  FILE * pipe = popen(cmd.toStdString().c_str(), "r");
  if (pipe == nullptr) return rows;
  char line[512] = {0};
  while (fgets(line, sizeof(line), pipe) != nullptr) {
    const QString value = QString::fromUtf8(line).trimmed();
    if (!value.isEmpty()) rows.push_back(value);
  }
  pclose(pipe);
  return rows;
}

std::vector<TopicTypeRow> ReadRos2TopicTypeRows()
{
  static TopicRateSampler sampler;
  return sampler.Snapshot();
}

std::vector<ParamRow> ReadRos2ParamRows(
  const QString & node_name, const std::atomic_bool * cancel_flag,
  int timeout_ms)
{
  std::vector<ParamRow> rows;
  if (node_name.trimmed().isEmpty()) return rows;
  const QString dump_cmd = QString("source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 param dump %1 2>/dev/null")
                             .arg(ShellQuote(node_name.trimmed()));
  QProcess dump_proc;
  dump_proc.setProcessChannelMode(QProcess::MergedChannels);
  dump_proc.start("bash", {"-lc", dump_cmd});
  if (!dump_proc.waitForStarted(200)) {
    return rows;
  }
  int waited_ms = 0;
  const int wait_step_ms = 50;
  while (!dump_proc.waitForFinished(wait_step_ms)) {
    waited_ms += wait_step_ms;
    if ((cancel_flag && cancel_flag->load()) || waited_ms >= timeout_ms) {
      dump_proc.kill();
      dump_proc.waitForFinished(100);
      return rows;
    }
  }
  const std::string yaml_text = QString::fromUtf8(dump_proc.readAllStandardOutput()).toStdString();
  if (yaml_text.empty()) {
    return rows;
  }
  try {
    YAML::Node root = YAML::Load(yaml_text); YAML::Node params;
    if (root[node_name.toStdString()]) params = root[node_name.toStdString()]["ros__parameters"];
    else if (root.begin() != root.end()) params = root.begin()->second["ros__parameters"];
    if (!params || !params.IsMap()) return rows;
    for (auto it = params.begin(); it != params.end(); ++it) {
      const std::string key = it->first.as<std::string>(); const YAML::Node val_node = it->second;
      QString value = "-";
      if (val_node.IsScalar()) value = QString::fromStdString(val_node.as<std::string>());
      else { YAML::Emitter out; out << val_node; if (out.good()) value = QString::fromStdString(out.c_str()); }
      rows.push_back({QString::fromStdString(key), value});
    }
  } catch (...) { return {}; }
  return rows;
}

ResourceUsage ReadResourceUsage(
  unsigned long long & prev_idle, unsigned long long & prev_total,
  unsigned long long & prev_rx, unsigned long long & prev_tx,
  unsigned long long & prev_disk_read, unsigned long long & prev_disk_write,
  const std::chrono::steady_clock::time_point & last_time,
  std::chrono::steady_clock::time_point & now_time)
{
  ResourceUsage usage; now_time = std::chrono::steady_clock::now();
  unsigned long long idle = 0, total = 0;
  if (ReadCpuCounters(idle, total)) {
    const unsigned long long total_diff = total - prev_total;
    const unsigned long long idle_diff = idle - prev_idle;
    if (total_diff > 0) usage.cpu = static_cast<int>(100.0 * (static_cast<double>(total_diff - idle_diff) / static_cast<double>(total_diff)));
    prev_idle = idle; prev_total = total;
  }
  std::ifstream mem_file("/proc/meminfo");
  if (mem_file.is_open()) {
    long total_kb = 0, avail_kb = 0, value = 0; std::string key, unit;
    while (mem_file >> key >> value >> unit) { if (key == "MemTotal:") total_kb = value; else if (key == "MemAvailable:") avail_kb = value; }
    if (total_kb > 0) usage.mem = static_cast<int>(100.0 * (total_kb - avail_kb) / total_kb);
  }
  FILE * gpu_pipe = popen("nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null", "r");
  if (gpu_pipe != nullptr) {
    char line[256] = {0};
    if (fgets(line, sizeof(line), gpu_pipe) != nullptr) {
      int gpu = 0, used = 0, total_mem = 0;
      if (sscanf(line, "%d, %d, %d", &gpu, &used, &total_mem) == 3) { usage.gpu = gpu; if (total_mem > 0) usage.vram = static_cast<int>(100.0 * used / total_mem); }
    }
    pclose(gpu_pipe);
  }
  unsigned long long rx = 0, tx = 0;
  if (ReadNetworkBytes(rx, tx)) {
    const double dt = std::chrono::duration<double>(now_time - last_time).count();
    if (dt > 0.0001) { usage.down_kbps = (static_cast<double>(rx - prev_rx) / 1024.0) / dt; usage.up_kbps = (static_cast<double>(tx - prev_tx) / 1024.0) / dt; }
    prev_rx = rx; prev_tx = tx;
  }
  unsigned long long disk_read = 0, disk_write = 0;
  if (ReadDiskBytes(disk_read, disk_write)) {
    const double dt = std::chrono::duration<double>(now_time - last_time).count();
    if (dt > 0.0001) { usage.disk_read_kbps = (static_cast<double>(disk_read - prev_disk_read) / 1024.0) / dt; usage.disk_write_kbps = (static_cast<double>(disk_write - prev_disk_write) / 1024.0) / dt; }
    prev_disk_read = disk_read; prev_disk_write = disk_write;
  }
  return usage;
}

}  // namespace ros_robot_assist_tools::ui
