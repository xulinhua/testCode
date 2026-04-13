#include "../include/bas_operate/bas_utils.hpp"
#include <random>
#include <thread>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <set>

#ifndef _WIN32
#include <cxxabi.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <dirent.h>
#endif

namespace fs = std::filesystem;

namespace basmodule {

// ============================================================================
// 系统资源监控函数实现
// ============================================================================

float get_cpu_usage() 
{
#ifdef _WIN32
    // Windows实现
    FILETIME idle_time, kernel_time, user_time;
    if (GetSystemTimes(&idle_time, &kernel_time, &user_time)) {
        static ULARGE_INTEGER last_idle = {0};
        static ULARGE_INTEGER last_kernel = {0};
        static ULARGE_INTEGER last_user = {0};
        
        ULARGE_INTEGER current_idle = {{idle_time.dwLowDateTime, idle_time.dwHighDateTime}};
        ULARGE_INTEGER current_kernel = {{kernel_time.dwLowDateTime, kernel_time.dwHighDateTime}};
        ULARGE_INTEGER current_user = {{user_time.dwLowDateTime, user_time.dwHighDateTime}};
        
        if (last_idle.QuadPart != 0) 
        {
            ULONGLONG idle_diff = current_idle.QuadPart - last_idle.QuadPart;
            ULONGLONG kernel_diff = current_kernel.QuadPart - last_kernel.QuadPart;
            ULONGLONG user_diff = current_user.QuadPart - last_user.QuadPart;
            ULONGLONG total_diff = kernel_diff + user_diff;
            
            if (total_diff > 0) {
                return static_cast<float>((total_diff - idle_diff) * 100.0 / total_diff);
            }
        }
        
        last_idle = current_idle;
        last_kernel = current_kernel;
        last_user = current_user;
    }
    return 0.0f;
#else
    // Linux实现
    static long last_total = 0;
    static long last_idle = 0;
    
    std::ifstream file("/proc/stat");
    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        file.close();
        
        std::istringstream ss(line);
        std::string cpu_label;
        long user, nice, system, idle, iowait, irq, softirq, steal;
        
        ss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
        
        long total = user + nice + system + idle + iowait + irq + softirq + steal;
        long total_diff = total - last_total;
        long idle_diff = idle - last_idle;
        
        if (total_diff > 0) {
            float usage = static_cast<float>((total_diff - idle_diff) * 100.0 / total_diff);
            last_total = total;
            last_idle = idle;
            return usage;
        }
    }
    return 0.0f;
#endif
}

void get_memory_usage(uint64_t& used_memory, uint64_t& total_memory, float& usage_percent) 
{
#ifdef _WIN32
    MEMORYSTATUSEX mem_info;
    mem_info.dwLength = sizeof(MEMORYSTATUSEX);
    if (GlobalMemoryStatusEx(&mem_info)) {
        total_memory = mem_info.ullTotalPhys / (1024 * 1024); // MB
        used_memory = (mem_info.ullTotalPhys - mem_info.ullAvailPhys) / (1024 * 1024); // MB
        usage_percent = static_cast<float>(mem_info.dwMemoryLoad);
    } else {
        total_memory = used_memory = 0;
        usage_percent = 0.0f;
    }
#else
    // Linux实现
    std::ifstream file("/proc/meminfo");
    if (file.is_open()) {
        std::string line;
        uint64_t mem_total = 0, mem_free = 0, mem_buffers = 0, mem_cached = 0;
        
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            uint64_t value;
            std::string unit;
            
            iss >> key >> value >> unit;
            
            if (key == "MemTotal:") {
                mem_total = value; // KB
            } else if (key == "MemFree:") {
                mem_free = value; // KB
            } else if (key == "Buffers:") {
                mem_buffers = value; // KB
            } else if (key == "Cached:") {
                mem_cached = value; // KB
            }
        }
        file.close();
        
        total_memory = mem_total / 1024; // MB
        uint64_t actual_free = mem_free + mem_buffers + mem_cached;
        used_memory = (mem_total - actual_free) / 1024; // MB
        usage_percent = static_cast<float>(used_memory * 100.0 / total_memory);
    } else {
        total_memory = used_memory = 0;
        usage_percent = 0.0f;
    }
#endif
}

float get_system_temperature() {
#ifndef _WIN32
    // Linux实现 - 读取系统温度
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (file.is_open()) {
        int temp_milli_celsius;
        file >> temp_milli_celsius;
        file.close();
        return static_cast<float>(temp_milli_celsius) / 1000.0f;
    }
#endif
    return 0.0f; // 无法获取温度时返回0
}

// ============================================================================
// 进程管理函数实现
// ============================================================================

bool is_process_alive(int pid) 
{
    if (pid <= 0) return false; 
#ifdef _WIN32
    HANDLE process = OpenProcess(SYNCHRONIZE, FALSE, pid);
    if (process != NULL) 
    {
        DWORD exit_code;
        if (GetExitCodeProcess(process, &exit_code)) {
            CloseHandle(process);
            return exit_code == STILL_ACTIVE;
        }
        CloseHandle(process);
    }
    return false;
#else
    // 使用 waitpid 检查进程状态，而不是 kill(pid, 0)
    // kill(pid, 0) 对僵尸进程也会返回成功，导致误判
    int status;
    pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == -1) 
	{
        // 进程不存在或错误
        if (errno == ECHILD) {
            // 没有这样的子进程，说明已经退出并被回收
            return false;
        }
        // 其他错误，可能是权限问题，用 kill 再次确认
        return (kill(pid, 0) == 0);
    } else if (result == 0) {
        // 进程还在运行
        return true;
    } else {
        // 进程已经退出（result == pid）
        // waitpid 已经回收了子进程，避免僵尸进程
        return false;
    }
#endif
}

int launch_process(const std::string& executable_path, const std::vector<std::string>& arguments, const std::string& working_directory) 
{
    if (executable_path.empty()) {
        return -1;
    }
    
#ifdef _WIN32
    std::string cmd = executable_path;
    for (const auto& arg : arguments) {
        cmd += " " + arg;
    }
    
    STARTUPINFO si = {0};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi = {0};
    
    if (CreateProcess(NULL, const_cast<char*>(cmd.c_str()), NULL, NULL, FALSE, 0, NULL, 
                      working_directory.empty() ? NULL : working_directory.c_str(), &si, &pi)) {
        CloseHandle(pi.hThread);
        int pid = pi.dwProcessId;
        CloseHandle(pi.hProcess);
        return pid;
    }
    return -1;
#else
    pid_t pid = fork();
    if (pid == 0) 
	{
        // 子进程
        // 创建新的进程组，使子进程成为进程组组长
        // 这样我们可以向整个进程组发送信号，确保所有子进程都能收到
        setpgid(0, 0);
        
        // 忽略 SIGINT 信号，让父进程完全控制子进程的生命周期
        // 这样当用户按下 Ctrl+C 时，子进程不会立即退出，而是由父进程发送 SIGTERM 优雅停止
        signal(SIGINT, SIG_IGN);
        
        // 注意：execvp 会重置所有信号处理器为默认值（除了 SIG_IGN）
        // 所以这里设置 SIGTERM 为默认处理器实际上是有效的
        // 因为 ROS2 节点会安装自己的 SIGTERM 处理器
        
        if (!working_directory.empty()) {
            chdir(working_directory.c_str());
        }
        
        // 构建参数数组
        std::vector<char*> args;
        args.push_back(const_cast<char*>(executable_path.c_str()));
        for (const auto& arg : arguments) {
            args.push_back(const_cast<char*>(arg.c_str()));
        }
        args.push_back(nullptr);
        
        execvp(executable_path.c_str(), args.data());
        exit(1); // 如果execvp失败
    } 
	else if (pid > 0) 
	{
        // 父进程：确保子进程已经加入新的进程组
        // 这样可以向进程组发送信号，确保所有子线程/子进程都能收到
        setpgid(pid, pid);
        return static_cast<int>(pid);
    }
    return -1;
#endif
}

bool stop_process(int pid, bool force)
{
    if (pid <= 0) return false;
    
#ifdef _WIN32
    HANDLE process = OpenProcess(PROCESS_TERMINATE, FALSE, pid);
    if (process != NULL) {
        bool result = TerminateProcess(process, 1) != 0;
        CloseHandle(process);
        return result;
    }
    return false;
#else
    int signal = force ? SIGKILL : SIGTERM;
    // 向整个进程组发送信号，确保所有子线程/子进程都能收到
    // 使用 -pid 表示向进程组发送信号
    int result = kill(-pid, signal);
    // 如果进程组发送失败，尝试向单个进程发送
    if (result != 0) 
    {
        result = kill(pid, signal);
    }
    return (result == 0);
#endif
}

bool wait_for_process(int pid, int timeout_ms) 
{
    if (pid <= 0) return false;
    auto start_time = std::chrono::steady_clock::now();
    while (is_process_alive(pid)) 
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            return false;
        }
        sleep_ms(100); // 每100ms检查一次
    }
    return true;
}

std::vector<int> get_process_list(const std::string& name_filter) 
{
    std::vector<int> pids;
#ifdef _WIN32
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot != INVALID_HANDLE_VALUE) {
        PROCESSENTRY32 entry;
        entry.dwSize = sizeof(PROCESSENTRY32);
        
        if (Process32First(snapshot, &entry)) {
            do {
                std::string process_name(entry.szExeFile);
                if (name_filter.empty() || 
                    process_name.find(name_filter) != std::string::npos) {
                    pids.push_back(entry.th32ProcessID);
                }
            } while (Process32Next(snapshot, &entry));
        }
        CloseHandle(snapshot);
    }
#else
    DIR* dir = opendir("/proc");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (std::isdigit(entry->d_name[0])) {
                std::string pid_str(entry->d_name);
                int pid = std::stoi(pid_str);
                
                if (!name_filter.empty()) {
                    std::string cmdline_path = "/proc/" + pid_str + "/cmdline";
                    std::ifstream cmdline_file(cmdline_path);
                    if (cmdline_file.is_open()) {
                        std::string cmdline;
                        std::getline(cmdline_file, cmdline, '\0');
                        cmdline_file.close();
                        
                        if (cmdline.find(name_filter) != std::string::npos) {
                            pids.push_back(pid);
                        }
                    }
                } else {
                    pids.push_back(pid);
                }
            }
        }
        closedir(dir);
    }
#endif
    return pids;
}

// ============================================================================
// 拓扑排序函数实现
// ============================================================================

std::vector<std::string> topological_sort(const std::map<std::string, std::vector<std::string>>& dependencies) 
{
    std::vector<std::string> result;
    std::map<std::string, int> in_degree;
    std::map<std::string, bool> visited;
    
    // 收集所有模块（包括只作为依赖项出现的模块）
    std::set<std::string> all_modules;
    for (const auto& pair : dependencies) 
    {
        all_modules.insert(pair.first);
        for (const auto& dep : pair.second) 
        {
            all_modules.insert(dep);
        }
    }
    
    // 初始化入度
    for (const auto& module : all_modules) 
    {
        in_degree[module] = 0;
        visited[module] = false;
    }
    
    // 计算入度：入度表示有多少模块依赖该模块
    for (const auto& pair : dependencies) 
    {
        // pair.first 依赖 pair.second 中的模块
        // 所以 pair.first 的入度应该增加（因为它依赖其他模块，需要等其他模块先启动）
        in_degree[pair.first] += pair.second.size();
    }
    
    // 使用队列进行拓扑排序（从入度为0的模块开始）
    std::vector<std::string> queue;
    for (const auto& pair : in_degree) 
    {
        if (pair.second == 0) {
            queue.push_back(pair.first);
        }
    }
    
    while (!queue.empty()) 
    {
        std::string current = queue.back();
        queue.pop_back();
        result.push_back(current);
        visited[current] = true;
        
        // 查找所有依赖当前模块的模块，减少它们的入度
        for (const auto& pair : dependencies) 
        {
            // pair.first 依赖 pair.second
            // 如果 current 在 pair.second 中，说明 pair.first 依赖 current
            bool depends_on_current = false;
            for (const auto& dep : pair.second) 
            {
                if (dep == current) 
                {
                    depends_on_current = true;
                    break;
                }
            }
            
            if (depends_on_current && !visited[pair.first]) 
            {
                in_degree[pair.first]--;
                if (in_degree[pair.first] == 0) {
                    queue.push_back(pair.first);
                }
            }
        }
    }  
    
    // 检查是否存在循环依赖
    if (result.size() != all_modules.size()) {
        throw std::runtime_error("Circular dependency detected in module dependencies");
    }  
    
    return result;
}

bool has_circular_dependency(const std::map<std::string, std::vector<std::string>>& dependencies) 
{
    try 
    {
        topological_sort(dependencies);
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

// ============================================================================
// 时间工具函数实现（补充）
// ============================================================================

void sleep_ms(int milliseconds) 
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

long long duration_ms(const std::chrono::steady_clock::time_point& start, 
                      const std::chrono::steady_clock::time_point& end) 
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

std::string format_timestamp(const std::chrono::system_clock::time_point& timestamp, 
                             const std::string& format) 
{
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), format.c_str());
    return ss.str();
}

std::string get_current_timestamp() 
{
    auto now = std::chrono::system_clock::now();
    return format_timestamp(now, "%Y%m%d_%H%M%S");
}

// ============================================================================
// 原有工具函数实现
// ============================================================================

// 辅助函数：格式化浮点数为字符串，默认保留6位小数
std::string format_float(float value, int precision) 
{
    std::stringstream ss;
    ss.precision(precision);
    ss << std::fixed << value;
    return ss.str();
}

std::string format_double(double value, int precision) 
{
    std::stringstream ss;
    ss.precision(precision);
    ss << std::fixed << value;
    return ss.str();
}

// 字符串工具函数实现

//判断字符串是否为空或仅包含空格
bool is_empty_or_only_spaces(const std::string& str) 
{
    if (str.empty()) {
        return true;
    }
    for (char c : str) 
    {
        if (c != ' ') { // 仅判断是否是空格
            return false;
        }
    }
    return true;
}

std::string trim(const std::string& str) 
{
    if (str.empty()) {
        return str;
    }
    
    // 找到第一个非空白字符的位置
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (first == std::string::npos) {
        return "";
    }
    
    // 找到最后一个非空白字符的位置
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    
    // 返回修剪后的子字符串
    return str.substr(first, (last - first + 1));
}

std::string to_lower(const std::string& str) 
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return result;
}

std::string to_upper(const std::string& str) 
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c){ return std::toupper(c); });
    return result;
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter) 
{
    std::vector<std::string> tokens;
    if (str.empty()) {
        return tokens;
    }
    
    size_t start = 0;
    size_t end = str.find(delimiter);
    
    while (end != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + delimiter.length();
        end = str.find(delimiter, start);
    }
    
    // 添加最后一个token
    tokens.push_back(str.substr(start));
    
    return tokens;
}

std::string get_n_from_right(const std::string& str, char delimiter, int n)
{
    if (str.empty() || n <= 0) {
        return "";
    }
    size_t pos = str.length();
    int count = 0;
    // 从右往左查找第n个分隔符
    for (int i = str.length() - 1; i >= 0; --i) 
    {
        if (str[i] == delimiter) 
        {
            count++;
            if (count == n) 
            {
                pos = i;
                break;
            }
        }
    }
    // 如果找到了第n个分隔符，则返回其右侧的内容
    if (count == n) {
        return str.substr(pos + 1);
    }
    // 如果没有找到足够的分隔符，返回空字符串
    return "";
}

std::string replace(const std::string& str, const std::string& from, const std::string& to) 
{
    std::string result = str;
    if (from.empty()) {
        return result;
    }
    
    size_t start_pos = 0;
    while ((start_pos = result.find(from, start_pos)) != std::string::npos) {
        result.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
    
    return result;
}

bool starts_with(const std::string& str, const std::string& prefix) 
{
    if (prefix.length() > str.length()) {
        return false;
    }
    return str.substr(0, prefix.length()) == prefix;
}

bool ends_with(const std::string& str, const std::string& suffix) 
{
    if (suffix.length() > str.length()) {
        return false;
    }
    return str.substr(str.length() - suffix.length()) == suffix;
}

// 数学工具函数实现
static std::mt19937 rng(static_cast<unsigned int>(std::time(nullptr)));

int random_int(int min, int max) {
    if (min > max) {
        std::swap(min, max);
    }
    
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng);
}

double random_double(double min, double max) {
    if (min > max) {
        std::swap(min, max);
    }
    
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

bool float_equal(double a, double b, double epsilon) {
    return std::abs(a - b) < epsilon;
}

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radians_to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

std::string get_cur_date_str()
{
    auto now = std::chrono::system_clock::now();// 获取当前时间点
    auto time_t_now = std::chrono::system_clock::to_time_t(now);// 转换为时间结构
    std::tm tm_now = *std::localtime(&time_t_now);
    std::ostringstream oss;// 格式化为字符串：YYYYMMDD
    oss << std::put_time(&tm_now, "%Y%m%d");
    return oss.str();
}

std::string get_cur_time_str()
{
    auto now = std::chrono::system_clock::now();// 获取当前时间点
    auto time_t_now = std::chrono::system_clock::to_time_t(now);// 转换为时间结构
    std::tm tm_now = *std::localtime(&time_t_now);
    auto duration = now.time_since_epoch();// 获取毫秒部分
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;
    std::ostringstream oss;// 格式化为字符串：HH:MM:SS.fff
    oss << std::put_time(&tm_now, "%H:%M:%S") << "." << std::setw(3) << std::setfill('0') << milliseconds;
    return oss.str();
}

std::string pid_to_str(int pid)
{
    return std::to_string(pid);
}

std::string get_timestamp() 
{
    std::string date_part = get_cur_date_str();
    std::string time_part = get_cur_time_str();
    return date_part + "_" + time_part;
}

// 时间工具函数实现
long long get_timestamp_ms() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

std::string format_timestamp(long long timestamp, const std::string& format) {
    // 将毫秒时间戳转换为time_t
    std::time_t t = static_cast<std::time_t>(timestamp / 1000);
    
    // 转换为本地时间
    std::tm* tm_local = std::localtime(&t);
    
    // 格式化时间
    std::ostringstream oss;
    oss << std::put_time(tm_local, format.c_str());
    
    return oss.str();
}

long long duration_ms(const std::chrono::high_resolution_clock::time_point& start,
                   const std::chrono::high_resolution_clock::time_point& end) {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    return duration.count();
}

// 重载版本的format_float_array函数实现
std::string format_float_array(const std::vector<float>& values, int precision) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << std::fixed << std::setprecision(precision) << values[i];
    }
    oss << "]";
    return oss.str();
}

std::string format_float_array(const std::vector<double>& values, int precision) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << std::fixed << std::setprecision(precision) << values[i];
    }
    oss << "]";
    return oss.str();
}

// 字符串显示宽度工具函数实现

size_t getDisplayWidth(const std::string& str) 
{
    size_t width = 0;
    for (size_t i = 0; i < str.length();) {
        unsigned char ch = static_cast<unsigned char>(str[i]);
        if (ch >= 0xE0 && ch <= 0xEF) { // 3字节UTF-8字符（通常是中文）
            width += 2; // 中文字符在终端中通常占用2个字符宽度
            i += 3;
        } else if (ch >= 0x80) { // 其他多字节字符
            width += 2; // 保守估计，也按2个字符宽度计算
            i += 2;
        } else { // ASCII字符
            width += 1;
            i += 1;
        }
    }
    return width;
}

std::string padToWidth(const std::string& str, size_t targetWidth) 
{
    size_t currentWidth = getDisplayWidth(str);
    if (currentWidth >= targetWidth) {
        return str;
    }
    return str + std::string(targetWidth - currentWidth, ' ');
}

std::string get_type_name(const std::type_info& type) 
{
#ifdef _WIN32
    // Windows: 返回原始类型名称
    return type.name();
#else
    // Linux: 使用abi::__cxa_demangle解析类型名称
    int status = 0;
    char* demangled = abi::__cxa_demangle(type.name(), 0, 0, &status);
    std::string result = (status == 0 && demangled) ? demangled : type.name();
    if (demangled) free(demangled);
    return result;
#endif
}

// 文件路径工具函数实现

// 生成缩进
std::string generateIndent(int level) 
{
    const int SPACES_PER_INDENT = 2; // 每个缩进级别2个空格
    return std::string(level * SPACES_PER_INDENT, ' ');
}

} // namespace basmodule