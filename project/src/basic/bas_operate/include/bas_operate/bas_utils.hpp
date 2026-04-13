#ifndef BAS_OPERATE_BAS_UTILS_HPP
#define BAS_OPERATE_BAS_UTILS_HPP

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <random>
#include <algorithm>
#include <functional>
#include <type_traits>
#include <iomanip>
#include <sstream>
#include <typeinfo>
#include <memory>
#include <ctime>
#include <thread>
#include <cstdint>
#include <future>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#include <tlhelp32.h>
#include <psapi.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#endif

namespace basmodule {

// ============================================================================
// 系统资源监控函数
// ============================================================================

/**
 * @brief 获取系统CPU使用率
 * @return CPU使用率百分比
 */
float get_cpu_usage();

/**
 * @brief 获取系统内存使用情况
 * @param used_memory[out] 已使用内存(MB)
 * @param total_memory[out] 总内存(MB)
 * @param usage_percent[out] 使用率百分比
 */
void get_memory_usage(uint64_t& used_memory, uint64_t& total_memory, float& usage_percent);

/**
 * @brief 获取系统温度
 * @return 系统温度(摄氏度)
 */
float get_system_temperature();

// ============================================================================
// 进程管理函数
// ============================================================================

/**
 * @brief 检查进程是否存活
 * @param pid 进程ID
 * @return true if process is alive, false otherwise
 */
bool is_process_alive(int pid);

/**
 * @brief 启动进程
 * @param executable_path 可执行文件路径
 * @param arguments 启动参数
 * @param working_directory 工作目录
 * @return 进程ID，失败返回-1
 */
int launch_process(const std::string& executable_path, const std::vector<std::string>& arguments = {}, 
    const std::string& working_directory = "");

/**
 * @brief 停止进程
 * @param pid 进程ID
 * @param force 是否强制停止
 * @return true if successful, false otherwise
 */
bool stop_process(int pid, bool force = false);

/**
 * @brief 等待进程结束
 * @param pid 进程ID
 * @param timeout_ms 超时时间(毫秒)
 * @return true if process ended, false if timeout
 */
bool wait_for_process(int pid, int timeout_ms = 5000);

/**
 * @brief 获取进程ID列表
 * @param name_filter 名称过滤器(可选)
 * @return 匹配的进程ID列表
 */
std::vector<int> get_process_list(const std::string& name_filter = "");

// ============================================================================
// 拓扑排序函数
// ============================================================================

/**
 * @brief 解析依赖关系图并生成拓扑排序
 * @param dependencies 依赖关系映射
 * @return 排序结果
 * @throws std::runtime_error 如果存在循环依赖
 */
std::vector<std::string> topological_sort(const std::map<std::string, std::vector<std::string>>& dependencies);

/**
 * @brief 检查是否存在循环依赖
 * @param dependencies 依赖关系映射
 * @return true if circular dependency exists, false otherwise
 */
bool has_circular_dependency(const std::map<std::string, std::vector<std::string>>& dependencies);

// ============================================================================
// 时间工具函数（补充）
// ============================================================================

/**
 * @brief 睡眠指定时间
 * @param milliseconds 时间(毫秒)
 */
void sleep_ms(int milliseconds);

/**
 * @brief 计算两个时间点的时间差(毫秒) - steady_clock版本
 * @param start 开始时间
 * @param end 结束时间
 * @return 时间差(毫秒)
 */
long long duration_ms(const std::chrono::steady_clock::time_point& start, 
                      const std::chrono::steady_clock::time_point& end);

/**
 * @brief 格式化时间戳 - system_clock::time_point版本
 * @param timestamp 时间戳
 * @param format 格式字符串
 * @return 格式化后的时间字符串
 */
std::string format_timestamp(const std::chrono::system_clock::time_point& timestamp, 
                             const std::string& format = "%Y-%m-%d %H:%M:%S");

/**
 * @brief 获取当前时间戳字符串
 * @return 时间戳字符串
 */
std::string get_current_timestamp();

// 辅助函数：格式化浮点数为字符串，默认保留6位小数
std::string format_float(float value, int precision = 6);

std::string format_double(double value, int precision = 6);

//判断字符串是否为空或仅包含空格
bool is_empty_or_only_spaces(const std::string& str);

/**
 * @brief 将uint8_t向量转换为字符串表示形式
 * 
 * @param list uint8_t类型的向量
 * @return std::string 格式化的字符串，例如"[0,1,2,3]"
 */
template<typename T>
std::string get_list_string(const std::vector<T>& list)
{
    // 构建ID列表字符串，格式为[0,1,2,3]
    std::string list_str = "[";
    for (size_t i = 0; i < list.size(); ++i) 
    {
        if constexpr (std::is_same_v<T, bool>) {
            list_str += list[i] ? "true" : "false";
        } else if constexpr (std::is_same_v<T, std::string>) {
            list_str += "\"" + list[i] + "\"";
        } else if constexpr (std::is_same_v<T, float>) {
            list_str += format_float(list[i]);
        } else if constexpr (std::is_same_v<T, double>) {
            list_str += format_double(list[i]);
        } else if constexpr (std::is_same_v<T, uint8_t>) {
            list_str += std::to_string(static_cast<int>(list[i]));
        } else {
            list_str += std::to_string(list[i]);
        }
        if (i < list.size() - 1) 
        {
            list_str += ",";
        }
    }
    list_str += "]";
    return list_str;
}

/**
 * @brief 将字符串转换为向量列表
 * 
 * @tparam T 向量元素类型
 * @param str 输入的字符串，格式如"[1,2,3,4]"
 * @return std::vector<T> 解析后的向量
 */
template<typename T>
std::vector<T> parse_list_from_string(const std::string& str)
{
    std::vector<T> result;
    
    // 检查字符串是否以'['开头和']'结尾
    if (str.empty() || str.front() != '[' || str.back() != ']') {
        return result; // 返回空向量
    }
    
    // 提取[]内的内容
    std::string content = str.substr(1, str.length() - 2);
    if (content.empty()) {
        return result; // 返回空向量
    }

    // 分割字符串
    std::stringstream ss(content);
    std::string item;
    
    while (std::getline(ss, item, ',')) 
    {
        // 去除前后空格
        size_t start = item.find_first_not_of(" \t\r\n");
        size_t end = item.find_last_not_of(" \t\r\n");
        
        if (start != std::string::npos && end != std::string::npos) 
        {
            item = item.substr(start, end - start + 1);
            
            if constexpr (std::is_same_v<T, bool>) 
            {
                // 处理bool类型
                if (item == "true" || item == "1") {
                    result.push_back(true);
                } else if (item == "false" || item == "0") {
                    result.push_back(false);
                }
            } 
            else 
            {
                // 处理其他类型
                std::istringstream iss(item);
                T value;
                if (iss >> value) {
                    result.push_back(value);
                }
            }
        }
    } 
    return result;
}

// 字符串工具函数

/**
 * @brief 去除字符串首尾空白字符
 * 
 * @param str 输入字符串
 * @return std::string 修剪后的字符串
 */
std::string trim(const std::string& str);

/**
 * @brief 将字符串转为小写
 * 
 * @param str 输入字符串
 * @return std::string 转换后的小写字符串
 */
std::string to_lower(const std::string& str);

/**
 * @brief 将字符串转为大写
 * 
 * @param str 输入字符串
 * @return std::string 转换后的大写字符串
 */
std::string to_upper(const std::string& str);

/**
 * @brief 分割字符串
 * 
 * @param str 输入字符串
 * @param delimiter 分隔符
 * @return std::vector<std::string> 分割后的字符串向量
 */
std::vector<std::string> split(const std::string& str, const std::string& delimiter);

/**
 * @brief 获取字符串中从右往左数第n个分隔符右侧的内容
 * 
 * @param str 输入字符串
 * @param delimiter 分隔符
 * @param n 从右往左数的第几个分隔符（1表示最后一个，2表示倒数第二个，以此类推）
 * @return std::string 第n个分隔符右侧的内容，如果不存在则返回空字符串
 */
std::string get_n_from_right(const std::string& str, char delimiter, int n);

/**
 * @brief 替换字符串中的子串
 * 
 * @param str 输入字符串
 * @param from 被替换的子串
 * @param to 替换后的子串
 * @return std::string 替换后的字符串
 */
std::string replace(const std::string& str, const std::string& from, const std::string& to);

/**
 * @brief 检查字符串是否以指定前缀开始
 * 
 * @param str 输入字符串
 * @param prefix 前缀
 * @return bool 是否以前缀开始
 */
bool starts_with(const std::string& str, const std::string& prefix);

/**
 * @brief 检查字符串是否以指定后缀结束
 * 
 * @param str 输入字符串
 * @param suffix 后缀
 * @return bool 是否以后缀结束
 */
bool ends_with(const std::string& str, const std::string& suffix);

// 数学工具函数

/**
 * @brief 生成指定范围内的随机整数
 * 
 * @param min 最小值
 * @param max 最大值
 * @return int 随机整数
 */
int random_int(int min, int max);

/**
 * @brief 生成指定范围内的随机浮点数
 * 
 * @param min 最小值
 * @param max 最大值
 * @return double 随机浮点数
 */
double random_double(double min, double max);

/**
 * @brief 比较两个浮点数是否相等（考虑精度误差）
 * 
 * @param a 第一个浮点数
 * @param b 第二个浮点数
 * @param epsilon 精度阈值
 * @return bool 是否相等
 */
bool float_equal(double a, double b, double epsilon = 1e-9);

/**
 * @brief 将角度转换为弧度
 * 
 * @param degrees 角度值
 * @return double 弧度值
 */
double degrees_to_radians(double degrees);

/**
 * @brief 将弧度转换为角度
 * 
 * @param radians 弧度值
 * @return double 角度值
 */
double radians_to_degrees(double radians);

// 时间工具函数
//获取日期字符串
std::string get_cur_date_str();

//获取时间字符串（包含时分秒和微秒）
std::string get_cur_time_str();

std::string pid_to_str(int pid);

/**
 * @brief 生成基于启动时间的唯一日志文件名
 * 
 * 生成格式为：YYYYMMDD_HHMMSS_microseconds 的唯一时间戳
 * 用于确保同一进程多次启动时日志文件名不冲突
 * 
 * @return std::string 唯一的时间戳字符串
 */
std::string get_timestamp();

/**
 * @brief 获取当前时间戳（毫秒）
 * 
 * @return long long 当前时间戳
 */
long long get_timestamp_ms();

/**
 * @brief 格式化时间戳为字符串
 * 
 * @param timestamp 时间戳（毫秒）
 * @param format 格式字符串，如"%Y-%m-%d %H:%M:%S"
 * @return std::string 格式化后的时间字符串
 */
std::string format_timestamp(long long timestamp, const std::string& format = "%Y-%m-%d %H:%M:%S");

/**
 * @brief 计算两个时间点之间的时间差（毫秒）
 * 
 * @param start 开始时间点
 * @param end 结束时间点
 * @return long long 时间差（毫秒）
 */
long long duration_ms(const std::chrono::high_resolution_clock::time_point& start,
                     const std::chrono::high_resolution_clock::time_point& end);

// 容器工具函数

/**
 * @brief 检查容器中是否存在指定元素
 * 
 * @tparam Container 容器类型
 * @tparam T 元素类型
 * @param container 容器
 * @param value 元素值
 * @return bool 是否存在
 */
template<typename Container, typename T>
bool contains(const Container& container, const T& value) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

/**
 * @brief 过滤容器中的元素
 * 
 * @tparam Container 容器类型
 * @tparam Predicate 谓词函数类型
 * @param container 容器
 * @param predicate 谓词函数
 * @return Container 过滤后的容器
 */
template<typename Container, typename Predicate>
Container filter(const Container& container, Predicate predicate) {
    Container result;
    std::copy_if(container.begin(), container.end(), std::back_inserter(result), predicate);
    return result;
}

/**
 * @brief 对容器中的元素进行映射转换
 * 
 * @tparam Container 容器类型
 * @tparam Mapper 映射函数类型
 * @param container 容器
 * @param mapper 映射函数
 * @return std::vector<decltype(mapper(*container.begin()))> 转换后的元素向量
 */
template<typename Container, typename Mapper>
auto map(const Container& container, Mapper mapper) -> std::vector<decltype(mapper(*container.begin()))> {
    std::vector<decltype(mapper(*container.begin()))> result;
    result.reserve(container.size());
    std::transform(container.begin(), container.end(), std::back_inserter(result), mapper);
    return result;
}

/**
 * @brief 将浮点数数组格式化为字符串，保留指定精度的小数位
 * 
 * @tparam T 浮点数类型 (float, double)
 * @param values 浮点数数组
 * @param precision 小数位精度，默认为6
 * @return std::string 格式化后的字符串，格式为"[value1, value2, ...]"
 */
template<typename T>
std::string format_float_array(const std::vector<T>& values, int precision = 6)
{
    static_assert(std::is_floating_point_v<T>, "T must be a floating point type");
    
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << std::fixed << std::setprecision(precision) << values[i];
    }
    oss << "]";
    return oss.str();
}

/**
 * @brief 将浮点数数组格式化为字符串，保留指定精度的小数位（重载版本）
 * 
 * @param values 浮点数数组
 * @param precision 小数位精度，默认为6
 * @return std::string 格式化后的字符串，格式为"[value1, value2, ...]"
 */
std::string format_float_array(const std::vector<float>& values, int precision = 6);

std::string format_float_array(const std::vector<double>& values, int precision = 6);

// 字符串显示宽度工具函数

/**
 * @brief 计算字符串在终端中的显示宽度（考虑中文字符）
 * 
 * @param str 输入字符串
 * @return size_t 显示宽度
 */
size_t getDisplayWidth(const std::string& str);

/**
 * @brief 填充空格以达到指定显示宽度
 * 
 * @param str 输入字符串
 * @param targetWidth 目标显示宽度
 * @return std::string 填充后的字符串
 */
std::string padToWidth(const std::string& str, size_t targetWidth);

// 类型信息工具函数

/**
 * @brief 获取类型名称的可读字符串
 * 
 * @param type std::type_info 对象
 * @return std::string 人类可读的类型名称
 */
std::string get_type_name(const std::type_info& type);

/**
 * @brief 生成缩进字符串
 * 
 * @param level 缩进级别，默认为1
 * @return std::string 指定级别的缩进字符串
 */
std::string generateIndent(int level = 1);

// ============================================================================
// 异步执行工具函数
// ============================================================================

/**
 * @brief 异步执行函数
 * @tparam Func 函数类型
 * @tparam Args 参数类型
 * @param func 函数
 * @param args 参数
 * @return std::future对象
 */
template<typename Func, typename... Args>
auto async_execute(Func&& func, Args&&... args) -> std::future<decltype(func(args...))> {
    return std::async(std::launch::async, std::forward<Func>(func), std::forward<Args>(args)...);
}

/**
 * @brief 执行函数（带超时）
 * @tparam Func 函数类型
 * @tparam ReturnType 返回类型
 * @param func 要执行的函数
 * @param timeout_ms 超时时间(毫秒)
 * @param default_value 超时的默认返回值
 * @return 函数执行结果或默认值
 */
template<typename Func, typename ReturnType>
ReturnType execute_with_timeout(Func func, int timeout_ms, const ReturnType& default_value) {
    std::promise<ReturnType> promise;
    std::future<ReturnType> future = promise.get_future();
    
    std::thread worker([&promise, func]() {
        try {
            promise.set_value(func());
        } catch (...) {
            promise.set_exception(std::current_exception());
        }
    });
    
    worker.detach();
    
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
        try {
            return future.get();
        } catch (...) {
            return default_value;
        }
    }
    
    return default_value;
}

/**
 * @brief 重试执行函数
 * @tparam Func 函数类型
 * @param func 要执行的函数
 * @param max_attempts 最大尝试次数
 * @param delay_ms 重试间隔(毫秒)
 * @return 函数执行结果
 * @throws std::exception 如果所有尝试都失败
 */
template<typename Func>
auto retry_execute(Func func, int max_attempts, int delay_ms = 1000) -> decltype(func()) {
    std::exception_ptr last_exception;
    
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        try {
            return func();
        } catch (...) {
            last_exception = std::current_exception();
            if (attempt < max_attempts - 1) {
                sleep_ms(delay_ms);
            }
        }
    }
    
    if (last_exception) {
        std::rethrow_exception(last_exception);
    }
    
    throw std::runtime_error("All retry attempts failed");
}

} // namespace basmodule

// ============================================================================
// YAML配置工具函数（需要yaml-cpp支持）
// ============================================================================

#ifdef USE_YAML_CPP
#include <yaml-cpp/yaml.h>

namespace basmodule {

/**
 * @brief 从YAML节点读取配置参数
 * @tparam T 参数类型
 * @param node YAML节点
 * @param key 参数键
 * @param default_value 默认值
 * @return 参数值
 */
template<typename T>
T get_param_from_yaml(const YAML::Node& node, const std::string& key, const T& default_value) {
    try {
        YAML::Node subnode = node[key];
        if (subnode.IsDefined() && !subnode.IsNull()) {
            return subnode.as<T>();
        }
    } catch (const std::exception& e) {
        // 忽略解析错误，返回默认值
    }
    return default_value;
}

} // namespace basmodule
#endif // USE_YAML_CPP

#endif // BAS_OPERATE_BAS_UTILS_HPP