#include "../include/bas_operate/file_operate.hpp"
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <algorithm>
#include <typeinfo>
#include <memory>
#include <cstring>
#include <thread>
#include <sstream>
#include <functional>
#include <regex>
#include <cstdio>

// 检查操作系统平台
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #include <process.h>
    #include <shlwapi.h>
    #pragma comment(lib, "shlwapi.lib")
    #define IS_WINDOWS 1
#else
    #define IS_WINDOWS 0
    #include <unistd.h>
    #include <limits.h>
    #include <sys/sysinfo.h>
    #include <dirent.h>      // 目录操作：DIR, opendir, readdir, closedir
    #include <sys/stat.h>    // 文件状态：stat, S_ISREG, S_IXUSR
#endif

enum class Level {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};
Level g_level = Level::WARN; // 调试等级：0-DEBUG、1-INFO、2-WARN、3-ERROR、4-FATAL

// Ubuntu 22.04 + ROS Humble 使用 GCC 11，支持完整的C++17 filesystem
#ifdef __has_include
#  if __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#    include <experimental/filesystem>
#  else
#    error "No filesystem support"
#  endif
#else
#  if __GNUC__ >= 8
#    include <filesystem>
     namespace fs = std::filesystem;
#  else
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  endif
#endif

namespace basmodule {

Level _get_level(const std::string& prefix)
{
    if (prefix.find("[DEBUG]") != std::string::npos) return Level::DEBUG;
    else if (prefix.find("[INFO]") != std::string::npos) return Level::INFO;
    else if (prefix.find("[WARN]") != std::string::npos) return Level::WARN;
    else if (prefix.find("[ERROR]") != std::string::npos) return Level::ERROR;
    else if (prefix.find("[FATAL]") != std::string::npos) return Level::FATAL;
    else return Level::ERROR;
}

std::string _get_level_prefix(Level level)
{
    switch (level)
    {
        case Level::DEBUG:
            return "[DEBUG]";
        case Level::INFO:
            return "[INFO]";
        case Level::WARN:
            return "[WARN]";
        case Level::ERROR:
            return "[ERROR]";
        case Level::FATAL:
            return "[FATAL]";
        default:
            return "[ERROR]";
    }
}

// 智能输出宏：自动检测是否包含格式化符号并选择合适的函数
#ifdef __cplusplus
#if __cplusplus >= 201703L
#define OUT_CONSOLE(level, fmt, ...) \
    do { \
        if ((int)(level) >= (int)g_level) { \
            if constexpr (contains_format_specifier(fmt)) { \
                OUT_CONSOLE_FMT(level, fmt, ##__VA_ARGS__); \
            } else { \
                output_console(_get_level_prefix(level), std::string(fmt), true, __FILE__, __FUNCTION__, __LINE__); \
            } \
        } \
    } while(0)
#else
#define OUT_CONSOLE(level, fmt, ...) \
    do { \
        if ((int)(level) >= (int)g_level) { \
            if (contains_format_specifier(fmt)) { \
                OUT_CONSOLE_FMT(level, fmt, ##__VA_ARGS__); \
            } else { \
                output_console(_get_level_prefix(level), std::string(fmt), true, __FILE__, __FUNCTION__, __LINE__); \
            } \
        } \
    } while(0)
#endif
#else
#define OUT_CONSOLE(level, fmt, ...) \
    do { \
        if ((int)(level) >= (int)g_level) { \
            if (contains_format_specifier(fmt)) { \
                OUT_CONSOLE_FMT(level, fmt, ##__VA_ARGS__); \
            } else { \
                output_console(_get_level_prefix(level), std::string(fmt), true, __FILE__, __FUNCTION__, __LINE__); \
            } \
        } \
    } while(0)
#endif

// 用于调用没有格式化字符串的output接口的宏
#define OUT_CONSOLE_STD(level, msg) \
    do { \
        if ((int)(level) >= (int)g_level) { \
            output_console(_get_level_prefix(level), std::string(msg), true, __FILE__, __FUNCTION__, __LINE__); \
        } \
    } while(0)

// 用于调用有格式化字符串的output接口的宏
#define OUT_CONSOLE_FMT(level, fmt, ...) \
    do { \
        if ((int)(level) >= (int)g_level) { \
            output_console_fmt(_get_level_prefix(level), fmt, true, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        } \
    } while(0)

// 输出消息函数，支持输出到不同的流
bool output_console(const std::string& prefix, const std::string& msg, bool bShowBasMsg, 
    const char* file, const char* func, int line)
{
    std::string output_msg = msg;
    if (output_msg.empty())
    {
        std::cout << std::endl;// 输出空行
        return true;
    }
    if (bShowBasMsg) 
    {
        std::string fileName = get_file_name(std::string(file));
        std::string base_info = "[" + fileName + ":" + std::string(func) + ":" + std::to_string(line) + "] ";
        output_msg = prefix + base_info + msg;
    }
    else 
    {
        output_msg = msg;
    }
    // 根据前缀设置颜色
    std::string color_code = "\033[0m"; // 默认重置颜色（无色）
    std::ostream* stream = &std::cout; // 默认使用cout
    if (prefix.find("[DEBUG]") != std::string::npos) {
        color_code = "\033[36m"; // 青色
    } else if (prefix.find("[INFO]") != std::string::npos) {
        color_code = "\033[32m"; // 绿色
    } else if (prefix.find("[WARN]") != std::string::npos) {
        color_code = "\033[33m"; // 黄色
    } else if (prefix.find("[ERROR]") != std::string::npos) {
        stream = &std::cerr; // 使用cerr指针
        color_code = "\033[31m"; // 红色
    } else if (prefix.find("[FATAL]") != std::string::npos) {
        stream = &std::cerr; // 使用cerr指针
        color_code = "\033[35m"; // 紫红色/洋红色
    } 
    // 如果没有匹配到任何预定义的前缀，则使用默认系统颜色（color_code 已经初始化为 "\033[0m"）
    (*stream) << color_code << output_msg << "\033[0m" << std::endl;
    return true;
}

bool output_file(const std::string& prefix, const std::string& msg, const std::string& file_path, 
    bool bShowBasMsg, const char* file, const char* func, int line)
{
    std::string output_msg = msg;
    if (output_msg.empty())
    {
        std::ofstream outfile;
        outfile.open(file_path, std::ios_base::app);
        if (!outfile.is_open()) {
            return false;
        }
        outfile << std::endl; // 输出空行
        outfile.close();
        return true;
    }
    
    if (bShowBasMsg) 
    {
        std::string fileName = get_file_name(std::string(file));
        std::string base_info = "[" + fileName + ":" + std::string(func) + ":" + std::to_string(line) + "] ";
        output_msg = prefix + base_info + msg;
    }
    else 
    {
        output_msg = prefix + msg;
    }
    
    std::ofstream outfile;
    outfile.open(file_path, std::ios_base::app);
    if (!outfile.is_open()) {
        return false;
    }
    outfile << output_msg << std::endl;
    outfile.close();
    return true;
}

// 支持格式化字符串的输出函数重载
void output_console_fmt(const std::string& prefix, const char* fmt, 
    bool bShowBasMsg, const char* file, const char* func, int line, ...) 
{
    va_list args;
    va_start(args, line);
    // 计算所需缓冲区大小
    va_list args_copy;
    va_copy(args_copy, args);
    int size = vsnprintf(nullptr, 0, fmt, args_copy);
    va_end(args_copy);
    if (size > 0) 
    {
        // 分配缓冲区并格式化字符串
        std::unique_ptr<char[]> buffer(new char[size + 1]);
        vsnprintf(buffer.get(), size + 1, fmt, args);
        output_console(prefix, std::string(buffer.get()), bShowBasMsg, file, func, line);// 调用基础版本的output函数
    }
    va_end(args);
}

// 文件输出的格式化版本
void output_file_fmt(const std::string& prefix, const std::string& file_path, const char* fmt, 
    bool bShowBasMsg, const char* file, const char* func, int line, ...) 
{
    va_list args;
    va_start(args, line);
    // 计算所需缓冲区大小
    va_list args_copy;
    va_copy(args_copy, args);
    int size = vsnprintf(nullptr, 0, fmt, args_copy);
    va_end(args_copy);
    if (size > 0) 
    {
        // 分配缓冲区并格式化字符串
        std::unique_ptr<char[]> buffer(new char[size + 1]);
        vsnprintf(buffer.get(), size + 1, fmt, args);
        output_file(prefix, std::string(buffer.get()), file_path, bShowBasMsg, file, func, line);// 调用基础版本的output函数
    }
    va_end(args);
}

// 验证是否为有效的ROS2安装目录
bool _is_valid_ros2_install_dir(const std::string& path) 
{
    if (path.empty()) return false;
    
    try 
    {
        std::filesystem::path fs_path(path);
        if (!std::filesystem::exists(fs_path) || 
            !std::filesystem::is_directory(fs_path)) {
            return false;
        }
        
        // 对于根install目录，只需要验证路径存在且为目录即可
        // 不强制要求必须包含share和lib子目录
        return true;
        
        // 如果需要更严格的验证，可以检查是否存在setup.*文件
        
        std::filesystem::path setup_bash = fs_path / "setup.bash";
        std::filesystem::path setup_sh = fs_path / "setup.sh";
        std::filesystem::path setup_zsh = fs_path / "setup.zsh";
        
        bool has_setup = (std::filesystem::exists(setup_bash) || 
                         std::filesystem::exists(setup_sh) ||
                         std::filesystem::exists(setup_zsh));
        
        return has_setup;
    } catch (const std::exception& e) {
        return false;
    }
}

// 从环境变量获取安装目录路径
std::string _get_install_path_from_env() 
{
    const char* env_vars[] = {
        "COLCON_PREFIX_PATH",  // colcon构建系统的环境变量
        "AMENT_PREFIX_PATH",   // ament构建系统的环境变量
        "ROS_DISTRO_PATH",     // 某些ROS发行版路径
        nullptr
    };
    // 首先检查COLCON_PREFIX_PATH
    const char* colcon_prefix = std::getenv("COLCON_PREFIX_PATH");
    if (colcon_prefix) 
    {
        std::string paths(colcon_prefix);
        size_t pos = paths.find(':');
        std::string first_path = (pos != std::string::npos) ? paths.substr(0, pos) : paths;
        
        // 清理路径（去除首尾空格）
        first_path.erase(0, first_path.find_first_not_of(" \t"));
        first_path.erase(first_path.find_last_not_of(" \t") + 1);
        
        OUT_CONSOLE(Level::DEBUG, "COLCON_PREFIX_PATH第一个路径: %s", first_path.c_str());
        
        // 从左到右解析路径，一旦解析到/install，则截断后面的字符内容
        size_t install_pos = first_path.find("/install");
        if (install_pos != std::string::npos) {
            std::string path = first_path.substr(0, install_pos + 8); // +8 是 "/install" 的长度
            
            OUT_CONSOLE(Level::DEBUG, "COLCON_PREFIX_PATH截断后路径: %s", path.c_str());
            
            // 使用_is_valid_ros2_install_dir进行路径校验
            if (_is_valid_ros2_install_dir(path)) 
            {
                OUT_CONSOLE(Level::DEBUG, "返回COLCON_PREFIX_PATH中的install路径: %s", path.c_str());
                return path;
            }
        }
    }
    
    // 如果COLCON_PREFIX_PATH不可用，尝试其他环境变量
    for (int i = 0; env_vars[i] != nullptr; ++i) 
    {
        const char* env_value = std::getenv(env_vars[i]);
        if (env_value) 
        {
            std::string path(env_value);
            
            OUT_CONSOLE(Level::DEBUG, "环境变量 %s 的值: %s", env_vars[i], env_value);
            
            // 处理多个路径的情况（用冒号或分号分隔）
            size_t pos = path.find_first_of(":;");
            
            if (pos != std::string::npos) {
                path = path.substr(0, pos);
            }
            
            // 清理路径（去除首尾空格）
            path.erase(0, path.find_first_not_of(" \t"));
            path.erase(path.find_last_not_of(" \t") + 1);

            OUT_CONSOLE(Level::DEBUG, "清理后的路径: %s", path.c_str());
            
            // 从左到右解析路径，一旦解析到/install，则截断后面的字符内容
            size_t install_pos = path.find("/install");
            if (install_pos != std::string::npos) 
            {
                OUT_CONSOLE(Level::DEBUG, "找到/install位置: %zu", install_pos);
                
                // 截断/install后面的内容，只保留到/install为止的路径
                std::string original_path = path;
                path = path.substr(0, install_pos + 8); // +8 是 "/install" 的长度
                
                OUT_CONSOLE(Level::DEBUG, "截断前路径: %s, 截断后路径: %s", original_path.c_str(), path.c_str());
                
                // 使用_is_valid_ros2_install_dir进行路径校验
                if (_is_valid_ros2_install_dir(path)) {
                    OUT_CONSOLE(Level::DEBUG, "返回有效的install路径: %s", path.c_str());
                    return path;
                }
            }
        }
    }
    return "";
}

std::string _get_install_path() 
{
    // 首先尝试从环境变量获取安装目录
    std::string env_install_path = _get_install_path_from_env();
    OUT_CONSOLE(Level::DEBUG, "从环境变量获取的路径: '%s'", env_install_path.c_str());
    if (!env_install_path.empty()) 
    {
        // 使用_is_valid_ros2_install_dir进行路径校验
        if (_is_valid_ros2_install_dir(env_install_path)) 
        {
            OUT_CONSOLE(Level::DEBUG, "环境变量路径验证通过: %s", env_install_path.c_str());
            return env_install_path;
        } else {
            OUT_CONSOLE(Level::WARN, "环境变量路径验证失败: %s", env_install_path.c_str());
        }
    }
    // 然后尝试从可执行文件路径推导
    std::string exe_path = get_executable_path();
    OUT_CONSOLE(Level::DEBUG, "可执行文件路径: %s", exe_path.c_str());
    if (exe_path.empty()) 
    {
        OUT_CONSOLE(Level::ERROR, "获取可执行文件路径失败");
        return "";  // 获取可执行文件路径失败
    }
    OUT_CONSOLE(Level::DEBUG, "可执行文件路径: %s", exe_path.c_str());
    std::filesystem::path exe_fs_path(exe_path);// 将可执行文件路径转换为std::filesystem::path对象
    // 通过向上回溯目录层级找到install目录
    try 
    {
        std::filesystem::path current = exe_fs_path.parent_path();
        // 增加搜索深度
        for (int i = 0; i < 15 && !current.empty(); ++i) 
        {
            // 检查当前目录是否包含lib和share子目录
            if (std::filesystem::exists(current / "share") && 
                std::filesystem::exists(current / "lib") &&
                std::filesystem::is_directory(current / "share") &&
                std::filesystem::is_directory(current / "lib")) 
            {
                
                OUT_CONSOLE(Level::DEBUG, "找到候选install路径: %s", current.string().c_str());
                // 按照用户建议的方法：从左到右解析路径，一旦解析到/install，则截断后面的字符内容
                std::string path_str = current.string();
                size_t install_pos = path_str.find("/install");
                if (install_pos != std::string::npos) 
                {
                    // 截断/install后面的内容，只保留到/install为止的路径
                    path_str = path_str.substr(0, install_pos + 8); // +8 是 "/install" 的长度
                    OUT_CONSOLE(Level::DEBUG, "截断后路径: %s", path_str.c_str());
                    
                    // 使用_is_valid_ros2_install_dir进行路径校验
                    if (_is_valid_ros2_install_dir(path_str)) {
                        OUT_CONSOLE(Level::DEBUG, "截断后路径验证通过: %s", path_str.c_str());
                        return path_str;
                    } else {
                        OUT_CONSOLE(Level::WARN, "截断后路径验证失败: %s", path_str.c_str());
                    }
                }
                OUT_CONSOLE(Level::DEBUG, "直接返回当前路径: %s", current.string().c_str());
                // 如果没有找到/install或者截断后的路径无效，则直接返回当前路径
                return current.string();
            }
            
            if (current.has_parent_path() && current != current.parent_path()) {
                current = current.parent_path();
            } else {
                break;
            }
        }
    } catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "回溯目录失败: %s", e.what());// 回溯目录失败
        return "";
    }
    OUT_CONSOLE(Level::ERROR, "未找到有效的install路径");
    return "";
}

bool _create_directory_traditional(const std::string& dir_path) 
{
    // 使用system命令创建目录
    std::string command = "mkdir -p \"" + dir_path + "\"";
    int result = system(command.c_str());
    if (result == 0) 
    {
        OUT_CONSOLE(Level::DEBUG, "目录创建成功: %s", dir_path.c_str());
        return true;
    } 
    else 
    {
        OUT_CONSOLE(Level::ERROR, "目录创建失败: %s", dir_path.c_str());
        return false;
    }
}

bool is_valid_name(const std::string& project_name) 
{
    // 检查是否为空
    if (project_name.empty()) 
    {
        OUT_CONSOLE(Level::ERROR, "项目名称为空");
        return false;
    }
    // 检查长度是否合理（1-100个字符）
    if (project_name.length() < 1 || project_name.length() > 100) {
        OUT_CONSOLE(Level::ERROR, "项目名称长度不合理: %s", project_name.c_str());
        return false;
    }
    // 检查是否包含非法字符（路径分隔符、特殊字符等）
    const std::string illegal_chars = "/\\:*?\"<>|";
    if (project_name.find_first_of(illegal_chars) != std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目名称包含非法字符: %s", project_name.c_str());
        return false;
    }
    // 检查是否以点开头或结尾（可能表示隐藏文件或目录）
    if (project_name.front() == '.' || project_name.back() == '.') {
        OUT_CONSOLE(Level::ERROR, "项目名称以点开头或结尾: %s", project_name.c_str());
        return false;
    }
    // 检查是否包含连续的点（可能表示路径遍历）
    if (project_name.find("..") != std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目名称包含连续的点: %s", project_name.c_str());
        return false;
    }
    // 检查是否只包含空格
    if (project_name.find_first_not_of(" \t\n\r") == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目名称只包含空格: %s", project_name.c_str());
        return false;
    }
    OUT_CONSOLE(Level::DEBUG, "项目名称有效: %s", project_name.c_str());
    return true;
}

bool create_directory_safe(const std::string& dir_path) 
{
    try 
    {
        OUT_CONSOLE(Level::DEBUG, "尝试创建目录: %s", dir_path.c_str());
        if (fs::exists(dir_path))// 首先尝试使用filesystem库 
        {
            OUT_CONSOLE(Level::DEBUG, "目录已存在: %s", dir_path.c_str());
            return true;
        }
        bool created = fs::create_directories(dir_path);
        if (created) 
        {
            OUT_CONSOLE(Level::DEBUG, "目录创建成功: %s", dir_path.c_str());
            return true;
        }
    } 
    catch (const std::exception& e) 
    {
        OUT_CONSOLE(Level::ERROR, "创建目录失败: %s", e.what());
    }
    OUT_CONSOLE(Level::WARN, "创建目录失败，使用传统方式创建: %s", dir_path.c_str());
    return _create_directory_traditional(dir_path);// 如果filesystem库失败，使用传统方式
}

/**
 * @brief 清理指定目录中的文件
 * 
 * 根据指定的模式删除目录中的文件：
 * - 检查目录是否存在
 * - 遍历目录中的所有条目
 * - 删除符合模式匹配的文件
 * 
 * @param dir_path 目录路径
 * @param pattern  文件模式（扩展名匹配）
 */
void clear_directory(const std::string& dir_path, const std::string& pattern) 
{
    try 
    {
        // 检查目录是否存在
        if (!fs::exists(dir_path)) {
            OUT_CONSOLE(Level::DEBUG, "目录不存在: %s", dir_path.c_str());
            return;
        }
        // 遍历目录中的所有条目
        for (const auto& entry : fs::directory_iterator(dir_path)) 
        {
            // 只处理常规文件，跳过子目录
            if (entry.is_regular_file()) 
            {
                // 获取文件名和扩展名
                std::string filename = entry.path().filename().string();
                std::string ext = entry.path().extension().string();

                // 检查扩展名是否匹配指定模式
                if (pattern.find(ext) != std::string::npos) 
                {
                    fs::remove(entry.path());// 删除匹配的文件
                }
            }
        }
        OUT_CONSOLE(Level::DEBUG, "目录清理完成: %s", dir_path.c_str());
    }
    catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "清理目录失败: %s", e.what());
    }
}

std::string get_file_name(const std::string& file_path)
{
    size_t lastSlash = file_path.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        return file_path.substr(lastSlash + 1);
    }
    OUT_CONSOLE(Level::DEBUG, "带后缀的文件名: %s", file_path.c_str());
    return file_path;
}

std::string get_file_name_no_ext(const std::string& file_path) 
{
    std::string full_name = get_file_name(file_path);
    size_t last_dot = full_name.find_last_of(".");
    if (last_dot != std::string::npos) {
        return full_name.substr(0, last_dot);
    }
    OUT_CONSOLE(Level::DEBUG, "不带后缀的文件名: %s", full_name.c_str());
    return full_name;
}

//获取当前线程ID
uint64_t get_thread_id()
{
    std::thread::id tid = std::this_thread::get_id();
    OUT_CONSOLE(Level::DEBUG, "当前线程ID: %s", get_thread_id_string().c_str());
    // 将 thread::id 转换为 uint64_t
    // 使用 std::hash 来生成唯一的哈希值
    return static_cast<uint64_t>(std::hash<std::thread::id>{}(tid));
}

//获取当前线程ID的字符串表示
std::string get_thread_id_string()
{
    std::thread::id tid = std::this_thread::get_id();
    // 将 thread::id 转换为字符串
    std::ostringstream oss;
    oss << tid;
    return oss.str();
}

// 实现检查路径是否存在的函数
bool is_path_exists(const std::string& path) 
{
    return fs::exists(path);
}

/**
 * @brief 解析路径中的波浪号(~)为用户主目录
 * @param path 待解析的路径字符串
 * @return 解析后的路径字符串，如果包含波浪号则替换为实际的主目录路径，否则返回原路径
 * @details 该函数用于将路径中的波浪号(~)展开为实际的用户主目录路径
 *          例如：输入"~/ros2_logs/"，当前用户名为user，则输出"/home/user/ros2_logs/"
 *          如果输入路径不包含波浪号，则按原路径返回
 */
std::string expand_path(const std::string& path)
{
    if (path.empty() || path[0] != '~') {
        OUT_CONSOLE(Level::DEBUG, "路径为空或不以波浪号开头: %s", path.c_str());
        // 如果路径为空或不以波浪号开头，则直接返回原路径
        return path;
    }
    std::string expanded_path = path;
    // 查找环境变量HOME的值作为主目录
    const char* home_env = std::getenv("HOME");
    if (home_env != nullptr) 
    {
        std::string home_dir = std::string(home_env);
        
        // 替换路径中的波浪号
        if (path.length() == 1) {
            // 如果路径就是单独的波浪号，直接返回主目录
            expanded_path = home_dir;
        } else if (path[1] == '/') {
            // 如果波浪号后跟路径分隔符，替换波浪号部分
            expanded_path = home_dir + path.substr(1);
        } else {
            // 如果波浪号后跟其他字符（如~username），按照要求自动处理为合法路径，即在波浪号后添加'/'
            expanded_path = home_dir + "/" + path.substr(1);
        }
        OUT_CONSOLE(Level::DEBUG, "展开后的路径: %s", expanded_path.c_str());
    }
    else {
        OUT_CONSOLE(Level::ERROR, "无法展开路径: %s", path.c_str());
    }
    return expanded_path;
}

// 实现路径拼接函数
std::string join_path(const std::vector<std::string>& paths) 
{
    if (paths.empty()) {
        OUT_CONSOLE(Level::DEBUG, "路径向量为空");
        return "";
    }
    std::string result;
    for (size_t i = 0; i < paths.size(); ++i) 
    {
        if (paths[i].empty()) {
            continue;
        }
        std::string path_segment = paths[i];
        // 规范化路径段：去除前导和尾随斜杠（特殊情况除外）
        // 处理第一个路径段
        if (i == 0) 
        {
            // 对于第一个路径段，如果是绝对路径（以/开头但不是只有/），保留斜杠
            if (path_segment.length() > 1 && path_segment[0] == '/') 
            {
                // 移除开头多余的斜杠，保留一个
                size_t start = 0;
                while (start < path_segment.length() && path_segment[start] == '/') {
                    start++;
                }
                // 至少保留一个斜杠
                if (start > 1) {
                    path_segment = "/" + path_segment.substr(start);
                }
                result = path_segment;
            } 
            else 
            {
                // 移除开头多余的斜杠
                size_t start = 0;
                while (start < path_segment.length() && path_segment[start] == '/') {
                    start++;
                }
                result = path_segment.substr(start);
            }
        } 
        else 
        {
            // 处理后续路径段：移除前导斜杠
            size_t start = 0;
            while (start < path_segment.length() && path_segment[start] == '/') {
                start++;
            }
            path_segment = path_segment.substr(start);
            // 如果当前结果不为空且不以/结尾，则添加/
            if (!result.empty()) {
                if (result[result.length()-1] != '/') {
                    result += "/";
                }
            }
            result += path_segment;
        }
    }
    // 确保最终结果不以斜杠结尾（除非是根目录）
    if (result.length() > 1 && result[result.length()-1] == '/') {
        result = result.substr(0, result.length()-1);
    } 
    OUT_CONSOLE(Level::DEBUG, "拼接后的路径: %s", result.c_str());
    return result;
}

//获取可执行文件的完整路径
//例如输出可执行文件路径: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
std::string get_executable_path() 
{
#if IS_WINDOWS
    // Windows平台实现
    char buffer[MAX_PATH];
    DWORD len = GetModuleFileNameA(NULL, buffer, MAX_PATH);
    if (len == 0 || len == MAX_PATH) {
        OUT_CONSOLE(Level::ERROR, "获取可执行文件路径失败: GetModuleFileNameA 返回 0");
        return "";  // 获取失败
    }
    return std::string(buffer);
#else
    // Linux/Unix平台实现
    char buffer[PATH_MAX];
    memset(buffer, 0, PATH_MAX);  // 初始化缓冲区
    ssize_t len = readlink("/proc/self/exe", buffer, PATH_MAX - 1);
    if (len == -1) {
        OUT_CONSOLE(Level::ERROR, "获取可执行文件路径失败: readlink 返回 -1");
        return "";  // 获取失败
    }
    buffer[len] = '\0';
    return std::string(buffer);
#endif
}

//获取进程ID(PID)
//例如输出进程ID(PID): 87243
bool get_process_id(pid_t* pid) 
{
    *pid = getpid();
    if (*pid <= 0) {
        OUT_CONSOLE(Level::ERROR, "获取进程ID失败: getpid() 返回无效值");
        return false;
    }
    return true;// getpid() 在Linux上几乎总是成功
}

//获取当前工作目录
//例如输出当前工作目录: /home/user/testCode/project
std::string get_current_working_directory() 
{
    char buffer[PATH_MAX];
    if (getcwd(buffer, PATH_MAX) != nullptr) {
        OUT_CONSOLE(Level::DEBUG, "当前工作目录: %s", buffer);
        return std::string(buffer);
    }
    OUT_CONSOLE(Level::ERROR, "获取当前工作目录失败: getcwd 返回 nullptr");
    return "";
}

//获取启动命令
//例如输出启动命令: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
std::string get_startup_command(pid_t pid) 
{
    std::string cmdline_path = "/proc/" + std::to_string(pid) + "/cmdline";
    FILE* cmdline_file = fopen(cmdline_path.c_str(), "r");
    if (!cmdline_file) {
        OUT_CONSOLE(Level::ERROR, "打开启动命令文件失败: %s", cmdline_path.c_str());
        return "";
    }
    char buffer[4096];
    size_t size = fread(buffer, 1, sizeof(buffer) - 1, cmdline_file);
    fclose(cmdline_file);
    if (size > 0) 
    {
        buffer[size] = '\0';
        std::string cmdline = std::string(buffer, size);
        // 将空字符替换为空格,便于阅读
        for (size_t i = 0; i < cmdline.size(); i++) 
        {
            if (cmdline[i] == '\0') {
                cmdline[i] = ' ';
            }
        }
        OUT_CONSOLE(Level::DEBUG, "启动命令: %s", cmdline.c_str());
        return cmdline;
    }
    OUT_CONSOLE(Level::ERROR, "读取启动命令文件失败: %s", cmdline_path.c_str());
    return "";
}

// 添加获取指定进程创建时间的函数
std::time_t get_process_create_time(pid_t pid)
{
    static const long HZ = sysconf(_SC_CLK_TCK);
    if (HZ <= 0) 
    {
        OUT_CONSOLE(Level::ERROR, "获取系统时钟频率失败: sysconf(_SC_CLK_TCK) 返回 %ld", HZ);
        return 0;  // 无法获取时钟频率
    }
    // 读取指定进程的stat文件
    std::string stat_path = "/proc/" + std::to_string(pid) + "/stat";
    std::ifstream stat_file(stat_path);
    if (!stat_file) 
    {
        OUT_CONSOLE(Level::ERROR, "打开进程状态文件失败: %s", stat_path.c_str());
        return 0;
    }
    std::string line;
    if (!std::getline(stat_file, line)) 
    {
        OUT_CONSOLE(Level::ERROR, "读取进程状态文件失败: getline 操作失败");
        return 0;
    }
    // 解析stat文件获取starttime字段
    std::istringstream iss(line);
    std::string token;
    // stat文件格式：pid (comm) state ppid pgrp session tty_nr tpgid flags ...
    // 需要第22个字段（从1开始计数）
    for (int i = 1; i <= 22; ++i) 
    {
        if (!(iss >> token)) 
        {
            OUT_CONSOLE(Level::ERROR, "解析进程状态文件失败: 字段数量不足，无法获取第22个字段");
            return 0;  // 字段不足
        }
    }
    try 
    {
        unsigned long long start_ticks = std::stoull(token);
        // 获取系统启动时间
        struct sysinfo info;
        if (sysinfo(&info) != 0) {
            OUT_CONSOLE(Level::ERROR, "获取系统信息失败: sysinfo 操作失败");
            return 0;
        }
        
        // 计算进程启动时间
        // 1. 当前时间 - 系统运行时间 = 系统启动时间
        // 2. 系统启动时间 + (进程启动时钟滴答数 / 时钟频率) = 进程启动时间
        std::time_t current_time = std::time(nullptr);
        if (current_time == static_cast<std::time_t>(-1)) {
            OUT_CONSOLE(Level::ERROR, "获取当前时间失败");
            return 0;  // 获取当前时间失败
        }
        std::time_t boot_time = current_time - info.uptime;
        std::time_t process_start_time = boot_time + (start_ticks / HZ);
        OUT_CONSOLE(Level::DEBUG, "进程启动时间: %s", std::ctime(&process_start_time));
        return process_start_time;  
    } catch (const std::exception& e) {
        // 转换失败
        OUT_CONSOLE(Level::ERROR, "转换进程启动时间失败: %s", e.what());
        return 0;
    }
}

// 添加获取进程创建时间的函数（默认获取当前进程）
std::time_t get_process_create_time() 
{
    return get_process_create_time(getpid());  // 调用重载函数，传入当前进程PID
}

// 将time_t转换为字符串格式，可选择是否包含日期
std::string format_time_to_string(std::time_t time_val, bool include_date /*= true*/)
{
    if (time_val == 0) {
        OUT_CONSOLE(Level::ERROR, "时间值为0，无法格式化");
        return "";
    }
    char time_str[100];
    struct tm* tm_info = std::localtime(&time_val);
    if (!tm_info) 
    {
        OUT_CONSOLE(Level::ERROR, "转换时间失败: localtime 返回 nullptr");
        return "";
    }
    if (include_date) {
        // 包含日期和时间
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d_%H:%M:%S", tm_info);
    } else {
        // 只包含时间，不包含日期
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_info);
    }
    OUT_CONSOLE(Level::DEBUG, "格式化后的时间: %s", time_str);
    return std::string(time_str);
}

// 添加将进程时间戳转换为字符串的函数
std::string format_process_create_time() 
{
    std::time_t start_time = get_process_create_time();
    if (start_time == 0) {
        OUT_CONSOLE(Level::ERROR, "获取进程启动时间失败: get_process_create_time 返回 0");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "进程启动时间: %s", std::ctime(&start_time));
    return format_time_to_string(start_time, true);  // 复用 format_time_to_string 函数
}

// 添加将指定进程PID的时间戳转换为字符串的函数重载
std::string format_process_create_time(pid_t pid) 
{
    std::time_t start_time = get_process_create_time(pid);
    if (start_time == 0) {
        OUT_CONSOLE(Level::ERROR, "获取进程启动时间失败: get_process_create_time 返回 0, PID: %d", pid);
        return "";
    }
    return format_time_to_string(start_time, true);  // 复用 format_time_to_string 函数
}

//获取当前进程的详细路径信息
std::string get_current_process_info() 
{
    std::string exe_path;
    std::string cwd_path;
    pid_t pid;
    std::string cmdline;
    std::string create_time;
    std::string info;

    // 1. 获取可执行文件的完整路径
    exe_path = get_executable_path();
    if (!exe_path.empty()) 
    {
        info += "可执行文件路径: " + exe_path;
        OUT_CONSOLE(Level::DEBUG, "可执行文件路径: %s", exe_path.c_str());
    } else {
        OUT_CONSOLE(Level::ERROR, "获取可执行文件路径失败");
    }

    // 2. 获取当前工作目录
    cwd_path = get_current_working_directory();
    if (!cwd_path.empty()) 
    {
        info += "当前工作目录: " + cwd_path;
        OUT_CONSOLE(Level::DEBUG, "当前工作目录: %s", cwd_path.c_str());
    } else {
        OUT_CONSOLE(Level::ERROR, "获取当前工作目录失败");
    }

    // 3. 获取进程ID
    if (get_process_id(&pid)) {
        info += "进程ID(PID): " + std::to_string(pid);
        OUT_CONSOLE(Level::DEBUG, "进程ID(PID): %d", pid);
    } else {
        OUT_CONSOLE(Level::ERROR, "获取进程ID失败");
    }

    // 4. 获取命令行参数
    cmdline = get_startup_command(pid);
    if (!cmdline.empty()) {
        info += "启动命令: " + cmdline;
        OUT_CONSOLE(Level::DEBUG, "启动命令: %s", cmdline.c_str());
    } else {
        OUT_CONSOLE(Level::ERROR, "获取启动命令失败");
    }

    // 5. 获取进程创建时间
    std::time_t proc_start_time = get_process_create_time(pid);
    if (proc_start_time != 0) 
    {
        create_time = format_time_to_string(proc_start_time);
        if (!create_time.empty()) 
        {
            info += "进程创建时间: " + create_time;
            OUT_CONSOLE(Level::DEBUG, "进程创建时间: %s", create_time.c_str());
        } else {
            OUT_CONSOLE(Level::ERROR, "格式化进程创建时间失败");
        }
    } else {
        OUT_CONSOLE(Level::ERROR, "获取进程创建时间失败");
    }
    OUT_CONSOLE(Level::DEBUG, "进程信息: %s", info.c_str());
    return info;
}

/**
 * @brief 获取进程项目所在目录
 * @param process_project_name 输出参数，用于存储项目名称
 * @return 项目目录路径，失败则返回空字符串
 * @details 基于可执行文件路径构建项目目录
 *          例如: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
 *          构建结果: /home/user/testCode/project/install/log_demo
 */
std::string get_process_project_name(std::string& process_project_name)
{
    process_project_name.clear();
    std::string exe_path = get_executable_path();
    if (exe_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "获取可执行文件路径失败，无法构建项目目录");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "可执行文件路径: %s", exe_path.c_str());
    // 查找 "/install/" 在路径中的位置
    size_t install_pos = exe_path.find("/install/");
    if (install_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "路径 %s 中未找到install目录", exe_path.c_str());
        return "";
    }
    // 从 "install/" 后面开始
    size_t start_pos = install_pos + strlen("/install/");
    // 查找 install/ 后面的第一个 "/"
    size_t end_pos = exe_path.find("/", start_pos);
    if (end_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "路径格式错误，未找到项目名后的分隔符: %s", exe_path.c_str());
        return "";
    }
    // 提取 process_project_name (install/ 后面到第一个 "/" 的内容)
    process_project_name = exe_path.substr(start_pos, end_pos - start_pos);
    // 检查提取的 process_project_name 是否为空
    if (process_project_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "提取的process_project_name为空: %s", exe_path.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目名称: %s", process_project_name.c_str());
    // 直接构建项目目录路径: install_path + process_project_name
    std::string process_project_dir = exe_path.substr(0, install_pos + strlen("/install/")) + process_project_name;
    if (!is_path_exists(process_project_dir)) // 检查路径是否存在
    {
        OUT_CONSOLE(Level::ERROR, "项目目录不存在: %s", process_project_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目目录: %s", process_project_dir.c_str());
    return process_project_dir;
}

//获取安装目录
//例如输出安装目录: /home/user/testCode/project/install/
std::string get_install_dir() 
{
    std::string project_name;
    std::string project_dir = get_process_project_name(project_name);
    if (project_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "获取项目目录失败，无法构建安装目录");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目目录: %s", project_dir.c_str());
    OUT_CONSOLE(Level::DEBUG, "获取到当前进程项目目录: %s", project_dir.c_str());
    // 从项目目录中移除项目名称部分，得到安装目录
    size_t install_pos = project_dir.find("/install/");
    if (install_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目目录中未找到install目录: %s", project_dir.c_str());
        return "";
    }
    std::string install_dir = project_dir.substr(0, install_pos + strlen("/install/"));
    if (!is_path_exists(install_dir)) // 检查路径是否存在
    {
        OUT_CONSOLE(Level::ERROR, "安装目录不存在: %s", install_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "获取到当前进程安装目录: %s", install_dir.c_str());
    return install_dir;
}

//获取项目目录
//例如输出项目目录: /home/user/testCode/project/install/log_demo
std::string get_project_dir(const std::string& project_name)
{
    std::string install_dir = get_install_dir();
    if (install_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "获取安装目录失败，无法构建项目目录");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "获取到当前install目录: %s", install_dir.c_str());
    std::string project_dir = install_dir + project_name; // 构建项目目录路径: install_dir + project_name
    if (!is_path_exists(project_dir)) // 检查路径是否存在
    {
        OUT_CONSOLE(Level::ERROR, "项目目录不存在: %s", project_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "获取到当前 %s 项目的安装目录: %s", project_name.c_str(), project_dir.c_str());
    return project_dir;
}

//根据项目目录获取项目名称
//例如输入 /home/user/testCode/project/install/log_demo
//例如输出项目名称: log_demo
std::string get_project_name_by_project_dir(const std::string& project_dir) 
{
    // 检查项目目录路径是否有效
    if (project_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "项目目录路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目目录: %s", project_dir.c_str());
    // 查找 "/install/" 在路径中的位置
    size_t install_pos = project_dir.find("/install/");
    if (install_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "路径 %s 中未找到install目录", project_dir.c_str());
        return "";
    }
    // 从 "install/" 后面开始
    size_t start_pos = install_pos + strlen("/install/");
    // 查找 install/ 后面的第一个 "/" 或字符串末尾
    size_t end_pos = project_dir.find("/", start_pos);
    if (end_pos == std::string::npos) {
        // 如果没有找到更多的"/"，则整个剩余部分都是项目名称
        end_pos = project_dir.length();
    }
    // 提取 project_name (install/ 后面到第一个 "/" 或字符串末尾的内容)
    std::string project_name = project_dir.substr(start_pos, end_pos - start_pos);
    // 检查提取的 project_name 是否为空
    if (project_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "提取的project_name为空: %s", project_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目名称: %s", project_name.c_str());
    return project_name;
}

// 获取项目配置文件所在目录
//例如输入: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
//输出结果: /home/user/testCode/project/install/log_demo/share/log_demo/config
std::string get_project_config_dir(const std::string& project_dir)
{
    // 获取项目名称
    std::string project_name = get_project_name_by_project_dir(project_dir);
    if (project_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "项目路径：%s 获取项目名称失败，无法构建config目录", project_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目名称: %s", project_name.c_str());
    OUT_CONSOLE(Level::DEBUG, "项目目录: %s", project_dir.c_str());
    std::string project_config_dir = project_dir + "/share/" + project_name + "/config";
    OUT_CONSOLE(Level::DEBUG, "项目配置目录: %s", project_config_dir.c_str());
    return project_config_dir;
}

//获取项目配置文件路径
//例如输入: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
//输出配置文件路径: /home/user/testCode/project/install/log_demo/share/log_demo/config/log_config.yaml
std::string get_project_config_file_path(const std::string& project_dir, const std::string& file_name) 
{
    std::string config_dir = get_project_config_dir(project_dir);
    if (config_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "项目路径：%s 获取项目配置目录失败，无法构建配置文件路径", project_dir.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "配置文件目录: %s", config_dir.c_str());
    std::string config_file_path = config_dir + "/" + file_name;
    if (!is_path_exists(config_file_path)) // 检查文件路径是否存在
    {
        OUT_CONSOLE(Level::ERROR, "配置文件不存在: %s", config_file_path.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "配置文件路径: %s", config_file_path.c_str());
    return config_file_path;
}

/**
 * @brief 根据项目配置文件路径获取项目名称
 * @param project_config_file_path 项目配置文件的完整路径
 * @param[out] project_name 输出参数，用于存储提取出的项目名称
 * @return 项目路径，如果解析失败则返回空字符串
 * @details 从项目配置文件路径中解析出项目名称，并返回项目的基本路径
 *          例如输入: /home/user/testCode/project/install/log_demo/share/log_demo/config/log_config.yaml
 *          输出project_name: log_demo，返回项目路径: /home/user/testCode/project/install/log_demo
 */
std::string get_project_name_by_config_file_path(const std::string& project_config_file_path, std::string& project_name)
{
    // 检查项目配置文件路径是否有效
    if (project_config_file_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "项目配置文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目配置文件路径: %s", project_config_file_path.c_str());
    // 查找 "/install/" 在路径中的位置
    size_t install_pos = project_config_file_path.find("/install/");
    if (install_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目配置文件路径 %s 中未找到install目录", project_config_file_path.c_str());
        return "";
    }
    // 从 "install/" 后面开始
    size_t start_pos = install_pos + strlen("/install/");
    // 查找 install/ 后面的第一个 "/" 
    size_t end_pos = project_config_file_path.find("/", start_pos);
    if (end_pos == std::string::npos) {
        OUT_CONSOLE(Level::ERROR, "项目配置文件路径 %s 格式错误，未找到项目名后的分隔符", project_config_file_path.c_str());
        return "";
    }
    // 提取 project_name (install/ 后面到第一个 "/" 的内容)
    project_name = project_config_file_path.substr(start_pos, end_pos - start_pos);
    // 检查提取的 project_name 是否为空
    if (project_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "从项目配置文件路径 %s 提取的project_name为空", project_config_file_path.c_str());
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "项目名称为: %s", project_name.c_str());
    // 构建并返回项目路径: 从开始到install/ + 项目名称
    std::string project_path = project_config_file_path.substr(0, install_pos + strlen("/install/")) + project_name;
    if (!is_path_exists(project_path)) // 检查路径是否存在
    {
        OUT_CONSOLE(Level::ERROR, "项目路径 %s 不存在", project_path.c_str());
        return "";
    }  
    OUT_CONSOLE(Level::DEBUG, "项目路径为: %s", project_path.c_str());
    return project_path;
}

std::string get_cmake_list_file_path(const std::string& file_name_path)
{
    // 首先验证输入参数是否有效
    if (file_name_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "输入文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "输入文件路径: %s", file_name_path.c_str());
    try 
    {
        std::filesystem::path file_path(file_name_path);
        std::filesystem::path current_dir = file_path.parent_path();// 获取文件所在的目录
        OUT_CONSOLE(Level::DEBUG, "文件所在目录: %s", current_dir.c_str());
        // 向上遍历目录直到找到CMakeLists.txt文件或到达根目录
        while (current_dir != current_dir.parent_path()) 
        {
            std::filesystem::path cmake_file = current_dir / "CMakeLists.txt";
            // 检查当前目录是否包含CMakeLists.txt文件
            if (std::filesystem::exists(cmake_file) && std::filesystem::is_regular_file(cmake_file)) 
            {
                OUT_CONSOLE(Level::DEBUG, "找到CMakeLists.txt文件: %s", cmake_file.c_str());
                return cmake_file.string();
            }
            // 移动到父目录继续搜索
            current_dir = current_dir.parent_path();
        }
        // 如果没有找到CMakeLists.txt文件，返回空字符串
        OUT_CONSOLE(Level::ERROR, "基于路径 %s 未能找到CMakeLists.txt文件", file_name_path.c_str());
        return "";
    } catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "搜索CMakeLists.txt文件时发生异常: %s", e.what());
        return "";
    }
}

std::string get_package_xml_file_path(const std::string& file_name_path)
{
    // 首先验证输入参数是否有效
    if (file_name_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "输入文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "输入文件路径: %s", file_name_path.c_str());
    try 
    {
        std::filesystem::path file_path(file_name_path);
        std::filesystem::path current_dir = file_path.parent_path();// 获取文件所在的目录
        OUT_CONSOLE(Level::DEBUG, "文件所在目录: %s", current_dir.c_str());
        while (current_dir != current_dir.parent_path()) // 向上遍历目录直到找到package.xml文件或到达根目录
        {
            std::filesystem::path package_file = current_dir / "package.xml";
            // 检查当前目录是否包含package.xml文件
            if (std::filesystem::exists(package_file) && std::filesystem::is_regular_file(package_file)) 
            {
                OUT_CONSOLE(Level::DEBUG, "找到package.xml文件: %s", package_file.c_str());
                return package_file.string();
            }
            current_dir = current_dir.parent_path();// 移动到父目录继续搜索
        }
        OUT_CONSOLE(Level::ERROR, "基于路径 %s 未能找到package.xml文件", file_name_path.c_str());
        return "";
    } catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "搜索package.xml文件时发生异常: %s", e.what());
        return "";
    }
}

std::string get_project_name_by_cmake_list_file(const std::string& cmake_list_file_path)
{
    // 首先验证输入参数是否有效
    if (cmake_list_file_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "CMakeLists.txt文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "CMakeLists.txt文件路径: %s", cmake_list_file_path.c_str());
    try 
    {
        if (!std::filesystem::exists(cmake_list_file_path)) // 检查CMakeLists.txt文件是否存在
        {
            OUT_CONSOLE(Level::ERROR, "CMakeLists.txt文件不存在: %s", cmake_list_file_path.c_str());
            return "";
        }
        std::ifstream file(cmake_list_file_path);// 打开并读取CMakeLists.txt文件
        if (!file.is_open()) 
        {
            OUT_CONSOLE(Level::ERROR, "无法打开CMakeLists.txt文件: %s", cmake_list_file_path.c_str());
            return "";
        }
        std::string line;
        // 正则表达式模式用于匹配project()命令
        // 支持多种格式如: project(MyProject), project(MyProject VERSION 1.0.0), etc.
        std::regex project_regex(R"(^\s*project\s*\(\s*([a-zA-Z0-9_-]+))", std::regex_constants::icase);
        while (std::getline(file, line)) 
        {
            std::smatch match;
            if (std::regex_search(line, match, project_regex)) 
            {
                if (match.size() > 1) {
                    std::string project_name = match[1].str();
                    // 验证项目名称是否有效
                    if (is_valid_name(project_name)) 
                    {
                        OUT_CONSOLE(Level::DEBUG, "解析到的项目名称有效: %s", project_name.c_str());
                        return project_name;
                    } else {
                        OUT_CONSOLE(Level::WARN, "解析到的项目名称无效: %s", project_name.c_str());
                    }
                }
            }
        }
        OUT_CONSOLE(Level::ERROR, "在CMakeLists.txt中未找到有效的project()命令: %s", cmake_list_file_path.c_str());
        return "";
    } catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "解析CMakeLists.txt时发生异常: %s", e.what());
        return "";
    }
}

std::string get_project_name_by_package_xml_file(const std::string& package_xml_file_path)
{
    // 首先验证输入参数是否有效
    if (package_xml_file_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "package.xml文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "package.xml文件路径: %s", package_xml_file_path.c_str());
    try 
    {
        // 检查package.xml文件是否存在
        if (!std::filesystem::exists(package_xml_file_path)) {
            OUT_CONSOLE(Level::ERROR, "package.xml文件不存在: %s", package_xml_file_path.c_str());
            return "";
        }
        // 打开并读取package.xml文件
        std::ifstream file(package_xml_file_path);
        if (!file.is_open()) {
            OUT_CONSOLE(Level::ERROR, "无法打开package.xml文件: %s", package_xml_file_path.c_str());
            return "";
        }
        std::string line;
        // 正则表达式模式用于匹配<name>标签
        std::regex name_regex("<name>([^<]*)</name>", std::regex_constants::icase);
        while (std::getline(file, line)) 
        {
            std::smatch match;
            if (std::regex_search(line, match, name_regex)) 
            {
                if (match.size() > 1) 
                {
                    std::string project_name = match[1].str();
                    // 验证项目名称是否有效
                    if (is_valid_name(project_name)) 
                    {
                        OUT_CONSOLE(Level::DEBUG, "解析到的项目名称有效: %s", project_name.c_str());
                        return project_name;
                    } else {
                        OUT_CONSOLE(Level::WARN, "解析到的项目名称无效: %s", project_name.c_str());
                    }
                }
            }
        }
        OUT_CONSOLE(Level::ERROR, "在package.xml中未找到有效的<name>标签: %s", package_xml_file_path.c_str());
        return "";
    } catch (const std::exception& e) {
        OUT_CONSOLE(Level::ERROR, "解析package.xml时发生异常: %s", e.what());
        return "";
    }
}

std::string get_project_name_by_file_path(const std::string& file_name_path)
{
    // 首先验证输入参数是否有效
    if (file_name_path.empty()) {
        OUT_CONSOLE(Level::ERROR, "输入文件路径为空");
        return "";
    }
    OUT_CONSOLE(Level::DEBUG, "输入文件路径: %s", file_name_path.c_str());
    
    // 1) 先获取安装目录
    std::string install_dir = get_install_dir();
    if (install_dir.empty()) 
    {
        OUT_CONSOLE(Level::WARN, "获取安装目录失败，尝试从可执行文件路径获取项目名称");
        std::string process_project_name;
        get_process_project_name(process_project_name);
        return process_project_name;
    }
    OUT_CONSOLE(Level::DEBUG, "安装目录: %s", install_dir.c_str());
    
    // 2) 查找从左往右数的第1个 /src
    size_t src_pos = file_name_path.find("/src/");
    if (src_pos == std::string::npos) 
    {
        OUT_CONSOLE(Level::WARN, "路径中未找到 /src/，尝试从可执行文件路径获取项目名称");
        std::string process_project_name;
        get_process_project_name(process_project_name);
        return process_project_name;
    }
    
    // 提取 /src/ 后面的路径部分
    std::string after_src = file_name_path.substr(src_pos + 5); // 跳过 "/src/"
    OUT_CONSOLE(Level::DEBUG, "/src/ 后的路径: %s", after_src.c_str());
    
    // 尝试从路径中提取项目名称
    // 查找第1个 / 的位置
    size_t first_slash = after_src.find('/');
    std::string first_candidate;
    std::string second_candidate;
    
    if (first_slash != std::string::npos) 
    {
        // 提取第1个路径段
        first_candidate = after_src.substr(0, first_slash);
        OUT_CONSOLE(Level::DEBUG, "第1个候选项目名称: %s", first_candidate.c_str());
        
        // 提取第1个 / 后的剩余路径
        std::string after_first_slash = after_src.substr(first_slash + 1);
        
        // 查找第2个 / 的位置
        size_t second_slash = after_first_slash.find('/');
        if (second_slash != std::string::npos) 
        {
            // 提取第2个路径段
            second_candidate = after_first_slash.substr(0, second_slash);
            OUT_CONSOLE(Level::DEBUG, "第2个候选项目名称: %s", second_candidate.c_str());
        }
    } 
    else 
    {
        // 如果没有 /，则整个 after_src 就是项目名称
        first_candidate = after_src;
        OUT_CONSOLE(Level::DEBUG, "只有1个候选项目名称: %s", first_candidate.c_str());
    }
    
    // 优先检查第2个候选项目名称（如果存在）
    if (!second_candidate.empty()) 
    {//例如传入的file_name_path为/home/user/testCode/dev/src/log/log_system/src/src/log_utils.cpp，提取的项目名称为log_system
        std::string project_dir = install_dir + second_candidate;
        OUT_CONSOLE(Level::DEBUG, "检查第2个候选项目目录是否存在: %s", project_dir.c_str());
        if (is_path_exists(project_dir)) {
            OUT_CONSOLE(Level::DEBUG, "项目目录存在，解析成功: %s", second_candidate.c_str());
            return second_candidate;
        }
        OUT_CONSOLE(Level::INFO, "第2个候选项目目录不存在: %s", project_dir.c_str());
    }
    
    // 检查第1个候选项目名称
    if (!first_candidate.empty()) 
    {//例如传入的file_name_path为/home/user/testCode/dev/src/bas_control/src/status_monitor.cpp，提取的项目名称为bas_control
        std::string project_dir = install_dir + first_candidate;
        OUT_CONSOLE(Level::DEBUG, "检查第1个候选项目目录是否存在: %s", project_dir.c_str());
        if (is_path_exists(project_dir)) {
            OUT_CONSOLE(Level::DEBUG, "项目目录存在，解析成功: %s", first_candidate.c_str());
            return first_candidate;
        }
        OUT_CONSOLE(Level::INFO, "第1个候选项目目录不存在: %s", project_dir.c_str());
    }
    
    // 3) 第3个候选：从右往左查找第一个"/src"，提取其左侧的路径段作为候选
    {
        std::string third_candidate;
        size_t right_src_pos = file_name_path.rfind("/src");
        if (right_src_pos != std::string::npos && right_src_pos > 0) {
            // 从"/src"前一个位置开始往左找第一个"/"
            size_t left_slash_pos = file_name_path.rfind('/', right_src_pos - 1);
            if (left_slash_pos != std::string::npos) {
                third_candidate = file_name_path.substr(left_slash_pos + 1, right_src_pos - left_slash_pos - 1);
            } else {
                // 如果左边没有"/"，则取从头到"/src"的部分
                third_candidate = file_name_path.substr(0, right_src_pos);
            }
            OUT_CONSOLE(Level::DEBUG, "第3个候选项目名称: %s", third_candidate.c_str());
            
            if (!third_candidate.empty()) {
                std::string project_dir = install_dir + third_candidate;
                OUT_CONSOLE(Level::DEBUG, "检查第3个候选项目目录是否存在: %s", project_dir.c_str());
                if (is_path_exists(project_dir)) {
                    OUT_CONSOLE(Level::DEBUG, "项目目录存在，解析成功: %s", third_candidate.c_str());
                    return third_candidate;
                }
                OUT_CONSOLE(Level::INFO, "第3个候选项目目录不存在: %s", project_dir.c_str());
            }
        }
    }
    
    // 4) 如果以上检查都不成功，从可执行文件路径获取项目名称
    OUT_CONSOLE(Level::WARN, "从文件路径解析项目名称失败，尝试从可执行文件路径获取");
    std::string process_project_name;
    get_process_project_name(process_project_name);
    if (!process_project_name.empty()) {
        OUT_CONSOLE(Level::DEBUG, "从可执行文件路径获取项目名称成功: %s", process_project_name.c_str());
    }
    return process_project_name;
}

/**
 * @brief 列出ROS2包中的所有可执行文件
 * @param package_name ROS2包名称
 * @return 可执行文件名列表
 * @details 在安装目录的lib/<package_name>/下查找所有可执行文件
 */
std::vector<std::string> list_ros_executables(const std::string& package_name) 
{
    std::vector<std::string> executables;
    
    if (package_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "包名称为空");
        return executables;
    }
    
    // 获取安装目录
    std::string install_dir = get_install_dir();
    if (install_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "获取安装目录失败");
        return executables;
    }
    
    // 构建lib目录路径
    std::string lib_dir = install_dir + package_name + "/lib/" + package_name;
    OUT_CONSOLE(Level::DEBUG, "查找可执行文件目录: %s", lib_dir.c_str());
    
    // 检查目录是否存在
    if (!is_path_exists(lib_dir)) {
        OUT_CONSOLE(Level::ERROR, "lib目录不存在: %s", lib_dir.c_str());
        return executables;
    }
    
    // 遍历目录查找可执行文件
    DIR* dir = opendir(lib_dir.c_str());
    if (dir == nullptr) {
        OUT_CONSOLE(Level::ERROR, "无法打开目录: %s", lib_dir.c_str());
        return executables;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string file_name = entry->d_name;
        
        // 跳过 . 和 ..
        if (file_name == "." || file_name == "..") {
            continue;
        }
        
        std::string file_path = lib_dir + "/" + file_name;
        
        // 检查是否为普通文件且可执行
        struct stat file_stat;
        if (stat(file_path.c_str(), &file_stat) == 0) {
            if (S_ISREG(file_stat.st_mode) && (file_stat.st_mode & S_IXUSR)) {
                executables.push_back(file_name);
                OUT_CONSOLE(Level::DEBUG, "找到可执行文件: %s", file_name.c_str());
            }
        }
    }
    
    closedir(dir);
    OUT_CONSOLE(Level::DEBUG, "共找到 %zu 个可执行文件", executables.size());
    return executables;
}

/**
 * @brief 查找ROS2包的可执行文件路径
 * @param package_name ROS2包名称
 * @param node_name 节点名称（可选，如果为空则返回找到的第一个可执行文件）
 * @return 可执行文件的完整路径，失败则返回空字符串
 * @details 在安装目录的lib/<package_name>/下查找可执行文件
 */
std::string find_ros_executable(const std::string& package_name, const std::string& node_name) 
{
    if (package_name.empty()) {
        OUT_CONSOLE(Level::ERROR, "包名称为空");
        return "";
    }
    
    // 获取所有可执行文件
    std::vector<std::string> executables = list_ros_executables(package_name);
    
    if (executables.empty()) {
        OUT_CONSOLE(Level::ERROR, "未找到可执行文件，包名: %s", package_name.c_str());
        return "";
    }
    
    // 获取安装目录
    std::string install_dir = get_install_dir();
    if (install_dir.empty()) {
        OUT_CONSOLE(Level::ERROR, "获取安装目录失败");
        return "";
    }
    
    // 如果指定了节点名称，优先匹配
    if (!node_name.empty()) {
        for (const auto& exec : executables) {
            if (exec == node_name || exec.find(node_name) != std::string::npos) {
                std::string executable_path = install_dir + package_name + "/lib/" + package_name + "/" + exec;
                OUT_CONSOLE(Level::DEBUG, "匹配到节点 %s，可执行文件: %s", node_name.c_str(), executable_path.c_str());
                return executable_path;
            }
        }
        OUT_CONSOLE(Level::WARN, "未找到匹配的节点 %s，返回第一个可执行文件", node_name.c_str());
    }
    
    // 返回第一个可执行文件
    std::string executable_path = install_dir + package_name + "/lib/" + package_name + "/" + executables[0];
    OUT_CONSOLE(Level::DEBUG, "使用可执行文件: %s", executable_path.c_str());
    return executable_path;
}


} // namespace basmodule