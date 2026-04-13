#ifndef BAS_OPERATE_FILE_OPERATE_HPP
#define BAS_OPERATE_FILE_OPERATE_HPP

#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <thread>
#include <iostream>
#include <cstdarg>
#include <cstring>

namespace basmodule {

/**
 * @brief 输出消息函数，支持输出到不同的流，自动添加[DEBUG]或[ERROR]前缀
 * @param stream 输出流，如 std::cout (输出[DEBUG]) 或 std::cerr/std::clog (输出[ERROR])
 * @param prefix 输出前缀，如 "[DEBUG]" 或 "[ERROR]"
 * @param msg 要输出的消息
 * @param bShowBasMsg 是否显示文件名、函数名和行号等信息，默认为true
 * @param file 文件名，使用 __FILE__ 宏传入，默认为当前文件
 * @param func 函数名，使用 __func__ 宏传入，默认为当前函数
 * @param line 行号，使用 __LINE__ 宏传入，默认为当前行
 * @details 示例用法:
 *          output(std::cout, "调试信息");  // 输出: [DEBUG] [file:func:line] 调试信息
 *          output(std::cerr, "错误信息");  // 输出: [ERROR] [file:func:line] 错误信息
 */
bool output_console(const std::string& prefix, const std::string& msg, bool bShowBasMsg = true, 
    const char* file = __FILE__, const char* func = __FUNCTION__, int line = __LINE__);

bool output_file(const std::string& prefix, const std::string& msg, const std::string& file_path, bool bShowBasMsg = true, 
    const char* file = __FILE__, const char* func = __FUNCTION__, int line = __LINE__);


// 自定义的 constexpr 版本的 strchr 函数
inline constexpr const char* custom_strchr(const char* str, char c) 
{
    while (*str) 
    {
        if (*str == c) 
        {
            return str;
        }
        ++str;
    }
    return nullptr;
}

// 检测字符串中是否包含格式化符号的辅助函数
inline constexpr bool contains_format_specifier(const char* str) 
{
    if (!str) return false;
    
    // 使用与 count_valid_placeholders 相同的逻辑
    const char* p = str;
    while (*p) 
    {
        if (*p == '%' && *(p + 1) != '\0') 
        {
            char next_char = *(p + 1);
            // 跳过连续的%%（转义的%）
            if (next_char == '%') 
            {
                p += 2;
                continue;
            }
            // 检查是否为有效的格式化占位符
            if (custom_strchr("diouxXeEfFgGaAcspn", next_char) != nullptr) {
                return true;
            }
        }
        p++;
    }
    return false;
}

// 重载版本，支持 std::string 类型
inline bool contains_format_specifier(const std::string& str)
{
    return contains_format_specifier(str.c_str());
}

/**
 * @brief 输出格式化消息函数，支持类似printf的格式化字符串
 * @param stream 输出流，如 std::cout (输出[DEBUG]) 或 std::cerr/std::clog (输出[ERROR])
 * @param fmt 格式化字符串，如 "错误代码: %d, 描述: %s"
 * @param prefix 输出前缀，如 "[DEBUG]" 或 "[ERROR]"
 * @param fmt 格式化字符串，如 "错误代码: %d, 描述: %s"
 * @param bShowBasMsg 是否显示文件名、函数名和行号等信息，默认为true
 * @param file 文件名，使用 __FILE__ 宏传入，默认为当前文件
 * @param func 函数名，使用 __func__ 宏传入，默认为当前函数
 * @param line 行号，使用 __LINE__ 宏传入，默认为当前行
 * @param ... 可变参数，根据格式化字符串提供相应参数
 * @details 示例用法:
 *          output(std::cout, "调试信息: %d", 42);  // 输出: [DEBUG] [file:func:line] 调试信息: 42
 *          output(std::cerr, "错误信息: %s", "发生错误");  // 输出: [ERROR] [file:func:line] 错误信息: 发生错误
 */
void output_console_fmt(const std::string& prefix, const char* fmt, bool bShowBasMsg = true,
    const char* file = __FILE__, const char* func = __FUNCTION__, int line = __LINE__, ...);

/**
 * @brief 输出格式化消息到文件，支持类似printf的格式化字符串
 * @param prefix 输出前缀，如 "[DEBUG]" 或 "[ERROR]"
 * @param file_path 要写入的文件路径
 * @param fmt 格式化字符串，如 "错误代码: %d, 描述: %s"
 * @param bShowBasMsg 是否显示文件名、函数名和行号等信息，默认为true
 * @param file 文件名，使用 __FILE__ 宏传入，默认为当前文件
 * @param func 函数名，使用 __func__ 宏传入，默认为当前函数
 * @param line 行号，使用 __LINE__ 宏传入，默认为当前行
 * @param ... 可变参数，根据格式化字符串提供相应参数
 * @details 示例用法:
 *          output_file_fmt("[INFO]", "/tmp/log.txt", "调试信息: %d", 42);  // 写入: [INFO] [file:func:line] 调试信息: 42
 *          output_file_fmt("[ERROR]", "/tmp/log.txt", "错误信息: %s", "发生错误");  // 写入: [ERROR] [file:func:line] 错误信息: 发生错误
 */
void output_file_fmt(const std::string& prefix, const std::string& file_path, const char* fmt, bool bShowBasMsg = true,
    const char* file = __FILE__, const char* func = __FUNCTION__, int line = __LINE__, ...);

/**
 * @brief 检查项目名称是否有效
 * 
 * 验证项目名称的合法性：
 * - 不能为空字符串
 * - 不能只包含空白字符
 * - 不能包含空字符（\0）
 * 
 * @param project_name 要检查的项目名称
 * @return bool 如果项目名称有效返回true，否则返回false
 */
bool is_valid_name(const std::string& project_name);

/**
 * @brief 安全创建目录（使用C++17 filesystem库）
 * 
 * 使用现代C++的filesystem库创建目录：
 * - 检查目录是否已存在
 * - 创建多级目录结构
 * - 异常安全的目录创建
 * 
 * @param dir_path 目录路径
 * @return bool 创建是否成功
 */
bool create_directory_safe(const std::string& dir_path);

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
void clear_directory(const std::string& dir_path, const std::string& pattern);

/**
 * @brief 获取文件名
 * @param file_path 文件路径，通常使用 __FILE__ 宏
 * @return std::string 文件名
 */
std::string get_file_name(const std::string& file_path = std::string(__FILE__));

/**
 * @brief 获取指定文件的基础文件名（不包含扩展名）
 * @param file_path 文件路径，通常使用 __FILE__ 宏
 * @return std::string 基础文件名（不包含扩展名）
 */
std::string get_file_name_no_ext(const std::string& file_path = std::string(__FILE__));

//获取当前线程ID
uint64_t get_thread_id();

//获取当前线程ID的字符串表示
std::string get_thread_id_string();

/**
 * @brief 检查路径是否存在（可以是目录或文件）
 * 
 * @param path 要检查的路径
 * @return bool 如果路径存在返回true，否则返回false
 */
bool is_path_exists(const std::string& path);

/**
 * @brief 解析路径中的波浪号(~)为用户主目录
 * @param path 待解析的路径字符串
 * @return 解析后的路径字符串，如果包含波浪号则替换为实际的主目录路径，否则返回原路径
 * @details 该函数用于将路径中的波浪号(~)展开为实际的用户主目录路径
 *          例如：输入"~/ros2_logs/"，当前用户名为user，则输出"/home/user/ros2_logs/"
 *          如果输入路径不包含波浪号，则按原路径返回
 */
std::string expand_path(const std::string& path);

/**
 * @brief 将多个字符串拼接成路径形式，自动处理多余的斜杠
 * 
 * @param paths 要拼接的路径字符串列表
 * @return std::string 拼接后的路径字符串
 */
std::string join_path(const std::vector<std::string>& paths);

/**
 * @brief 将多个字符串拼接成路径形式，自动处理多余的斜杠
 * 
 * @param paths 要拼接的路径字符串
 * @return std::string 拼接后的路径字符串
 */
template<size_t N>
std::string join_path(const std::array<std::string, N>& paths)
{
    std::vector<std::string> vec_paths(paths.begin(), paths.end());
    return join_path(vec_paths);
}

/**
 * @brief 将多个字符串拼接成路径形式，自动处理多余的斜杠
 * 
 * @param paths 要拼接的路径字符串
 * @return std::string 拼接后的路径字符串
 */
template<typename... Args>
std::string join_path(const Args&... args)
{
    std::vector<std::string> vec_paths = {args...};
    return join_path(vec_paths);
}

/**
 * @brief 获取当前可执行文件的完整路径
 * 
 * 跨平台获取当前运行的可执行文件的完整路径：
 * - Linux/Unix: 通过读取 /proc/self/exe 符号链接获取
 * - Windows: 通过 GetModuleFileName 获取
 * 
 * @return std::string 可执行文件的完整路径，失败时返回空字符串
例如输出可执行文件路径: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
 */
std::string get_executable_path();

//获取进程ID(PID)
bool get_process_id(pid_t* pid);

//获取当前工作目录
std::string get_current_working_directory();

//获取启动命令
std::string get_startup_command(pid_t pid);

/**
 * @brief 获取指定进程创建时间
 * 
 * @param pid 进程ID
 * @return std::time_t 进程创建时间
 */
std::time_t get_process_create_time(pid_t pid);

/**
 * @brief 获取当前进程创建时间
 * 
 * @return std::time_t 进程创建时间
 */
std::time_t get_process_create_time();
/**
 * @brief 获取进程创建时间
 * 
 * @return std::string 进程创建时间字符串
 */
std::string format_process_create_time(); 

// 添加将指定进程PID的时间戳转换为字符串的函数重载
std::string format_process_create_time(pid_t pid);

/**
 * @brief 将time_t转换为字符串格式
 * 
 * @param time_val 时间值
 * @param include_date 是否包含日期，默认为true
 * @return std::string 格式化后的时间字符串
 */
std::string format_time_to_string(std::time_t time_val, bool include_date = true);

//获取当前进程的详细路径信息
std::string get_current_process_info();

/**
 * @brief 获取进程项目所在目录
 * @param process_project_name 输出参数，用于存储项目名称
 * @return 项目目录路径，失败则返回空字符串
 * @details 基于可执行文件路径构建项目目录
 *          例如: /home/user/testCode/project/install/log_demo/lib/log_demo/log_demo
 *          构建结果: /home/user/testCode/project/install/log_demo
 *          输出项目名称: log_demo 
 */
std::string get_process_project_name(std::string& process_project_name);

// 获取安装目录
std::string get_install_dir();

//获取项目目录
std::string get_project_dir(const std::string& project_name);

//根据项目目录获取项目名称
std::string get_project_name_by_project_dir(const std::string& project_dir);

// 获取项目配置文件所在目录
std::string get_project_config_dir(const std::string& project_dir);

//获取项目配置文件路径
std::string get_project_config_file_path(const std::string& project_dir, const std::string& file_name);

/**
 * @brief 根据项目配置文件路径获取项目名称
 * @param project_config_file_path 项目配置文件的完整路径
 * @param[out] project_name 输出参数，用于存储提取出的项目名称
 * @return 项目路径，如果解析失败则返回空字符串
 * @details 从项目配置文件路径中解析出项目名称，并返回项目的基本路径
 *          例如输入: /home/user/testCode/project/install/log_demo/share/log_demo/config/log_config.yaml
 *          输出project_name: log_demo，返回项目路径: /home/user/testCode/project/install/log_demo
 */
std::string get_project_name_by_config_file_path(const std::string& project_config_file_path, std::string& project_name);

/**
 * @brief 获取当前文件所在ROS项目的CMakeLists.txt文件路径
 * @param file_name_path 文件路径（__FILE__）
 * @return CMakeLists.txt文件路径，失败则返回空字符串
 * @details 通过传入的文件路径解析得到当前文件所在的ROS项目的CMakeLists.txt文件
 *          例如输入: /home/user/testCode/dev_test/src/log/log_system/src/src/log_utils.cpp
 *          输出: /home/user/testCode/dev_test/src/log/log_system/src/CMakeLists.txt
 */
std::string get_cmake_list_file_path(const std::string& file_name_path = std::string(__FILE__));

/**
 * @brief 获取当前文件所在ROS项目的package.xml文件路径
 * @param file_name_path 文件路径（__FILE__）
 * @return package.xml文件路径，失败则返回空字符串
 * @details 通过传入的文件路径解析得到当前文件所在的ROS项目的package.xml文件
 *          例如输入: /home/user/testCode/dev_test/src/log/log_system/src/src/log_utils.cpp
 *          输出: /home/user/testCode/dev_test/src/log/log_system/src/package.xml
 */
std::string get_package_xml_file_path(const std::string& file_name_path = std::string(__FILE__));

/**
 * @brief 从CMakeLists.txt文件中解析项目名称
 * @param cmake_list_file_path CMakeLists.txt文件路径
 * @return 项目名称，解析失败则返回空字符串
 * @details 根据CMakeLists.txt文件路径解析文件内容，获取项目名称
 *          通过查找project()命令来提取项目名称
 */
std::string get_project_name_by_cmake_list_file(const std::string& cmake_list_file_path);

/**
 * @brief 从package.xml文件中解析项目名称
 * @param package_xml_file_path package.xml文件路径
 * @return 项目名称，解析失败则返回空字符串
 * @details 根据package.xml文件路径解析文件内容，获取项目名称
 *          通过查找<name>标签来提取项目名称
 */
std::string get_project_name_by_package_xml_file(const std::string& package_xml_file_path);

/**
 * @brief 综合获取项目名称（优先使用CMakeLists.txt，失败则使用package.xml）
 * @param file_name_path 文件路径（__FILE__）
 * @return 项目名称，获取失败则返回空字符串
 * @details 通过传入的文件路径，首先尝试从CMakeLists.txt获取项目名称
 *          如果失败则尝试从package.xml获取项目名称
 *          例如输入: /home/user/testCode/dev_test/src/log/log_system/src/src/log_utils.cpp
 *          输出: log_system
 */
std::string get_project_name_by_file_path(const std::string& file_name_path = std::string(__FILE__));

/**
 * @brief 查找ROS2包的可执行文件路径
 * @param package_name ROS2包名称
 * @param node_name 节点名称（可选，如果为空则返回找到的第一个可执行文件）
 * @return 可执行文件的完整路径，失败则返回空字符串
 * @details 在安装目录的lib/<package_name>/下查找可执行文件
 *          例如：package_name = "bas_sys_config_ros"
 *          查找路径：<install_dir>/bas_sys_config_ros/lib/bas_sys_config_ros/
 *          如果找到多个可执行文件，优先匹配node_name，否则返回第一个
 */
std::string find_ros_executable(const std::string& package_name, const std::string& node_name = "");

/**
 * @brief 列出ROS2包中的所有可执行文件
 * @param package_name ROS2包名称
 * @return 可执行文件名列表
 * @details 在安装目录的lib/<package_name>/下查找所有可执行文件
 */
std::vector<std::string> list_ros_executables(const std::string& package_name);

} // namespace basmodule

#endif