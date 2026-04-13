#include "cam_mgr_ros/utils.hpp"
#include <log_system/log_macros.hpp>
#include <bas_sys_config/sys_config_struct.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstring>
#include <thread>
#include <sys/resource.h>    // 添加资源管理头文件用于设置线程优先级
#include <unistd.h>          // 添加 unistd 头文件用于 syscall
#include <syscall.h>         // 添加 syscall 头文件
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_deserializer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <fstream>
#include <algorithm>


namespace cam_mgr_ros
{

// ============================================================================
// 进程与线程工具
// ============================================================================

bool is_pid_alive_and_not_zombie(pid_t pid)
{
    // 先用 kill 检查进程是否存在
    if (kill(pid, 0) == -1)
        return false;

    // 尝试读取 /proc/<pid>/stat 以检查进程状态
    std::string stat_path = std::string("/proc/") + std::to_string(pid) + "/stat";
    std::ifstream stat_file(stat_path);
    if (!stat_file.is_open())
    {
        // 无法打开 /proc 文件，回退为认为进程不存在
        return false;
    }

    std::string stat_line;
    std::getline(stat_file, stat_line);
    stat_file.close();

    // stat 文件格式: pid (comm) state ...，state 位于最后一个 ')' 之后的空格后面
    auto rparen = stat_line.rfind(')');
    if (rparen == std::string::npos || rparen + 2 >= stat_line.size())
    {
        // 无法解析 state，保守地认为进程存活
        return true;
    }

    char state = stat_line[rparen + 2];
    // 'Z' 表示僵尸进程
    if (state == 'Z')
        return false;

    return true;
}

void set_thread_priority(std::thread& t, int nice_value)
{
    try
    {
        if (t.joinable())
        {
            // 获取线程 ID
            pid_t tid = syscall(SYS_gettid);
            
            // 设置 nice 值（需要 CAP_SYS_NICE 能力或 root 权限）
            // 如果失败，nice 值将保持默认值 0
            if (setpriority(PRIO_PROCESS, tid, nice_value) == 0)
            {
                LOG_DEBUG("线程优先级已设置为 nice=%d", nice_value);
            }
            else
            {
                // 非 root 用户可能无法设置 nice 值，记录警告但不影响功能
                LOG_DEBUG("设置线程优先级失败（可能需要 root 权限），使用默认优先级");
            }
        }
    }
    catch (const std::exception& e)
    {
        LOG_DEBUG("设置线程优先级时发生异常：%s", e.what());
    }
}

// ============================================================================
// 相机配置与信息工具
// ============================================================================

void print_camera_device_info(const CamMgr::CamDevInfoList& cam_dev_list)
{
    LOG_INFO("总共遍历到 %zu 个相机设备", cam_dev_list.size());
    bool bShowExInfo = false;
    logsys::Color color = logsys::Color::BLUE;
    std::string strInterval = "    ";
    // 打印每个相机设备的信息
    for (size_t i = 0; i < cam_dev_list.size(); i++)
    {
        const auto &dev_info = cam_dev_list[i];
        LOG_INFO(bShowExInfo, color, "设备索引：%zu", i);
        LOG_INFO(bShowExInfo, color, "%s设备 ID: %s", strInterval.c_str(), dev_info.device_id.c_str());
        LOG_INFO(bShowExInfo, color, "%s相机类型：%d", strInterval.c_str(), static_cast<int>(dev_info.cam_type));
        LOG_INFO(bShowExInfo, color, "%s产品 ID: %s", strInterval.c_str(), dev_info.product_id.c_str());
        LOG_INFO(bShowExInfo, color, "%s设备名称：%s", strInterval.c_str(), dev_info.device_name.c_str());
        LOG_INFO(bShowExInfo, color, "%s序列号：%s", strInterval.c_str(), dev_info.serial_number.c_str());
        LOG_INFO(bShowExInfo, color, "%s物理端口：%s", strInterval.c_str(), dev_info.physical_port.c_str());
        LOG_INFO(bShowExInfo, color, "%s用户名称：%s", strInterval.c_str(), dev_info.user_name.c_str());
        LOG_INFO(bShowExInfo, color, "%s制造商：%s", strInterval.c_str(), dev_info.facturer_name.c_str());
        LOG_INFO(bShowExInfo, color, "%s固件版本：%s", strInterval.c_str(), dev_info.firmware_version.c_str());
    }
}

/**
 * @brief 根据流类型获取传感器类型字符串
 * @param stream_type 流类型
 * @return 传感器类型字符串，如 "color", "depth", "ir"
 */
std::string get_sensor_type_by_stream_type(CamMgr::CamStreamType stream_type)
{
    switch (stream_type)
    {
    case CamMgr::CamStreamType::STREAM_COLOR:
        return "color";
    case CamMgr::CamStreamType::STREAM_DEPTH:
        return "depth";
    case CamMgr::CamStreamType::STREAM_IR:
        return "ir";
    default:
        LOG_ERROR("无效的流类型：%d", static_cast<int>(stream_type));
        return "";
    }
}

void print_camera_config_info(size_t index, const SysConfig::CamConfigInfo& cam_info)
{
    LOG_INFO("相机配置 %zu:", index);
    LOG_INFO("  - 相机ID: %d", static_cast<int>(cam_info.cam_id));
    LOG_INFO("  - 是否启用: %s", cam_info.is_enable ? "是" : "否");
    LOG_INFO("  - 序列号: %s", cam_info.serial_number.c_str());
    LOG_INFO("  - 用户名称: %s", cam_info.user_name.c_str());
    LOG_INFO("  - 默认彩色分辨率: %d", static_cast<int>(cam_info.default_color_resolution));
    LOG_INFO("  - 默认深度分辨率: %d", static_cast<int>(cam_info.default_depth_resolution));
    LOG_INFO("  - 关联机械臂数量: %zu", cam_info.armInfoList.size());
}

// ============================================================================
// 时间戳工具
// ============================================================================

std::string get_current_timestamp()
{
    try
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("获取时间戳时发生错误: %s", e.what());
        return "";
    }
}

std::string format_timestamp_from_msg(const builtin_interfaces::msg::Time& stamp)
{
    try
    {
        time_t sec = static_cast<time_t>(stamp.sec);
        auto ms = stamp.nanosec / 1000000;  // 纳秒转毫秒
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&sec), "%Y%m%d%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms;
        
        return ss.str();
    }
    catch (const std::exception& e)
    {
        LOG_WARN("格式化时间戳失败：%s", e.what());
        // 返回当前时间作为回退
        return get_current_timestamp();
    }
}

// ============================================================================
// 图像处理工具
// ============================================================================

void draw_text(
    cv::Mat& image,
    const std::string& text,
    int font_face,
    double font_scale,
    cv::Scalar text_color,
    bool need_background,
    cv::Scalar bg_color,
    int thickness,
    cv::Point position)
{
    // 获取文本大小
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    
    if (need_background)
    {
        // 绘制文本背景矩形
        cv::Point text_org(position.x, position.y + text_size.height);
        cv::rectangle(
            image,
            cv::Point(position.x, position.y - text_size.height),
            cv::Point(position.x + text_size.width, position.y + baseline),
            bg_color,
            cv::FILLED
        );
    }
    
    // 绘制文本
    cv::putText(image, text, position, font_face, font_scale, text_color, thickness);
}

cv_bridge::CvImagePtr convert_ros_to_cv_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg,
    CamMgr::CamStreamType stream_type)
{
    // 智能指针版本：解引用后调用普通对象版本
    return convert_ros_to_cv_image(*msg, stream_type);
}

// 重载版本：接受普通对象引用
cv_bridge::CvImagePtr convert_ros_to_cv_image(
    const sensor_msgs::msg::Image& msg,
    CamMgr::CamStreamType stream_type)
{
    try
    {
        if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        {
            // 深度图像处理
            if (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                msg.encoding == sensor_msgs::image_encodings::MONO16 ||
                msg.encoding == sensor_msgs::image_encodings::MONO8)
            {
                return cv_bridge::toCvCopy(msg);
            }
            else
            {
                return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            }
        }
        else
        {
            // 彩色图像处理
            if (msg.encoding == sensor_msgs::image_encodings::BGR8 ||
                msg.encoding == sensor_msgs::image_encodings::BGRA8)
            {
                return cv_bridge::toCvCopy(msg);
            }
            else if (msg.encoding == sensor_msgs::image_encodings::RGB8)
            {
                return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            else if (msg.encoding == sensor_msgs::image_encodings::RGBA8)
            {
                return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
            }
            else if (msg.encoding == sensor_msgs::image_encodings::MONO8 ||
                     msg.encoding == sensor_msgs::image_encodings::MONO16)
            {
                return cv_bridge::toCvCopy(msg);
            }
            else
            {
                // 未知编码，尝试作为 BGR8 处理
                return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        LOG_ERROR("cv_bridge exception: %s", e.what());
        return nullptr;
    }
}

bool save_image_to_file(
    const cv::Mat& image,
    const std::string& file_extension,
    std::string& filename)
{
    bool save_success = false;
    
    if (file_extension == "bmp")
    {
        // BMP 格式：无压缩，质量最好
        save_success = cv::imwrite(filename, image);
    }
    else if (file_extension == "jpg" || file_extension == "jpeg")
    {
        // JPEG 格式：有损压缩，设置高质量参数（0-100，默认 95）
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(95);  // 高质量
        save_success = cv::imwrite(filename, image, params);
    }
    else if (file_extension == "png")
    {
        // PNG 格式：无损压缩，设置压缩级别（0-9，默认 3）
        std::vector<int> params;
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(3);  // 默认压缩级别
        save_success = cv::imwrite(filename, image, params);
    }
    else
    {
        // 不支持的格式，回退到 BMP
        LOG_WARN("不支持的图像格式：%s，使用 BMP 格式", file_extension.c_str());
        // 修改文件扩展名为 .bmp
        size_t dot_pos = filename.rfind('.');
        if (dot_pos != std::string::npos)
        {
            filename = filename.substr(0, dot_pos) + ".bmp";
        }
        else
        {
            filename += ".bmp";
        }
        save_success = cv::imwrite(filename, image);
    }
    
    return save_success;
}

// ============================================================================
// 点云处理工具
// ============================================================================

bool save_pointcloud_to_pcd(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    std::string& filename)
{
    try
    {
        // 打开文件（二进制模式）
        std::ofstream pcd_file(filename, std::ios::binary);
        if (!pcd_file.is_open())
        {
            LOG_ERROR("无法打开文件：%s", filename.c_str());
            return false;
        }
        
        // 写入 PCD 文件头
        pcd_file << "# .PCD v.7 - Point Cloud Data file format\n";
        pcd_file << "VERSION .7\n";
        pcd_file << "FIELDS x y z\n";
        pcd_file << "SIZE 4 4 4\n";
        pcd_file << "TYPE F F F\n";
        pcd_file << "COUNT 1 1 1\n";
        pcd_file << "WIDTH " << msg->width << "\n";
        pcd_file << "HEIGHT " << msg->height << "\n";
        pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        pcd_file << "POINTS " << (msg->width * msg->height) << "\n";
        pcd_file << "DATA binary\n";
        
        // 写入点云数据
        pcd_file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
        pcd_file.close();
        
        LOG_DEBUG("保存点云到：%s", filename.c_str());
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("保存点云失败：%s", e.what());
        return false;
    }
}

// 重载版本：接受普通对象引用
bool save_pointcloud_to_pcd(
    const sensor_msgs::msg::PointCloud2& msg,
    const std::string& filename)
{
    try
    {
        // 打开文件（二进制模式）
        std::ofstream pcd_file(filename, std::ios::binary);
        if (!pcd_file.is_open())
        {
            LOG_ERROR("无法打开文件：%s", filename.c_str());
            return false;
        }
        
        // 写入 PCD 文件头
        pcd_file << "# .PCD v.7 - Point Cloud Data file format\n";
        pcd_file << "VERSION .7\n";
        pcd_file << "FIELDS x y z\n";
        pcd_file << "SIZE 4 4 4\n";
        pcd_file << "TYPE F F F\n";
        pcd_file << "COUNT 1 1 1\n";
        pcd_file << "WIDTH " << msg.width << "\n";
        pcd_file << "HEIGHT " << msg.height << "\n";
        pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        pcd_file << "POINTS " << (msg.width * msg.height) << "\n";
        pcd_file << "DATA binary\n";
        
        // 写入点云数据
        pcd_file.write(reinterpret_cast<const char*>(msg.data.data()), msg.data.size());
        pcd_file.close();
        
        LOG_DEBUG("保存点云到：%s", filename.c_str());
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("保存点云失败：%s", e.what());
        return false;
    }
}

// ============================================================================
// BAG 文件处理工具
// ============================================================================

bool parse_bag_file(const std::string& bag_path, bool delete_bag_after_parse)
{
    try
    {
        LOG_INFO("开始解析bag文件: %s", bag_path.c_str());
        
        if (!std::filesystem::exists(bag_path))
        {
            LOG_ERROR("bag文件不存在: %s", bag_path.c_str());
            return false;
        }
        
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        
        // 创建ConverterOptions
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        rosbag2_cpp::readers::SequentialReader reader;
        
        // 打开bag文件，如果失败则重试
        int max_retries = 10;
        int retry_delay_ms = 500;
        bool opened = false;
        
        for (int retry = 0; retry < max_retries; retry++)
        {
            try
            {
                reader.open(storage_options, converter_options);
                LOG_INFO("成功打开bag文件");
                opened = true;
                break;
            }
            catch (const std::exception& e)
            {
                if (retry < max_retries - 1)
                {
                    LOG_WARN("打开bag文件失败（第%d次尝试），%dms后重试: %s", retry + 1, retry_delay_ms, e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                    retry_delay_ms *= 2; // 指数退避
                }
                else
                {
                    LOG_ERROR("打开bag文件失败，已达到最大重试次数: %s", e.what());
                    return false;
                }
            }
        }
        
        if (!opened)
        {
            LOG_ERROR("无法打开bag文件");
            return false;
        }
        
        std::filesystem::path bag_dir = std::filesystem::path(bag_path).parent_path();
        std::filesystem::path base_dir = bag_dir;
        
        std::map<std::string, std::filesystem::path> topic_dirs;
        
        // 添加统计信息
        int total_messages = 0;
        int saved_images = 0;
        int skipped_messages = 0;
        
        // 用于记录已经警告过的编码，避免重复警告
        static std::unordered_set<std::string> warned_empty_encodings;
        static std::unordered_set<std::string> warned_unknown_encodings;
        
        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (!bag_message)
            {
                skipped_messages++;
                continue;
            }
            
            total_messages++;
            std::string topic_name = bag_message->topic_name;
            
            // 只处理图像话题
            if (topic_name.find("color") != std::string::npos ||
                topic_name.find("depth") != std::string::npos ||
                topic_name.find("image") != std::string::npos ||
                topic_name.find("cloud") != std::string::npos)
            {
                // 从话题名提取相机 ID 和流类型
                // 示例话题：
                // /camera/rs_camera_1/color/image_rect_raw
                // /ob_camera_0/color/image_raw
                // /csi_camera_0/color/image_raw
                std::string cam_id_str;
                std::string stream_type_str;
                                
                // 尝试多种模式提取相机标识
                // 模式 1: /cam_X/... (如 /cam_0/color)
                // 模式 2: /xxx_camera_Y/... (如 /ob_camera_0/color, /rs_camera_1/color)
                // 模式 3: 直接使用第一个路径组件作为相机标识
                                
                std::vector<std::string> path_parts;
                std::stringstream ss(topic_name);
                std::string part;
                while (std::getline(ss, part, '/'))
                {
                    if (!part.empty())
                    {
                        path_parts.push_back(part);
                    }
                }
                                
                // 查找包含"camera"的部分
                size_t camera_idx = path_parts.size(); // 初始化为无效值
                for (size_t i = 0; i < path_parts.size(); ++i)
                {
                    if (path_parts[i].find("camera") != std::string::npos)
                    {
                        camera_idx = i;
                        break;
                    }
                }
                                
                if (camera_idx != path_parts.size() && camera_idx + 1 < path_parts.size())
                {
                    // 找到了 camera 部分，从中提取数字 ID
                    std::string camera_name = path_parts[camera_idx];
                    
                    // 从相机名中提取最后一个数字（如 rs_camera_1 -> 1, ob_camera_0 -> 0）
                    std::string cam_number;
                    for (auto it = camera_name.rbegin(); it != camera_name.rend(); ++it)
                    {
                        if (std::isdigit(*it))
                        {
                            cam_number = *it + cam_number;
                        }
                        else if (!cam_number.empty())
                        {
                            // 已经找到数字且遇到非数字字符，停止
                            break;
                        }
                    }
                    
                    // 如果提取到数字，使用 cam_X 格式；否则使用原始名称
                    if (!cam_number.empty())
                    {
                        cam_id_str = "cam_" + cam_number;
                    }
                    else
                    {
                        cam_id_str = camera_name; // 回退到原始名称
                    }
                    
                    // 下一个部分是流类型（从路径提取）
                    stream_type_str = path_parts[camera_idx + 1];
                    
                    // 根据实际话题类型覆盖 stream_type_str（优先级：cloud > depth > color）
                    bool is_color = (topic_name.find("color") != std::string::npos);
                    bool is_depth = (topic_name.find("depth") != std::string::npos) && !is_color;
                    bool is_cloud = (topic_name.find("cloud") != std::string::npos || 
                                    topic_name.find("pointcloud") != std::string::npos ||
                                    topic_name.find("points") != std::string::npos);
                    if (is_cloud)
                    {
                        stream_type_str = "cloud";  // 点云优先，覆盖路径提取的值
                    }
                    else if (is_depth)
                    {
                        stream_type_str = "depth";
                    }
                    else if (is_color)
                    {
                        stream_type_str = "color";
                    }
                }
                else
                {
                    // 没找到 camera 模式，使用默认值
                    cam_id_str = "unknown";
                    // 尝试从话题名推断流类型（优先级：cloud > depth > color）
                    bool is_color = (topic_name.find("color") != std::string::npos);
                    bool is_depth = (topic_name.find("depth") != std::string::npos) && !is_color;
                    bool is_cloud = (topic_name.find("cloud") != std::string::npos || 
                                    topic_name.find("pointcloud") != std::string::npos ||
                                    topic_name.find("points") != std::string::npos);
                    if (is_color)
                    {
                        stream_type_str = "color";
                    }
                    if (is_cloud)
                    {
                        stream_type_str = "cloud";  // 点云优先
                    }
                    if (is_depth)
                    {
                        stream_type_str = "depth";
                    }
                   
                }
                            
                // 创建目录结构：/{相机标识}/{流类型}/
                std::string dir_key = cam_id_str + "/" + stream_type_str;
                if (topic_dirs.find(dir_key) == topic_dirs.end())
                {
                    std::filesystem::path stream_dir = base_dir / cam_id_str / stream_type_str;
                    std::filesystem::create_directories(stream_dir);
                    topic_dirs[dir_key] = stream_dir;
                    LOG_INFO("创建目录：%s (来自话题：%s)", stream_dir.c_str(), topic_name.c_str());
                }
                
                try
                {
                    // 判断话题类型（先判断点云，再判断深度）
                    bool is_cloud = (topic_name.find("cloud") != std::string::npos || 
                                    topic_name.find("pointcloud") != std::string::npos ||
                                    topic_name.find("points") != std::string::npos);
                    bool is_depth = !is_cloud && (topic_name.find("depth") != std::string::npos);
                    
                    if (is_cloud)
                    {
                        // 点云话题：反序列化为 PointCloud2 并保存为 PCD 文件
                        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;
                        sensor_msgs::msg::PointCloud2 pc_msg;
                        rclcpp::SerializedMessage pc_serialized_msg(*bag_message->serialized_data);
                        
                        pc_serialization.deserialize_message(&pc_serialized_msg, &pc_msg);
                        
                        LOG_INFO("[DEBUG] 点云话题：%s, width=%d, height=%d",
                                 topic_name.c_str(), pc_msg.width, pc_msg.height);
                        
                        // 生成文件名（使用时间戳，.pcd 扩展名）
                        std::string filename = cam_mgr_ros::format_timestamp_from_msg(pc_msg.header.stamp) + ".pcd";
                        std::filesystem::path file_path = topic_dirs[dir_key] / filename;
                        
                        // 使用封装的 save_pointcloud_to_pcd 函数保存
                        bool save_success = cam_mgr_ros::save_pointcloud_to_pcd(pc_msg, file_path.string());
                        
                        if (save_success)
                        {
                            saved_images++;
                            LOG_DEBUG("保存点云：%s (话题：%s)", file_path.c_str(), topic_name.c_str());
                        }
                        else
                        {
                            LOG_ERROR("保存点云失败：%s", file_path.c_str());
                            skipped_messages++;
                        }
                    }
                    else
                    {
                        // 图像话题：反序列化为 Image 并保存
                        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
                        sensor_msgs::msg::Image image_msg;
                        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                        
                        serialization.deserialize_message(&serialized_msg, &image_msg);
                        
                        // [修改1] 检查编码是否为空
                        if (image_msg.encoding.empty())
                        {
                            // 每个话题只警告一次
                            if (warned_empty_encodings.find(topic_name) == warned_empty_encodings.end())
                            {
                                LOG_WARN("话题 %s 包含空编码的图像消息，将跳过此类消息", topic_name.c_str());
                                warned_empty_encodings.insert(topic_name);
                            }
                            skipped_messages++;
                            continue;
                        }
                        
                        // 转换为 OpenCV 图像
                        cv_bridge::CvImagePtr cv_ptr;
                        bool is_depth = (topic_name.find("depth") != std::string::npos);
                        bool is_cloud = (topic_name.find("cloud") != std::string::npos || 
                                        topic_name.find("pointcloud") != std::string::npos);
                        
                        // 打印详细日志用于调试
                        LOG_INFO("[DEBUG] 话题：%s, 编码：%s, is_depth=%d, is_cloud=%d, stream_type_str=%s",
                                 topic_name.c_str(), image_msg.encoding.c_str(), 
                                 is_depth ? 1 : 0, is_cloud ? 1 : 0, stream_type_str.c_str());
                        
                        try
                        {
                            // 使用封装的 convert_ros_to_cv_image 函数（普通对象版本）
                            CamMgr::CamStreamType stream_type = is_depth ? 
                                CamMgr::CamStreamType::STREAM_DEPTH : 
                                CamMgr::CamStreamType::STREAM_COLOR;
                            
                            LOG_INFO("[DEBUG] 准备转换，stream_type=%d (0=COLOR, 1=DEPTH, 2=IR, 3=CLOUD)", 
                                     static_cast<int>(stream_type));
                            
                            cv_ptr = cam_mgr_ros::convert_ros_to_cv_image(image_msg, stream_type);
                        }
                        catch (const cv_bridge::Exception& e)
                        {
                            LOG_DEBUG("cv_bridge 转换失败 (话题：%s, 编码：%s): %s", 
                                     topic_name.c_str(), image_msg.encoding.c_str(), e.what());
                            skipped_messages++;
                            continue;
                        }
                        
                        // 生成文件名（使用时间戳，去掉 image_前缀）
                        std::string filename = cam_mgr_ros::format_timestamp_from_msg(image_msg.header.stamp) + ".bmp";
                        
                        std::filesystem::path file_path = topic_dirs[dir_key] / filename;
                        
                        cv::imwrite(file_path.string(), cv_ptr->image);
                        
                        saved_images++;
                        
                        // [修改6] 降低INFO日志级别，避免输出过多
                        if (saved_images % 10 == 0)
                        {
                            LOG_INFO("已保存 %d 张图像，处理消息 %d 条，跳过 %d 条", 
                                    saved_images, total_messages, skipped_messages);
                        }
                        else
                        {
                            LOG_DEBUG("保存图像: %s (话题: %s, 编码: %s)", 
                                     file_path.c_str(), topic_name.c_str(), image_msg.encoding.c_str());
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    LOG_DEBUG("处理图像消息失败 (话题: %s): %s", topic_name.c_str(), e.what());
                    skipped_messages++;
                    continue;
                }
            }
            else
            {
                // 非图像话题，跳过
                //skipped_messages++;
            }
        }
        
        reader.close();
        LOG_INFO("bag 文件解析完成，总计处理消息 %d 条，保存图像 %d 张，跳过 %d 条", 
                total_messages, saved_images, skipped_messages);
                
        // 根据配置决定是否删除 ROS 包文件
        if (delete_bag_after_parse)
        {
            try
            {
                // 删除整个目录及其所有内容（包括 bag 文件、metadata 和保存的图像子目录）
                if (std::filesystem::exists(bag_path))
                {
                    std::uintmax_t count = std::filesystem::remove_all(bag_path);
                    LOG_INFO("已删除 bag 目录及所有内容：%s (共删除 %lu 项)", bag_path.c_str(), count);
                }
                else
                {
                    LOG_WARN("bag 目录不存在：%s", bag_path.c_str());
                }
            }
            catch (const std::exception& e)
            {
                LOG_ERROR("删除 bag 文件失败：%s", e.what());
            }
        }
        else
        {
            LOG_INFO("保留 bag 文件（根据配置）：%s", bag_path.c_str());
        }
                
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("解析 bag 文件时发生错误：%s", e.what());
        return false;
    }
}

// ============================================================================
// 文件夹压缩工具
// ============================================================================

bool compress_directory(const std::string& source_dir, const std::string& output_zip)
{
    // 检查源目录是否存在
    if (!std::filesystem::exists(source_dir))
    {
        LOG_ERROR("压缩文件夹失败：源目录不存在 [%s]", source_dir.c_str());
        return false;
    }
    
    // 检查是否为目录
    if (!std::filesystem::is_directory(source_dir))
    {
        LOG_ERROR("压缩文件夹失败：源路径不是目录 [%s]", source_dir.c_str());
        return false;
    }
    
    // 获取源目录的父目录和目录名
    std::filesystem::path dir_path(source_dir);
    std::string parent_dir = dir_path.parent_path().string();
    std::string dir_name = dir_path.filename().string();
    
    // 构建 zip 命令（使用相对路径压缩）
    // 切换到父目录，然后只压缩目录名，这样解压后就是 cam0/ 而不是完整路径
    std::string cmd = "cd \"" + parent_dir + "\" && zip -r -q -9 \"" + output_zip + "\" \"" + dir_name + "\"";
    
    LOG_INFO("开始压缩文件夹：%s -> %s", source_dir.c_str(), output_zip.c_str());
    
    // 执行压缩命令
    int ret = system(cmd.c_str());
    
    if (ret != 0)
    {
        LOG_ERROR("压缩文件夹失败，返回码：%d", ret);
        return false;
    }
    
    // 验证输出文件是否存在
    if (!std::filesystem::exists(output_zip))
    {
        LOG_ERROR("压缩文件夹成功但输出文件不存在 [%s]", output_zip.c_str());
        return false;
    }
    
    LOG_INFO("文件夹压缩成功：%s", output_zip.c_str());
    return true;
}

}  // namespace cam_mgr_ros
