#ifndef CAM_MGR_ROS__UTILS_HPP_
#define CAM_MGR_ROS__UTILS_HPP_

#include <string>
#include <fstream>
#include <sys/types.h>
#include <signal.h>
#include <map>
#include <filesystem>
#include <cam_config_mgr/cam_com_struct.hpp>
#include <bas_sys_config/sys_config_struct.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>

namespace cam_mgr_ros
{

// ============================================================================
// 进程与线程工具
// ============================================================================

/**
 * @brief 检查给定 pid 是否存在且不是僵尸进程
 * @param pid 进程 ID
 * @return 如果进程存在且不是僵尸进程，返回 true；否则返回 false
 */
bool is_pid_alive_and_not_zombie(pid_t pid);

/**
 * @brief 设置线程优先级（nice 值）
 * @param t 线程引用
 * @param nice_value nice 值（范围 -20~19，越大优先级越低）
 */
void set_thread_priority(std::thread& t, int nice_value);

// ============================================================================
// 相机配置与信息工具
// ============================================================================

/**
 * @brief 打印相机设备信息列表
 * @param cam_dev_list 相机设备信息列表
 */
void print_camera_device_info(const CamMgr::CamDevInfoList& cam_dev_list);

/**
 * @brief 根据流类型获取传感器类型字符串
 * @param stream_type 流类型
 * @return 传感器类型字符串，如 "color", "depth", "ir"
 */
std::string get_sensor_type_by_stream_type(CamMgr::CamStreamType stream_type);

/**
 * @brief 打印相机配置信息
 * @param index 配置索引
 * @param cam_info 相机配置信息
 */
void print_camera_config_info(size_t index, const SysConfig::CamConfigInfo& cam_info);

// ============================================================================
// 时间戳工具
// ============================================================================

/**
 * @brief 获取当前时间戳字符串
 * @return 时间戳字符串，格式为 "YYYY-MM-DD HH:MM:SS"
 */
std::string get_current_timestamp();

/**
 * @brief 从 ROS2 消息时间戳生成格式化的时间字符串
 * @param stamp ROS2 时间戳（builtin_interfaces::msg::Time）
 * @return 格式化后的时间字符串，格式为 "YYYYMMDD_HHMMSS_mmm"
 */
std::string format_timestamp_from_msg(const builtin_interfaces::msg::Time& stamp);

// ============================================================================
// 图像处理工具
// ============================================================================

/**
 * @brief 在图像上绘制文本
 * @param image 目标图像（引用传递，直接修改）
 * @param text 要绘制的文本内容
 * @param font_face 字体类型（如 cv::FONT_HERSHEY_SIMPLEX）
 * @param font_scale 字体缩放比例
 * @param text_color 文本颜色（BGR 格式，如 cv::Scalar(0, 255, 0) 表示绿色）
 * @param need_background 是否需要背景底色
 * @param bg_color 背景底色（BGR 格式，默认为黑色）
 * @param thickness 文本线条粗细
 * @param position 文本位置（左上角坐标）
 */
void draw_text(
    cv::Mat& image,
    const std::string& text,
    int font_face = cv::FONT_HERSHEY_SIMPLEX,
    double font_scale = 0.8,
    cv::Scalar text_color = cv::Scalar(0, 255, 0),
    bool need_background = true,
    cv::Scalar bg_color = cv::Scalar(0, 0, 0),
    int thickness = 1,
    cv::Point position = cv::Point(10, 20)
);

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 格式（智能指针版本）
 * @param msg ROS 图像消息（智能指针）
 * @param stream_type 流类型（用于判断是否为深度图）
 * @return 转换后的 OpenCV 图像指针，失败返回 nullptr
 */
cv_bridge::CvImagePtr convert_ros_to_cv_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg,
    CamMgr::CamStreamType stream_type
);

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 格式（普通对象版本）
 * @param msg ROS 图像消息（普通对象引用）
 * @param stream_type 流类型（用于判断是否为深度图）
 * @return 转换后的 OpenCV 图像指针，失败返回 nullptr
 */
cv_bridge::CvImagePtr convert_ros_to_cv_image(
    const sensor_msgs::msg::Image& msg,
    CamMgr::CamStreamType stream_type
);

/**
 * @brief 保存图像到文件
 * @param image OpenCV 图像矩阵
 * @param file_extension 文件扩展名（"bmp", "jpg", "jpeg", "png"）
 * @param filename 输出参数，保存后的实际文件名
 * @return 是否保存成功
 */
bool save_image_to_file(
    const cv::Mat& image,
    const std::string& file_extension,
    std::string& filename
);

// ============================================================================
// 点云处理工具
// ============================================================================

/**
 * @brief 保存点云数据到 PCD 文件
 * @param msg 点云消息（ROS2 格式）
 * @param filename 输出参数，保存后的实际文件名
 * @return 是否保存成功
 */
bool save_pointcloud_to_pcd(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    std::string& filename
);

// ============================================================================
// BAG 文件处理工具
// ============================================================================

/**
 * @brief 解析 bag 包中的图像数据
 * @param bag_path bag 包路径
 * @param delete_bag_after_parse 解析完成后是否删除 bag 文件
 * @return 是否解析成功
 */
bool parse_bag_file(const std::string& bag_path, bool delete_bag_after_parse = false);

// ============================================================================
// 文件夹压缩工具
// ============================================================================

/**
 * @brief 压缩文件夹
 * 使用系统的 zip 命令压缩指定目录
 * @param source_dir 源目录路径（要压缩的文件夹）
 * @param output_zip 输出的 zip 文件路径（包含文件名和.zip 扩展名）
 * @return bool 成功返回 true，失败返回 false
 */
bool compress_directory(const std::string& source_dir, const std::string& output_zip);

}  // namespace cam_mgr_ros

#endif  // CAM_MGR_ROS__UTILS_HPP_