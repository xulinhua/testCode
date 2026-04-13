#ifndef CAM_SDK_ARUCO_TEST_HPP
#define CAM_SDK_ARUCO_TEST_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>  // 添加atomic头文件

// ROS includes
#ifdef __ROS_ENV__
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  // 添加PoseStamped消息类型
#include <cv_bridge/cv_bridge.h>
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/srv/get_marker_detection.hpp"
#include "custom_msgs_comm/srv/get_markers_info.hpp"  // 添加GetMarkersInfo服务头文件
#include <vision_msgs/msg/detection2_d.hpp>  // 添加Detection2D消息类型
#endif

// 前置声明
namespace visualization {
class VisualizationMgr;
}
namespace aruco_alg {
class ArucoDetector;
}
namespace comm_alg {
struct MarkerInfo;
}

class CamBase;
class CameraManager;

/**
 * @class CamSdkArucoTest
 * @brief 相机SDK Aruco测试类
 * 
 * 该类负责实现相机SDK实时采图进行Aruco码识别，
 * 或加载本地图片文件列表进行Aruco码识别的功能
 */
class CamSdkArucoTest : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    CamSdkArucoTest();
    
    /**
     * @brief 析构函数
     */
    ~CamSdkArucoTest();
    
    /**
     * @brief 初始化
     * @param config_file 配置文件路径
     * @return 是否初始化成功
     */
    bool initialize(const std::string& config_file);
    
    /**
     * @brief 运行主循环
     * @return 是否运行成功
     */
    bool run();
    
    /**
     * @brief 关闭程序
     */
    void shutdown();
    
    /**
     * @brief 获取配置文件路径
     * @param default_config_path 默认配置文件路径
     * @return 实际配置文件路径
     */
    static std::string getConfigFilePath(const std::string& default_config_path);

private:
    /**
     * @brief 加载配置文件
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    bool loadConfig(const std::string& config_file);
    
    /**
     * @brief 初始化相机内参
     * @return 是否初始化成功
     */
    bool initCameraIntrinsics(const std::string& Intrinsics_config_file);
    
    /**
     * @brief 初始化相机模式
     * @return 是否初始化成功
     */
    bool initCameraMode();
    
    /**
     * @brief 初始化文件模式
     * @return 是否初始化成功
     */
    bool initFileMode();
    
    /**
     * @brief 处理相机模式下的图像
     * @return 是否处理成功
     */
    bool processCameraMode();
    
    /**
     * @brief 处理文件模式下的图像
     * @return 是否处理成功
     */
    bool processFileMode();
    
    /**
     * @brief 处理单帧图像的Aruco检测
     * @param image 输入图像
     */
    void processArucoDetection(const cv::Mat& image);
    
#ifdef __ROS_ENV__
    /**
     * @brief 处理检测结果回调
     * @param msg 检测结果消息
     */
    void resultCallback(const vision_msgs::msg::Detection2D::SharedPtr msg);
    
    /**
     * @brief 处理标定结果回调
     * @param msg 标定结果消息
     */
    void calibResultCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief 图像订阅回调
     * @param msg 图像消息
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    void reservice();
    
    /**
     * @brief 请求Aruco标记信息服务
     */
    void requestArucoMakersInfo();
    
    /**
     * @brief 持续调用reservice的线程函数
     */
    void reserviceThread();
#endif
    
    /**
     * @brief 获取文件夹中的所有图片文件
     * @param folder_path 文件夹路径
     * @param extensions 支持的文件扩展名
     * @return 图片文件路径列表
     */
    std::vector<std::string> getImageFiles(const std::string& folder_path, 
                                         const std::vector<std::string>& extensions);
    
    /**
     * @brief 检查文件扩展名是否支持
     * @param filename 文件名
     * @param extensions 支持的扩展名列表
     * @return 是否支持
     */
    bool isSupportedExtension(const std::string& filename, 
                             const std::vector<std::string>& extensions);


    /**
     * @brief 创建必要的目录
     */
    void createDirectories();
    void cleanupPreviousData();
    /**
     * @brief 删除目录中的所有文件
     * @param dir_path 目录路径
     * @param pattern 文件匹配模式
     */
    void clearDirectory(const std::string& dir_path, const std::string& pattern);

    bool saveDataPoint(int index, 
        const comm_alg::MarkerInfo& marker_info, 
        const cv::Mat& image,
        const cv::Mat& rendered_image);

private:
    // 配置相关
    std::string work_mode_;                     ///< 工作模式: "camera" 或 "file"
    std::string camera_config_file_;           ///< 相机配置文件路径
    std::string image_folder_;                 ///< 图片文件夹路径
    std::vector<std::string> image_extensions_; ///< 支持的图片扩展名
    std::string image_topic_;
    
    // 相机相关
    CameraManager* camera_manager_;            ///< 相机管理器
    short camera_id_;                          ///< 相机ID
    cv::Mat camera_matrix_;                    ///< 相机内参矩阵
    cv::Mat dist_coeffs_;                      ///< 畸变系数
    
    // Aruco检测相关
    std::unique_ptr<aruco_alg::ArucoDetector> aruco_detector_;  ///< Aruco检测器
    
    // 可视化相关
    std::unique_ptr<visualization::VisualizationMgr> visualizer_;  ///< 可视化管理器
    
    // 文件模式相关
    std::vector<std::string> image_files_;     ///< 图片文件列表
    size_t current_image_index_;              ///< 当前图片索引
    int current_file_index_;

    std::string base_save_dir_;             ///< 基础保存目录
    std::string coords_save_dir_;           ///< 坐标数据保存目录
    std::string images_save_dir_;           ///< 图像数据保存目录
    std::string aruco_images_save_dir_;     ///< Aruco渲染图像保存目录
    
    // 程序控制
    bool is_running_;                         ///< 程序是否运行中
    std::thread worker_thread_;               ///< 工作线程
    std::thread reservice_thread_;            ///< 持续调用reservice的线程
    std::atomic<bool> reservice_thread_running_;  ///< 控制reservice线程是否运行
    bool is_result_sub; ///< 是否订阅
    bool is_result_service_; ///< 是否提供服务
#ifdef __ROS_ENV__
    // ROS相关
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  ///< 图像发布器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  ///< 图像订阅器
    rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr result_sub_;  ///< 结果订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr calib_result_sub_;  ///< 标定结果订阅器
    rclcpp::Client<custom_msgs_comm::srv::GetMarkerDetection>::SharedPtr aruco_detection_client_;   ///< Aruco检测结果获取服务客户端
    rclcpp::Client<custom_msgs_comm::srv::GetMarkersInfo>::SharedPtr aruco_makers_info_client_;   ///< Aruco标记信息获取服务客户端
#endif
};

#endif // CAM_SDK_ARUCO_TEST_HPP