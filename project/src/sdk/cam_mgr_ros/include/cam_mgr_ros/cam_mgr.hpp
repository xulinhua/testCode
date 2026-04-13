#ifndef CAM_MGR_ROS__CAM_MGR_ROS_HPP_
#define CAM_MGR_ROS__CAM_MGR_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>  // 添加 CameraInfo 消息头文件
#include <sensor_msgs/msg/image.hpp>        // 添加 Image 消息头文件
#include <sensor_msgs/msg/point_cloud2.hpp> // 添加 PointCloud2 消息头文件
#include <std_msgs/msg/string.hpp>          // 添加 String 消息头文件
#include <string>
#include <optional>
// 添加服务消息头文件
#include "cam_mgr_ros/cam_base.hpp"
#include "cam_mgr_ros/cam_rs.hpp"
#include "cam_mgr_ros/cam_ob.hpp"
#include "cam_mgr_ros/cam_csi.hpp"
// 添加cam_config_mgr头文件
#include <cam_config_mgr/camera_config_manager.hpp>
#include <cam_config_mgr/cam_com_struct.hpp>
// Include log system
#include "log_system/log_macros.hpp"
// Include visualization system
#include "visualization/visualization_mgr.hpp"
// Include camera intrinsics handler
#include "custom_msgs_comm/srv/get_cam_intr.hpp"  // 添加 GetCamIntr 服务头文件
#include "custom_msgs_comm/srv/set_image_save.hpp"  // 添加 SetImageSave 服务头文件
#include <bas_operate_ros/module_status.hpp>  // 添加 ModuleStatusInfo 结构体定义
#include <mutex>

namespace cam_mgr_ros
{

class CamMgrRos : public rclcpp::Node, public std::enable_shared_from_this<CamMgrRos>
{
public:
    CamMgrRos();
    ~CamMgrRos();
    
public:
    ////////////硬件操作接口：相机开关，场景切换等操作////////////
    /**
     * @brief 启动所有相机进程
     */
    bool open_cam_all();
    
    /**
     * @brief 启动指定相机 ID 的相机进程
     * @param cam_id 相机 ID
     * @param sense_id 场景 ID
     * @param cam_info_opt 可选的临时相机配置
     */
    bool open_cam(int cam_id, int sense_id = 0, std::optional<CamMgr::CamInfo> cam_info_opt = std::nullopt);
    
    /**
     * @brief 停止所有相机进程
     */
    void close_cam_all();
        
    /**
     * @brief 停止指定相机 ID 的相机进程
     * @param cam_id 相机 ID
     */
    bool close_cam(int cam_id, bool is_offline_close = false);
        
    /**
     * @brief 切换相机场景参数
     * @param cam_id 相机 ID
     * @param sense_id 场景 ID
     * @return 是否切换成功
     */
    bool switch_cam_sence(int cam_id, int sense_id);
    


private:
    ////////////硬件参数读写：相机设备枚举和参数获取////////////

    /**
     * @brief 遍历所有相机设备，并打印设备信息
     * @param cam_dev_list 相机设备信息列表输出参数
     * @return 是否遍历成功
     */
    bool get_all_cameras();
    
    /**
     * @brief 获取并打印指定相机支持的ROI和FPS参数
     * @param cam_id 相机ID
     * @param cam_type 相机类型
     */
    void get_camera_roi_fps();

    /**
     * @brief 保存相机ROI和FPS信息
     * @param type 保存类型，0-保存为txt文件，1-保存为yaml配置文件
     * @return 是否保存成功
     */
    bool save_camera_roi_fps_infos(int type);
    
    /**
     * @brief 读取相机ROI和FPS信息
     * @return 是否读取成功
     */
    bool load_camera_roi_fps_infos();
    
    /**
     * @brief 获取并保存所有相机的图像话题名
     * @return 是否保存成功
     */
    bool get_cam_image_topics();
    
    /**
     * @brief 获取并保存所有相机的内参话题名
     * @return 是否保存成功
     */
    bool get_cam_info_topics();

    /**
     * @brief 获取指定相机 ID 和数据流类型的图像话题名称
     * @param cam_id 相机 ID
     * @param stream_type 数据流类型
     * @return 图像话题名称
     */
    std::string get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type);
    
    /**
     * @brief 获取指定相机 ID 和数据流类型的内参话题名称
     * @param cam_id 相机 ID
     * @param stream_type 数据流类型
     * @return 内参话题名称
     */
    std::string get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type);

private:
    ////////////文件操作：配置文件读取和数据保存////////////

    /**
     * @brief 加载配置文件
     * @param mode 加载模式，0-从文件加载，1-先从参数服务器加载，再从文件加载
     * @return 是否加载成功
     */
    bool load_config(int mode = 0);

    /**
     * @brief 从参数服务器加载相机配置服务
     * @return 是否加载成功
     */
    bool load_cam_config_srv();
    
    /**
     * @brief 保存相机内参到JSON文件
     * @param cam_id 相机ID
     * @param forward_msg 相机内参消息
     */
    void save_camera_intrinsics_to_json(int cam_id, const sensor_msgs::msg::CameraInfo& forward_msg);

    /**
     * @brief 保存所有遍历到的相机信息到all_cameras_info.yaml
     * @return 是否保存成功
     */
    bool save_all_cameras_info();
    
    /**
     * @brief 保存单个相机信息到对应文件夹下的cam_info.yaml
     * @param cam_id 相机ID
     * @return 是否保存成功
     */
    bool save_single_camera_info(int cam_id);

private:
    ////////////ROS相关：话题、服务、动作等相关接口////////////
    /**
     * @brief 发布所有相机内参
     */
    void publish_all_camera_info();
    
    /**
     * @brief 发布指定相机ID的内参
     * @param cam_id 相机ID
     */
    void publish_camera_info(int cam_id);

    /**
     * @brief 订阅所有已开启相机的图像话题
     */
    void subscribe_to_camera_images_clouds();

     /**
     * @brief 订阅所有已开启相机的内参话题
     */
    void subscribe_to_all_camera_info();
        
    /**
     * @brief 图像话题回调函数（带流类型版本）
     * @param msg 图像消息
     * @param cam_id 相机ID
     * @param stream_type 流类型
     */
    void image_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type);
    
    /**
     * @brief 图像保存回调函数（带流类型版本）- 专用于保存图像
     * @param msg 图像消息
     * @param cam_id 相机ID
     * @param stream_type 流类型
     */
    void image_save_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type);
    
    /**
     * @brief 图像显示回调函数（带流类型版本）- 专用于显示图像
     * @param msg 图像消息
     * @param cam_id 相机ID
     * @param stream_type 流类型
     */
    void image_display_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type);
    
    /**
     * @brief 点云回调函数
     * @param msg 点云消息
     * @param cam_id 相机ID
     */
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int cam_id);

    /**
     * @brief 检查并订阅所有已打开相机的图像话题
     */
    void check_and_subscribe_images();

    /**
     * @brief 相机内参话题回调函数
     * @param msg 相机内参消息
     * @param cam_id 相机ID
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, int cam_id);

    /**
     * @brief 订阅相机内参话题
     * @param cam_id 相机ID
     * @param camera_info_topic 相机内参话题名称
     * @return 是否订阅成功
     */
    bool subscribe_to_camera_info(int cam_id, const std::string& camera_info_topic);
    
    /**
     * @brief 相机控制服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     */
    void camera_control_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::CameraControl::Request> request,
        const std::shared_ptr<custom_msgs_comm::srv::CameraControl::Response> response);
    
    /**
     * @brief 相机内参服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     */
    void get_cam_intr_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::GetCamIntr::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::GetCamIntr::Response> response);

    /**
     * @brief 设置图像保存服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     */
    void set_image_save_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::SetImageSave::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::SetImageSave::Response> response);
    
    /**
     * @brief 存图超时检查定时器回调函数
     */
    void save_timeout_check_callback();

private:

    /**
     * @brief 查找支持的最低FPS（保持图像长宽不变）
     * @param cam_id 相机ID
     * @param stream_type 流类型
     * @param target_width 目标宽度
     * @param target_height 目标高度
     * @param min_fps 输出：最低支持的帧率
     * @return 是否成功找到
     */
    bool find_min_supported_fps(int cam_id, CamMgr::CamStreamType stream_type, int target_width, int target_height, int& min_fps);
    
    // 定时器回调函数（已移除，如需定时器功能请使用 create_wall_timer 并绑定具体回调）
    
    /**
     * @brief 检查是否已接收到内参数据
     * @param cam_id 相机ID
     * @return 是否已接收到内参数据
     */
    bool is_intrinsics_received(int cam_id) const;

    /**
     * @brief 设置内参转发功能的启用状态
     * @param enable 是否启用内参转发功能
     */
    void set_camera_info_forward_enable(bool enable);
    
    /**
     * @brief 杀死可能占用相机的进程
     */
    void kill_camera_processes();
    
    /**
     * @brief 设置遍历到的相机信息到配置参数
     * @return 是否设置成功
     */
    bool set_camera_info_to_config();    
    
private:
    // 相机配置管理器实例，用于加载和管理相机配置信息
    CamMgr::CamConfigMgr config_mgr_;
    
    // 相机设备信息列表
    CamMgr::CamDevInfoList cam_dev_list_;
    
    // 相机状态映射表，每个相机ID对应一个运行时信息
    std::map<int, CamMgr::CamRunInfo> cam_run_info_;
    
    // 相机支持的roi fps映射表
    std::map<int,std::map<std::string, CamMgr::CamRoiFpsList>> cam_roi_fps_map_;
    
    // 从YAML文件加载的相机支持的roi fps映射表，键为序列号
    std::map<std::string,std::map<std::string, CamMgr::CamRoiFpsList>> cam_roi_fps_map_load_;
    
    // 控制是否保存设备信息的标志
    bool is_save_device_info_;
    
    // 标志位，用于标记是否已从流中获取内参
    bool intrinsics_from_stream_received_;
    
    // 相机控制服务
    rclcpp::Service<custom_msgs_comm::srv::CameraControl>::SharedPtr camera_control_service_;
    
    // 相机内参服务服务器
    rclcpp::Service<custom_msgs_comm::srv::GetCamIntr>::SharedPtr get_cam_intr_service_;

    // 设置图像保存服务服务器
    rclcpp::Service<custom_msgs_comm::srv::SetImageSave>::SharedPtr set_image_save_service_;

    // 图像保存配置：cam_id -> stream_type -> 是否保存
    std::map<int, std::map<CamMgr::CamStreamType, bool>> image_save_config_;
    
    // 当前存图批次的时间戳目录
    std::string current_save_batch_timestamp_;
    
    // 存图开始时间（毫秒）
    int64_t save_start_time_ms_;
    
    // 图像保存状态标志
    std::atomic<bool> is_image_saving_{false};      // 是否正在保存图像
    std::atomic<bool> is_bag_recording_{false};     // 是否正在录制 BAG
    
    // 图像保存配置互斥锁
    mutable std::mutex image_save_mutex_;
    
    // ros2 bag录制配置：cam_id -> stream_type -> 是否录制
    std::map<int, std::map<CamMgr::CamStreamType, bool>> bag_record_config_;
    
    // ros2 bag录制进程PID（只有一个进程录制所有话题）
    pid_t bag_record_pid_;
    
    // ros2 bag录制进程组
    pid_t bag_record_pgid_;

    // ros2 bag录制开始时间（毫秒）
    int64_t bag_record_start_time_ms_;
    
    // ros2 bag录制目录
    std::string bag_record_dir_;
    
    // ros2 bag 录制配置互斥锁
    mutable std::mutex bag_record_mutex_;
        
    // 是否删除已解析的 ROS 包文件（默认 false，保留文件）
    bool delete_bag_after_parse_;
        
    // 图像保存格式（默认"bmp"，支持"bmp", "jpg", "png"等）
    std::string image_save_format_;
    
    // 相机内参发布器映射表，每个相机ID对应一个内参发布器
    std::map<int, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publisher_map_;
    
    // 相机内参订阅器映射表（每个相机ID对应一个订阅器）
    std::map<int, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscriber_map_;
    
    // 相机内参消息映射表（每个相机ID对应一个内参消息）
    std::map<int, sensor_msgs::msg::CameraInfo> camera_info_msg_map_;
    
    // 内参接收标志映射表（每个相机ID对应一个接收标志）
    std::map<int, bool> intrinsics_received_map_;
    
    // 内参转发标志映射表（每个相机ID对应一个转发标志）
    std::map<int, bool> has_forwarded_map_;
    
    //@brief 图像订阅器映射表，每个相机 ID 和流类型对应一个图像订阅器
    std::map<int, std::map<CamMgr::CamStreamType, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>> image_subscriber_map_;
    std::mutex image_subscriber_mutex_; ///< 图像订阅器互斥锁
        
    // 点云订阅器映射表，每个相机 ID 对应一个点云订阅器
    std::map<int, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subscriber_map_;
    std::mutex pointcloud_subscriber_mutex_; ///< 点云订阅器互斥锁
    
    // ModuleInfo 相关成员
    std::map<int, basros::ModuleStatusInfo> module_info_map_; ///< 模块状态信息，每个相机一个
    std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> module_info_publisher_map_; ///< 模块信息发布器映射
    rclcpp::TimerBase::SharedPtr module_info_timer_; ///< 模块信息定时器
    
    /**
     * @brief 刷新 ModuleInfo 数据
     * @details 根据 cam_run_info_确定模块状态
     */
    void refreshModuleInfo();
        
    void initModuleStatusInfo(int cam_id);
        
    void publishModuleInfo(int cam_id);
    
    // 用于跟踪每个相机接收的图像数量，用于在获取几张图像后获取内参
    std::map<int, int> camera_image_count_map_;
    
    // 可视化管理器映射表，每个相机ID和流类型组合对应一个可视化管理器
    std::map<std::pair<int, CamMgr::CamStreamType>, std::shared_ptr<visualization::VisualizationMgr>> visualization_manager_map_;
    
    // 定时器用于定期获取和发布内参
    rclcpp::TimerBase::SharedPtr camera_info_timer_;
    
    // 定时器用于定期检查并订阅图像话题
    rclcpp::TimerBase::SharedPtr subscription_timer_;
    
    // 定时器用于检查存图超时
    rclcpp::TimerBase::SharedPtr save_timeout_timer_;
    
    // 控制是否启用内参转发功能的标志
    bool enable_camera_info_forward_;

    // 系统配置节点名称
    std::string sys_config_node_name_; 
    // 系统配置客户端
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;
    
    //相机枚举互斥锁
    mutable std::mutex camera_enumeration_mutex_;
    // 相机状态互斥锁（保护 cam_run_info_ / camera_map_ 等运行时状态）
    mutable std::mutex camera_state_mutex_;

    // 场景切换标志（保护所有相机的监控逻辑）
    std::atomic<bool> is_switching_scene {false};
    int64_t last_scene_switch_time_ms_ {0}; ///< 最后一次场景切换完成的时间戳

    // 监视与重试线程
    std::thread monitor_thread_;
    std::thread retry_thread_;
    std::atomic<bool> threads_running_ {false};

    // 两个线程的时间间隔（毫秒），可通过 setter 修改
    std::atomic<int> monitor_interval_ms_{500}; // 监视已打开相机，默认 500ms（降低优先级，减少资源消耗）
    std::atomic<int> retry_interval_ms_{1000};  // 重试打开相机，默认1000ms
    
    // 图像保存线程（每个相机一个独立线程）
    struct ImageSaveTask {
        sensor_msgs::msg::Image image_msg;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        int cam_id;
        CamMgr::CamStreamType stream_type;
    };
    
    // 每个相机的保存队列和线程（支持图像和点云）
    std::map<int, std::queue<ImageSaveTask>> image_save_queues_;  // cam_id -> queue
    std::map<int, std::mutex> image_save_queue_mutex_;  // cam_id -> mutex
    std::map<int, std::condition_variable> image_save_queue_cv_;  // cam_id -> cv
    std::map<int, std::thread> image_save_threads_;  // cam_id -> thread
    std::map<int, bool> image_save_thread_running_;  // cam_id -> 线程是否运行中
    std::atomic<bool> stop_image_save_threads_;
    
    // 图像统计信息：cam_id + stream_type -> 接收数量 / 保存数量
    std::map<std::pair<int, CamMgr::CamStreamType>, std::atomic<uint64_t>> image_received_count_;  // 接收到的图像总数
    std::map<std::pair<int, CamMgr::CamStreamType>, std::atomic<uint64_t>> image_saved_count_;     // 实际保存的图像数
    std::map<std::pair<int, CamMgr::CamStreamType>, std::chrono::steady_clock::time_point> last_print_time_;  // 上次打印时间
    
    // 线程优先级设置（nice 值，范围 -20~19，越大优先级越低）
    static constexpr int SAVE_THREAD_NICE = 0;      // 保存线程：较低优先级
    static constexpr int MONITOR_THREAD_NICE = 0;   // 监视线程：更低优先级
    static constexpr int RETRY_THREAD_NICE = 0;     // 重试线程：更低优先级
    static constexpr int DISPLAY_THREAD_NICE = 0;    // 显示线程：默认优先级（保证 UI 响应）
    
    // 点云保存回调函数
    void pointcloud_save_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int cam_id);
    
    // 初始化每个相机的保存线程
    void init_camera_image_save_threads();
    // 单个相机的图像保存线程函数（同时处理图像和点云）
    void camera_image_save_thread_func(int cam_id);
    // 启动或停止指定相机的保存线程
    void start_camera_save_thread(int cam_id);
    void stop_camera_save_thread(int cam_id);
    
    // 图像显示线程池
    struct DisplayTask {
        sensor_msgs::msg::Image msg;
        int cam_id;
        CamMgr::CamStreamType stream_type;
    };
    std::queue<DisplayTask> display_tasks_;
    std::mutex display_task_mutex_;
    std::condition_variable display_task_cv_;
    std::vector<std::thread> display_threads_;
    std::atomic<bool> stop_display_threads_;
    
    // 文件夹压缩任务结构
    struct CompressTask {
        std::string source_dir;      // 源目录
        std::string output_zip;      // 输出 zip 文件路径
    };
    std::queue<CompressTask> compress_tasks_;
    std::mutex compress_task_mutex_;
    std::condition_variable compress_task_cv_;
    std::vector<std::thread> compress_threads_;
    std::atomic<bool> stop_compress_threads_;
    
    // 初始化压缩线程池
    void init_compress_threads(int num_threads = 1);
    // 压缩线程函数
    void compress_thread_func();
    // 提交压缩任务到线程池
    void submit_compress_task(const std::string& source_dir, const std::string& output_zip);
    
    // 初始化显示线程池
    void init_display_threads(int num_threads = 1);
    // 显示线程函数
    void display_thread_func();

    // 启动/停止线程的控制函数
    void start_background_threads();
    void stop_background_threads();
    
    // 线程回调实现（私有）
    void monitor_opened_cameras_loop();
    void retry_open_failed_cameras_loop();

    /**
     * @brief 创建所有启用的相机实例
     * @return 是否全部创建成功
     */
    bool create_all_camera_instances();

public:
    // 相机工厂和CamBase指针映射
    std::map<int, std::unique_ptr<cam_mgr_ros::CamBase>> camera_map_;
    
    /**
     * @brief 创建相机实例
     * @param cam_id相机ID
     * @param cam_type相机类型
     * @return 是否创建成功
     */
    bool create_camera_instance(int cam_id, CamMgr::CamType cam_type);
};

}

#endif