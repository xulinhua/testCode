#ifndef CAM_MGR_ROS__CAM_BASE_HPP_
#define CAM_MGR_ROS__CAM_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include "custom_msgs_comm/srv/camera_control.hpp"
#include <cam_config_mgr/camera_config_manager.hpp>
#include <cam_config_mgr/cam_com_struct.hpp>
#include "log_system/log_macros.hpp"
#include "visualization/visualization_mgr.hpp"
#include "custom_msgs_comm/srv/get_cam_intr.hpp"
#include <mutex>
#include <map>


namespace cam_mgr_ros
{

/**
 * @brief 相机抽象基类
 */
class CamBase
{
public:
    CamBase();
    virtual ~CamBase();
    
    /**
     * @brief 根据设备名称和制造商名称获取相机类型
     * @param device_name 设备名称
     * @param facturer_name 制造商名称
     * @return 相机类型
     */
    static CamMgr::CamType get_cam_type_by_str(const std::string& device_name, const std::string& facturer_name);
    
    /**
     * @brief 根据设备名称获取相机型号类型
     * @param device_name 设备名称
     * @return 相机型号类型
     */
    static CamMgr::CamModelType get_device_type_by_str(const std::string& device_name);
    
    /**
     * @brief 杀死可能占用相机的进程
     */
    static void kill_camera_processes();
    
    /**
     * @brief 查找最接近的ROI和FPS配置
     * @param roi_fps_list ROI和FPS列表
     * @param target_width 目标宽度
     * @param target_height 目标高度
     * @param target_fps 目标帧率
     * @param closest_width 输出：最接近的宽度
     * @param closest_height 输出：最接近的高度
     * @param closest_fps 输出：最接近的帧率
     * @return 是否成功找到匹配
     */
    static bool find_closest_roi_fps(const std::vector<CamMgr::CamRoiFps>& roi_fps_list, int target_width, int target_height, int target_fps,
                                     int& closest_width, int& closest_height, int& closest_fps);
    
public:
    /**
     * @brief启动指定相机 ID 的相机进程 - 纯虚函数
     * @param cam_id相机 ID
     * @param sense_id场 ID
     * @param cam_info相机配置信息（引用参数）
     * @param camera_state相机状态信息（引用参数）
     * @return 是否启动成功
     */
    virtual bool open_cam(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state) = 0;
    
    /**
     * @brief停指定相机 ID 的相机进程 - 纯虚函数
     * @param cam_id相机 ID
     * @param camera_state相机状态信息（引用参数）
     * @return 是否停止成功
     */
    virtual bool close_cam(int cam_id, CamMgr::CamRunInfo& camera_state) = 0;
        
    /**
     * @brief切换相机场景参数 - 纯虚函数
     * @param cam_id相机 ID
     * @param sense_id场 ID
     * @param cam_info 相机配置信息（引用参数）
     * @param camera_state相机状态信息（引用参数）
     * @return 是否切换成功
     */
    virtual bool switch_cam_sence(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state) = 0;
    
    /**
     * @brief 初始化相机 - 虚函数（可选重写）
     * @return 是否初始化成功
     */
    virtual bool initialize();

    /**
     * @brief 销毁相机资源 - 虚函数（可选重写）
     */
    virtual void destroy();

    /**
     * @brief 获取相机状态 - 虚函数（可选重写）
     * @param cam_id 相机ID
     * @return 相机状态
     */
    virtual bool get_camera_status(int cam_id);

    /**
     * @brief 设置相机参数 - 虚函数（可选重写）
     * @param cam_id 相机ID
     * @param param_name 参数名
     * @param param_value 参数值
     * @return 是否设置成功
     */
    virtual bool set_camera_param(int cam_id, const std::string& param_name, const std::string& param_value);

    /**
     * @brief 获取相机内参话题名（由子类实现或使用基类默认实现）
     */
    virtual std::string get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info) = 0;
    /**
     * @brief 获取相机图像话题名（由子类实现或使用基类默认实现）
     */
    virtual std::string get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info) = 0;

    /**
     * @brief 设置传感器ROI和FPS映射表（由子类实现）
     * @param sensor_roi_fps_map 传感器ROI和FPS映射表
     */
    virtual void set_sensor_roi_fps_map(const std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map) = 0;

protected:
    // 相机基类通用的保护成员
    bool initialized_;
    CamMgr::CamType camera_type_;
    
    // 相机配置信息映射表（cam_id -> CamInfo），所有相机子类共用
    std::map<int, CamMgr::CamInfo> cam_info_map_;
    mutable std::mutex cam_info_mutex_; // 保护 cam_info_map_ 的互斥量
    
    // 传感器 ROI 和 FPS 映射表，所有相机子类共用
    std::map<std::string, CamMgr::CamRoiFpsList> sensor_roi_fps_map_;
    mutable std::mutex get_all_cameras_mutex_; // 用于 get_all_cameras 函数的互斥量
    
    /**
     * @brief 设置相机类型
     * @param type 相机类型
     */
    void set_camera_type(CamMgr::CamType type);
    
    /**
     * @brief 获取相机类型
     * @return 相机类型
     */
    CamMgr::CamType get_camera_type() const;

    /**
     * @brief 在给定启动进程 PID 的基础上查找其派生出的子进程中，匹配指定可执行名和ROS节点名的进程 PID。
     * @param root 根启动进程 PID
     * @param exec_names 候选的可执行名列表（如 realsense2_camera_node, orbbec_camera_node, camera_node）
     * @param ros_node_name ROS节点名（如 /ob_camera_1）
     * @param timeout_ms 最长等待时间（毫秒），默认 500ms
     * @return 找到的进程 PID，未找到返回 0
     */
    pid_t find_descendant_exec_pid(pid_t root, const std::vector<std::string>& exec_names, const std::string& ros_node_name, int timeout_ms = 500);
};

}  // namespace cam_mgr_ros

#endif  // CAM_MGR_ROS__CAM_BASE_HPP_