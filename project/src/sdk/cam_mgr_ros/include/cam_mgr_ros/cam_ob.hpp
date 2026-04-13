#ifndef CAM_MGR_ROS__CAM_OB_HPP_
#define CAM_MGR_ROS__CAM_OB_HPP_

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
#include "cam_mgr_ros/cam_base.hpp"

namespace cam_mgr_ros
{

/**
 * @brief Orbbec相机管理类
 */
class CamOb : public CamBase
{
public:
    CamOb();
    ~CamOb();
    
public:
    /**
     * @brief 启动指定 Orbbec相机 ID 的相机进程
     * @param cam_id 相机 ID
     * @param sense_id 场ID
     * @param cam_info 相机配置信息（引用参数）
     * @param camera_state 相机状态信息（引用参数）
     * @return 是否启动成功
     */
    bool open_cam(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state) override;

    /**
     * @brief 停止指定 Orbbec相机 ID 的相机进程
     * @param cam_id 相机 ID
     * @param camera_state 相机状态信息（引用参数）
     * @return 是否停止成功
     */
    bool close_cam(int cam_id, CamMgr::CamRunInfo& camera_state) override;
    
    /**
     * @brief 切换 Orbbec相机 场景参数
     * @param cam_id 相机 ID
     * @param sense_id 场ID
     * @param cam_info 相机配置信息（引用参数）
     * @param camera_state 相机状态信息（引用参数）
     * @return 是否切换成功
     */
    bool switch_cam_sence(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state) override;

    /**
     * @brief 获取 Orbbec相机 的 ROI 和 FPS 列表（静态函数）
     * @param cam_id 相机 ID
     * @param sensor_roi_fps_map 传感器 ROI 和 FPS 列表输出参数
     * @param cam_info 相机配置信息（引用参数）
     * @return 是否获取成功
     */
    static bool get_roi_fps_list_static(int cam_id, std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map, const CamMgr::CamInfo& cam_info);
    
    /**
     * @brief 设置传感器 ROI 和 FPS 映射表
     * @param sensor_roi_fps_map 传感器 ROI 和 FPS 映射表
     */
    void set_sensor_roi_fps_map(const std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map);
    
    /**
     * @brief 枚举 Orbbec相机 设备（静态函数）
     * @param cam_dev_list Orbbec相机设备信息列表输出参数
     * @return 是否枚举成功
     */
    static bool get_all_cameras_static(CamMgr::CamDevInfoList& cam_dev_list);

    // 子类实现内参/图像话题名获取接口
    std::string get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info) override;
    std::string get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info) override;

private:
    // Orbbec 相机特定的私有成员
};

}  // namespace cam_mgr_ros

#endif  // CAM_MGR_ROS__CAM_OB_HPP_