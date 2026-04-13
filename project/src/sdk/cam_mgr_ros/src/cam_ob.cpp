#include "cam_mgr_ros/cam_ob.hpp"
#include "cam_mgr_ros/cam_base.hpp"
#include "cam_config_mgr/cam_com_struct.hpp"
#include <sys/wait.h>
// 添加Orbbec SDK头文件
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Sensor.hpp>
#include <libobsensor/hpp/StreamProfile.hpp>

namespace cam_mgr_ros
{

CamOb::CamOb() : CamBase()
{
    // 初始化Orbbec相机相关资源
    set_camera_type(CamMgr::CamType::CAM_TYPE_OB);
}

CamOb::~CamOb()
{
    // 清理Orbbec相机相关资源
}

bool CamOb::open_cam(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        //检查场景ID是否存在
        if (cam_info.sence_para.find(sense_id) == cam_info.sence_para.end())
        {
            LOG_ERROR("相机 %d 不存在场景ID %d", cam_id, sense_id);
            return false;
        }

        // 获取指定场景的参数
        const CamMgr::CamSencePara& sence_para = cam_info.sence_para.at(sense_id);

        //构建Orbbec相机启动命令
        std::string cmd = "ros2 launch orbbec_camera orbbec_camera.launch.py";
        if (!cam_info.serial_number.empty())
        {
            cmd += " serial_no:=" + cam_info.serial_number;
        }
        
        //可以添加更多参数，如设备编号等
        cmd += " camera_name:=ob_camera_" + std::to_string(cam_id);

        //根据启用的流设置参数（使用场景参数）
        cmd += " enable_color:=" + std::string(sence_para.enable_color_stream ? "true" : "false");
        cmd += " enable_depth:=" + std::string(sence_para.enable_depth_stream ? "true" : "false");
        cmd += " enable_ir:=" + std::string(sence_para.enable_ir_stream ? "true" : "false");
        cmd += " enable_point_cloud:=" + std::string(sence_para.enable_cloud_stream ? "true" : "false");

        // 设置曝光模式、曝光值和增益参数
        if (sence_para.enable_color_stream)
        {
            // 彩色流曝光设置
            cmd += " enable_color_auto_exposure:=" + std::string(sence_para.color_para.auto_exposure ? "true" : "false");
            if (!sence_para.color_para.auto_exposure && sence_para.color_para.exposure > 0)
            {
                // 手动曝光模式，设置曝光值
                cmd += " color_exposure:=" + std::to_string(static_cast<int>(sence_para.color_para.exposure));
            }
            if (sence_para.color_para.gain > 0)
            {
                // 设置增益值
                cmd += " color_gain:=" + std::to_string(static_cast<int>(sence_para.color_para.gain));
            }
        }

        if (sence_para.enable_depth_stream)
        {
            // 深度流曝光设置（奥比相机深度流使用红外参数）
            cmd += " enable_ir_auto_exposure:=" + std::string(sence_para.depth_para.auto_exposure ? "true" : "false");
            if (!sence_para.depth_para.auto_exposure && sence_para.depth_para.exposure > 0)
            {
                // 手动曝光模式，设置曝光值
                cmd += " ir_exposure:=" + std::to_string(static_cast<int>(sence_para.depth_para.exposure));
            }
            if (sence_para.depth_para.gain > 0)
            {
                // 设置增益值
                cmd += " ir_gain:=" + std::to_string(static_cast<int>(sence_para.depth_para.gain));
            }
        }
        
        // 设置ROI和FPS参数 -先查找最接近的匹配
        if (sence_para.enable_color_stream)
        {
            // 确定彩色流参数：优先使用匹配的配置，否则使用默认值
            int color_width = sence_para.color_para.width;
            int color_height = sence_para.color_para.height;
            int color_fps = sence_para.color_para.fps;
            
            auto it = sensor_roi_fps_map_.find("color");
            if (it != sensor_roi_fps_map_.end() && !it->second.empty())
            {
                // 查找最接近的 ROI/FPS 配置
                int closest_width, closest_height, closest_fps;
                if (CamBase::find_closest_roi_fps(it->second, 
                                                   sence_para.color_para.width, 
                                                   sence_para.color_para.height, 
                                                   sence_para.color_para.fps,
                                                   closest_width, closest_height, closest_fps))
                {
                    color_width = closest_width;
                    color_height = closest_height;
                    color_fps = closest_fps;
                }
            }
            LOG_INFO("相机 %d彩色流使用配置的 ROI/FPS置：%dx%d@%d", 
                     cam_id, color_width, color_height, color_fps);
            cmd += " color_width:=" + std::to_string(color_width);
            cmd += " color_height:=" + std::to_string(color_height);
            cmd += " color_fps:=" + std::to_string(color_fps);
        }

        if (sence_para.enable_depth_stream)
        {
            // 确定深度流参数：优先使用匹配的配置，否则使用默认值
            int depth_width = sence_para.depth_para.width;
            int depth_height = sence_para.depth_para.height;
            int depth_fps = sence_para.depth_para.fps;
            
            auto it = sensor_roi_fps_map_.find("depth");
            if (it != sensor_roi_fps_map_.end() && !it->second.empty())
            {
                // 查找最接近的 ROI/FPS 配置
                int closest_width, closest_height, closest_fps;
                if (CamBase::find_closest_roi_fps(it->second, 
                                                   sence_para.depth_para.width, 
                                                   sence_para.depth_para.height, 
                                                   sence_para.depth_para.fps,
                                                   closest_width, closest_height, closest_fps))
                {
                    depth_width = closest_width;
                    depth_height = closest_height;
                    depth_fps = closest_fps;
                }
            }
            LOG_INFO("相机 %d 深度流使用匹配的 ROI/FPS 配置：%dx%d@%d", 
                             cam_id, depth_width, depth_height, depth_fps);
            cmd += " depth_width:=" + std::to_string(depth_width);
            cmd += " depth_height:=" + std::to_string(depth_height);
            cmd += " depth_fps:=" + std::to_string(depth_fps);
        }

        LOG_INFO("启动Orbbec相机 %d场 %d进程: %s", cam_id, sense_id, cmd.c_str());

        // 创建子进程启动相机
        pid_t pid = fork();
        if (pid == 0)
        {
            //子进程
            execl("/bin/sh", "sh", "-c", cmd.c_str(), (char*)NULL);
            exit(127);
        }
        else if (pid > 0)
        {
            //父进程
            camera_state.cam_pid = pid;
            camera_state.is_cam_open = true;
            camera_state.is_offline = false;
            // 根据场景参数初始化流标志（期望状态）
            camera_state.is_color_stream_start = sence_para.enable_color_stream;
            camera_state.is_depth_stream_start = sence_para.enable_depth_stream;
            camera_state.is_ir_stream_start = sence_para.enable_ir_stream;
            camera_state.is_cloud_stream_start = sence_para.enable_cloud_stream;
            LOG_INFO("Orbbec 相机 %d 场 %d 进程已启动，PID: %d", cam_id, sense_id, pid);
                    
            // 设置相机节点名和话题名
             camera_state.cam_node_name = "/ob_camera_" + std::to_string(cam_id) + "/ob_camera_" + std::to_string(cam_id);
             LOG_INFO("Orbbec 相机 %d 名：%s", cam_id, camera_state.cam_node_name.c_str());
                    
            return true;
        }
        else
        {
            // fork失败
            LOG_ERROR("启动Orbbec相机 %d场景 %d进程失败", cam_id, sense_id);
            camera_state.is_offline = true;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("启动Orbbec相机 %d场景 %d进时程时发生错误: %s", cam_id, sense_id, e.what());
        camera_state.is_offline = true;
        return false;
    }
}

bool CamOb::close_cam(int cam_id, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        // 如果cam_node_pid为0，尝试重新查找
        if (camera_state.cam_node_pid == 0 && camera_state.cam_pid > 0)
        {
            LOG_INFO("相机 %d 的节点进程ID为空，尝试重新查找", cam_id);
            std::vector<std::string> candidates = {"orbbec_camera_node", "camera_node"};
            std::string ros_node_name = "/ob_camera_" + std::to_string(cam_id);
            pid_t node_pid = find_descendant_exec_pid(camera_state.cam_pid, candidates, ros_node_name, 2000);
            if (node_pid > 0)
            {
                camera_state.cam_node_pid = node_pid;
                LOG_INFO("重新找到Orbbec相机 %d 节点进程，PID: %d", cam_id, node_pid);
            }
        }

        if (camera_state.cam_node_pid > 0)
        {
            // 先终止节点进程
            kill(camera_state.cam_node_pid, SIGTERM);
            int status_node;
            waitpid(camera_state.cam_node_pid, &status_node, 0);
            LOG_INFO("Orbbec相机 %d 节点进程已停止，PID: %d", cam_id, camera_state.cam_node_pid);
        }

        if (camera_state.cam_pid > 0)
        {
            //终进程
            kill(camera_state.cam_pid, SIGTERM);

            //等待进程结束
            int status;
            waitpid(camera_state.cam_pid, &status, 0);

            LOG_INFO("Orbbec相机 %d进程已停止，PID: %d", cam_id, camera_state.cam_pid);
        }

        // 更新运行状态
        camera_state.is_cam_open = false;
        camera_state.cam_pid = 0;
        camera_state.cam_node_pid = 0;
        // 不重置is_offline标志，保持掉线状态以便重试

        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("停止Orbbec相机 %d 进程时发生错误: %s", cam_id, e.what());
        return false;
    }
}

bool CamOb::switch_cam_sence(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        LOG_INFO("切换 Orbbec 相机 %d 场景 %d：重启相机以应用参数", cam_id, sense_id);
        // Orbbec 相机通常无法动态修改分辨率等参数，先关闭再打开
        
        // 设置切换标志，避免监控线程误判为掉线
        camera_state.is_switching_scene = true;
        
        close_cam(cam_id, camera_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // 重新打开指定场景
        bool ret = open_cam(cam_id, sense_id, cam_info, camera_state);
        if (ret)
        {
            camera_state.cur_sence_id = sense_id;
            camera_state.is_switching_scene = false;  // 清除切换标志
            LOG_INFO("Orbbec 相机 %d 场景 %d 切换成功", cam_id, sense_id);
        }
        else
        {
            camera_state.is_switching_scene = false;  // 清除切换标志
            LOG_WARN("Orbbec 相机 %d 场景 %d 切换失败", cam_id, sense_id);
        }
        return ret;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("切换 Orbbec 相机 %d 场景 %d 时发生错误：%s", cam_id, sense_id, e.what());
        return false;
    }
}

void CamOb::set_sensor_roi_fps_map(const std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map)
{
    sensor_roi_fps_map_ = sensor_roi_fps_map;
}

bool CamOb::get_roi_fps_list_static(int cam_id, std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map, const CamMgr::CamInfo& cam_info)
{
    try
    {
        LOG_INFO("获取Orbbec相机 %d 的ROI和FPS列表", cam_id);

        ob::Context ctx;
        auto device_list = ctx.queryDeviceList();

        // 根据序列号查找匹配的设备
        std::shared_ptr<ob::Device> target_device;
        bool found = false;
        for (uint32_t i = 0; i < device_list->getCount(); i++)
        {
            std::string dev_serial = device_list->getSerialNumber(i);
            if (dev_serial == cam_info.serial_number)
            {
                target_device = device_list->getDevice(i);
                found = true;
                break;
            }
        }

        if (!found || !target_device)
        {
            LOG_WARN("未找到序列号为 %s 的Orbbec相机", cam_info.serial_number.c_str());
            return false;
        }

        sensor_roi_fps_map.clear();

        auto sensor_list = target_device->getSensorList();
        if (!sensor_list)
        {
            LOG_WARN("无法获取Orbbec相机 %d 的传感器列表", cam_id);
            return false;
        }

        for (uint32_t i = 0; i < sensor_list->getCount(); i++)
        {
            auto sensor = sensor_list->getSensor(i);
            std::string sensor_name = "unknown";

            auto sensor_type = sensor->getType();
            switch (sensor_type)
            {
            case OB_SENSOR_COLOR:
                sensor_name = "color";
                break;
            case OB_SENSOR_DEPTH:
                sensor_name = "depth";
                break;
            case OB_SENSOR_IR:
                sensor_name = "ir";
                break;
            case OB_SENSOR_IR_LEFT:
                sensor_name = "ir_left";
                break;
            case OB_SENSOR_IR_RIGHT:
                sensor_name = "ir_right";
                break;
            default:
                sensor_name = "unknown";
                break;
            }

            auto stream_profiles = sensor->getStreamProfileList();
            CamMgr::CamRoiFpsList roi_fps_list;
            if (!stream_profiles)
                continue;

            for (uint32_t j = 0; j < stream_profiles->getCount(); j++)
            {
                auto stream_profile = stream_profiles->getProfile(j);
                if (stream_profile->is<ob::VideoStreamProfile>())
                {
                    auto video_profile = stream_profile->as<ob::VideoStreamProfile>();
                    CamMgr::CamRoiFps roi_fps;
                    roi_fps.width = video_profile->getWidth();
                    roi_fps.height = video_profile->getHeight();
                    roi_fps.fps = video_profile->getFps();

                    bool exists = false;
                    for (const auto &existing : roi_fps_list)
                    {
                        if (existing.width == roi_fps.width && existing.height == roi_fps.height && existing.fps == roi_fps.fps)
                        {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists)
                        roi_fps_list.push_back(roi_fps);
                }
            }

            if (!roi_fps_list.empty())
            {
                sensor_roi_fps_map[sensor_name] = roi_fps_list;
            }
        }

        LOG_INFO("成功获取Orbbec相机 %d 的ROI和FPS列表", cam_id);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("获取Orbbec相机 %d 的ROI和FPS列表时发生错误: %s", cam_id, e.what());
        return false;
    }
}
bool CamOb::get_all_cameras_static(CamMgr::CamDevInfoList& cam_dev_list)
{
    static std::mutex static_mutex;
    std::lock_guard<std::mutex> lock(static_mutex);
    try
    {
        // 使用Orbbec SDK枚举连接的相机设备
        ob::Context ctx;
        auto device_list = ctx.queryDeviceList();

        //所有连接的设备
        for (uint32_t i = 0; i < device_list->getCount(); i++)
        {
            // 获取设备信息
            CamMgr::CamDevInfo dev_info;
            dev_info.cam_type = CamMgr::CamType::CAM_TYPE_OB; // Orbbec相机
            dev_info.device_id = std::to_string(i);           // 使用索引作为设备ID
            dev_info.product_id = std::to_string(device_list->getPid(i));
            dev_info.device_name = device_list->getName(i);
            dev_info.serial_number = device_list->getSerialNumber(i);
            dev_info.physical_port = device_list->getConnectionType(i);
            dev_info.user_name = device_list->getName(i); // 使用设备名称作为用户名称
            dev_info.facturer_name = "Orbbec";            //制商名称
            // 注意：Orbbec SDK没有直接提供固件版本信息

            cam_dev_list.push_back(dev_info);
        }

        LOG_INFO("枚举到 %zu 个Orbbec相机设备", cam_dev_list.size());
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("枚举Orbbec相机设备时发生错误: %s", e.what());
        return false;
    }
}

// Orbbec 子类实现内参/图像话题名
std::string CamOb::get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        return "/ob_camera_" + std::to_string(cam_id) + "/color/camera_info";
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        return "/ob_camera_" + std::to_string(cam_id) + "/depth/camera_info";
    return std::string();
}

std::string CamOb::get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        return "/ob_camera_" + std::to_string(cam_id) + "/color/image_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        return "/ob_camera_" + std::to_string(cam_id) + "/depth/image_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_IR)
        return "/ob_camera_" + std::to_string(cam_id) + "/ir/image_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_CLOUD)
        return "/ob_camera_" + std::to_string(cam_id) + "/depth/points";
    return std::string();
}

}  // namespace cam_mgr_ros
