#include "cam_mgr_ros/cam_csi.hpp"
#include "cam_mgr_ros/cam_base.hpp"
#include "cam_config_mgr/cam_com_struct.hpp"
#include <sys/wait.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <algorithm>
#include <regex>
#include <signal.h>
#include <chrono>
#include <thread>
// 添加V4L2头文件用于CSI相机支持
#include <linux/videodev2.h>

namespace cam_mgr_ros
{

CamCsi::CamCsi() : CamBase()
{
    set_camera_type(CamMgr::CamType::CAM_TYPE_CSI);
}

CamCsi::~CamCsi()
{
}

// 辅助函数：检查进程是否存在
static bool is_process_alive(pid_t pid)
{
    if (pid <= 0) return false;
    return kill(pid, 0) == 0;
}

// 辅助函数：强制杀死进程树
static void kill_process_tree(pid_t pid, int signal = SIGKILL)
{
    if (pid <= 0) return;
    
    // 杀死整个进程组
    kill(-pid, signal);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 如果进程还在，单独杀死
    if (is_process_alive(pid)) {
        kill(pid, signal);
    }
}

bool CamCsi::open_cam(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        // 1. 检查场景ID是否存在
        if (cam_info.sence_para.find(sense_id) == cam_info.sence_para.end())
        {
            LOG_ERROR("相机 %d 不存在场景ID %d", cam_id, sense_id);
            return false;
        }

        // 保存相机配置信息，用于后续场景切换
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            cam_info_map_[cam_id] = cam_info;
            LOG_INFO("保存 CSI相机 %d 配置信息", cam_id);
        }

        const CamMgr::CamSencePara& sence_para = cam_info.sence_para.at(sense_id);

        // 2. 构建启动命令
        std::string cmd = "ros2 launch csi_cam_service stereo_cam.launch.py";
        
        cmd += " mono_mode:=true";
        cmd += " camera_namespace:=camera_" + std::to_string(cam_id);
        cmd += " camera_name:=csi_camera_" + std::to_string(cam_id);
        cmd += " camera_id:=" + std::to_string(cam_id);
        cmd += " base_frame_id:=camera_" + std::to_string(cam_id) + "_base_link";
        cmd += " left_frame_id:=camera_" + std::to_string(cam_id) + "_camera_link";
        cmd += " publish_tf:=false";
        
        // 4. 设置图像参数
        if (sence_para.enable_color_stream)
        {
            int closest_width = sence_para.color_para.width;
            int closest_height = sence_para.color_para.height;
            int closest_fps = sence_para.color_para.fps;
            
            auto it = sensor_roi_fps_map_.find("color");
            if (it != sensor_roi_fps_map_.end() && !it->second.empty())
            {
                if (CamBase::find_closest_roi_fps(it->second, 
                                                   sence_para.color_para.width, 
                                                   sence_para.color_para.height, 
                                                   sence_para.color_para.fps,
                                                   closest_width, closest_height, closest_fps))
                {
                    LOG_INFO("CSI相机 %d ROI匹配: %dx%d@%d -> %dx%d@%d", 
                             cam_id, 
                             sence_para.color_para.width, sence_para.color_para.height,
                             sence_para.color_para.fps, 
                             closest_width, closest_height, closest_fps);
                }
            }
            
            cmd += " image_width:=" + std::to_string(closest_width);
            cmd += " image_height:=" + std::to_string(closest_height);
            cmd += " framerate:=" + std::to_string(closest_fps);
            
            // 曝光和增益参数
            if (sence_para.color_para.auto_exposure)
            {
                cmd += " auto_exposure:=true";
            }
            else
            {
                cmd += " auto_exposure:=false";
                if (sence_para.color_para.exposure > 0)
                {
                    cmd += " exposure_time:=" + std::to_string(static_cast<int>(sence_para.color_para.exposure));
                }
                if (sence_para.color_para.gain > 0)
                {
                    cmd += " gain:=" + std::to_string(static_cast<int>(sence_para.color_para.gain));
                }
            }
        }
        
        LOG_INFO("启动CSI相机 %d 场景 %d: %s", cam_id, sense_id, cmd.c_str());

        // 3. 启动子进程
        pid_t pid = fork();
        if (pid == 0)
        {
            // 子进程
            execl("/bin/sh", "sh", "-c", cmd.c_str(), (char*)NULL);
            exit(127);
        }
        else if (pid > 0)
        {
            camera_state.cam_pid = pid;
            camera_state.is_cam_open = true;
            camera_state.is_offline = false;
            camera_state.cur_sence_id = sense_id;

            // 设置相机节点名
            // 实际节点名是硬编码的 "stereo_cam_node"，完整路径为 /camera_{cam_id}/stereo_cam_node
            camera_state.cam_node_name = "/camera_" + std::to_string(cam_id) + "/stereo_cam_node";
            LOG_INFO("CSI相机 %d 进程已启动，PID: %d，节点名：%s", cam_id, pid, camera_state.cam_node_name.c_str());

            // 查找节点进程
            // stereo_cam_node 是 Python 脚本，实际进程名为 python3
            // 进程树保证多相机不串台；用 cmdline 含 "stereo_cam_node" 过滤
            // 避免误匹配到 launch 进程的 python3
            try
            {
                std::vector<std::string> candidates = {"python3"};
                pid_t node_pid = find_descendant_exec_pid(pid, candidates, "stereo_cam_node", 8000);
                if (node_pid > 0)
                {
                    camera_state.cam_node_pid = node_pid;
                    LOG_INFO("CSI相机 %d 节点进程 PID: %d", cam_id, node_pid);
                }
                else
                {
                    LOG_WARN("CSI相机 %d 未找到节点进程", cam_id);
                }
            }
            catch (...) {}

            return true;
        }
        else
        {
            LOG_ERROR("启动CSI相机 %d 进程失败", cam_id);
            camera_state.is_offline = true;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("启动CSI相机 %d 错误: %s", cam_id, e.what());
        camera_state.is_offline = true;
        return false;
    }
}

bool CamCsi::close_cam(int cam_id, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        LOG_INFO("CSI相机 %d 开始关闭", cam_id);

        // 如果 cam_node_pid 为0，尝试重新查找
        if (camera_state.cam_node_pid == 0 && camera_state.cam_pid > 0)
        {
            LOG_INFO("相机 %d 的节点进程ID为空，尝试重新查找", cam_id);
            // 进程树隔离保证多相机不串台；用 "stereo_cam_node" 过滤避免匹配 launch 的 python3
            std::vector<std::string> candidates = {"python3"};
            pid_t node_pid = find_descendant_exec_pid(camera_state.cam_pid, candidates, "stereo_cam_node", 2000);
            if (node_pid > 0)
            {
                camera_state.cam_node_pid = node_pid;
                LOG_INFO("重新找到CSI相机 %d 节点进程，PID: %d", cam_id, node_pid);
            }
        }

        // 1. 先终止节点进程
        if (camera_state.cam_node_pid > 0)
        {
            kill(camera_state.cam_node_pid, SIGTERM);
            int status_node;
            waitpid(camera_state.cam_node_pid, &status_node, 0);
            LOG_INFO("CSI相机 %d 节点进程已停止，PID: %d", cam_id, camera_state.cam_node_pid);
        }

        // 2. 终止主进程
        if (camera_state.cam_pid > 0)
        {
            kill(camera_state.cam_pid, SIGTERM);
            int status;
            waitpid(camera_state.cam_pid, &status, 0);
            LOG_INFO("CSI相机 %d 主进程已停止，PID: %d", cam_id, camera_state.cam_pid);
        }

        // 3. 更新状态
        camera_state.is_cam_open = false;
        camera_state.cam_pid = 0;
        camera_state.cam_node_pid = 0;
        // 不重置 is_offline 标志，保持掉线状态以便重试

        LOG_INFO("CSI相机 %d 关闭完成", cam_id);
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("关闭CSI相机 %d 错误: %s", cam_id, e.what());
        return false;
    }
}

bool CamCsi::switch_cam_sence(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        LOG_INFO("========== 切换 CSI 相机 %d 场景 %d ==========", cam_id, sense_id);

        // 优先使用保存的相机配置信息，如果没有则使用传入的参数
        CamMgr::CamInfo actual_cam_info;
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            auto it = cam_info_map_.find(cam_id);
            if (it != cam_info_map_.end())
            {
                actual_cam_info = it->second;
                LOG_INFO("使用保存的 CSI相机 %d 配置信息进行场景切换", cam_id);
            }
            else
            {
                actual_cam_info = cam_info;
                LOG_WARN("未找到保存的 CSI相机 %d 配置信息，使用传入参数", cam_id);
            }
        }

        // 检查场景ID
        if (actual_cam_info.sence_para.find(sense_id) == actual_cam_info.sence_para.end())
        {
            LOG_ERROR("相机 %d 不存在场景ID %d", cam_id, sense_id);
            return false;
        }

        const CamMgr::CamSencePara &sence_para = actual_cam_info.sence_para.at(sense_id);
        const CamMgr::CamSencePara &current_para = actual_cam_info.sence_para.at(camera_state.cur_sence_id);

        LOG_INFO("当前: %dx%d@%d, 曝光自动=%d, 曝光=%d, 增益=%d",
                 current_para.color_para.width, current_para.color_para.height, current_para.color_para.fps,
                 current_para.color_para.auto_exposure,
                 static_cast<int>(current_para.color_para.exposure),
                 static_cast<int>(current_para.color_para.gain));
        LOG_INFO("目标: %dx%d@%d, 曝光自动=%d, 曝光=%d, 增益=%d",
                 sence_para.color_para.width, sence_para.color_para.height, sence_para.color_para.fps,
                 sence_para.color_para.auto_exposure,
                 static_cast<int>(sence_para.color_para.exposure),
                 static_cast<int>(sence_para.color_para.gain));

        camera_state.is_switching_scene = true;

        // 1. 关闭相机
        LOG_INFO("CSI相机 %d 关闭当前相机", cam_id);
        if (!close_cam(cam_id, camera_state))
        {
            LOG_WARN("关闭失败，继续尝试重新打开");
        }

        // 2. 重新打开相机
        LOG_INFO("CSI相机 %d 重新打开场景 %d", cam_id, sense_id);
        bool ret = open_cam(cam_id, sense_id, actual_cam_info, camera_state);
        if (!ret)
        {
            LOG_ERROR("CSI相机 %d 重启失败", cam_id);
            camera_state.is_switching_scene = false;
            return false;
        }

        // 3. 更新场景ID
        camera_state.cur_sence_id = sense_id;

        LOG_INFO("========== CSI相机 %d 场景 %d 切换成功 ==========", cam_id, sense_id);
        camera_state.is_switching_scene = false;
        return true;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("切换CSI相机 %d 错误: %s", cam_id, e.what());
        camera_state.is_switching_scene = false;
        return false;
    }
}

void CamCsi::set_sensor_roi_fps_map(const std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map)
{
    sensor_roi_fps_map_ = sensor_roi_fps_map;
}

bool CamCsi::get_roi_fps_list_tegra(int cam_id, std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map)
{
    LOG_INFO("CSI相机 %d 使用Tegra/ARGUS配置", cam_id);
    
    std::vector<std::tuple<int, int, std::vector<int>>> supported_configs = {
        {640, 480, {15, 30, 60}},
        {1280, 720, {15, 30, 60, 90, 120}},
        {1920, 1080, {15, 30, 60}},
        {2104, 1560, {15, 30}},
        {2592, 1944, {15, 30}},
        {2616, 1472, {15, 30}},
        {3840, 2160, {15, 30}},
        {3896, 2192, {15, 30}},
        {4208, 3120, {15, 21}},
        {5632, 3168, {15, 21}},
        {5632, 4224, {15, 21}},
    };
    
    CamMgr::CamRoiFpsList roi_fps_list;
    
    for (const auto& config : supported_configs)
    {
        int width = std::get<0>(config);
        int height = std::get<1>(config);
        const auto& fps_list = std::get<2>(config);
        
        for (int fps : fps_list)
        {
            CamMgr::CamRoiFps roiFps;
            roiFps.width = width;
            roiFps.height = height;
            roiFps.fps = fps;
            roi_fps_list.push_back(roiFps);
        }
    }
    
    sensor_roi_fps_map["NV12"] = roi_fps_list;
    
    LOG_INFO("CSI相机 %d 配置完成，共 %zu 种", cam_id, roi_fps_list.size());
    return true;
}

bool CamCsi::get_roi_fps_list_static(int cam_id, std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map, const CamMgr::CamInfo& cam_info)
{
    try
    {
        sensor_roi_fps_map.clear();
        return get_roi_fps_list_tegra(cam_id, sensor_roi_fps_map);
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("获取ROI/FPS列表错误: %s", e.what());
        return false;
    }
}

bool CamCsi::get_all_cameras_static(CamMgr::CamDevInfoList& cam_dev_list)
{
    static std::mutex static_mutex;
    std::lock_guard<std::mutex> lock(static_mutex);
    
    try
    {
        DIR *dir = opendir("/dev");
        if (dir == nullptr) {
            LOG_ERROR("无法打开 /dev 目录");
            return false;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (strncmp(entry->d_name, "video", 5) == 0) {
                std::string device_path = "/dev/" + std::string(entry->d_name);
                
                int fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
                if (fd != -1) {
                    v4l2_capability caps;
                    std::string device_path_str = device_path;
                    std::replace(device_path_str.begin(), device_path_str.end(), '/', '_');
                    
                    if (ioctl(fd, VIDIOC_QUERYCAP, &caps) == 0) {
                        if (caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
                            CamMgr::CamDevInfo dev_info;
                            dev_info.cam_type = CamMgr::CamType::CAM_TYPE_CSI;
                            dev_info.device_id = device_path_str;
                            dev_info.product_id = std::string(reinterpret_cast<char*>(caps.card));
                            dev_info.device_name = "CSI " + std::string(reinterpret_cast<char*>(caps.card));
                            dev_info.serial_number = device_path_str;
                            dev_info.physical_port = std::string(reinterpret_cast<char*>(caps.bus_info));
                            dev_info.user_name = "";

                            std::string driver = std::string(reinterpret_cast<char*>(caps.driver));
                            if (driver.find("tegra") != std::string::npos) {
                                dev_info.facturer_name = "NVIDIA";
                            } else if (driver.find("uvcvideo") != std::string::npos) {
                                dev_info.facturer_name = "Generic USB Camera";
                            } else if (driver.find("imx") != std::string::npos) {
                                dev_info.facturer_name = "Sony";
                            } else if (driver.find("ov") != std::string::npos) {
                                dev_info.facturer_name = "OmniVision";
                            } else {
                                dev_info.facturer_name = driver;
                            }
                            dev_info.firmware_version = std::to_string(caps.version);
                                                        
                            // 筛选 CSI 相机：product_id 必须包含"vi-output"
                            std::string product_id_str = std::string(reinterpret_cast<char*>(caps.card));
                            if (product_id_str.find("vi-output") == std::string::npos)
                            {
                                LOG_DEBUG("跳过非 CSI 设备：%s (product_id: %s)", device_path_str.c_str(), product_id_str.c_str());
                                close(fd);
                                continue;
                            }
                                                        
                            cam_dev_list.push_back(dev_info);
                            LOG_INFO("发现 CSI 相机：%s (product_id: %s)", device_path_str.c_str(), product_id_str.c_str());
                        }
                    }
                    close(fd);
                }
            }
        }
        closedir(dir);

        LOG_INFO("枚举到 %zu 个CSI相机", cam_dev_list.size());
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("枚举CSI相机错误: %s", e.what());
        return false;
    }
}

std::string CamCsi::get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        return "/csi_camera_" + std::to_string(cam_id) + "/color/camera_info";
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        return "/csi_camera_" + std::to_string(cam_id) + "/depth/camera_info";
    return std::string();
}

std::string CamCsi::get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        return "/csi_camera_" + std::to_string(cam_id) + "/color/image_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        return "/csi_camera_" + std::to_string(cam_id) + "/depth/image_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_IR)
        return "/csi_camera_" + std::to_string(cam_id) + "/ir/image_raw";
    return std::string();
}

}  // namespace cam_mgr_ros
