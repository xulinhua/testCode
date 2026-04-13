#include "cam_mgr_ros/cam_rs.hpp"
#include "cam_mgr_ros/cam_base.hpp"
#include "cam_config_mgr/cam_com_struct.hpp"
#include <sys/wait.h>
// 添加RealSense SDK头文件
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#define SWITCH_CAM_SENCE_MODE 0       // 场景切换模式 0：停流设置参数再启用流 1:重开相机

namespace cam_mgr_ros
{

CamRs::CamRs() : CamBase()
{
    // 初始化RealSense相机相关资源
    set_camera_type(CamMgr::CamType::CAM_TYPE_RS);
}

CamRs::~CamRs()
{
    // 清理RealSense相机相关资源
}

bool CamRs::open_cam(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        //检查场景 ID 是否存在
        if (cam_info.sence_para.find(sense_id) == cam_info.sence_para.end())
        {
            LOG_ERROR("相机 %d 不存在场景 ID %d", cam_id, sense_id);
            return false;
        }

        // 保存相机配置信息，用于后续场景切换
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            cam_info_map_[cam_id] = cam_info;
            LOG_INFO("保存 RealSense相机 %d 配置信息，型号：%d", cam_id, static_cast<int>(cam_info.cam_model_type));
        }

        // 获取指定场景的参数
        const CamMgr::CamSencePara& sence_para = cam_info.sence_para.at(sense_id);

        //构建Realsense相机启动命令
        std::string cmd = "ros2 launch realsense2_camera rs_launch.py";
        if (!cam_info.serial_number.empty())
        {
            cmd += " serial_no:='\"" + cam_info.serial_number + "\"'";
        }

        // 添加关键参数
        cmd += " camera_name:=rs_camera_" + std::to_string(cam_id);
        cmd += " align_depth.enable:=true"; //确保深度和彩色对齐
        cmd += " enable_sync:=true";//时间戳同步
        cmd += " initial_reset:=true"; //启动时重置相机，清除之前的配置
        // cmd += " rgb_camera.frames_queue_size:=1"; //减少彩色帧队列大小以降低延时
        // cmd += " depth_module.frames_queue_size:=1"; //减少深度帧队列大小以降低延时
        
        // 根据启用的流设置参数（使用场景参数）
        cmd += " enable_color:=" + std::string(sence_para.enable_color_stream ? "true" : "false");
        cmd += " enable_depth:=" + std::string(sence_para.enable_depth_stream ? "true" : "false");
        cmd += " pointcloud.enable:=" + std::string(sence_para.enable_cloud_stream ? "true" : "false");
        cmd += " enable_infra1:=false"; //红外1
        cmd += " enable_infra2:=false"; //红外2

        // 设置曝光模式、曝光值和增益参数
        if (sence_para.enable_color_stream)
        {
             if (cam_info.cam_model_type != CamMgr::CamModelType::CAM_MODEL_RS_D405)
             {
                // 彩色流曝光设置
                cmd += " rgb_camera.enable_auto_exposure:=" + std::string(sence_para.color_para.auto_exposure ? "true" : "false");
                if (!sence_para.color_para.auto_exposure && sence_para.color_para.exposure > 0)
                {
                    // 手动曝光模式，设置曝光值
                    cmd += " rgb_camera.exposure:=" + std::to_string(static_cast<int>(sence_para.color_para.exposure));
                }
                if (sence_para.color_para.gain > 0)
                {
                    // 设置增益值
                    cmd += " rgb_camera.gain:=" + std::to_string(static_cast<int>(sence_para.color_para.gain));
                }
             }
             else
             {
                cmd += " depth_module.enable_auto_exposure:=" + std::string(sence_para.color_para.auto_exposure ? "true" : "false");
                if (!sence_para.color_para.auto_exposure && sence_para.color_para.exposure > 0)
                {
                    // 手动曝光模式，设置曝光值
                    cmd += " depth_module.exposure:=" + std::to_string(static_cast<int>(sence_para.color_para.exposure));
                }
                if (sence_para.color_para.gain > 0)
                {
                    // 设置增益值
                    cmd += " depth_module.gain:=" + std::to_string(static_cast<int>(sence_para.color_para.gain));
                }
             }
        }

        if (sence_para.enable_depth_stream)
        {
            // 深度流曝光设置
            cmd += " depth_module.enable_auto_exposure:=" + std::string(sence_para.depth_para.auto_exposure ? "true" : "false");
            if (!sence_para.depth_para.auto_exposure && sence_para.depth_para.exposure > 0)
            {
                // 手动曝光模式，设置曝光值
                cmd += " depth_module.exposure:=" + std::to_string(static_cast<int>(sence_para.depth_para.exposure));
            }
            if (sence_para.depth_para.gain > 0)
            {
                // 设置增益值
                cmd += " depth_module.gain:=" + std::to_string(static_cast<int>(sence_para.depth_para.gain));
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
                    LOG_INFO("相机 %d 彩色流使用匹配的 ROI/FPS 配置：%dx%d@%d", 
                             cam_id, color_width, color_height, color_fps);
                }
            }
            
            // 根据相机型号生成不同的参数名
            std::string profile_param = (cam_info.cam_model_type != CamMgr::CamModelType::CAM_MODEL_RS_D405) 
                                        ? "rgb_camera.color_profile" 
                                        : "depth_module.color_profile";
            cmd += " " + profile_param + ":=" + std::to_string(color_width) + "x" + 
                   std::to_string(color_height) + "x" + std::to_string(color_fps);
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
            cmd += " depth_module.depth_profile:=" + std::to_string(depth_width) + "x" + 
                   std::to_string(depth_height) + "x" + std::to_string(depth_fps);
        }
        else if (sence_para.enable_color_stream)
        {
            // 如果彩色流开启而深度流未开启，依然配置depth_module.profile，宽高与彩色流相同，fps设为10
            cmd += " depth_module.depth_profile:=" + std::to_string(sence_para.color_para.width) + "x" + 
                   std::to_string(sence_para.color_para.height) + "x5";
        }

        LOG_INFO("启动RealSense相机 %d场 %d进程: %s", cam_id, sense_id, cmd.c_str());

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
            LOG_INFO("RealSense 相机 %d 场景 %d 进程已启动，PID: %d", cam_id, sense_id, pid);
            
            // 设置相机节点名和话题名
             camera_state.cam_node_name = "/camera/rs_camera_" + std::to_string(cam_id);
             LOG_INFO("RealSense相机 %d名：%s", cam_id, camera_state.cam_node_name.c_str());

            return true;
        }
        else
        {
            // fork失败
            LOG_ERROR("启动RealSense相机 %d场景 %d进程失败", cam_id, sense_id);
            camera_state.is_offline = true;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("启动RealSense相机 %d场景 %d进时程时发生错误: %s", cam_id, sense_id, e.what());
        camera_state.is_offline = true;
        return false;
    }
}

bool CamRs::close_cam(int cam_id, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        // 如果cam_node_pid为0，尝试重新查找
        if (camera_state.cam_node_pid == 0 && camera_state.cam_pid > 0)
        {
            LOG_INFO("相机 %d 的节点进程ID为空，尝试重新查找", cam_id);
            std::vector<std::string> candidates = {"realsense2_camera_node"};
            std::string ros_node_name = "realsense2_camera_node";
            pid_t node_pid = find_descendant_exec_pid(camera_state.cam_pid, candidates, ros_node_name, 2000);
            if (node_pid > 0)
            {
                camera_state.cam_node_pid = node_pid;
                LOG_INFO("重新找到RealSense相机 %d 节点进程，PID: %d", cam_id, node_pid);
            }
        }

        if (camera_state.cam_node_pid > 0)
        {
            // 先终止节点进程
            kill(camera_state.cam_node_pid, SIGTERM);
            int status_node;
            waitpid(camera_state.cam_node_pid, &status_node, 0);
            LOG_INFO("RealSense相机 %d 节点进程已停止，PID: %d", cam_id, camera_state.cam_node_pid);
        }

        if (camera_state.cam_pid > 0)
        {
            //终进程
            kill(camera_state.cam_pid, SIGTERM);
            
            //等待进程结束
            int status;
            waitpid(camera_state.cam_pid, &status, 0);

            LOG_INFO("RealSense相机 %d进程已停止，PID: %d", cam_id, camera_state.cam_pid);
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
        LOG_ERROR("停止RealSense相机 %d 进程时发生错误: %s", cam_id, e.what());
        return false;
    }
}

bool CamRs::switch_cam_sence(int cam_id, int sense_id, const CamMgr::CamInfo& cam_info, CamMgr::CamRunInfo& camera_state)
{
    try
    {
        // 优先使用保存的相机配置信息，如果没有则使用传入的参数
        CamMgr::CamInfo actual_cam_info;
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            auto it = cam_info_map_.find(cam_id);
            if (it != cam_info_map_.end())
            {
                actual_cam_info = it->second;
                LOG_INFO("使用保存的 RealSense相机 %d 配置信息进行场景切换，型号：%d", 
                         cam_id, static_cast<int>(actual_cam_info.cam_model_type));
            }
            else
            {
                // 如果没有保存的配置，使用传入的参数
                actual_cam_info = cam_info;
                LOG_WARN("未找到保存的 RealSense相机 %d 配置信息，使用传入参数", cam_id);
            }
        }
        
        //检查场景 ID 是否存在
        if (actual_cam_info.sence_para.find(sense_id) == actual_cam_info.sence_para.end())
        {
            LOG_ERROR("相机 %d 不存在场景 ID %d", cam_id, sense_id);
            return false;
        }

        // 获取指定场景的参数
        const CamMgr::CamSencePara& sence_para = actual_cam_info.sence_para.at(sense_id);

        // Realsense相机参数设置
        LOG_INFO("设置 Realsense相机 %d 场景 %d 参数", cam_id, sense_id);

#if SWITCH_CAM_SENCE_MODE == 0
        // 模式 0：停流设置参数再启用流（现有逻辑）
        LOG_INFO("Realsense相机 %d 使用模式 0：停流设置参数再启用流", cam_id);

        // 先关闭已启用的流
        // 关闭彩色流
        if (camera_state.is_color_stream_start)
        {
            std::string disable_color_cmd = "ros2 param set " + camera_state.cam_node_name + " enable_color false";
            int disable_result = system(disable_color_cmd.c_str());
            if (disable_result != 0)
            {
                LOG_INFO("关闭 Realsense 相机 %d 彩流命令返回码：%d, 命令：%s", cam_id, disable_result, disable_color_cmd.c_str());
            }
            else
            {
                LOG_INFO("成功关闭 Realsense 相机 %d 彩色流", cam_id);
            }
            camera_state.is_color_stream_start = false; // 更新标志
        }
        
        // 关闭深度流
        if (camera_state.is_depth_stream_start)
        {
            std::string disable_depth_cmd = "ros2 param set " + camera_state.cam_node_name + " enable_depth false";
            int disable_result = system(disable_depth_cmd.c_str());
            if (disable_result != 0)
            {
                LOG_INFO("关闭 Realsense 相机 %d 深度流命令返回码：%d", cam_id, disable_result);
            }
            else
            {
                LOG_INFO("成功关闭 Realsense 相机 %d 深度流", cam_id);
            }
            camera_state.is_depth_stream_start = false; // 更新标志
        }
        
        // 关闭红外流
        if (camera_state.is_ir_stream_start)
        {
            std::string disable_infra1_cmd = "ros2 param set " + camera_state.cam_node_name + " enable_infra1 false";
            int disable_result = system(disable_infra1_cmd.c_str());
            if (disable_result != 0)
            {
                LOG_INFO("关闭 Realsense 相机 %d 红流 1 命令返回码：%d", cam_id, disable_result);
            }
            else
            {
                LOG_INFO("成功关闭 Realsense 相机 %d 红流 1", cam_id);
            }
            camera_state.is_ir_stream_start = false; // 更新标志
        }

        // 关闭点云流
        if (camera_state.is_cloud_stream_start)
        {
            std::string disable_cloud_cmd = "ros2 param set " + camera_state.cam_node_name + " pointcloud.enable false";
            int disable_result = system(disable_cloud_cmd.c_str());
            if (disable_result != 0)
            {
                LOG_INFO("关闭 Realsense 相机 %d 点云流命令返回码：%d", cam_id, disable_result);
            }
            else
            {
                LOG_INFO("成功关闭 Realsense 相机 %d 点云流", cam_id);
            }
            camera_state.is_cloud_stream_start = false; // 更新标志
        }

        //等待一段时间确保流已关闭
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // 设置ROI和FPS参数
        if (sence_para.enable_color_stream)
        {
            // 使用配置中的值
            std::string profile_cmd;
            if (actual_cam_info.cam_model_type != CamMgr::CamModelType::CAM_MODEL_RS_D405)
            {
                profile_cmd = "ros2 param set " + camera_state.cam_node_name +
                              " rgb_camera.color_profile \"" + std::to_string(sence_para.color_para.width) +
                              "x" + std::to_string(sence_para.color_para.height) + "x" + std::to_string(sence_para.color_para.fps) + "\"";
            }
            else
            { // 405
                profile_cmd = "ros2 param set " + camera_state.cam_node_name +
                              " depth_module.color_profile \"" + std::to_string(sence_para.color_para.width) +
                              "x" + std::to_string(sence_para.color_para.height) + "x" + std::to_string(sence_para.color_para.fps) + "\"";
            }
            int profile_result = system(profile_cmd.c_str());
            if (profile_result != 0)
            {
                LOG_WARN("设置Realsense相机 %d彩色流ROI和FPS参数失败", cam_id);
            }
            else
            {
                LOG_INFO("成功设置Realsense相机 %d彩色流ROI和FPS参数", cam_id);
            }

            
            
            // 设置彩色流曝光参数
            std::string auto_exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                            " rgb_camera.enable_auto_exposure " + std::string(sence_para.color_para.auto_exposure ? "true" : "false");
            if (actual_cam_info.cam_model_type == CamMgr::CamModelType::CAM_MODEL_RS_D405)
            {
                auto_exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                            " depth_module.enable_auto_exposure " + std::string(sence_para.color_para.auto_exposure ? "true" : "false");
            }
            int auto_exposure_result = system(auto_exposure_cmd.c_str());
            if (auto_exposure_result != 0)
            {
                LOG_WARN("设置Realsense相机 %d彩色流自动曝光模式失败", cam_id);
            }
            else
            {
                LOG_INFO("成功设置Realsense相机 %d彩色流自动曝光模式", cam_id);
            }

            if (!sence_para.color_para.auto_exposure && sence_para.color_para.exposure > 0)
            {
                std::string exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                           " rgb_camera.exposure " + std::to_string(static_cast<int>(sence_para.color_para.exposure));
                if (actual_cam_info.cam_model_type == CamMgr::CamModelType::CAM_MODEL_RS_D405)
                {
                    exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                   " depth_module.exposure " + std::to_string(static_cast<int>(sence_para.color_para.exposure));
                }
                int exposure_result = system(exposure_cmd.c_str());
                if (exposure_result != 0)
                {
                    LOG_WARN("设置Realsense相机 %d彩色流曝光值失败", cam_id);
                }
                else
                {
                    LOG_INFO("成功设置Realsense相机 %d彩色流曝光值", cam_id);
                }
            }

            // 设置彩色流增益参数
            // 注意：Realsense 大部分型号的彩色流不支持 gain，只有深度模块支持
            // D405 型号使用 depth_module.gain，其他型号理论上应使用 rgb_camera.gain（但可能不支持）
            if (sence_para.color_para.gain > 0)
            {
                std::string gain_cmd;
                if (actual_cam_info.cam_model_type == CamMgr::CamModelType::CAM_MODEL_RS_D405)
                {
                    // D405 型号：彩色流的增益实际配置在 depth_module 下
                    gain_cmd = "ros2 param set " + camera_state.cam_node_name + 
                               " depth_module.gain " + std::to_string(static_cast<int>(sence_para.color_para.gain));
                }
                else
                {
                    // 其他型号：尝试使用 rgb_camera.gain（如果相机不支持会输出警告）
                    gain_cmd = "ros2 param set " + camera_state.cam_node_name + 
                               " rgb_camera.gain " + std::to_string(static_cast<int>(sence_para.color_para.gain));
                }
                
                int gain_result = system(gain_cmd.c_str());
                if (gain_result != 0)
                {
                    LOG_WARN("设置 Realsense 相机 %d彩色流增益值失败（该型号可能不支持此参数）", cam_id);
                }
                else
                {
                    LOG_INFO("成功设置 Realsense 相机 %d彩色流增益值", cam_id);
                }
            }
        }

        if (sence_para.enable_depth_stream)
        {
            std::string profile_cmd = "ros2 param set " + camera_state.cam_node_name +
                                      " depth_module.depth_profile \"" + std::to_string(sence_para.depth_para.width) +
                                      "x" + std::to_string(sence_para.depth_para.height) + "x" + std::to_string(sence_para.depth_para.fps) + "\"";
            int profile_result = system(profile_cmd.c_str());
            if (profile_result != 0)
            {
                LOG_WARN("设置Realsense相机 %d流ROI和FPS参数失败", cam_id);
            }
            else
            {
                LOG_INFO("成功设置Realsense相机 %d深度流ROI和FPS参数", cam_id);
            }

            // 设置深度流曝光参数
            std::string auto_exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                            " depth_module.enable_auto_exposure " + std::string(sence_para.depth_para.auto_exposure ? "true" : "false");
            int auto_exposure_result = system(auto_exposure_cmd.c_str());
            if (auto_exposure_result != 0)
            {
                LOG_WARN("设置Realsense相机 %d深度流自动曝光模式失败", cam_id);
            }
            else
            {
                LOG_INFO("成功设置Realsense相机 %d深度流自动曝光模式", cam_id);
            }

            if (!sence_para.depth_para.auto_exposure && sence_para.depth_para.exposure > 0)
            {
                std::string exposure_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                           " depth_module.exposure " + std::to_string(static_cast<int>(sence_para.depth_para.exposure));
                int exposure_result = system(exposure_cmd.c_str());
                if (exposure_result != 0)
                {
                    LOG_WARN("设置Realsense相机 %d深度流曝光值失败", cam_id);
                }
                else
                {
                    LOG_INFO("成功设置Realsense相机 %d深度流曝光值", cam_id);
                }
            }

            if (sence_para.depth_para.gain > 0)
            {
                std::string gain_cmd = "ros2 param set " + camera_state.cam_node_name + 
                                       " depth_module.gain " + std::to_string(static_cast<int>(sence_para.depth_para.gain));
                int gain_result = system(gain_cmd.c_str());
                if (gain_result != 0)
                {
                    LOG_WARN("设置Realsense相机 %d深度流增益值失败", cam_id);
                }
                else
                {
                    LOG_INFO("成功设置Realsense相机 %d深度流增益值", cam_id);
                }
            }
        }
        else if (sence_para.enable_color_stream)
        {
            // 如果彩色流开启而深度流未开启，依然配置depth_module.profile，宽高与彩色流相同，fps设为0
            std::string profile_cmd = "ros2 param set " + camera_state.cam_node_name +
                                      " depth_module.depth_profile \"" + std::to_string(sence_para.color_para.width) +
                                      "x" + std::to_string(sence_para.color_para.height) + "x0\"";
            int profile_result = system(profile_cmd.c_str());
            if (profile_result != 0)
            {
                LOG_WARN("设置Realsense相机 %d流默认参数失败", cam_id);
            }
            else
            {
                LOG_INFO("成功设置Realsense相机 %d深度流默认参数", cam_id);
            }
        }

        //启动新场景的流
        if (sence_para.enable_color_stream)
        {
            std::string enable_color_cmd = "ros2 param set " + camera_state.cam_node_name + " enable_color true";
            int enable_result = system(enable_color_cmd.c_str());
                    
            if (enable_result == 0)
            {
                LOG_INFO("成功开启 Realsense 相机 %d 彩色流", cam_id);
                camera_state.is_color_stream_start = true;
            }
            else
            {
                LOG_WARN("开启 Realsense 相机 %d 彩色流失败", cam_id);
            }
        }

        if (sence_para.enable_depth_stream)
        {
            std::string enable_depth_cmd = "ros2 param set " + camera_state.cam_node_name + " enable_depth true";
            int enable_result = system(enable_depth_cmd.c_str());
                    
            if (enable_result == 0)
            {
                LOG_INFO("成功开启 Realsense 相机 %d 深度流", cam_id);
                camera_state.is_depth_stream_start = true;
            }
            else
            {
                LOG_WARN("开启 Realsense 相机 %d 深度流失败", cam_id);
            }
        }

        if (sence_para.enable_cloud_stream)
        {
            std::string enable_cloud_cmd = "ros2 param set " + camera_state.cam_node_name + " pointcloud.enable true";
            int enable_result = system(enable_cloud_cmd.c_str());
            if (enable_result != 0)
            {
                LOG_WARN("开启 Realsense相机 %d 点云流失败", cam_id);
            }
            else
            {
                LOG_INFO("成功开启 Realsense相机 %d 点云流", cam_id);
                camera_state.is_cloud_stream_start = true;
            }
        }

#else
        // 模式 1：先关闭相机，再使用对应的场景参数开启相机（参考 Orbbec 实现）
        LOG_INFO("Realsense 相机 %d 使用模式 1：关闭后重新开启", cam_id);
        
        // 设置切换标志，避免监控线程误判为掉线
        camera_state.is_switching_scene = true;
        
        // 先关闭相机
        close_cam(cam_id, camera_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 重新打开相机
        bool ret = open_cam(cam_id, sense_id, actual_cam_info, camera_state);
        if (ret)
        {
            LOG_INFO("Realsense 相机 %d 场景 %d 切换成功（模式 1）", cam_id, sense_id);
            camera_state.is_switching_scene = false;  // 清除切换标志
            return true;
        }
        else
        {
            LOG_ERROR("Realsense 相机 %d 场景 %d 切换失败（模式 1）", cam_id, sense_id);
            camera_state.is_switching_scene = false;  // 清除切换标志
            return false;
        }
#endif

        // 更新当前场景ID
        camera_state.cur_sence_id = sense_id;

        LOG_INFO("RealSense相机 %d场 %d 参数切换成功", cam_id, sense_id);
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("切换Realsense相机 %d场 %d 参数时发生错误: %s", cam_id, sense_id, e.what());
        return false;
    }
}

// RealSense 子类实现内参/图像话题名
std::string CamRs::get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/color/camera_info";
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/depth/camera_info";
    return std::string();
}

std::string CamRs::get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
    {
        if (cam_info.cam_model.find("D405") != std::string::npos)
            return "/camera/rs_camera_" + std::to_string(cam_id) + "/color/image_rect_raw";
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/color/image_raw";
    }
    if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
    {
        if (cam_info.cam_model.find("D405") != std::string::npos)
            return "/camera/rs_camera_" + std::to_string(cam_id) + "/depth/image_rect_raw";
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/aligned_depth_to_color/image_raw";
    }
    if (stream_type == CamMgr::CamStreamType::STREAM_IR)
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/infra1/image_rect_raw";
    if (stream_type == CamMgr::CamStreamType::STREAM_CLOUD)
        return "/camera/rs_camera_" + std::to_string(cam_id) + "/depth/color/points";
    return std::string();
}

void CamRs::set_sensor_roi_fps_map(const std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map)
{
    sensor_roi_fps_map_ = sensor_roi_fps_map;
}

bool CamRs::get_roi_fps_list_static(int cam_id, std::map<std::string, CamMgr::CamRoiFpsList>& sensor_roi_fps_map, const CamMgr::CamInfo& cam_info)
{
    try
    {
        //处理Realsense相机
        rs2::context ctx;
        auto device_list = ctx.query_devices();

        //根据序列号查找匹配的设备
        rs2::device target_device;
        bool found = false;
        for (size_t i = 0; i < device_list.size(); i++)
        {
            auto dev = device_list[i];
            std::string dev_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (dev_serial == cam_info.serial_number)
            {
                target_device = dev;
                found = true;
                break;
            }
        }

        if (found)
        {
            //清空之前的ROI和FPS列表
            sensor_roi_fps_map.clear();

            // 获取设备的传感器列表
            std::vector<rs2::sensor> sensors = target_device.query_sensors();

            if (sensors.empty())
            {
                LOG_WARN("RealSense相机 %d没有找到传感器", cam_id);
                return false;
            }

            //所有传感器
            for (const auto& sensor : sensors)
            {
                std::string sensor_name = "unknown";

                try
                {
                    sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
                }
                catch (const rs2::error& e)
                {
                    LOG_WARN("无法获取传感器名称: %s", e.what());
                }

                // 获取该传感器支持的流配置列表
                std::vector<rs2::stream_profile> profiles;
                try
                {
                    profiles = sensor.get_stream_profiles();
                }
                catch (const rs2::error& e)
                {
                    LOG_WARN("无法获取传感器流配置: %s", e.what());
                    continue;
                }

                // 用于存储不同类型流的ROI和FPS列表
                std::map<std::string, CamMgr::CamRoiFpsList> stream_roi_fps_map;

                //所有流配置
                for (const auto& profile : profiles)
                {
                    // 检查是否为视频流配置（ROI和FPS只适用于视频流）
                    if (profile.is<rs2::video_stream_profile>())
                    {
                        rs2::video_stream_profile video_profile = profile.as<rs2::video_stream_profile>();
                        rs2_stream stream_type = video_profile.stream_type();
                        int width = video_profile.width();
                        int height = video_profile.height();
                        int fps = video_profile.fps();

                        //根据流类型分类存储
                        std::string stream_type_str;
                        switch (stream_type)
                        {
                        case RS2_STREAM_COLOR:
                            stream_type_str = "color";
                            break;
                        case RS2_STREAM_DEPTH:
                            stream_type_str = "depth";
                            break;
                        case RS2_STREAM_INFRARED:
                            stream_type_str = "infrared";
                            break;
                        default:
                            stream_type_str = "other";
                            break;
                        }

                        // 检查是否已存在该分辨率和FPS的组合
                        bool exists = false;
                        for (const auto& roi_fps : stream_roi_fps_map[stream_type_str])
                        {
                            if (roi_fps.width == width && roi_fps.height == height && roi_fps.fps == fps)
                            {
                                exists = true;
                                break;
                            }
                        }

                        // 如果不存在，则添加到列表中
                        if (!exists)
                        {
                            CamMgr::CamRoiFps roi_fps;
                            roi_fps.width = width;
                            roi_fps.height = height;
                            roi_fps.fps = fps;
                            stream_roi_fps_map[stream_type_str].push_back(roi_fps);
                        }
                    }
                }

                //将传感器的流配置信息添加到总映射中
                for (const auto& pair : stream_roi_fps_map)
                {
                    sensor_roi_fps_map[sensor_name + "_" + pair.first] = pair.second;
                    
                    // 添加简化键映射，便于后续查找
                    if (pair.first == "color" || pair.first == "depth")
                    {
                        // 检查是否已存在该简化键，如果存在则合并列表
                        if (sensor_roi_fps_map.find(pair.first) != sensor_roi_fps_map.end())
                        {
                            // 合并列表，去重
                            for (const auto& roi_fps : pair.second)
                            {
                                bool exists = false;
                                for (const auto& existing : sensor_roi_fps_map[pair.first])
                                {
                                    if (existing.width == roi_fps.width && existing.height == roi_fps.height && existing.fps == roi_fps.fps)
                                    {
                                        exists = true;
                                        break;
                                    }
                                }
                                if (!exists)
                                {
                                    sensor_roi_fps_map[pair.first].push_back(roi_fps);
                                }
                            }
                            LOG_INFO("合并传感器 %s 的 %s 流配置，当前列表大小: %zu", sensor_name.c_str(), pair.first.c_str(), sensor_roi_fps_map[pair.first].size());
                        }
                        else
                        {
                            sensor_roi_fps_map[pair.first] = pair.second;
                            LOG_INFO("添加传感器 %s 的 %s 流配置，列表大小: %zu", sensor_name.c_str(), pair.first.c_str(), pair.second.size());
                        }
                    }
                }
            }

            LOG_INFO("成功获取RealSense相机 %d 的ROI和FPS参数", cam_id);
            return true;
        }
        else
        {
            LOG_WARN("未找到序列号为 %s 的RealSense相机", cam_info.serial_number.c_str());
            return false;
        }
    }
    catch (const rs2::error& e)
    {
        LOG_ERROR("获取RealSense相机 %d 的ROI和FPS列表时发生错误: %s", cam_id, e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("获取RealSense相机 %d 的ROI和FPS列表时发生错误: %s", cam_id, e.what());
        return false;
    }
}
bool CamRs::get_all_cameras_static(CamMgr::CamDevInfoList& cam_dev_list)
{
    static std::mutex static_mutex;
    std::lock_guard<std::mutex> lock(static_mutex);
    try
    {
        // 使用RealSense SDK枚举连接的相机设备
        rs2::context ctx;
        auto device_list = ctx.query_devices();

        //所有连接的设备
        for (size_t i = 0; i < device_list.size(); i++)
        {
            auto dev = device_list[i];

            // 获取设备信息
            CamMgr::CamDevInfo dev_info;
            dev_info.cam_type = CamMgr::CamType::CAM_TYPE_RS; // RealSense相机
            dev_info.device_id = std::to_string(i);           // 使用索引作为设备ID
            dev_info.product_id = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
            dev_info.device_name = dev.get_info(RS2_CAMERA_INFO_NAME);
            dev_info.serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            dev_info.physical_port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
            dev_info.user_name = dev.get_info(RS2_CAMERA_INFO_NAME);     // 使用设备名称作为用户名称
            dev_info.facturer_name = dev.get_info(RS2_CAMERA_INFO_NAME); // 使用设备名称作为制造商名称
            dev_info.firmware_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

            cam_dev_list.push_back(dev_info);
        }

        LOG_INFO("枚举到 %zu 个RealSense相机设备", cam_dev_list.size());
        return true;
    }
    catch (const rs2::error& e)
    {
        LOG_ERROR("枚举RealSense相机设备时发生错误: %s", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("枚举RealSense相机设备时发生错误: %s", e.what());
        return false;
    }
}

}