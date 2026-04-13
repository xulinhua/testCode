#include "cam_mgr_ros/cam_mgr.hpp"
#include "cam_mgr_ros/utils.hpp"  // 添加utils头文件
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bas_operate/file_operate.hpp"  // 添加bas_operate头文件
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <unordered_set>
#include <thread>
#include <atomic>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <nlohmann/json.hpp> // 添加json库头文件
#include <ctime>             // 添加时间库头文件
#include <iomanip>           // 添加iomanip头文件用于时间格式化
#include <cmath>             // 添加数学库用于计算距离
#include <climits>           // 添加 climits 头文件用于 INT_MAX
#include <cstring>           // 添加 cstring 头文件用于 memcpy
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// 添加bas_sys_config_ros头文件
#include "sys_info_src/sys_info_server.h"
#include "bas_sys_config_ros/calib_info_server.h"
// 添加bas_operate_ros头文件
#include "bas_operate_ros/param_utils.hpp"
// 添加cam_config_mgr头文件
#include <cam_config_mgr/camera_config_manager.hpp>
#include <cam_config_mgr/cam_com_struct.hpp>
// 添加RealSense SDK头文件
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
// 添加Orbbec SDK头文件
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Sensor.hpp>
#include <libobsensor/hpp/StreamProfile.hpp>
// 添加V4L2头文件用于CSI相机支持
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <dirent.h>

// 添加custom_msgs_comm服务头文件
#include "custom_msgs_comm/srv/get_cam_intr.hpp"

namespace cam_mgr_ros
{
    // 定义操作类型常量
    constexpr int CAMERA_OPERATION_START = 1;        // 开启相机
    constexpr int CAMERA_OPERATION_STOP = 2;         // 关闭相机
    constexpr int CAMERA_OPERATION_SWITCH_SCENE = 3; // 切换场景参数

    constexpr int CAMERA_LOAD_FROM_FILE = 0;         // 从文件加载相机配置
    constexpr int CAMERA_LOAD_FROM_PARAM_SERVER = 1; // 从参数服务器加载相机配置

    constexpr int MAX_SAVE_TIME_MS = 120000;       // 最大存图时间：2 分钟
    constexpr int STARTUP_TIMEOUT_MS = 60000;      // 启动超时时间：1 分钟
    

    CamMgrRos::CamMgrRos()
        : Node("camera_manager"),
          config_mgr_(),                           // 初始化CamConfigMgr实例，用于管理相机配置
          cam_dev_list_(),                         // 初始化相机设备信息列表
          cam_run_info_(),                        // 初始化相机状态映射表
          cam_roi_fps_map_(),                      // 初始化相机支持的roi fps映射表
          is_save_device_info_(false),             // 初始化是否保存设备信息标志为false
          intrinsics_from_stream_received_(false), // 初始化内参接收标志位
          enable_camera_info_forward_(true),        // 默认启用内参转发功能
          save_start_time_ms_(0),                  // 初始化存图开始时间为 0
          bag_record_pid_(0),                       // 初始化 bag 录制 PID
          bag_record_pgid_(0),                      // 初始化 bag 录制进程组 ID
          delete_bag_after_parse_(true),           // 默认删除 ROS 包文件
          image_save_format_("jpg")                 // 默认保存为 JPG 格式
    {

        LOG_INFO("=========================");
        LOG_INFO("相机管理器节点启动");
        LOG_INFO("配置文件目录: %s", config_mgr_.cam_cfg_dir_.c_str());
        LOG_INFO("内参转发功能状态: %s", enable_camera_info_forward_ ? "启用" : "禁用");

        sys_config_node_name_ = "sys_config_ros_node";
        sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, sys_config_node_name_);
        // 加载配置
        if (load_config(CAMERA_LOAD_FROM_PARAM_SERVER)) // 先从参数服务器加载，再从文件加载具体的相机配置
        {
            // 更新相机数量为配置管理器中的实际相机数量（直接从配置管理器读取）
            LOG_INFO("配置加载成功，相机数量: %d", config_mgr_.get_cam_num());

            cam_mgr_ros::CamBase::kill_camera_processes(); // 杀死可能占用相机的进程
            get_all_cameras();// 获取所有相机设备信息
            get_camera_roi_fps(); // 获取相机支持的roi fps

            // 在获取相机设备信息后，根据更新后的相机类型创建相机实例
            if (!create_all_camera_instances())
            {
                LOG_WARN("部分相机实例创建失败");
            }

            // 启动相机进程
            if (open_cam_all())
            {
                LOG_INFO("相机进程启动成功");
                get_cam_image_topics();// 获取并保存图像话题名
                get_cam_info_topics();// 获取并保存内参话题名
                std::this_thread::sleep_for(std::chrono::milliseconds(100));// 等待相机初始化完成
                LOG_INFO("相机节点初始化完成");
                LOG_INFO("=========================");
            }
            else
            {
                LOG_ERROR("相机进程启动失败！");
            }

            // 创建相机控制服务
            camera_control_service_ = this->create_service<custom_msgs_comm::srv::CameraControl>(
                "/camera_control",
                std::bind(&CamMgrRos::camera_control_service_callback, this, std::placeholders::_1, std::placeholders::_2));
            LOG_INFO("相机控制服务已创建: camera_control");

            // 创建相机内参服务
            basros::RosCommInfo comm_info;
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS_SRV, 0, 0);
            std::string service_name = comm_info.name;
            get_cam_intr_service_ = this->create_service<custom_msgs_comm::srv::GetCamIntr>(
                service_name, std::bind(&CamMgrRos::get_cam_intr_service_callback, this, std::placeholders::_1, std::placeholders::_2));
            LOG_INFO("相机内参获取服务已创建: %s", service_name.c_str());
                
            // 创建设置图像保存服务
            set_image_save_service_ = this->create_service<custom_msgs_comm::srv::SetImageSave>(
                "set_image_save",
                std::bind(&CamMgrRos::set_image_save_service_callback, this, std::placeholders::_1, std::placeholders::_2));
            LOG_INFO("设置图像保存服务已创建: set_image_save");

            // 创建定时器定期检查并订阅图像话题
            subscription_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), // 每秒检查一次
                std::bind(&CamMgrRos::check_and_subscribe_images, this));
            LOG_INFO("图像订阅定时器已创建");
            
            // 创建定时器检查存图超时
            save_timeout_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), // 每秒检查一次
                std::bind(&CamMgrRos::save_timeout_check_callback, this));
            LOG_INFO("存图超时检查定时器已创建");
            
            // 创建 ModuleInfo 刷新定时器，步长 200ms
            module_info_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), // 200ms 刷新一次
                std::bind(&CamMgrRos::refreshModuleInfo, this));
            LOG_INFO("ModuleInfo 刷新定时器已创建");
            
            start_background_threads();// 启动后台监视与重试线程
            init_camera_image_save_threads();// 初始化图像保存线程（每个相机独立）
            init_display_threads();// 初始化图像显示线程池
            init_compress_threads(); // 初始化文件夹压缩线程池
        }
        else
        {
            LOG_ERROR("配置文件加载失败！");
        }
    }

    CamMgrRos::~CamMgrRos()
    {
        try
        {
            LOG_INFO("正在关闭 cam_mgr_ros 节点...");
                
            // 1. 先停止图像保存线程
            stop_image_save_threads_ = true;
            for (auto& pair : image_save_queues_)
            {
                int cam_id = pair.first;
                image_save_queue_cv_[cam_id].notify_all();
            }
                
            // 等待所有保存线程退出
            for (auto& pair : image_save_threads_)
            {
                if (pair.second.joinable())
                {
                    pair.second.join();
                }
            }
            LOG_INFO("图像保存线程已停止");
                
            // 2. 停止显示线程
            stop_display_threads_ = true;
            display_task_cv_.notify_all();
            for (auto &thread : display_threads_)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }
            LOG_INFO("显示线程已停止");
            
            // 2.5 停止压缩线程
            stop_compress_threads_ = true;
            compress_task_cv_.notify_all();
            for (auto &thread : compress_threads_)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }
            LOG_INFO("压缩线程已停止");
            
            // 3. 停止后台监视线程
            stop_background_threads();
            
            // 4. 取消所有话题订阅
            for (const auto& sub_pair : image_subscriber_map_)
            {
                LOG_INFO("取消相机 %d 的图像订阅", sub_pair.first);
            }
            image_subscriber_map_.clear();
                
            for (const auto& sub_pair : pointcloud_subscriber_map_)
            {
                LOG_INFO("取消相机 %d 的点云订阅", sub_pair.first);
            }
            pointcloud_subscriber_map_.clear();
            LOG_INFO("所有话题订阅已取消");
                
            // 5. 强制关闭所有相机进程（使用 close_cam 函数）
            std::vector<int> cam_ids_to_close;
            for (const auto &pair : camera_map_)
            {
                cam_ids_to_close.push_back(pair.first);
            }
                
            for (int cam_id : cam_ids_to_close)
            {
                LOG_INFO("正在强制关闭相机 %d...", cam_id);
                try
                {
                    // 使用 close_cam 函数，它会处理所有的清理工作
                    close_cam(cam_id, false);
                }
                catch (const std::exception& e)
                {
                    LOG_ERROR("关闭相机 %d 时发生异常：%s", cam_id, e.what());
                }
            }
                
            // 6. 额外等待，确保所有相机进程完全退出
            std::this_thread::sleep_for(std::chrono::seconds(2));
                
            // 7. 清理可视化资源
            for (auto& pair : visualization_manager_map_)
            {
                if (pair.second)
                {
                    try
                    {
                        pair.second->closeWindows(false);
                    }
                    catch (...)
                    {
                        // 忽略任何异常
                    }
                }
            }
                
            LOG_INFO("cam_mgr_ros 节点已完全关闭");
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("析构函数中发生异常：%s", e.what());
        }
    }

    bool CamMgrRos::get_all_cameras()
    {
        try
        {
            CamMgr::CamDevInfoList cam_dev_list;

            CamMgr::CamDevInfoList rs_cam_list, ob_cam_list, csi_cam_list;

            // 通过相机子类的静态接口枚举相机设备
            CamRs::get_all_cameras_static(rs_cam_list);
            CamOb::get_all_cameras_static(ob_cam_list);
            CamCsi::get_all_cameras_static(csi_cam_list);

            // 将结果添加到总列表中（批量插入并预分配容量）
            size_t total = rs_cam_list.size() + ob_cam_list.size() + csi_cam_list.size();
            cam_dev_list.reserve(total);
            cam_dev_list.insert(cam_dev_list.end(), rs_cam_list.begin(), rs_cam_list.end());
            cam_dev_list.insert(cam_dev_list.end(), ob_cam_list.begin(), ob_cam_list.end());
            cam_dev_list.insert(cam_dev_list.end(), csi_cam_list.begin(), csi_cam_list.end());

            // 打印相机设备信息
            print_camera_device_info(cam_dev_list);

            cam_dev_list_ = cam_dev_list; // 调用接口设置相机信息到配置参数
            set_camera_info_to_config();
            // 保存所有遍历到的相机信息
            save_all_cameras_info();
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("遍历相机设备时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::find_min_supported_fps(int cam_id, CamMgr::CamStreamType stream_type, int target_width, int target_height, int& min_fps)
    {
        try
        {
            min_fps = 0;
            // 查找相机ID对应的ROI和FPS列表
            auto cam_it = cam_roi_fps_map_.find(cam_id);
            if (cam_it == cam_roi_fps_map_.end())
            {
                LOG_WARN("未找到相机 %d 的ROI和FPS列表", cam_id);
                return false;
            }
            // 根据流类型确定要查找的传感器类型
            std::string sensor_type = get_sensor_type_by_stream_type(stream_type);
            if (sensor_type.empty())
            {
                return false;
            }

            // 检查是否存在对应传感器类型的配置
            auto sensor_it = cam_it->second.find(sensor_type);
            if (sensor_it == cam_it->second.end())
            {
                LOG_WARN("相机 %d 未找到 %s 传感器的ROI和FPS列表", cam_id, sensor_type.c_str());
                return false;
            }

            const CamMgr::CamRoiFpsList &roi_fps_list = sensor_it->second;
            if (roi_fps_list.empty())
            {
                LOG_WARN("相机 %d 的 %s 传感器ROI和FPS列表为空", cam_id, sensor_type.c_str());
                return false;
            }

            // 查找与目标分辨率完全匹配的配置，并找到最低的FPS
            int minimum_fps = INT_MAX;
            bool found_match = false;

            for (const auto &roi_fps : roi_fps_list)
            {
                // 检查分辨率是否完全匹配
                if (roi_fps.width == target_width && roi_fps.height == target_height)
                {
                    if (roi_fps.fps < minimum_fps)
                    {
                        minimum_fps = roi_fps.fps;
                        found_match = true;
                    }
                }
            }

            // 检查是否找到了匹配项
            if (!found_match)
            {
                LOG_WARN("未找到相机 %d 分辨率 %dx%d 的配置", cam_id, target_width, target_height);
                return false;
            }

            min_fps = minimum_fps;
            LOG_INFO("找到最低支持的FPS: %d (分辨率: %dx%d)", min_fps, target_width, target_height);
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("查找最低FPS时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::load_config(int mode)
    {
        try
        {
            // 如果 mode==1，尝试先从参数服务器加载；失败时回退到文件模式 0
            int file_mode = mode;
            if (mode == 1)
            {
                if (load_cam_config_srv())
                {
                    LOG_INFO("从参数服务器加载配置成功");
                }
                else
                {
                    LOG_WARN("从参数服务器加载配置失败，回退到文件配置");
                    file_mode = 0;
                }
            }
            if (!config_mgr_.load_config(file_mode))
            {
                LOG_ERROR("加载配置文件失败");
                return false;
            }
            LOG_INFO("配置文件加载成功");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("加载配置文件时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::load_cam_config_srv()
    {
        try
        {
            LOG_INFO("开始从参数服务器加载相机配置信息...");

            SysConfig::CamConfigInfo1D sys_cam_list;
            if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
            {
                LOG_ERROR("无法连接到系统配置参数服务");
                return false;
            }
            else
            {
                if (!RosComm::getCamInfoListFromServer(sys_config_client_, SYS_ENABLE_CAM_LIST, sys_cam_list))
                {
                    LOG_ERROR("从参数服务器读取相机配置信息列表失败");
                    return false;
                }
            }

            LOG_INFO("成功从参数服务器获取到 %zu 个相机配置", sys_cam_list.size());

            // 打印所有获取到的相机配置信息
            LOG_INFO("=== 从参数服务器获取的相机配置信息 ===");

            // 遍历获取到的相机配置信息
            config_mgr_.cam_num_ = static_cast<int>(sys_cam_list.size());
            for (size_t i = 0; i < sys_cam_list.size(); ++i)
            {
                const auto &sys_cam_info = sys_cam_list[i];

                print_camera_config_info(i, sys_cam_info);

                // 创建新的相机配置信息
                CamMgr::CamInfo cam_info;
                cam_info.cam_id = static_cast<int>(sys_cam_info.cam_id);
                cam_info.enable = sys_cam_info.is_enable;
                cam_info.cam_usr_name = sys_cam_info.user_name;
                cam_info.serial_number = sys_cam_info.serial_number;

                // 设置其他默认值
                cam_info.cam_type = CamMgr::CamType::CAM_TYPE_NONE;
                cam_info.cam_model = "";
                cam_info.cam_index = 0;
                cam_info.show_topic_image = true;
                cam_info.sence_num = 1; // 默认场景数量为1

                // 更新配置管理器中的相机配置
                config_mgr_.set_camera_config(static_cast<int>(sys_cam_info.cam_id), cam_info);

                LOG_INFO("成功设置相机cam_id= %d 配置，序列号: %s，用户名称: %s",
                         static_cast<int>(sys_cam_info.cam_id), sys_cam_info.serial_number.c_str(), sys_cam_info.user_name.c_str());
            }
            LOG_INFO("=== 相机配置信息加载完成 ===");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("从参数服务器加载相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::create_camera_instance(int cam_id, CamMgr::CamType cam_type)
    {
        try
        {
            // 根据相机类型创建对应的相机实例
            switch (cam_type)
            {
            case CamMgr::CamType::CAM_TYPE_RS:
                camera_map_[cam_id] = std::make_unique<cam_mgr_ros::CamRs>();
                LOG_INFO("创建RealSense相机实例，ID: %d", cam_id);
                break;
            case CamMgr::CamType::CAM_TYPE_OB:
                camera_map_[cam_id] = std::make_unique<cam_mgr_ros::CamOb>();
                LOG_INFO("创建Orbbec相机实例，ID: %d", cam_id);
                break;
            case CamMgr::CamType::CAM_TYPE_CSI:
                camera_map_[cam_id] = std::make_unique<cam_mgr_ros::CamCsi>();
                LOG_INFO("创建CSI相机实例，ID: %d", cam_id);
                break;
            default:
                LOG_ERROR("不支持的相机类型: %d", static_cast<int>(cam_type));
                return false;
            }

            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("创建相机实例时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::create_all_camera_instances()
    {
        try
        {
            bool all_created = true;
            
            for (auto &config_pair : config_mgr_.camera_configs_)
            {
                if (config_pair.second.enable)
                {
                    cam_run_info_[config_pair.first] = CamMgr::CamRunInfo();
                    if (!create_camera_instance(config_pair.first, config_pair.second.cam_type))
                    {
                        LOG_ERROR("创建相机实例失败，ID: %d", config_pair.first);
                        all_created = false;
                    }
                }
            }
            
            return all_created;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("创建所有相机实例时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::open_cam_all()
    {
        try
        {
            // 启动所有配置中的相机
            LOG_INFO("开始启动所有配置中的相机");

            // 遍历所有启用的相机并启动对应的进程
            bool all_started = true;
#if 1
            // 遍历所有启用的相机状态
            int open_num = 0;
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                CamMgr::CamInfo cam_info;
                if (config_mgr_.get_camera_config(cam_id, cam_info))
                {
                    LOG_INFO("发现相机配置，ID: %d，开始启动", cam_id);
                    // 通过CamBase接口启动相机
                    if (!open_cam(cam_id, 0, std::nullopt))
                    {
                        LOG_ERROR("启动相机 %d失败", cam_id);
                        all_started = false;
                    }
                    else
                    {
                        // 保存单个相机信息
                        get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                        save_single_camera_info(cam_id);
                        open_num++;
                    }
                }
            }
#else
            // 分阶段启动：先启动非 Orbbec 相机，最后启动 Orbbec 相机
            int open_num = 0;
            
            // 第一阶段：启动所有非 Orbbec 相机（RS、CSI 等）
            LOG_INFO("========== 第一阶段：启动非 Orbbec 相机 ==========");
            for (const auto &config_pair : config_mgr_.camera_configs_)
            {
                if (config_pair.second.enable && 
                    config_pair.second.cam_type != CamMgr::CamType::CAM_TYPE_OB)
                {
                    int cam_id = config_pair.first;
                    CamMgr::CamInfo cam_info;
                    if (config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_INFO("启动非 Orbbec 相机，ID: %d, 类型：%d", 
                                 cam_id, static_cast<int>(config_pair.second.cam_type));
                        if (!open_cam(cam_id, 0, std::nullopt))
                        {
                            LOG_ERROR("启动非 Orbbec 相机 %d 失败", cam_id);
                            all_started = false;
                        }
                        else
                        {
                            get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                            save_single_camera_info(cam_id);
                            open_num++;
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));
            //第二阶段：启动所有 Orbbec 相机
            LOG_INFO("========== 第二阶段：启动 Orbbec 相机 ==========");
            for (const auto &config_pair : config_mgr_.camera_configs_)
            {
                if (config_pair.second.enable && 
                    config_pair.second.cam_type == CamMgr::CamType::CAM_TYPE_OB)
                {
                    int cam_id = config_pair.first;
                    CamMgr::CamInfo cam_info;
                    if (config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_INFO("启动 Orbbec 相机，ID: %d", cam_id);
                        if (!open_cam(cam_id, 0, std::nullopt))
                        {
                            LOG_ERROR("启动 Orbbec 相机 %d 失败", cam_id);
                            all_started = false;
                        }
                        else
                        {
                            get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                            save_single_camera_info(cam_id);
                            open_num++;
                        }
                    }
                }
            }
#endif
            if (all_started)
            {
                LOG_INFO("所有相机启动完成，共启动 %d 个相机", open_num);
            }
            else
            {
                LOG_WARN("部分相机启动失败");
            }

            return all_started;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("启动相机进程时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::open_cam(int cam_id, int sence_id, std::optional<CamMgr::CamInfo> cam_info_opt)
    {
        try
        {
            // 获取相机配置
            CamMgr::CamInfo cam_info;
            if (cam_info_opt.has_value())
            {
                cam_info = cam_info_opt.value();
                LOG_INFO("使用临时相机配置启动相机 %d", cam_id);
            }
            else
            {
                if (!config_mgr_.get_camera_config(cam_id, cam_info))
                {
                    LOG_ERROR("获取相机 %d配置失败", cam_id);
                    return false;
                }
            }

            // 检查场景ID是否存在
            if (cam_info.sence_para.find(sence_id) == cam_info.sence_para.end())
            {
                LOG_ERROR("相机 %d不存在场景ID %d", cam_id, sence_id);
                return false;
            }

            // 检查相机实例是否存在
            auto it = camera_map_.find(cam_id);
            if (it == camera_map_.end())
            {
                LOG_ERROR("未找到相机ID %d的实例", cam_id);
                return false;
            }

            // 检查并设置ROI/FPS数据到相机子类
            auto roi_fps_it = cam_roi_fps_map_.find(cam_id);
            if (roi_fps_it != cam_roi_fps_map_.end())
            {
                it->second->set_sensor_roi_fps_map(roi_fps_it->second);
                LOG_INFO("已将ROI/FPS数据设置到相机 %d", cam_id);
            }
            else
            {
                LOG_WARN("未找到相机 %d 的ROI/FPS数据", cam_id);
            }

            // 通过 CamBase 接口启动相机
            CamMgr::CamRunInfo &camera_state = cam_run_info_[cam_id];
            // 启动前设置为"启动中"状态
            camera_state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_STARTING;
            bool result = it->second->open_cam(cam_id, sence_id, cam_info, camera_state);
                        
            if (result)
            {
                // 打开成功，但先不标记为正常，等待图像/点云数据确认
                camera_state.is_offline = false;
                // 记录启动成功时间，用于后续超时判断
                camera_state.last_success_open_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                LOG_INFO("通过 CamBase 接口成功启动相机 %d，等待图像/点云数据确认", cam_id);
            }
            else
            {
                // 打开失败，标记为掉线和异常状态以触发重试逻辑
                camera_state.is_offline = true;
                camera_state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_ERROR;
                LOG_ERROR("通过 CamBase 接口启动相机 %d失败", cam_id);
            }

            return result;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("启动相机 %d场景 %d进程时发生错误: %s", cam_id, sence_id, e.what());
            return false;
        }
    }

    void CamMgrRos::close_cam_all()
    {
        try
        {
            // 停止所有相机进程
            for (const auto &pair : cam_run_info_)
            {
                close_cam(pair.first);
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("停止相机进程时发生错误: %s", e.what());
        }
    }

    bool CamMgrRos::close_cam(int cam_id, bool is_offline_close)
    {
        try
        {
            // 检查相机实例是否存在
            auto it = camera_map_.find(cam_id);
            if (it == camera_map_.end())
            {
                LOG_ERROR("未找到相机ID %d的实例", cam_id);
                return false;
            }

            // 通过CamBase接口关闭相机
            CamMgr::CamRunInfo &camera_state = cam_run_info_[cam_id];
            bool result = it->second->close_cam(cam_id, camera_state);

            if (result)
            {
                LOG_INFO("通过CamBase接口成功关闭相机 %d", cam_id);
                
                // 使用锁保护状态修改
                {
                    std::lock_guard<std::mutex> lock(camera_state_mutex_);
                    // 重置内参转发标志
                    has_forwarded_map_[cam_id] = false;
                    // 只有非掉线关闭时才重置掉线标志
                    if (!is_offline_close)
                    {
                        cam_run_info_[cam_id].is_offline = false;
                    }
                                    
                    // 关闭相机时设置为“已关闭”状态
                    cam_run_info_[cam_id].cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_CLOSED;
                                    
                    // 重置时间戳为 0，避免重连后立即被判断为超时
                    cam_run_info_[cam_id].last_color_msg_time = 0;
                    cam_run_info_[cam_id].last_depth_msg_time = 0;
                    cam_run_info_[cam_id].last_cloud_msg_time = 0;
                                    
                    // 重置流启动状态标志
                    cam_run_info_[cam_id].is_color_stream_start = false;
                    cam_run_info_[cam_id].is_depth_stream_start = false;
                    cam_run_info_[cam_id].is_cloud_stream_start = false;
                }
                
                // 等待2秒，确保相机进程完全关闭
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // 取消图像订阅
                if (image_subscriber_map_.find(cam_id) != image_subscriber_map_.end())
                {
                    image_subscriber_map_.erase(cam_id);
                    LOG_INFO("相机 %d 图像话题订阅器已取消（相机关闭）", cam_id);
                }
                
                // 取消点云订阅
                if (pointcloud_subscriber_map_.find(cam_id) != pointcloud_subscriber_map_.end())
                {
                    pointcloud_subscriber_map_.erase(cam_id);
                    LOG_INFO("相机 %d 点云话题订阅器已取消（相机关闭）", cam_id);
                }
                
                // 关闭显示窗口并删除可视化实例
                for (auto stream_type : {CamMgr::CamStreamType::STREAM_COLOR, CamMgr::CamStreamType::STREAM_DEPTH})
                {
                    std::pair<int, CamMgr::CamStreamType> viz_key = std::make_pair(cam_id, stream_type);
                    auto it = visualization_manager_map_.find(viz_key);
                    if (it != visualization_manager_map_.end() && it->second)
                    {
                        // 先关闭窗口
                        it->second->closeWindows(false);
                        LOG_INFO("相机 %d 流类型 %d 显示窗口已关闭", cam_id, static_cast<int>(stream_type));
                    }
                }
            }
            else
            {
                LOG_ERROR("通过CamBase接口关闭相机 %d失败", cam_id);
            }

            return result;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("停止相机 %d进程时发生错误: %s", cam_id, e.what());
            return false;
        }
    }

    bool CamMgrRos::switch_cam_sence(int cam_id, int sence_id)
    {
        try
        {
            // 获取相机配置
            CamMgr::CamInfo cam_info;
            if (!config_mgr_.get_camera_config(cam_id, cam_info))
            {
                LOG_ERROR("获取相机 %d 配置失败", cam_id);
                return false;
            }
    
            // 检查相机实例是否存在
            auto it = camera_map_.find(cam_id);
            if (it == camera_map_.end())
            {
                LOG_ERROR("未找到相机 ID %d的实例", cam_id);
                return false;
            }
    
            // 通过相机基类接口委托场景切换实现
            CamMgr::CamRunInfo &camera_state = cam_run_info_[cam_id];
                
            // 设置场景切换标志，暂停所有相机的监控
            is_switching_scene.store(true);
                
            // 切换场景前设置为"启动中"状态（避免被误判为掉线）
            camera_state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_STARTING;
                
            bool result = it->second->switch_cam_sence(cam_id, sence_id, cam_info, camera_state);
    
            if (result)
            {
                LOG_INFO("通过 CamBase 接口成功切换相机 %d 场景 %d", cam_id, sence_id);
                    
                // 切换场景成功后，设置为正常运行状态并清除掉线标记
                camera_state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_NORMAL;
                camera_state.is_offline = false;
                    
                // 切换场景后，重新订阅图像和点云话题（因为不同场景的点云开关可能不同）
                subscribe_to_camera_images_clouds();
                    
                // 重置消息时间戳，避免切换后立即被判断为超时（在订阅之后刷新）
                int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                camera_state.last_color_msg_time = current_time;
                camera_state.last_depth_msg_time = current_time;
                camera_state.last_cloud_msg_time = current_time;
            }
            else
            {
                LOG_ERROR("通过 CamBase 接口切换相机 %d 场景 %d 失败", cam_id, sence_id);
                // 切换失败则设置为异常状态
                camera_state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_ERROR;
            }
    
            // 清除场景切换标志前，刷新所有相机的最后接收时间戳，避免误判掉线
            {
                std::lock_guard<std::mutex> lock(camera_state_mutex_);
                int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                for (auto &pair : cam_run_info_)
                {
                    // 只刷新非切换相机的时间戳（切换相机的已经在上文刷新）
                    if (pair.first != cam_id && pair.second.is_cam_open)
                    {
                        pair.second.last_color_msg_time = current_time;
                        pair.second.last_depth_msg_time = current_time;
                        pair.second.last_cloud_msg_time = current_time;
                        LOG_DEBUG("刷新相机 %d 的最后接收时间戳：%lld", pair.first, current_time);
                    }
                }
            }
    
            // 清除场景切换标志，恢复监控
            is_switching_scene.store(false);
            
            // 记录场景切换完成时间
            last_scene_switch_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
    
            return result;
        }
        catch (const std::exception &e)
        {
            // 发生异常时也要清除标志
            is_switching_scene.store(false);
            LOG_ERROR("切换相机 %d 场景 %d 参数时发生错误：%s", cam_id, sence_id, e.what());
            return false;
        }
    }

    void CamMgrRos::publish_all_camera_info()
    {
        try
        {
            // 发布所有已开启相机的内参
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                if (pair.second.is_cam_open) // 相机正在运行
                {
                    publish_camera_info(cam_id);
                }
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("发布所有相机内参时发生错误: %s", e.what());
        }
    }

    void CamMgrRos::publish_camera_info(int cam_id)
    {
        try
        {
            // 性能日志：记录开始时间
            auto start_time = std::chrono::steady_clock::now();
                
            // 检查是否有对应的发布器
            auto it = camera_info_publisher_map_.find(cam_id);
            if (it != camera_info_publisher_map_.end())
            {
                // 发布内参消息
                auto publish_start = std::chrono::steady_clock::now();
                it->second->publish(camera_info_msg_map_[cam_id]);
                auto publish_end = std::chrono::steady_clock::now();
                    
                LOG_INFO("发布相机 %d 内参", cam_id);
                LOG_DEBUG("[性能] 发布相机 %d 内参耗时：%ld ms", 
                         cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(publish_end - publish_start).count());
                    
                auto end_time = std::chrono::steady_clock::now();
                LOG_DEBUG("[性能] 发布相机 %d 内参总耗时（含查找）：%ld ms", 
                         cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("发布相机 %d 内参时发生错误：%s", cam_id, e.what());
        }
    }

    void CamMgrRos::save_camera_intrinsics_to_json(int cam_id, const sensor_msgs::msg::CameraInfo &forward_msg)
    {
        try
        {
            CamMgr::CamInfo cam_info;
            if (!config_mgr_.get_camera_config(cam_id, cam_info))
            {
                return;
            }
            // 生成文件路径
            int cur_sence_id = cam_run_info_[cam_id].cur_sence_id;
            std::string file_path = config_mgr_.cam_cfg_dir_ + "/cam_" + std::to_string(cam_id) + "/cam_" + std::to_string(cam_id) + "_intrinsics_" + std::to_string(cam_info.sence_para[cur_sence_id].color_para.width) + "x" + std::to_string(cam_info.sence_para[cur_sence_id].color_para.height) + ".json";

            // 创建JSON对象并填充数据
            nlohmann::json intrinsics_json;

            // 添加基本信息
            intrinsics_json["camera_id"] = cam_id;
            intrinsics_json["version"] = "1.0";
            intrinsics_json["description"] = "相机内参";
            intrinsics_json["timestamp"] = cam_mgr_ros::get_current_timestamp();

            // 添加分辨率信息
            nlohmann::json resolution_json;
            resolution_json["width"] = forward_msg.width;
            resolution_json["height"] = forward_msg.height;
            intrinsics_json["resolution"] = resolution_json;

            // 添加内参信息
            nlohmann::json intrinsics_data_json;
            intrinsics_data_json["fx"] = forward_msg.k[0];  // fx
            intrinsics_data_json["fy"] = forward_msg.k[4];  // fy
            intrinsics_data_json["ppx"] = forward_msg.k[2]; // cx
            intrinsics_data_json["ppy"] = forward_msg.k[5]; // cy
            intrinsics_data_json["model"] = "distortion.inverse_brown_conrady";
            intrinsics_json["intrinsics"] = intrinsics_data_json;

            // 添加相机矩阵
            nlohmann::json camera_matrix_json = nlohmann::json::array();
            camera_matrix_json.push_back({forward_msg.k[0], forward_msg.k[1], forward_msg.k[2]});
            camera_matrix_json.push_back({forward_msg.k[3], forward_msg.k[4], forward_msg.k[5]});
            camera_matrix_json.push_back({forward_msg.k[6], forward_msg.k[7], forward_msg.k[8]});
            intrinsics_json["camera_matrix"] = camera_matrix_json;

            // 添加畸变系数
            nlohmann::json distortion_json;
            nlohmann::json coefficients_json = nlohmann::json::array();
            for (size_t i = 0; i < forward_msg.d.size() && i < 5; i++)
            {
                coefficients_json.push_back(forward_msg.d[i]);
            }
            // 确保有5个系数
            while (coefficients_json.size() < 5)
            {
                coefficients_json.push_back(0.0);
            }
            distortion_json["coefficients"] = coefficients_json;
            distortion_json["model"] = "Brown-Conrady";
            intrinsics_json["distortion"] = distortion_json;

            // 写入文件
            std::ofstream file(file_path);
            if (file.is_open())
            {
                file << intrinsics_json.dump(2);
                file.close();
                LOG_INFO("成功保存相机 %d 内参到文件: %s", cam_id, file_path.c_str());
            }
            else
            {
                LOG_WARN("无法打开文件保存相机 %d 内参: %s", cam_id, file_path.c_str());
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("保存相机 %d 内参到JSON文件时发生错误: %s", cam_id, e.what());
        }
    }
    
    void CamMgrRos::image_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type)
    {
        // 更新最后消息时间戳
        int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
        {
            cam_run_info_[cam_id].last_color_msg_time = current_time;
        }
        else if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
        {
            cam_run_info_[cam_id].last_depth_msg_time = current_time;
        }
        
        // 快速检查是否需要保存图像（避免不必要的任务入队）
        bool should_save = false;
        {
            std::lock_guard<std::mutex> lock(image_save_mutex_);
            auto cam_it = image_save_config_.find(cam_id);
            if (cam_it != image_save_config_.end())
            {
                auto stream_it = cam_it->second.find(stream_type);
                if (stream_it != cam_it->second.end() && stream_it->second)
                {
                    should_save = true;
                }
            }
        }
        
        // 跳过 IR 流
        if (stream_type == CamMgr::CamStreamType::STREAM_IR)
        {
            should_save = false;
        }
                
        // 只有在需要保存时才统计接收数量
        if (should_save)
        {
            // 统计接收到的图像数量
            auto key = std::make_pair(cam_id, stream_type);
            image_received_count_[key]++;
        }
        
        // 接收到图像后，如果当前状态是“启动中”，则更新为“正常”
        if (cam_run_info_[cam_id].cam_run_state == CamMgr::CamRunState::CAM_RUN_STATE_STARTING)
        {
            cam_run_info_[cam_id].cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_NORMAL;
        }
        
        // 快速检查是否需要显示图像
        bool should_display = false;
        {
            CamMgr::CamInfo cam_info;
            if (config_mgr_.get_camera_config(cam_id, cam_info))
            {
                should_display = cam_info.show_topic_image;
            }
        }

        // 只有需要显示时才将任务添加到显示队列
        if (should_display)
        {
            std::lock_guard<std::mutex> lock(display_task_mutex_);
            
            // 限制队列大小，避免显示滞后
            const size_t MAX_DISPLAY_QUEUE_SIZE = 3; // 只保留最近 3 张图像
            if (display_tasks_.size() > MAX_DISPLAY_QUEUE_SIZE)
            {
                // 清空队列，只保留最新的图像
                std::queue<DisplayTask> empty_queue;
                std::swap(display_tasks_, empty_queue);
                LOG_DEBUG("显示队列过大，已清空以避免滞后");
            }
            
            display_tasks_.push({*msg, cam_id, stream_type});
            display_task_cv_.notify_one();
        }

        // 只有需要保存时才将任务添加到队列
        if (should_save)
        {
            // 如果该相机的保存线程未启动，则启动它
            start_camera_save_thread(cam_id);
            
            ImageSaveTask task;
            task.image_msg = *msg;
            task.cam_id = cam_id;
            task.stream_type = stream_type;
            
            std::lock_guard<std::mutex> lock(image_save_queue_mutex_[cam_id]);
            image_save_queues_[cam_id].push(task);
            image_save_queue_cv_[cam_id].notify_one();
        }
    }
    
    void CamMgrRos::image_save_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type)
    {
        try
        {
            bool need_save = false;
            std::string save_dir_base;
            std::string file_extension;
            {
                std::lock_guard<std::mutex> save_lock(image_save_mutex_);
                
                // 获取相机配置
                CamMgr::CamInfo cam_info;
                if (!config_mgr_.get_camera_config(cam_id, cam_info))
                {
                    LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                    return;
                }
                
                // 检查是否需要保存图像
                auto it = image_save_config_.find(cam_id);
                if (it != image_save_config_.end())
                {
                    auto stream_it = it->second.find(stream_type);
                    if (stream_it != it->second.end())
                    {
                        need_save = stream_it->second;
                    }
                }
                
                if (!need_save)
                {
                    return;
                }
                
                // 复制需要的共享数据（在锁内）
                std::string install_path = basmodule::get_install_dir();
                save_dir_base = install_path + "/Images/" + current_save_batch_timestamp_ + "/cam_" + std::to_string(cam_id);
                file_extension = image_save_format_;
            }  // ✅ 立即释放锁

            // 根据流类型选择子目录
            std::string stream_dir;
            switch (stream_type)
            {
            case CamMgr::CamStreamType::STREAM_COLOR:
                stream_dir = "/color/";
                break;
            case CamMgr::CamStreamType::STREAM_DEPTH:
                stream_dir = "/depth/";
                break;
            default:
                return;
            }

            std::string save_dir = save_dir_base + stream_dir;
            
            // 每次保存前检查目录是否存在，不存在则创建
            if (!std::filesystem::exists(save_dir)) {
                std::filesystem::create_directories(save_dir);
            }

            // 使用消息中的时间戳生成文件名
            std::string timestamp = cam_mgr_ros::format_timestamp_from_msg(msg->header.stamp);
            
            // 使用封装的 convert_ros_to_cv_image 函数转换
            cv_bridge::CvImagePtr cv_ptr = cam_mgr_ros::convert_ros_to_cv_image(msg, stream_type);
            if (cv_ptr == nullptr)
            {
                LOG_ERROR("转换 ROS 图像到 OpenCV 格式失败");
                return;
            }
            
            // 根据配置的格式生成文件名
            if (file_extension.empty())
            {
                file_extension = "bmp";  // 默认为 bmp
            }
                        
            std::string filename = save_dir + timestamp + "." + file_extension;
                        
            // 使用封装的 save_image_to_file 函数保存图像
            bool save_success = cam_mgr_ros::save_image_to_file(cv_ptr->image, file_extension, filename);
                        
            if (save_success)
            {
                LOG_DEBUG("保存图像到：%s", filename.c_str());
                // 统计保存的图像数量
                auto key = std::make_pair(cam_id, stream_type);
                image_saved_count_[key]++;
            }
            else
            {
                LOG_ERROR("保存图像失败：%s", filename.c_str());
            }
        }
        catch (const std::exception &e)
        {
            LOG_WARN("保存图像失败: %s", e.what());
        }
    }

    void CamMgrRos::pointcloud_save_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int cam_id)
    {
        try
        {
            // Step 1: 短时间加锁，只读取配置和共享数据
            bool need_save = false;
            std::string save_dir_base;
            std::string file_extension;
            
            {
                std::lock_guard<std::mutex> save_lock(image_save_mutex_);
                
                // 获取相机配置
                CamMgr::CamInfo cam_info;
                if (!config_mgr_.get_camera_config(cam_id, cam_info))
                {
                    LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                    return;
                }
                
                // 检查是否需要保存点云
                auto it = image_save_config_.find(cam_id);
                if (it != image_save_config_.end())
                {
                    auto stream_it = it->second.find(CamMgr::CamStreamType::STREAM_CLOUD);
                    if (stream_it != it->second.end())
                    {
                        need_save = stream_it->second;
                    }
                }
                
                if (!need_save)
                {
                    return;
                }
                
                // 复制需要的共享数据（在锁内）
                std::string install_path = basmodule::get_install_dir();
                save_dir_base = install_path + "/PCDs/" + current_save_batch_timestamp_ + "/cam_" + std::to_string(cam_id) + "/cloud/";
                file_extension = "pcd";  // 点云固定为 pcd 格式
            }  // ✅ 立即释放锁
            
            if (!need_save)
            {
                return;
            }
            
            std::string install_path = basmodule::get_install_dir();
            std::string save_dir = save_dir_base + "/cloud/";
            
            // 每次保存前检查目录是否存在，不存在则创建
            if (!std::filesystem::exists(save_dir)) {
                std::filesystem::create_directories(save_dir);
            }

            // 使用消息中的时间戳生成文件名
            std::string timestamp = cam_mgr_ros::format_timestamp_from_msg(msg->header.stamp);
            
            // 保存为 PCD 格式
            std::string filename = save_dir + "cloud_" + timestamp + ".pcd";
            bool save_success = cam_mgr_ros::save_pointcloud_to_pcd(msg, filename);
        }
        catch (const std::exception &e)
        {
            LOG_WARN("保存点云失败: %s", e.what());
        }
    }
    
    void CamMgrRos::image_display_callback_with_stream_type(const sensor_msgs::msg::Image::SharedPtr msg, int cam_id, CamMgr::CamStreamType stream_type)
    {
        try
        {
            // 首先检查是否需要显示该相机的图像
            CamMgr::CamInfo cam_info;
            if (config_mgr_.get_camera_config(cam_id, cam_info))
            {
                if (!cam_info.show_topic_image)
                {
                    return;
                }
            }
            else
            {
                LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                return;
            }
            
            // 使用封装的 convert_ros_to_cv_image 函数转换
            cv_bridge::CvImagePtr cv_ptr = cam_mgr_ros::convert_ros_to_cv_image(msg, stream_type);
            if (cv_ptr == nullptr)
            {
                LOG_ERROR("转换 ROS 图像到 OpenCV 格式失败");
                return;
            }
            
            // 获取图像尺寸
            int width = cv_ptr->image.cols;
            int height = cv_ptr->image.rows;

            // 获取话题名称
            std::string topic_name = get_camera_image_topic(cam_id, stream_type);

            // 在图像上绘制文本信息
            std::string stream_type_str = (stream_type == CamMgr::CamStreamType::STREAM_COLOR) ? "Color" : 
                                          (stream_type == CamMgr::CamStreamType::STREAM_DEPTH) ? "Depth" : "Unknown";
            std::string text = "Cam:" + std::to_string(cam_id) + " " + stream_type_str +
                               " Size:" + std::to_string(width) + "x" + std::to_string(height) +
                               " Topic:" + topic_name;
            
            // 对于深度图像，转换为伪彩色图像
            cv::Mat display_image;
            if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH && 
                (cv_ptr->image.type() == CV_32FC1 || cv_ptr->image.type() == CV_16UC1))
            {
                cv::Mat depth_normalized;
                cv::normalize(cv_ptr->image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                cv::applyColorMap(depth_normalized, display_image, cv::COLORMAP_JET);
            }
            else
            {
                display_image = cv_ptr->image.clone();
            }
            
            // 在图像上绘制文本信息（在伪彩色转换后执行）
            cam_mgr_ros::draw_text(display_image, text, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), true, cv::Scalar(0, 0, 0), 1, cv::Point(10, 20));

            // 显示图像
            std::pair<int, CamMgr::CamStreamType> viz_key = std::make_pair(cam_id, stream_type);
            auto it = visualization_manager_map_.find(viz_key);
            std::string window_name = "Camera " + std::to_string(cam_id) + " " + stream_type_str;
            if (it != visualization_manager_map_.end() && it->second)
            {
                if (!it->second->isWindowOpen())
                {
                    it->second->initialize(window_name);
                }
                it->second->showImage(display_image);
            }
            else
            {
                auto viz_manager = std::make_shared<visualization::VisualizationMgr>();
                viz_manager->initialize(window_name);
                visualization_manager_map_[viz_key] = viz_manager;
                viz_manager->showImage(display_image);
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("图像显示回调处理失败: %s", e.what());
        }
    }

    void CamMgrRos::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int cam_id)
    {
        try
        {
            // 更新最后消息时间戳
            int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            cam_run_info_[cam_id].last_cloud_msg_time = current_time;
                
            // 接收到点云后，如果当前状态是“启动中”，则更新为“正常”
            if (cam_run_info_[cam_id].cam_run_state == CamMgr::CamRunState::CAM_RUN_STATE_STARTING)
            {
                cam_run_info_[cam_id].cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_NORMAL;
            }
                
            LOG_DEBUG("收到相机 %d 的点云数据，宽度：%d, 高度：%d, 点数：%lu", 
                     cam_id, msg->width, msg->height, msg->width * msg->height);

            // 快速检查是否需要保存点云
            bool should_save = false;
            {
                std::lock_guard<std::mutex> lock(image_save_mutex_);
                auto cam_it = image_save_config_.find(cam_id);
                if (cam_it != image_save_config_.end())
                {
                    auto stream_it = cam_it->second.find(CamMgr::CamStreamType::STREAM_CLOUD);
                    if (stream_it != cam_it->second.end() && stream_it->second)
                    {
                        should_save = true;
                    }
                }
            }

            // 只有需要保存时才将任务添加到队列
            if (should_save)
            {
                // 如果该相机的保存线程未启动，则启动它
                start_camera_save_thread(cam_id);
                
                ImageSaveTask task;
                task.cloud_msg = *msg;
                task.cam_id = cam_id;
                task.stream_type = CamMgr::CamStreamType::STREAM_CLOUD;
                
                std::lock_guard<std::mutex> lock(image_save_queue_mutex_[cam_id]);
                image_save_queues_[cam_id].push(task);
                image_save_queue_cv_[cam_id].notify_one();
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("点云回调处理失败: %s", e.what());
        }
    }

    void CamMgrRos::check_and_subscribe_images()
    {
        try
        {
            // 检查已打开的相机数量
            int opened_count = 0;
            int subscribed_count = 0;

            // 遍历所有相机
            for (const auto &pair : cam_run_info_)
            {
                // 检查相机是否启用
                CamMgr::CamInfo cam_info;
                if (!config_mgr_.get_camera_config(pair.first, cam_info))
                {
                    LOG_WARN("无法获取相机 %d 的配置信息", pair.first);
                    continue;
                }

                // 如果相机未启用，跳过
                if (!cam_info.enable)
                {
                    LOG_INFO("相机 %d 未启用，跳过图像订阅", pair.first);
                    continue;
                }

                if (pair.second.is_cam_open)
                {
                    opened_count++;
                    // 检查是否已经订阅彩色和深度图像
                    auto it_sub = image_subscriber_map_.find(pair.first);
                    if (it_sub != image_subscriber_map_.end())
                    {
                        auto &stream_map = it_sub->second;
                        // 检查是否已订阅彩色或深度图像中的任意一个
                        if (stream_map.find(CamMgr::CamStreamType::STREAM_COLOR) != stream_map.end() ||
                            stream_map.find(CamMgr::CamStreamType::STREAM_DEPTH) != stream_map.end())
                        {
                            subscribed_count++;
                        }
                    }
                }
            } // 如果所有相机都已打开，或者至少有一个相机已打开，且还有未订阅的相机，则订阅图像话题
            if (opened_count > 0 && subscribed_count < opened_count)
            {
                LOG_INFO("检测到 %d 个已打开的相机，其中 %d 个尚未订阅，开始订阅图像话题", opened_count, opened_count - subscribed_count);
                subscribe_to_camera_images_clouds();
                subscribe_to_all_camera_info();
            }
            else if (opened_count > 0)
            {
                // LOG_INFO( "检测到 %d 个已打开的相机，均已订阅图像话题，无需重复订阅", opened_count);
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("检查并订阅图像时发生错误: %s", e.what());
        }
    }
    // 相机控制服务回调函数
    void CamMgrRos::camera_control_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::CameraControl::Request> request,
        const std::shared_ptr<custom_msgs_comm::srv::CameraControl::Response> response)
    {
        try
        {
            LOG_INFO("相机控制服务回调函数被调用");
            // 获取请求参数
            int cam_id = request->cam_id;
            int sence_id = request->sence_id;
            int operate_type = request->operate_type;
            LOG_INFO("接收到相机控制服务请求: 相机ID=%d, 场景ID=%d, 操作类型=%d", cam_id, sence_id, operate_type);
            // 检查相机是否启用
            CamMgr::CamInfo cam_info;
            if (!config_mgr_.get_camera_config(cam_id, cam_info))
            {
                response->success = false;
                response->message = "无法获取相机 " + std::to_string(cam_id) + " 的配置信息";
                LOG_ERROR("无法获取相机 %d 的配置信息", cam_id);
                return;
            }

            // 如果相机未启用，不参与相关处理
            if (!cam_info.enable)
            {
                response->success = false;
                response->message = "相机 " + std::to_string(cam_id) + " 未启用，无法执行操作";
                LOG_WARN("相机 %d 未启用，无法执行操作", cam_id);
                return;
            }

            // 根据操作类型执行相应操作
            switch (operate_type)
            {
            case CAMERA_OPERATION_START:
            {
                // 检查相机是否已经打开
                auto it = cam_run_info_.find(cam_id);
                if (it != cam_run_info_.end() && it->second.is_cam_open)
                {
                    LOG_INFO("相机 %d 已经在运行，无需重复打开", cam_id);
                    response->success = true;
                    response->message = "相机已经在运行";
                }
                else
                {
                    // 启动指定场景的相机进程
                    response->success = open_cam(cam_id, sence_id, std::nullopt);
                    response->message = response->success ? "相机打开成功" : "相机打开失败";
                }

                // 如果相机启动成功，则订阅图像话题并发布内参
                if (response->success)
                {
                    LOG_INFO("相机 %d 启动成功，准备订阅图像话题", cam_id);
                    // 订阅图像话题和内参话题
                    subscribe_to_camera_images_clouds();
                    subscribe_to_all_camera_info();
                    // 发布所有相机内参
                    publish_all_camera_info();
                }
                break;
            }

            case CAMERA_OPERATION_STOP:
            {
                // 检查相机是否已经关闭
                auto it = cam_run_info_.find(cam_id);
                if (it == cam_run_info_.end() || !it->second.is_cam_open)
                {
                    LOG_INFO("相机 %d 已经关闭，无需重复关闭", cam_id);
                    response->success = true;
                    response->message = "相机已经关闭";
                }
                else
                {
                    response->success = close_cam(cam_id);
                    response->message = response->success ? "相机关闭成功" : "相机关闭失败";
                }
                break;
            }

            case CAMERA_OPERATION_SWITCH_SCENE:
            {
                if (switch_cam_sence(cam_id, sence_id))
                {
                    // 场景切换成功后，重新发布所有相机内参
                    publish_all_camera_info();
                    response->success = true;
                }
                else
                {
                    response->success = false;
                    response->message = "场景切换失败";
                }
                break;
            }

            default:
            {
                response->success = false;
                response->message = "未知的操作类型: " + std::to_string(operate_type);
                LOG_ERROR("未知的操作类型: %d", operate_type);
                break;
            }
            }

            LOG_INFO("相机控制服务响应: %s", response->message.c_str());
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("相机控制服务回调时发生错误: %s", e.what());
            response->success = false;
            response->message = "服务回调错误: " + std::string(e.what());
        }
    }

    void CamMgrRos::get_cam_intr_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::GetCamIntr::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::GetCamIntr::Response> response)
    {
        try
        {
            int cam_id = request->cam_id;

            // 检查相机是否启用
            CamMgr::CamInfo cam_info;
            if (!config_mgr_.get_camera_config(cam_id, cam_info))
            {
                response->success = false;
                response->message = "无法获取相机 " + std::to_string(cam_id) + " 的配置信息";
                LOG_ERROR("无法获取相机 %d 的配置信息", cam_id);
                return;
            }

            // 如果相机未启用，不参与相关处理
            if (!cam_info.enable)
            {
                response->success = false;
                response->message = "相机 " + std::to_string(cam_id) + " 未启用，无法获取内参";
                LOG_WARN("相机 %d 未启用，无法获取内参", cam_id);
                return;
            }
            
            // 检查是否已接收到该相机的内参
            if (is_intrinsics_received(cam_id))
            { // 从内部映射表获取内参信息
                auto it = camera_info_msg_map_.find(cam_id);
                if (it != camera_info_msg_map_.end())
                {
                    response->camera_info = it->second;
                    response->success = true;
                    response->message = "成功获取相机 " + std::to_string(cam_id) + " 内参";
                    LOG_INFO("成功响应相机 %d 内参服务请求", cam_id);
                }
                else
                {
                    response->success = false;
                    response->message = "未找到相机 " + std::to_string(cam_id) + " 内参数据";
                    LOG_WARN("未找到相机 %d 内参数据", cam_id);
                }
            }
            else
            {
                response->success = false;
                response->message = "相机 " + std::to_string(cam_id) + " 内参尚未接收到";
                LOG_WARN("相机 %d 内参尚未接收到", cam_id);
            }
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = "处理相机内参服务请求时发生错误: " + std::string(e.what());
            LOG_ERROR("处理相机内参服务请求时发生错误: %s", e.what());
        }
    }

    void CamMgrRos::set_image_save_service_callback(
        const std::shared_ptr<custom_msgs_comm::srv::SetImageSave::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::SetImageSave::Response> response)
    {
        try
        {
            int operate_type = request->operate_type;
            std::vector<int32_t> cam_ids = request->cam_ids;
            std::vector<int32_t> sence_ids = request->sence_ids;
            std::vector<int32_t> stream_types = request->stream_types;
            std::string save_path = request->save_path;  // 获取保存路径

            // 检查操作类型
            if (operate_type < 0 || operate_type > 3)
            {
                response->success = false;
                response->message = "无效的操作类型: " + std::to_string(operate_type) + "，有效值为 0(关闭) 或 1(开启) 或 2(开启bag录制) 或 3(停止bag录制)";
                LOG_ERROR("设置图像保存服务: 无效的操作类型 %d", operate_type);
                return;
            }
            else
            {
                LOG_INFO("设置图像保存服务: 操作类型 %d", operate_type);
            }
            bool enable_save = (operate_type == 1);
            bool disable_save = (operate_type == 0);
            bool enable_bag_record = (operate_type == 2);
            bool stop_bag_record = (operate_type == 3);
            
            // 防护：如果正在执行相反的任务，先停止它
            if (enable_save && is_bag_recording_.load())
            {
                LOG_WARN("检测到 BAG 录制正在进行，将先停止录制再启动存图");
                
                // 手动触发停止 BAG 录制的逻辑（参考 operate_type=3 的实现）
                std::lock_guard<std::mutex> lock(bag_record_mutex_);
                
                LOG_INFO("开始停止 bag 录制，当前 bag_record_pid_: %d", bag_record_pid_);
                
                // 停止 bag 录制进程
                if (bag_record_pid_ > 0)
                {
                    // 检查进程是否存在
                    if (kill(bag_record_pid_, 0) == 0)
                    {
                        LOG_INFO("bag 录制进程 %d 存在，准备停止", bag_record_pid_);
                        
                        // 只杀死特定进程组的进程
                        char kill_cmd[256];
                        snprintf(kill_cmd, sizeof(kill_cmd), "pkill -P %d", bag_record_pid_);
                        system(kill_cmd);
                        
                        // 同时也发送 SIGTERM 到 shell 进程
                        kill(bag_record_pid_, SIGTERM);
                        LOG_INFO("已发送 SIGTERM 到 bag 录制进程 %d", bag_record_pid_);
                    }
                    else
                    {
                        LOG_WARN("bag 录制进程 %d 不存在，跳过", bag_record_pid_);
                    }
                }
                
                bag_record_config_.clear();
                bag_record_start_time_ms_ = 0;
                
                // 等待进程结束，最多等待 5 秒
                if (bag_record_pid_ > 0)
                {
                    int status;
                    int timeout_ms = 5000; // 5 秒超时
                    int step_ms = 100;
                    int elapsed = 0;
                    bool exited = false;

                    while (elapsed < timeout_ms)
                    {
                        pid_t result = waitpid(bag_record_pid_, &status, WNOHANG);
                        if (result > 0)
                        {
                            LOG_INFO("bag 录制进程 %d 正常退出，状态：%d", bag_record_pid_, status);
                            exited = true;
                            break;
                        }
                        else if (result == -1 && errno == ECHILD)
                        {
                            LOG_INFO("bag 录制进程 %d 已不存在", bag_record_pid_);
                            exited = true;
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
                        elapsed += step_ms;

                        // 检查是否超时
                        if (elapsed >= timeout_ms)
                        {
                            LOG_WARN("bag 录制进程 %d 等待超时，尝试正常终止", bag_record_pid_);

                            // 先尝试发送 SIGTERM
                            kill(bag_record_pid_, SIGTERM);
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                            // 再次检查是否已退出
                            result = waitpid(bag_record_pid_, &status, WNOHANG);
                            if (result > 0)
                            {
                                LOG_INFO("bag 录制进程 %d 在 SIGTERM 后正常退出", bag_record_pid_);
                                exited = true;
                                break;
                            }

                            // 如果仍未退出，强制终止
                            LOG_WARN("bag 录制进程 %d 未响应 SIGTERM，强制终止", bag_record_pid_);
                            kill(bag_record_pid_, SIGKILL);

                            // 等待进程被杀死
                            int kill_elapsed = 0;
                            while (kill_elapsed < 1000) // 最多等待 1 秒
                            {
                                result = waitpid(bag_record_pid_, &status, WNOHANG);
                                if (result > 0 || (result == -1 && errno == ECHILD))
                                {
                                    LOG_INFO("bag 录制进程 %d 已被强制终止", bag_record_pid_);
                                    exited = true;
                                    break;
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                kill_elapsed += 100;
                            }
                            break;
                        }
                    }

                    if (!exited)
                    {
                        LOG_ERROR("无法终止 bag 录制进程 %d", bag_record_pid_);
                    }
                }
                
                bag_record_pid_ = 0;
                is_bag_recording_.store(false);
                LOG_INFO("BAG 录制已停止，可以开始存图");
            }
            
            if (enable_bag_record && is_image_saving_.load())
            {
                LOG_WARN("检测到图像保存正在进行，将先停止存图再启动录制");
                
                // 手动触发停止图像保存的逻辑
                std::lock_guard<std::mutex> lock(image_save_mutex_);
                
                for (auto &config_pair : image_save_config_)
                {
                    config_pair.second.clear();
                }
                save_start_time_ms_ = 0;
                is_image_saving_.store(false);
                
                // 停止所有保存线程
                for (auto& pair : image_save_queues_)
                {
                    int cam_id = pair.first;
                    stop_camera_save_thread(cam_id);
                }
                
                LOG_INFO("图像保存已停止，可以开始 BAG 录制");
            }

            // 处理 cam_ids: 如果只有 -1，表示所有相机
            std::vector<int> target_cam_ids;
            if (cam_ids.size() == 1 && cam_ids[0] == -1)
            {
                // 获取所有启用的相机ID
                for (const auto &pair : config_mgr_.camera_configs_)
                {
                    if (pair.second.enable)
                    {
                        target_cam_ids.push_back(pair.first);
                    }
                }
            }
            else
            {
                target_cam_ids.assign(cam_ids.begin(), cam_ids.end());
            }

            // 处理 sence_ids: 如果只有 -1，表示所有场景
            std::vector<int> target_sence_ids;
            if (sence_ids.size() == 1 && sence_ids[0] == -1)
            {
                // 根据每个相机的实际场景数量确定场景ID范围
                for (int cam_id : target_cam_ids)
                {
                    auto it = config_mgr_.camera_configs_.find(cam_id);
                    if (it != config_mgr_.camera_configs_.end())
                    {
                        const auto &cam_info = it->second;
                        // 根据 cam_info 中的 sence_para 数量添加实际场景ID
                        for (size_t i = 0; i < cam_info.sence_para.size(); i++)
                        {
                            target_sence_ids.push_back(i);
                        }
                    }
                }
                // 去重，确保每个场景ID只出现一次
                std::sort(target_sence_ids.begin(), target_sence_ids.end());
                target_sence_ids.erase(std::unique(target_sence_ids.begin(), target_sence_ids.end()), target_sence_ids.end());
            }
            else
            {
                target_sence_ids.assign(sence_ids.begin(), sence_ids.end());
            }

            // 处理 stream_types: 如果只有 -1，表示所有流类型
            std::vector<CamMgr::CamStreamType> target_stream_types;
            if (stream_types.size() == 1 && stream_types[0] == -1)
            {
                target_stream_types = {
                    CamMgr::CamStreamType::STREAM_COLOR,
                    CamMgr::CamStreamType::STREAM_DEPTH,
                    CamMgr::CamStreamType::STREAM_IR,
                    CamMgr::CamStreamType::STREAM_CLOUD};
            }
            else
            {
                for (int32_t st : stream_types)
                {
                    switch (st)
                    {
                    case 0:
                        target_stream_types.push_back(CamMgr::CamStreamType::STREAM_COLOR);
                        break;
                    case 1:
                        target_stream_types.push_back(CamMgr::CamStreamType::STREAM_DEPTH);
                        break;
                    case 2:
                        target_stream_types.push_back(CamMgr::CamStreamType::STREAM_IR);
                        break;
                    case 3:
                        target_stream_types.push_back(CamMgr::CamStreamType::STREAM_CLOUD);
                        break;
                    default:
                        LOG_WARN("设置图像保存服务: 未知的流类型 %d，已跳过", st);
                        break;
                    }
                }
            }

            // 更新图像保存配置
            {
                std::lock_guard<std::mutex> lock(image_save_mutex_);
                
                // 如果是开启存图，生成新的时间戳目录或使用自定义路径
                if (enable_save)
                {
                    auto now = std::chrono::system_clock::now();
                                    
                    // 如果 save_path 为空，使用时间戳；否则使用自定义路径
                    if (save_path.empty())
                    {
                        auto time_t = std::chrono::system_clock::to_time_t(now);
                        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                        std::stringstream ss;
                        ss << std::put_time(std::localtime(&time_t), "%Y%m%d%H%M%S");
                        ss << std::setfill('0') << std::setw(3) << ms.count();
                        current_save_batch_timestamp_ = ss.str();
                        LOG_INFO("开启存图，使用时间戳目录：%s", current_save_batch_timestamp_.c_str());
                    }
                    else
                    {
                        current_save_batch_timestamp_ = save_path;
                        LOG_INFO("开启存图，使用自定义路径目录：%s", current_save_batch_timestamp_.c_str());
                    }
                                    
                    // 记录存图开始时间
                    save_start_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                    
                    // 设置保存状态标志
                    is_image_saving_.store(true);
                    LOG_INFO("图像保存状态已设置为：正在保存");
                }
                
                for (int cam_id : target_cam_ids)
                {
                    if (enable_save)
                    {
                        // 开启存图：先清除该相机的所有流类型配置，然后只开启指定的流类型
                        image_save_config_[cam_id].clear();
                        for (CamMgr::CamStreamType stream_type : target_stream_types)
                        {
                            image_save_config_[cam_id][stream_type] = true;
                        }
                    }
                    else if (disable_save)
                    {
                        // operate_type=0：关闭所有相机的所有存图配置
                        // 先停止所有保存线程，等待队列处理完成后再统计
                        for (int stop_cam_id : target_cam_ids)
                        {
                            stop_camera_save_thread(stop_cam_id);
                        }
                        
                        // 等待一小段时间确保所有保存操作完成（避免统计时数据还在变化）
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        
                        // 打印统计信息
                        for (int stat_cam_id : target_cam_ids)
                        {
                            auto cam_it = image_save_config_.find(stat_cam_id);
                            if (cam_it != image_save_config_.end())
                            {
                                for (const auto& stream_pair : cam_it->second)
                                {
                                    CamMgr::CamStreamType stream_type = stream_pair.first;
                                    auto key = std::make_pair(stat_cam_id, stream_type);
                                    
                                    uint64_t received = image_received_count_[key].load();
                                    uint64_t saved = image_saved_count_[key].load();
                                    double rate = (received > 0) ? (static_cast<double>(saved) / static_cast<double>(received) * 100.0) : 0.0;
                                    
                                    std::string stream_name = "未知";
                                    switch (stream_type) {
                                        case CamMgr::CamStreamType::STREAM_COLOR: stream_name = "彩色"; break;
                                        case CamMgr::CamStreamType::STREAM_DEPTH: stream_name = "深度"; break;
                                        case CamMgr::CamStreamType::STREAM_IR: stream_name = "红外"; break;
                                        case CamMgr::CamStreamType::STREAM_CLOUD: stream_name = "点云"; break;
                                    }
                                    
                                    LOG_INFO("[相机 %d %s] 最终统计：接收=%lu 张，保存=%lu 张，保存率=%.1f%%",
                                             stat_cam_id, stream_name.c_str(), received, saved, rate);
                                    
                                    // 清空统计数据
                                    image_received_count_[key].store(0);
                                    image_saved_count_[key].store(0);
                                }
                            }
                        }
                        
                        // 清空配置
                        for (int clear_cam_id : target_cam_ids)
                        {
                            image_save_config_[clear_cam_id].clear();
                        }
                        
                        // 提交压缩任务到线程池（在统计完成后，异步压缩该批次图像）
                        // 使用局部变量保存当前批次，避免重复提交
                        std::string current_batch = current_save_batch_timestamp_;
                        if (!current_batch.empty())
                        {
                            // 构建源目录和输出 zip 路径
                            std::string install_path = basmodule::get_install_dir();
                            std::string source_dir = install_path + "/Images/" + current_batch;
                            std::string output_zip = source_dir + "/" + current_batch + ".zip";
                            
                            LOG_INFO("准备提交压缩任务：[%s] -> [%s]", source_dir.c_str(), output_zip.c_str());
                            
                            // 检查目录是否存在
                            if (std::filesystem::exists(source_dir) && std::filesystem::is_directory(source_dir))
                            {
                                submit_compress_task(source_dir, output_zip);
                                // 立即清空当前批次时间戳，防止重复提交压缩任务
                                current_save_batch_timestamp_.clear();
                            }
                            else
                            {
                                LOG_WARN("存图目录不存在，跳过压缩：%s", source_dir.c_str());
                            }
                        }
                    }
                    else
                    {
                        // 其他情况（operate_type=2/3）：不处理图像保存配置
                    }
                }
                
                // 如果是 disable_save，重置保存状态标志
                if (disable_save)
                {
                    is_image_saving_.store(false);
                    save_start_time_ms_ = 0;
                    LOG_INFO("图像保存状态已设置为：未保存");
                }
            }

            // 处理 ros2 bag 录制
            if (enable_bag_record)
            {
                std::lock_guard<std::mutex> lock(bag_record_mutex_);
                
                // 检查是否已有正在运行的bag录制进程
                if (bag_record_pid_ > 0)
                {
                    // 检查进程是否存在
                    if (kill(bag_record_pid_, 0) == 0)
                    {
                        LOG_WARN("已有bag录制进程正在运行 (PID: %d)，先停止它", bag_record_pid_);
                        // 停止现有的bag录制进程
                        kill(-bag_record_pid_, SIGTERM);
                        
                        // 等待进程结束，最多等待5秒
                        int status;
                        int timeout = 5;
                        int elapsed = 0;
                        bool exited = false;
                        
                        while (elapsed < timeout)
                        {
                            pid_t result = waitpid(-bag_record_pid_, &status, WNOHANG);
                            if (result > 0)
                            {
                                exited = true;
                                LOG_INFO("旧的bag录制进程组 %d 已正常退出", bag_record_pid_);
                                break;
                            }
                            else if (result == -1)
                            {
                                if (errno == ECHILD)
                                {
                                    LOG_INFO("旧的bag录制进程组 %d 已不存在", bag_record_pid_);
                                    exited = true;
                                }
                                break;
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            elapsed++;
                        }
                        
                        // 如果超时，强制杀死进程组
                        if (!exited)
                        {
                            LOG_WARN("旧的bag录制进程组 %d 未在 %d 秒内退出，强制终止", bag_record_pid_, timeout);
                            kill(-bag_record_pid_, SIGKILL);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }
                        
                        bag_record_pid_ = 0;
                        bag_record_config_.clear();
                        bag_record_start_time_ms_ = 0;
                    }
                    else
                    {
                        LOG_INFO("之前的bag录制进程 %d 已不存在，可以启动新的录制", bag_record_pid_);
                        bag_record_pid_ = 0;
                    }
                }
                
                // 生成新的时间戳目录或使用自定义路径
                auto now = std::chrono::system_clock::now();
                                
                // 如果 save_path 为空，使用时间戳；否则使用自定义路径
                if (save_path.empty())
                {
                    auto time_t = std::chrono::system_clock::to_time_t(now);
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                    std::stringstream ss;
                    ss << std::put_time(std::localtime(&time_t), "%Y%m%d%H%M%S");
                    ss << std::setfill('0') << std::setw(3) << ms.count();
                    current_save_batch_timestamp_ = ss.str();
                    LOG_INFO("开启 bag 录制，使用时间戳目录：%s", current_save_batch_timestamp_.c_str());
                }
                else
                {
                    current_save_batch_timestamp_ = save_path;
                    LOG_INFO("开启 bag 录制，使用自定义路径目录：%s", current_save_batch_timestamp_.c_str());
                }
                                
                // 记录 bag 录制开始时间
                bag_record_start_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                
                // 设置 BAG 录制状态标志
                is_bag_recording_.store(true);
                LOG_INFO("BAG 录制状态已设置为：正在录制");
                
                std::string install_path = basmodule::get_install_dir();
                
                // 移除路径末尾的斜杠
                while (!install_path.empty() && install_path.back() == '/')
                {
                    install_path.pop_back();
                }
                
                std::string bag_base_dir = install_path + "/Images/" + current_save_batch_timestamp_;
                
                bag_record_dir_ = bag_base_dir;
                
                // 创建bag录制目录
                std::filesystem::path bag_dir(bag_base_dir);
                try
                {
                    if (!std::filesystem::exists(bag_dir))
                    {
                        std::filesystem::create_directories(bag_dir);
                        LOG_INFO("创建bag录制目录: %s", bag_base_dir.c_str());
                    }
                }
                catch (const std::exception& e)
                {
                    LOG_ERROR("创建bag录制目录失败: %s", e.what());
                    response->success = false;
                    response->message = "创建bag录制目录失败";
                    return;
                }
                
                LOG_INFO("开启bag录制，存放路径: %s", bag_base_dir.c_str());
                LOG_INFO("开启bag录制，使用时间戳目录: %s，开始时间: %lld", current_save_batch_timestamp_.c_str(), bag_record_start_time_ms_);
                // 清理目录下可能存在的旧 bag 文件和子目录
            try
            {
                if (std::filesystem::exists(bag_dir))
                {
                    // 删除目录下的所有内容（包括 bag 文件、metadata 和图像子目录）
                    std::uintmax_t count = std::filesystem::remove_all(bag_dir);
                    LOG_INFO("清理旧 bag 目录及内容：%s (共删除 %lu 项)", bag_base_dir.c_str(), count);

                    // 重新创建空目录
                    std::filesystem::create_directories(bag_dir);
                    LOG_INFO("重新创建 bag 录制目录：%s", bag_base_dir.c_str());
                }
            }
            catch (const std::exception& e)
            {
                LOG_ERROR("清理旧 bag 目录失败：%s", e.what());
                response->success = false;
                response->message = "清理旧 bag 目录失败：" + std::string(e.what());
                return;
            }

                // 更新bag录制配置
                for (int cam_id : target_cam_ids)
                {
                    // 开启bag录制：先清除该相机的所有流类型配置，然后只开启指定的流类型
                    bag_record_config_[cam_id].clear();
                    for (CamMgr::CamStreamType stream_type : target_stream_types)
                    {
                        bag_record_config_[cam_id][stream_type] = true;
                    }
                }
                
                // 构建ros2 bag record命令
                std::string bag_cmd = "ros2 bag record";
                std::vector<std::string> topics;
                
                for (int cam_id : target_cam_ids)
                {
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                        continue;
                    }
                    
                    auto camera_it = camera_map_.find(cam_id);
                    if (camera_it == camera_map_.end() || !camera_it->second)
                    {
                        LOG_WARN("无法找到相机 %d 的实例", cam_id);
                        continue;
                    }
                    
                    for (CamMgr::CamStreamType stream_type : target_stream_types)
                    {
                        if (bag_record_config_[cam_id][stream_type])
                        {
                            std::string topic_name = camera_it->second->get_camera_image_topic(cam_id, stream_type, cam_info);
                            if (!topic_name.empty())
                            {
                                bool stream_enabled = false;
                                
                                for (const auto& sence_pair : cam_info.sence_para)
                                {
                                    const CamMgr::CamSencePara& sence_para = sence_pair.second;
                                    
                                    if (stream_type == CamMgr::CamStreamType::STREAM_COLOR && sence_para.enable_color_stream)
                                    {
                                        stream_enabled = true;
                                        break;
                                    }
                                    else if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH && sence_para.enable_depth_stream)
                                    {
                                        stream_enabled = true;
                                        break;
                                    }
                                    else if (stream_type == CamMgr::CamStreamType::STREAM_IR && sence_para.enable_ir_stream)
                                    {
                                        stream_enabled = true;
                                        break;
                                    }
                                    else if (stream_type == CamMgr::CamStreamType::STREAM_CLOUD && sence_para.enable_cloud_stream)
                                    {
                                        stream_enabled = true;
                                        break;
                                    }
                                }
                                
                                if (stream_enabled)
                                {
                                    if (stream_type != CamMgr::CamStreamType::STREAM_IR)
                                    {
                                        topics.push_back(topic_name);
                                        LOG_INFO("添加录制话题: %s", topic_name.c_str());
                                    }
                                    else
                                    {
                                        LOG_INFO("跳过IR流录制: %s", topic_name.c_str());
                                    }
                                }
                                else
                                {
                                    LOG_INFO("相机 %d 的流类型 %d 未启用，跳过录制", cam_id, static_cast<int>(stream_type));
                                }
                            }
                        }
                    }
                }
                
                if (topics.empty())
                {
                    LOG_WARN("没有可录制的话题");
                    response->success = false;
                    response->message = "没有可录制的话题";
                    return;
                }
                
                bag_cmd += " -o " + bag_base_dir + "/record.bag";
                for (const auto& topic : topics)
                {
                    bag_cmd += " " + topic;
                }
                
                LOG_INFO("执行bag录制命令: %s", bag_cmd.c_str());
                LOG_INFO("准备fork子进程启动bag录制...");
                
                // 创建子进程启动bag录制
                pid_t pid = fork();
                LOG_INFO("fork()返回，pid: %d", pid);
                
                if (pid == 0)
                {
                    //setpgid(0, 0);  // 创建新进程组
                    LOG_INFO("子进程启动，进程组ID: %d", getpgid(0));
                    execl("/bin/sh", "sh", "-c", bag_cmd.c_str(), (char*)NULL);
                    exit(127);
                }
                else if (pid > 0)
                {
                    //setpgid(pid, pid);  // 设置子进程为进程组组长
                    bag_record_pid_ = pid;
                    bag_record_pgid_ = pid;
                    LOG_INFO("bag录制进程已启动，PID: %d, PGID: %d", pid, pid);
                }
                else
                {
                    LOG_ERROR("启动bag录制进程失败，fork()返回-1，错误: %s", strerror(errno));
                    response->success = false;
                    response->message = "启动bag录制进程失败";
                    return;
                }
            }
            
            if (stop_bag_record)
            {
                std::lock_guard<std::mutex> lock(bag_record_mutex_);
                
                LOG_INFO("开始停止bag录制，当前bag_record_pid_: %d", bag_record_pid_);
                
                // 停止bag录制进程
                if (bag_record_pid_ > 0)
                {
                    // 检查进程是否存在
                    if (kill(bag_record_pid_, 0) == 0)
                    {
                        LOG_INFO("bag录制进程 %d 存在，准备停止", bag_record_pid_);
                        
                        // 只杀死特定进程组的进程
                        char kill_cmd[256];
                        snprintf(kill_cmd, sizeof(kill_cmd), "pkill -P %d", bag_record_pid_);
                        system(kill_cmd);
                        //LOG_INFO("执行pkill命令: %s, 返回值: %d", kill_cmd, result);
                        
                        // 同时也发送SIGTERM到shell进程
                        kill(bag_record_pid_, SIGTERM);
                        LOG_INFO("已发送SIGTERM到bag录制进程 %d", bag_record_pid_);
                    }
                    else
                    {
                        LOG_WARN("bag录制进程 %d 不存在，跳过", bag_record_pid_);
                    }
                }
                
                bag_record_config_.clear();
                bag_record_start_time_ms_ = 0;
                
                // 等待进程结束，最多等待5秒
                if (bag_record_pid_ > 0)
                {
                    int status;
                    int timeout_ms = 5000; // 5秒超时
                    int step_ms = 100;
                    int elapsed = 0;
                    bool exited = false;

                    while (elapsed < timeout_ms)
                    {
                        pid_t result = waitpid(bag_record_pid_, &status, WNOHANG);
                        if (result > 0)
                        {
                            LOG_INFO("bag录制进程 %d 正常退出，状态: %d", bag_record_pid_, status);
                            exited = true;
                            break;
                        }
                        else if (result == -1 && errno == ECHILD)
                        {
                            LOG_INFO("bag录制进程 %d 已不存在", bag_record_pid_);
                            exited = true;
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
                        elapsed += step_ms;

                        // 检查是否超时
                        if (elapsed >= timeout_ms)
                        {
                            LOG_WARN("bag录制进程 %d 等待超时，尝试正常终止", bag_record_pid_);

                            // 先尝试发送SIGTERM
                            kill(bag_record_pid_, SIGTERM);
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                            // 再次检查是否已退出
                            result = waitpid(bag_record_pid_, &status, WNOHANG);
                            if (result > 0)
                            {
                                LOG_INFO("bag录制进程 %d 在SIGTERM后正常退出", bag_record_pid_);
                                exited = true;
                                break;
                            }

                            // 如果仍未退出，强制终止
                            LOG_WARN("bag录制进程 %d 未响应SIGTERM，强制终止", bag_record_pid_);
                            kill(bag_record_pid_, SIGKILL);

                            // 等待进程被杀死
                            int kill_elapsed = 0;
                            while (kill_elapsed < 1000) // 最多等待1秒
                            {
                                result = waitpid(bag_record_pid_, &status, WNOHANG);
                                if (result > 0 || (result == -1 && errno == ECHILD))
                                {
                                    LOG_INFO("bag录制进程 %d 已被强制终止", bag_record_pid_);
                                    exited = true;
                                    break;
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                kill_elapsed += 100;
                            }
                            break;
                        }
                    }

                    if (!exited)
                    {
                        LOG_ERROR("无法终止bag录制进程 %d", bag_record_pid_);
                    }
                    
                }
                
                bag_record_pid_ = 0;
                LOG_INFO("已停止所有相机的 bag 录制功能");
                                
                // 重置 BAG 录制状态标志
                is_bag_recording_.store(false);
                LOG_INFO("BAG 录制状态已设置为：未录制");
                
                // 异步解析bag文件（在进程完全退出后）
                if (!bag_record_dir_.empty())
                {
                    std::string bag_path = bag_record_dir_ + "/record.bag";
                    std::string bag_dir_copy = bag_record_dir_;
                    bag_record_dir_.clear();
                    
                    // 额外等待1秒，确保数据库完全释放
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    std::thread([bag_path, bag_dir_copy, this]() {
                        LOG_INFO("开始异步解析bag文件: %s", bag_path.c_str());
                        if (cam_mgr_ros::parse_bag_file(bag_path, delete_bag_after_parse_))
                        {
                            LOG_INFO("bag文件解析成功: %s", bag_path.c_str());
                        }
                        else
                        {
                            LOG_WARN("bag文件解析失败: %s", bag_path.c_str());
                        }
                    }).detach();
                }
            }

            // 构建响应消息
            std::string operation_str;
            if (enable_save)
            {
                operation_str = "开启";
            }
            else if (enable_bag_record)
            {
                operation_str = "开启bag录制";
            }
            else if (stop_bag_record)
            {
                operation_str = "停止bag录制";
            }
            else
            {
                operation_str = "关闭";
            }
            std::string cam_str = (cam_ids.size() == 1 && cam_ids[0] == -1) ? "所有相机" : std::to_string(target_cam_ids.size()) + "个相机";
            std::string sence_str = (sence_ids.size() == 1 && sence_ids[0] == -1) ? "所有场景" : std::to_string(target_sence_ids.size()) + "个场景";
            std::string stream_str = (stream_types.size() == 1 && stream_types[0] == -1) ? "所有流" : std::to_string(target_stream_types.size()) + "种流";

            response->success = true;
            response->message = "成功" + operation_str + "图像保存功能: " + cam_str + ", " + sence_str + ", " + stream_str;
            LOG_INFO("设置图像保存服务: 成功%s图像保存功能 (%s, %s, %s)", operation_str.c_str(), cam_str.c_str(), sence_str.c_str(), stream_str.c_str());
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = "处理设置图像保存服务请求时发生错误: " + std::string(e.what());
            LOG_ERROR("处理设置图像保存服务请求时发生错误: %s", e.what());
        }
    }

    void CamMgrRos::save_timeout_check_callback()
    {
        // 检查存图超时
        if (save_start_time_ms_ != 0)
        {
            auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            auto elapsed_time = current_time - save_start_time_ms_;
            
            if (elapsed_time >= MAX_SAVE_TIME_MS)
            {
                LOG_WARN("存图时间已达到最大限制 %d 毫秒，将自动发送服务请求停止存图", MAX_SAVE_TIME_MS);
                
                // 通过服务接口停止存图（复用正常停止逻辑）
                auto request = std::make_shared<custom_msgs_comm::srv::SetImageSave::Request>();
                request->operate_type = 0;  // 关闭存图
                request->cam_ids = {-1};     // 所有相机
                request->sence_ids = {-1};   // 所有场景
                request->stream_types = {-1}; // 所有流类型
                
                auto response = std::make_shared<custom_msgs_comm::srv::SetImageSave::Response>();
                set_image_save_service_callback(request, response);
                
                if (response->success)
                {
                    LOG_INFO("超时自动停止存图成功");
                }
                else
                {
                    LOG_WARN("超时自动停止存图失败：%s", response->message.c_str());
                }
            }
        }
        
        // 检查 bag 录制超时
        if (bag_record_start_time_ms_ != 0)
        {
            auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            auto elapsed_time = current_time - bag_record_start_time_ms_;
            
            if (elapsed_time >= MAX_SAVE_TIME_MS)
            {
                LOG_WARN("bag 录制时间已达到最大限制 %d 毫秒，将自动发送服务请求停止录制", MAX_SAVE_TIME_MS);
                            
                // 通过服务接口停止 BAG 录制（复用正常停止逻辑）
                auto request = std::make_shared<custom_msgs_comm::srv::SetImageSave::Request>();
                request->operate_type = 3;  // 停止 bag 录制
                request->cam_ids = {-1};     // 所有相机
                request->sence_ids = {-1};   // 所有场景
                request->stream_types = {-1}; // 所有流类型
                            
                auto response = std::make_shared<custom_msgs_comm::srv::SetImageSave::Response>();
                set_image_save_service_callback(request, response);
                            
                if (response->success)
                {
                    LOG_INFO("超时自动停止 BAG 录制成功");
                }
                else
                {
                    LOG_WARN("超时自动停止 BAG 录制失败：%s", response->message.c_str());
                }
            }
        }
    }               
    
    bool CamMgrRos::set_camera_info_to_config()
    {
        try
        {
            LOG_INFO("config_mgr_.camera_configs_ 数量: %zu", config_mgr_.camera_configs_.size());

            for (auto &config_pair : config_mgr_.camera_configs_)
            {
                int cam_id = config_pair.first;
                CamMgr::CamInfo &cam_info = config_pair.second;

                LOG_INFO("cam_dev_list_ 数量: %zu", cam_dev_list_.size());

                for (const auto &dev_info : cam_dev_list_)
                {
                    // 如果序列号匹配，则设置相机型号
                    if (dev_info.serial_number == cam_info.serial_number)
                    {

                        cam_info.cam_model = dev_info.device_name;

                        // 使用封装的静态方法获取相机类型和型号类型
                        cam_info.cam_type = cam_mgr_ros::CamBase::get_cam_type_by_str(dev_info.device_name, dev_info.facturer_name);
                        cam_info.cam_model_type = cam_mgr_ros::CamBase::get_device_type_by_str(dev_info.device_name);

                        LOG_INFO("相机 ID %d 的序列号 %s 匹配成功，设置相机型号为: %s，型号类型为: %d",
                                 cam_id, cam_info.serial_number.c_str(), dev_info.device_name.c_str(), static_cast<int>(cam_info.cam_model_type));
                        break;
                    }
                }
                // 若设备名未能确定型号，则再尝试使用已有配置中的 cam_model 字段做二次匹配
                if (cam_info.cam_model_type == CamMgr::CamModelType::CAM_MODEL_NONE)
                {
                    cam_info.cam_model_type = cam_mgr_ros::CamBase::get_device_type_by_str(cam_info.cam_model);
                    LOG_INFO("相机 ID %d 的序列号 %s 匹配成功，型号类型为: %d cam_info.cam_model: %s",
                             cam_id, cam_info.serial_number.c_str(), static_cast<int>(cam_info.cam_model_type), cam_info.cam_model.c_str());
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("设置相机信息到配置参数时发生错误: %s", e.what());
            return false;
        }
    }

    void CamMgrRos::get_camera_roi_fps()
    {
        try
        {
            // 首先尝试从YAML文件加载相机的ROI和FPS信息
            load_camera_roi_fps_infos();

            // 遍历所有配置的相机，检查是否已有对应的ROI/FPS信息，如果没有则获取
            for (const auto &config_pair : config_mgr_.camera_configs_)
            {
                int cam_id = config_pair.first;
                const CamMgr::CamInfo &cam_info = config_pair.second;

                // 根据相机序列号检查是否已经有该相机的ROI/FPS信息
                bool has_roi_fps_info = cam_roi_fps_map_load_.find(cam_info.serial_number) != cam_roi_fps_map_load_.end();

                if (has_roi_fps_info)
                {
                    // 如果已从YAML加载，则将数据复制到cam_roi_fps_map_
                    cam_roi_fps_map_[cam_id] = cam_roi_fps_map_load_[cam_info.serial_number];
                    LOG_INFO("相机 %d (序列号: %s) 的ROI和FPS信息已从YAML文件加载", cam_id, cam_info.serial_number.c_str());
                }
                else
                {
                    // 如果没有该相机的ROI/FPS信息，则获取
                    std::map<std::string, CamMgr::CamRoiFpsList> sensor_roi_fps_map;

                    if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_RS)
                    {
                        CamRs::get_roi_fps_list_static(cam_id, sensor_roi_fps_map, cam_info);
                    }
                    else if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_OB)
                    {
                        CamOb::get_roi_fps_list_static(cam_id, sensor_roi_fps_map, cam_info);
                    }
                    else if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_CSI)
                    {
                        CamCsi::get_roi_fps_list_static(cam_id, sensor_roi_fps_map, cam_info);
                    }
                    cam_roi_fps_map_[cam_id] = sensor_roi_fps_map;
                    LOG_INFO("已获取相机 %d (序列号: %s) 的ROI和FPS信息", cam_id, cam_info.serial_number.c_str());
                    
                    // 将ROI和FPS数据赋值到相机子类成员变量
                    auto cam_it = camera_map_.find(cam_id);
                    if (cam_it != camera_map_.end())
                    {
                        if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_RS)
                        {
                            CamRs* cam_rs = dynamic_cast<CamRs*>(cam_it->second.get());
                            if (cam_rs != nullptr)
                            {
                                cam_rs->set_sensor_roi_fps_map(sensor_roi_fps_map);
                            }
                        }
                        else if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_OB)
                        {
                            CamOb* cam_ob = dynamic_cast<CamOb*>(cam_it->second.get());
                            if (cam_ob != nullptr)
                            {
                                cam_ob->set_sensor_roi_fps_map(sensor_roi_fps_map);
                            }
                        }
                        else if (cam_info.cam_type == CamMgr::CamType::CAM_TYPE_CSI)
                        {
                            CamCsi* cam_csi = dynamic_cast<CamCsi*>(cam_it->second.get());
                            if (cam_csi != nullptr)
                            {
                                cam_csi->set_sensor_roi_fps_map(sensor_roi_fps_map);
                            }
                        }
                    }
                }
            }

            // 创建文件流用于写入ROI和FPS数据（仅当启用保存设备信息时）
            for (const auto &cam_pair : cam_roi_fps_map_)
            {
                int cam_id = cam_pair.first;
                const auto &sensor_map = cam_pair.second;

                LOG_DEBUG("===== 相机 %d ROI和FPS信息 =====", cam_id);
                for (const auto &sensor_pair : sensor_map)
                {
                    const std::string &sensor_name = sensor_pair.first;
                    const CamMgr::CamRoiFpsList &roi_fps_list = sensor_pair.second;

                    LOG_DEBUG("传感器: %s", sensor_name.c_str());
                    for (const auto &roi_fps : roi_fps_list)
                    {
                        LOG_DEBUG("  分辨率: %dx%d, FPS: %d",
                                 roi_fps.width, roi_fps.height, roi_fps.fps);
                    }
                }
                LOG_DEBUG("===============================");
            }
            // 调用新接口保存为yaml格式
            save_camera_roi_fps_infos(0);
            save_camera_roi_fps_infos(1);
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("获取并打印相机ROI和FPS参数时发生错误: %s", e.what());
        }
    }

    bool CamMgrRos::save_camera_roi_fps_infos(int type)
    {
        try
        {
            if (type == 0)
            {
                // 执行现有代码，保存为txt文件

                if (is_save_device_info_)
                {
                    std::ofstream roi_fps_file;
                    std::string file_path = config_mgr_.cam_cfg_dir_ + "/camera_roi_fps_info.txt";
                    roi_fps_file.open(file_path);
                    if (roi_fps_file.is_open())
                    {
                        roi_fps_file << "相机ROI和FPS信息\n";
                        roi_fps_file << "===============================\n\n";
                    }
                    else
                    {
                        LOG_WARN("无法打开文件用于写入: %s", file_path.c_str());
                        return false;
                    }
                    // 写入获取到的ROI和FPS数据
                    for (const auto &cam_pair : cam_roi_fps_map_)
                    {
                        int cam_id = cam_pair.first;
                        const auto &sensor_map = cam_pair.second;

                        LOG_INFO("===== 相机 %d ROI和FPS信息 =====", cam_id);

                        // 写入文件
                        if (roi_fps_file.is_open())
                        {
                            roi_fps_file << "===== 相机 " << cam_id << " ROI和FPS信息 =====\n";
                        }

                        for (const auto &sensor_pair : sensor_map)
                        {
                            const std::string &sensor_name = sensor_pair.first;
                            const CamMgr::CamRoiFpsList &roi_fps_list = sensor_pair.second;

                            // 写入文件
                            if (roi_fps_file.is_open())
                            {
                                roi_fps_file << "传感器: " << sensor_name << "\n";
                            }

                            for (const auto &roi_fps : roi_fps_list)
                            {
                                // 写入文件
                                if (roi_fps_file.is_open())
                                {
                                    roi_fps_file << "  分辨率: " << roi_fps.width << "x" << roi_fps.height << ", FPS: " << roi_fps.fps << "\n";
                                }
                            }
                        }
                        LOG_INFO("===============================");

                        // 写入文件
                        if (roi_fps_file.is_open())
                        {
                            roi_fps_file << "===============================\n\n";
                        }
                    }

                    // 关闭文件
                    if (roi_fps_file.is_open())
                    {
                        roi_fps_file.close();
                        LOG_INFO("相机ROI和FPS信息已写入文件: %s/camera_roi_fps_info.txt", config_mgr_.cam_cfg_dir_.c_str());
                    }
                }
                return true;
            }
            else if (type == 1)
            {
                // 保存为yaml配置文件
                // 创建目录
                std::string base_path = ament_index_cpp::get_package_share_directory("cam_mgr_ros") + "/cam_data/";
                std::filesystem::create_directories(base_path);

                for (const auto &cam_pair : cam_roi_fps_map_)
                {
                    int cam_id = cam_pair.first;
                    const auto &sensor_map = cam_pair.second;

                    // 获取相机序列号
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                        continue;
                    }

                    std::string serial_number = cam_info.serial_number;
                    std::string yaml_file_path = base_path + "cam_" + serial_number + ".yaml";
                    // 将csi设备路径转换为安全的文件名
                    if(cam_info.cam_type == CamMgr::CamType::CAM_TYPE_CSI)
                    {
                        std::string safe_serial_number = serial_number;
                        safe_serial_number = safe_serial_number.replace(safe_serial_number.begin(), safe_serial_number.end(), '/', '_');
                        safe_serial_number = safe_serial_number.replace(safe_serial_number.begin(), safe_serial_number.end(), ':', '_');
                        yaml_file_path = base_path + "cam_" + safe_serial_number + ".yaml";
                    }   
                    // 创建YAML节点
                    YAML::Node cam_node;

                    // 添加相机基本信息
                    cam_node["cam_id"] = cam_id;
                    cam_node["serial_number"] = serial_number;
                    cam_node["cam_type"] = static_cast<int>(cam_info.cam_type);

                    // 添加传感器信息
                    for (const auto &sensor_pair : sensor_map)
                    {
                        const std::string &sensor_name = sensor_pair.first;
                        const CamMgr::CamRoiFpsList &roi_fps_list = sensor_pair.second;

                        YAML::Node sensor_node;
                        for (const auto &roi_fps : roi_fps_list)
                        {
                            YAML::Node roi_node;
                            roi_node["width"] = roi_fps.width;
                            roi_node["height"] = roi_fps.height;
                            roi_node["fps"] = roi_fps.fps;
                            sensor_node.push_back(roi_node);
                        }
                        cam_node["sensors"][sensor_name] = sensor_node;
                    }

                    // 保存YAML文件
                    std::ofstream yaml_file(yaml_file_path);
                    yaml_file << cam_node;
                    yaml_file.close();
                    LOG_INFO("相机 %d (序列号: %s) ROI和FPS信息已保存至: %s", cam_id, serial_number.c_str(), yaml_file_path.c_str());
                }

                // 生成并保存MD5校验文件
                std::string md5_file_path = base_path + "checksums.md5";
                std::ofstream md5_file(md5_file_path);

                // 遍历保存的yaml文件并计算MD5
                std::map<std::string, std::string> md5_map;
                for (const auto &cam_pair : cam_roi_fps_map_)
                {
                    int cam_id = cam_pair.first;

                    // 获取相机序列号
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                        continue;
                    }

                    std::string serial_number = cam_info.serial_number;
                    std::string yaml_file_path = base_path + "cam_" + serial_number + ".yaml";

                    // 计算yaml文件的MD5
                    std::ifstream read_file(yaml_file_path, std::ios::binary);
                    if (read_file.is_open())
                    {
                        std::string content((std::istreambuf_iterator<char>(read_file)),
                                            std::istreambuf_iterator<char>());
                        read_file.close();

                        // 使用简单哈希函数模拟MD5计算
                        std::hash<std::string> hash_fn;
                        size_t hash_value = hash_fn(content);
                        std::stringstream ss;
                        ss << std::hex << hash_value;

                        md5_map["cam_" + serial_number + ".yaml"] = ss.str();
                    }
                    else
                    {
                        LOG_WARN("无法打开文件进行MD5计算: %s", yaml_file_path.c_str());
                    }
                }

                // 将MD5值写入校验文件
                for (const auto &entry : md5_map)
                {
                    md5_file << entry.first << " " << entry.second << "\n";
                }
                md5_file.close();
                LOG_INFO("MD5校验文件已保存至: %s", md5_file_path.c_str());
                return true;
            }
            else
            {
                LOG_ERROR("未知的保存类型: %d", type);
                return false;
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("保存相机ROI和FPS信息时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::load_camera_roi_fps_infos()
    {
        try
        {
            // 清空之前加载的数据
            cam_roi_fps_map_load_.clear();

            std::string base_path = ament_index_cpp::get_package_share_directory("cam_mgr_ros") + "/cam_data/";

            // 检查目录是否存在
            if (!std::filesystem::exists(base_path))
            {
                LOG_WARN("目录不存在: %s", base_path.c_str());
                return false;
            }

            // 读取MD5校验文件
            std::map<std::string, std::string> stored_md5_map;
            std::string md5_file_path = base_path + "checksums.md5";
            if (std::filesystem::exists(md5_file_path))
            {
                std::ifstream md5_file(md5_file_path);
                std::string line;
                while (std::getline(md5_file, line))
                {
                    std::istringstream iss(line);
                    std::string filename, md5_value;
                    if (iss >> filename >> md5_value)
                    {
                        stored_md5_map[filename] = md5_value;
                    }
                }
                md5_file.close();
            }

            // 遍历目录中的所有cam_*文件
            for (const auto &entry : std::filesystem::directory_iterator(base_path))
            {
                if (entry.is_regular_file())
                {
                    std::string filename = entry.path().filename().string();
                    if (filename.substr(0, 4) == "cam_" && filename.substr(filename.length() - 5) == ".yaml")
                    {
                        // 提取序列号
                        std::string serial_number = filename.substr(4, filename.length() - 9); // 移除"cam_"前缀和".yaml"后缀
                        std::string file_path = entry.path().string();

                        // 进行MD5校验
                        bool is_valid = true;
                        if (stored_md5_map.count(filename)) // 使用完整的文件名作为键
                        {
                            // 读取文件内容
                            std::ifstream read_file(file_path, std::ios::binary);
                            std::string content((std::istreambuf_iterator<char>(read_file)),
                                                std::istreambuf_iterator<char>());
                            read_file.close();

                            // 计算当前文件的MD5
                            std::hash<std::string> hash_fn;
                            size_t hash_value = hash_fn(content);
                            std::stringstream ss;
                            ss << std::hex << hash_value;

                            std::string current_md5 = ss.str();
                            std::string stored_md5 = stored_md5_map[filename]; // 使用完整的文件名作为键

                            if (current_md5 != stored_md5)
                            {
                                LOG_WARN("文件校验失败: %s (期望: %s, 实际: %s)",
                                         filename.c_str(), stored_md5.c_str(), current_md5.c_str());
                                is_valid = false;
                            }
                            else
                            {
                                LOG_INFO("文件校验成功: %s", filename.c_str());
                            }
                        }
                        else
                        {
                            LOG_WARN("未找到校验信息: %s", filename.c_str());
                            // 如果未找到校验信息，仍允许加载（出于兼容性考虑）
                            // 如果需要严格校验，可取消下面一行的注释
                            // is_valid = false;
                        }

                        // 如果校验成功，读取YAML文件内容
                        if (is_valid)
                        {
                            try
                            {
                                YAML::Node cam_node = YAML::LoadFile(file_path);

                                int cam_id = cam_node["cam_id"].as<int>();
                                std::string serial_num = cam_node["serial_number"].as<std::string>();

                                // 创建传感器ROI和FPS映射
                                std::map<std::string, CamMgr::CamRoiFpsList> sensor_roi_fps_map;

                                if (cam_node["sensors"] && cam_node["sensors"].IsMap())
                                {
                                    for (YAML::const_iterator it = cam_node["sensors"].begin();
                                         it != cam_node["sensors"].end(); ++it)
                                    {
                                        std::string sensor_name = it->first.as<std::string>();
                                        YAML::Node sensor_node = it->second;

                                        CamMgr::CamRoiFpsList roi_fps_list;
                                        for (YAML::const_iterator jt = sensor_node.begin();
                                             jt != sensor_node.end(); ++jt)
                                        {
                                            YAML::Node roi_node = *jt;
                                            CamMgr::CamRoiFps roi_fps;
                                            roi_fps.width = roi_node["width"].as<int>();
                                            roi_fps.height = roi_node["height"].as<int>();
                                            roi_fps.fps = roi_node["fps"].as<int>();
                                            roi_fps_list.push_back(roi_fps);
                                        }
                                        sensor_roi_fps_map[sensor_name] = roi_fps_list;
                                    }
                                }

                                // 保存到cam_roi_fps_map_load_，以序列号为键
                                cam_roi_fps_map_load_[serial_num] = sensor_roi_fps_map;

                                LOG_INFO("成功读取相机 %d (序列号: %s) 的ROI和FPS信息", cam_id, serial_num.c_str());
                            }
                            catch (const YAML::Exception &e)
                            {
                                LOG_ERROR("解析YAML文件失败: %s, 错误: %s", file_path.c_str(), e.what());
                            }
                        }
                    }
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("读取相机ROI和FPS信息时发生错误: %s", e.what());
            return false;
        }
    }



    void CamMgrRos::subscribe_to_camera_images_clouds()
    {
        // 性能日志：记录函数开始时间
        auto start_time = std::chrono::steady_clock::now();
        
        try
        {
            LOG_INFO("[性能] 开始订阅相机图像和点云话题...");
            
            // 加锁保护，避免多线程并发访问
            auto lock_start = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(image_subscriber_mutex_);
            std::lock_guard<std::mutex> cloud_lock(pointcloud_subscriber_mutex_);
            auto lock_end = std::chrono::steady_clock::now();
            LOG_DEBUG("[性能] 获取锁耗时：%ld ms", 
                     std::chrono::duration_cast<std::chrono::milliseconds>(lock_end - lock_start).count());
            // 遍历所有已打开的相机
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                bool is_running = pair.second.is_cam_open;

                // 只为正在运行的相机订阅图像话题
                if (is_running)
                {
                    // 获取相机配置
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_ERROR("无法获取相机 %d 配置", cam_id);
                        continue;
                    }

                    // 检查相机是否启用
                    if (!cam_info.enable)
                    {
                        LOG_INFO("相机 %d 未启用，跳过图像订阅", cam_id);
                        continue;
                    }

                    // 获取当前场景ID
                    int cur_sence_id = cam_run_info_[cam_id].cur_sence_id;
                    
                    // 检查场景参数是否存在
                    if (cam_info.sence_para.find(cur_sence_id) == cam_info.sence_para.end())
                    {
                        LOG_WARN("相机 %d 场景 %d 参数不存在，跳过图像订阅", cam_id, cur_sence_id);
                        continue;
                    }

                    // 检查是否需要订阅彩色图像话题
                    bool enable_color = cam_info.sence_para[cur_sence_id].enable_color_stream;
                    if (enable_color)
                    {
                        // 检查是否已经订阅彩色流
                        if (image_subscriber_map_[cam_id].find(CamMgr::CamStreamType::STREAM_COLOR) == image_subscriber_map_[cam_id].end())
                        {
                            // 获取彩色图像话题名称
                            auto topic_start = std::chrono::steady_clock::now();
                            std::string color_topic = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                            auto topic_end = std::chrono::steady_clock::now();
                            LOG_DEBUG("[性能] 相机 %d 获取彩色话题耗时：%ld ms", 
                                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(topic_end - topic_start).count());
                        
                            // 创建彩色图像订阅器
                            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
                            qos_profile.depth = 10;
                            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
                                                    
                            auto sub_start = std::chrono::steady_clock::now();
                            auto color_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
                                color_topic, qos,
                                [this, cam_id](const sensor_msgs::msg::Image::SharedPtr msg)
                                {
                                    this->image_callback_with_stream_type(msg, cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                                });
                            auto sub_end = std::chrono::steady_clock::now();
                            LOG_DEBUG("[性能] 相机 %d 创建彩色订阅器耗时：%ld ms", 
                                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(sub_end - sub_start).count());
                        
                            // 将订阅器添加到映射表中
                            image_subscriber_map_[cam_id][CamMgr::CamStreamType::STREAM_COLOR] = color_subscriber;
                        
                            // 设置彩色流启动状态为 true
                            cam_run_info_[cam_id].is_color_stream_start = true;
                        
                            LOG_INFO("相机 %d 彩色图像话题订阅器已创建：%s", cam_id, color_topic.c_str());
                        }
                    }
                    else
                    {
                        // 彩色流未启用，取消订阅
                        if (image_subscriber_map_[cam_id].find(CamMgr::CamStreamType::STREAM_COLOR) != image_subscriber_map_[cam_id].end())
                        {
                            image_subscriber_map_[cam_id].erase(CamMgr::CamStreamType::STREAM_COLOR);
                            
                            // 设置彩色流启动状态为false
                            cam_run_info_[cam_id].is_color_stream_start = false;
                            
                            LOG_INFO("相机 %d 彩色图像话题订阅器已取消（场景 %d 未启用彩色流）", cam_id, cur_sence_id);
                        }
                    }

                    // 检查是否需要订阅深度图像话题
                    bool enable_depth = cam_info.sence_para[cur_sence_id].enable_depth_stream;
                    if (enable_depth)
                    {
                        // 检查是否已经订阅深度流
                        if (image_subscriber_map_[cam_id].find(CamMgr::CamStreamType::STREAM_DEPTH) == image_subscriber_map_[cam_id].end())
                        {
                            // 获取深度图像话题名称
                            auto topic_start = std::chrono::steady_clock::now();
                            std::string depth_topic = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_DEPTH);
                            auto topic_end = std::chrono::steady_clock::now();
                            LOG_DEBUG("[性能] 相机 %d 获取深度话题耗时：%ld ms", 
                                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(topic_end - topic_start).count());
                        
                            // 创建深度图像订阅器
                            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
                            qos_profile.depth = 10;
                            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
                                                    
                            auto sub_start = std::chrono::steady_clock::now();
                            auto depth_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
                                depth_topic, qos,
                                [this, cam_id](const sensor_msgs::msg::Image::SharedPtr msg)
                                {
                                    this->image_callback_with_stream_type(msg, cam_id, CamMgr::CamStreamType::STREAM_DEPTH);
                                });
                            auto sub_end = std::chrono::steady_clock::now();
                            LOG_DEBUG("[性能] 相机 %d 创建深度订阅器耗时：%ld ms", 
                                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(sub_end - sub_start).count());
                        
                            // 将订阅器添加到映射表中
                            image_subscriber_map_[cam_id][CamMgr::CamStreamType::STREAM_DEPTH] = depth_subscriber;
                        
                            // 设置深度流启动状态为 true
                            cam_run_info_[cam_id].is_depth_stream_start = true;
                        
                            LOG_INFO("相机 %d 深度图像话题订阅器已创建：%s", cam_id, depth_topic.c_str());
                        }
                    }
                    else
                    {
                        // 深度流未启用，取消订阅
                        if (image_subscriber_map_[cam_id].find(CamMgr::CamStreamType::STREAM_DEPTH) != image_subscriber_map_[cam_id].end())
                        {
                            image_subscriber_map_[cam_id].erase(CamMgr::CamStreamType::STREAM_DEPTH);
                            
                            // 设置深度流启动状态为false
                            cam_run_info_[cam_id].is_depth_stream_start = false;
                            
                            LOG_INFO("相机 %d 深度图像话题订阅器已取消（场景 %d 未启用深度流）", cam_id, cur_sence_id);
                        }
                    }
                    
                    // 检查是否需要订阅点云话题
                    bool enable_cloud = cam_info.sence_para[cur_sence_id].enable_cloud_stream;
                    
                    if (enable_cloud)
                    {
                        // 检查是否已经订阅点云
                        if (pointcloud_subscriber_map_.find(cam_id) == pointcloud_subscriber_map_.end())
                        {
                            // 获取点云话题名称
                            auto topic_start = std::chrono::steady_clock::now();
                            std::string cloud_topic = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_CLOUD);
                            auto topic_end = std::chrono::steady_clock::now();
                            LOG_DEBUG("[性能] 相机 %d 获取点云话题耗时：%ld ms", 
                                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(topic_end - topic_start).count());
                                                        
                            if (!cloud_topic.empty())
                            {
                                // 创建点云订阅器，使用高可靠性和最佳努力的 QoS 配置
                                rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
                                qos_profile.depth = 10;
                                auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
                                                            
                                auto sub_start = std::chrono::steady_clock::now();
                                auto pointcloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                    cloud_topic, qos,
                                    [this, cam_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                                    {
                                        this->pointcloud_callback(msg, cam_id);
                                    });
                                auto sub_end = std::chrono::steady_clock::now();
                                LOG_DEBUG("[性能] 相机 %d 创建点云订阅器耗时：%ld ms", 
                                         cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(sub_end - sub_start).count());
                                                            
                                // 将订阅器添加到映射表中
                                pointcloud_subscriber_map_[cam_id] = pointcloud_subscriber;
                                                            
                                // 设置点云流启动状态为 true
                                cam_run_info_[cam_id].is_cloud_stream_start = true;
                                                            
                                LOG_INFO("相机 %d 点云话题订阅器已创建：%s", cam_id, cloud_topic.c_str());
                            }
                        }
                    }
                    else
                    {
                        // 点云未启用，取消订阅
                        if (pointcloud_subscriber_map_.find(cam_id) != pointcloud_subscriber_map_.end())
                        {
                            pointcloud_subscriber_map_.erase(cam_id);
                            
                            // 设置点云流启动状态为false
                            cam_run_info_[cam_id].is_cloud_stream_start = false;
                            
                            LOG_INFO("相机 %d 点云话题订阅器已取消（场景 %d 未启用点云）", cam_id, cur_sence_id);
                        }
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("订阅相机图像话题时发生错误：%s", e.what());
        }
            
        // 性能日志：记录总耗时
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        LOG_INFO("[性能] 订阅相机图像和点云话题完成，总耗时：%ld ms", duration_ms);
    }

    void CamMgrRos::subscribe_to_all_camera_info()
    {
        // 性能日志：记录开始时间
        auto start_time = std::chrono::steady_clock::now();
        LOG_INFO("[性能] 开始订阅所有相机内参话题...");
        
        try
        {
            // 遍历所有已打开的相机
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                bool is_running = pair.second.is_cam_open;

                // 只为正在运行的相机订阅内参话题
                if (is_running)
                {
                    // 检查是否已经订阅
                    if (camera_info_subscriber_map_.find(cam_id) != camera_info_subscriber_map_.end())
                    {
                        LOG_INFO("相机 %d 内参话题已订阅，跳过", cam_id);
                        continue;
                    }

                    // 获取相机配置
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_ERROR("无法获取相机 %d 配置", cam_id);
                        continue;
                    }

                    // 检查相机是否启用
                    if (!cam_info.enable)
                    {
                        LOG_INFO("相机 %d 未启用，跳过内参订阅", cam_id);
                        continue;
                    }

                    // 构造内参话题名称：优先通过相机实例委托，失败则回退到默认命名
                    std::string camera_info_topic;
                    auto inst_it = camera_map_.find(cam_id);
                    if (inst_it != camera_map_.end() && inst_it->second)
                    {
                        camera_info_topic = inst_it->second->get_camera_info_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR, cam_info);
                    }
                    if (camera_info_topic.empty())
                    {
                        // 回退默认命名
                        camera_info_topic = "/camera/" + std::to_string(cam_id) + "/color/camera_info";
                    }

                    // 订阅相机内参话题
                    if (subscribe_to_camera_info(cam_id, camera_info_topic))
                    {
                        LOG_INFO("相机 %d 内参话题订阅成功: %s", cam_id, camera_info_topic.c_str());
                    }
                    else
                    {
                        LOG_ERROR("相机 %d 内参话题订阅失败: %s", cam_id, camera_info_topic.c_str());
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("订阅所有相机内参话题时发生错误：%s", e.what());
        }
            
        // 性能日志：记录总耗时
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        LOG_INFO("[性能] 订阅所有相机内参话题完成，总耗时：%ld ms", duration_ms);
    }
    void CamMgrRos::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, int cam_id)
    {
        try
        {
            // 性能日志：记录回调开始时间
            auto callback_start = std::chrono::steady_clock::now();
            
            // 更新内部的相机内参数据
            camera_info_msg_map_[cam_id] = *msg;
            intrinsics_received_map_[cam_id] = true;
            
            //////////////////////////////内参转发/////////////////////////////////
            // 只有在启用内参转发功能时才进行转发
            if (enable_camera_info_forward_)
            {
                // 转发内参信息（只转发一次）
                // 检查是否已有转发发布器，如果没有则创建
                if (camera_info_publisher_map_.find(cam_id) == camera_info_publisher_map_.end())
                {
                    std::string topic_name = "/cam_" + std::to_string(cam_id) + "/intrinsics";
                    camera_info_publisher_map_[cam_id] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                        topic_name, 10);
                    LOG_INFO("创建内参转发发布器，话题: %s", topic_name.c_str());
                }

                // 只有在尚未转发过的情况下才转发内参
                // 注意：在场景切换后，我们需要重新转发内参
                // 检查是否需要重新转发（场景切换后）
                bool need_retransmit = !has_forwarded_map_[cam_id] || !intrinsics_received_map_[cam_id];

                if (need_retransmit)
                {
                    // 更新时间戳并转发
                    sensor_msgs::msg::CameraInfo forward_msg = *msg;
                    forward_msg.header.stamp = this->now();
                    forward_msg.header.frame_id = "camera_" + std::to_string(cam_id) + "_frame";

                    // 发布转发的内参信息
                    auto forward_start = std::chrono::steady_clock::now();
                    camera_info_publisher_map_[cam_id]->publish(forward_msg);
                    auto forward_end = std::chrono::steady_clock::now();
                    LOG_INFO("成功转发相机 %d 的内参信息", cam_id);
                    LOG_DEBUG("[性能] 相机 %d 转发内参耗时：%ld ms", 
                             cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(forward_end - forward_start).count());
                    // 标记为已转发
                    has_forwarded_map_[cam_id] = true;

                    // 重新标记内参已接收
                    intrinsics_received_map_[cam_id] = true;
                }
            }

            //////////////////////////////内参设置到管理类/////////////////////////////////
            // 增加该相机的消息计数
            if (camera_image_count_map_[cam_id] <= 5)
            {
                camera_image_count_map_[cam_id]++;
            }

            // 在第5次接收到消息时处理内参（避免前几次可能不稳定的内参）
            // 或者在场景切换后重新处理内参
            bool need_update_config = (camera_image_count_map_[cam_id] == 5) || !intrinsics_received_map_[cam_id];

            if (need_update_config)
            {
                // 直接保存forward_msg的内参数据到JSON文件
                save_camera_intrinsics_to_json(cam_id, *msg);

                // 打印内参信息
                LOG_INFO("相机 %d 接收到内参:", cam_id);
                LOG_INFO("  分辨率: %dx%d", msg->width, msg->height);
                LOG_INFO("  内参矩阵 K: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
                         msg->k[0], msg->k[1], msg->k[2],
                         msg->k[3], msg->k[4], msg->k[5],
                         msg->k[6], msg->k[7], msg->k[8]);
                LOG_INFO("  畸变系数: [%f, %f, %f, %f, %f]",
                         msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);

                // 将内参保存到camera_configs_中对应的相机配置
                CamMgr::CamInfo cam_info;
                if (config_mgr_.get_camera_config(cam_id, cam_info))
                {
                    // 更新相机内参（使用默认场景，场景ID为0）
                    if (cam_info.sence_para.find(0) != cam_info.sence_para.end())
                    {
                        CamMgr::CamSencePara &sence_para = cam_info.sence_para.at(0);
                        sence_para.color_intr.width = msg->width;
                        sence_para.color_intr.height = msg->height;
                        sence_para.color_intr.fx = msg->k[0]; // fx
                        sence_para.color_intr.fy = msg->k[4]; // fy
                        sence_para.color_intr.cx = msg->k[2]; // cx
                        sence_para.color_intr.cy = msg->k[5]; // cy

                        // 复制畸变系数（最多5个）
                        for (int i = 0; i < 5 && i < static_cast<int>(msg->d.size()); i++)
                        {
                            sence_para.color_intr.dist_coeffs[i] = msg->d[i];
                        }

                        // 更新配置管理器中的相机配置
                        if (config_mgr_.set_camera_config(cam_id, cam_info))
                        {
                            LOG_INFO("相机 %d 配置已保存到配置管理器", cam_id);

                            // 保存单个场景信息
                            if (config_mgr_.save_sence_info(cam_id, 0))
                            {
                                LOG_INFO("相机 %d 场景0内参已保存到文件", cam_id);
                            }
                            else
                            {
                                LOG_ERROR("无法将相机 %d 场景0内参保存到文件", cam_id);
                            }
                        }
                        else
                        {
                            LOG_ERROR("无法将相机 %d 内参保存到配置管理器", cam_id);
                        }
                    }
                    else
                    {
                        LOG_ERROR("相机 %d 没有默认场景参数", cam_id);
                    }
                }
                else
                {
                    LOG_ERROR("无法获取相机 %d 的配置信息", cam_id);
                }

                // 重新标记内参已接收
                intrinsics_received_map_[cam_id] = true;
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("相机 %d 内参回调处理失败: %s", cam_id, e.what());
        }
    }

    bool CamMgrRos::subscribe_to_camera_info(int cam_id, const std::string &camera_info_topic)
    {
        try
        {
            // 性能日志：记录开始时间
            auto start_time = std::chrono::steady_clock::now();
                
            // 创建相机内参订阅器
            auto sub_start = std::chrono::steady_clock::now();
            auto camera_info_subscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic, 10,
                [this, cam_id](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                {
                    this->camera_info_callback(msg, cam_id);
                });
            auto sub_end = std::chrono::steady_clock::now();
            LOG_DEBUG("[性能] 相机 %d 创建内参订阅器耗时：%ld ms", 
                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(sub_end - sub_start).count());
    
            // 将订阅器添加到映射表中
            camera_info_subscriber_map_[cam_id] = camera_info_subscriber;
    
            // 初始化内参接收标志
            intrinsics_received_map_[cam_id] = false;
    
            // 创建相机内参发布器（用于转发内参）
            auto pub_start = std::chrono::steady_clock::now();
            std::string publisher_topic = "/cam_" + std::to_string(cam_id) + "/intrinsics";
            camera_info_publisher_map_[cam_id] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                publisher_topic, 10);
            auto pub_end = std::chrono::steady_clock::now();
            LOG_DEBUG("[性能] 相机 %d 创建内参发布器耗时：%ld ms", 
                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(pub_end - pub_start).count());
            LOG_INFO("创建相机 %d 内参发布器，话题：%s", cam_id, publisher_topic.c_str());
            LOG_INFO("相机 %d 内参订阅器已创建：%s", cam_id, camera_info_topic.c_str());
                
            auto end_time = std::chrono::steady_clock::now();
            LOG_DEBUG("[性能] 相机 %d 订阅内参总耗时：%ld ms", 
                     cam_id, std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("创建相机 %d 内参订阅器失败：%s", cam_id, e.what());
        }
        return false;
    }

    bool CamMgrRos::get_cam_image_topics()
    {
        try
        {
            LOG_INFO("开始获取并保存所有相机图像话题名");

            // 遍历所有已知的相机，获取它们的图像话题名
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                if (pair.second.is_cam_open) // 相机正在运行
                {
                    // 检查相机是否启用
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                        continue;
                    }

                    // 如果相机未启用，跳过
                    if (!cam_info.enable)
                    {
                        LOG_INFO("相机 %d 未启用，跳过图像话题获取", cam_id);
                        continue;
                    }

                    // 获取相机的图像话题名并保存到映射表中
                    std::string topic_name_color = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                    cam_run_info_[cam_id].cam_color_img_topic_name = topic_name_color;

                    LOG_INFO("相机 %d 的彩色图像话题名: %s", cam_id, topic_name_color.c_str());

                    // 获取相机的图像话题名并保存到映射表中
                    std::string topic_name_depth = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_DEPTH);
                    cam_run_info_[cam_id].cam_depth_img_topic_name = topic_name_depth;

                    LOG_INFO("相机 %d 的深度图像话题名: %s", cam_id, topic_name_depth.c_str());

                    std::string topic_name_cloud = get_camera_image_topic(cam_id, CamMgr::CamStreamType::STREAM_CLOUD);
                    cam_run_info_[cam_id].cam_point_cloud_topic_name = topic_name_cloud;

                    LOG_INFO("相机 %d 的点云图像话题名: %s", cam_id, topic_name_cloud.c_str());

                    basros::RosCommInfo comm_info;

                    // // 设置图像话题名称
                    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE, cam_id, 0);
                    std::string key_color = comm_info.name;
                    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE, cam_id, 0);
                    std::string key_depth = comm_info.name;
                    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_POINT_CLOUD, cam_id, 0);
                    std::string key_cloud = comm_info.name;
                    // 等待参数服务可用
                    if (!sys_config_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        LOG_ERROR("无法连接到系统配置参数服务");
                    }
                    else
                    {
                        std::vector<rclcpp::Parameter> parameters;
                        parameters.push_back(rclcpp::Parameter(key_color, topic_name_color));
                        parameters.push_back(rclcpp::Parameter(key_depth, topic_name_depth));
                        parameters.push_back(rclcpp::Parameter(key_cloud, topic_name_cloud));
                        auto results = sys_config_client_->set_parameters(parameters);
                    }
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("获取并保存图像话题名时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::get_cam_info_topics()
    {
        try
        {
            LOG_INFO("开始获取并保存所有相机内参话题名");

            // 遍历所有已知的相机，获取它们的内参话题名
            for (const auto &pair : cam_run_info_)
            {
                int cam_id = pair.first;
                if (pair.second.is_cam_open) // 相机正在运行
                {
                    // 检查相机是否启用
                    CamMgr::CamInfo cam_info;
                    if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        LOG_WARN("无法获取相机 %d 的配置信息", cam_id);
                        continue;
                    }

                    // 如果相机未启用，跳过
                    if (!cam_info.enable)
                    {
                        LOG_INFO("相机 %d 未启用，跳过内参话题获取", cam_id);
                        continue;
                    }

                    // 获取相机的内参话题名并保存到映射表中
                    std::string topic_name_intr = get_camera_info_topic(cam_id, CamMgr::CamStreamType::STREAM_COLOR);
                    cam_run_info_[cam_id].cam_color_intr_topic_name = topic_name_intr;

                    LOG_INFO("相机 %d 的内参话题名: %s", cam_id, topic_name_intr.c_str());

                    basros::RosCommInfo comm_info;
                    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS, cam_id, 0);
                    std::string key_intr = comm_info.name;
                    // 等待参数服务可用
                    if (!sys_config_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        LOG_ERROR("无法连接到系统配置参数服务");
                        // return false;
                    }
                    else
                    {
                        std::vector<rclcpp::Parameter> parameters;
                        parameters.push_back(rclcpp::Parameter(key_intr, topic_name_intr));
                        auto results = sys_config_client_->set_parameters(parameters);
                    }
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("获取并保存内参话题名时发生错误: %s", e.what());
            return false;
        }
    }
    bool CamMgrRos::is_intrinsics_received(int cam_id) const
    {
        auto it = intrinsics_received_map_.find(cam_id);
        if (it != intrinsics_received_map_.end())
        {
            return it->second;
        }
        return false;
    }

    bool CamMgrRos::save_all_cameras_info()
    {
        try
        {
            // 构建保存路径
            std::string file_path = config_mgr_.cam_cfg_dir_ + "all_cameras_info.yaml";

            // 创建YAML节点
            YAML::Node cameras_node;

            // 遍历所有相机设备信息
            for (size_t i = 0; i < cam_dev_list_.size(); ++i)
            {
                const CamMgr::CamDevInfo &dev_info = cam_dev_list_[i];

                // 创建相机节点
                YAML::Node camera_node;
                camera_node["index"] = i;
                camera_node["cam_type"] = static_cast<int>(dev_info.cam_type);
                camera_node["device_id"] = dev_info.device_id;
                camera_node["product_id"] = dev_info.product_id;
                camera_node["device_name"] = dev_info.device_name;
                camera_node["serial_number"] = dev_info.serial_number;
                camera_node["physical_port"] = dev_info.physical_port;
                camera_node["user_name"] = dev_info.user_name;
                camera_node["facturer_name"] = dev_info.facturer_name;
                camera_node["firmware_version"] = dev_info.firmware_version;

                cameras_node["cameras"].push_back(camera_node);
            }

            // 保存到文件
            std::ofstream fout(file_path);
            fout << cameras_node;
            fout.close();

            LOG_INFO("成功保存所有相机信息到: %s", file_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR("YAML保存错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("保存所有相机信息时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamMgrRos::save_single_camera_info(int cam_id)
    {
        try
        {
            // 检查相机配置是否存在
            CamMgr::CamInfo cam_info;
            if (!config_mgr_.get_camera_config(cam_id, cam_info))
            {
                LOG_ERROR("找不到相机ID %d 的配置", cam_id);
                return false;
            }

            // 根据序列号在cam_dev_list_中查找对应的设备信息
            const CamMgr::CamDevInfo *dev_info_ptr = nullptr;
            for (const auto &dev_info : cam_dev_list_)
            {
                if (dev_info.serial_number == cam_info.serial_number)
                {
                    dev_info_ptr = &dev_info;
                    break;
                }
            }

            if (dev_info_ptr == nullptr)
            {
                LOG_WARN("未找到相机ID %d 序列号 %s 对应的设备信息", cam_id, cam_info.serial_number.c_str());
                return false;
            }

            // 构建保存路径
            std::string camera_folder_path = config_mgr_.cam_cfg_dir_ + "cam_" + std::to_string(cam_id);
            // 确保文件夹存在
            if (!std::filesystem::exists(camera_folder_path))
            {
                std::filesystem::create_directory(camera_folder_path);
            }

            std::string file_path = camera_folder_path + "/cam_" + std::to_string(cam_id) + "_device_info.yaml";

            // 创建YAML节点
            YAML::Node cam_info_node;

            // 保存相机配置信息
            cam_info_node["cam_id"] = cam_id;
            cam_info_node["cam_usr_name"] = cam_info.cam_usr_name;
            cam_info_node["serial_number"] = cam_info.serial_number;
            cam_info_node["cam_type"] = static_cast<int>(cam_info.cam_type);
            cam_info_node["cam_model"] = cam_info.cam_model;
            cam_info_node["cam_index"] = cam_info.cam_index;
            cam_info_node["enable"] = cam_info.enable;
            cam_info_node["show_topic_image"] = cam_info.show_topic_image;

            // 保存设备信息
            cam_info_node["device_info"]["device_id"] = dev_info_ptr->device_id;
            cam_info_node["device_info"]["product_id"] = dev_info_ptr->product_id;
            cam_info_node["device_info"]["device_name"] = dev_info_ptr->device_name;
            cam_info_node["device_info"]["physical_port"] = dev_info_ptr->physical_port;
            cam_info_node["device_info"]["user_name"] = dev_info_ptr->user_name;
            cam_info_node["device_info"]["facturer_name"] = dev_info_ptr->facturer_name;
            cam_info_node["device_info"]["firmware_version"] = dev_info_ptr->firmware_version;

            // 保存话题名
            cam_info_node["topics"]["image_topic"] = cam_run_info_[cam_id].cam_color_img_topic_name;
            // cam_info_node["topics"]["camera_info_topic"] = camera_info_topic_name_map_[cam_id];

            // 保存ROI和FPS信息
            auto roi_fps_it = cam_roi_fps_map_.find(cam_id);
            if (roi_fps_it != cam_roi_fps_map_.end())
            {
                YAML::Node roi_fps_node;
                const auto &sensor_map = roi_fps_it->second;

                for (const auto &sensor_pair : sensor_map)
                {
                    const std::string &sensor_name = sensor_pair.first;
                    const CamMgr::CamRoiFpsList &roi_fps_list = sensor_pair.second;

                    YAML::Node sensor_node;
                    for (const auto &roi_fps : roi_fps_list)
                    {
                        YAML::Node roi_fps_entry;
                        std::string str_roi_fps = std::to_string(roi_fps.width) +
                                                  " X " + std::to_string(roi_fps.height) +
                                                  " X " + std::to_string(roi_fps.fps);
                        roi_fps_entry["roi_fps"] = str_roi_fps;
                        sensor_node.push_back(roi_fps_entry);
                    }

                    roi_fps_node[sensor_name] = sensor_node;
                }

                cam_info_node["roi_fps"] = roi_fps_node;
            }
            else
            {
                LOG_WARN("未找到相机ID %d 的ROI/FPS信息", cam_id);
            }

            // 保存到文件
            std::ofstream fout(file_path);
            fout << cam_info_node;
            fout.close();

            LOG_INFO("成功保存相机ID %d 信息到: %s", cam_id, file_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR("YAML保存错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("保存相机ID %d 信息时发生错误: %s", cam_id, e.what());
            return false;
        }
    }

    std::string CamMgrRos::get_camera_image_topic(int cam_id, CamMgr::CamStreamType stream_type)
    {
        // 委托给相机实例（若存在），否则使用基类默认实现或返回空
        CamMgr::CamInfo cam_info;
        if (!config_mgr_.get_camera_config(cam_id, cam_info))
        {
            return std::string();
        }

        auto it = camera_map_.find(cam_id);
        if (it != camera_map_.end() && it->second)
        {
            std::string topic = it->second->get_camera_image_topic(cam_id, stream_type, cam_info);
            // 缓存到状态
            if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
                cam_run_info_[cam_id].cam_color_img_topic_name = topic;
            else if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
                cam_run_info_[cam_id].cam_depth_img_topic_name = topic;
            return topic;
        }

        // 无实例可委托时返回空字符串（调用方需处理回退策略）
        return std::string();
    }

    std::string CamMgrRos::get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type)
    {
        // 获取配置并委托给相机子类实现（若存在），否则使用基于配置的默认命名规则
        CamMgr::CamInfo cam_info;
        if (!config_mgr_.get_camera_config(cam_id, cam_info))
        {
            return "";
        }

        // 若有对应的相机实例，则通过基类指针委托实现
        auto it = camera_map_.find(cam_id);
        if (it != camera_map_.end() && it->second)
        {
            std::string topic = it->second->get_camera_info_topic(cam_id, stream_type, cam_info);
            // 同时更新状态中的话题名缓存
            if (stream_type == CamMgr::CamStreamType::STREAM_COLOR)
                cam_run_info_[cam_id].cam_color_intr_topic_name = topic;
            else if (stream_type == CamMgr::CamStreamType::STREAM_DEPTH)
                cam_run_info_[cam_id].cam_depth_intr_topic_name = topic;
            return topic;
        }
        // 没有实例可委托时返回空字符串（调用方需处理回退策略）
        return std::string();
    }
}

namespace cam_mgr_ros
{
    // 启动后台监视与重试线程
    void CamMgrRos::start_background_threads()
    {
        bool expected = false;
        if (!threads_running_.compare_exchange_strong(expected, true))
        {
            // 已经在运行
            return;
        }

        // 启动线程
        LOG_INFO("启动后台监视与重试线程");
        monitor_thread_ = std::thread(&CamMgrRos::monitor_opened_cameras_loop, this);
        retry_thread_ = std::thread(&CamMgrRos::retry_open_failed_cameras_loop, this);
        
        // 设置较低的优先级（nice 值越大优先级越低）
        cam_mgr_ros::set_thread_priority(monitor_thread_, MONITOR_THREAD_NICE);
        cam_mgr_ros::set_thread_priority(retry_thread_, RETRY_THREAD_NICE);
                
        LOG_INFO("后台线程已创建：monitor_thread %s, retry_thread %s", monitor_thread_.joinable() ? "joinable" : "not-joinable", retry_thread_.joinable() ? "joinable" : "not-joinable");
    }

    // 停止后台线程
    void CamMgrRos::stop_background_threads()
    {
        bool expected = true;
        if (!threads_running_.compare_exchange_strong(expected, false))
        {
            // 线程未运行
            return;
        }

        // 通知线程退出并等待
        LOG_INFO("停止后台线程，等待 join");
        if (monitor_thread_.joinable())
            monitor_thread_.join();
        if (retry_thread_.joinable())
            retry_thread_.join();
        LOG_INFO("后台线程已停止");
    }

    // 初始化保存线程池
    void CamMgrRos::init_camera_image_save_threads()
    {
        stop_image_save_threads_ = false;
        // 不再预先创建线程，改为按需创建
        LOG_INFO("图像保存线程已初始化（按需启动，每个相机独立线程）");
    }
    
    // 单个相机的图像保存线程函数（同时处理图像和点云）
    void CamMgrRos::camera_image_save_thread_func(int cam_id)
    {
        while (true)
        {
            ImageSaveTask task;
            {
                std::unique_lock<std::mutex> lock(image_save_queue_mutex_[cam_id]);
                image_save_queue_cv_[cam_id].wait(lock, [this, cam_id]() {
                    return !image_save_queues_[cam_id].empty() || !image_save_thread_running_[cam_id];
                });
                    
                if (!image_save_thread_running_[cam_id] && image_save_queues_[cam_id].empty())
                {
                    break;
                }
                    
                task = image_save_queues_[cam_id].front();
                image_save_queues_[cam_id].pop();
            }
                
            // 根据流类型执行不同的保存操作
            if (task.stream_type == CamMgr::CamStreamType::STREAM_CLOUD)
            {
                // 点云保存
                auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(task.cloud_msg);
                pointcloud_save_callback(msg_ptr, task.cam_id);
            }
            else
            {
                // 图像保存
                auto msg_ptr = std::make_shared<sensor_msgs::msg::Image>(task.image_msg);
                image_save_callback_with_stream_type(msg_ptr, task.cam_id, task.stream_type);
            }
        }
            
        // 线程退出前清理资源
        {
            std::lock_guard<std::mutex> lock(image_save_queue_mutex_[cam_id]);
            image_save_thread_running_[cam_id] = false;
        }
        LOG_INFO("相机 %d 的图像保存线程已停止", cam_id);
    }
        
    // 启动指定相机的保存线程
    void CamMgrRos::start_camera_save_thread(int cam_id)
    {
        std::lock_guard<std::mutex> lock(image_save_queue_mutex_[cam_id]);
            
        // 如果线程已经在运行，不需要重复启动
        if (image_save_thread_running_[cam_id])
        {
            return;
        }
            
        // 创建新线程
        image_save_threads_[cam_id] = std::thread(&CamMgrRos::camera_image_save_thread_func, this, cam_id);
        image_save_thread_running_[cam_id] = true;
        
        // 设置较低的优先级（nice 值越大优先级越低）
        cam_mgr_ros::set_thread_priority(image_save_threads_[cam_id], SAVE_THREAD_NICE);
        
        LOG_INFO("相机 %d 的图像保存线程已启动", cam_id);
    }
        
    // 停止指定相机的保存线程
    void CamMgrRos::stop_camera_save_thread(int cam_id)
    {
        // 先标记该相机的保存线程需要停止
        {
            std::lock_guard<std::mutex> lock(image_save_queue_mutex_[cam_id]);
            image_save_thread_running_[cam_id] = false;
        }
        
        // 唤醒线程并等待其结束
        image_save_queue_cv_[cam_id].notify_all();
            
        if (image_save_threads_[cam_id].joinable())
        {
            image_save_threads_[cam_id].join();
        }
            
        LOG_INFO("相机 %d 的图像保存线程已停止（请求）", cam_id);
    }

    // 初始化显示线程池
    void CamMgrRos::init_display_threads(int num_threads)
    {
        stop_display_threads_ = false;
        display_threads_.reserve(num_threads);
        for (int i = 0; i < num_threads; ++i)
        {
            display_threads_.emplace_back(&CamMgrRos::display_thread_func, this);
            // 显示线程保持默认优先级（nice=0），以保证 UI 响应速度
            // set_thread_priority(display_threads_.back(), DISPLAY_THREAD_NICE);  // DISPLAY_THREAD_NICE=0，无需设置
        }
        LOG_INFO("图像显示线程池已初始化，线程数：%d", num_threads);
    }

    // 显示线程函数
    void CamMgrRos::display_thread_func()
    {
        while (!stop_display_threads_)
        {
            DisplayTask task;
            {
                std::unique_lock<std::mutex> lock(display_task_mutex_);
                display_task_cv_.wait(lock, [this]() {
                    return !display_tasks_.empty() || stop_display_threads_;
                });
                
                if (stop_display_threads_ && display_tasks_.empty())
                {
                    break;
                }
                
                task = display_tasks_.front();
                display_tasks_.pop();
            }
            
            // 执行显示操作，将值类型转换为SharedPtr
            auto msg_ptr = std::make_shared<sensor_msgs::msg::Image>(task.msg);
            image_display_callback_with_stream_type(msg_ptr, task.cam_id, task.stream_type);
        }
    }

    // 初始化压缩线程池
    void CamMgrRos::init_compress_threads(int num_threads)
    {
        stop_compress_threads_ = false;
        compress_threads_.reserve(num_threads);
        for (int i = 0; i < num_threads; ++i)
        {
            compress_threads_.emplace_back(&CamMgrRos::compress_thread_func, this);
        }
        LOG_INFO("文件夹压缩线程池已初始化，线程数：%d", num_threads);
    }
    
    // 压缩线程函数
    void CamMgrRos::compress_thread_func()
    {
        while (!stop_compress_threads_)
        {
            CompressTask task;
            {
                std::unique_lock<std::mutex> lock(compress_task_mutex_);
                compress_task_cv_.wait(lock, [this]() {
                    return !compress_tasks_.empty() || stop_compress_threads_;
                });
                
                if (stop_compress_threads_ && compress_tasks_.empty())
                {
                    break;
                }
                
                task = compress_tasks_.front();
                compress_tasks_.pop();
            }
            
            bool success = cam_mgr_ros::compress_directory(task.source_dir, task.output_zip);
            
            if (success)
            {
                LOG_INFO("文件夹压缩成功：%s", task.output_zip.c_str());
            }
            else
            {
                LOG_ERROR("文件夹压缩失败：%s -> %s", task.source_dir.c_str(), task.output_zip.c_str());
            }
        }
    }
    
    // 提交压缩任务到线程池
    void CamMgrRos::submit_compress_task(const std::string& source_dir, const std::string& output_zip)
    {
        {
            std::lock_guard<std::mutex> lock(compress_task_mutex_);
            CompressTask task;
            task.source_dir = source_dir;
            task.output_zip = output_zip;
            compress_tasks_.push(task);
        }
        compress_task_cv_.notify_one();
    }

    // 监视已打开相机的循环实现：检测相机进程是否还存活，若不存活则执行关闭清理
    void CamMgrRos::monitor_opened_cameras_loop()
    {
        LOG_INFO("monitor_opened_cameras_loop 启动");
        while (threads_running_.load())
        {
            std::vector<int> to_close;
            {
                std::lock_guard<std::mutex> lock(camera_state_mutex_);
                
                // 如果有相机正在切换场景，跳过所有相机的监控
                if (is_switching_scene.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                
                for (auto &pair : cam_run_info_)
                {
                    int cam_id = pair.first;
                    CamMgr::CamRunInfo &state = pair.second;
                    if (!state.is_cam_open)
                        continue;

                    pid_t pid = state.cam_pid;
                    if (pid <= 0)
                        continue;

                    // 如果相机处于启动中状态（如切换场景），跳过所有掉线检测
                    if (state.cam_run_state == CamMgr::CamRunState::CAM_RUN_STATE_STARTING) {
                        continue;
                    }
                    
                    // 如果相机正在切换场景（模式 1：先关再开），跳过掉线检测
                    if (state.is_switching_scene) {
                        LOG_DEBUG("相机 %d 正在切换场景，跳过掉线检测", cam_id);
                        continue;
                    }

                    // 更严格地检查进程：不存在或为僵尸均视为需要清理
                    if (!cam_mgr_ros::is_pid_alive_and_not_zombie(pid))
                    {
                        LOG_WARN("监测到相机 %d 进程 %d 已终止或为僵尸，稍后执行清理", cam_id, pid);
                        // 标记为掉线和异常状态
                        state.is_offline = true;
                        state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_ERROR;
                        to_close.push_back(cam_id);
                    }
                    
                    // 检查消息接收超时
                    int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    const int64_t MESSAGE_TIMEOUT_MS = 10000; // 10 秒超时（考虑场景切换时的系统资源占用）
                    
                    // 检查是否配置了对应的数据流
                    CamMgr::CamInfo cam_info;
                    bool has_timeout = false;
                    if (config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        int cur_sence_id = state.cur_sence_id;
                        auto sence_it = cam_info.sence_para.find(cur_sence_id);
                        if (sence_it != cam_info.sence_para.end())
                        {
                            const CamMgr::CamSencePara &sence_para = sence_it->second;
                            
                            // 检查该相机当前是否正在保存图像
                            bool is_cam_saving = false;
                            {
                                std::lock_guard<std::mutex> save_lock(image_save_mutex_);
                                auto cam_it = image_save_config_.find(cam_id);
                                if (cam_it != image_save_config_.end())
                                {
                                    for (const auto& stream_pair : cam_it->second)
                                    {
                                        if (stream_pair.second)
                                        {
                                            is_cam_saving = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            
                            // 检查启动超时：如果相机已启动超过 1 分钟但仍然没有收到任何图像/点云数据
                            if (state.last_success_open_time > 0 && 
                                current_time - state.last_success_open_time > STARTUP_TIMEOUT_MS)
                            {
                                // 检查是否启用了任何数据流但没有收到数据
                                bool should_have_data = false;
                                if (sence_para.enable_color_stream || sence_para.enable_depth_stream || sence_para.enable_cloud_stream)
                                {
                                    should_have_data = true;
                                }
                                
                                // 如果应该有数据但一直没收到
                                if (should_have_data && state.last_color_msg_time == 0 && 
                                    state.last_depth_msg_time == 0 && state.last_cloud_msg_time == 0)
                                {
                                    LOG_ERROR("相机 %d 启动超时（>1 分钟），进程已启动但未收到任何图像/点云数据，判定为掉线", cam_id);
                                    has_timeout = true;
                                }
                            }
                            // 检查彩色流
                            if (sence_para.enable_color_stream && state.is_color_stream_start)
                            {
                                if (state.last_color_msg_time != 0 && current_time - state.last_color_msg_time > MESSAGE_TIMEOUT_MS)
                                {
                                    // 如果正在存图、切换场景或刚切换完场景（3 秒内），不判定为超时
                                    bool is_just_switched = (current_time - last_scene_switch_time_ms_) < 3000;
                                    if (!is_cam_saving && !is_switching_scene.load() && !is_just_switched)
                                    {
                                        LOG_WARN("相机 %d 彩色流消息超时，最后接收时间：%lld，当前时间：%lld", 
                                                 cam_id, state.last_color_msg_time, current_time);
                                        has_timeout = true;
                                    }
                                }
                            }
                            // 检查深度流
                            if (sence_para.enable_depth_stream && state.is_depth_stream_start)
                            {
                                if (state.last_depth_msg_time != 0 && current_time - state.last_depth_msg_time > MESSAGE_TIMEOUT_MS)
                                {
                                    // 如果正在存图、切换场景或刚切换完场景（3 秒内），不判定为超时
                                    bool is_just_switched = (current_time - last_scene_switch_time_ms_) < 3000;
                                    if (!is_cam_saving && !is_switching_scene.load() && !is_just_switched)
                                    {
                                        LOG_WARN("相机 %d 深度流消息超时，最后接收时间：%lld，当前时间：%lld", 
                                                 cam_id, state.last_depth_msg_time, current_time);
                                        has_timeout = true;
                                    }
                                }
                            }
                            // 检查点云流
                            if (sence_para.enable_cloud_stream && state.is_cloud_stream_start)
                            {
                                if (state.last_cloud_msg_time != 0 && current_time - state.last_cloud_msg_time > MESSAGE_TIMEOUT_MS)
                                {
                                    // 如果正在存图、切换场景或刚切换完场景（3 秒内），不判定为超时
                                    bool is_just_switched = (current_time - last_scene_switch_time_ms_) < 3000;
                                    if (!is_cam_saving && !is_switching_scene.load() && !is_just_switched)
                                    {
                                        LOG_WARN("相机 %d 点云流消息超时，最后接收时间：%lld，当前时间：%lld", 
                                                 cam_id, state.last_cloud_msg_time, current_time);
                                        has_timeout = true;
                                    }
                                }
                            }
                        }
                    }
                    
                    if (has_timeout)
                    {
                        LOG_WARN("相机 %d 消息接收超时，标记为掉线", cam_id);
                        state.is_offline = true;
                        state.cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_OFFLINE;
                        
                        // 记录掉线原因（已经在锁的作用域内）
                        CamMgr::OfflineRecord record(CamMgr::OfflineReason::OFFLINE_REASON_MESSAGE_TIMEOUT, current_time);
                        auto it = cam_run_info_.find(cam_id);
                        if (it != cam_run_info_.end())
                        {
                            it->second.offline_records.push_back(record);
                            if (it->second.offline_records.size() > 3)
                            {
                                it->second.offline_records.erase(it->second.offline_records.begin());
                            }
                        }
                        
                        to_close.push_back(cam_id);
                    }
                }
            }

            // 在解锁后执行关闭操作以避免死锁
            for (int cam_id : to_close)
            {
                close_cam(cam_id, true); // 掉线导致的关闭
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(monitor_interval_ms_.load()));
        }
    }

    // 重试打开失败或未打开相机的循环实现：尝试打开配置中启用但未打开的相机
    void CamMgrRos::retry_open_failed_cameras_loop()
    {
        LOG_INFO("retry_open_failed_cameras_loop 启动");
        while (threads_running_.load())
        {
            // 仅遍历 cam_run_info_，对标记为掉线（is_offline）的相机执行重试
            std::vector<int> to_retry;
            {
                std::lock_guard<std::mutex> lock(camera_state_mutex_);
                for (const auto &pair : cam_run_info_)
                {
                    int cam_id = pair.first;
                    const CamMgr::CamRunInfo &state = pair.second;
                    // 只有曾经标记为掉线且当前未打开的相机才重试
                    if (!state.is_cam_open && state.is_offline)
                    {
                        to_retry.push_back(cam_id);
                    }
                }
            }
            for (int cam_id : to_retry)
            {
                CamMgr::CamInfo cam_info;
                if (!config_mgr_.get_camera_config(cam_id, cam_info))
                    continue;
                if (!cam_info.enable)
                    continue;

                int sence_id = 0;
                {
                    std::lock_guard<std::mutex> lock(camera_state_mutex_);
                    auto it = cam_run_info_.find(cam_id);
                    if (it != cam_run_info_.end())
                        sence_id = it->second.cur_sence_id;
                }

                // 检查带宽是否足够
                bool need_reduce_fps = false;
                int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                
                // 检查最近的掉线记录
                std::vector<CamMgr::OfflineRecord> offline_records;
                {
                    std::lock_guard<std::mutex> lock(camera_state_mutex_);
                    auto it = cam_run_info_.find(cam_id);
                    if (it != cam_run_info_.end())
                    {
                        offline_records = it->second.offline_records;
                    }
                }
                
                // 检查是否是因为消息超时导致的重连，且时间间隔小于10秒
                int message_timeout_count = 0;
                int64_t last_offline_time = 0;
                bool has_bandwidth_record = false;
                int64_t last_success_open_time = 0;
                
                // 获取最后成功打开相机的时间
                { 
                    std::lock_guard<std::mutex> lock(camera_state_mutex_);
                    auto it = cam_run_info_.find(cam_id);
                    if (it != cam_run_info_.end())
                    {
                        last_success_open_time = it->second.last_success_open_time;
                    }
                }
                
                for (const auto &record : offline_records)
                {
                    if (record.reason == CamMgr::OfflineReason::OFFLINE_REASON_MESSAGE_TIMEOUT)
                    {
                        message_timeout_count++;
                        last_offline_time = record.timestamp;
                    }
                    else if (record.reason == CamMgr::OfflineReason::OFFLINE_REASON_BANDWIDTH_INSUFFICIENT)
                    {
                        has_bandwidth_record = true;
                    }
                }
                
                // 带宽不足判断逻辑
                // 1. 消息超时，且从上次成功打开到现在的时间间隔小于10秒
                // 2. 或者之前已经有带宽不足记录，且再次出现消息超时
                // 这样可以确保带宽不足能够被正确识别
                const int64_t RECONNECT_INTERVAL_THRESHOLD = 30000; // 30秒阈值
                
                // 打印时间信息用于调试
                if (last_success_open_time > 0)
                {
                    LOG_INFO("相机 %d 上次成功打开时间: %lld, 当前时间: %lld, 时间差: %lld毫秒", 
                             cam_id, last_success_open_time, current_time, current_time - last_success_open_time);
                }
                
                if (message_timeout_count >= 1 && last_success_open_time > 0 && (current_time - last_success_open_time) < RECONNECT_INTERVAL_THRESHOLD)
                {
                    LOG_WARN("相机 %d 因消息超时重连，时间间隔小于30秒，判断为带宽不足，需要降低帧率", cam_id);
                    need_reduce_fps = true;
                }
                
                // 准备相机配置
                std::optional<CamMgr::CamInfo> temp_cam_info;
                if (need_reduce_fps)
                {
                    CamMgr::CamInfo cam_info;
                    if (config_mgr_.get_camera_config(cam_id, cam_info))
                    {
                        auto sence_it = cam_info.sence_para.find(sence_id);
                        if (sence_it != cam_info.sence_para.end())
                        {
                            // 创建临时配置副本
                            temp_cam_info = cam_info;
                            CamMgr::CamSencePara &sence_para = temp_cam_info->sence_para[sence_id];
                            
                            // 降低彩色流帧率
                            if (sence_para.enable_color_stream)
                            {
                                int min_fps = 0;
                                if (find_min_supported_fps(cam_id, CamMgr::CamStreamType::STREAM_COLOR, 
                                                         sence_para.color_para.width, sence_para.color_para.height, min_fps))
                                {
                                    LOG_INFO("将相机 %d 彩色流帧率从 %d 降低到 %d", 
                                             cam_id, sence_para.color_para.fps, min_fps);
                                    sence_para.color_para.fps = min_fps;
                                }
                                else
                                {
                                    // 如果找不到最小适配fps，直接设置为5
                                    LOG_WARN("未找到相机 %d 彩色流的最小适配FPS，直接设置为5", cam_id);
                                    sence_para.color_para.fps = 5;
                                }
                            }
                            
                            // 降低深度流帧率
                            if (sence_para.enable_depth_stream)
                            {
                                int min_fps = 0;
                                if (find_min_supported_fps(cam_id, CamMgr::CamStreamType::STREAM_DEPTH, 
                                                         sence_para.depth_para.width, sence_para.depth_para.height, min_fps))
                                {
                                    LOG_INFO("将相机 %d 深度流帧率从 %d 降低到 %d", 
                                             cam_id, sence_para.depth_para.fps, min_fps);
                                    sence_para.depth_para.fps = min_fps;
                                }
                                else
                                {
                                    // 如果找不到最小适配fps，直接设置为5
                                    LOG_WARN("未找到相机 %d 深度流的最小适配FPS，直接设置为5", cam_id);
                                    sence_para.depth_para.fps = 5;
                                }
                            }
                            
                            // 记录带宽不足的掉线原因
                            CamMgr::OfflineRecord record(CamMgr::OfflineReason::OFFLINE_REASON_BANDWIDTH_INSUFFICIENT, current_time);
                            std::lock_guard<std::mutex> lock(camera_state_mutex_);
                            auto it = cam_run_info_.find(cam_id);
                            if (it != cam_run_info_.end())
                            {
                                it->second.offline_records.push_back(record);
                                if (it->second.offline_records.size() > 3)
                                {
                                    it->second.offline_records.erase(it->second.offline_records.begin());
                                }
                            }
                        }
                    }
                }

                LOG_INFO("重试打开相机 %d 场景 %d", cam_id, sence_id);
                // 等待3秒，确保相机设备完全释放
                std::this_thread::sleep_for(std::chrono::seconds(3));
                if (open_cam(cam_id, sence_id, temp_cam_info))
                {
                    LOG_INFO("重试成功打开相机 %d", cam_id);
                    try
                    {
                        std::lock_guard<std::mutex> lock(camera_state_mutex_);
                        save_single_camera_info(cam_id);
                        // 成功打开后清除掉线标记并设置为正常运行状态
                        cam_run_info_[cam_id].is_offline = false;
                        cam_run_info_[cam_id].cam_run_state = CamMgr::CamRunState::CAM_RUN_STATE_NORMAL;
                    }
                    catch (...)
                    {
                    }

                    // 订阅与发布操作不需要持有 camera_state_mutex_
                    subscribe_to_camera_images_clouds();
                    subscribe_to_all_camera_info();
                    publish_all_camera_info();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_.load()));
        }
    }
    
    void CamMgrRos::refreshModuleInfo()
    {
        std::lock_guard<std::mutex> lock(camera_state_mutex_);
            
        // 获取当前时间用于判断数据是否新鲜
        int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        const int64_t DATA_FRESH_MS = 5000; // 5 秒内收到数据视为新鲜
            
        // 遍历所有已打开的相机，刷新 ModuleStatusInfo 并发布
        for (const auto& [cam_id, cam_run_info] : cam_run_info_) {
            // 检查 module_info_map_中是否已有该相机，没有则初始化
            if (module_info_map_.find(cam_id) == module_info_map_.end()) {
                initModuleStatusInfo(cam_id);
            }
            
            auto& module_info = module_info_map_[cam_id];
                
            // 根据该相机的 run_info 确定状态
            if (cam_run_info.is_offline) {
                // 相机掉线
                module_info.status = basros::ModuleStatus::ERROR;
                module_info.status_msg = "相机掉线";
            }
            else if (cam_run_info.is_cam_open)
            {
                // 相机已启动，检查是否收到新鲜数据
                bool has_fresh_data = false;
                
                // 检查是否有任何流类型收到了新鲜数据
                if (cam_run_info.last_color_msg_time > 0 && 
                    current_time - cam_run_info.last_color_msg_time < DATA_FRESH_MS) {
                    has_fresh_data = true;
                }
                else if (cam_run_info.last_depth_msg_time > 0 && 
                         current_time - cam_run_info.last_depth_msg_time < DATA_FRESH_MS) {
                    has_fresh_data = true;
                }
                else if (cam_run_info.last_cloud_msg_time > 0 && 
                         current_time - cam_run_info.last_cloud_msg_time < DATA_FRESH_MS) {
                    has_fresh_data = true;
                }
                
                if (has_fresh_data) {
                    // 相机在运行且收到新鲜数据
                    module_info.status = basros::ModuleStatus::RUNNING;
                    module_info.status_msg = "";
                } else {
                    // 相机已启动但未收到数据（可能是启动中或异常）
                    module_info.status = basros::ModuleStatus::STARTING;
                    module_info.status_msg = "相机启动中，等待数据...";
                }
            }
            else {
                // 没有该相机信息，模块处于停止状态
                module_info.status = basros::ModuleStatus::STOPPED;
                module_info.status_msg = "没有相机信息";
            }
                
            // 发布该相机的 ModuleInfo（发布前会检查并初始化发布器）
            publishModuleInfo(cam_id);
        }
            
        // 记录调试信息（可选）
        LOG_DEBUG("ModuleInfo 刷新完成：共 %zu 个相机", module_info_map_.size());
    }
    
    /**
     * @brief 初始化指定相机的 ModuleStatusInfo 并创建发布器
     * @param cam_id 相机 ID
     */
    void CamMgrRos::initModuleStatusInfo(int cam_id)
    {
        // 检查是否已初始化
        if (module_info_map_.find(cam_id) != module_info_map_.end()) {
            LOG_DEBUG("相机 %d 的 ModuleStatusInfo 已初始化", cam_id);
            return;
        }
        
        // 初始化 ModuleStatusInfo
        basros::ModuleStatusInfo status_info;
        status_info.module_name = "cam_mgr_ros";
        status_info.cam_id = cam_id;
        status_info.status = basros::ModuleStatus::RUNNING;
        status_info.status_msg = "相机正常运行";
        module_info_map_[cam_id] = status_info;
        
        // 创建发布器
        basros::RosCommInfo comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MODULE_INFO_CAM, cam_id, 0);
        std::string topic_name = comm_info.name;
        module_info_publisher_map_[cam_id] = this->create_publisher<std_msgs::msg::String>(
            topic_name, rclcpp::QoS(10).reliable());
        LOG_INFO("相机 %d 的 ModuleStatusInfo 发布器已创建：%s", cam_id, topic_name.c_str());
    }
    
    void CamMgrRos::publishModuleInfo(int cam_id)
    {
        auto it = module_info_map_.find(cam_id);
        if (it == module_info_map_.end()) {
            LOG_ERROR("未找到相机 %d 的 ModuleStatusInfo", cam_id);
            return;
        }
        
        // 检查发布器是否已初始化，如果没有则先初始化
        auto pub_it = module_info_publisher_map_.find(cam_id);
        if (pub_it == module_info_publisher_map_.end()) {
            // 发布器未初始化，先初始化
            initModuleStatusInfo(cam_id);
            // 再次检查
            pub_it = module_info_publisher_map_.find(cam_id);
            if (pub_it == module_info_publisher_map_.end()) {
                LOG_ERROR("相机 %d 的发布器初始化失败", cam_id);
                return;
            }
        }
        
        // 直接使用已存储的 ModuleStatusInfo
        const auto& status_info = it->second;
        
        // 序列化为 JSON 字符串
        std_msgs::msg::String ros_msg;
        ros_msg.data = moduleStatusInfoToJson(status_info);
        
        // 发布消息
        pub_it->second->publish(ros_msg);
        LOG_DEBUG("发布相机 %d 的 ModuleStatusInfo: %s", cam_id, ros_msg.data.c_str());
    }
} // namespace cam_mgr_ros