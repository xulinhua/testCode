#include "../include/cam_manage/cam_realsense.hpp"
#include <iostream>
#include <iomanip>
// 添加系统相关头文件
#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>

#include <librealsense2/rs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>   // 用于 normalize, applyColorMap
#include <opencv2/imgcodecs.hpp> // 用于图像编码

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = std::shared_ptr<PointCloudXYZ>;
using PointCloudXYZConstPtr = std::shared_ptr<const PointCloudXYZ>;

std::vector<CamDevInfo> CamRealsense::cam_info_list_;
rs2::device_list CamRealsense::device_list_;

CamRealsense::CamRealsense()
{
    // 参数初始化
    cam_para = CamComPara();
}

CamRealsense::~CamRealsense()
{
    close_cam();
}

// 检测并杀死占用指定设备的进程
void CamRealsense::killOccupyingProcesses(const std::string &serial_number)
{
    try
    {
        // 在Linux系统中，可以通过/proc文件系统查找占用设备的进程
        DIR *proc_dir = opendir("/proc");
        if (!proc_dir)
        {
            LOG_WARN("cam_manage",  "Failed to open /proc directory");
            return;
        }

        struct dirent *entry;
        while ((entry = readdir(proc_dir)) != nullptr)
        {
            // 检查是否为进程目录（数字命名）
            if (entry->d_type == DT_DIR && std::isdigit(entry->d_name[0]))
            {
                std::string pid_str = entry->d_name;
                int pid = std::stoi(pid_str);

                // 检查该进程是否占用相机设备
                if (isProcessUsingDevice(pid, serial_number))
                {
                    // 杀死占用设备的进程
                    LOG_INFO("cam_manage",  "Killing process %d that occupies device %s", pid, serial_number.c_str());
                    if (kill(pid, SIGTERM) == 0)
                    {
                        LOG_INFO("cam_manage",  "Successfully sent SIGTERM to process %d", pid);
                        // 等待一段时间让进程优雅退出
                        sleep(1);
                        // 如果进程仍未退出，发送SIGKILL
                        if (kill(pid, 0) == 0)
                        {
                            LOG_WARN("cam_manage",  "Process %d still alive, sending SIGKILL", pid);
                            kill(pid, SIGKILL);
                        }
                    }
                    else
                    {
                        LOG_ERROR("cam_manage",  "Failed to kill process %d", pid);
                    }
                }
            }
        }
        closedir(proc_dir);
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage",  "Error in killOccupyingProcesses: %s", e.what());
    }
}

// 检查指定进程是否正在使用指定的相机设备
bool CamRealsense::isProcessUsingDevice(int pid, const std::string &serial_number)
{
    try
    {
        // 构造fd目录路径
        std::string fd_path = "/proc/" + std::to_string(pid) + "/fd";
        DIR *fd_dir = opendir(fd_path.c_str());
        if (!fd_dir)
        {
            return false; // 无法打开fd目录，可能进程已退出
        }

        struct dirent *entry;
        bool found = false;
        while ((entry = readdir(fd_dir)) != nullptr)
        {
            if (entry->d_type == DT_LNK) // 符号链接
            {
                char link_path[512];
                char target_path[512];
                snprintf(link_path, sizeof(link_path), "%s/%s", fd_path.c_str(), entry->d_name);

                ssize_t len = readlink(link_path, target_path, sizeof(target_path) - 1);
                if (len != -1)
                {
                    target_path[len] = '\0';

                    // 检查目标路径是否包含设备序列号
                    std::string target(target_path);
                    if (target.find(serial_number) != std::string::npos)
                    {
                        found = true;
                        break;
                    }
                }
            }
        }
        closedir(fd_dir);
        return found;
    }
    catch (...)
    {
        return false;
    }
}
RtnType CamRealsense::get_all_devices(std::vector<CamDevInfo> &cam_info_list)
{
    try
    {
        // std::cout << "CamRealsense::get_all_devices start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_all_devices start");
        // 如果已经有值则不再遍历
        if (!cam_info_list_.empty())
        {
            cam_info_list = cam_info_list_;
            // std::cout << "CamRealsense::get_all_devices using cached data" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_all_devices using cached data");
            return RtnType::RTN_SUCCESS;
        }
        rs2::context ctx;
        device_list_ = ctx.query_devices();
        cam_info_list_.clear();

        for (auto &&dev : device_list_)
        {
            CamDevInfo info;
            info.cam_type = CamType::CAM_TYPE_RS;
            info.device_name = dev.get_info(RS2_CAMERA_INFO_NAME);                  // 设备名称
            info.serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);       // 序列号
            info.firmware_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION); // 固件版本
            info.physical_port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);       // 接口类型
            info.product_id = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);             // 产品id
            cam_info_list_.push_back(info);
        }
        cam_info_list = cam_info_list_;
        // std::cout << "CamRealsense::get_all_devices success" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_all_devices success");
        return RtnType::RTN_SUCCESS;
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
        LOG_ERROR("cam_manage",  "Exception in get_all_devices: %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::init(const CamConfigInfo &config)
{
    try
    {
        if (config.cam_type != CamType::CAM_TYPE_RS)
        {
            // std::cout << __func__ << "配置文件类型不匹配" << std::endl;
            LOG_ERROR("cam_manage",  "%s 配置文件类型不匹配", __func__);
            return RtnType::RTN_FAILIURE;
        }

        // 只有在设备列表为空时才重新枚举设备
        if (cam_info_list_.empty())
        {
            get_all_devices(cam_info_list_);
        }

        // 根据config配置对应相机
        if (cam_info_list_.size() == 0)
        {
            return RtnType::RTN_FAILIURE;
        }
        bool is_match_cam = false;
        for (size_t i = 0; i < cam_info_list_.size(); i++)
        {
            if (cam_info_list_[i].serial_number == config.serial_number)
            {
                cam_info_ = cam_info_list_[i];
                device_ = device_list_[i];
                is_match_cam = true;
                break;
            }
            else if (i == config.cam_index)
            {
                cam_info_ = cam_info_list_[i];
                device_ = device_list_[i];
                is_match_cam = true;
                break;
            }
        }
        if (is_match_cam)
        {
            // std::cout << "CamRealsense::init 初始化成功" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::init 初始化成功");
            return RtnType::RTN_SUCCESS;
        }
        else
        { // 配置文件未匹配到对应相机
            // std::cout << "CamRealsense::init 未匹配到对应相机" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::init 未匹配到对应相机");
            return RtnType::RTN_FAILIURE_NOFOUND;
        }
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::init " << e.what() << '\n';
        LOG_ERROR("cam_manage",  "CamRealsense::init %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::open_cam()
{
    try
    {
        // std::cout << "CamRealsense::open_cam start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::open_cam start");
        if (!device_.get())
        {
            // std::cout << "CamRealsense::open_cam 无匹配相机" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::open_cam 无匹配相机");
            return RtnType::RTN_FAILIURE_NONE;
        }
        if (!cam_para.enable_depth_stream && !cam_para.enable_color_stream && !cam_para.enable_ir_stream && !cam_para.enable_cloud_stream)
        {
            // std::cout << "CamRealsense::open_cam 未开启任何流,请重新设置" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::open_cam 未开启任何流,请重新设置");
            return RtnType::RTN_FAILIURE_NONE;
        }

        // 检查设备是否被占用，如果被占用则杀死对应进程
        try
        {
            std::string serial_number = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            LOG_INFO("cam_manage",  "Checking if device %s is occupied", serial_number.c_str());

            // 杀死占用设备的进程
            killOccupyingProcesses(serial_number);
        }
        catch (const rs2::error &e)
        {
            LOG_WARN("cam_manage",  "Failed to get device serial number: %s", e.what());
        }

        // 尝试重置设备以解决电源状态问题
        // try {
        //     LOG_INFO("cam_manage",  "Attempting to reset device to fix power state issue");
        //     device_.hardware_reset();
        //     // 等待设备重启
        //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        // } catch (const rs2::error& e) {
        //     LOG_WARN("cam_manage",  "Failed to reset device: %s", e.what());
        // }

        // 获取当前相机适合的roi和fps
        std::map<std::string, CamRoiFpsList> cam_roi_fps_list;
        get_cam_roi_fps_list(cam_roi_fps_list);

        // 将设备信息存储到配置中，而不是直接关联到pipeline
        config_.enable_device(device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        if (cam_para.enable_depth_stream || cam_para.enable_cloud_stream)
        { // 深度图和点云
            int fps_dep = cam_para.depth_stream_para.fps;
            int width_dep = cam_para.depth_stream_para.width;
            int height_dep = cam_para.depth_stream_para.height;

            // 使用新接口计算最接近的配置
            int closest_width_dep, closest_height_dep, closest_fps_dep;
            RtnType rtn = get_closest_roi_fps(CamStreamType::STREAM_DEPTH, width_dep, height_dep, fps_dep,
                                              closest_width_dep, closest_height_dep, closest_fps_dep);
            if (rtn == RtnType::RTN_SUCCESS)
            {
                // 使用最接近的配置
                width_dep = closest_width_dep;
                height_dep = closest_height_dep;
                fps_dep = closest_fps_dep;
                LOG_INFO("cam_manage", "Using closest depth configuration: %dx%d@%d FPS (requested: %dx%d@%d FPS)", width_dep, height_dep, 
                    fps_dep, cam_para.depth_stream_para.width, cam_para.depth_stream_para.height, cam_para.depth_stream_para.fps);
            }
            else
            {
                LOG_WARN("cam_manage", "Failed to find closest depth configuration, using original settings");
            }

            if (fps_dep < 1)
                fps_dep = 0;
            // std::cout << "CamRealsense::open_cam::fps_depth:" << fps_dep << " width:" << width_dep << " height:" << height_dep << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::open_cam::fps_depth:%d width:%d height:%d", fps_dep, width_dep, height_dep);
            config_.enable_stream(RS2_STREAM_DEPTH, width_dep, height_dep, RS2_FORMAT_Z16, fps_dep);
        }
        else
        {
            // std::cout << "CamRealsense::open_cam::深度流/点云流关闭" << " width:" << cam_para.depth_stream_para.width << " height:" << cam_para.depth_stream_para.height << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::open_cam::深度流/点云流关闭 width:%d height:%d", cam_para.depth_stream_para.width, cam_para.depth_stream_para.height);
            config_.disable_stream(RS2_STREAM_DEPTH);
        }
        if (cam_para.enable_color_stream)
        {
            int fps_clr = cam_para.color_stream_para.fps;
            int width_clr = cam_para.color_stream_para.width;
            int height_clr = cam_para.color_stream_para.height;

            // 使用新接口计算最接近的配置
            int closest_width_clr, closest_height_clr, closest_fps_clr;
            RtnType rtn = get_closest_roi_fps(CamStreamType::STREAM_COLOR, width_clr, height_clr, fps_clr, 
                closest_width_clr, closest_height_clr, closest_fps_clr);
            if (rtn == RtnType::RTN_SUCCESS)
            {
                // 使用最接近的配置
                width_clr = closest_width_clr;
                height_clr = closest_height_clr;
                fps_clr = closest_fps_clr;
                LOG_INFO("cam_manage", "Using closest color configuration: %dx%d@%d FPS (requested: %dx%d@%d FPS)", width_clr, height_clr, 
                    fps_clr, cam_para.color_stream_para.width, cam_para.color_stream_para.height, cam_para.color_stream_para.fps);
            }
            else
            {
                LOG_WARN("cam_manage", "Failed to find closest color configuration, using original settings");
            }
            // if (fps_clr < 1)
            //     fps_clr = 0;
            // std::cout << "CamRealsense::open_cam::fps_color:" << fps_clr << " width:" << width_clr << " height:" << height_clr << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::open_cam::fps_color:%d width:%d height:%d", fps_clr, width_clr, height_clr);
            config_.enable_stream(RS2_STREAM_COLOR, width_clr, height_clr, RS2_FORMAT_BGR8, fps_clr);
        }
        else
        {
            // std::cout << "CamRealsense::open_cam::图像流关闭" << " width:" << cam_para.color_stream_para.width << " height:" << cam_para.color_stream_para.height << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::open_cam::图像流关闭 width:%d height:%d", cam_para.color_stream_para.width, cam_para.color_stream_para.height);
            config_.disable_stream(RS2_STREAM_COLOR);
        }

        pipe_.start(config_);
        // std::cerr << "CamRealsense::open_cam 打开相机成功" << std::endl;
        LOG_INFO("cam_manage",  "打开相机成功：serial num:%s", cam_info_.serial_number.c_str());
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::open_cam error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::open_cam error: %s", e.what());

        // 如果是电源状态错误，记录日志并提示用户
        if (std::string(e.what()).find("failed to set power state") != std::string::npos)
        {
            LOG_INFO("cam_manage",  "Detected power state error. Please try reloading uvcvideo driver with: sudo rmmod uvcvideo && sudo modprobe uvcvideo");
        }
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::open_cam Error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::open_cam Error: %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}
RtnType CamRealsense::close_cam()
{
    try
    {
        // std::cerr << "CamRealsense::close_cam start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::close_cam start");
        if (is_cam_start())
        { // 开启后再关闭
            pipe_.stop();
        }
        // std::cerr << "CamRealsense::close_cam 关闭相机成功" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::close_cam 关闭相机成功");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "RealSense error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "RealSense error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "Error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "Error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}
RtnType CamRealsense::init_defult_para()
{
    return RtnType::RTN_SUCCESS;
}
RtnType CamRealsense::init_special_para()
{
    return RtnType::RTN_SUCCESS;
}
RtnType CamRealsense::get_stream_enable(CamStreamType modu, bool &enable)
{
    try
    {
        // 先检查管道是否已启动
        // 修复：使用正确的API检查管道状态
        try {
            auto profile = pipe_.get_active_profile();
            if (!profile) {
                enable = false;
                LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable no active profile");
                return RtnType::RTN_SUCCESS;
            }
        } catch (const rs2::error& e) {
            enable = false;
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable pipe not active: %s", e.what());
            return RtnType::RTN_SUCCESS;
        } catch (const std::exception& e) {
            enable = false;
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable pipe not active: %s", e.what());
            return RtnType::RTN_SUCCESS;
        }
        
        // 获取当前活动配置
        auto profile = pipe_.get_active_profile();
        if (!profile) {
            enable = false;
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable no active profile");
            return RtnType::RTN_SUCCESS;
        }
        
        // 获取配置中的流列表
        std::vector<rs2::stream_profile> streams;
        try {
            streams = profile.get_streams();
        } catch (const std::exception& e) {
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable failed to get streams: %s", e.what());
            enable = false;
            return RtnType::RTN_SUCCESS;
        }
        
        // 检查请求的流是否存在
        bool stream_found = false;
        try {
            for (auto&& stream : streams) {
                if (modu == CamStreamType::STREAM_DEPTH && stream.stream_type() == RS2_STREAM_DEPTH) {
                    stream_found = true;
                    break;
                }
                else if (modu == CamStreamType::STREAM_COLOR && stream.stream_type() == RS2_STREAM_COLOR) {
                    stream_found = true;
                    break;
                }
            }
        } catch (const std::exception& e) {
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable failed to check streams: %s", e.what());
            enable = false;
            return RtnType::RTN_SUCCESS;
        }
        
        enable = stream_found;
        
        if (stream_found) {
            LOG_INFO("cam_manage",  "CamRealsense::get_stream_enable stream is enabled");
        } else {
            LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable stream is not enabled");
        }
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        enable = false;
        LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable failed: %s", e.what());
        return RtnType::RTN_SUCCESS; // 即使出错也返回成功，但enable为false
    }
    catch (const std::exception &e)
    {
        enable = false;
        LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable failed: %s", e.what());
        return RtnType::RTN_SUCCESS; // 即使出错也返回成功，但enable为false
    }
    catch (...)
    {
        enable = false;
        LOG_WARN("cam_manage",  "CamRealsense::get_stream_enable failed with unknown error");
        return RtnType::RTN_SUCCESS; // 即使出错也返回成功，但enable为false
    }
}
RtnType CamRealsense::set_stream_enable(CamStreamType modu, bool enable)
{
    try
    {
        // std::cout << "CamRealsense::set_stream_enable start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_stream_enable start");
        if (enable)
        {
            if (modu == CamStreamType::STREAM_DEPTH)
            {
                config_.enable_stream(RS2_STREAM_DEPTH, cam_para.depth_stream_para.width,
                                      cam_para.depth_stream_para.height, RS2_FORMAT_Z16, cam_para.depth_stream_para.fps);
            }
            else if (modu == CamStreamType::STREAM_COLOR)
            {
                config_.enable_stream(RS2_STREAM_COLOR, cam_para.color_stream_para.width,
                                      cam_para.color_stream_para.height, RS2_FORMAT_BGR8, cam_para.color_stream_para.fps);
            }
        }
        else
        {
            if (modu == CamStreamType::STREAM_DEPTH)
            {
                config_.disable_stream(RS2_STREAM_DEPTH);
            }
            else if (modu == CamStreamType::STREAM_COLOR)
            {
                config_.disable_stream(RS2_STREAM_COLOR);
            }
        }
        if (is_cam_start())
        {
            try {
                close_cam();
            } catch (const std::exception& e) {
                LOG_WARN("cam_manage",  "Failed to close camera before reconfiguration: %s", e.what());
            }
        }
        
        try {
            pipe_.start(config_);
        } catch (const rs2::error& e) {
            LOG_ERROR("cam_manage",  "Failed to start camera pipeline: %s", e.what());
            return RtnType::RTN_FAILIURE;
        } catch (const std::exception& e) {
            LOG_ERROR("cam_manage",  "Failed to start camera pipeline: %s", e.what());
            return RtnType::RTN_FAILIURE;
        }
        // std::cout << "CamRealsense::set_stream_enable success" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_stream_enable success");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::set_stream_enable error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_stream_enable error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::set_stream_enable error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_stream_enable error: %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}
RtnType CamRealsense::set_cam_config(const CamConfigInfo &config)
{
    // std::cout << "CamRealsense::set_cam_config start" << std::endl;
    LOG_INFO("cam_manage",  "CamRealsense::set_cam_config start");

    RtnType rtn = RtnType::RTN_SUCCESS;
    cam_para.enable_depth_stream = config.enable_depth_stream;
    cam_para.enable_color_stream = config.enable_color_stream;
    cam_para.enable_ir_stream = config.enable_ir_stream;
    cam_para.enable_cloud_stream = config.enable_cloud_stream;

    cam_para.depth_stream_para.width = config.depth_para.width;
    cam_para.depth_stream_para.height = config.depth_para.height;
    cam_para.depth_stream_para.fps = config.depth_para.fps;
    cam_para.color_stream_para.width = config.color_para.width;
    cam_para.color_stream_para.height = config.color_para.height;
    cam_para.color_stream_para.fps = config.color_para.fps;

    // std::cout << "CamRealsense::set_cam_config end" << std::endl;
    LOG_INFO("cam_manage",  "CamRealsense::set_cam_config end");
    return rtn;
}
RtnType CamRealsense::init_pixel_type()
{
    return RtnType::RTN_SUCCESS;
}
CamType CamRealsense::get_cam_type()
{
    return CamType::CAM_TYPE_RS;
}
CamInterfaceType CamRealsense::get_cam_interface_type()
{
    return CamInterfaceType::INF_TYPE_USB;
}

std::string CamRealsense::get_serial_number()
{
    return cam_info_.serial_number;
}

RtnType CamRealsense::get_cam_roi_fps_list(std::map<std::string, CamRoiFpsList> &cam_roi_fps_list)
{
    try
    {
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_roi_fps_list start");

        if (!device_.get())
        {
            LOG_ERROR("cam_manage",  "Device not initialized");
            return RtnType::RTN_FAILIURE;
        }

        cam_roi_fps_list.clear();

        // 使用更精确的方法查询传感器
        std::vector<rs2::sensor> sensors = device_.query_sensors();

        if (sensors.empty())
        {
            LOG_ERROR("cam_manage",  "No sensors found on device");
            return RtnType::RTN_FAILIURE;
        }

        // 为每种流类型创建单独的列表
        std::map<rs2_stream, CamRoiFpsList> stream_profiles_map;

        for (const auto &sensor : sensors)
        {
            std::string sensor_name = "unknown";
            try
            {
                sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
                LOG_DEBUG("cam_manage", "Processing sensor: %s", sensor_name.c_str());
            }
            catch (const rs2::error &e)
            {
                LOG_WARN("cam_manage", "Failed to get sensor name: %s", e.what());
            }

            std::vector<rs2::stream_profile> profiles;
            try
            {
                profiles = sensor.get_stream_profiles();
            }
            catch (const rs2::error &e)
            {
                LOG_WARN("cam_manage", "Failed to get stream profiles for sensor '%s': %s", sensor_name.c_str(), e.what());
                continue;
            }
            LOG_DEBUG("cam_manage", "Sensor '%s' has %d profiles", sensor_name.c_str(), static_cast<int>(profiles.size()));
            // 遍历所有配置，按流类型分类
            for (const auto &profile : profiles)
            {
                if (!profile.is<rs2::video_stream_profile>())
                    continue;

                try
                {
                    rs2::video_stream_profile videoProfile = profile.as<rs2::video_stream_profile>();
                    rs2_stream stream_type = profile.stream_type();
                    int stream_index = profile.stream_index();

                    // 创建流类型的唯一标识
                    std::string stream_key;
                    switch (stream_type)
                    {
                    case RS2_STREAM_COLOR:
                        stream_key = "color";
                        break;
                    case RS2_STREAM_DEPTH:
                        stream_key = "depth";
                        break;
                    case RS2_STREAM_INFRARED:
                        // 红外流可能有多个（1和2），需要区分
                        stream_key = (stream_index == 1) ? "infrared1" : (stream_index == 2) ? "infrared2"
                                                                                             : "infrared";
                        break;
                    default:
                        // 跳过不需要的流类型
                        continue;
                    }

                    // 创建ROI和FPS信息
                    CamRoiFps roiFps;
                    roiFps.width = videoProfile.width();
                    roiFps.height = videoProfile.height();
                    roiFps.fps = static_cast<int>(videoProfile.fps());

                    // 检查是否已存在相同的配置
                    bool exists = false;
                    for (const auto &existing : stream_profiles_map[stream_type])
                    {
                        if (existing.width == roiFps.width &&
                            existing.height == roiFps.height &&
                            existing.fps == roiFps.fps)
                        {
                            exists = true;
                            break;
                        }
                    }

                    if (!exists)
                    {
                        stream_profiles_map[stream_type].push_back(roiFps);
                        LOG_DEBUG("cam_manage", "Added profile for %s: %dx%d @ %d fps", stream_key.c_str(), roiFps.width, roiFps.height, roiFps.fps);
                    }
                }
                catch (const rs2::error &e)
                {
                    LOG_WARN("cam_manage", "Failed to process profile: %s", e.what());
                    continue;
                }
            }
        }

        // 转换为输出格式
        for (const auto &entry : stream_profiles_map)
        {
            rs2_stream stream_type = entry.first;
            const CamRoiFpsList &profiles = entry.second;

            std::string stream_key;
            switch (stream_type)
            {
            case RS2_STREAM_COLOR:
                stream_key = "color";
                break;
            case RS2_STREAM_DEPTH:
                stream_key = "depth";
                break;
            case RS2_STREAM_INFRARED:
                stream_key = "infrared";
                break;
            default:
                continue; // 跳过不需要的流类型
            }

            if (!profiles.empty())
            {
                cam_roi_fps_list[stream_key] = profiles;
                if (is_show_roi_fps_log_)
                {
                    LOG_INFO("cam_manage", "Found %d profiles for %s", static_cast<int>(profiles.size()), stream_key.c_str());
                }
            }
        }
        // 检查是否有color和depth数据
        bool has_color = (cam_roi_fps_list.find("color") != cam_roi_fps_list.end());
        bool has_depth = (cam_roi_fps_list.find("depth") != cam_roi_fps_list.end());
        if (is_show_roi_fps_log_)
        {
            LOG_INFO("cam_manage", "Result - Has color: %s, Has depth: %s", has_color ? "YES" : "NO", has_depth ? "YES" : "NO");
        }

        // 如果都没有，尝试备用方案
        if (!has_color && !has_depth)
        {
            // return try_alternative_query_method(cam_roi_fps_list);
        }

        cam_roi_fps_list_ = cam_roi_fps_list;

        // 打印获取到的ROI和FPS列表
        if (is_show_roi_fps_log_)
        {
            LOG_INFO("cam_manage",  "Retrieved ROI and FPS list:");
            for (const auto &entry : cam_roi_fps_list)
            {
                const std::string &sensorType = entry.first;
                const CamRoiFpsList &roiFpsList = entry.second;

                LOG_INFO("cam_manage",  "Sensor type: %s, Supported profiles:", sensorType.c_str());
                for (const auto &roiFps : roiFpsList)
                {
                    LOG_INFO("cam_manage",  "  Resolution: %dx%d, FPS: %d", roiFps.width, roiFps.height, roiFps.fps);
                }
            }
        }
        LOG_INFO("cam_manage",  "Successfully retrieved and printed ROI and FPS list");
        return RtnType::RTN_SUCCESS;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage", "Exception in get_cam_roi_fps_list: %s", e.what());
    }
    catch (...)
    {
        LOG_ERROR("cam_manage", "Unknown exception in get_cam_roi_fps_list");
    }

    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::get_closest_roi_fps(CamStreamType stream_type, int target_width, int target_height, int target_fps, int &closest_width, int &closest_height, int &closest_fps)
{
    try
    {
        LOG_DEBUG("cam_manage",  "CamRealsense::get_closest_roi_fps start");
        if (cam_roi_fps_list_.size() == 0)
        {
            LOG_ERROR("cam_manage",  "Failed to get camera ROI and FPS list");
            return RtnType::RTN_FAILIURE;
        }
        // 根据流类型确定要查找的传感器类型
        std::string sensor_type;
        switch (stream_type)
        {
        case CamStreamType::STREAM_COLOR:
            sensor_type = "color";
            break;
        case CamStreamType::STREAM_DEPTH:
            sensor_type = "depth";
            break;
        case CamStreamType::STREAM_IR:
            sensor_type = "ir";
            break;
        default:
            LOG_ERROR("cam_manage",  "Invalid stream type: %d", static_cast<int>(stream_type));
            return RtnType::RTN_FAILIURE;
        }

        // 检查是否存在对应传感器类型的配置
        auto it = cam_roi_fps_list_.find(sensor_type);
        if (it == cam_roi_fps_list_.end())
        {
            LOG_ERROR("cam_manage",  "No configurations found for sensor type: %s", sensor_type.c_str());
            return RtnType::RTN_FAILIURE;
        }

        const CamRoiFpsList &roi_fps_list = it->second;
        if (roi_fps_list.empty())
        {
            LOG_ERROR("cam_manage",  "Empty configuration list for sensor type: %s", sensor_type.c_str());
            return RtnType::RTN_FAILIURE;
        }

        // 查找匹配的分辨率和最接近的FPS
        int best_match_index = -1;
        int min_resolution_diff = INT_MAX;
        int closest_fps_diff = INT_MAX;

        // 第一步：查找最匹配的分辨率（长宽）
        for (size_t i = 0; i < roi_fps_list.size(); i++)
        {
            const CamRoiFps &roi_fps = roi_fps_list[i];

            // 计算与目标配置的分辨率差异
            int width_diff = abs(roi_fps.width - target_width);
            int height_diff = abs(roi_fps.height - target_height);
            int resolution_diff = width_diff + height_diff;

            if (resolution_diff < min_resolution_diff)
            {
                // 找到更接近的分辨率，更新最佳匹配
                min_resolution_diff = resolution_diff;
                best_match_index = static_cast<int>(i);
                // 在相同分辨率下，查找最接近的FPS
                closest_fps_diff = abs(roi_fps.fps - target_fps);
            }
            else if (resolution_diff == min_resolution_diff)
            {
                // 分辨率相同，比较FPS差异
                int fps_diff = abs(roi_fps.fps - target_fps);
                if (fps_diff < closest_fps_diff)
                {
                    best_match_index = static_cast<int>(i);
                    closest_fps_diff = fps_diff;
                }
            }
        }

        // 检查是否找到了匹配项
        if (best_match_index == -1)
        {
            LOG_ERROR("cam_manage",  "No suitable configuration found");
            return RtnType::RTN_FAILIURE;
        }
        // 使用找到的最佳匹配配置
        const CamRoiFps &best_match = roi_fps_list[best_match_index];
        closest_width = best_match.width;
        closest_height = best_match.height;
        if (target_fps > 0)
        {
            closest_fps = best_match.fps;
        }
        else
        {
            closest_fps = 0;
        }
        LOG_INFO("cam_manage", "Found closest configuration for %s stream - Target: %dx%d@%d FPS, Closest: %dx%d@%d FPS", 
            sensor_type.c_str(), target_width, target_height, target_fps, closest_width, closest_height, closest_fps);
        return RtnType::RTN_SUCCESS;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage",  "Exception in get_closest_roi_fps: %s", e.what());
        // 检查是否是电源状态错误
        std::string error_msg(e.what());
        if (error_msg.find("failed to set power state") != std::string::npos)
        {
            LOG_ERROR("cam_manage", "Power state error in get_closest_roi_fps. This may be caused by driver issues or another process using the camera.");
        }
    }
    catch (...)
    {
        LOG_ERROR("cam_manage",  "Unknown exception in get_closest_roi_fps");
    }

    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::get_cam_para_enable(CamParaType para_type, CamStreamType modu, bool &enable)
{
    try
    {
        // std::cout << "CamRealsense::get_cam_para_enable start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_enable start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        auto stream_profile = pipe_.get_active_profile().get_stream(modu == CamStreamType::STREAM_DEPTH ? RS2_STREAM_DEPTH : RS2_STREAM_COLOR);
        if (para_type == CamParaType::PARA_AUTO_EXPOSURE)
        {
            enable = sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE) &&
                     sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE) > 0;
        }
        else if (para_type == CamParaType::PARA_AUTO_WHITE_BALANCE)
        {
            enable = sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) &&
                     sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) > 0;
        }
        // std::cout << "CamRealsense::get_cam_para_enable success" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_enable success");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        enable = false;
        // std::cout << "CamRealsense::get_cam_para_enable error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_enable error");
        return RtnType::RTN_FAILIURE;
    }
}
RtnType CamRealsense::set_cam_para_enable(CamParaType para_type, CamStreamType modu, bool enable)
{
    try
    {
        // std::cout << "CamRealsense::set_cam_para_enable start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_cam_para_enable start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (para_type == CamParaType::PARA_AUTO_EXPOSURE)
        {
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable ? 1 : 0);
            }
        }
        else if (para_type == CamParaType::PARA_AUTO_WHITE_BALANCE)
        {
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, enable ? 1 : 0);
            }
        }
        // std::cout << "CamRealsense::set_cam_para_enable end" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_cam_para_enable end");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::set_cam_para_enable error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para_enable error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::set_cam_para_enable error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para_enable error: %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::get_cam_para_range(CamParaType para_type, float &min_val, float &max_val, CamStreamType modu)
{
    try
    {
        // std::cout << "CamRealsense::get_cam_para_range start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_range start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }
        if (sensor.supports(option))
        {
            min_val = sensor.get_option_range(option).min;
            max_val = sensor.get_option_range(option).max;
            // std::cout << "CamRealsense::get_cam_para_range success" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_range success");
            return RtnType::RTN_SUCCESS;
        }
        // std::cout << "CamRealsense::get_cam_para_range error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_range error");
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::get_cam_para_range error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_range error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::get_cam_para_range error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_range error: %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}
RtnType CamRealsense::set_cam_para(CamParaType para_type, float value, CamStreamType modu)
{
    try
    {
        // std::cout << "CamRealsense::set_cam_para start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_cam_para start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }
        if (sensor.supports(option))
        {
            sensor.set_option(option, value);
            // std::cout << "CamRealsense::set_cam_para success" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::set_cam_para success");
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::set_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::set_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}
RtnType CamRealsense::get_cam_para(CamParaType para_type, float &value, CamStreamType modu)
{
    try
    {
        // std::cout << "CamRealsense::get_cam_para start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_para start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }

        if (sensor.supports(option))
        {
            value = sensor.get_option(option);
            // std::cout << "CamRealsense::get_cam_para success" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_cam_para success");
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::get_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::get_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::get_cam_para_range(CamParaType para_type, int &min_val, int &max_val, CamStreamType modu)
{
    try
    {
        // std::cout << "CamRealsense::get_cam_para_range start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_range start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }

        if (sensor.supports(option))
        {
            min_val = static_cast<int>(sensor.get_option_range(option).min);
            max_val = static_cast<int>(sensor.get_option_range(option).max);
            // std::cout << "CamRealsense::get_cam_para_range success" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_cam_para_range success");
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::get_cam_para_range error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_range error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::get_cam_para_range error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_para_range error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::set_cam_para(CamParaType para_type, int value, CamStreamType modu)
{
    try
    {
        // std::cout << "CamRealsense::set_cam_para start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::set_cam_para start");
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }

        if (sensor.supports(option))
        {
            sensor.set_option(option, static_cast<float>(value));
            // std::cout << "CamRealsense::set_cam_para success" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::set_cam_para success");
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::set_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::set_cam_para error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_cam_para error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_cam_para(CamParaType para_type, int &value, CamStreamType modu)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        rs2_option option = RS2_OPTION_COUNT;
        switch (para_type)
        {
        case CamParaType::PARA_EXPOSURE:
            option = RS2_OPTION_EXPOSURE;
            break;
        case CamParaType::PARA_GAIN:
            option = RS2_OPTION_GAIN;
            break;
        case CamParaType::PARA_GAMMA:
            option = RS2_OPTION_GAMMA;
            break;
        case CamParaType::PARA_BRIGHTNESS:
            option = RS2_OPTION_BRIGHTNESS;
            break;
        case CamParaType::PARA_CONTRAST:
            option = RS2_OPTION_CONTRAST;
            break;
        case CamParaType::PARA_SATURATION:
            option = RS2_OPTION_SATURATION;
            break;
        case CamParaType::PARA_SHARPNESS:
            option = RS2_OPTION_SHARPNESS;
            break;
        case CamParaType::PARA_HUE:
            option = RS2_OPTION_HUE;
            break;
        default:
            return RtnType::RTN_FAILIURE;
        }

        if (sensor.supports(option))
        {
            value = static_cast<int>(sensor.get_option(option));
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "RealSense error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "RealSense error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "Error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "Error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    return RtnType::RTN_FAILIURE;
}

RtnType CamRealsense::is_cam_start(bool &is_start)
{
    try
    {
        if (pipe_.get_active_profile().get_device())
            is_start = true;
    }
    catch (const rs2::error &e)
    {
    }
    catch (...)
    {
    }
    is_start = false;

    return RtnType::RTN_SUCCESS;
}

bool CamRealsense::is_cam_start()
{
    bool is_start = false;
    is_cam_start(is_start);
    return is_start;
}

RtnType CamRealsense::get_cam_intrinsics(CamStreamType modu, CamIntrinsics &intrinsics)
{
    try
    {
        // std::cout << "CamRealsense::get_cam_intrinsics start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_intrinsics start");
        for (int i = 0; i < 1; i++)
        { // 需要等传感器稳定 //为提高速度，先设置1张
            pipe_.wait_for_frames();
        }
        auto profile = pipe_.get_active_profile();
        rs2::stream_profile stream_profile;

        switch (modu)
        {
        case CamStreamType::STREAM_COLOR:
            if (!cam_para.enable_color_stream)
            {
                // std::cout << "CamRealsense::get_cam_intrinsics 未开启图像流!" << std::endl;
                LOG_WARN("cam_manage",  "CamRealsense::get_cam_intrinsics 未开启图像流!");
                return RtnType::RTN_FAILIURE;
            }
            stream_profile = profile.get_stream(RS2_STREAM_COLOR);
            break;
        case CamStreamType::STREAM_DEPTH:
            if (!cam_para.enable_depth_stream && !cam_para.enable_cloud_stream)
            {
                // std::cout << "CamRealsense::get_cam_intrinsics 未开启深度流!" << std::endl;
                LOG_WARN("cam_manage",  "CamRealsense::get_cam_intrinsics 未开启深度流!");
                return RtnType::RTN_FAILIURE;
            }
            stream_profile = profile.get_stream(RS2_STREAM_DEPTH);
            break;
        case CamStreamType::STREAM_IR:
            if (!cam_para.enable_ir_stream)
            {
                // std::cout << "CamRealsense::get_cam_intrinsics 未开启红外流!" << std::endl;
                LOG_WARN("cam_manage",  "CamRealsense::get_cam_intrinsics 未开启红外流!");
                return RtnType::RTN_FAILIURE;
            }
            stream_profile = profile.get_stream(RS2_STREAM_DEPTH);
            break;
        default:
        {
            // std::cout << "CamRealsense::get_cam_intrinsics 未知流类型!" << std::endl;
            LOG_ERROR("cam_manage",  "CamRealsense::get_cam_intrinsics 未知流类型!");
            return RtnType::RTN_FAILIURE;
        }
        }

        auto video_profile = stream_profile.as<rs2::video_stream_profile>();
        auto intrinsics_rs = video_profile.get_intrinsics();

        intrinsics.width = intrinsics_rs.width;
        intrinsics.height = intrinsics_rs.height;
        intrinsics.fx = intrinsics_rs.fx;
        intrinsics.fy = intrinsics_rs.fy;
        intrinsics.cx = intrinsics_rs.ppx;
        intrinsics.cy = intrinsics_rs.ppy;
        for (int i = 0; i < 5; i++)
        {
            intrinsics.dist_coeffs[i] = intrinsics_rs.coeffs[i];
        }

        // std::cout << "CamRealsense::get_cam_intrinsics success" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_cam_intrinsics success");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << __func__ << "CamRealsense::get_cam_intrinsics error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_intrinsics error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (const std::exception &e)
    {
        // std::cerr << __func__ << "CamRealsense::get_cam_intrinsics error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_cam_intrinsics error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_pixel_format(PixelFormat &pixel_format)
{
    return RtnType::RTN_SUCCESS;
}
RtnType CamRealsense::set_pixel_format(PixelFormat pixel_format)
{
    return RtnType::RTN_SUCCESS;
}

RtnType CamRealsense::set_white_balance_mode(WhiteBalenceMode balanceWhiteMode)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
        {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
                              balanceWhiteMode == WhiteBalenceMode::AUTO ? 1.0f : 0.0f);
            return RtnType::RTN_SUCCESS;
        }
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << " CamRealsense::set_white_balance_mode error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_white_balance_mode error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_white_balance_mode(WhiteBalenceMode &balanceWhiteMode)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
        {
            balanceWhiteMode = sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) > 0.5f
                                   ? WhiteBalenceMode::AUTO
                                   : WhiteBalenceMode::MANUAL;
            return RtnType::RTN_SUCCESS;
        }
        // std::cout << "CamRealsense::get_white_balance_mode error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_white_balance_mode error");
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "RealSense error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "RealSense error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_balance_ratio_range(int &balan_min, int &balan_max)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (sensor.supports(RS2_OPTION_WHITE_BALANCE))
        {
            auto range = sensor.get_option_range(RS2_OPTION_WHITE_BALANCE);
            balan_min = static_cast<int>(range.min);
            balan_max = static_cast<int>(range.max);

            return RtnType::RTN_SUCCESS;
        }
        // std::cout << "CamRealsense::get_balance_ratio_range error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_balance_ratio_range error");
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::get_balance_ratio_range error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_balance_ratio_range error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::set_balance_ratio(int balan_ratio)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (sensor.supports(RS2_OPTION_WHITE_BALANCE))
        {
            sensor.set_option(RS2_OPTION_WHITE_BALANCE, static_cast<float>(balan_ratio));
            return RtnType::RTN_SUCCESS;
        }
        // std::cout << "CamRealsense::set_balance_ratio error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_balance_ratio error");
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "CamRealsense::set_balance_ratio error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::set_balance_ratio error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_balance_ratio(int &balan_ratio)
{
    try
    {
        auto sensor = pipe_.get_active_profile().get_device().first<rs2::sensor>();
        if (sensor.supports(RS2_OPTION_WHITE_BALANCE))
        {
            balan_ratio = static_cast<int>(sensor.get_option(RS2_OPTION_WHITE_BALANCE));
            return RtnType::RTN_SUCCESS;
        }
        // std::cout << "CamRealsense::get_balance_ratio error" << std::endl;
        LOG_ERROR("cam_manage",  "CamRealsense::get_balance_ratio error");
        return RtnType::RTN_FAILIURE;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "RealSense error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "RealSense error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_one_frame(CamFramelist &frames)
{

    try
    {
        // std::cout << "CamRealsense::get_one_frame start" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_one_frame start");
        frames.clear();
        if (!cam_para.enable_cloud_stream && !cam_para.enable_color_stream && !cam_para.enable_depth_stream && !cam_para.enable_ir_stream)
        {
            // std::cout << "CamRealsense::get_one_frame 未开启任何流" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 未开启任何流");
        }
        rs2::frameset rs_frames = pipe_.wait_for_frames();
        if (rs_frames.size() < 1)
        {
            // std::cout << "CamRealsense::get_one_frame 帧数据为空！" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 帧数据为空！");
            return RtnType::RTN_FAILIURE;
        }
        rs2::depth_frame dep_fra = rs_frames.get_depth_frame();
        if (dep_fra.get_data_size() > 0)
        {
            if (!cam_para.enable_depth_stream && !cam_para.enable_cloud_stream)
            {
                // std::cout << "CamRealsense::get_one_frame 深度流未开启！" << std::endl;
                LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 深度流未开启！");
            }
            CamFrameData *cam_depth_frame = new CamFrameData();
            cam_depth_frame->frame_type = CamStreamType::STREAM_DEPTH;
            int datasize = dep_fra.get_data_size();
            cam_depth_frame->data_buffer = new uint8_t[datasize];
            memcpy((void *)cam_depth_frame->data_buffer, (void *)dep_fra.get_data(), datasize);
            cam_depth_frame->data_size = datasize;
            cam_depth_frame->width = dep_fra.get_width();
            cam_depth_frame->height = dep_fra.get_height();
            cam_depth_frame->stride = dep_fra.get_stride_in_bytes();
            cam_depth_frame->time_stamp = dep_fra.get_timestamp();
            frames.push_back(cam_depth_frame);
            // std::cout << "depth datasize:" << cam_depth_frame->data_size << std::endl;
        }
        else
        {
            // std::cout << "CamRealsense::get_one_frame 深度帧数据为空！" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 深度帧数据为空！");
            return RtnType::RTN_FAILIURE;
        }
        rs2::video_frame col_fra = rs_frames.get_color_frame();
        if (col_fra.get_data_size() > 0)
        {
            if (!cam_para.enable_color_stream)
            {
                // std::cout << "CamRealsense::get_one_frame 深度流未开启！" << std::endl;
                LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 深度流未开启！");
            }
            CamFrameData *cam_color_frame = new CamFrameData();
            cam_color_frame->frame_type = CamStreamType::STREAM_COLOR;
            int datasize = col_fra.get_data_size();
            cam_color_frame->data_buffer = new uint8_t[datasize];
            memcpy((void *)cam_color_frame->data_buffer, (void *)col_fra.get_data(), datasize);
            cam_color_frame->data_size = datasize;
            cam_color_frame->width = col_fra.get_width();
            cam_color_frame->height = col_fra.get_height();
            cam_color_frame->stride = col_fra.get_stride_in_bytes();
            cam_color_frame->time_stamp = col_fra.get_timestamp();
            frames.push_back(cam_color_frame);
            // std::cout << "color datasize:" << cam_color_frame->data_size << std::endl;
        }
        else
        {
            // std::cout << "CamRealsense::get_one_frame 图像帧数据为空！" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 图像帧数据为空！");
            return RtnType::RTN_FAILIURE;
        }
        // std::cout << "get one frame:" << frames.size() << std::endl;
        LOG_INFO("cam_manage",  "get one frame: %d", frames.size());
        // std::cout << "CamRealsense::get_one_frame success" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_one_frame success");
        return RtnType::RTN_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        // std::cerr << "RealSense error: " << e.what() << std::endl;
        LOG_ERROR("cam_manage",  "RealSense error: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CamRealsense::get_one_frame(cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud)
{
    RtnType rtn = RtnType::RTN_FAILIURE;
    // std::cout << "CamRealsense::get_one_frame_color:start" << std::endl;
    LOG_INFO("cam_manage",  "CamRealsense::get_one_frame_color:start");
    rs2::frameset rs_frames = pipe_.wait_for_frames();
    if (rs_frames.size() == 0)
    {
        // std::cout << "CamRealsense::get_one_frame_color: 帧数据为空" << std::endl;
        LOG_WARN("cam_manage",  "CamRealsense::get_one_frame_color: 帧数据为空");
        return RtnType::RTN_FAILIURE;
    }
    if (!cam_para.enable_cloud_stream && !cam_para.enable_color_stream && !cam_para.enable_depth_stream && !cam_para.enable_ir_stream)
    {
        // std::cout << "CamRealsense::get_one_frame 未开启任何流" << std::endl;
        LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 未开启任何流");
        return RtnType::RTN_FAILIURE;
    }

    if (!rs_frames.get_depth_frame() || !rs_frames.get_color_frame() || !rs_frames.get_infrared_frame())
    {
        // std::cout << "CamRealsense::get_one_frame 帧数据为空" << std::endl;
        LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 帧数据为空");
        return RtnType::RTN_FAILIURE;
    }
    // 对齐深度帧到彩色帧（确保空间同步）
    // auto aligned_frames = align_to_color.process(rs_frames);

    rs2::video_frame clr_fra = rs_frames.get_color_frame();
    if (cam_para.enable_color_stream && clr_fra.get_data_size() > 0)
    {

        int width = clr_fra.get_width();
        int height = clr_fra.get_height();
        int stride = clr_fra.get_stride_in_bytes();
        int size = clr_fra.get_data_size();
        int channels = clr_fra.get_bytes_per_pixel();
        int type = CV_8UC3;
        if (channels == 1)
            type = CV_8UC1;
        else if (channels == 2)
            type = CV_8UC2;
        else if (channels == 3)
            type = CV_8UC3;
        else if (channels == 4)
            type = CV_8UC4;
        if (img_color == nullptr)
        {
            img_color = new cv::Mat();
        }
        *img_color = cv::Mat(height, width, type, (void *)clr_fra.get_data(), stride);
    }

    rs2::depth_frame dep_fra = rs_frames.get_depth_frame();
    if ((cam_para.enable_depth_stream || cam_para.enable_cloud_stream) && dep_fra.get_data_size() > 0)
    {
        int width = dep_fra.get_width();
        int height = dep_fra.get_height();

        // 深度图获取
        if (cam_para.enable_depth_stream)
        {
            if (img_depth == nullptr)
            {
                img_depth = new cv::Mat();
            }
            *img_depth = cv::Mat(cv::Size(width, height), CV_16UC1,
                                 (void *)dep_fra.get_data(), cv::Mat::AUTO_STEP);
            cv::normalize(*img_depth, *img_depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(*img_depth, *img_depth, cv::COLORMAP_JET);
        }

        // 点云数据
        if (cam_para.enable_cloud_stream)
        {
            if (cloud == nullptr)
            {
                cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            }
            // 使用 RealSense 的点云转换功能（推荐方式）
            rs2::pointcloud rs_pc;
            rs2::points points = rs_pc.calculate(dep_fra);
            auto vertices = points.get_vertices();
            cloud->width = width;
            cloud->height = height;
            cloud->is_dense = false;
            cloud->points.resize(width * height);
            for (int i = 0; i < points.size(); i++)
            {
                if (vertices[i].z > 0)
                { // 过滤无效点
                    cloud->points[i].x = vertices[i].x;
                    cloud->points[i].y = vertices[i].y;
                    cloud->points[i].z = vertices[i].z;
                }
                else
                {
                    // 无效点设为 NaN
                    cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
    // std::cout << "CamRealsense::get_one_frame success" << std::endl;
    LOG_INFO("cam_manage",  "CamRealsense::get_one_frame success");
    rtn = RtnType::RTN_SUCCESS;
    return rtn;
}
RtnType CamRealsense::get_one_frame_color(cv::Mat *&img)
{
    RtnType rtn = RtnType::RTN_FAILIURE;
    try
    {
        if (!cam_para.enable_color_stream)
        { // 关闭图像流
            // std::cout << "CamRealsense::get_one_frame_color 未开启图像流" << std::endl;
            LOG_WARN("cam_manage",  "CamRealsense::get_one_frame_color 未开启图像流");
            return RtnType::RTN_FAILIURE;
        }

        rs2::frameset rs_frames = pipe_.wait_for_frames();
        if (rs_frames.size() == 0)
        {
            return RtnType::RTN_FAILIURE;
        }
        rs2::video_frame clr_fra = rs_frames.get_color_frame();
        if (clr_fra.get_data_size() > 0)
        {
            int width = clr_fra.get_width();
            int height = clr_fra.get_height();
            int stride = clr_fra.get_stride_in_bytes();
            int size = clr_fra.get_data_size();
            int channels = clr_fra.get_bytes_per_pixel();
            int type = CV_8UC3;
            if (channels == 1)
                type = CV_8UC1;
            else if (channels == 2)
                type = CV_8UC2;
            else if (channels == 3)
                type = CV_8UC3;
            else if (channels == 4)
                type = CV_8UC4;
            if (img == nullptr)
            {
                img = new cv::Mat();
            }

            *img = cv::Mat(height, width, type, (void *)clr_fra.get_data(), stride);
            // std::cout << "CamRealsense::get_one_frame_color end" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_one_frame_color end");
            rtn = RtnType::RTN_SUCCESS;
        }
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::get_one_frame_color" << e.what() << '\n';
        LOG_ERROR("cam_manage",  "CamRealsense::get_one_frame_color %s", e.what());
    }
    return rtn;
}
RtnType CamRealsense::get_one_frame_cloud(PointCloudXYZPtr &cloud)
{

    RtnType rtn = RtnType::RTN_FAILIURE;
    if (!cam_para.enable_cloud_stream)
    { // 关闭深度流
        return RtnType::RTN_FAILIURE;
    }
    if (cloud == nullptr)
    {
        cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }
    rs2::frameset rs_frames = pipe_.wait_for_frames();
    if (rs_frames.size() == 0)
    {
        return RtnType::RTN_FAILIURE;
    }

    rs2::depth_frame dep_fra = rs_frames.get_depth_frame();
    if (dep_fra.get_data_size() > 0)
    {
        int width = dep_fra.get_width();
        int height = dep_fra.get_height();

        // 使用 RealSense 的点云转换功能（推荐方式）
        rs2::pointcloud rs_pc;
        rs2::points points = rs_pc.calculate(dep_fra);
        auto vertices = points.get_vertices();

        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;
        cloud->points.resize(width * height);

        // 正确转换点云数据
        for (int i = 0; i < points.size(); i++)
        {
            if (vertices[i].z > 0)
            { // 过滤无效点
                cloud->points[i].x = vertices[i].x;
                cloud->points[i].y = vertices[i].y;
                cloud->points[i].z = vertices[i].z;
            }
            else
            {
                // 无效点设为 NaN
                cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
                cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
                cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
        // std::cout << "CamRealsense::get_one_frame_cloud:end:" << std::endl;
        LOG_INFO("cam_manage",  "CamRealsense::get_one_frame_cloud:end:");
        rtn = RtnType::RTN_SUCCESS;
    }
    return rtn;
}

RtnType CamRealsense::get_one_frame_depth(cv::Mat *&img)
{
    RtnType rtn = RtnType::RTN_FAILIURE;
    try
    {
        if (!cam_para.enable_depth_stream)
        { // 关闭深度流
            // std::cout << "CamOrbbec get_one_frame_depth 深度流关闭" << std::endl;
            LOG_WARN("cam_manage",  "CamOrbbec get_one_frame_depth 深度流关闭");
            return RtnType::RTN_FAILIURE;
        }
        rs2::frameset rs_frames = pipe_.wait_for_frames();
        if (rs_frames.size() == 0)
        {
            return RtnType::RTN_FAILIURE;
        }

        rs2::depth_frame dep_fra = rs_frames.get_depth_frame();
        if (dep_fra.get_data_size() > 0)
        {
            int width = dep_fra.get_width();
            int height = dep_fra.get_height();

            // 使用 RealSense 的点云转换功能（推荐方式）
            rs2::pointcloud rs_pc;
            rs2::points points = rs_pc.calculate(dep_fra);
            auto vertices = points.get_vertices();

            if (img == nullptr)
            {
                img = new cv::Mat();
            }
            *img = cv::Mat(cv::Size(width, height), CV_16UC1,
                           (void *)dep_fra.get_data(), cv::Mat::AUTO_STEP);
            cv::normalize(*img, *img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(*img, *img, cv::COLORMAP_JET);

            // std::cout << "CamRealsense::get_one_frame_depth:end:" << std::endl;
            LOG_INFO("cam_manage",  "CamRealsense::get_one_frame_depth:end:");
            rtn = RtnType::RTN_SUCCESS;
        }
        return rtn;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "CamRealsense::get_one_frame_depth" << e.what() << '\n';
        LOG_ERROR("cam_manage",  "CamRealsense::get_one_frame_depth %s", e.what());
    }
    return RtnType::RTN_FAILIURE;
}