#include "../include/cam_manage/cam_orbbec.hpp"
#include <iostream>
// #include <libobsensor/hpp/Utils.hpp>

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Pipeline.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Sensor.hpp>
#include <libobsensor/hpp/Device.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

// 添加系统相关头文件
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>
#include <fstream>
#include <sstream>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = std::shared_ptr<PointCloudXYZ>;
using PointCloudXYZConstPtr = std::shared_ptr<const PointCloudXYZ>;

std::vector<CamDevInfo> CamOrbbec::cam_info_list_;
std::shared_ptr<ob::DeviceList> CamOrbbec::device_list_;
std::shared_ptr<ob::Context> CamOrbbec::context_;

CamOrbbec::CamOrbbec()
{
     // context_ 现在是静态变量，在get_all_devices中初始化
     device_ = nullptr;
     pipeline_ = nullptr;
     config_ = std::make_shared<ob::Config>();
     // 参数初始化
     cam_para.depth_stream_para.width = 640;
     cam_para.depth_stream_para.height = 480;
     cam_para.depth_stream_para.fps = 30;
     cam_para.color_stream_para.width = 640;
     cam_para.color_stream_para.height = 480;
     cam_para.color_stream_para.fps = 30;
}

CamOrbbec::~CamOrbbec()
{
     close_cam();
}

// 检测并杀死占用指定设备的进程
void CamOrbbec::killOccupyingProcesses(const std::string &serial_number)
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
bool CamOrbbec::isProcessUsingDevice(int pid, const std::string &serial_number)
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
RtnType CamOrbbec::get_all_devices(std::vector<CamDevInfo> &cam_info_list)
{
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec::get_all_devices start");

          // 如果已经有值则不再遍历
          if (!cam_info_list_.empty())
          {
               cam_info_list = cam_info_list_;
               LOG_INFO("cam_manage",  "CamOrbbec::get_all_devices using cached data");
               return RtnType::RTN_SUCCESS;
          }

          cam_info_list_.clear();
          // 初始化静态context_对象（如果尚未初始化）
          if (!context_)
          {
               context_ = std::make_shared<ob::Context>();
          }
          device_list_ = context_->queryDeviceList();
          int num = device_list_->getCount();
          if (num == 0)
          {
               LOG_WARN("cam_manage",  "ob device num is 0");
               return RtnType::RTN_FAILIURE;
          }

          for (size_t i = 0; i < num; i++)
          {
               auto cam_obj = device_list_->getDevice(i);
               CamDevInfo info;
               info.cam_type = CamType::CAM_TYPE_OB;
               auto dev_info = cam_obj->getDeviceInfo();
               info.device_name = dev_info->getName();
               info.serial_number = dev_info->getSerialNumber();
               info.product_id = std::to_string(dev_info->getPid());
               info.firmware_version = dev_info->getFirmwareVersion();

               cam_info_list_.push_back(info);
          }

          cam_info_list = cam_info_list_;
          LOG_INFO("cam_manage",  "CamOrbbec::get_all_devices success");

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::init(const CamConfigInfo &config)
{
     try
     {
          if (config.cam_type != CamType::CAM_TYPE_OB)
          {
               LOG_ERROR("cam_manage",  "%s配置文件类型不匹配", __func__);
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
                    device_ = device_list_->getDevice(i);
                    is_match_cam = true;
                    break;
               }
               else if (i == config.cam_index)
               {
                    cam_info_ = cam_info_list_[i];
                    device_ = device_list_->getDevice(i);
                    is_match_cam = true;
                    break;
               }
          }
          if (is_match_cam)
          {
               LOG_INFO("cam_manage",  "CamOrbbec::init 初始化成功");
               return RtnType::RTN_SUCCESS;
          }
          else
          { // 配置文件未匹配到对应相机
               LOG_WARN("cam_manage",  "CamOrbbec::init 未匹配到对应相机");
               return RtnType::RTN_FAILIURE_NOFOUND;
          }
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::init %s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::open_cam()
{
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec::open_cam start");
          if (!device_.get())
          {
               LOG_WARN("cam_manage",  "CamOrbbec::open_cam 无匹配相机");
               return RtnType::RTN_FAILIURE_NONE;
          }
          if (!cam_para.enable_depth_stream && !cam_para.enable_color_stream && !cam_para.enable_ir_stream && !cam_para.enable_cloud_stream)
          {
               LOG_WARN("cam_manage",  "CamOrbbec::open_cam 未开启任何流,请重新设置");
               return RtnType::RTN_FAILIURE_NONE;
          }

          // 检查设备是否被占用，如果被占用则杀死对应进程
          if (device_.get())
          {
               auto dev_info = device_->getDeviceInfo();
               if (dev_info)
               {
                    std::string serial_number = dev_info->getSerialNumber();
                    LOG_INFO("cam_manage",  "Checking if device %s is occupied", serial_number.c_str());

                    // 杀死占用设备的进程
                    killOccupyingProcesses(serial_number);
               }
          }

          pipeline_ = std::make_shared<ob::Pipeline>(device_);
          // 获取当前相机适合的roi和fps
          std::map<std::string, CamRoiFpsList> cam_roi_fps_list;
          get_cam_roi_fps_list(cam_roi_fps_list);

          if (cam_para.enable_depth_stream || cam_para.enable_cloud_stream)
          { // 深度图和点云
               uint32_t fps_dep = cam_para.depth_stream_para.fps;
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
                    LOG_INFO("cam_manage", 
                                 "Using closest depth configuration: %dx%d@%d FPS (requested: %dx%d@%d FPS)",
                                 width_dep, height_dep, fps_dep,
                                 cam_para.depth_stream_para.width, cam_para.depth_stream_para.height, cam_para.depth_stream_para.fps);
               }
               else
               {
                    LOG_WARN("cam_manage", 
                                 "Failed to find closest depth configuration, using original settings");
               }

               if (fps_dep < 1)
                    fps_dep = OB_FPS_ANY;
               LOG_INFO("cam_manage",  "CamOrbbec::open_cam::fps_depth:%d width:%d height:%d", fps_dep, width_dep, height_dep);
               // config_->enableVideoStream(OB_SENSOR_DEPTH);
               config_->enableVideoStream(OB_STREAM_DEPTH, width_dep, height_dep, (short)fps_dep, OB_FORMAT_UNKNOWN);
          }
          else
          {
               LOG_INFO("cam_manage",  "CamOrbbec::open_cam::深度流/点云流关闭 width:%d height:%d", cam_para.depth_stream_para.width, cam_para.depth_stream_para.height);

               config_->disableStream(OB_STREAM_DEPTH);
          }
          if (cam_para.enable_color_stream)
          {
               uint32_t fps_clr = cam_para.color_stream_para.fps;
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
                    LOG_INFO("cam_manage", 
                                 "Using closest color configuration: %dx%d@%d FPS (requested: %dx%d@%d FPS)",
                                 width_clr, height_clr, fps_clr,
                                 cam_para.color_stream_para.width, cam_para.color_stream_para.height, cam_para.color_stream_para.fps);
               }
               else
               {
                    LOG_WARN("cam_manage", 
                                 "Failed to find closest color configuration, using original settings");
               }

               if (fps_clr < 1)
                    fps_clr = OB_FPS_ANY;
               LOG_INFO("cam_manage",  "CamOrbbec::open_cam::fps_color:%d width:%d height:%d", fps_clr, width_clr, height_clr);
               // config_->enableVideoStream(OB_SENSOR_COLOR);
               config_->enableVideoStream(OB_STREAM_COLOR, width_clr, height_clr, (short)fps_clr, OB_FORMAT_BGR);
          }
          else
          {
               LOG_INFO("cam_manage",  "CamOrbbec::open_cam::图像流关闭 width:%d height:%d", cam_para.color_stream_para.width, cam_para.color_stream_para.height);

               config_->disableStream(OB_STREAM_COLOR);
          }
          if (cam_para.enable_color_stream && (cam_para.enable_depth_stream || cam_para.enable_cloud_stream))
          {
               config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
          }

          // pipeline_->enableFrameSync();//异步
          pipeline_->disableFrameSync(); // 同步

          pipeline_->start(config_);
          LOG_INFO("cam_manage",  "打开相机成功：serial num:%s", cam_info_.serial_number.c_str());
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::open_cam Error: %s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::close_cam()
{
     try
     {
          if (is_cam_start())
          {
               pipeline_->stop();
          }
          return RtnType::RTN_SUCCESS;
     }
     catch (...)
     {
     }
     return RtnType::RTN_FAILIURE;
}
RtnType CamOrbbec::init_defult_para()
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::init_special_para()
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::get_stream_enable(CamStreamType modu, bool &enable)
{
     try
     {
          // 检查pipeline是否已初始化
          if (!pipeline_)
          {
               LOG_ERROR("cam_manage",  "Pipeline not initialized");
               enable = false;
               return RtnType::RTN_SUCCESS;
          }

          // 获取当前配置
          auto config = pipeline_->getConfig();
          if (!config)
          {
               LOG_ERROR("cam_manage",  "Failed to get pipeline config");
               enable = false;
               return RtnType::RTN_SUCCESS;
          }

          // 获取已启用的流配置列表
          std::shared_ptr<ob::StreamProfileList> enabledProfiles = nullptr;
          try {
               enabledProfiles = config->getEnabledStreamProfileList();
          } catch (const std::exception& e) {
               LOG_WARN("cam_manage",  "Failed to get enabled stream profiles: %s", e.what());
               enable = false;
               return RtnType::RTN_SUCCESS;
          }
          
          if (!enabledProfiles)
          {
               LOG_WARN("cam_manage",  "Enabled stream profiles is null");
               enable = false;
               return RtnType::RTN_SUCCESS;
          }

          // 检查指定类型的流是否在启用列表中
          enable = false;
          uint32_t profileCount = 0;
          try {
               profileCount = enabledProfiles->count();
          } catch (const std::exception& e) {
               LOG_WARN("cam_manage",  "Failed to get profile count: %s", e.what());
               enable = false;
               return RtnType::RTN_SUCCESS;
          }
          
          for (uint32_t i = 0; i < profileCount; i++)
          {
               std::shared_ptr<ob::StreamProfile> profile = nullptr;
               try {
                    profile = enabledProfiles->getProfile(i);
               } catch (const std::exception& e) {
                    LOG_WARN("cam_manage",  "Failed to get profile %d: %s", i, e.what());
                    continue;
               }
               
               if (!profile)
                    continue;

               OBStreamType streamType = OB_STREAM_UNKNOWN;
               try {
                    streamType = profile->type();
               } catch (const std::exception& e) {
                    LOG_WARN("cam_manage",  "Failed to get stream type for profile %d: %s", i, e.what());
                    continue;
               }
               
               switch (modu)
               {
               case CamStreamType::STREAM_DEPTH:
                    if (streamType == OB_STREAM_DEPTH)
                    {
                         enable = true;
                         break;
                    }
                    break;
               case CamStreamType::STREAM_COLOR:
                    if (streamType == OB_STREAM_COLOR)
                    {
                         enable = true;
                         break;
                    }
                    break;
               case CamStreamType::STREAM_IR:
                    if (streamType == OB_STREAM_IR)
                    {
                         enable = true;
                         break;
                    }
                    break;
               default:
                    LOG_WARN("cam_manage",  "Invalid stream type: %d", static_cast<int>(modu));
                    enable = false;
                    return RtnType::RTN_SUCCESS;
               }

               if (enable)
                    break;
          }

          LOG_INFO("cam_manage",  "Stream %d enable status: %s", static_cast<int>(modu), enable ? "true" : "false");
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_WARN("cam_manage",  "Exception in get_stream_enable: %s", e.what());
          enable = false;
          return RtnType::RTN_SUCCESS;
     }
     catch (...)
     {
          LOG_WARN("cam_manage",  "Unknown exception in get_stream_enable");
          enable = false;
          return RtnType::RTN_SUCCESS;
     }
}
RtnType CamOrbbec::set_stream_enable(CamStreamType modu, bool enable)
{
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec::set_stream_enable start");

          if (modu == CamStreamType::STREAM_DEPTH)
          {
               if (enable)
               {
                    config_->enableVideoStream(OB_STREAM_DEPTH, cam_para.depth_stream_para.width, cam_para.depth_stream_para.height, cam_para.color_stream_para.fps, OB_FORMAT_ANY);
               }
               else
               {
                    config_->disableStream(OB_STREAM_DEPTH);
               }
          }
          else if (modu == CamStreamType::STREAM_COLOR)
          {
               if (enable)
               {
                    config_->enableVideoStream(OB_STREAM_COLOR, cam_para.color_stream_para.width, cam_para.color_stream_para.height, cam_para.color_stream_para.fps, OB_FORMAT_RGB);
               }
               else
               {
                    config_->disableStream(OB_STREAM_COLOR);
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
          // pipeline_->stop();
          try {
               pipeline_->start(config_);
          } catch (const std::exception& e) {
               LOG_ERROR("cam_manage",  "Failed to start camera pipeline: %s", e.what());
               return RtnType::RTN_FAILIURE;
          }
          LOG_INFO("cam_manage",  "CamOrbbec::set_stream_enable success");

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }
     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::set_cam_config(const CamConfigInfo &config)
{
     LOG_INFO("cam_manage",  "CamOrbbec::set_cam_config start");

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

     LOG_INFO("cam_manage",  "CamOrbbec::set_cam_config end");

     return rtn;
}

RtnType CamOrbbec::init_pixel_type()
{
     return RtnType::RTN_SUCCESS;
}
CamType CamOrbbec::get_cam_type()
{
     return CamType::CAM_TYPE_OB;
}
CamInterfaceType CamOrbbec::get_cam_interface_type()
{
     return CamInterfaceType::INF_TYPE_USB;
}

std::string CamOrbbec::get_serial_number()
{
     return cam_info_.serial_number;
}

RtnType CamOrbbec::get_cam_roi_fps_list(std::map<std::string, CamRoiFpsList> &cam_roi_fps_list)
{
     try
     {
          LOG_DEBUG("cam_manage",  "CamOrbbec::get_cam_roi_fps_list start");
          // 检查设备是否已初始化
          if (!device_.get())
          {
               LOG_ERROR("cam_manage",  "Device not initialized");
               return RtnType::RTN_FAILIURE;
          }

          // 清空之前的ROI和FPS列表
          cam_roi_fps_list.clear();

          // 获取设备的传感器列表
          auto sensorList = device_->getSensorList();
          if (!sensorList)
          {
               LOG_ERROR("cam_manage",  "Failed to get sensor list");
               return RtnType::RTN_FAILIURE;
          }

          // 遍历所有传感器
          for (size_t i = 0; i < sensorList->getCount(); i++)
          {
               auto sensor = sensorList->getSensor(i);
               if (!sensor)
                    continue;

               // 获取该传感器支持的流配置列表
               auto profileList = sensor->getStreamProfileList();
               if (!profileList)
                    continue;

               // 创建一个用于存储该传感器支持的ROI和FPS列表
               CamRoiFpsList roiFpsList;

               // 遍历所有流配置
               for (size_t j = 0; j < profileList->getCount(); j++)
               {
                    auto profile = profileList->getProfile(j);
                    if (!profile)
                         continue;

                    // 检查是否为视频流配置（ROI和FPS只适用于视频流）
                    if (profile->type() == OB_STREAM_COLOR ||
                        profile->type() == OB_STREAM_DEPTH ||
                        profile->type() == OB_STREAM_IR)
                    {
                         auto videoProfile = profile->as<ob::VideoStreamProfile>();
                         if (videoProfile)
                         {
                              // 创建ROI和FPS信息
                              CamRoiFps roiFps;
                              roiFps.width = videoProfile->getWidth();
                              roiFps.height = videoProfile->getHeight();
                              roiFps.fps = videoProfile->getFps();

                              // 检查是否已存在相同的配置，避免重复
                              bool exists = false;
                              for (const auto &existing : roiFpsList)
                              {
                                   if (existing.width == roiFps.width &&
                                       existing.height == roiFps.height &&
                                       existing.fps == roiFps.fps)
                                   {
                                        exists = true;
                                        break;
                                   }
                              }

                              // 如果不存在，则添加到列表中
                              if (!exists)
                              {
                                   roiFpsList.push_back(roiFps);
                              }
                         }
                    }
               }

               // 根据传感器类型将ROI和FPS列表添加到输出参数中
               std::string sensorType;
               switch (sensor->getType())
               {
               case OB_SENSOR_COLOR:
                    sensorType = "color";
                    break;
               case OB_SENSOR_DEPTH:
                    sensorType = "depth";
                    break;
               case OB_SENSOR_IR:
                    sensorType = "ir";
                    break;
               case OB_SENSOR_IR_LEFT:
                    sensorType = "ir_left";
                    break;
               case OB_SENSOR_IR_RIGHT:
                    sensorType = "ir_right";
                    break;
               default:
                    sensorType = "unknown";
                    break;
               }

               // 将该传感器的ROI和FPS列表添加到输出参数中
               cam_roi_fps_list[sensorType] = roiFpsList;
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
                         LOG_INFO("cam_manage",  "  Resolution: %dx%d, FPS: %d",
                                      roiFps.width, roiFps.height, roiFps.fps);
                    }
               }
          }

          LOG_INFO("cam_manage",  "Successfully retrieved and printed ROI and FPS list");
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "Exception in get_cam_roi_fps_list: %s", e.what());
     }
     catch (...)
     {
          LOG_ERROR("cam_manage",  "Unknown exception in get_cam_roi_fps_list");
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::get_closest_roi_fps(CamStreamType stream_type, int target_width, int target_height, int target_fps, int &closest_width, int &closest_height, int &closest_fps)
{
     try
     {
          LOG_DEBUG("cam_manage",  "CamOrbbec::get_closest_roi_fps start");

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
          closest_fps = best_match.fps;

          LOG_INFO("cam_manage", 
                       "Found closest configuration for %s stream - Target: %dx%d@%d FPS, Closest: %dx%d@%d FPS",
                       sensor_type.c_str(), target_width, target_height, target_fps,
                       closest_width, closest_height, closest_fps);

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "Exception in get_closest_roi_fps: %s", e.what());
     }
     catch (...)
     {
          LOG_ERROR("cam_manage",  "Unknown exception in get_closest_roi_fps");
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::get_cam_para_enable(CamParaType para_type, CamStreamType modu, bool &enable)
{
     try
     {
          // 消除未使用参数警告
          (void)modu;

          if (pipeline_ == nullptr)
               return RtnType::RTN_FAILIURE;
          device_ = pipeline_->getDevice();
          if (device_ == nullptr)
               return RtnType::RTN_FAILIURE;
          if (para_type == CamParaType::PARA_AUTO_EXPOSURE)
          {
               enable = device_->getBoolProperty(OBPropertyID::OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
          }
          else if (para_type == CamParaType::PARA_AUTO_WHITE_BALANCE)
          {
               enable = device_->getBoolProperty(OBPropertyID::OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
          }
          LOG_INFO("cam_manage",  "CamOrbbec::get_cam_para_enable success");

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_cam_para_enable error%s", e.what());
     }
     return RtnType::RTN_FAILIURE;
}
RtnType CamOrbbec::set_cam_para_enable(CamParaType para_type, CamStreamType modu, bool enable)
{
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec::set_cam_para_enable start");
          // 消除未使用参数警告
          (void)modu;

          if (pipeline_ == nullptr)
          {
               LOG_WARN("cam_manage",  "CamOrbbec::set_cam_para_enable pipeline_ is nullptr");
               return RtnType::RTN_FAILIURE;
          }
          device_ = pipeline_->getDevice();
          if (device_ == nullptr)
          {
               LOG_WARN("cam_manage",  "CamOrbbec::set_cam_para_enable device_ is nullptr");
               return RtnType::RTN_FAILIURE;
          }

          if (para_type == CamParaType::PARA_AUTO_EXPOSURE)
          {
               device_->setBoolProperty(OBPropertyID::OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable);
          }
          else if (para_type == CamParaType::PARA_AUTO_WHITE_BALANCE)
          {
               device_->setBoolProperty(OBPropertyID::OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, enable);
          }
          LOG_INFO("cam_manage",  "CamOrbbec::set_cam_para_enable success");
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }
     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::get_cam_para_range(CamParaType para_type, float &min_val, float &max_val, CamStreamType modu)
{
     try
     {
          if (pipeline_ == nullptr)
               return RtnType::RTN_FAILIURE;
          device_ = pipeline_->getDevice();
          if (device_ == nullptr)
               return RtnType::RTN_FAILIURE;

          OBIntPropertyRange range;
          if (para_type == CamParaType::PARA_EXPOSURE)
          {
               range = device_->getIntPropertyRange(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_EXPOSURE_INT : OBPropertyID::OB_PROP_DEPTH_EXPOSURE_INT);
          }
          else if (para_type == CamParaType::PARA_GAIN)
          {
               range = device_->getIntPropertyRange(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_GAIN_INT : OBPropertyID::OB_PROP_DEPTH_GAIN_INT);
          }
          else if (para_type == CamParaType::PARA_GAMMA && modu == CamStreamType::STREAM_COLOR)
          {
               range = device_->getIntPropertyRange(OBPropertyID::OB_PROP_COLOR_GAMMA_INT);
          }
          else if (para_type == CamParaType::PARA_CONTRAST && modu == CamStreamType::STREAM_COLOR)
          {
               range = device_->getIntPropertyRange(OBPropertyID::OB_PROP_COLOR_CONTRAST_INT);
          }
          else if (para_type == CamParaType::PARA_SHARPNESS && modu == CamStreamType::STREAM_COLOR)
          {
               range = device_->getIntPropertyRange(OBPropertyID::OB_PROP_COLOR_SHARPNESS_INT);
          }
          min_val = range.min;
          max_val = range.max;

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_cam_para_range%s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}
RtnType CamOrbbec::set_cam_para(CamParaType para_type, float value, CamStreamType modu)
{
     try
     {
          if (pipeline_ == nullptr)
               return RtnType::RTN_FAILIURE;
          device_ = pipeline_->getDevice();
          if (device_ == nullptr)
               return RtnType::RTN_FAILIURE;

          if (para_type == CamParaType::PARA_EXPOSURE)
          {
               device_->setIntProperty(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_EXPOSURE_INT : OBPropertyID::OB_PROP_DEPTH_EXPOSURE_INT, value);
          }
          else if (para_type == CamParaType::PARA_GAIN)
          {
               device_->setIntProperty(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_GAIN_INT : OBPropertyID::OB_PROP_DEPTH_GAIN_INT, value);
          }
          else if (para_type == CamParaType::PARA_GAMMA && modu == CamStreamType::STREAM_COLOR)
          {
               device_->setIntProperty(OBPropertyID::OB_PROP_COLOR_GAMMA_INT, value);
          }
          else if (para_type == CamParaType::PARA_CONTRAST && modu == CamStreamType::STREAM_COLOR)
          {
               device_->setIntProperty(OBPropertyID::OB_PROP_COLOR_CONTRAST_INT, value);
          }
          else if (para_type == CamParaType::PARA_SHARPNESS && modu == CamStreamType::STREAM_COLOR)
          {
               device_->setIntProperty(OBPropertyID::OB_PROP_COLOR_SHARPNESS_INT, value);
          }
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::set_cam_para%s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}
RtnType CamOrbbec::get_cam_para(CamParaType para_type, float &value, CamStreamType modu)
{
     try
     {
          if (pipeline_ == nullptr)
               return RtnType::RTN_FAILIURE;
          device_ = pipeline_->getDevice();
          if (device_ == nullptr)
               return RtnType::RTN_FAILIURE;

          if (para_type == CamParaType::PARA_EXPOSURE)
          {
               value = device_->getIntProperty(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_EXPOSURE_INT : OBPropertyID::OB_PROP_DEPTH_EXPOSURE_INT);
          }
          else if (para_type == CamParaType::PARA_GAIN)
          {
               value = device_->getIntProperty(modu == CamStreamType::STREAM_COLOR ? OBPropertyID::OB_PROP_COLOR_GAIN_INT : OBPropertyID::OB_PROP_DEPTH_GAIN_INT);
          }
          else if (para_type == CamParaType::PARA_GAMMA && modu == CamStreamType::STREAM_COLOR)
          {
               value = device_->getIntProperty(OBPropertyID::OB_PROP_COLOR_GAMMA_INT);
          }
          else if (para_type == CamParaType::PARA_CONTRAST && modu == CamStreamType::STREAM_COLOR)
          {
               value = device_->getIntProperty(OBPropertyID::OB_PROP_COLOR_CONTRAST_INT);
          }
          else if (para_type == CamParaType::PARA_SHARPNESS && modu == CamStreamType::STREAM_COLOR)
          {
               value = device_->getIntProperty(OBPropertyID::OB_PROP_COLOR_SHARPNESS_INT);
          }

          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }

     return RtnType::RTN_FAILIURE;
}

RtnType CamOrbbec::get_cam_para_range(CamParaType para_type, int &min_val, int &max_val, CamStreamType modu)
{
     if (pipeline_ == nullptr)
          return RtnType::RTN_FAILIURE;
     device_ = pipeline_->getDevice();
     if (device_ == nullptr)
          return RtnType::RTN_FAILIURE;

     // 目前只有FPS是整数型参数，开启流时设置

     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::set_cam_para(CamParaType para_type, int value, CamStreamType modu)
{
     if (pipeline_ == nullptr)
          return RtnType::RTN_FAILIURE;
     device_ = pipeline_->getDevice();
     if (device_ == nullptr)
          return RtnType::RTN_FAILIURE;

     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::get_cam_para(CamParaType para_type, int &value, CamStreamType modu)
{
     if (pipeline_ == nullptr)
          return RtnType::RTN_FAILIURE;
     device_ = pipeline_->getDevice();
     if (device_ == nullptr)
          return RtnType::RTN_FAILIURE;

     return RtnType::RTN_SUCCESS;
}

RtnType CamOrbbec::is_cam_start(bool &is_start)
{
     try
     {
          if (pipeline_ && pipeline_->getDevice())
          {
               pipeline_->getConfig();
               is_start = true;
          }
     }
     catch (const std::exception &e)
     {
     }
     is_start = false;
}

bool CamOrbbec::is_cam_start()
{
     bool is_start = false;
     is_cam_start(is_start);
     return is_start;
}

RtnType CamOrbbec::get_cam_intrinsics(CamStreamType modu, CamIntrinsics &intrinsics)
{
     ob_error *error = NULL;
     try
     {
          if (!device_)
          {
               return RtnType::RTN_FAILIURE;
          }
          LOG_INFO("cam_manage",  "get_cam_intrinsics start");
          std::shared_ptr<ob::FrameSet> frameset = nullptr;
          for (size_t i = 0; i < 1; i++)
          {
               frameset = pipeline_->waitForFrameset(1000);
          }
          LOG_INFO("cam_manage",  "get_cam_intrinsics grab images");
          std::shared_ptr<ob::StreamProfileList> profiles = nullptr;
          OBCameraIntrinsic intr;
          OBCameraDistortion dist;
          if (modu == CamStreamType::STREAM_DEPTH)
          {
               intr = pipeline_->getCameraParam().depthIntrinsic;
               dist = pipeline_->getCameraParam().depthDistortion;
          }
          else if (modu == CamStreamType::STREAM_COLOR)
          {
               intr = pipeline_->getCameraParam().rgbIntrinsic;
               dist = pipeline_->getCameraParam().rgbDistortion;
          }
          else
          {
               return RtnType::RTN_FAILIURE;
          }

          intrinsics.fx = intr.fx;
          intrinsics.fy = intr.fy;
          intrinsics.cx = intr.cx;
          intrinsics.cy = intr.cy;
          intrinsics.width = intr.width;
          intrinsics.height = intr.height;

          intrinsics.dist_coeffs[0] = dist.k1;
          intrinsics.dist_coeffs[1] = dist.k2;
          intrinsics.dist_coeffs[2] = dist.p1;
          intrinsics.dist_coeffs[3] = dist.p2;
          intrinsics.dist_coeffs[4] = dist.k3;

          return RtnType::RTN_SUCCESS;
     }
     catch (...)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_cam_intrinsics error");
          return RtnType::RTN_FAILIURE;
     }
}
RtnType CamOrbbec::get_pixel_format(PixelFormat &pixel_format)
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::set_pixel_format(PixelFormat pixel_format)
{
     return RtnType::RTN_SUCCESS;
}

RtnType CamOrbbec::set_white_balance_mode(WhiteBalenceMode mode)
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::get_white_balance_mode(WhiteBalenceMode &mode)
{
     return RtnType::RTN_SUCCESS;
}

RtnType CamOrbbec::get_balance_ratio_range(int &balan_min, int &balan_max)
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::set_balance_ratio(int balan_ratio)
{
     return RtnType::RTN_SUCCESS;
}
RtnType CamOrbbec::get_balance_ratio(int &balan_ratio)
{
     return RtnType::RTN_SUCCESS;
}

RtnType CamOrbbec::get_one_frame(CamFramelist &frames)
{
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame start");
          frames.clear();
          if (!cam_para.enable_cloud_stream && !cam_para.enable_color_stream && !cam_para.enable_depth_stream && !cam_para.enable_ir_stream)
          {
               LOG_WARN("cam_manage",  "CamOrbbec::get_one_frame 未开启任何流");
          }
          auto pointCloud = std::make_shared<ob::PointCloudFilter>();
          auto align = std::make_shared<ob::Align>(OB_STREAM_COLOR); // align depth frame to color frame
          std::shared_ptr<ob::FrameSet> frameset = nullptr;
          frameset = pipeline_->waitForFrameset(1000);
          if (frameset == nullptr)
          {
               LOG_ERROR("cam_manage",  "CamOrbbec::get_one_frame 帧数据为空！");
               return RtnType::RTN_FAILIURE;
          }
          auto alignedFrameset = align->process(frameset);

          auto col_fra = frameset->getFrame(OBFrameType::OB_FRAME_COLOR);
          if (col_fra->getDataSize() > 0)
          {
               if (!cam_para.enable_color_stream)
               {
                    LOG_WARN("cam_manage",  "CamRealsense::get_one_frame 图像流未开启！");
               }
               CamFrameData *cam_color_frame = new CamFrameData();
               cam_color_frame->frame_type = CamStreamType::STREAM_COLOR;
               int datasize = col_fra->getDataSize();
               cam_color_frame->data_buffer = new uint8_t[datasize];
               memcpy((void *)cam_color_frame->data_buffer, (void *)col_fra->getData(), datasize);
               cam_color_frame->data_size = datasize;
               cam_color_frame->time_stamp = col_fra->getTimeStampUs();
               frames.push_back(cam_color_frame);
               LOG_INFO("cam_manage",  "color datasize:%d", cam_color_frame->data_size);
          }

          pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
          std::shared_ptr<ob::Frame> dep_fra = pointCloud->process(alignedFrameset);
          if (dep_fra->getDataSize() > 0)
          {
               if (!cam_para.enable_depth_stream && !cam_para.enable_cloud_stream)
               {
                    LOG_WARN("cam_manage",  "CamOrbbec::get_one_frame 深度流未开启！");
               }
               CamFrameData *cam_depth_frame = new CamFrameData();
               cam_depth_frame->frame_type = CamStreamType::STREAM_DEPTH;
               int datasize = dep_fra->getDataSize();
               cam_depth_frame->data_buffer = new uint8_t[datasize];
               memcpy((void *)cam_depth_frame->data_buffer, (void *)dep_fra->getData(), datasize);
               cam_depth_frame->data_size = datasize;
               cam_depth_frame->time_stamp = dep_fra->getTimeStampUs();
               frames.push_back(cam_depth_frame);
               // LOG_INFO("cam_manage",  "depth datasize:%d", cam_depth_frame->data_size);
          }
          LOG_INFO("cam_manage",  "get one frame:%d", frames.size());
          LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame success");
          return RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }
     return RtnType::RTN_SUCCESS;
}

RtnType CamOrbbec::get_one_frame(cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud)
{
     RtnType rtn = RtnType::RTN_FAILIURE;
     try
     {

          LOG_INFO("cam_manage",  "CamOrbbec get_one_frame start");

          std::shared_ptr<ob::FrameSet> frameset = nullptr;
          frameset = pipeline_->waitForFrameset(1000);
          if (frameset == nullptr)
          {
               LOG_ERROR("cam_manage",  "CamOrbbec get_one_frame_color:frameset == nullptr");
               return RtnType::RTN_FAILIURE;
          }
          if (!cam_para.enable_cloud_stream && !cam_para.enable_color_stream && !cam_para.enable_depth_stream && !cam_para.enable_ir_stream)
          {
               LOG_WARN("cam_manage",  "CamOrbbec::get_one_frame 未开启任何流");
               return RtnType::RTN_FAILIURE;
          }

          if (!frameset->getFrame(OBFrameType::OB_FRAME_COLOR) || !frameset->getFrame(OBFrameType::OB_FRAME_DEPTH) || !frameset->getFrame(OBFrameType::OB_FRAME_IR))
          {
               LOG_ERROR("cam_manage",  "CamOrbbec::get_one_frame 帧数据为空");
               return RtnType::RTN_FAILIURE;
          }
          ////////////////彩色图////////////////
          auto clr_fra = frameset->getFrame(OBFrameType::OB_FRAME_COLOR);
          LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_color:get framesDataSize:%d", clr_fra->getDataSize());
          if (cam_para.enable_color_stream && clr_fra->getDataSize() > 0)
          {
               if (img_color == nullptr)
               {
                    img_color = new cv::Mat();
               }
               OBFormat format = clr_fra->getFormat();
               auto intr = pipeline_->getCameraParam().rgbIntrinsic;
               int width = intr.width;
               int height = intr.height;

               *img_color = cv::Mat(height, width, CV_8UC3, clr_fra->getData());

               LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_color:end");
          }
          ////////////////深度图和点云////////////////
          auto dep_fra = frameset->getFrame(OBFrameType::OB_FRAME_DEPTH);
          if ((cam_para.enable_depth_stream || cam_para.enable_cloud_stream) && dep_fra->getDataSize() > 0)
          {

               auto intr = pipeline_->getCameraParam().rgbIntrinsic;
               int width = intr.width;
               int height = intr.height;

               // 深度图
               if (cam_para.enable_depth_stream)
               {
                    if (img_depth == nullptr)
                    {
                         img_depth = new cv::Mat();
                    }
                    *img_depth = cv::Mat(cv::Size(width, height), CV_16UC1, (void *)dep_fra->data(), cv::Mat::AUTO_STEP);
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
                    uint32_t pointsSize = dep_fra->dataSize() / sizeof(OBPoint);
                    OBPoint *point = (OBPoint *)dep_fra->data();
                    cloud->width = width;
                    cloud->height = height;
                    cloud->points.resize(pointsSize);
                    cloud->is_dense = false;

                    // 点云
                    for (uint32_t i = 0; i < pointsSize; i++)
                    {
                         if (point->z > 0 && !std::isnan(point->z))
                         { // 过滤无效点
                              cloud->points[i].x = point->x;
                              cloud->points[i].y = point->y;
                              cloud->points[i].z = point->z;
                         }
                         else
                         {
                              // 无效点设为 NaN
                              cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
                              cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
                              cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
                         }

                         point++;
                    }
               }
          }
          LOG_INFO("cam_manage",  "CamOrbbec get_one_frame success");
          rtn = RtnType::RTN_SUCCESS;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "%s", e.what());
     }
     return rtn;
}
RtnType CamOrbbec::get_one_frame_color(cv::Mat *&img)
{
     RtnType rtn = RtnType::RTN_FAILIURE;
     try
     {
          LOG_INFO("cam_manage",  "CamOrbbec get_one_frame_color");
          if (!cam_para.enable_color_stream)
          { // 关闭图像流
               LOG_WARN("cam_manage",  "CamOrbbec::get_one_frame_color 未开启图像流");
               return RtnType::RTN_FAILIURE;
          }
          std::shared_ptr<ob::FrameSet> frameset = nullptr;
          frameset = pipeline_->waitForFrameset(1000);
          if (frameset == nullptr)
          {
               LOG_ERROR("cam_manage",  "CamOrbbec get_one_frame_color:frameset == nullptr");
               return RtnType::RTN_FAILIURE;
          }
          auto clr_fra = frameset->getFrame(OBFrameType::OB_FRAME_COLOR);
          LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_color:get framesDataSize:%d", clr_fra->getDataSize());
          if (clr_fra->getDataSize() > 0)
          {
               OBFormat format = clr_fra->getFormat();
               auto intr = pipeline_->getCameraParam().rgbIntrinsic;
               int width = intr.width;
               int height = intr.height;
               if (img == nullptr)
               {
                    img = new cv::Mat();
               }
               *img = cv::Mat(height, width, CV_8UC3, clr_fra->getData());
               rtn = RtnType::RTN_SUCCESS;
               LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_color:end");
          }
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_one_frame_color 异常%s", e.what());
     }

     return rtn;
}
RtnType CamOrbbec::get_one_frame_cloud(PointCloudXYZPtr &cloud)
{

     RtnType rtn = RtnType::RTN_FAILIURE;
     LOG_INFO("cam_manage",  "CamOrbbec get_one_frame_cloud start");
     if (!cam_para.enable_cloud_stream)
     { // 关闭深度流
          return RtnType::RTN_FAILIURE;
     }
     if (cloud == nullptr)
     {
          cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
     }

     auto pointCloud = std::make_shared<ob::PointCloudFilter>();
     auto align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
     std::shared_ptr<ob::FrameSet> frameset = nullptr;
     frameset = pipeline_->waitForFrameset(1000);
     auto alignedFrameset = align->process(frameset);
     if (frameset == nullptr)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_one_frame_cloud 帧数据为空");
          return RtnType::RTN_FAILIURE;
     }
     pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
     std::shared_ptr<ob::Frame> dep_fra = pointCloud->process(alignedFrameset);
     if (dep_fra->getDataSize() > 0)
     {
          uint32_t pointsSize = dep_fra->dataSize() / sizeof(OBPoint);
          OBPoint *point = (OBPoint *)dep_fra->data();

          auto intr = pipeline_->getCameraParam().rgbIntrinsic;
          int width = intr.width;
          int height = intr.height;

          cloud->width = width;
          cloud->height = height;
          cloud->points.resize(pointsSize);

          cloud->is_dense = false;

          // std::cout << "get_one_frame_depth:dep_fra datasize:" << pointsSize << std::endl;
          // std::cout << intr.width << " " << intr.height << " " << cloud->points.size() << std::endl;
          for (uint32_t i = 0; i < pointsSize; i++)
          {
               if (point->z > 0 && !std::isnan(point->z))
               { // 过滤无效点
                    cloud->points[i].x = point->x;
                    cloud->points[i].y = point->y;
                    cloud->points[i].z = point->z;
               }
               else
               {
                    // 无效点设为 NaN
                    cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
               }

               point++;
          }

          LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_cloud:end:%d", pointsSize);
          rtn = RtnType::RTN_SUCCESS;
     }
     return rtn;
}

RtnType CamOrbbec::get_one_frame_depth(cv::Mat *&img)
{
     RtnType rtn = RtnType::RTN_FAILIURE;
     try
     {
          if (!cam_para.enable_depth_stream)
          { // 关闭深度流
               return RtnType::RTN_FAILIURE;
          }
          LOG_INFO("cam_manage",  "CamOrbbec get_one_frame_depth");
          if (img == nullptr)
          {
               img = new cv::Mat();
          }
          auto pointCloud = std::make_shared<ob::PointCloudFilter>();
          auto align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
          std::shared_ptr<ob::FrameSet> frameset = nullptr;
          frameset = pipeline_->waitForFrameset(1000);
          auto alignedFrameset = align->process(frameset);
          if (frameset == nullptr)
          {
               return RtnType::RTN_FAILIURE;
          }
          pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
          std::shared_ptr<ob::Frame> dep_fra = pointCloud->process(alignedFrameset);
          if (dep_fra->getDataSize() > 0)
          {
               uint32_t pointsSize = dep_fra->dataSize() / sizeof(OBPoint);
               OBPoint *point = (OBPoint *)dep_fra->data();

               auto intr = pipeline_->getCameraParam().rgbIntrinsic;
               int width = intr.width;
               int height = intr.height;

               *img = cv::Mat(cv::Size(width, height), CV_16UC1, (void *)dep_fra->data(), cv::Mat::AUTO_STEP);
               cv::normalize(*img, *img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
               cv::applyColorMap(*img, *img, cv::COLORMAP_JET);

               rtn = RtnType::RTN_SUCCESS;
               LOG_INFO("cam_manage",  "CamOrbbec::get_one_frame_depth:success:%d", pointsSize);
          }
          return rtn;
     }
     catch (const std::exception &e)
     {
          LOG_ERROR("cam_manage",  "CamOrbbec::get_one_frame_depth%s", e.what());
     }
     return RtnType::RTN_FAILIURE;
}