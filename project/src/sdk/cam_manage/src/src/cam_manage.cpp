#include "../include/cam_manage/cam_base.hpp"
#include "../include/cam_manage/cam_orbbec.hpp"
#include "../include/cam_manage/cam_realsense.hpp"
#include "../include/cam_manage/cam_manage.hpp"
#include "log_system/log_macros.hpp"
#include <mutex>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

std::unique_ptr<CameraManager> CameraManager::instance_ = nullptr;
std::mutex CameraManager::instance_mutex_;

CameraManager &CameraManager::get_instance()
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_)
    {
        instance_ = std::unique_ptr<CameraManager>(new CameraManager());
    }
    return *instance_;
}
CameraManager::CameraManager()
{
}

CameraManager::~CameraManager()
{
    std::lock_guard<std::mutex> lock(mutex_);
    cam_all_device.clear();
    cam_all_paras.clear();
}

RtnType CameraManager::get_all_devices(CamDevInfoList &cam_list)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RtnType rtn = RtnType::RTN_FAILIURE;
    cam_list.clear();

    // 枚举Orbbec相机
    CamDevInfoList ob_cam_list;
    rtn = CamOrbbec::get_all_devices(ob_cam_list);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        cam_list.insert(cam_list.end(), ob_cam_list.begin(), ob_cam_list.end());
    }
    //  枚举Realsense相机
    CamDevInfoList rs_cam_list;
    rtn = CamRealsense::get_all_devices(rs_cam_list);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        cam_list.insert(cam_list.end(), rs_cam_list.begin(), rs_cam_list.end());
    }
    this->cam_list = cam_list;

    std::cout << "cam_list.size():" << cam_list.size() << std::endl;
    for (size_t i = 0; i < cam_list.size(); i++)
    {
        std::cout << "cam_list[" << i << "].cam_type:" << (int)cam_list[i].cam_type << std::endl;
        std::cout << "cam_list[" << i << "].device_name:" << cam_list[i].device_name << std::endl;
        std::cout << "cam_list[" << i << "].serial_number:" << cam_list[i].serial_number << std::endl;
        std::cout << "cam_list[" << i << "].physical_port:" << cam_list[i].physical_port << std::endl;
        std::cout << "cam_list[" << i << "].product_id:" << cam_list[i].product_id << std::endl;
    }

    // std::cout << "get_all_devices end" << std::endl;
    return RtnType::RTN_SUCCESS;
}

RtnType CameraManager::init_all_camera(const CamConfigInfo1D &cam_configs)
{
    try 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        RtnType rtn = RtnType::RTN_FAILIURE;
        // std::cout << "init_all_camera start" << cam_configs.size() << std::endl;
        if (cam_configs.size() == 0)
        {
            return RtnType::RTN_FAILIURE;
        }

        for (const auto &config : cam_configs)
        {
            try 
            {
                // std::cout << "init_all_camera for" << config.cam_id << std::endl;
                std::shared_ptr<CamBase> cam_obj = nullptr;
                if (config.cam_type == CamType::CAM_TYPE_OB)
                {
                    cam_obj = std::make_shared<CamOrbbec>();
                }
                else if (config.cam_type == CamType::CAM_TYPE_RS)
                {
                    cam_obj = std::make_shared<CamRealsense>();
                }
                else
                {
                    continue; // 未知相机类型，跳过
                }
                
                rtn = cam_obj->init(config);
                if (rtn != RtnType::RTN_SUCCESS)
                {
                    LOG_ERROR("cam_manage",  "cam_obj init fail");
                    continue; // 初始化相机失败，跳过
                }
                
                rtn = cam_obj->set_cam_config(config);
                rtn = cam_obj->open_cam();
                if (rtn != RtnType::RTN_SUCCESS)
                {//失败后帧率改为0，再连接一次
                    
                    LOG_WARN("cam_manage",  "cam_obj open fail,second connect try...");
                    CamConfigInfo config_new=  config;
                    config_new.depth_para.fps = 0;
                    config_new.color_para.fps = 0;
                    rtn = cam_obj->set_cam_config(config_new);
                    rtn = cam_obj->open_cam();
                }
                if (rtn != RtnType::RTN_SUCCESS)
                {
                    LOG_ERROR("cam_manage",  "cam_obj open fail");
                    continue; // 打开相机失败，跳过
                }
                if (cam_all_device.find(config.cam_id) != cam_all_device.end())
                {
                    //CamID重复
                    LOG_ERROR("cam_manage",  "Cam id 重复，请检查配置文件");
                    return RtnType::RTN_FAILIURE;
                }
                cam_all_device[config.cam_id] = cam_obj;
                rtn = RtnType::RTN_SUCCESS;
            }
            catch (const std::exception &e)
            {
                LOG_ERROR("cam_manage",  "Exception occurred while initializing camera id %d: %s", config.cam_id, e.what());
                continue; // 发生异常，跳过当前相机继续处理下一个
            }
            catch (...)
            {
                LOG_ERROR("cam_manage",  "Unknown exception occurred while initializing camera id %d", config.cam_id);
                continue; // 发生未知异常，跳过当前相机继续处理下一个
            }
        }
        
        return rtn;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage",  "Exception occurred in init_all_camera: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (...)
    {
        LOG_ERROR("cam_manage",  "Unknown exception occurred in init_all_camera");
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CameraManager::get_stream_enable(short cam_id, CamStreamType modu, bool &enable)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_stream_enable(modu, enable);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CameraManager::set_stream_enable(short cam_id, CamStreamType modu, bool enable)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_stream_enable(modu, enable);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CameraManager::get_cam_intrinsics(short cam_id, CamStreamType modu, CamIntrinsics &intrinsics)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_cam_intrinsics(modu, intrinsics);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CameraManager::save_cam_intrinsics(int cam_id, std::string file_path)
{
    try
    {
        // 获取相机对象
        CamBase *cam_obj = get_cam_obj(cam_id);
        if (cam_obj == nullptr)
        {
            LOG_ERROR("cam_manage",  "Camera id %d not found", cam_id);
            return RtnType::RTN_FAILIURE_NOFOUND;
        }

        // 获取相机信息
        CamDevInfo cam_info;
        bool cam_found = false;
        for (const auto &info : cam_list) {
            if ((cam_obj->get_cam_type() == info.cam_type) && 
                (cam_obj->get_serial_number() == info.serial_number)) {
                cam_info = info;
                cam_found = true;
                break;
            }
        }

        // 获取彩色图像和深度图像的内参
        CamIntrinsics color_intrinsics, depth_intrinsics;
        RtnType color_rtn = cam_obj->get_cam_intrinsics(CamStreamType::STREAM_COLOR, color_intrinsics);
        RtnType depth_rtn = cam_obj->get_cam_intrinsics(CamStreamType::STREAM_DEPTH, depth_intrinsics);

        if (color_rtn != RtnType::RTN_SUCCESS && depth_rtn != RtnType::RTN_SUCCESS)
        {
            LOG_ERROR("cam_manage",  "Failed to get intrinsics for camera id %d", cam_id);
            return RtnType::RTN_FAILIURE;
        }

        // 创建JSON对象
        nlohmann::json json_data;
        
        // 添加基本信息
        json_data["description"] = "Camera intrinsics";
        json_data["version"] = "1.0";
        json_data["camera_id"] = cam_id;
        
        // 获取相机类型和名称
        CamType cam_type = cam_obj->get_cam_type();
        std::string cam_type_str = (cam_type == CamType::CAM_TYPE_RS) ? "RealSense" : 
                                  (cam_type == CamType::CAM_TYPE_OB) ? "Orbbec" : "Unknown";
        json_data["camera_type"] = cam_type_str;
        
        // 添加相机型号和序列号
        if (cam_found) {
            json_data["camera_model"] = cam_info.device_name;
            json_data["serial_number"] = cam_info.serial_number;
            json_data["product_id"] = cam_info.product_id;
        } else {
            json_data["camera_model"] = "Unknown";
            json_data["serial_number"] = cam_obj->get_serial_number();
            json_data["product_id"] = "Unknown";
        }
        
        // 添加时间戳
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        json_data["timestamp"] = std::ctime(&time_t);
        
        // 添加彩色图像内参（如果获取成功）
        if (color_rtn == RtnType::RTN_SUCCESS)
        {
            nlohmann::json color_json;
            color_json["resolution"]["width"] = color_intrinsics.width;
            color_json["resolution"]["height"] = color_intrinsics.height;
            
            color_json["intrinsics"]["fx"] = color_intrinsics.fx;
            color_json["intrinsics"]["fy"] = color_intrinsics.fy;
            color_json["intrinsics"]["ppx"] = color_intrinsics.cx;
            color_json["intrinsics"]["ppy"] = color_intrinsics.cy;
            color_json["intrinsics"]["model"] = "pinhole";
            
            color_json["distortion"]["coefficients"] = {
                color_intrinsics.dist_coeffs[0],
                color_intrinsics.dist_coeffs[1],
                color_intrinsics.dist_coeffs[2],
                color_intrinsics.dist_coeffs[3],
                color_intrinsics.dist_coeffs[4]
            };
            color_json["distortion"]["model"] = "Brown-Conrady";
            
            // 相机矩阵
            color_json["camera_matrix"] = {
                {color_intrinsics.fx, 0.0, color_intrinsics.cx},
                {0.0, color_intrinsics.fy, color_intrinsics.cy},
                {0.0, 0.0, 1.0}
            };
            
            json_data["color"] = color_json;
        }
        
        // 添加深度图像内参（如果获取成功）
        if (depth_rtn == RtnType::RTN_SUCCESS)
        {
            nlohmann::json depth_json;
            depth_json["resolution"]["width"] = depth_intrinsics.width;
            depth_json["resolution"]["height"] = depth_intrinsics.height;
            
            depth_json["intrinsics"]["fx"] = depth_intrinsics.fx;
            depth_json["intrinsics"]["fy"] = depth_intrinsics.fy;
            depth_json["intrinsics"]["ppx"] = depth_intrinsics.cx;
            depth_json["intrinsics"]["ppy"] = depth_intrinsics.cy;
            depth_json["intrinsics"]["model"] = "pinhole";
            
            depth_json["distortion"]["coefficients"] = {
                depth_intrinsics.dist_coeffs[0],
                depth_intrinsics.dist_coeffs[1],
                depth_intrinsics.dist_coeffs[2],
                depth_intrinsics.dist_coeffs[3],
                depth_intrinsics.dist_coeffs[4]
            };
            depth_json["distortion"]["model"] = "Brown-Conrady";
            
            // 相机矩阵
            depth_json["camera_matrix"] = {
                {depth_intrinsics.fx, 0.0, depth_intrinsics.cx},
                {0.0, depth_intrinsics.fy, depth_intrinsics.cy},
                {0.0, 0.0, 1.0}
            };
            
            json_data["depth"] = depth_json;
        }
        
        // 写入文件
        std::ofstream file(file_path);
        if (!file.is_open())
        {
            LOG_ERROR("cam_manage",  "Failed to open file %s for writing", file_path.c_str());
            return RtnType::RTN_FAILIURE;
        }
        
        file << json_data.dump(4); // 4是缩进空格数
        file.close();
        
        LOG_INFO("cam_manage",  "Successfully saved camera intrinsics to %s", file_path.c_str());
        return RtnType::RTN_SUCCESS;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage",  "Exception occurred while saving camera intrinsics: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (...)
    {
        LOG_ERROR("cam_manage",  "Unknown exception occurred while saving camera intrinsics");
        return RtnType::RTN_FAILIURE;
    }
}

RtnType CameraManager::save_all_cam_intrinsics(std::string file_path)
{
    try
    {
        RtnType final_result = RtnType::RTN_SUCCESS;
        bool any_success = false;
        
        // 为每个相机调用save_cam_intrinsics
        for (const auto& pair : cam_all_device) {
            int cam_id = pair.first;
            
            // 生成针对该相机的文件名
            std::string cam_file_path = file_path;
            // 在文件名中插入相机ID（在最后一个点之前插入）
            size_t last_dot = cam_file_path.find_last_of('.');
            if (last_dot != std::string::npos) {
                cam_file_path = cam_file_path.substr(0, last_dot) + "_cam" + std::to_string(cam_id) + cam_file_path.substr(last_dot);
            } else {
                // 如果没有扩展名，则直接添加
                cam_file_path = cam_file_path + "_cam" + std::to_string(cam_id) + ".json";
            }
            
            // 调用save_cam_intrinsics保存该相机的内参
            RtnType result = save_cam_intrinsics(cam_id, cam_file_path);
            if (result == RtnType::RTN_SUCCESS) {
                any_success = true;
            } else {
                final_result = result;
                LOG_WARN("cam_manage",  
                            "Failed to save intrinsics for camera id %d to %s", cam_id, cam_file_path.c_str());
            }
        }
        
        if (any_success) {
            LOG_INFO("cam_manage",  
                        "Successfully saved camera intrinsics for at least one camera");
            return RtnType::RTN_SUCCESS;
        } else {
            LOG_ERROR("cam_manage",  
                         "Failed to save intrinsics for all cameras");
            return final_result;
        }
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("cam_manage",  
                     "Exception occurred while saving all camera intrinsics: %s", e.what());
        return RtnType::RTN_FAILIURE;
    }
    catch (...)
    {
        LOG_ERROR("cam_manage",  
                     "Unknown exception occurred while saving all camera intrinsics");
        return RtnType::RTN_FAILIURE;
    }
}

// 获取相机变量
CamType CameraManager::get_cam_type(short cam_id)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_cam_type();
    }
    else
    {
        return CamType::CAM_TYPE_NONE;
    }
}

CamInterfaceType CameraManager::get_cam_interface_type(short cam_id)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_cam_interface_type();
    }
    else
    {
        return CamInterfaceType::INF_TYPE_NONE;
    }
}

RtnType CameraManager::get_cam_roi_fps_list(short cam_id, std::map<std::string, CamRoiFpsList> &cam_roi_fps_list)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_cam_roi_fps_list(cam_roi_fps_list);
    }
    else
    {
        LOG_ERROR("cam_manage",  "cam_id %d not found", cam_id);
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

// 曝光
RtnType CameraManager::get_exposure_range(short cam_id, float &min_exposure, float &max_exposure, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_exposure_range(min_exposure, max_exposure, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_exposure_val(short cam_id, float &exposure, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_exposure_val(exposure, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_exposure_val(short cam_id, float exposure, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_exposure_val(exposure, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

// 帧率
RtnType CameraManager::get_acqu_fps_enable(short cam_id, bool &enable, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_acqu_fps_enable(enable, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_acqu_fps_enable(short cam_id, bool enable, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_acqu_fps_enable(enable, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_fps_range(short cam_id, int &min_fps, int &max_fps, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_fps_range(min_fps, max_fps, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_fps(short cam_id, int &fps, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_fps(fps, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_fps(short cam_id, int fps, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_fps(fps, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

// RtnType CameraManager::get_result_fps(short cam_id, int &fps, CamStreamType modu)
// {
//     CamBase *cam_obj = get_cam_obj(cam_id);
//     if (cam_obj != nullptr)
//     {
//         return cam_obj->get_result_fps(fps, modu);
//     }
//     else
//     {
//         return RtnType::RTN_FAILIURE_NOFOUND;
//     }
// }

// 增益
RtnType CameraManager::get_gain_range(short cam_id, float &min_gain, float &max_gain, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_gain_range(min_gain, max_gain, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_gain_val(short cam_id, float &gain, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_gain_val(gain, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_gain_val(short cam_id, float gain, CamStreamType modu)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_gain_val(gain, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

// 亮度
RtnType CameraManager::get_brightness_range(short cam_id, float &min_bright, float &max_bright)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_brightness_range(min_bright, max_bright);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_brightness(short cam_id, float &bright)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_brightness(bright);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_brightness(short cam_id, float bright)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_brightness(bright);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

// Gamma
RtnType CameraManager::get_gamma_range(short cam_id, float &min_gamma, float &max_gamma)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_gamma_range(min_gamma, max_gamma);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_gamma(short cam_id, float &gamma)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_gamma(gamma);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::set_gamma(short cam_id, float gamma)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->set_gamma(gamma);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

std::string CameraManager::get_serial_number(short cam_id)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_serial_number();
    }
    else
    {
        return "";
    }
}

CamBase *CameraManager::get_cam_obj(short cam_id)
{
    auto it = cam_all_device.find(cam_id);
    if (it != cam_all_device.end())
        return it->second.get();
    else
        return nullptr;
}

RtnType CameraManager::get_one_frame(short cam_id, CamFramelist &frames)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_one_frame(frames);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

RtnType CameraManager::get_one_frame_color(short cam_id, cv::Mat *&img)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_one_frame_color(img);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_one_frame_depth(short cam_id, cv::Mat *&img)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_one_frame_depth(img);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}
RtnType CameraManager::get_one_frame_cloud(short cam_id, PointCloudXYZPtr &cloud)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_one_frame_cloud(cloud);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}

RtnType CameraManager::get_one_frame(short cam_id, cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud)
{
    CamBase *cam_obj = get_cam_obj(cam_id);
    if (cam_obj != nullptr)
    {
        return cam_obj->get_one_frame(img_color, img_depth, cloud);
    }
    else
    {
        return RtnType::RTN_FAILIURE_NOFOUND;
    }
}