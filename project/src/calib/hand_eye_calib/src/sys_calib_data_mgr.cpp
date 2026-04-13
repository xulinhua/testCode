#include "hand_eye_calib/sys_calib_data_mgr.hpp"
#include "log_system/log_macros.hpp"
#include "internal_constants.h"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_reflector.hpp"
#include "calib_data_operate.hpp"
#include "data_handler/json_operate.h"
#include "bas_operate/bas_utils.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace handeyecalib;

namespace handeyecalib 
{

// 静态成员初始化
SysCalibMgr* SysCalibMgr::instance_ = nullptr;
std::mutex SysCalibMgr::mutex_;

/**
 * @brief 获取单例实例
 * 使用双重检查锁定模式实现线程安全的单例
 * @return 单例实例引用
 */
SysCalibMgr& SysCalibMgr::getInstance() 
{
    if (instance_ == nullptr) 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr) {
            instance_ = new SysCalibMgr();
        }
    }
    return *instance_;
}

/**
 * @brief 获取指定相机的标定数据
 * @param cam_id 相机ID
 * @return 相机标定信息
 */
CamCalibInfo SysCalibMgr::getCamCalibData(uint8_t cam_id) const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    // 遍历查找匹配的相机ID，而不是直接使用索引
    for (const auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id) {
            return cam_calib_info;
        }
    }
    LOG_WARN("未找到相机%d的标定数据，返回默认值", static_cast<int>(cam_id));
    return CamCalibInfo(); // 返回默认构造的相机标定信息
}

/**
 * @brief 获取所有相机的标定数据列表
 * @return 相机标定数据列表
 */
CamCalibInfoList SysCalibMgr::getCamCalibDataList() const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    return cam_calib_data_list_;
}

/**
 * @brief 检查是否存在指定相机的标定数据
 * @param cam_id 相机ID
 * @return 是否存在
 */
bool SysCalibMgr::hasCamCalibData(uint8_t cam_id) const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    // 遍历查找匹配的相机ID，而不是直接使用索引
    for (const auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id && !cam_calib_info.arm_calib1D.empty()) 
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 获取指定相机下指定机械臂的标定数据
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 机械臂标定信息
 */
ArmCalibInfo SysCalibMgr::getArmCalibData(uint8_t cam_id, uint8_t arm_id) const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    for (const auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id) 
        {
            if (cam_calib_info.hasArmCalibInfo(arm_id)) 
            {
                return cam_calib_info.getArmCalibInfo(arm_id);
            }
            else 
            {
                LOG_WARN("未找到相机%d下机械臂%d的标定数据，返回默认值", static_cast<int>(cam_id), static_cast<int>(arm_id));
                return ArmCalibInfo(); // 返回默认构造的机械臂标定信息
            }
        }
    }
    LOG_WARN("未找到相机%d下机械臂%d的标定数据，返回默认值", static_cast<int>(cam_id), static_cast<int>(arm_id));
    return ArmCalibInfo(); // 返回默认构造的机械臂标定信息
}

/**
 * @brief 检查是否存在指定相机下指定机械臂的标定数据
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 是否存在
 */
bool SysCalibMgr::hasArmCalibData(uint8_t cam_id, uint8_t arm_id) const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    // 遍历查找匹配的相机ID，而不是直接使用索引
    for (const auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id) 
        {
            return cam_calib_info.hasArmCalibInfo(arm_id);
        }
    }
    return false;
}

/**
 * @brief 设置指定相机的标定数据
 * @param cam_id 相机ID
 * @param cam_calib_info 相机标定信息
 */
void SysCalibMgr::setCamCalibData(uint8_t cam_id, const CamCalibInfo& cam_calib_info)
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    bool found = false;// 查找是否已存在相同cam_id的记录
    for (auto it = cam_calib_data_list_.begin(); it != cam_calib_data_list_.end(); ++it) 
    {
        if (it->cam_id == cam_id) 
        {
            LOG_DEBUG("找到相机cam_id= %d 的现有标定数据记录，刷新标定数据", static_cast<int>(cam_id));
            *it = cam_calib_info;
            found = true;
            break;
        }
    }
    
    if (!found) // 如果没有找到相同cam_id的记录，则添加新的记录
    {
        LOG_DEBUG("未找到相机cam_id= %d 的现有标定数据记录，添加新的标定数据记录", static_cast<int>(cam_id));
        cam_calib_data_list_.push_back(cam_calib_info);
        LOG_DEBUG("添加后列表大小: %zu", cam_calib_data_list_.size());
    } else {
        LOG_DEBUG("替换后列表大小保持为: %zu", cam_calib_data_list_.size());
    }
}

/**
 * @brief 重置指定相机的标定数据
 * @param cam_id 相机ID
 */
void SysCalibMgr::resetCamCalibData(uint8_t cam_id) 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    for (auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id) 
        {
            cam_calib_info.Init(); // 重新初始化相机标定数据
            LOG_INFO("刷新相机%d的标定数据成功", static_cast<int>(cam_id));
            return;
        }
    }
    LOG_WARN("未找到相机%d的标定数据，无法刷新", static_cast<int>(cam_id));
}

/**
 * @brief 重置指定相机下指定机械臂的标定数据
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 */
void SysCalibMgr::resetArmCalibData(uint8_t cam_id, uint8_t arm_id) 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    for (auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id)
        {
            // 查找机械臂数据
            // 这里我们简单地重新初始化机械臂标定数据
            // 在实际应用中，可能需要更复杂的刷新逻辑
            ArmCalibInfo arm_calib_info;
            arm_calib_info.arm_id = arm_id;
            arm_calib_info.Init();
            cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info);
            LOG_INFO("刷新相机%d下机械臂%d的标定数据成功", static_cast<int>(cam_id), static_cast<int>(arm_id));
            return;
        }
    }
    LOG_WARN("未找到相机%d的标定数据，无法刷新其下机械臂%d的数据", static_cast<int>(cam_id), static_cast<int>(arm_id));
}

/**
 * @brief 设置系统最大相机数量
 * @param max_cam_num 最大相机数量
 */
void SysCalibMgr::setSysCamNum(uint8_t sys_cam_num) 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    sys_cam_num_ = sys_cam_num;
    LOG_INFO("设置系统相机个数为（包含禁用的相机）: %d", static_cast<int>(sys_cam_num));
}

/**
 * @brief 获取系统最大相机数量
 * @return 最大相机数量
 */
uint8_t SysCalibMgr::getSysCamNum() const 
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    return sys_cam_num_;
}

/**
 * @brief 加载指定相机和机械臂的偏移补偿参数
 * @param bas_config_data_path bas_config_data路径
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 是否加载成功
 */
bool SysCalibMgr::loadTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id)
{
    try 
    {
        std::lock_guard<std::mutex> lock(calib_data_mutex_);// 获取当前的标定数据
        for (auto& cam_calib_info : cam_calib_data_list_) // 遍历查找匹配的相机ID，而不是直接使用索引
        {
            if (cam_calib_info.cam_id == cam_id) 
            {
                if (cam_calib_info.hasArmCalibInfo(arm_id)) 
                {
                    CalibRes calib_res = cam_calib_info.getArmCalibInfo(arm_id).calib_info.calib_res;// 调用普通接口函数加载偏移补偿参数
                    if (handeyecalib::loadTcpOffset(bas_config_data_path, cam_id, arm_id, calib_res))
                    {
                        ArmCalibInfo arm_calib_info = cam_calib_info.getArmCalibInfo(arm_id);// 更新标定数据中的偏移补偿参数
                        arm_calib_info.calib_info.calib_res = calib_res;
                        cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info);
                        return true;
                    }
                }
                break; // 找到匹配的相机ID后退出循环
            }
        }
        LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d的标定数据，无法加载偏移补偿参数", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d的偏移补偿参数时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
}

/**
 * @brief 修改指定相机和机械臂的偏移补偿参数
 * @param bas_config_data_path bas_config_data路径
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @param offset_compensation 偏移补偿参数 [x, y, z, rx, ry, rz]
 * @return 是否修改成功
 */
bool SysCalibMgr::saveTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const std::vector<double>& offset_compensation)
{
    std::lock_guard<std::mutex> lock(calib_data_mutex_);
    for (auto& cam_calib_info : cam_calib_data_list_) 
    {
        if (cam_calib_info.cam_id == cam_id) 
        {
            if (cam_calib_info.hasArmCalibInfo(arm_id)) 
            {
                ArmCalibInfo arm_calib_info = cam_calib_info.getArmCalibInfo(arm_id);// 获取当前的机械臂标定信息
                arm_calib_info.calib_info.calib_res.offset_compensation = offset_compensation;// 更新偏移补偿参数
                cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info);// 保存更新后的标定信息
                if (handeyecalib::saveTcpOffset(bas_config_data_path, cam_id, arm_id, arm_calib_info.calib_info.calib_res))
                {
                    LOG_INFO("成功修改并保存相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数", static_cast<int>(cam_id), static_cast<int>(arm_id));
                    return true;
                }
                else
                {
                    LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数失败", static_cast<int>(cam_id), static_cast<int>(arm_id));
                    return false;
                }
            }
            else
            {
                LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d 的标定数据", static_cast<int>(cam_id), static_cast<int>(arm_id));
                return false;
            }
        }
    }
    LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d 的标定数据", static_cast<int>(cam_id), static_cast<int>(arm_id));
    return false;
}

/**
    * @brief 加载指定相机和机械臂的标定结果
    * @param bas_config_data_path bas_config_data路径
    * @param cam_id 相机ID
    * @param arm_id 机械臂ID
    * @param calib_res 标定结果
    * @return 是否加载成功
    */
// 加载指定相机和机械臂的标定结果
bool SysCalibMgr::loadCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res)
{
    try 
    {
        std::lock_guard<std::mutex> lock(calib_data_mutex_);
        for (auto& cam_calib_info : cam_calib_data_list_) // 遍历查找匹配的相机ID，而不是直接使用索引
        {
            if (cam_calib_info.cam_id == cam_id) 
            {
                if (cam_calib_info.hasArmCalibInfo(arm_id)) 
                {
                    // 调用普通接口函数加载标定结果
                    if (handeyecalib::loadCalibRes(bas_config_data_path, cam_id, arm_id, calib_res))
                    {
                        // 更新标定数据中的标定结果
                        ArmCalibInfo arm_calib_info = cam_calib_info.getArmCalibInfo(arm_id);
                        arm_calib_info.calib_info.calib_res = calib_res;
                        cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info);
                        return true;
                    }
                }
                break; // 找到匹配的相机ID后退出循环
            }
        }
        LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d的标定数据，无法加载标定结果", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d的标定结果时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
}

/**
    * @brief 保存指定相机和机械臂的标定结果
    * @param bas_config_data_path bas_config_data路径
    * @param cam_id 相机ID
    * @param arm_id 机械臂ID
    * @param calib_res 标定结果
    * @return 是否保存成功
    */
// 保存指定相机和机械臂的标定结果
bool SysCalibMgr::saveCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res)
{
    try
    {
        std::lock_guard<std::mutex> lock(calib_data_mutex_);
        for (auto& cam_calib_info : cam_calib_data_list_) 
        {
            if (cam_calib_info.cam_id == cam_id) 
            {
                if (cam_calib_info.hasArmCalibInfo(arm_id)) 
                {
                    ArmCalibInfo arm_calib_info = cam_calib_info.getArmCalibInfo(arm_id); // 获取当前的机械臂标定信息
                    arm_calib_info.calib_info.calib_res = calib_res; // 更新标定结果
                    cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info); // 保存更新后的标定信息
                    if (handeyecalib::saveCalibRes(bas_config_data_path, cam_id, arm_id, arm_calib_info.calib_info.calib_res))
                    {
                        LOG_INFO("成功保存相机cam_id= %d 下机械臂arm_id= %d 的标定结果", static_cast<int>(cam_id), static_cast<int>(arm_id));
                        return true;
                    }
                    else
                    {
                        LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的标定结果失败", static_cast<int>(cam_id), static_cast<int>(arm_id));
                        return false;
                    }
                }
                else
                {
                    LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d 的标定数据", static_cast<int>(cam_id), static_cast<int>(arm_id));
                    return false;
                }
            }
        }
        LOG_WARN("未找到相机cam_id= %d 下机械臂arm_id= %d 的标定数据", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的标定结果时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
}

/**
 * @brief 保存指定相机和机械臂的手眼标定数据
 * @param bas_config_data_path bas_config_data路径
 * @param cam_id 相机ID
 * @return 是否保存成功
 */
bool SysCalibMgr::saveArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfo& arm_calib_info)
{
    try
    {
        if (cam_id >= sys_cam_num_) // 检查相机ID是否超出系统相机个数
        {
            LOG_ERROR("相机cam_id= %d 超出系统相机个数 %d", static_cast<int>(cam_id), static_cast<int>(sys_cam_num_));
            return false;
        }
        if (handeyecalib::saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info))// 保存手眼标定数据和偏移补偿参数
        {
            // 同步更新内存中的数据
            std::lock_guard<std::mutex> lock(calib_data_mutex_);
            bool found = false;
            for (auto it = cam_calib_data_list_.begin(); it != cam_calib_data_list_.end(); ++it) 
            {
                if (it->cam_id == cam_id) 
                {
                    LOG_DEBUG("找到相机cam_id= %d 的现有标定数据记录，刷新标定数据", static_cast<int>(cam_id));
                    it->setArmCalibInfo(arm_calib_info.arm_id, arm_calib_info);
                    found = true;
                    break;
                }
            }
            if (!found) // 如果没有找到相同cam_id的记录，则添加新的记录
            {
                LOG_DEBUG("未找到相机cam_id= %d 的现有标定数据记录，添加新的标定数据记录", static_cast<int>(cam_id));
                CamCalibInfo cam_calib_info;
                cam_calib_info.cam_id = cam_id;
                cam_calib_info.setArmCalibInfo(arm_calib_info.arm_id, arm_calib_info);
                cam_calib_data_list_.push_back(cam_calib_info);
                LOG_DEBUG("添加后列表大小: %zu", cam_calib_data_list_.size());
            } else {
                LOG_DEBUG("替换后列表大小保持为: %zu", cam_calib_data_list_.size());
            }
            LOG_INFO("成功保存相机cam_id= %d 下机械臂arm_id= %d 的手眼标定数据", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
            return true;
        }
        else
        {
            LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的手眼标定数据失败", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
            return false;
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的手眼标定数据时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id), e.what());
        return false;
    }
}

/**
 * @brief 加载某个相机下的所有机械臂对应的手眼标定数据
 * @param bas_config_data_path bas_config_data路径
 * @param cam_id 相机ID
 * @return 是否加载成功
 */
bool SysCalibMgr::loadCamCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const std::vector<uint8_t>& arm_id_list)
{
    // 调用普通接口函数加载相机标定数据
    CamCalibInfo cam_calib_info;
    if (handeyecalib::loadCamCalibInfo(bas_config_data_path, cam_id, arm_id_list, getSysCamNum(), cam_calib_info)) 
    {
        std::lock_guard<std::mutex> lock(calib_data_mutex_);
        bool found = false;// 查找是否已存在相同cam_id的记录
        for (auto it = cam_calib_data_list_.begin(); it != cam_calib_data_list_.end(); ++it) 
        {
            if (it->cam_id == cam_id) 
            {
                LOG_DEBUG("找到相机cam_id= %d 的现有标定数据记录，刷新标定数据", static_cast<int>(cam_id));
                *it = cam_calib_info;
                found = true;
                break;
            }
        }
        std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
        if (!found) // 如果没有找到相同cam_id的记录，则添加新的记录
        {
            LOG_DEBUG("未找到相机cam_id= %d 的现有标定数据记录，添加新的标定数据记录", static_cast<int>(cam_id));
            cam_calib_data_list_.push_back(cam_calib_info);
            LOG_DEBUG("添加后当前相机cam_id= %d 标定数据列表 %s 大小: %zu", static_cast<int>(cam_id), arm_ids_str.c_str(), cam_calib_data_list_.size());
        } else {
            LOG_DEBUG("替换后当前相机cam_id= %d 标定数据列表大小保持为: %zu", static_cast<int>(cam_id), cam_calib_data_list_.size());
        }
        LOG_INFO("加载相机cam_id= %d 的标定数据成功！机械臂列表：%s", static_cast<int>(cam_id), arm_ids_str.c_str());
        return true;
    }
    return false;
}

/**
 * @brief 加载系统中指定相机列表下的指定机械臂对应的手眼标定数据
 * @param bas_config_data_path bas_config_data路径
 * @param cam_arm_id_list 相机ID和相机ID下配置的机械臂ID映射表(key值为相机ID)
 * @return 是否加载成功
 */
bool SysCalibMgr::loadCamCalibInfoList(const std::string& bas_config_data_path, const std::map<uint8_t, std::vector<uint8_t>>& cam_arm_id_list)
{
    try 
    {
        bool all_loaded = true;
        for (const auto& cam_arm_pair : cam_arm_id_list) // 遍历相机ID和机械臂ID映射表，加载每个相机下指定机械臂的标定数据
        {
            uint8_t cam_id = cam_arm_pair.first;
            const std::vector<uint8_t>& arm_id_list = cam_arm_pair.second;
            if (cam_id >= getSysCamNum()) // 检查相机ID是否合法
            {
                LOG_WARN("跳过相机ID %d，超出系统相机个数 %d", static_cast<int>(cam_id), static_cast<int>(getSysCamNum()));
                all_loaded = false;
                continue;
            }
            if (!SysCalibMgr::loadCamCalibInfo(bas_config_data_path, cam_id, arm_id_list)) //加载单个相机的标定数据
            {
                LOG_WARN("加载相机ID %d 的标定数据失败", static_cast<int>(cam_id));
                all_loaded = false;
            }
        }
        if (all_loaded) 
        {
            LOG_INFO("成功加载指定相机列表下的指定机械臂标定数据");
        } 
        else 
        {
            LOG_WARN("部分相机的机械臂标定数据加载失败");
        }
        return all_loaded;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("加载指定相机列表的机械臂标定数据时发生异常: %s", e.what());
        return false;
    }
}

} // namespace handeyecalib
