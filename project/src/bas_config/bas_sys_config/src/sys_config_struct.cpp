/**
 * @file sys_config_struct.cpp
 * @brief 系统配置信息类实现
 * 
 * 实现ArmConfigInfo和CamConfigInfo类的构造函数、拷贝构造函数等成员函数
 */
#include "bas_sys_config/sys_config_struct.hpp"
#include <algorithm>
#define varName(x) #x // 定义获取变量名字符串的宏

namespace SysConfig
{

// ArmConfigInfo 类实现

ArmConfigInfo::ArmConfigInfo()
{
    Init();
}

ArmConfigInfo::ArmConfigInfo(const ArmConfigInfo& para)
{
    CopyFrom(para);
}

ArmConfigInfo& ArmConfigInfo::operator=(const ArmConfigInfo& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

ArmConfigInfo::~ArmConfigInfo()
{
    // 析构函数，如果有需要清理的资源可以在这里处理
}

void ArmConfigInfo::Init()
{
    is_enable = false;
    arm_id = 0;       // 机械臂ID
    robot_arm_ip = "";
    user_name = "";
}

void ArmConfigInfo::Rst()
{
    Init();
}

void ArmConfigInfo::CopyFrom(const ArmConfigInfo& para)
{
    if (this != &para)
		para.CopyTo(*this);
}

void ArmConfigInfo::CopyTo(ArmConfigInfo& para) const
{
    if (this != &para)
    {
        para.is_enable = is_enable;
        para.arm_id = arm_id;
        para.robot_arm_ip = robot_arm_ip;
        para.user_name = user_name;
    } 
}

// 获取参数名称字符串
const char* ArmConfigInfo::getParamNameString(ParamName paramName)  
{
    switch (paramName) 
    {
        case ParamName::IS_ENABLE: return varName(is_enable);
        case ParamName::ARM_ID: return varName(arm_id);
        case ParamName::ROBOT_ARM_IP: return varName(robot_arm_ip);
        case ParamName::USER_NAME: return varName(user_name);
        default: return "";
    }
}

// 根据参数名称字符串获取ParamName枚举值
ArmConfigInfo::ParamName ArmConfigInfo::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(is_enable)) return ParamName::IS_ENABLE;
    if (paramNameStr == varName(arm_id)) return ParamName::ARM_ID;
    if (paramNameStr == varName(robot_arm_ip)) return ParamName::ROBOT_ARM_IP;
    if (paramNameStr == varName(user_name)) return ParamName::USER_NAME;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

// CamConfigInfo 类实现
CamConfigInfo::CamConfigInfo()
{
    Init();
}

CamConfigInfo::CamConfigInfo(const CamConfigInfo& para)
{
    CopyFrom(para);
}

CamConfigInfo& CamConfigInfo::operator=(const CamConfigInfo& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

CamConfigInfo::~CamConfigInfo()
{
    // 析构函数，如果有需要清理的资源可以在这里处理
}

void CamConfigInfo::Init()
{
    is_enable = false;
    cam_id = 0;       // 相机ID
    serial_number = "";
    user_name = "";
    armInfoList.clear();
    default_color_resolution = ColorResolution::RGB_NATIVE;  // 当前相机的默认彩色分辨率
    default_depth_resolution = DepthResolution::DEPTH_NATIVE;  // 当前相机的默认深度分辨率
}

void CamConfigInfo::Rst()
{
    Init();
}

void CamConfigInfo::CopyFrom(const CamConfigInfo& para)
{
    if (this != &para)
		para.CopyTo(*this);
}

void CamConfigInfo::CopyTo(CamConfigInfo& para) const
{
    if (this != &para)
    {
        para.is_enable = is_enable;
        para.cam_id = cam_id; 
        para.serial_number = serial_number;
        para.user_name = user_name;
        para.default_color_resolution = default_color_resolution;  // 复制默认彩色分辨率
        para.default_depth_resolution = default_depth_resolution;  // 复制默认深度分辨率
        // 拷贝机械臂信息列表
        para.armInfoList.clear();
        for (const auto& armInfo : armInfoList) 
        {
            ArmConfigInfo info;
            armInfo.CopyTo(info);
            para.armInfoList.push_back(info);
        }
    } 
}

// 获取参数名称字符串
const char* CamConfigInfo::getParamNameString(ParamName paramName)  
{
    switch (paramName) 
    {
        case ParamName::IS_ENABLE: return varName(is_enable);
        case ParamName::CAM_ID: return varName(cam_id);
        case ParamName::SERIAL_NUMBER: return varName(serial_number);
        case ParamName::USER_NAME: return varName(user_name);
        case ParamName::DEFAULT_COLOR_RESOLUTION: return varName(default_color_resolution);
        case ParamName::DEFAULT_DEPTH_RESOLUTION: return varName(default_depth_resolution);
        case ParamName::ARM_IDS: return varName(arm_list);
        default: return "";
    }
}

// 根据参数名称字符串获取ParamName枚举值
CamConfigInfo::ParamName CamConfigInfo::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(is_enable)) return ParamName::IS_ENABLE;
    if (paramNameStr == varName(cam_id)) return ParamName::CAM_ID;
    if (paramNameStr == varName(serial_number)) return ParamName::SERIAL_NUMBER;
    if (paramNameStr == varName(user_name)) return ParamName::USER_NAME;
    if (paramNameStr == varName(default_color_resolution)) return ParamName::DEFAULT_COLOR_RESOLUTION;
    if (paramNameStr == varName(default_depth_resolution)) return ParamName::DEFAULT_DEPTH_RESOLUTION;
    if (paramNameStr == varName(arm_list)) return ParamName::ARM_IDS;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

/**
 * @brief 从机械臂配置信息列表中提取所有机械臂ID
 * 该函数遍历机械臂配置信息列表，提取每个机械臂的ID并组成一个新的ID列表返回。
 * 主要用于参数服务器存储和日志输出等场景。
 * @param arm_list 机械臂配置信息列表
 * @return 包含所有机械臂ID的向量
 */
std::vector<uint8_t> getArmIds(const ArmConfigInfoList& arm_list)
{
    std::vector<uint8_t> arm_ids;
    for (const auto& arm_info : arm_list) 
    {
        arm_ids.push_back(arm_info.arm_id);
    }
    return arm_ids;
}

std::vector<uint8_t> getCamIds(const CamConfigInfo1D& cam_list)
{
    std::vector<uint8_t> cam_ids;
    for (const auto& cam_info : cam_list) 
    {
        cam_ids.push_back(cam_info.cam_id);
    }
    return cam_ids;
}

// 根据机械臂ID列表获取完整的机械臂配置信息列表
SysConfig::ArmConfigInfoList getArmInfoListByIds(const std::vector<uint8_t>& arm_ids, const SysConfig::ArmConfigInfoList& full_arm_list)
{
    SysConfig::ArmConfigInfoList result_list;
    for (const auto& arm_id : arm_ids) // 遍历机械臂ID列表
    {
        if (arm_id < full_arm_list.size()) // 检查ID是否在有效范围内
        {
            for (const auto& arm_info : full_arm_list) 
            {
                if (arm_info.arm_id == arm_id) 
                {
                    result_list.push_back(arm_info);// 将对应的完整机械臂信息添加到结果列表中
                    break;
                }
            }   
        }
    }
    return result_list;
}

// 根据arm_ids列表初始化armInfoList，确保每个元素的arm_id与arm_ids列表中的ID一一对应
void CamConfigInfo::initArmInfoListByArmIds(const std::vector<uint8_t>& arm_ids)
{
    armInfoList.clear();
    for (uint8_t arm_id : arm_ids) 
    {
        ArmConfigInfo arm_info;
        arm_info.Init();
        arm_info.arm_id = arm_id;
        armInfoList.push_back(arm_info);
    }
}

}  // namespace SysConfig