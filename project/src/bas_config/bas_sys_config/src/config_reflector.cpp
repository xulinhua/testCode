#include "bas_sys_config/config_reflector.hpp"
#include "bas_operate/bas_utils.hpp"
#include "log_system/log_macros.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <typeinfo>

namespace datahandler {

// 添加枚举到字符串的转换函数实现
std::string colorResolutionToString(SysConfig::ColorResolution resolution) 
{
    switch(resolution) {
        case SysConfig::ColorResolution::RGB_NATIVE: return "RGB_NATIVE";
        case SysConfig::ColorResolution::RGB_1920x1080: return "RGB_1920x1080";
        case SysConfig::ColorResolution::RGB_1280x720: return "RGB_1280x720";
        case SysConfig::ColorResolution::RGB_640x480: return "RGB_640x480";
        default: return "UNKNOWN";
    }
}

SysConfig::ColorResolution stringToColorResolution(const std::string& str) 
{
    if (str == "RGB_NATIVE") return SysConfig::ColorResolution::RGB_NATIVE;
    if (str == "RGB_1920x1080") return SysConfig::ColorResolution::RGB_1920x1080;
    if (str == "RGB_1280x720") return SysConfig::ColorResolution::RGB_1280x720;
    if (str == "RGB_640x480") return SysConfig::ColorResolution::RGB_640x480;
    return SysConfig::ColorResolution::RGB_NATIVE; // 默认值
}

std::string depthResolutionToString(SysConfig::DepthResolution resolution) 
{
    switch(resolution) {
        case SysConfig::DepthResolution::DEPTH_NATIVE: return "DEPTH_NATIVE";
        case SysConfig::DepthResolution::DEPTH_1280x720: return "DEPTH_1280x720";
        case SysConfig::DepthResolution::DEPTH_640x480: return "DEPTH_640x480";
        case SysConfig::DepthResolution::DEPTH_320x240: return "DEPTH_320x240";
        default: return "UNKNOWN";
    }
}

SysConfig::DepthResolution stringToDepthResolution(const std::string& str) 
{
    if (str == "DEPTH_NATIVE") return SysConfig::DepthResolution::DEPTH_NATIVE;
    if (str == "DEPTH_1280x720") return SysConfig::DepthResolution::DEPTH_1280x720;
    if (str == "DEPTH_640x480") return SysConfig::DepthResolution::DEPTH_640x480;
    if (str == "DEPTH_320x240") return SysConfig::DepthResolution::DEPTH_320x240;
    return SysConfig::DepthResolution::DEPTH_NATIVE; // 默认值
}

// 为SysConfig::ColorResolution和SysConfig::DepthResolution添加setValue特化实现
template<> 
bool ParamInfo::setValue<SysConfig::ColorResolution>(SysConfig::ColorResolution& val) {
    value = colorResolutionToString(val);
    setType<SysConfig::ColorResolution>();
    return true; // 总是成功
}

template<> 
bool ParamInfo::setValue<SysConfig::DepthResolution>(SysConfig::DepthResolution& val) {
    value = depthResolutionToString(val);
    setType<SysConfig::DepthResolution>();
    return true; // 总是成功
}

// 为SysConfig::ColorResolution和SysConfig::DepthResolution添加getValue特化实现
template<> 
SysConfig::ColorResolution ParamInfo::getValue<SysConfig::ColorResolution>() const {
    std::string str = std::any_cast<std::string>(value);
    return stringToColorResolution(str);
}

template<> 
SysConfig::DepthResolution ParamInfo::getValue<SysConfig::DepthResolution>() const {
    std::string str = std::any_cast<std::string>(value);
    return stringToDepthResolution(str);
}

} // namespace datahandler

namespace SysConfig {

// ArmConfigInfoReflector 实现
ArmConfigInfoReflector::ArmConfigInfoReflector(ArmConfigInfo& info) 
    : info_(info) // 保存ArmConfigInfo引用
{
    initReflection(info);
}

void ArmConfigInfoReflector::initReflection(ArmConfigInfo& info) 
{
    params_.clear();
    saved_params_.clear();
    // 注册所有参数
    registerParam(ArmConfigInfo::getParamNameString(ArmConfigInfo::ParamName::ARM_ID), info.arm_id);
    registerParam(ArmConfigInfo::getParamNameString(ArmConfigInfo::ParamName::IS_ENABLE), info.is_enable);
    registerParam(ArmConfigInfo::getParamNameString(ArmConfigInfo::ParamName::ROBOT_ARM_IP), info.robot_arm_ip);
    registerParam(ArmConfigInfo::getParamNameString(ArmConfigInfo::ParamName::USER_NAME), info.user_name);

    for (const auto& param : params_) // 填充需要从文件读取的参数列表
	{
        if (param.name != ArmConfigInfo::getParamNameString(ArmConfigInfo::ParamName::ARM_ID)) //排除ARM_ID
        {
            saved_params_.push_back(param);
        }
    }
}

 // 打印日志输出函数
void ArmConfigInfoReflector::printLog(const std::string& project_path, int log_level, int color, 
    const char* file_name_path, const char* func, int line) const
{
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "ArmConfigInfo反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
}

// 打印保存的参数信息
void ArmConfigInfoReflector::printLog_saved_params(const std::string& project_path, int log_level, int color, 
    const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "ArmConfigInfo反射器[saved_params]参数信息:");
    datahandler::printLog(saved_params_, project_path, log_level, color, file_name_path, func, line);
}
    
bool ArmConfigInfoReflector::tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo> &param_infos)
{
    return true;
} 

//在加载配置文件后更新参数
void ArmConfigInfoReflector::updateParamAftLoad()
{
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

// CamConfigInfoReflector 实现
CamConfigInfoReflector::CamConfigInfoReflector(CamConfigInfo& info) 
    : info_(info) // 保存CamConfigInfo引用
{
    initReflection(info);
}

void CamConfigInfoReflector::initReflection(CamConfigInfo& info) 
{
    params_.clear();
    saved_params_.clear();
    registerParam(CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::IS_ENABLE), info.is_enable);
    registerParam(CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::CAM_ID), info.cam_id);
    registerParam(CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::SERIAL_NUMBER), info.serial_number);
    registerParam(CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::USER_NAME), info.user_name);
    
    // 为ColorResolution和DepthResolution创建特殊的ParamInfo，将其作为STRING类型处理
    // 将枚举类型参数的ptr设为nullptr，避免在loadParamInfoFromYaml中直接赋值
    ParamInfo colorResInfo;
    colorResInfo.name = CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_COLOR_RESOLUTION);
    colorResInfo.type = datahandler::ParamType::STRING;
    colorResInfo.ptr = nullptr;  // 不指向实际内存地址，避免直接赋值
    colorResInfo.size = sizeof(SysConfig::ColorResolution);
    std::string colorResStr = datahandler::colorResolutionToString(info.default_color_resolution);
    colorResInfo.value = colorResStr;
    params_.push_back(colorResInfo);
    
    ParamInfo depthResInfo;
    depthResInfo.name = CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_DEPTH_RESOLUTION);
    depthResInfo.type = datahandler::ParamType::STRING;
    depthResInfo.ptr = nullptr;  // 不指向实际内存地址，避免直接赋值
    depthResInfo.size = sizeof(SysConfig::DepthResolution);
    std::string depthResStr = datahandler::depthResolutionToString(info.default_depth_resolution);
    depthResInfo.value = depthResStr;
    params_.push_back(depthResInfo);

    ParamInfo arm_ids_para;// 注意：armInfoList 是复杂类型，这里记录其机械臂ID列表信息
    arm_ids_para.name = CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::ARM_IDS);// 创建一个特殊的ParamInfo来存储armInfoList的机械臂ID列表
    arm_ids_para.type = ParamType::UINT8_ARRAY; // 使用UINT8_ARRAY类型来存储ID列表
    arm_ids_para.ptr = nullptr; // 不指向实际内存地址
    std::vector<uint8_t> arm_ids = SysConfig::getArmIds(info.armInfoList);// 获取机械臂ID列表
    arm_ids_para.value = arm_ids;
    params_.push_back(arm_ids_para);

    for (const auto& param : params_) // 填充需要从文件读取的参数列表
	{
        if (param.name != CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::CAM_ID)) // 排除CAM_ID
        {
            saved_params_.push_back(param);
        }
    }
}

// 打印日志输出函数
void CamConfigInfoReflector::printLog(const std::string& project_path, int log_level, int color, 
    const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "CamConfigInfo反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
    // 打印每个机械臂的配置信息
    for (const auto& arm_info : info_.armInfoList) 
    {
        // 创建ArmConfigInfoReflector实例并打印
        SysConfig::ArmConfigInfoReflector arm_reflector(const_cast<SysConfig::ArmConfigInfo&>(arm_info));
        arm_reflector.printLog(project_path, log_level, color, file_name_path, func, line); // 调用机械臂配置信息打印函数
    }
}

// 打印保存的参数信息
void CamConfigInfoReflector::printLog_saved_params(const std::string& project_path, int log_level, int color, 
    const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "CamConfigInfo反射器[saved_params]参数信息:");
    datahandler::printLog(saved_params_, project_path, log_level, color, file_name_path, func, line);
    // 打印每个机械臂保存的配置信息
    for (const auto& arm_info : info_.armInfoList) 
    {
        // 创建ArmConfigInfoReflector实例并打印
        SysConfig::ArmConfigInfoReflector arm_reflector(const_cast<SysConfig::ArmConfigInfo&>(arm_info));
        arm_reflector.printLog_saved_params(project_path, log_level, color, file_name_path, func, line); 
    }
}

// 静态方法：从参数信息列表中获取ARM_IDS参数值
bool CamConfigInfoReflector::getArmIdsFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos, std::vector<uint8_t>& arm_ids)
{
    arm_ids.clear(); // 清空输出参数
    for (const auto& param : param_infos) 
    {
        if (param.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::ARM_IDS)) 
        {
            if (param.value.type() == typeid(std::vector<uint8_t>)) 
            {
                arm_ids = std::any_cast<std::vector<uint8_t>>(param.value);
                return true; // 成功获取ARM_IDS参数值
            }
        }
    }
    return false;// 如果没有找到ARM_IDS参数或类型不匹配，返回false
}

bool CamConfigInfoReflector::tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo> &param_infos)
{
    for (const auto& param : param_infos) 
    {
        // 转换参数
        if (param.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_COLOR_RESOLUTION)) 
        {
            info_.default_color_resolution = param.getValue<SysConfig::ColorResolution>();
        } else if (param.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_DEPTH_RESOLUTION)) 
        {
            info_.default_depth_resolution = param.getValue<SysConfig::DepthResolution>();
        }
    }
    return true;
}

//在加载配置文件后更新参数
void CamConfigInfoReflector::updateParamAftLoad()
{
    for (const auto& saved_param : saved_params_) 
    {
        if (saved_param.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_COLOR_RESOLUTION)) {
            // 更新info_中的实际值
            info_.default_color_resolution = saved_param.getValue<SysConfig::ColorResolution>();
        } else if (saved_param.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_DEPTH_RESOLUTION)) {
            // 更新info_中的实际值
            info_.default_depth_resolution = saved_param.getValue<SysConfig::DepthResolution>();
        }
    }
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

} // namespace SysConfig