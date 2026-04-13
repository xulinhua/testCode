#ifndef SYS_CONFIG_REFLECTOR_HPP
#define SYS_CONFIG_REFLECTOR_HPP

#include "data_handler/param_reflector.hpp"
#include "bas_sys_config/sys_config_struct.hpp"

namespace datahandler {

// 添加枚举到字符串的转换函数声明
std::string colorResolutionToString(SysConfig::ColorResolution resolution);
std::string depthResolutionToString(SysConfig::DepthResolution resolution);

SysConfig::ColorResolution stringToColorResolution(const std::string& str);
SysConfig::DepthResolution stringToDepthResolution(const std::string& str);

// 为SysConfig::ColorResolution和SysConfig::DepthResolution添加setValue和getValue特化声明
template<> 
bool ParamInfo::setValue<SysConfig::ColorResolution>(SysConfig::ColorResolution& val);
template<> 
bool ParamInfo::setValue<SysConfig::DepthResolution>(SysConfig::DepthResolution& val);
template<> 
SysConfig::ColorResolution ParamInfo::getValue<SysConfig::ColorResolution>() const;
template<> 
SysConfig::DepthResolution ParamInfo::getValue<SysConfig::DepthResolution>() const;

// 为SysConfig::ColorResolution和SysConfig::DepthResolution添加模板特化，将其作为STRING类型处理
template<> inline void ParamInfo::setType<SysConfig::ColorResolution>() { type = ParamType::STRING; }
template<> inline void ParamInfo::setType<SysConfig::DepthResolution>() { type = ParamType::STRING; }

} // namespace datahandler

namespace SysConfig {

// 重命名参数类型枚举以避免冲突
using ParamType = datahandler::ParamType;

// 重命名参数信息结构体以避免冲突
using ParamInfo = datahandler::ParamInfo;

// 配置反射器基类
class ConfigReflector : public datahandler::ConfigReflector {
public:
    virtual ~ConfigReflector() = default;
    
    // 继承基类的方法
    using datahandler::ConfigReflector::getParams;
    using datahandler::ConfigReflector::getParamsSaved;
    using datahandler::ConfigReflector::getParamByName;
    //using datahandler::ConfigReflector::getParamsMsg;
    using datahandler::ConfigReflector::registerParam;
    using datahandler::ConfigReflector::updateParamAftLoad;
};
// ArmConfigInfo的反射器
class ArmConfigInfoReflector : public ConfigReflector {
public:
    explicit ArmConfigInfoReflector(ArmConfigInfo& info);
    
    // 重写getParamsSaved方法，只返回需要从文件读取的参数
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }
    
    // 继承基类的静态方法
    //using datahandler::ConfigReflector::getParamsMsg;
    
    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, int color, 
        const char* file_name_path, const char* func, int line) const;

    // 打印保存的参数信息
    void printLog_saved_params(const std::string& project_path, int log_level, int color, 
        const char* file_name_path, const char* func, int line) const;

    // 从参数信息列表中获取参数值
    bool tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos);
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
private:
    void initReflection(ArmConfigInfo& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存ArmConfigInfo引用
    ArmConfigInfo& info_;
};

// CamConfigInfo的反射器
class CamConfigInfoReflector : public ConfigReflector {
public:
    explicit CamConfigInfoReflector(CamConfigInfo& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }
    
    // 继承基类的静态方法
    //using datahandler::ConfigReflector::getParamsMsg;

    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, int color, 
        const char* file_name_path, const char* func, int line) const;

    // 打印保存的参数信息
    void printLog_saved_params(const std::string& project_path, int log_level, int color, 
        const char* file_name_path, const char* func, int line) const;
    
    // 静态方法：从参数信息列表中获取ARM_IDS参数值
    static bool getArmIdsFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos, std::vector<uint8_t>& arm_ids);
    
    // 从参数信息列表中获取参数值
    bool tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos);
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
private:
    void initReflection(CamConfigInfo& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存CamConfigInfo引用
    CamConfigInfo& info_;
};

} // namespace SysConfig

#endif // SYS_CONFIG_REFLECTOR_HPP