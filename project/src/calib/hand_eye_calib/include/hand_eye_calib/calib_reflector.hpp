#ifndef HAND_EYE_CALIB__CALIB_REFLECTOR_HPP_
#define HAND_EYE_CALIB__CALIB_REFLECTOR_HPP_

#include "data_handler/param_reflector.hpp"
#include "hand_eye_calib/calib_struct.hpp"

namespace datahandler {

// 为cv::Mat添加模板特化
template<> inline void ParamInfo::setType<cv::Mat>() { type = ParamType::STRING; }
// 为Eigen::Matrix4d添加模板特化
template<> inline void ParamInfo::setType<Eigen::Matrix4d>() { type = ParamType::STRING; }

// 为CalibRes添加模板特化，将其作为STRING类型处理
template<> inline void ParamInfo::setType<handeyecalib::CalibRes>() { type = ParamType::STRING; }
// 为QualityMetrics添加模板特化，将其作为STRING类型处理
template<> inline void ParamInfo::setType<handeyecalib::QualityMetrics>() { type = ParamType::STRING; }

} // namespace datahandler

namespace handeyecalib {

#define  TCP_OFFSET_ENABLE  "enable"    // 偏移补偿使能
#define  TCP_OFFSET_X       "offset_x"  // 偏移补偿X
#define  TCP_OFFSET_Y       "offset_y"  // 偏移补偿Y
#define  TCP_OFFSET_Z       "offset_z"  // 偏移补偿Z
#define  TCP_OFFSET_RX      "offset_rx" // 偏移补偿_RX
#define  TCP_OFFSET_RY      "offset_ry" // 偏移补偿_RY
#define  TCP_OFFSET_RZ      "offset_rz" // 偏移补偿_RZ

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
    using datahandler::ConfigReflector::registerParam;
    using datahandler::ConfigReflector::updateParamAftLoad;
};

// CalibRes的反射器
class CalibResReflector : public ConfigReflector {
public:
    explicit CalibResReflector(handeyecalib::CalibRes& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }

    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;
         
    // 输出标定矩阵信息日志
    void printLog_CalibMatrix(const std::string& project_path, int log_level, const std::string& prefix_msg, 
        int color, const char* file_name_path, const char* func, int line) const;
     
    // 输出Eigen标定矩阵信息日志
    void printLog_EigenCalibMatrix(const std::string& project_path, int log_level, const std::string& prefix_msg, 
        int color, const char* file_name_path, const char* func, int line) const;
 
    void printLog_TcpOffset(const std::string& project_path, int log_level, const std::string& prefix_msg, 
        int color, const char* file_name_path, const char* func, int line) const;

    void printLog_HeadMotorAngles(const std::string& project_path, int log_level, const std::string& prefix_msg, 
        int color, const char* file_name_path, const char* func, int line) const;

    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
    // 同步OpenCV矩阵到Eigen矩阵的接口
    void syncOpencvToEigen();

    // 从参数信息列表中获取参数值
    bool tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos);

private:
    void initReflection(handeyecalib::CalibRes& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存CalibRes引用
    handeyecalib::CalibRes& info_;
};

// QualityMetrics的反射器
class QualityMetricsReflector : public ConfigReflector {
public:
    explicit QualityMetricsReflector(handeyecalib::QualityMetrics& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }

    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
private:
    void initReflection(handeyecalib::QualityMetrics& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存QualityMetrics引用
    handeyecalib::QualityMetrics& info_;
};

// CalibInfo的反射器
class CalibInfoReflector : public ConfigReflector {
public:
    explicit CalibInfoReflector(handeyecalib::CalibInfo& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }

    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
private:
    void initReflection(handeyecalib::CalibInfo& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存CalibInfo引用
    handeyecalib::CalibInfo& info_;
};

// ArmCalibInfo的反射器
class ArmCalibInfoReflector : public ConfigReflector {
public:
    explicit ArmCalibInfoReflector(handeyecalib::ArmCalibInfo& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }

     // 打印日志输出函数
    void printLog_armCalibInfo(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
private:
    void initReflection(handeyecalib::ArmCalibInfo& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存ArmCalibInfo引用
    handeyecalib::ArmCalibInfo& info_;
};

// CamCalibInfo的反射器
class CamCalibInfoReflector : public ConfigReflector {
public:
    explicit CamCalibInfoReflector(handeyecalib::CamCalibInfo& info);
    
    // 实现getParamsSaved方法
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override { return saved_params_; }

    // 打印日志输出函数
    void printLog(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;
    
    // 重写基类的updateParamAftLoad方法
    void updateParamAftLoad() override;
    
private:
    void initReflection(handeyecalib::CamCalibInfo& info);
    
    // 保存需要从文件读取的参数列表
    std::vector<datahandler::ParamInfo> saved_params_;
    
    // 保存CamCalibInfo引用
    handeyecalib::CamCalibInfo& info_;
};


// 全局打印日志函数声明
void printLog_CalibRes(const CalibRes& res, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_QualityMetrics(const handeyecalib::QualityMetrics& quality_metrics, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_CalibInfo(const handeyecalib::CalibInfo& calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_ArmCalibInfo(const handeyecalib::ArmCalibInfo& arm_calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_ArmCalibInfoList(const handeyecalib::ArmCalibInfoList& arm_calib_info_list, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_CamCalibInfo(const handeyecalib::CamCalibInfo& cam_calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

void printLog_CamCalibInfoList(const handeyecalib::CamCalibInfoList& cam_calib_info_list, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);
    
} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_REFLECTOR_HPP_