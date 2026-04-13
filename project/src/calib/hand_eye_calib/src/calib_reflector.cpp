#include "hand_eye_calib/calib_reflector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "bas_operate/bas_utils.hpp"
#include "log_system/log_macros.hpp"
#include <algorithm>
#include <sstream>
#include <typeinfo>

namespace handeyecalib {

// CalibResReflector 实现
CalibResReflector::CalibResReflector(handeyecalib::CalibRes& info) 
    : info_(info) // 保存CalibRes引用
{
    initReflection(info);
}

void CalibResReflector::initReflection(handeyecalib::CalibRes& info)
{
    params_.clear();
    saved_params_.clear();
    // 注册所有参数

    // 注册标定类型参数
    ParamInfo calibTypeInfo;
    calibTypeInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::CALIB_TYPE);
    calibTypeInfo.type = ParamType::STRING;
    calibTypeInfo.ptr = nullptr;
    calibTypeInfo.size = sizeof(uint8_t);
    calibTypeInfo.value = std::string(handeyecalib::CalibRes::getCalibTypeString(info.calib_type));
    params_.push_back(calibTypeInfo);

    // 为cv::Mat类型的参数创建特殊的ParamInfo，将其作为STRING类型处理
    ParamInfo camToBaseInfo;
    camToBaseInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::CAM_TO_BASE_TRANSFORM);
    camToBaseInfo.type = ParamType::STRING;
    //camToBaseInfo.ptr = &info.cam_to_base_transform;
    camToBaseInfo.ptr = nullptr;
    camToBaseInfo.size = sizeof(cv::Mat);
    camToBaseInfo.value = handeyecalib::matToString(info.cam_to_base_transform);
    params_.push_back(camToBaseInfo);

    ParamInfo baseToCamInfo;
    baseToCamInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::BASE_TO_CAM_TRANSFORM);
    baseToCamInfo.type = ParamType::STRING;
    //baseToCamInfo.ptr = &info.base_to_cam_transform;
    baseToCamInfo.ptr = nullptr;
    baseToCamInfo.size = sizeof(cv::Mat);
    baseToCamInfo.value = handeyecalib::matToString(info.base_to_cam_transform);
    params_.push_back(baseToCamInfo);

    // 为Eigen::Matrix4d类型的参数创建特殊的ParamInfo，将其作为STRING类型处理
    ParamInfo eigenCamToBaseInfo;
    eigenCamToBaseInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::EIGEN_CAM_TO_BASE);
    eigenCamToBaseInfo.type = ParamType::STRING;
    //eigenCamToBaseInfo.ptr = &info.eigen_cam_to_base;
    eigenCamToBaseInfo.ptr = nullptr;
    eigenCamToBaseInfo.size = sizeof(Eigen::Matrix4d);
    eigenCamToBaseInfo.value = handeyecalib::eigenMatrixToString(info.eigen_cam_to_base);
    params_.push_back(eigenCamToBaseInfo);

    ParamInfo eigenBaseToCamInfo;
    eigenBaseToCamInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::EIGEN_BASE_TO_CAM);
    eigenBaseToCamInfo.type = ParamType::STRING;
    //eigenBaseToCamInfo.ptr = &info.eigen_base_to_cam;
    eigenBaseToCamInfo.ptr = nullptr;
    eigenBaseToCamInfo.size = sizeof(Eigen::Matrix4d);
    eigenBaseToCamInfo.value = handeyecalib::eigenMatrixToString(info.eigen_base_to_cam);
    params_.push_back(eigenBaseToCamInfo);

    // 为std::vector<double>类型的参数创建ParamInfo，使用DOUBLE_ARRAY类型处理
    ParamInfo offsetCompensationInfo;
    offsetCompensationInfo.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::OFFSET_COMPENSATION);
    offsetCompensationInfo.type = ParamType::DOUBLE_ARRAY;
    offsetCompensationInfo.ptr = &info.offset_compensation;
    offsetCompensationInfo.size = sizeof(std::vector<double>);
    offsetCompensationInfo.value = info.offset_compensation;
    params_.push_back(offsetCompensationInfo);

    // 为std::vector<double>类型的参数创建ParamInfo，使用DOUBLE_ARRAY类型处理
    ParamInfo head_motor_angles_Info;
    head_motor_angles_Info.name = handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::HEAD_MOTOR_ANGLES);
    head_motor_angles_Info.type = ParamType::DOUBLE_ARRAY;
    head_motor_angles_Info.ptr = &info.head_motor_angles;
    head_motor_angles_Info.size = sizeof(std::vector<double>);
    head_motor_angles_Info.value = info.head_motor_angles;
    params_.push_back(head_motor_angles_Info);

    // 填充需要从文件读取的参数列表，但排除Eigen矩阵（因为它们应该从cv::Mat同步）
    for (const auto& param : params_) 
    {
        if (param.name != handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::EIGEN_CAM_TO_BASE) &&
            param.name != handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::EIGEN_BASE_TO_CAM)) {
            saved_params_.push_back(param);
        }
    }
}

// 从参数信息列表中获取参数值
bool CalibResReflector::tranInfoFromParamInfos(const std::vector<datahandler::ParamInfo>& param_infos)
{
    for (const auto& param_info : param_infos) // 更新info_中的实际值，使用非const引用以允许修改
    {
        handeyecalib::CalibRes::ParamName paramName = handeyecalib::CalibRes::getParamNameEnum(param_info.name);
        std::string strVal;
        switch (paramName)
        {
        case handeyecalib::CalibRes::ParamName::CALIB_TYPE: 
            strVal = param_info.getValue<std::string>();
            info_.calib_type = handeyecalib::CalibRes::getCalibTypeEnum(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::CAM_TO_BASE_TRANSFORM: 
            strVal = param_info.getValue<std::string>();
            LOG_INFO(("已更新标定结果参数: " + strVal).c_str());
            info_.cam_to_base_transform = handeyecalib::stringToMat(strVal);
            LOG_INFO("已更新标定结果参数维度: cam_to_base_transform(%dx%d)",
                       info_.cam_to_base_transform.rows, info_.cam_to_base_transform.cols);
            break;
        case handeyecalib::CalibRes::ParamName::BASE_TO_CAM_TRANSFORM: 
            strVal = param_info.getValue<std::string>();
            LOG_INFO(("已更新标定结果参数: " + strVal).c_str());
            info_.base_to_cam_transform = handeyecalib::stringToMat(strVal);
            LOG_INFO("已更新标定结果参数维度: base_to_cam_transform(%dx%d)",
                       info_.base_to_cam_transform.rows, info_.base_to_cam_transform.cols);
            break;
        case handeyecalib::CalibRes::ParamName::EIGEN_CAM_TO_BASE: 
            strVal = param_info.getValue<std::string>();
            LOG_INFO(("已更新标定结果参数: " + strVal).c_str());
            info_.eigen_cam_to_base = handeyecalib::stringToEigenMatrix4d(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::EIGEN_BASE_TO_CAM: 
            strVal = param_info.getValue<std::string>();
            LOG_INFO(("已更新标定结果参数: " + strVal).c_str());
            info_.eigen_base_to_cam = handeyecalib::stringToEigenMatrix4d(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::OFFSET_COMPENSATION: 
            //已经通过调用datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
            break;
        case handeyecalib::CalibRes::ParamName::HEAD_MOTOR_ANGLES: 
            //已经通过调用datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
            break;
        default: 
            break;
        } 
    }
    return true;
}

void CalibResReflector::syncOpencvToEigen()
{
    std::string class_name = basmodule::get_type_name(typeid(*this));
    std::string project_path = "hand_eye_calib\\" + class_name;
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(project_path, log_level))
    {
        LOG_OUT(project_path, log_level, "开始同步OpenCV标定矩阵到Eigen矩阵，原始的OpenCV标定矩阵信息如下：");
        printLog_CalibMatrix(project_path, (int)log_level, "原始的OpenCV标定矩阵信息：", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    // 将cam_to_base_transform的数据同步到eigen_cam_to_base
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            info_.eigen_cam_to_base(i, j) = info_.cam_to_base_transform.at<double>(i, j);
        }
    }
    LOG_DEBUG(project_path, "cam_to_base_transform已同步到eigen_cam_to_base");
    // 将base_to_cam_transform的数据同步到eigen_base_to_cam
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            info_.eigen_base_to_cam(i, j) = info_.base_to_cam_transform.at<double>(i, j);
        }
    }
    LOG_DEBUG(project_path, "base_to_cam_transform已同步到eigen_base_to_cam");
    if (LOG_ON(project_path, log_level))
    {
        LOG_OUT(project_path, log_level, "OpenCV标定矩阵到Eigen矩阵同步完成，已生成的Eigen标定矩阵信息如下：");
        printLog_EigenCalibMatrix(project_path, (int)log_level, "已生成的Eigen标定矩阵信息：", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
}

void CalibResReflector::printLog(const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    bool bShowExMsg = false;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "CalibRes反射器参数信息:");
    std::vector<ParamInfo> print_params;
    for (const auto& param : params_) 
    {
        if (param.name == handeyecalib::CalibRes::getParamNameString(handeyecalib::CalibRes::ParamName::CALIB_TYPE)) {
            print_params.push_back(param);
        }
    }
    datahandler::printLog(print_params, project_path, (int)log_level, color, file_name_path, func, line);
    printLog_CalibMatrix(project_path, (int)log_level, "标定结果矩阵信息：", color, file_name_path, func, line);
    printLog_EigenCalibMatrix(project_path, (int)log_level, "标定结果Eigen矩阵信息：", color, file_name_path, func, line);
    printLog_TcpOffset(project_path, (int)log_level, "偏移补偿信息：", color, file_name_path, func, line);
    printLog_HeadMotorAngles(project_path, (int)log_level,  "头部电机角度信息：", color, file_name_path, func, line);
}

void CalibResReflector::printLog_CalibMatrix(const std::string& project_path, int log_level, const std::string& prefix_msg, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    handeyecalib::printLog_CalibMatrix(project_path, log_level, info_.cam_to_base_transform, info_.base_to_cam_transform, prefix_msg, color, file_name_path, func, line);
}

void CalibResReflector::printLog_EigenCalibMatrix(const std::string& project_path, int log_level, const std::string& prefix_msg, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    handeyecalib::printLog_EigenCalibMatrix(project_path, log_level, info_.eigen_cam_to_base, info_.eigen_base_to_cam, prefix_msg, color, file_name_path, func, line);
}

void CalibResReflector::printLog_TcpOffset(const std::string& project_path, int log_level, const std::string& prefix_msg, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    handeyecalib::printLog_TcpOffset(project_path, log_level, info_.offset_compensation, prefix_msg, color, file_name_path, func, line);
}

void CalibResReflector::printLog_HeadMotorAngles(const std::string& project_path, int log_level, const std::string& prefix_msg, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    handeyecalib::printLog_HeadMotorAngles(project_path, log_level, info_.head_motor_angles, prefix_msg, color, file_name_path, func, line);
}

//在加载配置文件后更新参数
void CalibResReflector::updateParamAftLoad()
{
    std::string class_name = basmodule::get_type_name(typeid(*this));
    std::string project_path = "hand_eye_calib/" + class_name;
    LOG_INFO(project_path, "开始更新标定结果参数...");
    syncOpencvToEigen(); 
    LOG_INFO(project_path, "同步OpenCV到Eigen矩阵完成");
    for (const auto& saved_param : saved_params_) // 更新info_中的实际值，使用非const引用以允许修改
    {
        handeyecalib::CalibRes::ParamName paramName = handeyecalib::CalibRes::getParamNameEnum(saved_param.name);
        std::string strVal;
        switch (paramName)
        {
        case handeyecalib::CalibRes::ParamName::CALIB_TYPE: 
            strVal = saved_param.getValue<std::string>();
            info_.calib_type = handeyecalib::CalibRes::getCalibTypeEnum(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::CAM_TO_BASE_TRANSFORM: 
            strVal = saved_param.getValue<std::string>();
            LOG_INFO(project_path, ("已更新标定结果参数: " + strVal).c_str());
            info_.cam_to_base_transform = handeyecalib::stringToMat(strVal);
            LOG_INFO(project_path, "已更新标定结果参数维度: cam_to_base_transform(%dx%d)",
                       info_.cam_to_base_transform.rows, info_.cam_to_base_transform.cols);
            break;
        case handeyecalib::CalibRes::ParamName::BASE_TO_CAM_TRANSFORM: 
            strVal = saved_param.getValue<std::string>();
            LOG_INFO(project_path, ("已更新标定结果参数: " + strVal).c_str());
            info_.base_to_cam_transform = handeyecalib::stringToMat(strVal);
            LOG_INFO(project_path, "已更新标定结果参数维度: base_to_cam_transform(%dx%d)",
                       info_.base_to_cam_transform.rows, info_.base_to_cam_transform.cols);
            break;
        case handeyecalib::CalibRes::ParamName::EIGEN_CAM_TO_BASE: 
            strVal = saved_param.getValue<std::string>();
            LOG_INFO(project_path, ("已更新标定结果参数: " + strVal).c_str());
            info_.eigen_cam_to_base = handeyecalib::stringToEigenMatrix4d(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::EIGEN_BASE_TO_CAM: 
            strVal = saved_param.getValue<std::string>();
            LOG_INFO(project_path, ("已更新标定结果参数: " + strVal).c_str());
            info_.eigen_base_to_cam = handeyecalib::stringToEigenMatrix4d(strVal);
            break;
        case handeyecalib::CalibRes::ParamName::OFFSET_COMPENSATION: 
            //已经通过调用datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
            break;
        case handeyecalib::CalibRes::ParamName::HEAD_MOTOR_ANGLES: 
            //已经通过调用datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
            break;
        default: 
            break;
        } 
    }
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(project_path, log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(project_path, log_level, "标定结果参数更新完成，参数信息如下：");
        printLog(project_path, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
}

// QualityMetricsReflector 实现
QualityMetricsReflector::QualityMetricsReflector(handeyecalib::QualityMetrics& info) 
    : info_(info) // 保存QualityMetrics引用
{
    initReflection(info);
}

void QualityMetricsReflector::initReflection(handeyecalib::QualityMetrics& info) 
{
    params_.clear();
    saved_params_.clear();
    // 注册所有参数
    registerParam(handeyecalib::QualityMetrics::getParamNameString(handeyecalib::QualityMetrics::ParamName::REPROJECTION_ERROR), info.reprojection_error);
    registerParam(handeyecalib::QualityMetrics::getParamNameString(handeyecalib::QualityMetrics::ParamName::TRANSLATION_ERROR), info.translation_error);
    registerParam(handeyecalib::QualityMetrics::getParamNameString(handeyecalib::QualityMetrics::ParamName::ROTATION_ERROR), info.rotation_error);
    registerParam(handeyecalib::QualityMetrics::getParamNameString(handeyecalib::QualityMetrics::ParamName::CONDITION_NUMBER), info.condition_number);
    registerParam(handeyecalib::QualityMetrics::getParamNameString(handeyecalib::QualityMetrics::ParamName::DATA_POINT_COUNT), info.data_point_count);
    
    // 填充需要从文件读取的参数列表
    saved_params_ = params_;
}

void QualityMetricsReflector::printLog(const std::string& project_path, int log_level, int color,  
    const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    bool bShowExMsg = false;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "QualityMetrics反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
}

//在加载配置文件后更新参数
void QualityMetricsReflector::updateParamAftLoad()
{
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

// CalibInfoReflector 实现
CalibInfoReflector::CalibInfoReflector(handeyecalib::CalibInfo& info) 
    : info_(info) // 保存CalibInfo引用
{
    initReflection(info);
}

void CalibInfoReflector::initReflection(handeyecalib::CalibInfo& info) 
{
    params_.clear();
    saved_params_.clear();
    // 注册所有参数
    registerParam(handeyecalib::CalibInfo::getParamNameString(handeyecalib::CalibInfo::ParamName::CALIB_METHOD), info.calib_method);
    registerParam(handeyecalib::CalibInfo::getParamNameString(handeyecalib::CalibInfo::ParamName::TIMESTAMP), info.timestamp);
    
    #if 0
    // 使用CalibResReflector获取CalibRes的参数
    handeyecalib::CalibResReflector calibResReflector(const_cast<handeyecalib::CalibRes&>(info.calib_res));
    const auto& calibResParams = calibResReflector.getParams();
    for (const auto& param : calibResParams) // 将CalibResReflector的参数添加到当前params_中
    {
        param.name = ".calib_res." + param.name;
        params_.push_back(param);
    }
    // 使用QualityMetricsReflector获取QualityMetrics的参数
    handeyecalib::QualityMetricsReflector qualityMetricsReflector(const_cast<handeyecalib::QualityMetrics&>(info.quality_metrics));
    const auto& qualityMetricsParams = qualityMetricsReflector.getParams();
    for (const auto& param : qualityMetricsParams) // 将QualityMetricsReflector的参数添加到当前params_中 
    {
        param.name = ".quality_metrics." + param.name;
        params_.push_back(param);
    }    
    for (const auto& param : params_) // 填充需要从文件读取的参数列表
    {
        saved_params_.push_back(param);
    }
    #endif
    saved_params_ = params_;
}

void CalibInfoReflector::printLog(const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "CalibInfo反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
    handeyecalib::CalibResReflector calibResReflector(const_cast<handeyecalib::CalibRes&>(info_.calib_res));
    calibResReflector.printLog(project_path, log_level, color, file_name_path, func, line);
    handeyecalib::QualityMetricsReflector qualityMetricsReflector(const_cast<handeyecalib::QualityMetrics&>(info_.quality_metrics));
    qualityMetricsReflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

//在加载配置文件后更新参数
void CalibInfoReflector::updateParamAftLoad()
{
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

// ArmCalibInfoReflector 实现
ArmCalibInfoReflector::ArmCalibInfoReflector(handeyecalib::ArmCalibInfo& info) 
    : info_(info) // 保存ArmCalibInfo引用
{
    initReflection(info);
}

void ArmCalibInfoReflector::initReflection(handeyecalib::ArmCalibInfo& info) 
{
    params_.clear();
    saved_params_.clear();
    // 注册所有参数
    registerParam(handeyecalib::ArmCalibInfo::getParamNameString(handeyecalib::ArmCalibInfo::ParamName::ARM_ID), info.arm_id);
    #if 0
    // 使用CalibInfoReflector获取CalibInfo的参数
    handeyecalib::CalibInfoReflector calibInfoReflector(const_cast<handeyecalib::CalibInfo&>(info.calib_info));
    const auto& calibInfoParams = calibInfoReflector.getParams();
    for (const auto& param : calibInfoParams) // 将CalibInfoReflector的参数添加到当前params_中
    {
        param.name = ".calib_info." + param.name;
        params_.push_back(param);
    }
    #endif
    for (const auto& param : params_) // 填充需要从文件读取的参数列表
    {
        if (param.name != handeyecalib::ArmCalibInfo::getParamNameString(handeyecalib::ArmCalibInfo::ParamName::ARM_ID)) 
        {
            saved_params_.push_back(param);
        }
    }
}

 // 打印日志输出函数
void ArmCalibInfoReflector::printLog_armCalibInfo(const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "ArmCalibInfo反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
    handeyecalib::CalibInfoReflector calibInfoReflector(const_cast<handeyecalib::CalibInfo&>(info_.calib_info));
    calibInfoReflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

//在加载配置文件后更新参数
void ArmCalibInfoReflector::updateParamAftLoad()
{
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

// CamCalibInfoReflector 实现
CamCalibInfoReflector::CamCalibInfoReflector(handeyecalib::CamCalibInfo& info) 
    : info_(info) // 保存CamCalibInfo引用
{
    initReflection(info);
}

void CamCalibInfoReflector::initReflection(handeyecalib::CamCalibInfo& info) 
{
    params_.clear();
    saved_params_.clear();
    registerParam(handeyecalib::CamCalibInfo::getParamNameString(handeyecalib::CamCalibInfo::ParamName::CAM_ID), info.cam_id);
    
    // 注意：arm_calib1D是复杂类型，这里记录其机械臂ID列表信息
    // 创建一个特殊的ParamInfo来存储arm_calib1D的机械臂ID列表
    ParamInfo arm_ids_para;
    arm_ids_para.name = handeyecalib::CamCalibInfo::getParamNameString(handeyecalib::CamCalibInfo::ParamName::ARM_IDS);
    arm_ids_para.type = ParamType::UINT8_ARRAY; // 使用UINT8_ARRAY类型来存储ID列表
    arm_ids_para.ptr = nullptr; // 不指向实际内存地址
    std::vector<uint8_t> arm_ids = handeyecalib::getArmIds(info.arm_calib1D);// 获取机械臂ID列表
    arm_ids_para.value = arm_ids;
    params_.push_back(arm_ids_para);

    // 填充需要从文件读取的参数列表
    // 排除CAM_ID，因为它通常由系统分配
    for (const auto& param : params_) 
    {
        if (param.name != handeyecalib::CamCalibInfo::getParamNameString(handeyecalib::CamCalibInfo::ParamName::CAM_ID) 
        && param.name != handeyecalib::CamCalibInfo::getParamNameString(handeyecalib::CamCalibInfo::ParamName::ARM_IDS)) 
        {
            saved_params_.push_back(param);
        }
    }
}

// CamCalibInfoReflector printLog实现
void CamCalibInfoReflector::printLog(const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line) const
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    false, static_cast<logsys::Color>(color), "CamCalibInfo反射器参数信息:");
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);
    for (const auto& pair : info_.arm_calib1D) // 打印每个机械臂的标定信息
    {
        const handeyecalib::ArmCalibInfo& arm_calib_info = pair.second;
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "机械臂arm_id= %d 的ArmCalibInfo反射器参数信息:", arm_calib_info.arm_id);
        handeyecalib::ArmCalibInfoReflector arm_reflector(const_cast<handeyecalib::ArmCalibInfo&>(arm_calib_info));
        arm_reflector.printLog(project_path, log_level, color, file_name_path, func, line); // 调用机械臂标定信息打印函数
    }
}

//在加载配置文件后更新参数
void CamCalibInfoReflector::updateParamAftLoad()
{
    datahandler::ConfigReflector::updateParamAftLoad();// 调用基类实现
}

void printLog_CalibRes(const CalibRes& res, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    handeyecalib::CalibResReflector reflector(const_cast<handeyecalib::CalibRes&>(res));
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "开始打印CalibRes反射器参数信息:");
    reflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

// 打印单个QualityMetrics的函数
void printLog_QualityMetrics(const handeyecalib::QualityMetrics& quality_metrics, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    // 创建QualityMetricsReflector实例并打印
    handeyecalib::QualityMetricsReflector reflector(const_cast<handeyecalib::QualityMetrics&>(quality_metrics));
    reflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

void printLog_CalibInfo(const handeyecalib::CalibInfo& calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    handeyecalib::CalibInfoReflector reflector(const_cast<handeyecalib::CalibInfo&>(calib_info));
    reflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

// 打印单个ArmCalibInfo的函数
void printLog_ArmCalibInfo(const handeyecalib::ArmCalibInfo& arm_calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    handeyecalib::ArmCalibInfoReflector reflector(const_cast<handeyecalib::ArmCalibInfo&>(arm_calib_info));
    reflector.printLog(project_path, log_level, color, file_name_path, func, line);
}

// 打印ArmCalibInfoList的函数
void printLog_ArmCalibInfoList(const handeyecalib::ArmCalibInfoList& arm_calib_info_list, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "ArmCalibInfoList - 机械臂数量: %d", static_cast<int>(arm_calib_info_list.size()));
    std::vector<uint8_t> arm_ids;
    for (const auto& pair : arm_calib_info_list) {
        arm_ids.push_back(pair.first);
    }
    std::string arm_ids_str = basmodule::get_list_string(arm_ids);
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "ArmCalibInfoList - 机械臂ID列表: %s", arm_ids_str.c_str());
    for (const auto& pair : arm_calib_info_list) 
    {
        printLog_ArmCalibInfo(pair.second, project_path, log_level, color, file_name_path, func, line);
    }
    std::cout << std::endl; // 输出空行，确保下次终端输出时能空一行再显示
}

// 打印单个CamCalibInfo的函数
void printLog_CamCalibInfo(const handeyecalib::CamCalibInfo& cam_calib_info, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "当前相机cam_id= %d 的CamCalibInfoList信息：", cam_calib_info.cam_id);
    handeyecalib::CamCalibInfoReflector reflector(const_cast<handeyecalib::CamCalibInfo&>(cam_calib_info));
    reflector.printLog(project_path, log_level, color, file_name_path, func, line);// 调用单个相机标定信息打印函数
}

// 打印CamCalibInfoList的函数
void printLog_CamCalibInfoList(const handeyecalib::CamCalibInfoList& cam_calib_info_list, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "CamCalibInfoList - 相机数量: %d", static_cast<int>(cam_calib_info_list.size()));
    std::vector<uint8_t> cam_ids;
    for (const auto& cam_calib_info : cam_calib_info_list) 
    {
        cam_ids.push_back(cam_calib_info.cam_id);
    }
    std::string cam_ids_str = basmodule::get_list_string(cam_ids);
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        false, static_cast<logsys::Color>(color), "CamCalibInfoList - 相机ID列表: %s", cam_ids_str.c_str());
    for (const auto& cam_calib_info : cam_calib_info_list) 
    {
        printLog_CamCalibInfo(cam_calib_info, project_path, log_level, color, file_name_path, func, line);
        std::cout << std::endl; // 输出空行，确保下次终端输出时能空一行再显示
    }
}

} // namespace handeyecalib