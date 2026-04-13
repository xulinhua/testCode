#include "calib_data_operate.hpp"
#include "hand_eye_calib/calib_reflector.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>
#include "log_system/log_macros.hpp"
#include "internal_constants.h"
#include "bas_operate/bas_utils.hpp"
#include "bas_operate/file_operate.hpp"
#include "data_handler/json_operate.h"
#include "data_handler/yaml_operate.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace handeyecalib {


// 获取指定相机和机械臂的标定数据文件夹路径
std::string getCalibDataDirPath(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id)
{
    return bas_config_data_path + "/cam_config/cam_" + 
           std::to_string(cam_id) + "/cam" + std::to_string(cam_id) + 
           "_arm" + std::to_string(arm_id);
}

// 获取指定相机和机械臂的偏移补偿参数文件路径
std::string getTcpOffsetFilePath(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id)
{
    return getCalibDataDirPath(bas_config_data_path, cam_id, arm_id) + "/tcp_offset_cam" + 
           std::to_string(cam_id) + "_arm" + std::to_string(arm_id) + ".yaml";
}

/**
 * @brief 获取指定相机和机械臂的质量评估指标文件路径
 * @param bas_config_data_path 基础配置数据路径
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 质量评估指标文件路径
 */
std::string getQualityMetricsFilePath(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id)
{
    return getCalibDataDirPath(bas_config_data_path, cam_id, arm_id) + "/calib_quality_cam" + 
           std::to_string(cam_id) + "_arm" + std::to_string(arm_id) + ".json";
}

// 普通接口函数，用于获取指定相机和机械臂的标定结果文件路径
std::string getCalibResFilePath(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id)
{
    return getCalibDataDirPath(bas_config_data_path, cam_id, arm_id) + "/calib_result_cam" + 
           std::to_string(cam_id) + "_arm" + std::to_string(arm_id) + ".json";
}

/**
 * @brief 加载标定结果从文件
 * @param data JSON数据对象
 * @param res 标定结果（输出）
 * @return 是否加载成功
 */
bool loadCalibResFromJson(const nlohmann::json& data, CalibRes& res) 
{
    bool bRet = false;
    try 
    {
        // 检查必要字段是否存在
        if (!data.contains(CAM_TO_BASE_NAME) || !data.contains(BASE_TO_CAM_NAME) ||
            !data[CAM_TO_BASE_NAME].contains(MATRIX_NAME) || !data[BASE_TO_CAM_NAME].contains(MATRIX_NAME)) 
        {
            LOG_ERROR("❌ 标定结果文件格式不正确");
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前标定结果信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        // 先直接加载变换矩阵，避免反射器覆盖加载的值
        bRet = datahandler::loadMatrix4x4FromJson(data, CAM_TO_BASE_NAME, MATRIX_NAME, res.cam_to_base_transform);//加载相机到基座变换矩阵
        bRet = datahandler::loadMatrix4x4FromJson(data, BASE_TO_CAM_NAME, MATRIX_NAME, res.base_to_cam_transform);//加载基座到相机变换矩阵
        CalibResReflector reflector(res);// 使用反射器机制加载常规参数
        const auto& params_saved = reflector.getParamsSaved();
        for (const auto& param_info : params_saved)
        {
            // 跳过需要特殊处理的参数（变换矩阵）或者不保存在当前文件中的参数（如Eigen矩阵、offset_compensation）
            if (param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::CAM_TO_BASE_TRANSFORM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::BASE_TO_CAM_TRANSFORM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::EIGEN_CAM_TO_BASE) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::EIGEN_BASE_TO_CAM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::OFFSET_COMPENSATION))
            {
                continue;
            }
            bRet = loadParamFromJson(data, const_cast<datahandler::ParamInfo&>(param_info));// 加载其他参数
        }        
        LOG_INFO("✅ 标定结果已从JSON对象加载");
        reflector.updateParamAftLoad();// 调用反射器更新参数    
        LOG_INFO("✅ 反射器CalibResReflector更新参数完成");
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载后标定结果信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("✅ 标定结果已从JSON对象成功加载！");
        return true;   
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载标定结果失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 加载标定结果失败: 未知错误");
        return false;
    }
}

/**
 * @brief 保存标定结果到JSON对象
 * @param data JSON数据对象
 * @param res 标定结果
 * @return 是否保存成功
 */
bool saveCalibResToJson(nlohmann::json& data, const CalibRes& res)
{
    try 
    {
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前标定结果信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        // 先直接保存变换矩阵，避免反射器覆盖保存的值
        bool bRet = datahandler::saveMatrix4x4ToJson(data, CAM_TO_BASE_NAME, MATRIX_NAME, res.cam_to_base_transform);// 保存相机到基座变换矩阵
        if (!bRet)
        {
            LOG_ERROR("❌ 保存4x4变换矩阵失败: %s", CAM_TO_BASE_NAME);
            return false;
        }
        bRet = datahandler::saveMatrix4x4ToJson(data, BASE_TO_CAM_NAME, MATRIX_NAME, res.base_to_cam_transform);// 保存基座到相机变换矩阵
        if (!bRet)
        {
            LOG_ERROR("❌ 保存4x4变换矩阵失败: %s", BASE_TO_CAM_NAME);
            return false;
        }
        CalibResReflector reflector(const_cast<CalibRes&>(res));// 使用反射器机制保存常规参数
        const auto& params_saved = reflector.getParamsSaved();
        for (const auto& param_info : params_saved)
        {
            // 跳过需要特殊处理的参数（变换矩阵）或者不保存在当前文件中的参数（如Eigen矩阵、offset_compensation）
            if (param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::CAM_TO_BASE_TRANSFORM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::BASE_TO_CAM_TRANSFORM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::EIGEN_CAM_TO_BASE) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::EIGEN_BASE_TO_CAM) ||
                param_info.name == CalibRes::getParamNameString(CalibRes::ParamName::OFFSET_COMPENSATION))
            {
                continue;
            }
            bRet = datahandler::saveParamToJson(data, param_info);// 保存其他参数
            if (!bRet)
            {
                LOG_ERROR("❌ 保存参数失败: %s", param_info.name.c_str());
                return false;
            }
        }
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存后标定结果信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("✅ 标定结果已成功保存到JSON对象！");
        return true;   
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存标定结果失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 保存标定结果失败: 未知错误");
        return false;
    }
}

/**
 * @brief 从JSON对象中加载质量评估指标
 * @param data JSON数据对象
 * @param quality_metrics 质量评估指标(输出)
 * @return 是否加载成功
 * 该函数从JSON对象中提取质量评估指标数据,包括重投影误差、平移误差、旋转误差、条件数和数据点数
 */
bool loadQualityMetricsFromJson(const nlohmann::json& data, QualityMetrics& quality_metrics)
{
    try
    {
        bool bRet = true;
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前质量评估指标信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (data.contains(QUALITY_METRICS_NAME)) // 检查是否有质量评估指标
        {
            auto metrics_data = data[QUALITY_METRICS_NAME];
            QualityMetricsReflector reflector(quality_metrics);// 使用反射器机制加载质量评估指标参数
            const auto& params_saved = reflector.getParamsSaved();
            for (const auto& param_info : params_saved)
            {
                bRet =loadParamFromJson(metrics_data, const_cast<datahandler::ParamInfo&>(param_info));
                if (!bRet)
                {
                    LOG_WARN("❌ 加载参数失败: %s", param_info.name.c_str());
                    break;
                }
            }
            reflector.updateParamAftLoad(); // 调用反射器更新参数
            LOG_INFO("✅ 反射器QualityMetricsReflector更新参数完成");
            if (LOG_ON(log_level))
            {
                LOG_OUT(log_level, "📊 加载后质量评估指标信息:");
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
            }
            LOG_INFO("✅ 质量评估指标已加载完成！\n");
            return bRet;
        }
        return bRet;
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载标定质量评估指标失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 加载标定质量评估指标失败: 未知错误");
        return false;
    }
}

/**
 * @brief 保存质量评估指标到JSON对象
 * @param data JSON数据对象
 * @param quality_metrics 质量评估指标
 * @return 是否保存成功
 * 该函数将质量评估指标保存到JSON对象中,包括重投影误差、平移误差、旋转误差、条件数和数据点数
 */
bool saveQualityMetricsToJson(nlohmann::json& data, const QualityMetrics& quality_metrics)
{
    try 
    {
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前质量评估指标信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        json quality_metrics_obj;
        QualityMetricsReflector reflector(const_cast<QualityMetrics&>(quality_metrics));// 使用反射器机制保存质量评估指标参数
        const auto& params_saved = reflector.getParamsSaved();
        bool bRet = true;
        for (const auto& param_info : params_saved)
        {
            bRet = datahandler::saveParamToJson(quality_metrics_obj, param_info);// 保存质量评估指标参数
            if (!bRet)
            {
                LOG_ERROR("❌ 保存质量评估指标参数失败: %s", param_info.name.c_str());
                return false;
            }
        }
        data[QUALITY_METRICS_NAME] = quality_metrics_obj;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存后质量评估指标信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("✅ 质量评估指标已成功保存到JSON对象！\n");    
        return true;
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存质量评估指标失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 保存质量评估指标失败: 未知错误");
        return false;
    }
}

/**
 * @brief 从配置文件加载TCP偏移数据
 * @param config_file_path 配置文件路径
 * @param offset_compensation 偏移补偿参数（输出）
 * @return 是否加载成功
 */
bool loadTcpOffsetFromYaml(const std::string& config_file_path, std::vector<double>& offset_compensation)
{
    try 
    {
        YAML::Node config;
        if (!datahandler::loadYamlFile(config_file_path, config)) // 使用统一的YAML加载函数
        {
            LOG_ERROR("加载偏移补偿参数文件失败: %s", config_file_path.c_str());
            return false;
        }
        bool enable = config[TCP_OFFSET_ENABLE].as<bool>(true);// 先读取启用状态
        if (enable)
        {// 读取偏移补偿参数
            double offset_x = config[TCP_OFFSET_X].as<double>(0.0);
            double offset_y = config[TCP_OFFSET_Y].as<double>(0.0);
            double offset_z = config[TCP_OFFSET_Z].as<double>(0.0);
            double offset_rx = config[TCP_OFFSET_RX].as<double>(0.0);
            double offset_ry = config[TCP_OFFSET_RY].as<double>(0.0);
            double offset_rz = config[TCP_OFFSET_RZ].as<double>(0.0);
            offset_compensation = {offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz};
            CalibRes temp_calib_res;
            temp_calib_res.offset_compensation = offset_compensation;
            std::string offset_info;
            if(temp_calib_res.getTcpOffset(offset_info))
            {
                LOG_INFO(true, logsys::Color::BLUE, "成功加载TCP偏移补偿参数 (%s):\n%s", enable ? "启用" : "禁用", offset_info.c_str());
            }
            else
            {
                LOG_INFO("成功加载TCP偏移补偿参数: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] (%s)", 
                    offset_compensation[0], offset_compensation[1], offset_compensation[2],
                    offset_compensation[3], offset_compensation[4], offset_compensation[5], enable ? "启用" : "禁用");
            }
        }
        else
        {// 如果禁用，偏移补偿值直接置为0
            offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // 使用CalibRes类的getTcpOffset接口输出详细日志
            CalibRes temp_calib_res;
            temp_calib_res.offset_compensation = offset_compensation;
            std::string offset_info;
            if(temp_calib_res.getTcpOffset(offset_info))
            {
                LOG_WARN("⚠️ TCP偏移补偿参数已被禁用，所有补偿值设为0:\n%s", offset_info.c_str());
            }
            else
            {
                LOG_WARN("⚠️ TCP偏移补偿参数已被禁用，所有补偿值设为0: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                    offset_compensation[0], offset_compensation[1], offset_compensation[2],
                    offset_compensation[3], offset_compensation[4], offset_compensation[5]);
            }
        }
        return true;
    } 
    catch (const YAML::Exception& e) 
    {
        LOG_ERROR("加载TCP偏移补偿参数时YAML解析异常: %s", e.what());
        return false;
    }
    catch (const std::exception& e) 
    {
        LOG_ERROR("加载TCP偏移补偿参数时发生异常: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("加载TCP偏移补偿参数时发生未知异常");
        return false;
    }
}

/**
 * @brief 保存TCP偏移数据到配置文件
 * @param config_file_path 配置文件路径
 * @param offset_compensation 偏移补偿参数
 * @return 是否保存成功
 */
bool saveTcpOffsetToYaml(const std::string& config_file_path, const std::vector<double>& offset_compensation)
{
    try 
    {
        YAML::Node config;// 创建YAML节点
        config[TCP_OFFSET_ENABLE] = true;
        if (offset_compensation.size() >= 6) 
        {
            config[TCP_OFFSET_X] = offset_compensation[0];
            config[TCP_OFFSET_Y] = offset_compensation[1];
            config[TCP_OFFSET_Z] = offset_compensation[2];
            config[TCP_OFFSET_RX] = offset_compensation[3];
            config[TCP_OFFSET_RY] = offset_compensation[4];
            config[TCP_OFFSET_RZ] = offset_compensation[5];
        } 
        else 
        {
            LOG_WARN("偏移补偿参数数量不足6个，使用默认值");
            // 使用默认值
            config[TCP_OFFSET_X] = 0.0;
            config[TCP_OFFSET_Y] = 0.0;
            config[TCP_OFFSET_Z] = 0.0;
            config[TCP_OFFSET_RX] = 0.0;
            config[TCP_OFFSET_RY] = 0.0;
            config[TCP_OFFSET_RZ] = 0.0;
        }
        if (!datahandler::saveYamlFile(config_file_path, config))
        {
            LOG_ERROR("保存TCP偏移补偿参数文件失败: %s", config_file_path.c_str());
            return false;
        }
        LOG_INFO("成功保存TCP偏移补偿参数到文件: %s", config_file_path.c_str());
        return true;
    } 
    catch (const YAML::Exception& e) 
    {
        LOG_ERROR("保存TCP偏移补偿参数时YAML序列化异常: %s", e.what());
        return false;
    }
    catch (const std::exception& e) 
    {
        LOG_ERROR("保存TCP偏移补偿参数时发生异常: %s", e.what());
        return false;
    }
}

// 加载指定相机和机械臂的偏移补偿参数
bool loadTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res)
{
    bool bRet = false;
    try 
    {
        std::string offset_file_path = getTcpOffsetFilePath(bas_config_data_path, cam_id, arm_id);// 构造偏移补偿参数文件路径
        LOG_INFO("偏移补偿参数文件路径: %s", offset_file_path.c_str());
        if (!fs::exists(offset_file_path)) // 检查文件是否存在
        {// 如果文件不存在，使用默认值并创建默认文件
            calib_res.offset_compensation = std::vector<double>(6, 0.0);
            LOG_WARN("相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数文件不存在: %s，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id), offset_file_path.c_str());
            bRet = false;  
            // 创建默认的偏移补偿参数文件
            std::string offset_file_path = getTcpOffsetFilePath(bas_config_data_path, cam_id, arm_id);
            if (!saveTcpOffsetToYaml(offset_file_path, calib_res.offset_compensation))
            {
                LOG_ERROR("创建相机cam_id= %d 下机械臂arm_id= %d 的默认偏移补偿参数文件失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), offset_file_path.c_str());
                bRet = false;  
            }
        }
        else
        {
            std::vector<double> offset_compensation;// 加载偏移补偿参数
            if (loadTcpOffsetFromYaml(offset_file_path, offset_compensation))
            {
                calib_res.offset_compensation = offset_compensation;
                LOG_INFO("成功加载相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数:", static_cast<int>(cam_id), static_cast<int>(arm_id));
                bRet = true;       
            }
            else
            {
                calib_res.offset_compensation = std::vector<double>(6, 0.0);
                LOG_WARN("加载相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数文件失败: %s，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id), offset_file_path.c_str());
                bRet = false;  
            }
        }
    } 
    catch (const std::exception& e) 
    {
        bRet = false;  
        calib_res.offset_compensation = std::vector<double>(6, 0.0);// 使用默认值
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数时发生异常: %s，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
    } catch (...) 
    {
        bRet = false;
        calib_res.offset_compensation = std::vector<double>(6, 0.0);// 使用默认值
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d 的偏移补偿参数时发生未知异常，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id));
    }
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "完成加载相机cam_id= %d 和机械臂arm_id= %d 的偏移补偿参数：", static_cast<int>(cam_id), static_cast<int>(arm_id));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        handeyecalib::printLog_TcpOffset(project_name, (int)log_level, calib_res.offset_compensation, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    return bRet;        
}

// 保存指定相机和机械臂的偏移补偿参数
bool saveTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res)
{
    try 
    {
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "当前要保存的相机cam_id= %d 和机械臂arm_id= %d 的偏移补偿参数：", static_cast<int>(cam_id), static_cast<int>(arm_id));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_TcpOffset(project_name, (int)log_level, calib_res.offset_compensation, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        std::string offset_file_path = getTcpOffsetFilePath(bas_config_data_path, cam_id, arm_id);// 构造偏移补偿参数文件路径
        return saveTcpOffsetToYaml(offset_file_path, calib_res.offset_compensation);
    } catch (const YAML::Exception& e) 
    {
        LOG_ERROR("保存相机%d下机械臂%d的偏移补偿参数时发生YAML序列化异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
    catch (const std::exception& e) 
    {
        LOG_ERROR("保存相机%d下机械臂%d的偏移补偿参数时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
}

bool loadQualityMetrics(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, QualityMetrics& quality_metrics)
{
    bool bRet = false;
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    try 
    {
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标：", static_cast<int>(cam_id), static_cast<int>(arm_id));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        std::string quality_metrics_file_path = getQualityMetricsFilePath(bas_config_data_path, cam_id, arm_id);// 构造质量评估指标文件路径
        LOG_INFO("标定质量评估指标文件路径: %s", quality_metrics_file_path.c_str());
        if (!fs::exists(quality_metrics_file_path)) // 检查文件是否存在
        {
            // 如果文件不存在，使用默认值并创建默认文件
            quality_metrics.Init(); // 初始化质量评估指标为默认值
            LOG_WARN("相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标文件不存在: %s，使用默认值:", 
                static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
            bRet = false;  
            if (!saveQualityMetrics(bas_config_data_path, cam_id, arm_id, quality_metrics))// 创建默认的标定质量评估指标文件
            {
                LOG_ERROR("创建相机cam_id= %d 下机械臂arm_id= %d 的默认标定质量评估指标文件失败: %s", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
                bRet = false;  
            }
        }
        else
        {
            json data; // 加载质量评估指标
            if (datahandler::loadJsonFile(quality_metrics_file_path, data))
            {
                if (loadQualityMetricsFromJson(data, quality_metrics))
                {
                    LOG_INFO("成功加载相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标:", static_cast<int>(cam_id), static_cast<int>(arm_id));
                    bRet = true;       
                }
                else
                {
                    quality_metrics.Init(); // 初始化质量评估指标为默认值
                    LOG_WARN("加载相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标文件失败: %s，使用默认值:", 
                        static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
                    bRet = false;  
                }
            }
            else
            {
                quality_metrics.Init(); // 初始化质量评估指标为默认值
                LOG_WARN("加载相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标文件失败: %s，使用默认值:", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
                bRet = false;  
            }
        }
    } 
    catch (const std::exception& e) 
    {
        bRet = false;  
        quality_metrics.Init(); // 初始化质量评估指标为默认值
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标时发生异常: %s，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
    } catch (...) 
    {
        bRet = false;
        quality_metrics.Init(); // 初始化质量评估指标为默认值
        LOG_ERROR("加载相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标时发生未知异常，使用默认值:", static_cast<int>(cam_id), static_cast<int>(arm_id));
    }
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "📊 加载后相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标：", static_cast<int>(cam_id), static_cast<int>(arm_id));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        handeyecalib::printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    return bRet;        
}

bool saveQualityMetrics(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const QualityMetrics& quality_metrics)
{
    try 
    {
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标：", static_cast<int>(cam_id), static_cast<int>(arm_id));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        std::string quality_metrics_file_path = getQualityMetricsFilePath(bas_config_data_path, cam_id, arm_id);// 构造质量评估指标文件路径
        LOG_INFO("相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标文件路径: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
        std::string quality_metrics_dir_path = getCalibDataDirPath(bas_config_data_path, cam_id, arm_id);// 创建目录（如果不存在）
        fs::create_directories(fs::path(quality_metrics_dir_path));
        json data;// 构建JSON数据
        if (saveQualityMetricsToJson(data, quality_metrics)) // 保存质量评估指标到JSON数据
        {
            if (datahandler::saveJsonFile(quality_metrics_file_path, data)) // 保存JSON数据到文件
            {
                LOG_INFO("✅ 相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标已成功保存到 %s 文件！", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
                if (LOG_ON(log_level))
                {
                    LOG_OUT(log_level, "📊 保存后相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标：", static_cast<int>(cam_id), static_cast<int>(arm_id));
                    const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                    handeyecalib::printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
                }
                return true;
            }
            else
            {
                LOG_ERROR("❌ 保存相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标到 %s 文件失败！", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
                return false;
            }
        }
        else
        {
            LOG_ERROR("❌ 保存相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标到 %s 文件失败！", 
                static_cast<int>(cam_id), static_cast<int>(arm_id), quality_metrics_file_path.c_str());
            return false;
        }
    } catch (const std::exception& e) 
    {
        LOG_ERROR("保存相机cam_id= %d 下机械臂arm_id= %d 的标定质量评估指标时发生异常: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    }
}

bool loadCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res)
{
    bool bRet = false;
    std::string calib_res_file_path = getCalibResFilePath(bas_config_data_path, cam_id, arm_id);// 构造标定结果文件路径
    try 
    {
        LOG_INFO("尝试加载相机cam_id= %d 下机械臂arm_id= %d 的标定结果文件路径: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
        json data;// 读取JSON文件
        if (!datahandler::loadJsonFile(calib_res_file_path, data))
        {
            LOG_ERROR("❌ 加载相机cam_id= %d 下机械臂arm_id= %d 的标定结果文件失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前标定结果:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(calib_res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (handeyecalib::loadCalibResFromJson(data, calib_res)) // 从JSON数据载基础的标定结果
        {
            if (loadTcpOffset(bas_config_data_path, cam_id, arm_id, calib_res))// 从YAML文件中加载偏移补偿参数
            {
                LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的偏移补偿参数", static_cast<int>(cam_id), static_cast<int>(arm_id));
            }
            else
            {
                LOG_WARN("⚠️  加载偏移补偿参数失败，使用默认值: %s", calib_res_file_path.c_str());
            }
            LOG_INFO("✅ 标定结果已从 %s 文件中加载成功！", calib_res_file_path.c_str());
            if (LOG_ON(log_level))
            {
                LOG_OUT(log_level, "📊 加载后标定结果:");
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                printLog_CalibRes(calib_res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
            }
            return true;
        }
        else
        {
            LOG_ERROR("❌ 从文件 %s 中加载相机cam_id=%d 下机械臂arm_id=%d 的标定结果数据失败！", 
                calib_res_file_path.c_str(), static_cast<int>(cam_id), static_cast<int>(arm_id));
            return false;
        }
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 从文件 %s 中加载相机cam_id=%d 下机械臂arm_id=%d 的标定结果失败: %s", 
            calib_res_file_path.c_str(), static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 从文件 %s 中加载相机cam_id=%d 下机械臂arm_id=%d 的标定结果失败: 未知错误", 
            calib_res_file_path.c_str(), static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
}

// 保存指定相机和机械臂的标定结果
bool saveCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res)
{
    bool bRet = false;
    std::string calib_res_file_path = getCalibResFilePath(bas_config_data_path, cam_id, arm_id);// 构造标定结果文件路径
    try 
    {
        LOG_INFO("尝试保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果文件路径: %s", 
            static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
        std::string calib_dir_path = getCalibDataDirPath(bas_config_data_path, cam_id, arm_id);// 创建目录（如果不存在）
        fs::create_directories(fs::path(calib_dir_path));
        json data;// 构建JSON数据
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前标定结果:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CalibRes(calib_res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (handeyecalib::saveCalibResToJson(data, calib_res)) // 保存基础的标定结果到JSON数据
        {
            if (datahandler::saveJsonFile(calib_res_file_path, data)) // 保存JSON数据到文件
            {
                if (saveTcpOffset(bas_config_data_path, cam_id, arm_id, calib_res))// 保存偏移补偿参数到YAML文件
                {
                    LOG_INFO("成功保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果偏移补偿参数", static_cast<int>(cam_id), static_cast<int>(arm_id));
                }
                else
                {
                    LOG_WARN("⚠️  保存偏移补偿参数失败: %s", calib_res_file_path.c_str());
                }
                LOG_INFO("✅ 相机cam_id=%d 下机械臂arm_id=%d 的标定结果已成功保存到 %s 文件！", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
                if (LOG_ON(log_level))
                {
                    LOG_OUT(log_level, "📊 保存后标定结果:");
                    const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                    printLog_CalibRes(calib_res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
                }
                return true;
            }
            else
            {
                LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果到 %s 文件失败！", 
                    static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
                return false;
            }
        }
        else
        {
            LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果到 %s 文件失败！", 
                static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
            return false;
        }
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果到文件 %s 失败: %s", 
            static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str(), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果到文件 %s 失败: 未知错误", 
            static_cast<int>(cam_id), static_cast<int>(arm_id), calib_res_file_path.c_str());
        return false;
    }
}

/**
 * @brief 从JSON文件加载标定信息
 * @param bas_config_data_path 基础配置数据路径
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @param calib_info 标定信息（输出）
 * @return 是否加载成功
 *
 * 该函数从JSON文件中加载标定信息，包括标定结果、质量评估指标等。
 * 注意：加载的CalibInfo信息和CalibRes实际上是存储在同一个文件calib_result_cam0_arm0.json中的
 */
bool loadCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibInfo& calib_info)
{
    try 
    {
        LOG_INFO("尝试加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前标定信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_CalibInfo(calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (handeyecalib::loadCalibRes(bas_config_data_path, cam_id, arm_id, calib_info.calib_res))// 先加载标定结果
        {
            LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的标定结果", static_cast<int>(cam_id), static_cast<int>(arm_id));
        }
        else
        {
            LOG_WARN("⚠️  加载相机cam_id=%d 下机械臂arm_id=%d 的标定结果失败，使用默认值。", static_cast<int>(cam_id), static_cast<int>(arm_id));
        }
        if (handeyecalib::loadQualityMetrics(bas_config_data_path, cam_id, arm_id, calib_info.quality_metrics))// 加载标定质量评估指标
        {
            LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的标定质量评估指标", static_cast<int>(cam_id), static_cast<int>(arm_id));
        }
        else
        {
            LOG_WARN("⚠️  加载相机cam_id=%d 下机械臂arm_id=%d 的标定质量评估指标失败，使用默认值。", static_cast<int>(cam_id), static_cast<int>(arm_id));
            calib_info.quality_metrics.Init();// 使用默认值
        }
        LOG_INFO("✅ 相机cam_id=%d 下机械臂arm_id=%d 的标定信息已加载完成！", static_cast<int>(cam_id), static_cast<int>(arm_id));
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载后标定信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_CalibInfo(calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        return true;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败: 未知错误", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
}


/**
 * @brief 保存标定信息到JSON文件
 * @param bas_config_data_path 基础配置数据路径
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @param calib_info 标定信息
 * @return 是否保存成功
 *
 * 该函数将标定信息保存到JSON文件中，包括标定结果、质量评估指标等。
 * 注意：保存的CalibInfo信息和CalibRes实际上是存储在同一个文件calib_result_cam0_arm0.json中的
 */
bool saveCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibInfo& calib_info)
{
    try 
    {
        LOG_INFO("尝试保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前标定信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_CalibInfo(calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (handeyecalib::saveCalibRes(bas_config_data_path, cam_id, arm_id, calib_info.calib_res))// 保存标定结果
        {
            LOG_INFO("成功保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果", static_cast<int>(cam_id), static_cast<int>(arm_id));
        }
        else
        {
            LOG_ERROR("⚠️  保存相机cam_id=%d 下机械臂arm_id=%d 的标定结果失败！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            return false; // 如果标定结果保存失败，则整个保存过程失败
        }
        if (handeyecalib::saveQualityMetrics(bas_config_data_path, cam_id, arm_id, calib_info.quality_metrics))// 保存质量评估指标
        {
            LOG_INFO("成功保存相机cam_id=%d 下机械臂arm_id=%d 的质量评估指标！", static_cast<int>(cam_id), static_cast<int>(arm_id));
        }
        else
        {
            LOG_WARN("⚠️  保存相机cam_id=%d 下机械臂arm_id=%d 的质量评估指标失败！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            // 质量评估指标保存失败不视为致命错误，继续执行
        }
        LOG_INFO("✅ 相机cam_id=%d 下机械臂arm_id=%d 的标定信息已保存完成！", static_cast<int>(cam_id), static_cast<int>(arm_id));
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存后标定信息:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_CalibInfo(calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        return true;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败: 未知错误", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
}

/**
 * @brief 从JSON文件加载机械臂标定信息
 * @param bas_config_data_path 基础配置数据路径
 * @param cam_id 相机ID
 * @param arm_calib_info 机械臂标定信息（输出）
 * @return 是否加载成功
 *
 * 该函数从JSON文件中加载指定相机下的机械臂标定信息，包括标定结果、质量评估指标等。
 * 从arm_calib_info.arm_id获取机械臂ID，如果没有设置则尝试加载默认机械臂ID=0
 */
bool loadArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, ArmCalibInfo& arm_calib_info)
{
    arm_calib_info.arm_id = arm_id; // 设置机械臂ID
    try 
    {
        LOG_INFO("尝试加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载前相机cam_id=%d 下机械臂arm_id=%d 的标定信息:", static_cast<int>(cam_id), static_cast<int>(arm_id));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            handeyecalib::printLog_ArmCalibInfo(arm_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);// 调用打印日志函数
        }
        if (handeyecalib::loadCalibInfo(bas_config_data_path, cam_id, arm_id, arm_calib_info.calib_info))
        {
            LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
            LOG_INFO("✅ 相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息已加载完成！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            if (LOG_ON(log_level))
            {
                LOG_OUT(log_level, "📊 加载后相机cam_id=%d 下机械臂arm_id=%d 的标定信息:", static_cast<int>(cam_id), static_cast<int>(arm_id));
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                printLog_ArmCalibInfo(arm_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);// 调用打印日志函数
            }
            return true;
        }
        else
        {
            LOG_WARN("⚠️  加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败，使用默认值。", static_cast<int>(cam_id), static_cast<int>(arm_id));
            return false;
        }
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息失败: 未知错误", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
}

/**
 * @brief 保存机械臂标定信息到JSON文件
 * @param bas_config_data_path 基础配置数据路径
 * @param cam_id 相机ID
 * @param calib_res 标定结果
 * @return 是否保存成功
 *
 * 该函数将标定结果保存到JSON文件中，使用默认机械臂ID=0。
 * 在实际应用中，如果需要指定机械臂ID，可能需要修改函数签名
 */
bool saveArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfo& arm_calib_info)
{
    const uint8_t arm_id = arm_calib_info.arm_id;
    try 
    {
        LOG_INFO("尝试保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
        if (handeyecalib::saveCalibInfo(bas_config_data_path, cam_id, arm_id, arm_calib_info.calib_info))
        {
            LOG_INFO("✅ 已成功保存相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            return true;
        }
        else
        {
            LOG_ERROR("❌  保存相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息失败！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            return false;
        }
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息失败: %s", static_cast<int>(cam_id), static_cast<int>(arm_id), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的机械臂标定信息失败: 未知错误", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return false;
    }
}

bool loadArmCalibInfoList(const std::string& bas_config_data_path, const uint8_t cam_id, const std::vector<uint8_t>& arm_id_list, ArmCalibInfoList& arm_calib_list)
{
    arm_calib_list.clear();// 清空现有的机械臂标定信息列表
    if (arm_id_list.empty())
    {
        LOG_WARN("⚠️  机械臂ID列表为空，没有需要加载的标定信息");
        return false;
    }
    std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
    try 
    {
        LOG_INFO("尝试加载相机cam_id=%d 下指定的机械臂列表 %s 中的标定信息", static_cast<int>(cam_id), arm_ids_str.c_str());
        int success_count = 0;  // 记录加载成功的数量
        int total_count = 0;    // 记录总的数量
        for (const auto& arm_id : arm_id_list)// 遍历指定的机械臂ID列表，逐个加载
        {
            total_count++;
            ArmCalibInfo arm_calib_info;// 创建机械臂标定信息对象
            if (handeyecalib::loadArmCalibInfo(bas_config_data_path, cam_id, arm_id, arm_calib_info))// 加载机械臂标定信息
            {
                arm_calib_list[arm_id] = arm_calib_info;// 将加载成功的机械臂标定信息添加到列表中
                LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
                success_count++;
            }
            else
            {
                LOG_WARN("⚠️  加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败", static_cast<int>(cam_id), static_cast<int>(arm_id));
            }
        }
        LOG_INFO("✅ 相机cam_id=%d 下共需加载 %d 个机械臂列表 %s 中的标定信息，成功 %d 个，失败 %d 个", 
            static_cast<int>(cam_id), total_count, arm_ids_str.c_str(), success_count, total_count - success_count);
        if (!arm_calib_list.empty())
        {
            logsys::Level log_level = logsys::Level::INFO;
            logsys::Color color = logsys::Color::BLUE;
            if (LOG_ON(log_level))
            {
                LOG_OUT(log_level, "📊 加载后机械臂列表 %s 中的标定信息:", arm_ids_str.c_str());
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                printLog_ArmCalibInfoList(arm_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
            }
        }
        return success_count > 0;// 如果至少有一个加载成功，则返回true
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下指定机械臂列表 %s 中的标定信息失败: %s", static_cast<int>(cam_id), arm_ids_str.c_str(), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下指定机械臂列表 %s 中的标定信息失败: 未知错误", static_cast<int>(cam_id), arm_ids_str.c_str());
        return false;
    }
}

bool saveArmCalibInfoList(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfoList& arm_calib_list)
{
    if (arm_calib_list.empty())
    {
        LOG_WARN("⚠️  相机cam_id=%d 下没有机械臂标定信息需要保存", static_cast<int>(cam_id));
        return false;
    }
    std::vector<uint8_t> arm_id_list = getArmIds(arm_calib_list);
    std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
    try 
    {
        LOG_INFO("尝试保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息", static_cast<int>(cam_id), arm_ids_str.c_str());
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前机械臂列表 %s 中的标定信息:", arm_ids_str.c_str());
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_ArmCalibInfoList(arm_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        int success_count = 0;  // 记录保存成功的数量
        int total_count = 0;      // 记录总的数量
        // 遍历机械臂标定信息列表，逐个保存
        for (const auto& pair : arm_calib_list)
        {
            uint8_t arm_id = pair.first;
            const ArmCalibInfo& arm_calib_info = pair.second;
            total_count++;  
            // 保存单个机械臂标定信息
            if (handeyecalib::saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info))
            {
                LOG_INFO("✅ 成功保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
                success_count++;
            }
            else
            {
                LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            }
        }
        LOG_INFO("✅ 相机cam_id=%d 下共需保存 %d 个机械臂列表 %s 中的标定信息，成功 %d 个，失败 %d 个", 
            static_cast<int>(cam_id), total_count, arm_ids_str.c_str(), success_count, total_count - success_count);
        return success_count == total_count;// 如果所有保存都成功，则返回true
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息失败: %s", static_cast<int>(cam_id), arm_ids_str.c_str(), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息失败: 未知错误", static_cast<int>(cam_id), arm_ids_str.c_str());
        return false;
    }
}

// 加载指定相机目录下的所有机械臂标定数据及偏移补偿参数
bool loadCamCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, 
    const std::vector<uint8_t>& arm_id_list, const uint8_t sys_cam_num, CamCalibInfo& cam_calib_info)
{
    cam_calib_info.cam_id = cam_id; // 设置相机ID
    if (arm_id_list.empty())
    {
        LOG_WARN("⚠️  机械臂ID列表为空，没有需要加载的标定信息");
        return false;
    }
    std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
    try 
    {
        LOG_INFO("尝试加载相机cam_id=%d 下指定的机械臂列表 %s 中的标定信息", static_cast<int>(cam_id), arm_ids_str.c_str());
        int success_count = 0;  // 记录加载成功的数量
        int total_count = 0;    // 记录总的数量
        for (const auto& arm_id : arm_id_list) // 遍历指定的机械臂ID列表，逐个加载
        {
            total_count++;
            ArmCalibInfo arm_calib_info; // 创建机械臂标定信息对象
            if (handeyecalib::loadArmCalibInfo(bas_config_data_path, cam_id, arm_id, arm_calib_info)) // 加载机械臂标定信息
            {
                cam_calib_info.setArmCalibInfo(arm_id, arm_calib_info); // 将加载成功的机械臂标定信息添加到相机标定信息中
                LOG_INFO("成功加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
                success_count++;
            }
            else
            {
                LOG_WARN("⚠️  加载相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败", static_cast<int>(cam_id), static_cast<int>(arm_id));
            }
        }
        LOG_INFO("✅ 相机cam_id=%d 下共需加载 %d 个机械臂列表 %s 中的标定信息，成功 %d 个，失败 %d 个", 
            static_cast<int>(cam_id), total_count, arm_ids_str.c_str(), success_count, total_count - success_count);
        if (success_count > 0)
        {
            logsys::Level log_level = logsys::Level::INFO;
            logsys::Color color = logsys::Color::BLUE;
            if (LOG_ON(log_level))
            {
                LOG_OUT(log_level, "📊 加载后相机cam_id=%d 下机械臂列表 %s 中的标定信息:", static_cast<int>(cam_id), arm_ids_str.c_str());
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
                printLog_CamCalibInfo(cam_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
            }
        }
        return success_count > 0; // 如果至少有一个加载成功，则返回true
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下指定机械臂列表 %s 中的标定信息失败: %s", static_cast<int>(cam_id), arm_ids_str.c_str(), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 加载相机cam_id=%d 下指定机械臂列表 %s 中的标定信息失败: 未知错误", static_cast<int>(cam_id), arm_ids_str.c_str());
        return false;
    }
}

bool saveCamCalibInfo(const std::string& bas_config_data_path, const CamCalibInfo& cam_calib_info)
{
    const uint8_t cam_id = cam_calib_info.cam_id;
    if (cam_calib_info.arm_calib1D.empty())
    {
        LOG_WARN("⚠️  相机cam_id=%d 下没有机械臂标定信息需要保存", static_cast<int>(cam_id));
        return false;
    }
    std::vector<uint8_t> arm_id_list = handeyecalib::getArmIds(cam_calib_info.arm_calib1D);
    std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
    try 
    {
        LOG_INFO("尝试保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息", static_cast<int>(cam_id), arm_ids_str.c_str());
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 保存前相机cam_id=%d 下机械臂列表 %s 中的标定信息:", static_cast<int>(cam_id), arm_ids_str.c_str());
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CamCalibInfo(cam_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        int success_count = 0;  // 记录保存成功的数量
        int total_count = 0;      // 记录总的数量
        // 遍历机械臂标定信息列表，逐个保存
        for (const auto& pair : cam_calib_info.arm_calib1D)
        {
            uint8_t arm_id = pair.first;
            const ArmCalibInfo& arm_calib_info = pair.second;
            total_count++;  
            // 保存单个机械臂标定信息
            if (handeyecalib::saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info))
            {
                LOG_INFO("✅ 成功保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
                success_count++;
            }
            else
            {
                LOG_ERROR("❌ 保存相机cam_id=%d 下机械臂arm_id=%d 的标定信息失败！", static_cast<int>(cam_id), static_cast<int>(arm_id));
            }
        }
        LOG_INFO("✅ 相机cam_id=%d 下共需保存 %d 个机械臂列表 %s 中的标定信息，成功 %d 个，失败 %d 个", 
            static_cast<int>(cam_id), total_count, arm_ids_str.c_str(), success_count, total_count - success_count);
        return success_count == total_count; // 如果所有保存都成功，则返回true
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息失败: %s", static_cast<int>(cam_id), arm_ids_str.c_str(), e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存相机cam_id=%d 下所有机械臂列表 %s 中的标定信息失败: 未知错误", static_cast<int>(cam_id), arm_ids_str.c_str());
        return false;
    }
}

} // namespace handeyecalib