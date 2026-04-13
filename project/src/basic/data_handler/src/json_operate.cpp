#include "data_handler/json_operate.h"
#include <fstream>
#include <filesystem>
#include <any>
#include "log_system/log_macros.hpp"
#include "bas_operate/bas_utils.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace datahandler {

/**
 * @brief 通用JSON文件加载函数
 * @param file_path 加载文件路径
 * @param data JSON数据对象（输出）
 * @return 是否加载成功
 */
bool loadJsonFile(const std::string& file_path, nlohmann::json& data)
{
    try 
    {
        LOG_INFO("🔄 开始加载JSON文件: %s", file_path.c_str());
        if (!fs::exists(file_path)) // 检查文件是否存在
        {
            LOG_ERROR("❌ JSON文件不存在: %s", file_path.c_str());
            return false;
        } 
        std::ifstream test_file(file_path); 
        if (!test_file.good()) // 检查文件是否可读
        {
            LOG_ERROR("❌ JSON文件无法读取: %s", file_path.c_str());
            test_file.close();
            return false;
        }
        test_file.close();
        std::ifstream json_file(file_path);
        if (!json_file.is_open()) // 读取JSON文件
        {
            LOG_ERROR("❌ 无法打开JSON文件: %s", file_path.c_str());
            return false;
        }
        json_file >> data;
        json_file.close();
        LOG_INFO("✅ JSON文件已从 %s 加载", file_path.c_str());
        return true;   
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载JSON文件失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 加载JSON文件失败: 未知错误");
        return false;
    }
}
/**
 * @brief 通用JSON文件保存函数
 * @param file_path 保存文件路径
 * @param data JSON数据对象
 * @return 是否保存成功
 */
bool saveJsonFile(const std::string& file_path, const nlohmann::json& data)
{
    try 
    {
        fs::create_directories(fs::path(file_path).parent_path());// 创建目录（如果不存在）
        LOG_INFO("🔄 开始保存JSON文件: %s", file_path.c_str());
        std::ofstream json_file(file_path); 
        if (!json_file.is_open()) // 检查文件是否可打开
        {
            LOG_ERROR("❌ 无法创建JSON文件: %s", file_path.c_str());
            return false;
        }
        json_file << std::setw(4) << data << std::endl;
        json_file.close();     
        LOG_INFO("✅ JSON文件已保存到 %s", file_path.c_str());
        return true;   
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存JSON文件失败: %s", e.what());
        return false;
    } catch (...) 
    {
        LOG_ERROR("❌ 保存JSON文件失败: 未知错误");
        return false;
    }
}

/**
 * @brief 从JSON对象中加载4x4变换矩阵
 * @param data JSON数据对象
 * @param matrix_key 矩阵在JSON中的键名（如"CAM_TO_BASE_NAME"或"BASE_TO_CAM_NAME"）
 * @param matrix_name 矩阵名称，用于指定JSON中具体的矩阵字段
 * @param res_matrix 输出的cv::Mat矩阵引用
 * @return 是否加载成功
 */
bool loadMatrix4x4FromJson(const nlohmann::json& data, const std::string& matrix_key, const std::string& matrix_name, cv::Mat& res_matrix)
{
    try 
    {
        if (!data.contains(matrix_key)) 
        {
            LOG_WARN("⚠️ JSON中未找到矩阵键: %s", matrix_key.c_str());
            return false;
        }
        if (!data[matrix_key].contains(matrix_name)) 
        {
            LOG_WARN("⚠️ JSON中%s缺少matrix字段", matrix_key.c_str());
            return false;
        }
        auto matrix_data = data[matrix_key][matrix_name];
        // 验证矩阵维度
        if (matrix_data.size() != 4) 
        {
            LOG_ERROR("❌ 矩阵行数不正确，期望4行，实际%d行", matrix_data.size());
            return false;
        }
        for (size_t i = 0; i < matrix_data.size(); ++i) 
        {
            if (matrix_data[i].size() != 4) 
            {
                LOG_ERROR("❌ 矩阵列数不正确，第%d行期望4列，实际%d列", i, matrix_data[i].size());
                return false;
            }
        } 
        res_matrix = cv::Mat::eye(4, 4, CV_64F);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                res_matrix.at<double>(i, j) = matrix_data[i][j].get<double>();
            }
        }
        LOG_INFO("✅ 成功加载4x4变换矩阵: %s", matrix_key.c_str());
        return true;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 加载变换矩阵失败: %s", e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 加载变换矩阵失败: 未知错误");
        return false;
    }
}

/**
 * @brief 将4x4变换矩阵保存到JSON对象中
 * @param data JSON数据对象（输出）
 * @param matrix_key 矩阵在JSON中的键名（如"CAM_TO_BASE_NAME"或"BASE_TO_CAM_NAME"）
 * @param matrix_name 矩阵名称，用于指定JSON中具体的矩阵字段
 * @param src_matrix 输入的cv::Mat源矩阵
 * @return 是否保存成功
 */
bool saveMatrix4x4ToJson(nlohmann::json& data, const std::string& matrix_key, const std::string& matrix_name, const cv::Mat& src_matrix)
{
    try 
    {
        if (src_matrix.rows != 4 || src_matrix.cols != 4) // 验证矩阵维度
        {
            LOG_ERROR("❌ 源矩阵维度不正确，期望4x4，实际%dx%d", src_matrix.rows, src_matrix.cols);
            return false;
        }
        json matrix_json = json::array();// 创建JSON数组存储矩阵
        for (int i = 0; i < 4; i++)
        {
            json row = json::array();
            for (int j = 0; j < 4; j++)
            {
                row.push_back(src_matrix.at<double>(i, j));
            }
            matrix_json.push_back(row);
        }
        data[matrix_key][matrix_name] = matrix_json;// 将矩阵数据保存到JSON对象中
        LOG_INFO("✅ 成功保存4x4变换矩阵: %s", matrix_key.c_str());
        return true;
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存变换矩阵失败: %s", e.what());
        return false;
    } 
    catch (...) 
    {
        LOG_ERROR("❌ 保存变换矩阵失败: 未知错误");
        return false;
    }
}

/**
 * @brief 从JSON数据对象中加载参数到ParamInfo结构体
 * @param data JSON数据对象
 * @param param_info ParamInfo结构体引用（输出）
 * @return 是否加载成功
 */
bool loadParamFromJson(const nlohmann::json& data, datahandler::ParamInfo& param_info) 
{
    if (!data.contains(param_info.name)) // 检查参数是否存在于JSON数据中
    {
        LOG_WARN("参数 %s 不存在，使用默认值: %s", param_info.name.c_str(), convertParamValueToString(param_info).c_str());
        return false;
    }
    try 
    {
        switch (param_info.type) 
        {
            case datahandler::ParamType::BOOL:
                param_info.value = data[param_info.name].get<bool>();
                break;
            case datahandler::ParamType::INT8:
                param_info.value = static_cast<int8_t>(data[param_info.name].get<int>());
                break;
            case datahandler::ParamType::UINT8:
                param_info.value = static_cast<uint8_t>(data[param_info.name].get<int>());
                break;
            case datahandler::ParamType::INT16:
                param_info.value = static_cast<int16_t>(data[param_info.name].get<int>());
                break;
            case datahandler::ParamType::UINT16:
                param_info.value = static_cast<uint16_t>(data[param_info.name].get<int>());
                break;
            case datahandler::ParamType::INT32:
                param_info.value = data[param_info.name].get<int32_t>();
                break;
            case datahandler::ParamType::UINT32:
                param_info.value = static_cast<uint32_t>(data[param_info.name].get<int>());
                break;
            case datahandler::ParamType::INT64:
                param_info.value = data[param_info.name].get<int64_t>();
                break;
            case datahandler::ParamType::UINT64:
                param_info.value = data[param_info.name].get<uint64_t>();
                break;
            case datahandler::ParamType::FLOAT:
                param_info.value = static_cast<float>(data[param_info.name].get<double>());
                break;
            case datahandler::ParamType::DOUBLE:
                param_info.value = data[param_info.name].get<double>();
                break;
            case datahandler::ParamType::STRING:
                param_info.value = data[param_info.name].get<std::string>();
                break;
            case datahandler::ParamType::BOOL_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<bool>>();
                break;
            case datahandler::ParamType::INT8_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<int8_t>>();
                break;
            case datahandler::ParamType::UINT8_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<uint8_t>>();
                break;
            case datahandler::ParamType::INT16_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<int16_t>>();
                break;
            case datahandler::ParamType::UINT16_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<uint16_t>>();
                break;
            case datahandler::ParamType::INT32_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<int32_t>>();
                break;
            case datahandler::ParamType::UINT32_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<uint32_t>>();
                break;
            case datahandler::ParamType::INT64_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<int64_t>>();
                break;
            case datahandler::ParamType::UINT64_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<uint64_t>>();
                break;
            case datahandler::ParamType::FLOAT_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<float>>();
                break;
            case datahandler::ParamType::DOUBLE_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<double>>();
                break;
            case datahandler::ParamType::STRING_ARRAY:
                param_info.value = data[param_info.name].get<std::vector<std::string>>();
                break;
            default:
                LOG_ERROR("未知的参数类型 %d，参数名: %s", static_cast<int>(param_info.type), param_info.name.c_str());
                return false;
                break;
        }
        return param_info.updatePtrVal();
    } catch (const std::exception& e) {
        LOG_ERROR("加载参数 %s 时发生异常: %s，使用默认值", param_info.name.c_str(), e.what());
        return false;
    }
}

/**
 * @brief 将ParamInfo保存到JSON数据对象中
 * @param data JSON数据对象（输出）
 * @param param_info ParamInfo结构体常量引用
 * @return 是否保存成功
 */
bool saveParamToJson(nlohmann::json& data, const datahandler::ParamInfo& param_info) 
{
    try 
    {
        switch (param_info.type) 
        {
            case datahandler::ParamType::BOOL:
                data[param_info.name] = std::any_cast<bool>(param_info.value);
                break;
            case datahandler::ParamType::INT8:
                data[param_info.name] = static_cast<int>(std::any_cast<int8_t>(param_info.value));
                break;
            case datahandler::ParamType::UINT8:
                data[param_info.name] = static_cast<int>(std::any_cast<uint8_t>(param_info.value));
                break;
            case datahandler::ParamType::INT16:
                data[param_info.name] = static_cast<int>(std::any_cast<int16_t>(param_info.value));
                break;
            case datahandler::ParamType::UINT16:
                data[param_info.name] = static_cast<int>(std::any_cast<uint16_t>(param_info.value));
                break;
            case datahandler::ParamType::INT32:
                data[param_info.name] = std::any_cast<int32_t>(param_info.value);
                break;
            case datahandler::ParamType::UINT32:
                data[param_info.name] = static_cast<int>(std::any_cast<uint32_t>(param_info.value));
                break;
            case datahandler::ParamType::INT64:
                data[param_info.name] = std::any_cast<int64_t>(param_info.value);
                break;
            case datahandler::ParamType::UINT64:
                data[param_info.name] = std::any_cast<uint64_t>(param_info.value);
                break;
            case datahandler::ParamType::FLOAT:
                data[param_info.name] = static_cast<double>(std::any_cast<float>(param_info.value));
                break;
            case datahandler::ParamType::DOUBLE:
                data[param_info.name] = std::any_cast<double>(param_info.value);
                break;
            case datahandler::ParamType::STRING:
                data[param_info.name] = std::any_cast<std::string>(param_info.value);
                break;
            case datahandler::ParamType::BOOL_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<bool>>(param_info.value);
                break;
            case datahandler::ParamType::INT8_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<int8_t>>(param_info.value);
                break;
            case datahandler::ParamType::UINT8_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<uint8_t>>(param_info.value);
                break;
            case datahandler::ParamType::INT16_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<int16_t>>(param_info.value);
                break;
            case datahandler::ParamType::UINT16_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<uint16_t>>(param_info.value);
                break;
            case datahandler::ParamType::INT32_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<int32_t>>(param_info.value);
                break;
            case datahandler::ParamType::UINT32_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<uint32_t>>(param_info.value);
                break;
            case datahandler::ParamType::INT64_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<int64_t>>(param_info.value);
                break;
            case datahandler::ParamType::UINT64_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<uint64_t>>(param_info.value);
                break;
            case datahandler::ParamType::FLOAT_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<float>>(param_info.value);
                break;
            case datahandler::ParamType::DOUBLE_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<double>>(param_info.value);
                break;
            case datahandler::ParamType::STRING_ARRAY:
                data[param_info.name] = std::any_cast<std::vector<std::string>>(param_info.value);
                break;
            default:
                LOG_WARN("未知的参数类型 %d，参数名: %s", static_cast<int>(param_info.type), param_info.name.c_str());
                return false;
                break;
        }
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("保存参数 %s 时发生异常: %s", param_info.name.c_str(), e.what());
        return false;
    }
}

} // namespace datahandler