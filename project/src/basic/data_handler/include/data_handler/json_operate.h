#ifndef DATA_HANDLER_JSON_OPERATE_H
#define DATA_HANDLER_JSON_OPERATE_H

#include <string>
#include <nlohmann/json.hpp>
#include "param_reflector.hpp"
#include <opencv2/opencv.hpp>

namespace datahandler {

/**
 * @brief 通用JSON文件加载函数
 * @param data JSON数据对象
 * @param data JSON数据对象（输出）
 * @return 是否加载成功
 */
bool loadJsonFile(const std::string& file_path, nlohmann::json& data);

/**
* @brief 通用JSON文件保存函数
* @param file_path 保存文件路径
* @param data JSON数据对象
* @return 是否保存成功
*/
bool saveJsonFile(const std::string& file_path, const nlohmann::json& data);

/**
 * @brief 从JSON对象中加载4x4变换矩阵
 * @param data JSON数据对象
 * @param matrix_key 矩阵在JSON中的键名（如"CAM_TO_BASE_NAME"或"BASE_TO_CAM_NAME"）
 * @param matrix_name 矩阵名称，用于指定JSON中具体的矩阵字段
 * @param res_matrix 输出的cv::Mat矩阵引用
 * @return 是否加载成功
 */
bool loadMatrix4x4FromJson(const nlohmann::json& data, const std::string& matrix_key, const std::string& matrix_name, cv::Mat& res_matrix);

/**
 * @brief 将4x4变换矩阵保存到JSON对象中
 * @param data JSON数据对象（输出）
 * @param matrix_key 矩阵在JSON中的键名（如"CAM_TO_BASE_NAME"或"BASE_TO_CAM_NAME"）
 * @param matrix_name 矩阵名称，用于指定JSON中具体的矩阵字段
 * @param src_matrix 输入的cv::Mat源矩阵
 * @return 是否保存成功
 */
bool saveMatrix4x4ToJson(nlohmann::json& data, const std::string& matrix_key, const std::string& matrix_name, const cv::Mat& src_matrix);

/**
 * @brief 从JSON数据对象中加载参数到ParamInfo结构体
 * @param data JSON数据对象
 * @param param_info ParamInfo结构体引用（输出）
 * @return 是否加载成功
 */
bool loadParamFromJson(const nlohmann::json& data, datahandler::ParamInfo& param_info);

/**
 * @brief 将ParamInfo保存到JSON数据对象中
 * @param data JSON数据对象（输出）
 * @param param_info ParamInfo结构体常量引用
 * @return 是否保存成功
 */
bool saveParamToJson(nlohmann::json& data, const datahandler::ParamInfo& param_info);

} // namespace datahandler

#endif // DATA_HANDLER_JSON_OPERATE_H