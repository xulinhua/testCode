#ifndef DATA_HANDLER_YAML_OPERATE_H
#define DATA_HANDLER_YAML_OPERATE_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include "data_handler/param_reflector.hpp"  // 包含ParamInfo和ParamType定义

namespace datahandler 
{

/**
 * @brief 从YAML节点中获取ID列表，支持数组和字符串格式
 * 
 * @param node YAML节点
 * @param node_name 节点名称
 * @return std::vector<uint8_t> ID列表
 */
std::vector<uint8_t> getNodeIDs(const YAML::Node& node, const std::string& node_name);

/**
 * @brief 模板化的参数加载函数，从YAML配置中加载参数值
 * 
 * @tparam ParamInfoType 参数信息类型
 * @tparam ParamTypeEnum 参数类型枚举类型
 * @param config YAML配置节点
 * @param param_info 参数信息对象
 */
template<typename ParamInfoType, typename ParamTypeEnum>
void loadParamInfoFromYaml(const YAML::Node& config, const ParamInfoType& param_info);

/**
 * @brief 加载YAML配置文件
 * 
 * @param config_file_path 配置文件路径
 * @param config YAML节点引用，用于返回加载的配置
 * @return bool 是否成功加载配置文件
 */
bool loadYamlFile(const std::string& config_file_path, YAML::Node& config);

/**
 * @brief 保存YAML配置文件
 * 
 * @param config_file_path 配置文件路径
 * @param config YAML节点，包含要保存的配置
 * @return bool 是否成功保存配置文件
 */
bool saveYamlFile(const std::string& config_file_path, const YAML::Node& config);

} // namespace datahandler

#endif // DATA_HANDLER_YAML_OPERATE_H