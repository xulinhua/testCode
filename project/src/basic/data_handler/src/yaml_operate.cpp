#include "../include/data_handler/yaml_operate.h"
#include "log_system/log_macros.hpp"        // 包含日志宏
#include "bas_operate/bas_utils.hpp"         // 包含基础工具函数
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <cctype>

namespace datahandler {

// 通用函数用于从YAML节点中获取ID列表，支持数组和字符串格式
std::vector<uint8_t> getNodeIDs(const YAML::Node& node, const std::string& node_name)
{
  std::vector<uint8_t> ids;
  const YAML::Node& ids_node = node[node_name];
  
  // 如果节点不存在
  if (!ids_node) 
  {
    return ids;
  }
  
  // 如果是数组格式
  if (ids_node.IsSequence()) 
  {
    for (size_t j = 0; j < ids_node.size(); j++) 
    {
      uint8_t id = ids_node[j].as<uint8_t>();
      ids.push_back(id);
    }
  }
  // 如果是字符串格式
  else if (ids_node.IsScalar())
  {
    // 解析字符串格式的ID列表，例如 "[0,1]"
    std::string ids_str = ids_node.as<std::string>();
    // 简单解析，移除方括号并按逗号分割
    if (!ids_str.empty() && ids_str.front() == '[' && ids_str.back() == ']')
    {
      ids_str = ids_str.substr(1, ids_str.length() - 2); // 移除方括号
      std::stringstream ss(ids_str);
      std::string id_str;
      while (std::getline(ss, id_str, ','))
      {
        // 去除空格
        id_str.erase(0, id_str.find_first_not_of(" \t"));
        id_str.erase(id_str.find_last_not_of(" \t") + 1);
        if (!id_str.empty())
        {
          try {
            uint8_t id = static_cast<uint8_t>(std::stoi(id_str));
            ids.push_back(id);
          } catch (const std::exception& e) {
            // 忽略无效的ID
          }
        }
      }
    }
    // 如果是简单的数字字符串（如"0"）
    else if (!ids_str.empty())
    {
      try {
        uint8_t id = static_cast<uint8_t>(std::stoi(ids_str));
        ids.push_back(id);
      } catch (const std::exception& e) {
        // 忽略无效的ID
      }
    }
  }
  return ids;
}

// 模板化的参数加载函数，参照convertParamInfoToParameter的设计模式
template<typename ParamInfoType, typename ParamTypeEnum>
void loadParamInfoFromYaml(const YAML::Node& config, const ParamInfoType& param_info)
{
    try 
    { 
        if (!config[param_info.name]) // 检查配置的参数是否存在
        {
            LOG_WARN("参数 %s 不存在，使用默认值", param_info.name.c_str());
            return;
        }
        if (config[param_info.name].IsNull()) // 检查配置的参数是否为null
        {
            LOG_WARN("参数 %s 为null，使用默认值", param_info.name.c_str());
            return;
        }
        switch (param_info.type) 
        {
            case ParamTypeEnum::BOOL: 
            {
                bool* ptr = static_cast<bool*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望bool类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<bool>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %s", param_info.name.c_str(), *ptr ? "true" : "false");
                break;
            }
            case ParamTypeEnum::INT8: 
            {
                int8_t* ptr = static_cast<int8_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望int8类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<int8_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %d", param_info.name.c_str(), static_cast<int>(*ptr));
                break;
            }
            case ParamTypeEnum::UINT8: 
            {
                uint8_t* ptr = static_cast<uint8_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望uint8类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<uint8_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %d", param_info.name.c_str(), static_cast<int>(*ptr));
                break;
            }
            case ParamTypeEnum::INT16: 
            {
                int16_t* ptr = static_cast<int16_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望int16类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<int16_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %d", param_info.name.c_str(), static_cast<int>(*ptr));
                break;
            }
            case ParamTypeEnum::UINT16: 
            {
                uint16_t* ptr = static_cast<uint16_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望uint16类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<uint16_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %d", param_info.name.c_str(), static_cast<int>(*ptr));
                break;
            }
            case ParamTypeEnum::INT32: 
            {
                int32_t* ptr = static_cast<int32_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望int32类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<int32_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %d", param_info.name.c_str(), static_cast<int>(*ptr));
                break;
            }
            case ParamTypeEnum::UINT32: 
            {
                uint32_t* ptr = static_cast<uint32_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望uint32类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<uint32_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %u", param_info.name.c_str(), static_cast<unsigned int>(*ptr));
                break;
            }
            case ParamTypeEnum::INT64: 
            {
                int64_t* ptr = static_cast<int64_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望int64类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<int64_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %ld", param_info.name.c_str(), static_cast<long>(*ptr));
                break;
            }
            case ParamTypeEnum::UINT64: 
            {
                uint64_t* ptr = static_cast<uint64_t*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望uint64类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<uint64_t>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %lu", param_info.name.c_str(), static_cast<unsigned long>(*ptr));
                break;
            }
            case ParamTypeEnum::FLOAT: 
            {
                float* ptr = static_cast<float*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望float类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<float>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %f", param_info.name.c_str(), *ptr);
                break;
            }
            case ParamTypeEnum::DOUBLE: 
            {
                double* ptr = static_cast<double*>(param_info.ptr);
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望double类型，使用默认值", param_info.name.c_str());
                    return;
                }
                *ptr = config[param_info.name].template as<double>();
                // 更新ParamInfo中的value字段
                const_cast<ParamInfoType&>(param_info).setValue(*ptr);
                LOG_INFO("参数 %s = %f", param_info.name.c_str(), *ptr);
                break;
            }
            case ParamTypeEnum::STRING: 
            {
                if (config[param_info.name].Type() != YAML::NodeType::Scalar) // 检查YAML节点类型是否匹配
                {
                    LOG_WARN("参数 %s 类型不匹配，期望string类型，使用默认值", param_info.name.c_str());
                    return;
                }
                std::string value_str = config[param_info.name].template as<std::string>();
                if (param_info.ptr != nullptr) 
                {
                    std::string* ptr = static_cast<std::string*>(param_info.ptr);
                    *ptr = value_str;
                    const_cast<ParamInfoType&>(param_info).setValue(*ptr);// 更新ParamInfo中的value字段
                } else {
                    const_cast<ParamInfoType&>(param_info).value = value_str;// 直接更新ParamInfo中的value字段，保持与实际值一致
                }
                LOG_INFO("参数 %s = %s", param_info.name.c_str(), value_str.c_str());
                break;
            }
            case ParamTypeEnum::BOOL_ARRAY: 
            {
                // 检查YAML节点类型是否为序列类型
                if (config[param_info.name].Type() != YAML::NodeType::Sequence)
                {
                    // 如果不是序列类型，尝试作为字符串解析
                    if (config[param_info.name].Type() == YAML::NodeType::Scalar)
                    {
                        LOG_WARN("参数 %s 类型为标量，将尝试解析为数组", param_info.name.c_str());
                    }
                    else
                    {
                        LOG_WARN("参数 %s 类型不匹配，期望数组类型，使用默认值", param_info.name.c_str());
                        return;
                    }
                }
                std::vector<bool> value_vec;// 解析数组值
                if (config[param_info.name].IsSequence())
                {
                    // 标准数组格式 [true, false, true]
                    for (size_t i = 0; i < config[param_info.name].size(); ++i)
                    {
                        value_vec.push_back(config[param_info.name][i].template as<bool>());
                    }
                }
                else
                {
                    // 字符串格式 "[true,false]" - 但bool数组通常不会以字符串形式存储
                    LOG_WARN("数组类型参数 %s 不支持字符串格式解析", param_info.name.c_str());
                }
                if (param_info.ptr != nullptr) // 对于ptr为nullptr的参数（如虚拟参数），不进行赋值操作
                {
                    LOG_WARN("数组类型参数 %s 不支持直接内存赋值", param_info.name.c_str());
                }
                const_cast<ParamInfoType&>(param_info).value = value_vec;// 更新ParamInfo中的value字段
                std::string array_str = basmodule::get_list_string(value_vec);
                LOG_INFO("参数 %s = %s", param_info.name.c_str(), array_str.c_str());
                break;
            }
            case ParamTypeEnum::UINT8_ARRAY: 
            {
                // 检查YAML节点类型是否为序列类型
                if (config[param_info.name].Type() != YAML::NodeType::Sequence)
                {
                    // 如果不是序列类型，尝试作为字符串解析
                    if (config[param_info.name].Type() == YAML::NodeType::Scalar)
                    {
                        LOG_WARN("参数 %s 类型为标量，将尝试解析为数组", param_info.name.c_str());
                    }
                    else
                    {
                        LOG_WARN("参数 %s 类型不匹配，期望数组类型，使用默认值", param_info.name.c_str());
                        return;
                    }
                }
                std::vector<uint8_t> value_vec;// 解析数组值
                if (config[param_info.name].IsSequence())
                {
                    // 标准数组格式 [0, 1, 2]
                    for (size_t i = 0; i < config[param_info.name].size(); ++i)
                    {
                        value_vec.push_back(config[param_info.name][i].template as<uint8_t>());
                    }
                }
                else
                {
                    // 字符串格式 "[0,1]"
                    std::string value_str = config[param_info.name].template as<std::string>();
                    value_vec = getNodeIDs(config, param_info.name);
                }
                if (param_info.ptr != nullptr) // 对于ptr为nullptr的参数（如虚拟参数），不进行赋值操作
                {
                    // 注意：对于数组类型，我们不会直接赋值到ptr指向的内存，
                    // 因为这需要预分配内存，复杂度较高。
                    // 实际应用中，数组类型参数通常通过专门的处理函数处理
                    LOG_WARN("数组类型参数 %s 不支持直接内存赋值", param_info.name.c_str());
                }
                const_cast<ParamInfoType&>(param_info).value = value_vec;// 更新ParamInfo中的value字段
                std::string array_str = basmodule::get_list_string(value_vec);
                LOG_INFO("参数 %s = %s", param_info.name.c_str(), array_str.c_str());
                break;
            }
            default: {
                LOG_WARN("未处理的参数类型: %s (%s)", param_info.name.c_str(), param_info.getTypeString().c_str());
                break;
            }
        }
    } catch (const YAML::BadConversion& e) {
        LOG_WARN("参数 %s 类型转换错误: %s，使用默认值", param_info.name.c_str(), e.what());
    } catch (const std::exception& e) {
        LOG_WARN("参数 %s 读取时发生异常: %s，使用默认值", param_info.name.c_str(), e.what());
    }
}

// 为模板函数提供显式实例化以避免链接错误
template void datahandler::loadParamInfoFromYaml<datahandler::ParamInfo, datahandler::ParamType>(const YAML::Node& config, const datahandler::ParamInfo& param_info);

bool loadYamlFile(const std::string& config_file_path, YAML::Node& config)
{
  try 
  {
    if (!std::filesystem::exists(config_file_path)) // 检查文件是否存在
    {
      LOG_ERROR("配置文件不存在: %s", config_file_path.c_str());
      return false;
    }
    std::ifstream test_file(config_file_path);
    if (!test_file.good()) // 检查文件是否可读
    {
        LOG_ERROR("配置文件无法读取: %s", config_file_path.c_str());
        test_file.close(); 
        return false;
    }
    test_file.close();
    config = YAML::LoadFile(config_file_path);
    return true;
  } catch (const YAML::Exception & e) {
    LOG_ERROR("加载配置文件出错: %s, 异常: %s", config_file_path.c_str(), e.what());
    return false;
  }
}

bool saveYamlFile(const std::string& config_file_path, const YAML::Node& config)
{
  try 
  {
    std::filesystem::path filepath(config_file_path);
    std::filesystem::create_directories(filepath.parent_path());// 创建目录（如果不存在）
    std::ofstream fout(config_file_path);// 写入YAML文件
    if (!fout.is_open()) // 检查文件是否可以打开写入
    {
        LOG_ERROR("配置文件无法写入: %s", config_file_path.c_str());
        return false;
    }
    fout << config;
    fout.close();
    LOG_INFO("成功保存配置文件: %s", config_file_path.c_str());
    return true;
  } catch (const YAML::Exception & e) {
    LOG_ERROR("保存配置文件出错: %s, 异常: %s", config_file_path.c_str(), e.what());
    return false;
  }
}

} // namespace datahandler