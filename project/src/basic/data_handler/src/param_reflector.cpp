#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"
#include "log_system/log_macros.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace datahandler {

std::string getTypeString(ParamType type)
{
    switch(type) 
    {
        case ParamType::BOOL: return "bool";
        case ParamType::INT8: return "int8_t";
        case ParamType::UINT8: return "uint8_t";
        case ParamType::INT16: return "int16_t";
        case ParamType::UINT16: return "uint16_t";
        case ParamType::INT32: return "int32_t";
        case ParamType::UINT32: return "uint32_t";
        case ParamType::INT64: return "int64_t";
        case ParamType::UINT64: return "uint64_t";
        case ParamType::FLOAT: return "float";
        case ParamType::DOUBLE: return "double";
        case ParamType::STRING: return "std::string";
        case ParamType::BOOL_ARRAY: return "std::vector<bool>";
        case ParamType::INT8_ARRAY: return "std::vector<int8_t>";
        case ParamType::UINT8_ARRAY: return "std::vector<uint8_t>";
        case ParamType::INT16_ARRAY: return "std::vector<int16_t>";
        case ParamType::UINT16_ARRAY: return "std::vector<uint16_t>";
        case ParamType::INT32_ARRAY: return "std::vector<int32_t>";
        case ParamType::UINT32_ARRAY: return "std::vector<uint32_t>";
        case ParamType::INT64_ARRAY: return "std::vector<int64_t>";
        case ParamType::UINT64_ARRAY: return "std::vector<uint64_t>";
        case ParamType::FLOAT_ARRAY: return "std::vector<float>";
        case ParamType::DOUBLE_ARRAY: return "std::vector<double>";
        case ParamType::STRING_ARRAY: return "std::vector<std::string>";
        default: return "unknown";
    }
}

// 将参数值转换为字符串表示
std::string convertParamValueToString(const ParamInfo& param)
{
    std::string param_value_str;
    try 
    {
        switch (param.type) 
        {
            case ParamType::BOOL: 
            {
                bool value = std::any_cast<bool>(param.value);
                param_value_str = value ? "true" : "false";
                break;
            }
            case ParamType::INT8: 
            {
                int8_t value = std::any_cast<int8_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::UINT8: 
            {
                uint8_t value = std::any_cast<uint8_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::INT16: 
            {
                int16_t value = std::any_cast<int16_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::UINT16: 
            {
                uint16_t value = std::any_cast<uint16_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::INT32: 
            {
                int32_t value = std::any_cast<int32_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::UINT32:
            {
                uint32_t value = std::any_cast<uint32_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::INT64: 
            {
                int64_t value = std::any_cast<int64_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::UINT64: 
            {
                uint64_t value = std::any_cast<uint64_t>(param.value);
                param_value_str = std::to_string(value);
                break;
            }
            case ParamType::FLOAT: 
            {
                float value = std::any_cast<float>(param.value);
                std::ostringstream float_stream;
                float_stream << std::fixed << std::setprecision(6) << value;
                param_value_str = float_stream.str();
                break;
            }
            case ParamType::DOUBLE: 
            {
                double value = std::any_cast<double>(param.value);
                std::ostringstream double_stream;
                double_stream << std::fixed << std::setprecision(6) << value;
                param_value_str = double_stream.str();
                break;
            }
            case ParamType::STRING: 
            {
                param_value_str = std::any_cast<std::string>(param.value);
                break;
            }
            case ParamType::BOOL_ARRAY: 
            {
                auto values = std::any_cast<std::vector<bool>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::INT8_ARRAY: 
            {
                auto values = std::any_cast<std::vector<int8_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::UINT8_ARRAY: 
            {
                auto values = std::any_cast<std::vector<uint8_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::INT16_ARRAY: 
            {
                auto values = std::any_cast<std::vector<int16_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::UINT16_ARRAY: 
            {
                auto values = std::any_cast<std::vector<uint16_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::INT32_ARRAY: 
            {
                auto values = std::any_cast<std::vector<int32_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::UINT32_ARRAY: 
            {
                auto values = std::any_cast<std::vector<uint32_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::INT64_ARRAY: 
            {
                auto values = std::any_cast<std::vector<int64_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::UINT64_ARRAY: 
            {
                auto values = std::any_cast<std::vector<uint64_t>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            case ParamType::FLOAT_ARRAY: 
            {
                auto values = std::any_cast<std::vector<float>>(param.value);
                param_value_str = basmodule::format_float_array(values, 6);
                break;
            }
            case ParamType::DOUBLE_ARRAY: 
            {
                auto values = std::any_cast<std::vector<double>>(param.value);
                param_value_str = basmodule::format_float_array(values, 6);
                break;
            }
            case ParamType::STRING_ARRAY: 
            {
                auto values = std::any_cast<std::vector<std::string>>(param.value);
                param_value_str = basmodule::get_list_string(values);
                break;
            }
            default:
                param_value_str = "unknown";
                break;
        }
    } catch (const std::bad_any_cast& e) {
        param_value_str = "[无法获取值]";
    }
    return param_value_str;
}

// 将value值刷新赋值给ptr指针内存
bool ParamInfo::updatePtrVal() 
{
    if (ptr == nullptr) {
        return true;  // 如果ptr为nullptr，则不需要赋值，返回true
    }
    try 
    {
        switch (type) 
        {
            case ParamType::BOOL:
                if (value.type() == typeid(bool)) {
                    *static_cast<bool*>(ptr) = std::any_cast<bool>(value);
                }
                break;
            case ParamType::INT8:
                if (value.type() == typeid(int8_t)) {
                    *static_cast<int8_t*>(ptr) = std::any_cast<int8_t>(value);
                }
                break;
            case ParamType::UINT8:
                if (value.type() == typeid(uint8_t)) {
                    *static_cast<uint8_t*>(ptr) = std::any_cast<uint8_t>(value);
                }
                break;
            case ParamType::INT16:
                if (value.type() == typeid(int16_t)) {
                    *static_cast<int16_t*>(ptr) = std::any_cast<int16_t>(value);
                }
                break;
            case ParamType::UINT16:
                if (value.type() == typeid(uint16_t)) {
                    *static_cast<uint16_t*>(ptr) = std::any_cast<uint16_t>(value);
                }
                break;
            case ParamType::INT32:
                if (value.type() == typeid(int32_t)) {
                    *static_cast<int32_t*>(ptr) = std::any_cast<int32_t>(value);
                }
                break;
            case ParamType::UINT32:
                if (value.type() == typeid(uint32_t)) {
                    *static_cast<uint32_t*>(ptr) = std::any_cast<uint32_t>(value);
                }
                break;
            case ParamType::INT64:
                if (value.type() == typeid(int64_t)) {
                    *static_cast<int64_t*>(ptr) = std::any_cast<int64_t>(value);
                }
                break;
            case ParamType::UINT64:
                if (value.type() == typeid(uint64_t)) {
                    *static_cast<uint64_t*>(ptr) = std::any_cast<uint64_t>(value);
                }
                break;
            case ParamType::FLOAT:
                if (value.type() == typeid(float)) {
                    *static_cast<float*>(ptr) = std::any_cast<float>(value);
                }
                break;
            case ParamType::DOUBLE:
                if (value.type() == typeid(double)) {
                    *static_cast<double*>(ptr) = std::any_cast<double>(value);
                }
                break;
            case ParamType::STRING:
                if (value.type() == typeid(std::string)) {
                    *static_cast<std::string*>(ptr) = std::any_cast<std::string>(value);
                }
                break;
            case ParamType::BOOL_ARRAY:
                if (value.type() == typeid(std::vector<bool>)) {
                    *static_cast<std::vector<bool>*>(ptr) = std::any_cast<std::vector<bool>>(value);
                }
                break;
            case ParamType::INT8_ARRAY:
                if (value.type() == typeid(std::vector<int8_t>)) {
                    *static_cast<std::vector<int8_t>*>(ptr) = std::any_cast<std::vector<int8_t>>(value);
                }
                break;
            case ParamType::UINT8_ARRAY:
                if (value.type() == typeid(std::vector<uint8_t>)) {
                    *static_cast<std::vector<uint8_t>*>(ptr) = std::any_cast<std::vector<uint8_t>>(value);
                }
                break;
            case ParamType::INT16_ARRAY:
                if (value.type() == typeid(std::vector<int16_t>)) {
                    *static_cast<std::vector<int16_t>*>(ptr) = std::any_cast<std::vector<int16_t>>(value);
                }
                break;
            case ParamType::UINT16_ARRAY:
                if (value.type() == typeid(std::vector<uint16_t>)) {
                    *static_cast<std::vector<uint16_t>*>(ptr) = std::any_cast<std::vector<uint16_t>>(value);
                }
                break;
            case ParamType::INT32_ARRAY:
                if (value.type() == typeid(std::vector<int32_t>)) {
                    *static_cast<std::vector<int32_t>*>(ptr) = std::any_cast<std::vector<int32_t>>(value);
                }
                break;
            case ParamType::UINT32_ARRAY:
                if (value.type() == typeid(std::vector<uint32_t>)) {
                    *static_cast<std::vector<uint32_t>*>(ptr) = std::any_cast<std::vector<uint32_t>>(value);
                }
                break;
            case ParamType::INT64_ARRAY:
                if (value.type() == typeid(std::vector<int64_t>)) {
                    *static_cast<std::vector<int64_t>*>(ptr) = std::any_cast<std::vector<int64_t>>(value);
                }
                break;
            case ParamType::UINT64_ARRAY:
                if (value.type() == typeid(std::vector<uint64_t>)) {
                    *static_cast<std::vector<uint64_t>*>(ptr) = std::any_cast<std::vector<uint64_t>>(value);
                }
                break;
            case ParamType::FLOAT_ARRAY:
                if (value.type() == typeid(std::vector<float>)) {
                    *static_cast<std::vector<float>*>(ptr) = std::any_cast<std::vector<float>>(value);
                }
                break;
            case ParamType::DOUBLE_ARRAY:
                if (value.type() == typeid(std::vector<double>)) {
                    *static_cast<std::vector<double>*>(ptr) = std::any_cast<std::vector<double>>(value);
                }
                break;
            case ParamType::STRING_ARRAY:
                if (value.type() == typeid(std::vector<std::string>)) {
                    *static_cast<std::vector<std::string>*>(ptr) = std::any_cast<std::vector<std::string>>(value);
                }
                break;
            default:
                return false;  // 不支持的类型
        }
        return true;  // 成功赋值
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("类型转换失败: %s", e.what());
        return false;// 类型转换失败，返回false
    }
}

// 将value值刷新赋值给ptr指针内存
bool updatePtrVal(ParamInfo& param) 
{
    return param.updatePtrVal();
}

// 获取参数索引字符串的宽度，以"9999"的长度为统一长度
size_t getDefaultParamIndexWidth() 
{
    std::string referenceStr = "[9999]";
    size_t referenceWidth = referenceStr.length();
    size_t defaultIndexWidth = std::string("参数索引").length();
    return std::max(defaultIndexWidth, referenceWidth);
}

// 打印日志输出函数
void printLog(const ParamInfo& param, const std::string& project_path, int log_level, bool bShowExMsg, 
    int color, const char* file_name_path, const char* func, int line,
    uint16_t maxNameWidth, uint16_t maxTypeWidth, uint16_t maxValueWidth, int paraIdx)
{
    bool bAddIndent = true; // 是否添加缩进
    // 如果传入的宽度为默认值，则使用默认宽度
    uint16_t actualMaxNameWidth = maxNameWidth;
    uint16_t actualMaxTypeWidth =  maxTypeWidth;
    uint16_t actualMaxValueWidth = maxValueWidth;
    
    // 如果传入的宽度为0，则自动计算合适的宽度
    if (maxNameWidth == 0 || maxTypeWidth == 0 || maxValueWidth == 0) 
    {
        // 通过临时向量计算实际所需的最大宽度
        std::vector<ParamInfo> tempParams = {param};
        actualMaxNameWidth = ConfigReflector::getMaxParamNameWidth(tempParams);
        actualMaxTypeWidth = ConfigReflector::getMaxParamTypeWidth(tempParams);
        actualMaxValueWidth = ConfigReflector::getMaxParamValueWidth(tempParams);
        
        // 如果计算结果为0，则使用默认宽度
        if (actualMaxNameWidth == 0) actualMaxNameWidth = 25;
        if (actualMaxTypeWidth == 0) actualMaxTypeWidth = 15;
        if (actualMaxValueWidth == 0) actualMaxValueWidth = 30;
    } 
    
    std::string param_value_str = convertParamValueToString(param);
    // 构建格式化输出字符串，确保对齐
    std::string namePart = param.name;
    std::string typePart = param.getTypeString();
    std::string valuePart = param_value_str;
    // 使用指定的宽度进行对齐输出
    std::stringstream ss;
    
    // 添加缩进
    if (bAddIndent) {
        ss << basmodule::generateIndent();
    }
    
    // 索引部分
    std::string indexPart = "";
    size_t indexDisplayWidth = 0;
    if (paraIdx > 0) 
    {
        indexPart = "[" + std::to_string(paraIdx) + "]";
        indexDisplayWidth = basmodule::getDisplayWidth(indexPart);
    }
    
    // 计算索引部分的固定宽度
    size_t indexTargetWidth = getDefaultParamIndexWidth();
    
    // 构建输出字符串，不使用std::setw，手动控制对齐
    if (paraIdx > 0) 
    {
        // 索引部分
        ss << indexPart;
        // 计算需要填充的空格数
        if (indexDisplayWidth < indexTargetWidth) {
            ss << std::string(indexTargetWidth - indexDisplayWidth, ' ');
        }
        ss << " "; // 减少为一个空格分隔
    }
    else 
    {
        ss << std::string(indexTargetWidth + 1, ' '); // 减少为一个空格分隔
    }
    
    // 使用padToWidth函数确保每个部分达到指定的显示宽度
    // 将列之间的两个空格减少为一个空格
    ss << basmodule::padToWidth(namePart, actualMaxNameWidth) << " "
       << basmodule::padToWidth(typePart, actualMaxTypeWidth) << " "
       << valuePart; // 值部分不需要填充，直接输出
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
    bShowExMsg, static_cast<logsys::Color>(color), "%s", ss.str().c_str());
}

// 打印日志输出函数
void printLog(const std::vector<ParamInfo>& params, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    bool bShowExMsg = false;
    if (params.empty()) 
    {
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        bShowExMsg, static_cast<logsys::Color>(color), "无参数可打印");
        return;
    }
    // 计算参数列表的最大宽度
    uint16_t maxNameWidth = ConfigReflector::getMaxParamNameWidth(params);
    uint16_t maxTypeWidth = ConfigReflector::getMaxParamTypeWidth(params);
    uint16_t maxValueWidth = ConfigReflector::getMaxParamValueWidth(params);
    
    // 确保最小宽度不小于表头文字的实际显示宽度
    // 注意：中文字符显示宽度为2
    size_t headerNameWidth = basmodule::getDisplayWidth("参数名"); // 应为4 * 2=8
    size_t headerTypeWidth = basmodule::getDisplayWidth("类型");   // 应为2 * 2=4
    size_t headerValueWidth = basmodule::getDisplayWidth("值");    // 应为1 * 2=2
    
    // 使用实际显示宽度比较
    maxNameWidth = std::max(maxNameWidth, static_cast<uint16_t>(headerNameWidth));
    maxTypeWidth = std::max(maxTypeWidth, static_cast<uint16_t>(headerTypeWidth));
    maxValueWidth = std::max(maxValueWidth, static_cast<uint16_t>(headerValueWidth));
    
    // 输出表头信息，与后续参数信息对齐
    std::stringstream header_ss;
    std::string indexHeader = "参数索引";
    size_t indexTargetWidth = getDefaultParamIndexWidth();
    
    // 添加缩进（表头也需要向右偏移）
    header_ss << basmodule::generateIndent();
    
    // 手动构建表头，不使用std::setw
    // 将列之间的两个空格减少为一个空格
    header_ss << basmodule::padToWidth(indexHeader, indexTargetWidth) << " "
              << basmodule::padToWidth("参数名", maxNameWidth) << " "
              << basmodule::padToWidth("类型", maxTypeWidth) << " "
              << "值";  // 值部分不需要填充
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        bShowExMsg, static_cast<logsys::Color>(color), "%s", header_ss.str().c_str());
    int paraIdx = 1; // 参数索引，从1开始
    for (const auto& param : params) // 逐个打印每个参数
    {
        printLog(param, project_path, log_level, bShowExMsg, color, file_name_path, func, line, 
            maxNameWidth, maxTypeWidth, maxValueWidth, paraIdx);
        paraIdx++;
    }
}

//在加载配置文件后更新参数
void ConfigReflector::updateParamAftLoad() 
{
    LOG_DEBUG("更新参数列表实际值");
    // 默认实现：将 saved_params_ 中的值更新到 params_ 中
    const auto& saved_params = getParamsSaved();
    for (const auto& saved_param : saved_params) 
    {
        for (auto& param : params_) 
        {
            if (param.name == saved_param.name) 
            {
                // 将参数值转换为字符串表示
                std::string param_value_str = convertParamValueToString(param);
                LOG_DEBUG("更新参数%s 的值为 %s", param.name.c_str(), param_value_str.c_str());
                param.value = saved_param.value;
                // 同时更新指针指向的内存值
                param.updatePtrVal();
                break;
            }
        }
    }
}

// ConfigReflector 实现
const ParamInfo* ConfigReflector::getParamByName(const std::string& name) const 
{
    auto it = std::find_if(params_.begin(), params_.end(), 
        [&name](const ParamInfo& param) { return param.name == name; });
    
    if (it != params_.end()) {
        return &(*it);
    }
    return nullptr;
}

// 静态函数：获取参数列表名的最大宽度
size_t ConfigReflector::getMaxParamNameWidth(const std::vector<ParamInfo>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxNameWidth = std::string("参数名: ").length();// 将字符串"参数名: "的长度作为maxNameWidth的起始值
    uint8_t offsetAddWidth = 4;// 偏移加宽度
    for (const auto& param : params) 
    {
        maxNameWidth = std::max(maxNameWidth, param.name.length() + offsetAddWidth); // 直接使用参数名称的长度，而不是解析格式化后的消息
    }
    return maxNameWidth;
}

// 静态函数：获取参数列表类型的最大宽度
size_t ConfigReflector::getMaxParamTypeWidth(const std::vector<ParamInfo>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxTypeWidth = std::string("类型: ").length();// 将字符串"类型: "的长度作为maxTypeWidth的起始值
    uint8_t offsetAddWidth = 4;// 偏移加宽度
    for (const auto& param : params) 
    {
        maxTypeWidth = std::max(maxTypeWidth, param.getTypeString().length() + offsetAddWidth); // 使用参数类型的字符串长度
    }
    return maxTypeWidth;
}

// 静态函数：获取参数列表值的最大宽度
size_t ConfigReflector::getMaxParamValueWidth(const std::vector<ParamInfo>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxValueWidth = std::string("值: ").length();// 将字符串"值: "的长度作为maxValueWidth的起始值
    uint8_t offsetAddWidth = 4;// 偏移加宽度
    for (const auto& param : params) 
    {
        std::string param_value_str = convertParamValueToString(param);// 获取参数值的字符串表示
        maxValueWidth = std::max(maxValueWidth, param_value_str.length() + offsetAddWidth); // 使用参数值的字符串长度
    }
    return maxValueWidth;
}

// 打印参数列表的日志信息
void  ConfigReflector::printLog(const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line) const
{
    datahandler::printLog(params_, project_path, log_level, color, file_name_path, func, line);// 打印日志输出函数
}

} // namespace datahandler