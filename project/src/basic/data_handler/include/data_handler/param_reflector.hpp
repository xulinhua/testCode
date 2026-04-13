#ifndef DATA_HANDLER_PARAM_REFLECTOR_HPP
#define DATA_HANDLER_PARAM_REFLECTOR_HPP

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <any>
#include <typeinfo>
#include <iostream>
#include <string>

namespace datahandler {

// 参数类型枚举
enum class ParamType {
    BOOL,
    INT8,
    UINT8,
    INT16,
    UINT16,
    INT32,
    UINT32,
    INT64,
    UINT64,
    FLOAT,
    DOUBLE,
    STRING,
    BOOL_ARRAY,
    INT8_ARRAY,
    UINT8_ARRAY,
    INT16_ARRAY,
    UINT16_ARRAY,
    INT32_ARRAY,
    UINT32_ARRAY,
    INT64_ARRAY,
    UINT64_ARRAY,
    FLOAT_ARRAY,
    DOUBLE_ARRAY,
    STRING_ARRAY,
    UNKNOWN
};

// 获取类型字符串
std::string getTypeString(ParamType type);

// 参数信息结构体
struct ParamInfo {
    std::string name;              // 参数名称
    ParamType type;                // 参数类型
    std::any value;                // 参数值（使用std::any存储不同类型）
    void* ptr;                     // 参数内存地址指针
    size_t size;                   // 参数大小
    
    // 构造函数
    ParamInfo() : type(ParamType::UNKNOWN), ptr(nullptr), size(0) {}
    
    template<typename T>
    ParamInfo(const std::string& n, T& v) : name(n), ptr(&v), size(sizeof(T)) {
        setValue(v);
    }
    
    // 设置值的模板函数
    template<typename T>
    bool setValue(T& val) {
        value = val;
        setType<T>();
        return updatePtrVal();
    }
    
    // 获取值的模板函数
    template<typename T>
    T getValue() const {
        return std::any_cast<T>(value);
    }
    
    // 根据类型设置ParamType
    template<typename T>
    void setType();

    
    // 获取类型的字符串表示
    std::string getTypeString() const
    {
        return ::datahandler::getTypeString(type);
    }
    
    // 将value值刷新赋值给ptr指针内存
    bool updatePtrVal();
};

// 基础类型特化实现
template<> inline void ParamInfo::setType<bool>() { type = ParamType::BOOL; }
template<> inline void ParamInfo::setType<int8_t>() { type = ParamType::INT8; }
template<> inline void ParamInfo::setType<uint8_t>() { type = ParamType::UINT8; }
template<> inline void ParamInfo::setType<int16_t>() { type = ParamType::INT16; }
template<> inline void ParamInfo::setType<uint16_t>() { type = ParamType::UINT16; }
template<> inline void ParamInfo::setType<int32_t>() { type = ParamType::INT32; }
template<> inline void ParamInfo::setType<uint32_t>() { type = ParamType::UINT32; }
template<> inline void ParamInfo::setType<int64_t>() { type = ParamType::INT64; }
template<> inline void ParamInfo::setType<uint64_t>() { type = ParamType::UINT64; }
template<> inline void ParamInfo::setType<float>() { type = ParamType::FLOAT; }
template<> inline void ParamInfo::setType<double>() { type = ParamType::DOUBLE; }
template<> inline void ParamInfo::setType<std::string>() { type = ParamType::STRING; }
template<> inline void ParamInfo::setType<std::vector<bool>>() { type = ParamType::BOOL_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<int8_t>>() { type = ParamType::INT8_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<uint8_t>>() { type = ParamType::UINT8_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<int16_t>>() { type = ParamType::INT16_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<uint16_t>>() { type = ParamType::UINT16_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<int32_t>>() { type = ParamType::INT32_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<uint32_t>>() { type = ParamType::UINT32_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<int64_t>>() { type = ParamType::INT64_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<uint64_t>>() { type = ParamType::UINT64_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<float>>() { type = ParamType::FLOAT_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<double>>() { type = ParamType::DOUBLE_ARRAY; }
template<> inline void ParamInfo::setType<std::vector<std::string>>() { type = ParamType::STRING_ARRAY; }

// 将参数值转换为字符串表示
std::string convertParamValueToString(const ParamInfo& param);

//将value值刷新赋值给ptr指针内存
bool updatePtrVal(ParamInfo& param);

// 获取参数索引字符串的宽度，以"9999"的长度为统一长度
size_t getDefaultParamIndexWidth();

// 打印日志输出函数
void printLog(const ParamInfo& param, const std::string& project_path, int log_level, bool bShowExMsg, 
    int color, const char* file_name_path, const char* func, int line, 
    uint16_t maxNameWidth = 25, uint16_t maxTypeWidth = 15, uint16_t maxValueWidth = 30, int paraIdx = -1);

void printLog(const std::vector<ParamInfo>& params, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

// 配置反射器基类
class ConfigReflector {
public:
    virtual ~ConfigReflector() = default;
    
    // 获取参数信息列表
    const std::vector<ParamInfo>& getParams() const { return params_; }
    
    // 获取需要从文件保存/读取的参数信息列表（纯虚函数，强制派生类实现）
    virtual const std::vector<ParamInfo>& getParamsSaved() const = 0;
    
    // 虚函数接口：在加载配置文件后更新参数
    virtual void updateParamAftLoad();
    
    // 根据名称获取参数信息
    const ParamInfo* getParamByName(const std::string& name) const;

    // 静态函数：获取参数列表名的最大宽度
    static size_t getMaxParamNameWidth(const std::vector<ParamInfo>& params);
    
    // 静态函数：获取参数列表类型的最大宽度
    static size_t getMaxParamTypeWidth(const std::vector<ParamInfo>& params);
    
    // 静态函数：获取参数列表值的最大宽度
    static size_t getMaxParamValueWidth(const std::vector<ParamInfo>& params);
 
    // 打印参数列表的日志信息
    void printLog(const std::string& project_path, int log_level, 
        int color, const char* file_name_path, const char* func, int line) const;

    // 注册参数的模板函数
    template<typename T>
    void registerParam(const std::string& name, T& value) {
        params_.emplace_back(name, value);
    }
    
protected:
    std::vector<ParamInfo> params_;
};

} // namespace datahandler

#endif // DATA_HANDLER_PARAM_REFLECTOR_HPP