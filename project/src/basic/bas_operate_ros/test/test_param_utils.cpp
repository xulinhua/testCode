#include "bas_operate_ros/param_utils.hpp"
#include "data_handler/param_reflector.hpp"
#include "log_system/log_macros.hpp"
#include <iostream>
#include <vector>

int main() {
    std::cout << "Testing modified printLog_paraInfo functions..." << std::endl;

    // 创建测试参数
    std::vector<datahandler::ParamInfo> params;
    
    // 添加一个bool类型参数
    datahandler::ParamInfo bool_param;
    bool_param.name = "test_bool";
    bool_param.type = datahandler::ParamType::BOOL;
    bool_param.value = true;
    params.push_back(bool_param);
    
    // 添加一个int类型参数
    datahandler::ParamInfo int_param;
    int_param.name = "test_int";
    int_param.type = datahandler::ParamType::INT32;
    int_param.value = 42;
    params.push_back(int_param);
    
    // 添加一个string类型参数
    datahandler::ParamInfo string_param;
    string_param.name = "test_string";
    string_param.type = datahandler::ParamType::STRING;
    string_param.value = std::string("Hello, World!");
    params.push_back(string_param);
    
    // 添加一个float类型参数
    datahandler::ParamInfo float_param;
    float_param.name = "test_float";
    float_param.type = datahandler::ParamType::FLOAT;
    float_param.value = 3.14159f;
    params.push_back(float_param);

    // 测试单个参数打印
    std::cout << "\nTesting single parameter print:" << std::endl;
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    uint16_t param_id = 1;
    basros::printLog_paraInfo(params[0], "test_project", (int)log_level, "test_prefix", param_id, true, (int)color, __FILE__, __FUNCTION__, __LINE__);
    // 测试批量参数打印
    std::cout << "\nTesting batch parameter print:" << std::endl;
    if (LOG_ON("test_project", log_level))
    {
        basros::printLog_paraInfo(params, "test_project", (int)log_level, "test_prefix", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }

    std::cout << "\nTest completed successfully!" << std::endl;
    return 0;
}