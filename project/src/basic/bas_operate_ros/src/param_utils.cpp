#include "bas_operate_ros/param_utils.hpp"
#include "log_system/log_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include <sstream>
#include <iomanip>
#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"

namespace basros {

// 辅助函数：将rclcpp::Parameter的值转换为字符串表示
std::string convertRosParamValueToString(const rclcpp::Parameter& param)
{
    switch (param.get_type()) 
    {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return param.as_bool() ? "true" : "false";
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return std::to_string(param.as_int());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            {
                std::ostringstream double_stream;
                double_stream << std::fixed << std::setprecision(6) << param.as_double();
                return double_stream.str();
            }
        case rclcpp::ParameterType::PARAMETER_STRING:
            return param.as_string();
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            {
                auto values = param.as_bool_array();
                return basmodule::get_list_string(values);
            }
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            {
                auto values = param.as_integer_array();
                return basmodule::get_list_string(values);
            }
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            {
                auto values = param.as_double_array();
                return basmodule::format_float_array(values, 6);
            }
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            {
                auto values = param.as_string_array();
                return basmodule::get_list_string(values);
            }
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            {
                auto values = param.as_byte_array();
                return basmodule::get_list_string(values);
            }
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
        default:
            return "NOT_SET";
    }
}

/**
 * @brief 将bas_operate中的ParamType转换为ROS ParameterType
 * @param param_type bas_operate中的参数类型
 * @return ROS中的参数类型
 */
uint8_t param_type_to_ros_type(datahandler::ParamType param_type) 
{
    switch (param_type) 
    {
        case datahandler::ParamType::BOOL:
            return rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        case datahandler::ParamType::INT8:
        case datahandler::ParamType::INT16:
        case datahandler::ParamType::INT32:
        case datahandler::ParamType::UINT8:
        case datahandler::ParamType::UINT16:
        case datahandler::ParamType::UINT32:
        case datahandler::ParamType::INT64:
        case datahandler::ParamType::UINT64:
            return rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        case datahandler::ParamType::FLOAT:
        case datahandler::ParamType::DOUBLE:
            return rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        case datahandler::ParamType::STRING:
            return rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        case datahandler::ParamType::BOOL_ARRAY:
            return rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
        case datahandler::ParamType::INT8_ARRAY:
        case datahandler::ParamType::INT16_ARRAY:
        case datahandler::ParamType::INT32_ARRAY:
        case datahandler::ParamType::UINT8_ARRAY:
        case datahandler::ParamType::UINT16_ARRAY:
        case datahandler::ParamType::UINT32_ARRAY:
        case datahandler::ParamType::INT64_ARRAY:
        case datahandler::ParamType::UINT64_ARRAY:
            return rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
        case datahandler::ParamType::FLOAT_ARRAY:
        case datahandler::ParamType::DOUBLE_ARRAY:
            return rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        case datahandler::ParamType::STRING_ARRAY:
            return rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
        default:
            return rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
    }
}

std::string rosTypeName(const rclcpp::Parameter& param)
{
    switch (param.get_type()) 
    {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return "PARAMETER_BOOL";
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return "PARAMETER_INTEGER";
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return "PARAMETER_DOUBLE";
        case rclcpp::ParameterType::PARAMETER_STRING:
            return "PARAMETER_STRING";
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return "PARAMETER_BOOL_ARRAY";
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return "PARAMETER_INTEGER_ARRAY";
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return "PARAMETER_DOUBLE_ARRAY";
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return "PARAMETER_STRING_ARRAY";
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            return "PARAMETER_BYTE_ARRAY";
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
        default:
            return "PARAMETER_NOT_SET";
    }
}

std::string generateCommName(basros::RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id)
{
    std::string name;
    // 根据消息类型生成名称
    switch (msg_type)
    {
        case basros::RosCommMsgType::COMM_SYS_CAM_NUM:                 ///< 获取系统相机个数的服务/话题名
            name = "sys_cam_num";
            break;
        case basros::RosCommMsgType::COMM_CAM_INTRINSICS:              ///< 获取相机内参的话题名
            name = "cam_" + std::to_string(cam_id) + "/intrinsics";
            break;
        case basros::RosCommMsgType::COMM_CAM_INTRINSICS_SRV:          ///< 获取相机内参的服务名
            name = "cam_/get_intrinsics";
            break;
        case basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE:             ///< 获取源彩色图像的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/src_color_image";
            break;
        case basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE:             ///< 获取源深度图像的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/src_depth_image";
            break;
        case basros::RosCommMsgType::COMM_SRC_POINT_CLOUD:             ///< 获取点云的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/src_point_cloud";
            break;
        case basros::RosCommMsgType::COMM_MARKER_RESULTS:               ///< 获取marker识别结果的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/results";
            break;
        case basros::RosCommMsgType::COMM_MARKER_CALIB_RESULTS:         ///< 获取marker识别标定转换后的结果的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/calib_results";
            break;
        case basros::RosCommMsgType::COMM_MARKER_SRC_IMAGE:         ///< 获取marker识别结果渲染图像的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/src_image";
            break;
        case basros::RosCommMsgType::COMM_MARKER_RESULTS_IMAGE:         ///< 获取marker识别结果渲染图像的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/results_image";
            break;
        case basros::RosCommMsgType::COMM_MARKER_MARKERS_INFO:           ///< 获取marker标记信息的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/markers_info";
            break;
        case basros::RosCommMsgType::COMM_MARKER_CLEAR:                 ///< 获取marker识别结果清理的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/marker_detection/clear";
            break;
        case basros::RosCommMsgType::COMM_ARM_CURRENT_POSE:           ///< 获取机械臂当前位姿的服务/话题名
            name = "robot_cartesian_pose";
            break;
        case basros::RosCommMsgType::COMM_MODULE_INFO_CAM:            ///< 获取模块信息相机的服务/话题名
            name = "cam_" + std::to_string(cam_id) + "/module_status";
            break;
        case basros::RosCommMsgType::COMM_MODULE_INFO_PCL:            ///< 获取模块信息点云转激光的服务/话题名
            name = "pcl2laserscan/module_status";
            break;
        case basros::RosCommMsgType::COMM_MAX:
        default:
            name = "服务/话题名称异常！";
            LOG_FATAL("%s", name.c_str());
            throw std::invalid_argument(name);
            name.clear();
            break;
    }
    return name;
}

basros::RosCommInfo parseCommInfo(basros::RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id)
{
    basros::RosCommInfo comm_info;
    comm_info.cam_id = cam_id;
    comm_info.arm_id = arm_id;
    comm_info.name = generateCommName(msg_type, cam_id, arm_id);
    return comm_info;
}

std::vector<std::string> updateNodeName(const std::string& prefix_node, const std::vector<std::string>& para_names)
{
    std::vector<std::string> full_para_names;
    full_para_names.reserve(para_names.size());
    for (const auto& para_name : para_names) // 构建完整参数名列表
    {
        std::string full_name = updateNodeName(prefix_node, para_name);
        full_para_names.push_back(full_name);
    }
    return full_para_names;
}

std::string updateNodeName(const std::string& prefix_node, const std::string& para_name)
{
    if (prefix_node.empty()) {
        return para_name;
    }
    // 检查prefix_node的最后一个字符是否是"."
    if (prefix_node.back() == '.') 
    {
        // 如果para_name第一个字符也是"."，则去掉para_name的开头"."
        if (!para_name.empty() && para_name[0] == '.') {
            return prefix_node + para_name.substr(1);
        }
        return prefix_node + para_name;
    } 
    else 
    {
        // 如果不是"."，则需要额外加"."
        // 但如果para_name第一个字符是"."，则不需要额外添加"."
        if (!para_name.empty() && para_name[0] == '.') {
            return prefix_node + para_name;
        }
        return prefix_node + "." + para_name;
    }
}

std::vector<std::string> updateNodeName(const std::string& prefix_node, const std::vector<datahandler::ParamInfo>& para_infos)
{
    if (prefix_node.empty()) 
    {
        LOG_WARN("参数前缀节点名称不能为空");
        // 如果prefix_node为空，直接返回参数名称列表
        std::vector<std::string> para_names;
        para_names.reserve(para_infos.size());
        for (const auto& para_info : para_infos) {
            para_names.push_back(para_info.name);
        }
        return para_names;
    }
    std::vector<std::string> para_names;// 获取参数名称列表
    para_names.reserve(para_infos.size());
    for (const auto& para_info : para_infos) {
        para_names.push_back(updateNodeName(prefix_node, para_info.name));
    }
    return para_names;
}

std::string formatParamsList(const std::vector<rclcpp::Parameter>& params, const std::string& description)
{
    // 构建参数列表字符串，按索引格式一次性输出
    std::string params_str = "\n";
    int index = 1;
    int total_params = params.size();
    int max_width = std::to_string(total_params).length(); // 计算最大参数数量的位数
    for (const auto& param : params) 
    {
        char index_str[10];
        sprintf(index_str, "%*d", max_width, index); // 使用动态宽度，右对齐
        params_str += "  参数" + std::string(index_str) + ": " + param.get_name() + " = " + param.value_to_string() + "\n";
        index++;
    }
    // 如果description为空或最后一个字符是':'，则不添加冒号
    if (description.empty() || description.back() == ':') {
        return description + params_str;
    } else {
        return description + ":" + params_str;
    }
}

std::string formatParamsList(const std::map<std::string, rclcpp::Parameter>& params, const std::string& description)
{
    // 构建参数列表字符串，按索引格式一次性输出
    std::string params_str = "\n";
    int index = 1;
    int total_params = params.size();
    int max_width = std::to_string(total_params).length(); // 计算最大参数数量的位数
    for (const auto& param : params) 
    {
        char index_str[10];
        sprintf(index_str, "%*d", max_width, index); // 使用动态宽度，右对齐
        params_str += "  参数" + std::string(index_str) + ": " + param.first + " = " + param.second.value_to_string() + "\n";
        index++;
    }
    // 如果description为空或最后一个字符是':'，则不添加冒号
    if (description.empty() || description.back() == ':') {
        return description + params_str;
    } else {
        return description + ":" + params_str;
    }
}

void printLog_paraInfo(const datahandler::ParamInfo& param_info, const std::string& project_path, int log_level, 
    const std::string& prefix_node, uint16_t& param_id, bool bShowExMsg, int color, 
    const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    // 创建一个修改后的ParamInfo，将name更新为带前缀的名称
    datahandler::ParamInfo modified_param = param_info;
    modified_param.name = updateNodeName(prefix_node, param_info.name);
    uint16_t maxNameWidth = 50; 
    uint16_t maxTypeWidth = 15;
    uint16_t maxValueWidth = 30;
    int paraIdx = param_id;  // 使用当前参数ID作为索引
    datahandler::printLog(modified_param, project_path, log_level, bShowExMsg, color, 
        file_name_path, func, line, maxNameWidth, maxTypeWidth, maxValueWidth, paraIdx);
    param_id++;  // 增加参数ID计数器
}

/**
 * @brief 参数信息打印日志函数，批量处理paraInfo参数向量
 * @param prefix_node 参数前缀节点名称
 * @param param_infos bas_operate中的参数信息向量
 * @return 是否成功输出日志
 */
void printLog_paraInfo(const std::vector<datahandler::ParamInfo>& param_infos, const std::string& project_path, int log_level, 
    const std::string& prefix_node, int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    bool bShowExMsg = false; // 定义bShowExMsg变量
    if (param_infos.empty()) // 如果参数向量为空，输出相应信息
    {
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "无参数可打印");
        return;
    }
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        bShowExMsg, static_cast<logsys::Color>(color), "paraInfo参数列表：");
    std::vector<datahandler::ParamInfo> modified_params;// 创建一个修改后的ParamInfo向量，将每个参数的name更新为带前缀的名称
    modified_params.reserve(param_infos.size());
    for (const auto& param_info : param_infos) 
    {
        datahandler::ParamInfo modified_param = param_info;
        modified_param.name = updateNodeName(prefix_node, param_info.name);
        modified_params.push_back(modified_param);
    }
    datahandler::printLog(modified_params, project_path, log_level, color, file_name_path, func, line);// 调用param_reflector.cpp中的printLog函数实现对齐格式
}

/**
 * @brief 参数信息打印日志函数，批量处理ROS rosParam参数向量
 * @param ros_paras ROS参数对象向量
 * @return 是否成功输出日志
 */
// 计算ROS参数列表的最大宽度
size_t getMaxParamNameWidth(const std::vector<rclcpp::Parameter>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxNameWidth = std::string("参数名: ").length();
    uint8_t offsetAddWidth = 4;
    for (const auto& param : params) 
    {
        maxNameWidth = std::max(maxNameWidth, param.get_name().length() + offsetAddWidth);
    }
    return maxNameWidth;
}

size_t getMaxParamTypeWidth(const std::vector<rclcpp::Parameter>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxTypeWidth = std::string("类型: ").length();
    uint8_t offsetAddWidth = 4;
    for (const auto& param : params) 
    {
        std::string param_type_str = rosTypeName(param);
        maxTypeWidth = std::max(maxTypeWidth, param_type_str.length() + offsetAddWidth);
    }
    return maxTypeWidth;
}

size_t getMaxParamValueWidth(const std::vector<rclcpp::Parameter>& params)
{
    if (params.empty()) {
        return 0;
    }
    size_t maxValueWidth = std::string("值: ").length();
    uint8_t offsetAddWidth = 4;
    for (const auto& param : params) 
    {
        std::string param_value_str = convertRosParamValueToString(param);
        maxValueWidth = std::max(maxValueWidth, param_value_str.length() + offsetAddWidth);
    }
    return maxValueWidth;
}

bool printLog_rosParam(const std::string& project_path, int log_level, const rclcpp::Parameter& param, 
              uint16_t maxNameWidth, uint16_t maxTypeWidth, uint16_t maxValueWidth, int paraIdx, bool bShowExMsg, 
              int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return true;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    // 如果传入的宽度为默认值，则使用默认宽度
    uint16_t actualMaxNameWidth = maxNameWidth;
    uint16_t actualMaxTypeWidth = maxTypeWidth;
    uint16_t actualMaxValueWidth = maxValueWidth;
    
    // 如果传入的宽度为0，则使用默认宽度
    if (maxNameWidth == 0 || maxTypeWidth == 0 || maxValueWidth == 0) 
    {
        // 如果计算结果为0，则使用默认宽度
        if (actualMaxNameWidth == 0) actualMaxNameWidth = 50;
        if (actualMaxTypeWidth == 0) actualMaxTypeWidth = 15;
        if (actualMaxValueWidth == 0) actualMaxValueWidth = 30;
    } 
    
    std::string param_value_str = convertRosParamValueToString(param);
    std::string param_type_str = rosTypeName(param);
    
    // 构建格式化输出字符串，确保对齐
    std::string namePart = param.get_name();
    std::string typePart = param_type_str;
    std::string valuePart = param_value_str;
    // 使用指定的宽度进行对齐输出
    std::stringstream ss;
    
    // 添加缩进
    ss << basmodule::generateIndent();
    
    // 索引部分
    std::string indexPart = "";
    size_t indexDisplayWidth = 0;
    if (paraIdx > 0) 
    {
        indexPart = "[" + std::to_string(paraIdx) + "]";
        indexDisplayWidth = basmodule::getDisplayWidth(indexPart);
    }
    
    // 计算索引部分的固定宽度
    size_t indexTargetWidth = datahandler::getDefaultParamIndexWidth();
    
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
    return true;
}

// 打印日志输出函数 - 批量版本
bool printLog_rosParam(const std::vector<rclcpp::Parameter>& ros_paras, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return true;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    bool bShowExMsg = false; // 定义bShowExMsg变量
    if (ros_paras.empty()) 
    {
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "无参数可打印");
        return true;
    }
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        bShowExMsg, static_cast<logsys::Color>(color), "rosParam参数列表：");
    // 计算参数列表的最大宽度
    uint16_t maxNameWidth = getMaxParamNameWidth(ros_paras);
    uint16_t maxTypeWidth = getMaxParamTypeWidth(ros_paras);
    uint16_t maxValueWidth = getMaxParamValueWidth(ros_paras);
    
    // 确保最小宽度不小于表头文字的实际显示宽度
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
    size_t indexTargetWidth = datahandler::getDefaultParamIndexWidth();
    
    // 添加缩进（表头也需要向右偏移）
    header_ss << basmodule::generateIndent();
    
    // 手动构建表头，不使用std::setw
    header_ss << basmodule::padToWidth(indexHeader, indexTargetWidth) << " "
              << basmodule::padToWidth("参数名", maxNameWidth) << " "
              << basmodule::padToWidth("类型", maxTypeWidth) << " "
              << "值";  // 值部分不需要填充
    LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
        bShowExMsg, static_cast<logsys::Color>(color), "%s", header_ss.str().c_str());
    bool overall_success = true;
    int paraIdx = 1; // 参数索引，从1开始
    for (const auto& param : ros_paras) // 逐个打印每个参数
    {
        bool bRet = printLog_rosParam(project_path, log_level, param, maxNameWidth, maxTypeWidth, maxValueWidth, 
            paraIdx, bShowExMsg, color, file_name_path, func, line);
        overall_success = overall_success && bRet;
        paraIdx++;
    }
    return overall_success;
}

/**
 * @brief 参数信息打印日志函数，同时输出ParamInfo和ROS Parameter的参数信息并进行校验
 * @param param_info bas_operate中的参数信息
 * @param param ROS参数对象
 * @param prefix_node 参数前缀节点名称
 * @param param_id 参数ID引用，用于计数
 * @return 是否成功输出日志
 */
bool printLog_paraInfo_rosPara(const datahandler::ParamInfo& param_info, const std::string& project_path, int log_level, 
    const rclcpp::Parameter& param, const std::string& prefix_node, uint16_t& param_id, bool bShowExMsg, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return true;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (node_name != param.get_name()) // 校验参数名称是否匹配
        {
            LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), logsys::Level::ERROR, 
                true, logsys::Color::DEFAULT, "参数名称不匹配: ParamInfo名称=%s, Parameter名称=%s", 
                node_name.c_str(), param.get_name().c_str());
            return false;
        }
        uint8_t param_info_ros_type = param_type_to_ros_type(param_info.type);
        uint8_t param_ros_type = param.get_type();
        if (param_info_ros_type != param_ros_type) // 校验参数类型是否匹配
        {
            LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), logsys::Level::ERROR, 
                true, logsys::Color::DEFAULT, "参数类型不匹配: ParamInfo类型=%d, Parameter类型=%d, 参数名称=%s", 
                param_info_ros_type, param_ros_type, param.get_name().c_str());
            return false;
        }
        uint16_t original_param_id = param_id;// 保存当前参数ID，用于恢复
        // 输出paraInfo参数信息
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "参数%d (ParamInfo): %s = ", param_id, param.get_name().c_str());
        // 调用第一个重载接口输出ParamInfo信息 - 这里应该是printLog_paraInfo
        basros::printLog_paraInfo(const_cast<datahandler::ParamInfo&>(param_info), project_path, log_level, 
            prefix_node, param_id, bShowExMsg, color, file_name_path, func, line);
        param_id = original_param_id;// 重置参数ID为原始值，用于输出ROS Parameter信息
        // 输出ROS rosParam参数信息
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            bShowExMsg, static_cast<logsys::Color>(color), "参数%d (Parameter): %s = ", param_id, param.get_name().c_str());
        // 调用第二个重载接口输出ROS Parameter信息 - 这里应该是printLog_rosParam
        uint16_t maxNameWidth = 0;
        uint16_t maxTypeWidth = 0;
        uint16_t maxValueWidth = 0;
        bool bRet = printLog_rosParam(project_path, log_level, param, maxNameWidth, maxTypeWidth, maxValueWidth, 
            param_id, bShowExMsg, color, file_name_path, func, line);
        return bRet;
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR(project_path, "参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
        param_id++; // 发生异常也要增加计数器
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR(project_path, "转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
        param_id++; // 发生异常也要增加计数器
        return false;
    }
}

/**
 * @brief 参数信息打印日志函数，批量处理参数向量
 * @param param_info bas_operate中的参数信息向量
 * @param param ROS参数对象向量
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功输出日志
 */
bool printLog_paraInfo_rosPara(const std::vector<datahandler::ParamInfo>& param_infos, const std::vector<rclcpp::Parameter>& ros_paras, 
    const std::string& project_path, int log_level, const std::string& prefix_node, 
    int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return true;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    basros::printLog_paraInfo(param_infos, project_path, log_level, prefix_node, color, file_name_path, func, line);
    bool bRet = printLog_rosParam(ros_paras, project_path, log_level, color, file_name_path, func, line);
    if (param_infos.size() != ros_paras.size()) // 检查参数向量大小是否匹配
    {
        LOG_OUT_EX(file_name_path, func, line, project_path.c_str(), static_cast<logsys::Level>(log_level), 
            true, static_cast<logsys::Color>(color), 
            "参数向量大小不匹配: ParamInfo数量=%zu, Parameter数量=%zu", param_infos.size(), ros_paras.size());
        return false;
    }
    return bRet; // 确保两个都成功时才返回true
}

} // namespace basros