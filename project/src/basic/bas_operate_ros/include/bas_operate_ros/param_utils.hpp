#ifndef BAS_OPERATE_ROS_PARAM_UTILS_HPP
#define BAS_OPERATE_ROS_PARAM_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"
#include "bas_operate_ros/ros_comm_info.h"
#include <iostream>

namespace basros {

/**
 * @brief 将bas_operate中的ParamType转换为ROS ParameterType
 * @param param_type bas_operate中的参数类型
 * @return ROS中的参数类型
 */
uint8_t param_type_to_ros_type(datahandler::ParamType param_type);

/**
 * @brief 将ROS参数类型转换为字符串表示
 * @param param_type ROS参数类型
 * @return 参数类型的字符串表示
 */
std::string rosTypeName(const rclcpp::Parameter& param);

/**
 * @brief 生成ROS通信名称
 * 根据消息类型、相机ID和机械臂ID生成对应的ROS服务或话题名称
 * @param msg_type 消息类型枚举
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 生成的通信名称字符串
 */
std::string generateCommName(basros::RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id);

/**
 * @brief 解析通信信息
 * 根据消息类型、相机ID和机械臂ID生成对应的通信信息结构体
 * @param msg_type 消息类型枚举
 * @param cam_id 相机ID
 * @param arm_id 机械臂ID
 * @return 解析后的通信信息结构体
 */
basros::RosCommInfo parseCommInfo(basros::RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id);

std::vector<std::string> updateNodeName(const std::string& prefix_node, const std::vector<std::string>& para_names);

/**
 * @brief 更新节点名称，将前缀节点和参数名组合成完整的节点名
 * @param prefix_node 参数前缀节点名称
 * @param para_name 参数名称
 * @return 组合后的完整节点名称
 *         - 如果prefix_node为空，则直接返回para_name
 *         - 如果prefix_node非空且末尾是"."，para_name开头也是"."，则去除para_name开头的"."
 *         - 如果prefix_node非空且末尾不是"."，para_name开头是"."，则直接连接
 *         - 其他情况在prefix_node和para_name之间添加"."
 */
std::string updateNodeName(const std::string& prefix_node, const std::string& para_name);

/**
 * @brief 更新节点名称，将前缀节点和ParamInfo向量中的参数名组合成完整的节点名向量
 * @param prefix_node 参数前缀节点名称
 * @param para_infos bas_operate中的参数信息向量
 * @return 组合后的完整节点名称向量
 *         - 如果prefix_node为空，则返回原始参数名称列表
 */
std::vector<std::string> updateNodeName(const std::string& prefix_node, const std::vector<datahandler::ParamInfo>& para_infos);

/**
 * @brief 通用参数列表格式化函数（重载1：处理vector<rclcpp::Parameter>）
 * 该函数用于将参数容器中的所有参数格式化为字符串
 * @param params 参数容器引用，vector<rclcpp::Parameter>类型
 * @param description 参数列表的描述信息
 * @return 格式化后的参数列表字符串
 */
std::string formatParamsList(const std::vector<rclcpp::Parameter>& params, const std::string& description);

/**
 * @brief 通用参数列表格式化函数（重载2：处理map<string, rclcpp::Parameter>）
 * 该函数用于将参数容器中的所有参数格式化为字符串
 * @param params 参数容器引用，map<string, rclcpp::Parameter>类型
 * @param description 参数列表的描述信息
 * @return 格式化后的参数列表字符串
 */
std::string formatParamsList(const std::map<std::string, rclcpp::Parameter>& params, const std::string& description);

/**
 * @brief 参数信息打印日志函数，专门用于输出paraInfo参数信息
 * @param param_info bas_operate中的参数信息
 * @param param_id 参数ID引用，用于计数
 * @return 是否成功输出日志
 */
void printLog_paraInfo(const datahandler::ParamInfo& param_info, const std::string& project_path, int log_level, 
    const std::string& prefix_node, uint16_t& param_id, bool bShowExMsg, int color, 
    const char* file_name_path, const char* func, int line);

/**
 * @brief 参数信息打印日志函数，批量处理paraInfo参数向量
 * @param prefix_node 参数前缀节点名称
 * @param param_infos bas_operate中的参数信息向量
 * @return 是否成功输出日志
 */
void printLog_paraInfo(const std::vector<datahandler::ParamInfo>& param_infos, const std::string& project_path, int log_level, 
    const std::string& prefix_node, int color, const char* file_name_path, const char* func, int line);
    
/**
 * @brief 参数信息打印日志函数，专门用于输出ROS参数形式Parameter的参数信息（对齐格式）
 * 参照param_reflector.cpp中的printLog函数实现，输出节点名、参数类型、参数值并保持对齐
 * @param project_path 项目路径
 * @param log_level 日志级别
 * @param param ROS参数对象
 * @param bShowExMsg 是否显示扩展消息
 * @param maxNameWidth 参数名最大宽度
 * @param maxTypeWidth 参数类型最大宽度
 * @param maxValueWidth 参数值最大宽度
 * @param paraIdx 参数索引
 * @return 是否成功输出日志
 */
bool printLog_rosParam(const rclcpp::Parameter& param, const std::string& project_path, int log_level, 
              uint16_t maxNameWidth, uint16_t maxTypeWidth, uint16_t maxValueWidth, int paraIdx, bool bShowExMsg, 
              int color, const char* file_name_path, const char* func, int line);

/**
 * @brief 参数信息打印日志函数，批量处理ROS rosParam参数向量
 * @param ros_paras ROS参数对象向量
 * @return 是否成功输出日志
 */
bool printLog_rosParam(const std::vector<rclcpp::Parameter>& ros_paras, const std::string& project_path, int log_level, 
    int color, const char* file_name_path, const char* func, int line);

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
    int color, const char* file_name_path, const char* func, int line);

/**
 * @brief 参数信息打印日志函数，批量处理参数向量
 * @param param_info bas_operate中的参数信息向量
 * @param param ROS参数对象向量
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功输出日志
 */
bool printLog_paraInfo_rosPara(const std::vector<datahandler::ParamInfo>& param_infos, const std::vector<rclcpp::Parameter>& ros_paras, 
    const std::string& project_path, int log_level, const std::string& prefix_node, 
    int color, const char* file_name_path, const char* func, int line);

} // namespace basros

#endif // BAS_OPERATE_ROS_PARAM_UTILS_HPP