/**
 * @file param_to_server.h
 * @brief ROS通信工具函数
 *
 * 提供系统配置信息与参数服务器格式之间的转换函数，以及通信信息解析功能
 */
#ifndef BAS_SYS_CONFIG_ROS__CALIB_INFO_SERVER_H_
#define BAS_SYS_CONFIG_ROS__CALIB_INFO_SERVER_H_

#include "bas_sys_config/sys_config_struct.hpp"
#include "bas_operate_ros/ros_comm_info.h"
#include "hand_eye_calib/calib_struct.hpp"
#include "data_handler/param_reflector.hpp"
#include <map>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

// 前向声明
namespace SysConfig {
    using ParamInfo = ::datahandler::ParamInfo;
}
namespace RosComm
{

/**
 * @brief 获取ArmCalibInfo参数前缀
 * @param cam_id 机械臂标定信息结构体引用
 * @param arm_id 参数前缀字符串，不能为空
 * @return std::string ArmCalibInfo参数前缀
 */
std::string getArmCalibInfoParamPrefix(uint8_t cam_id, uint8_t arm_id);

/**
 * @brief 将单个机械臂的标定信息转换为参数服务器可存储的格式
 * 该函数用于将单个机械臂的标定信息转换为参数向量，便于存储到ROS 2参数服务器中。
 * @param para_info 机械臂标定信息结构体引用
 * @param param_prefix 参数前缀字符串，不能为空
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armCalibInfoToRos(const handeyecalib::ArmCalibInfo& para_info, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras);
/**
 * @brief 将ArmCalibInfoList转换为参数服务器可存储的格式
 * 该函数用于将机械臂标定信息列表转换为参数向量，便于批量存储到ROS 2参数服务器中。
 * @param para_infos 机械臂标定信息列表引用
 * @param param_prefix 参数前缀字符串，不能为空
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armCalibInfoListToRos(const handeyecalib::ArmCalibInfoList& para_infos, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将CamCalibInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换CamCalibInfo的所有参数，避免手动逐个转换。
 * @param para_info 相机标定信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camCalibInfoToRos(const handeyecalib::CamCalibInfo& para_info, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras);
/**
 * @brief 将CamCalibInfoList转换为参数服务器可存储的格式
 * 该函数用于将相机标定信息列表转换为参数向量，便于批量存储到ROS 2参数服务器中。
 * @param para_infos 相机标定信息列表
 * @param param_prefix 参数前缀字符串，不能为空
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 * @throws std::invalid_argument 当param_prefix为空时抛出异常
 */
bool camCalibInfoListToRos(const handeyecalib::CamCalibInfoList& para_infos, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将CamCalibInfoList转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数用于将相机标定信息列表转换为参数并向参数服务器声明和设置这些参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param para_infos 相机标定信息列表引用
 * @param param_prefix 参数前缀字符串
 * @return 操作是否成功
 */
bool setCamCalibInfoListToServer(rclcpp::Node::SharedPtr node, 
    const handeyecalib::CamCalibInfoList& para_infos, const std::string& param_prefix);

/**
 * @brief 将CalibRes转换为参数服务器可存储的格式
 * 该函数将CalibRes结构体中的所有参数转换为参数向量，便于存储到ROS 2参数服务器中。
 * @param para_info 标定结果结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool calibResToRos(const handeyecalib::CalibRes& para_info, const std::string& param_prefix,
    std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将QualityMetrics转换为参数服务器可存储的格式
 * 该函数将QualityMetrics结构体中的所有参数转换为参数向量，便于存储到ROS 2参数服务器中。
 * @param para_info 质量评估指标结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool qualityMetricsToRos(const handeyecalib::QualityMetrics& para_info, const std::string& param_prefix,
    std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 从参数服务器读取单个ArmCalibInfo
 * 该函数用于从参数服务器中读取单个机械臂标定信息。
 * @param client 参数服务器客户端节点
 * @param arm_id 机械臂ID
 * @param arm_calib_info[out] 读取到的机械臂标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info);

bool getArmCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info);

/**
 * @brief 设置单个ArmCalibInfo参数服务器
 * 该函数用于从参数服务器中读取单个机械臂标定信息。
 * @param client 参数服务器客户端节点
 * @param arm_id 机械臂ID
 * @param arm_calib_info[out] 读取到的机械臂标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool SetArmCalibInfoToServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info);

/**
 * @brief 从参数服务器读取ArmCalibInfoList数据
 * 该函数用于从参数服务器中读取机械臂标定信息列表，并返回读取状态。
 * 函数使用索引方式遍历所有可能的机械臂标定配置
 * 通过检查param_prefix.arm_calib_{index}.arm_id参数是否存在来判断。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param arm_calib_list[out] 读取到的机械臂标定信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmCalibInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& param_prefix, handeyecalib::ArmCalibInfoList& arm_calib_list);

/**
 * @brief 从参数服务器读取CalibRes数据
 * 该函数用于从参数服务器中读取标定结果信息，并返回读取状态。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param[out] calib_res 读取到的标定结果信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCalibResFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& param_prefix, handeyecalib::CalibRes& calib_res);

/**
 * @brief 从参数服务器读取单个CamCalibInfo
 * 该函数用于从参数服务器中读取单个相机标定信息。
 * 该函数会从sys_cam_calib_list.cam_{cam_id}路径读取相机标定信息
 * @param client 参数服务器客户端节点
 * @param cam_id 相机ID
 * @param cam_calib_info[out] 读取到的相机标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, handeyecalib::CamCalibInfo& cam_calib_info);

/**
 * @brief 从参数服务器读取CamCalibInfoList数据
 * 该函数用于从参数服务器中读取相机标定信息列表，并返回读取状态。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param cam_calib_list[out] 读取到的相机标定信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamCalibInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& param_prefix, handeyecalib::CamCalibInfoList& cam_calib_list);

/**
 * @brief 从参数服务器读取QualityMetrics数据
 * 该函数用于从参数服务器中读取质量评估指标信息，并返回读取状态。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param[out] quality_metrics 读取到的质量评估指标结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getQualityMetricsFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& param_prefix, handeyecalib::QualityMetrics& quality_metrics);

}
#endif  // BAS_SYS_CONFIG_ROS__CALIB_INFO_SERVER_H_