/**
 * @file ros_comm_info.h
 * @brief ROS通信信息数据结构定义和通信类型枚举
 * 
 * 该文件定义了ROS通信信息的数据结构RosCommInfo和通信类型的枚举类RosCommMsgType，
 * 用于在不同ROS项目之间共享通信配置信息。
 */

#ifndef BAS_OPERATE_ROS__ROS_COMM_INFO_H_
#define BAS_OPERATE_ROS__ROS_COMM_INFO_H_

#include <string>
#include <cstdint>

namespace basros
{

/**
 * @brief ROS通信方式枚举
 * 
 * 定义ROS中可用的通信方式
 */
enum class RosCommMethod : uint8_t
{
  TOPIC = 0,      ///< 话题通信
  SERVICE = 1,    ///< 服务通信
  PARAMETER = 2,  ///< 参数服务器
  ACTION = 3      ///< 动作通信
};

/**
 * @brief ROS通信消息类型枚举
 * 
 * 定义系统中各种通信服务或话题的消息类型，用于标识不同的通信目的
 */
enum class RosCommMsgType : uint16_t
{
  COMM_SYS_CAM_NUM = 0,             ///< 获取系统相机个数的服务/话题名
  COMM_CAM_INTRINSICS,              ///< 获取相机内参的话题名
  COMM_CAM_INTRINSICS_SRV,          ///< 获取相机内参的服务名
  COMM_MODULE_INFO_CAM,             ///< 获取模块信息相机的服务/话题名
  COMM_MODULE_INFO_PCL,             ///< 获取模块信息点云转激光的服务/话题名
  COMM_SRC_COLOR_IMAGE,             ///< 获取源彩色图像的服务/话题名
  COMM_SRC_DEPTH_IMAGE,             ///< 获取源深度图像的服务/话题名
  COMM_SRC_POINT_CLOUD,             ///< 获取源点云数据的服务/话题名
  COMM_MARKER_RESULTS,               ///< 获取标记识别结果的服务/话题名
  COMM_MARKER_CALIB_RESULTS,         ///< 获取标记识别标定转换后的结果的服务/话题名
  COMM_MARKER_SRC_IMAGE,             ///< 获取标记识别结果渲染图像的服务/话题名
  COMM_MARKER_RESULTS_IMAGE,         ///< 获取标记识别结果渲染图像的服务/话题名
  COMM_MARKER_MARKERS_INFO,          ///< 获取标记信息的服务/话题名
  COMM_MARKER_CLEAR,                 ///< 获取标记识别结果清理的服务/话题名
  COMM_ARM_CURRENT_POSE,            ///< 获取机械臂当前位姿的服务/话题名
  COMM_MAX
};

/**
 * @brief ROS通信信息数据结构
 * 
 * 包含进行ROS通信所需的所有信息，如通信方式、相机ID、机械臂ID和服务/话题名称等
 */
struct RosCommInfo
{
  //RosCommMethod comm_method;  ///< 通信方式
  uint8_t cam_id;             ///< 相机ID
  uint8_t arm_id;             ///< 机械臂ID
  std::string name;           ///< 服务/话题名称
};

}  // namespace basros

#endif  // BAS_OPERATE_ROS__ROS_COMM_INFO_H_