/**
 * @file calib_param_handler.h
 * @brief 统一标定参数处理器
 * 
 * 提供使用ROS 2原生回调机制处理各种标定参数的通用类
 * 支持相机标定参数和机械臂标定参数的统一处理
 */

#ifndef BAS_SYS_CONFIG_ROS__CALIB_PARAM_HANDLER_H_
#define BAS_SYS_CONFIG_ROS__CALIB_PARAM_HANDLER_H_

#include "hand_eye_calib/calib_struct.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <map>
#include <vector>
#include <regex>

namespace RosComm {

/**
 * @brief 标定参数处理器类模板
 * 
 * 使用ROS 2原生add_on_set_parameters_callback回调机制，
 * 实现标定参数变化时的自动处理
 */
template<typename DataType>
class CalibParamHandler {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     */
    explicit CalibParamHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数
     */
    ~CalibParamHandler();

    /**
     * @brief 注册标定参数回调函数
     * @param callback 回调函数，当标定参数发生变化时调用
     */
    void registerCalibParamCallback(std::function<void(const DataType&)> callback);

    /**
     * @brief 从参数服务器获取标定数据
     * @param param_prefix 参数前缀
     * @param calib_data[out] 标定数据
     * @return 是否成功获取
     */
    bool getCalibDatFromServer(const std::string& param_prefix, DataType& calib_data);

    /**
     * @brief 获取最新的标定数据
     * @return 最新的标定数据
     */
    const DataType& getLatestCalibData() const;

protected:
    /**
     * @brief 参数回调函数
     * @param parameters 参数列表
     * @return 参数设置结果
     */
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);

    // 成员变量
    rclcpp::Node::SharedPtr node_;  ///< ROS节点指针
    std::function<void(const DataType&)> callback_;  ///< 回调函数
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  ///< 参数回调句柄
    DataType latest_data_;  ///< 最新数据
};

// 类型别名，方便使用
using CamCalibParamHandler = CalibParamHandler<handeyecalib::CamCalibInfoList>;
using ArmCalibParamHandler = CalibParamHandler<handeyecalib::ArmCalibInfo>;

}  // namespace RosComm

#include "calib_param_handler.tpp"

#endif  // BAS_SYS_CONFIG_ROS__CALIB_PARAM_HANDLER_H_