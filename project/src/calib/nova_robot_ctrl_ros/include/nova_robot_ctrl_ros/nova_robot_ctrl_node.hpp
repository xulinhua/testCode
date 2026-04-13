#ifndef NOVA_ROBOT_CTRL_ROS__NOVA_ROBOT_CTRL_NODE_HPP_
#define NOVA_ROBOT_CTRL_ROS__NOVA_ROBOT_CTRL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <map>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msgs_comm/msg/robot_std_pose.hpp"
#include "custom_msgs_comm/msg/bool_stamped.hpp"
#include <std_srvs/srv/trigger.hpp>

// 包含自定义服务
#include "custom_msgs_comm/srv/project_control.hpp"
#include "custom_msgs_comm/srv/get_calibration_points.hpp"
#include "custom_msgs_comm/srv/get_robot_pose.hpp"
#include "custom_msgs_comm/srv/keyboard_control.hpp"
#include "custom_msgs_comm/srv/move_to_pose.hpp"


// 包含标准服务
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

// 包含nova_robot_ctrl库
#include "nova_robot_ctrl/nova_robot_ctrl.h"
#include "nova_robot_ctrl/robot_mgr.h"  // 添加RobotMgr头文件

namespace nova_robot_ctrl_ros {

/**
 * @class NovaRobotCtrlNode
 * @brief ROS节点类，用于封装nova_robot_ctrl功能
 * 
 * 该类实现了ROS接口，用于与hand_eye_calib_ros项目进行通信，
 * 控制机械臂移动到指定位置并反馈状态。
 */
class NovaRobotCtrlNode : public rclcpp::Node, public std::enable_shared_from_this<NovaRobotCtrlNode> {
public:
    /**
     * @brief 构造函数
     * @param options ROS节点选项
     */
    explicit NovaRobotCtrlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~NovaRobotCtrlNode();

private:
    /**
     * @brief 移动机械臂到指定位置
     * @param target_pose 目标位姿
     * @param speed 运动速度(%)，默认为30(较慢的速度用于手眼标定)
     * @return 是否移动成功
     */
    bool moveRobotToPose(const geometry_msgs::msg::PoseStamped& target_pose, int speed = 30);

    /**
     * @brief 移动机械臂到指定位置（指定机械臂ID）
     * @param target_pose 目标位姿
     * @param speed 运动速度(%)
     * @param arm_id 机械臂ID
     * @return 是否移动成功
     */
    bool moveRobotToPoseWithArmId(const geometry_msgs::msg::PoseStamped& target_pose, int speed, int arm_id);

    /**
     * @brief 移动机械臂到标准位置
     * @param speed 运动速度(%)，默认为30(较慢的速度用于手眼标定)
     * @return 是否移动成功
     */
    bool moveRobotToStandardPose(int speed = 30);
    
    /**
     * @brief 点动移动机械臂
     * @param axis 移动轴 ('x', 'y', 'z')
     * @param distance 移动距离(mm)
     * @return 是否移动成功
     */
    bool jogMoveRobot(char axis, double distance);
    
    /**
     * @brief 获取机械臂当前位姿
     * @param pose 输出的位姿信息
     * @param arm_id 机械臂ID
     * @return 是否获取成功
     */
    bool getCurrentPose(geometry_msgs::msg::PoseStamped& pose, int arm_id);

    /**
     * @brief 机械臂目标位姿回调函数
     * @param msg 机械臂目标位姿消息
     */
    void robotTargetPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg);

    /**
     * @brief 启动手眼标定回调函数
     */
    void startCalibrationCallback();

    /**
     * @brief 键盘控制服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     */
    void keyboardControlServiceCallback(
        const std::shared_ptr<custom_msgs_comm::srv::KeyboardControl::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::KeyboardControl::Response> response);

    /**
     * @brief 移动到指定位姿服务回调函数
     * @param request 服务请求
     * @param response 服务响应
     */
    void moveToPoseServiceCallback(
        const std::shared_ptr<custom_msgs_comm::srv::MoveToPose::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::MoveToPose::Response> response);

    // 订阅器
    rclcpp::Subscription<custom_msgs_comm::msg::RobotStdPose>::SharedPtr robot_target_sub_;  ///< 机械臂目标位姿订阅器
    rclcpp::Subscription<std_srvs::srv::Trigger>::SharedPtr start_calibration_sub_;      ///< 启动标定订阅器

    // 发布器
    rclcpp::Publisher<custom_msgs_comm::msg::RobotStdPose>::SharedPtr robot_pose_pub_;       ///< 机械臂当前位姿发布器映射表
    rclcpp::Publisher<custom_msgs_comm::msg::BoolStamped>::SharedPtr robot_status_pub_;                 ///< 机械臂状态发布器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_ready_state_pub_;            ///< 机械臂准备状态发布器

    // 服务服务器
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr robot_control_service_server_;           ///< 机械臂控制服务服务器
    // rclcpp::Service<custom_msgs_comm::srv::GetCalibrationPoints>::SharedPtr get_calib_points_service_server_;  ///< 获取标定点数据服务服务器
    rclcpp::Service<custom_msgs_comm::srv::GetRobotPose>::SharedPtr get_robot_pose_service_server_;  ///< 获取机械臂位姿服务服务器
    rclcpp::Service<custom_msgs_comm::srv::KeyboardControl>::SharedPtr keyboard_control_service_server_;  ///< 键盘控制服务服务器
    rclcpp::Service<custom_msgs_comm::srv::MoveToPose>::SharedPtr move_to_pose_service_server_;  ///< 移动到指定位姿服务服务器

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;                                                ///< 定时器，用于周期性检查状态
    rclcpp::TimerBase::SharedPtr param_load_timer_;                                     ///< 定时器，用于从参数服务器加载配置

    nova_robot_ctrl::RobotMgr robot_mgr_;                                               ///< Nova机器人管理对象（包含夹爪控制）

    //机械臂ID
    int arm_id_;              ///< 机械臂ID，用于标识不同的机械臂

    int robot_num_;           ///< 机械臂数量
    std::vector<int> arm_id_list_; ///< 机械臂ID列表
    // 夹爪控制相关变量
    int gripper_position_;      ///< 夹爪位置
    int gripper_speed_;         ///< 夹爪速度
    int gripper_force_;         ///< 夹爪力度
    int gripper_abs_position_;  ///< 夹爪绝对位置
    
    // 机械臂控制相关变量
    double current_x_;          ///< 当前X位置
    double current_y_;          ///< 当前Y位置
    double current_z_;          ///< 当前Z位置
    double step_size_;          ///< 点动步长(mm)

    // 状态变量
    bool robot_connected_;                                                              ///< 机械臂是否已连接
    bool robot_enabled_;                                                                ///< 机械臂是否已使能
    bool robot_moving_;                                                                 ///< 机械臂是否正在移动
    std::string robot_ip_;                                                              ///< 机械臂IP地址
    geometry_msgs::msg::PoseStamped current_robot_pose_;                                ///< 当前机械臂位姿

    /**
     * @brief 初始化参数
     */
    void initParameters();

    /**
     * @brief 初始化订阅器
     */
    void initSubscribers();

    /**
     * @brief 初始化发布器
     */
    void initPublishers();

    /**
     * @brief 初始化服务服务器
     */
    void initServiceServers();
    
    /**
     * @brief 从参数服务器加载机器人系统配置
     * @return true-成功，false-失败
     */
    bool load_robot_sys_srv();

    /**
     * @brief 加载配置
     * @param mode 加载模式，0-从文件加载，1-从参数服务器加载
     * @return true-成功，false-失败
     */
    bool load_config(int mode = 0);

    /**
     * @brief 初始化定时器
     */
    void initTimer();

    /**
     * @brief 连接机械臂
     * @return 是否连接成功
     */
    bool connectRobot();

    /**
     * @brief 使能机械臂
     * @return 是否使能成功
     */
    bool enableRobot();

    /**
     * @brief 验证机械臂是否在指定位置
     * @param target_pose 目标位置
     * @param tolerance 位置容差（毫米），默认为1.0
     * @return 是否在指定位置
     */
    bool verifyRobotAtPosition(const geometry_msgs::msg::PoseStamped& target_pose, double tolerance = 1.0);

    /**
     * @brief 验证指定机械臂是否在指定位置
     * @param target_pose 目标位置
     * @param tolerance 位置容差（毫米），默认为1.0
     * @param arm_id 机械臂ID
     * @return 是否在指定位置
     */
    bool verifyRobotAtPositionWithArmId(const geometry_msgs::msg::PoseStamped& target_pose, double tolerance, int arm_id);

    /**
     * @brief 发布机械臂状态
     * @param is_ready 机械臂是否准备就绪
     */
    void publishRobotStatus(bool is_ready, std::string frame_id = "");

    /**
     * @brief 发布机械臂准备状态
     * @param is_ready 机械臂是否准备就绪
     */
    void publishRobotReadyState(bool is_ready);

    /**
     * @brief 发布机械臂位姿
     */
    void publishRobotPose();

    /**
     * @brief 等待机械臂停止移动
     */
    void waitForRobotToStop();

    // 回调函数

    /**
     * @brief 启动标定回调函数
     * @param request 请求
     * @param response 响应
     */
    void robotControlServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief 定时器回调函数
     */
    void timerCallback();

    /**
     * @brief 获取标定点数据服务回调函数
     * @param request 请求
     * @param response 响应
     */
    // void getCalibrationPointsServiceCallback(
    //     const std::shared_ptr<custom_msgs_comm::srv::GetCalibrationPoints::Request> request,
    //     std::shared_ptr<custom_msgs_comm::srv::GetCalibrationPoints::Response> response);

    /**
     * @brief 获取机械臂位姿服务回调函数
     * @param request 请求
     * @param response 响应
     */
    void getRobotPoseServiceCallback(
        const std::shared_ptr<custom_msgs_comm::srv::GetRobotPose::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::GetRobotPose::Response> response);

    /**
     * @brief 初始化夹爪控制
     */
    void initGripperControl();

    /**
     * @brief 打开夹爪
     */
    void openGripper();

    /**
     * @brief 闭合夹爪
     */
    void closeGripper();

    /**
     * @brief 启用夹爪
     */
    void enableGripper();

    /**
     * @brief 禁用夹爪
     */
    void disableGripper();

    /**
     * @brief Ping夹爪连接
     */
    void pingGripper();

    /**
     * @brief 调整夹爪位置（相对位置）
     * @param delta_position 位置变化量，正值为增加，负值为减少
     */
    void adjustGripperPosition(int delta_position);

    /**
     * @brief 调整夹爪绝对位置
     * @param delta_position 位置变化量，正值为增加，负值为减少
     */
    void adjustGripperAbsPosition(int delta_position);
    
    /**
     * @brief 打印按键操作说明
     */
    void print_key_operate();
};

}  // namespace nova_robot_ctrl_ros

#endif  // NOVA_ROBOT_CTRL_ROS__NOVA_ROBOT_CTRL_NODE_HPP_