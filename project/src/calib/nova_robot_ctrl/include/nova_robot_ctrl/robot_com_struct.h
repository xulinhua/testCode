#ifndef ROBOT_COM_STRUCT_H
#define ROBOT_COM_STRUCT_H

#include <string>
#include <vector>
#include <map>

namespace nova_robot_ctrl {

/**
 * @brief 机器人控制范围参数结构体
 * 包含XYZ三个轴的最小值和最大值
 */
struct RobotCtrlRange {
    double x_min;  ///< X轴最小值
    double x_max;  ///< X轴最大值
    double y_min;  ///< Y轴最小值
    double y_max;  ///< Y轴最大值
    double z_min;  ///< Z轴最小值
    double z_max;  ///< Z轴最大值
};

/**
 * @brief 机器人管理系统配置结构体
 */
struct RobotConfig {
    std::string ip;                             ///< 机器人IP地址
    std::string user_name;                       ///< 机器人用户名
    std::map<int, RobotCtrlRange> ranges;       ///< 移动范围参数，key为工具坐标系ID，value为RobotCtrlRange结构体
};

/**
 * @brief 舵机控制范围参数结构体
 * 包含位置、速度和力度的最小值和最大值
 */
struct ServoCtrlRange {
    int pos_min;    ///< 位置最小值
    int pos_max;    ///< 位置最大值
    int speed_min;  ///< 速度最小值
    int speed_max;  ///< 速度最大值
    int force_min;  ///< 力度最小值
    int force_max;  ///< 力度最大值
};

/**
 * @brief 夹爪配置结构体
 */
struct GripperConfig {
    int id;                                 ///< 夹爪ID
    std::string serial_port;                ///< 串口路径
    std::string user_name;                  ///< 夹爪用户名
    int baudrate;                           ///< 波特率
    bool enable;                            ///< 是否启用夹爪
    ServoCtrlRange servo_ranges;            ///< 舵机参数范围
};

// 夹爪状态信息结构体
struct GripperStatus {
    int position;     ///< 当前位置
    int speed;        ///< 当前速度
    int load;         ///< 负载
    int voltage;      ///< 电压
    int temperature;   ///< 温度
    int moving;       ///< 是否正在移动
    int current;      ///< 电流
};

/**
 * @brief 机器人状态结构体
 * 用于存储机器人的连接和夹爪状态
 */
struct RobotState {
    
    RobotState() : robot_open(false), gripper_open(false) {}
    
    RobotState(bool robot_open_state, bool gripper_open_state) 
        : robot_open(robot_open_state), gripper_open(gripper_open_state) {}

    bool robot_open = false;    ///< 机器人是否已连接
    bool gripper_open = false;  ///< 夹爪是否已连接
};

} // namespace nova_robot_ctrl

#endif // ROBOT_COM_STRUCT_H