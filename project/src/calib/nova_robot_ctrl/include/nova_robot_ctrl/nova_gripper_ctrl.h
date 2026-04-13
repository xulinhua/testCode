#ifndef NOVA_GRIPPER_CTRL_H
#define NOVA_GRIPPER_CTRL_H

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include "nova_robot_ctrl/robot_com_struct.h"
#include "SCSCL.h"

namespace nova_robot_ctrl
{

/**
 * @brief Nova夹爪控制类
 * 提供对Nova机械臂夹爪的基本控制功能
 */
 
class NovaGripperCtrl {
public:
    // 静态成员变量，用于设置扫描ID的范围
    static int scan_id_min_;
    static int scan_id_max_;

    /**
     * @brief 获取可用的USB端口列表
     * @return vector<string> 包含可用USB端口路径的向量
     */
    static std::vector<std::string> get_available_usb_ports();

    /**
     * @brief 构造函数 - 只初始化参数，不打开连接
     * @param port USB端口路径 (例如: "/dev/ttyUSB0")
     * @param id_name 伺服ID
     * @param servo_pos 包含[min_servo_pos, max_servo_pos]的向量
     */
    NovaGripperCtrl(const std::string& port, int id_name, const std::vector<int>& servo_pos);
    
    /**
     * @brief 析构函数
     */
    ~NovaGripperCtrl();
    
    /**
     * @brief 设置是否禁用日志输出
     * @param disable true-禁用日志输出，false-启用日志输出
     */
    void set_disable_log(bool disable);
    
    /**
     * @brief 获取日志禁用状态
     * @return true-日志被禁用，false-日志已启用
     */
    bool get_disable_log() const;
    
    /**
     * @brief 打开连接并初始化夹爪
     * @return true-成功，false-失败
     */
    bool connect();
    
    /**
     * @brief 检查是否已连接
     * @return true-已连接，false-未连接
     */
    bool is_connected() const;
    
    /**
     * @brief Ping伺服器以检查连接
     * @return true-Ping成功
     */
    bool ping();
    
    /**
     * @brief 设置伺服器扭矩限制
     * @param limit 扭矩限制值
     * @return true-成功
     */
    bool set_torque_limit(int limit);
    
    /**
     * @brief 设置串口路径和伺服ID
     * @param port 串口路径
     * @param id 伺服ID
     * @return true-成功
     */
    bool set_port_and_id(const std::string& port, int id);
    
    /**
     * @brief 断开夹爪连接
     */
    void disconnect();
    
    /**
     * @brief 获取最小位置值
     * @return 最小位置
     */
    int get_min_position() const;
    
    /**
     * @brief 获取最大位置值  
     * @return 最大位置
     */
    int get_max_position() const;
    
    /**
     * @brief 获取打开位置 (与最小位置相同)
     * @return 打开位置
     */
    int get_open_position() const;
    
    /**
     * @brief 获取闭合位置 (与最大位置相同)
     * @return 闭合位置
     */
    int get_closed_position() const;
    
    /**
     * @brief 检查夹爪是否处于打开位置
     * @return true-打开
     */
    bool is_open() const;
    
    /**
     * @brief 检查夹爪是否处于闭合位置
     * @return true-闭合
     */
    bool is_closed() const;
    
    /**
     * @brief 从硬件获取当前位置
     * @return 当前位置
     */
    int get_current_position() const;
    
    /**
     * @brief 获取当前速度
     * @return 当前速度
     */
    int get_current_speed() const;
    
    /**
     * @brief 移动夹爪到指定位置
     * @param position 目标位置 (0-255)
     * @param speed 移动速度 (0-100)
     * @param force 夹持力 (0-100) - 硬件未实现
     * @return pair<success, error_code>
     */
    std::pair<bool, int> move(int position, int speed, int force);
    
    /**
     * @brief 获取夹爪的完整状态信息
     * @return GripperStatus结构体，包含位置、速度、负载、电压、温度、移动状态和电流信息
     */
    GripperStatus get_status() const;
    
    /**
     * @brief 复位夹爪位置到指定位置(4000)
     * @param speed 复位速度 (0-100)
     * @param force 夹持力 (0-100) - 硬件未实现
     * @return pair<success, error_code>
     */
    std::pair<bool, int> reset_position(int speed = 50, int force = 0);
    
    /**
     * @brief 通过绝对位置移动夹爪
     * @param position 绝对位置值
     * @param speed 移动速度 (0-100)
     * @param force 夹持力 (0-100) - 硬件未实现
     * @return pair<success, error_code>
     */
    std::pair<bool, int> move_by_abs_pos(int position, int speed = 1500, int force = 0);
    
    /**
     * @brief 遍历当前开发板上的所有USB串口和串口下对应的舵机
     * @return vector<map<port_name, servo_id>> 找到的端口和对应的舵机ID列表
     */
    static std::vector<std::map<std::string, int>> scan_usb_ports_and_servos();
    
    /**
     * @brief 获取指定ID范围内的连接舵机ID列表
     * @param scan_min 扫描ID的最小值
     * @param scan_max 扫描ID的最大值
     * @return vector<int> 连接的舵机ID列表
     */
    std::vector<int> get_connect_id(int scan_min, int scan_max);

private:
    /**
     * @brief 设置延迟定时器
     */
    void set_latency_timer();
    
    std::string port_;              ///< 端口路径
    int servo_id_;                  ///< 伺服ID
    bool connected_;                ///< 连接状态标志
    int baudrate_;                  ///< 串口波特率
    
    // 外部范围 (API)
    int min_position_;              ///< 最小位置
    int max_position_;              ///< 最大位置
    int min_speed_;                 ///< 最小速度
    int max_speed_;                 ///< 最大速度
    int min_force_;                 ///< 最小力
    int max_force_;                 ///< 最大力
    
    // 内部伺服范围
    int min_servo_pos_;             ///< 最小伺服位置
    int max_servo_pos_;             ///< 最大伺服位置
    int min_servo_speed_;           ///< 最小伺服速度
    int max_servo_speed_;           ///< 最大伺服速度
    int min_servo_force_;           ///< 最小伺服力
    int max_servo_force_;           ///< 最大伺服力
    
    SCSCL* scservo_;                ///< SCServo控制对象
    
    // 线程安全
    mutable std::mutex command_lock_;  ///< 命令锁
    
    bool disable_log_;//测试时关闭日志
};

} // namespace nova_robot_ctrl

#endif // NOVA_GRIPPER_CTRL_H