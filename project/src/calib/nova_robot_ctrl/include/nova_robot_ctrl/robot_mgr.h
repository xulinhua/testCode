#ifndef ROBOT_MGR_H
#define ROBOT_MGR_H

#include "nova_robot_ctrl/nova_robot_ctrl.h"
#include "nova_robot_ctrl/nova_gripper_ctrl.h"
#include "nova_robot_ctrl/robot_com_struct.h"
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace nova_robot_ctrl {

/**
 * @brief 机器人管理类
 * 提供对机械臂和夹爪的统一管理功能
 */
class RobotMgr {
public:
    
    /**
     * @brief 构造函数
     */
    RobotMgr();
    
    /**
     * @brief 析构函数
     */
    ~RobotMgr();
    
    /**
     * @brief 初始化机器人管理系统
     * @return true-成功，false-失败
     */
    bool init();
    
    /**
     * @brief 释放机器人管理系统资源
     * @return true-成功，false-失败
     */
    bool release();

    /**
     * @brief 初始化机器人参数
     * @return true-成功，false-失败
     */
    bool init_default_param();
    
    /* *********************************配置文件管理************************************ */
    
    /**
     * @brief 创建机器人系统配置文件(系统级配置)
     * @return true-成功，false-失败
     */
    bool create_robot_sys_config();
    
    /**
     * @brief 创建单个机械臂配置文件
     * @param robot_id 机器人ID
     * @return true-成功，false-失败
     */
    bool create_arm_sys_config(int robot_id);
    
    /**
     * @brief 加载机器人系统配置文件(系统级配置)
     * @return true-成功，false-失败
     */
    bool load_robot_sys_config();
    
    /**
     * @brief 加载单个机械臂配置文件
     * @param robot_id 机器人ID
     * @return true-成功，false-失败
     */
    bool load_arm_sys_config(int robot_id);
    
    /**
     * @brief 从参数服务器加载机器人系统配置
     * @return true-成功，false-失败
     */
    bool load_robot_sys_srv();
    
    /**
     * @brief 加载配置文件(整合接口)
     * @param mode 加载模式，0-从文件加载，1-从参数服务器加载
     * @return true-成功，false-失败
     */
    bool load_config(int mode = 0);
    
    
    /* *******************************配置访问接口************************************** */
    
    /**
     * @brief 获取指定ID的机器人配置
     * @param robot_id 机器人ID
     * @param config 输出的机器人配置
     * @return true-成功，false-失败
     */
    bool get_robot_config(int robot_id, RobotConfig& config) const;
    
    /**
     * @brief 设置指定ID的机器人配置
     * @param robot_id 机器人ID
     * @param config 机器人配置
     * @return true-成功，false-失败
     */
    bool set_robot_config(int robot_id, const RobotConfig& config);
    
    /**
     * @brief 获取所有机器人配置
     * @return 机器人配置映射表
     */
    const std::map<int, RobotConfig>& get_all_robot_configs() const;
    
    /**
     * @brief 设置所有机器人配置
     * @param configs 机器人配置映射表
     */
    void set_all_robot_configs(const std::map<int, RobotConfig>& configs);
    
    /**
     * @brief 获取指定ID的机器人启用状态
     * @param robot_id 机器人ID
     * @param enabled 输出的启用状态
     * @return true-成功，false-失败
     */
    bool get_robot_enabled(int robot_id, bool& enabled) const;
    
    /**
     * @brief 设置指定ID的机器人启用状态
     * @param robot_id 机器人ID
     * @param enabled 启用状态
     * @return true-成功，false-失败
     */
    bool set_robot_enabled(int robot_id, bool enabled);
    
    /**
     * @brief 获取所有机器人启用状态
     * @return 机器人启用状态映射表
     */
    const std::map<int, bool>& get_all_robot_enables() const;
    
    /**
     * @brief 设置所有机器人启用状态
     * @param enables 机器人启用状态映射表
     */
    void set_all_robot_enables(const std::map<int, bool>& enables);
    
    
    /* *******************************机械手管理************************************** */
    /**
     * @brief 启用指定ID的机器人
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool enable_robot(int robot_id = 0);

    /**
     * @brief 禁用指定ID的机器人
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool disable_robot(int robot_id = 0);

    /**
     * @brief 控制机器人关节运动
     * @param joint 关节角度向量
     * @param t 运动时间，默认0.1秒
     * @param aheadtime 前瞻时间，默认50
     * @param gain 增益系数，默认500
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool servo_j_robot(const std::vector<double>& joint, float t = 0.1, float aheadtime = 50, float gain = 500, int robot_id = 0);

    /**
     * @brief 控制机器人位置运动
     * @param joint 位置坐标向量
     * @param t 运动时间，默认0.1秒
     * @param aheadtime 前瞻时间，默认50
     * @param gain 增益系数，默认500
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool servo_p_robot(const std::vector<double>& joint, float t = 0.1, float aheadtime = 50, float gain = 500, int robot_id = 0);
   
    /**
     * @brief 获取机器人当前关节角度
     * @param joint 输出的关节角度向量
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool get_angle_robot(std::vector<double>& joint, int robot_id = 0);

    /**
     * @brief 获取机器人当前位姿
     * @param pose 输出的位姿结构体
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool get_current_pose_robot(Pose& pose, int robot_id = 0);

    /**
     * @brief 设置用户坐标系
     * @param user_id 用户坐标系ID
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool set_user_coordinate_robot(int user_id, int robot_id = 0);
    
    /**
     * @brief 设置工具坐标系
     * @param tool_id 工具坐标系ID
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool set_tool_coordinate_robot(int tool_id, int robot_id = 0);
    
    /**
     * @brief 检查指定位置是否有效
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param robot_id 机器人ID，默认为0
     * @return true-位置有效，false-位置无效
     */
    bool is_position_valid_robot(double x, double y, double z, int robot_id = 0);
    
    /**
     * @brief 直线运动到指定位置
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param rx 绕X轴旋转角度，默认180.0
     * @param ry 绕Y轴旋转角度，默认0.0
     * @param rz 绕Z轴旋转角度，默认90.0
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool move_l_robot(double x, double y, double z, double rx = 180.0, double ry = 0.0, double rz = 90.0, int robot_id = 0);

    /**
     * @brief 点动运动
     * @param axis 运动轴 ('x', 'y', 'z', 'rx', 'ry', 'rz')
     * @param distance 运动距离
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool jog_move_robot(char axis, double distance, int robot_id = 0);

    bool sync(int robot_id = 0);

    /**
     * @brief 检查机器人是否已连接
     * @param robot_id 机器人ID，默认为0
     * @return true-已连接，false-未连接
     */
    bool is_connected_robot(int robot_id = 0) const;

    /**
     * @brief 获取机器人标准位姿
     * @param pose 输出的标准位姿结构体
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool get_standard_pose_robot(Pose& pose, int robot_id = 0);
    
    /**
     * @brief 移动到标准位姿
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool move_to_standard_pose_robot(int robot_id = 0);
    
    /**
     * @brief 设置机器人ID
     * @param robot_id 机器人ID
     * @param robot_index 机器人索引，默认为0
     */
    void set_robot_id_robot(int robot_id, int robot_index = 0);
    
    /**
     * @brief 获取机器人推送信息
     * @param robot_id 机器人ID，默认为0
     * @return pushed_info 推送信息结构体
     */
    nova_robot_ctrl::NovaRobotCtrl::pushed_info get_pushed_info_robot(int robot_id = 0);
    
    /**
     * @brief 直线运动到指定位置（带速度参数）
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param rx 绕X轴旋转角度
     * @param ry 绕Y轴旋转角度
     * @param rz 绕Z轴旋转角度
     * @param speed 运动速度
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool move_l_robot(double x, double y, double z, double rx, double ry, double rz, int speed, int robot_id = 0);
    
    /**
     * @brief 获取机器人标定位置管理器
     * @param robot_id 机器人ID，默认为0
     * @return CalibRobotPosMgr* 标定位置管理器指针
     */
    // nova_robot_ctrl::CalibRobotPosMgr* get_calib_robot_pos_mgr_robot(int robot_id = 0);
    
    /**
     * @brief 启用夹爪控制器
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool enable_gripper(int robot_id = 0);

    /**
     * @brief 禁用夹爪控制器
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool disable_gripper(int robot_id = 0);

    /**
     * @brief 打开夹爪
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool gripper_open(int robot_id = 0);
    
    /**
     * @brief 关闭夹爪
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool gripper_close(int robot_id = 0);
    
    /**
     * @brief 检查夹爪是否打开
     * @param robot_id 机器人ID，默认为0
     * @return true-夹爪打开，false-夹爪关闭
     */
    bool is_open_gripper(int robot_id = 0) const;
    
    /**
     * @brief 检查夹爪是否关闭
     * @param robot_id 机器人ID，默认为0
     * @return true-夹爪关闭，false-夹爪打开
     */
    bool is_closed_gripper(int robot_id = 0) const;
    
    /**
     * @brief 获取夹爪当前位置
     * @param robot_id 机器人ID，默认为0
     * @return int 夹爪当前位置，-1表示失败
     */
    int get_current_position_gripper(int robot_id = 0) const;
    
    /**
     * @brief 获取夹爪当前速度
     * @param robot_id 机器人ID，默认为0
     * @return int 夹爪当前速度，-1表示失败
     */
    int get_current_speed_gripper(int robot_id = 0) const;
    
    /**
     * @brief 控制夹爪运动到指定位置
     * @param position 目标位置
     * @param speed 运动速度
     * @param force 夹持力
     * @param robot_id 机器人ID，默认为0
     * @return pair<bool, int> 第一个值表示是否成功，第二个值为错误码
     */
    std::pair<bool, int> move_gripper(int position, int speed, int force, int robot_id = 0);
    
    /**
     * @brief 重置夹爪位置
     * @param speed 运动速度，默认50
     * @param force 夹持力，默认0
     * @param robot_id 机器人ID，默认为0
     * @return pair<bool, int> 第一个值表示是否成功，第二个值为错误码
     */
    std::pair<bool, int> reset_position_gripper(int speed = 50, int force = 0, int robot_id = 0);
    
    /**
     * @brief 通过绝对位置控制夹爪运动
     * @param position 绝对位置
     * @param speed 运动速度，默认1500
     * @param force 夹持力，默认0
     * @param robot_id 机器人ID，默认为0
     * @return pair<bool, int> 第一个值表示是否成功，第二个值为错误码
     */
    std::pair<bool, int> move_by_abs_pos_gripper(int position, int speed = 1500, int force = 0, int robot_id = 0);
    
    /**
     * @brief Ping夹爪连接以检查连接状态
     * @param robot_id 机器人ID，默认为0
     * @return true-Ping成功，false-Ping失败
     */
    bool ping_gripper(int robot_id = 0);
    
    /**
     * @brief 设置夹爪串口路径和ID
     * @param port 串口路径
     * @param id 伺服ID
     * @param robot_id 机器人ID，默认为0
     * @return true-成功，false-失败
     */
    bool set_gripper_port_and_id(const std::string& port, int id, int robot_id = 0);

    /**
     * @brief 获取所有已启用的机器人ID列表
     * @return vector<int> 已启用的机器人ID列表
     */
    std::vector<int> get_all_enable_robots_id() const;

private:
    // 存储多个机器人的配置和实例
    std::map<int, bool> robot_enables_;                         ///< 机器人是否启用(配置参数)
    std::map<int, RobotConfig> robot_configs_;                  ///< 机器人配置
    std::map<int, GripperConfig> gripper_configs_;              ///< 夹爪配置
    std::map<int, NovaRobotCtrl*> robot_ctrls_;                 ///< 机械臂控制对象
    std::map<int, NovaGripperCtrl*> gripper_ctrls_;             ///< 夹爪控制对象
    std::map<int, RobotState> robot_states_;                    ///< 机器人状态信息
    
    // 配置文件路径
    std::string robot_sys_config_path_;                         ///< 机器人系统配置文件路径
    
    /**
     * @brief 根据机器人ID生成机械臂配置文件路径
     * @param robot_id 机器人ID
     * @return std::string 配置文件路径
     */
    std::string get_arm_config_path(int robot_id) const;
    
    // 自动创建配置文件标志
    bool is_create_default_file_;                               ///< 是否自动创建默认配置文件
    
    // 保存USB端口和舵机ID的扫描结果
    std::vector<std::map<std::string, int>> usb_scan_results_;   ///< USB端口扫描结果
    bool usb_scan_performed_;                                   ///< 是否已执行USB端口扫描

};

} // namespace nova_robot_ctrl

#endif // ROBOT_MGR_H