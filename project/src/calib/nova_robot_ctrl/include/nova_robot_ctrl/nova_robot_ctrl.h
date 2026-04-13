#ifndef NOVA_ROBOT_CTRL_H
#define NOVA_ROBOT_CTRL_H

#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <array>
#include <vector>
#include <string>
// #include "nova_robot_ctrl/calib_robot_pos_mgr.h"

namespace nova_robot_ctrl
{

/**
 * @brief 位姿结构体
 * 包含机械手的位置和姿态信息
 */
struct Pose
{
    double x;  ///< X坐标
    double y;  ///< Y坐标
    double z;  ///< Z坐标
    double rx; ///< Rx旋转角
    double ry; ///< Ry旋转角
    double rz; ///< Rz旋转角
};

/**
 * @brief Nova机械臂控制类
 * 提供对Nova机械臂的基本控制功能，包括连接、关节控制、位姿控制等
 */
class NovaRobotCtrl
{
public:
    /**
     * @brief 推送信息结构体
     * 包含机械臂的实时状态信息
     */
    struct pushed_info
    {
        uint64_t time_stamp;                    ///< 时间戳
        std::vector<double> q_actual;           ///< 实际关节位置
        std::vector<double> q_d_actual;         ///< 实际关节速度
        uint8_t enable_status;                  ///< 使能状态
        uint8_t error_status;                   ///< 错误状态
    };

private:
    // 移动范围参数结构体
    struct MoveRangeParams
    {
        double x_max;  ///< X轴最大值
        double x_min;  ///< X轴最小值
        double y_max;  ///< Y轴最大值
        double y_min;  ///< Y轴最小值
        double z_max;  ///< Z轴最大值
        double z_min;  ///< Z轴最小值
    };
    
    // 标准位置参数结构体
    struct StandardPoseParams
    {
        double x;   ///< X坐标
        double y;   ///< Y坐标
        double z;   ///< Z坐标
        double rx;  ///< Rx旋转角
        double ry;  ///< Ry旋转角
        double rz;  ///< Rz旋转角
    };

public:
    /**
     * @brief 构造函数
     * @param user_coordinate_id 用户坐标系ID，默认为0
     * @param tool_coordinate_id 工具坐标系ID，默认为0
     */
    NovaRobotCtrl(int user_coordinate_id = 0, int tool_coordinate_id = 0);

    /**
     * @brief 析构函数
     */
    ~NovaRobotCtrl();

    /**
     * @brief 打开机械臂连接
     * @param ip 机械臂IP地址
     * @return true-连接成功，false-连接失败
     */
    bool open(const std::string& ip);

    /**
     * @brief 关闭机械臂连接
     */
    void close();

    /**
     * @brief 关节空间伺服控制
     * @param joint 关节角度向量(6个关节)
     * @param t 时间参数(默认0.1)
     * @param aheadtime 前瞻时间(默认50)
     * @param gain 增益参数(默认500)
     * @return true-发送成功，false-发送失败
     */
    bool servo_j(const std::vector<double>& joint, float t = 0.1, float aheadtime = 50, float gain = 500);

    bool servo_p(const std::vector<double>& joint, float t = 0.1, float aheadtime = 50, float gain = 500);
    /**
     * @brief 获取当前关节角度
     * @param joint 输出的关节角度向量
     * @return true-获取成功，false-获取失败
     */
    bool get_angle(std::vector<double>& joint);

    /**
     * @brief 使能机械臂
     * @return true-使能成功，false-使能失败
     */
    bool enable_robot();

    /**
     * @brief 获取推送信息
     * @return 推送信息结构体
     */
    pushed_info get_pushed_info();

    /**
     * @brief 获取当前机械手的实时坐标
     * @param pose 输出的位姿信息
     * @return true-获取成功，false-获取失败
     */
    bool get_current_pose(Pose& pose);

    /**
     * @brief 设置用户坐标系
     * @param user_id 用户坐标系ID
     * @return true-设置成功，false-设置失败
     */
    bool set_user_coordinate(int user_id);
    
    /**
     * @brief 设置工具坐标系
     * @param tool_id 工具坐标系ID
     * @return true-设置成功，false-设置失败
     */
    bool set_tool_coordinate(int tool_id);
    
    /**
     * @brief 检查位置是否在允许范围内
     * @param x X坐标(mm)
     * @param y Y坐标(mm)
     * @param z Z坐标(mm)
     * @return true-位置有效，false-超出范围
     */
    bool is_position_valid(double x, double y, double z);
    
    /**
     * @brief 直线运动到目标位置
     * @param x X坐标(mm)
     * @param y Y坐标(mm)
     * @param z Z坐标(mm)
     * @param rx Rx旋转角(度)，默认为180
     * @param ry Ry旋转角(度)，默认为0
     * @param rz Rz旋转角(度)，默认为90
     * @param speed 运动速度(%)，默认为100
     * @return true-移动成功，false-移动失败
     */
    bool move_l(double x, double y, double z, double rx = 180.0, double ry = 0.0, double rz = 90.0, int speed = 80);

    /**
     * @brief 沿指定轴向移动指定距离
     * @param axis 移动轴向('x', 'y', 'z')
     * @param distance 移动距离(mm)
     * @param speed 运动速度(%)，默认为100
     * @return true-移动成功，false-移动失败
     */
    bool jog_move(char axis, double distance, int speed = 80);

    bool sync();

    /**
     * @brief 检查是否已连接
     * @return true-已连接，false-未连接
     */
    bool is_connected() const;

    /**
     * @brief 获取标准位置
     * @param pose 输出的标准位置信息
     * @return true-获取成功，false-获取失败
     */
    bool get_standard_pose(Pose& pose);
    
    /**
     * @brief 移动到标准位置
     * @return true-移动成功，false-移动失败
     */
    bool move_to_standard_pose();
    
    /**
     * @brief 设置机械手ID
     * @param robot_id 机械手ID
     */
    void set_robot_id(int robot_id);
    
    /**
     * @brief 获取标定点管理器
     * @return 标定点管理器指针
     */
    //CalibRobotPosMgr* get_calib_robot_pos_mgr();

private:
    /**
     * @brief 开始接收数据
     */
    void start_receive();

    /**
     * @brief 接收数据回调函数
     * @param error 错误码
     * @param bytes_transferred 传输字节数
     */
    void on_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

    /**
     * @brief 从字节数组中获取数据
     * @tparam T 数据类型
     * @param array 字节数组
     * @param start 起始位置
     * @param end 结束位置
     * @return 解析出的数据
     */
    template <typename T>
    static T get_data_from_bytes(const std::array<char, 1440>& array, int start, int end);

    /**
     * @brief 从字节数组中获取多个数据
     * @tparam T 数据类型
     * @param array 字节数组
     * @param start 起始位置
     * @param end 结束位置
     * @param datasize 数据个数
     * @return 解析出的数据向量
     */
    template <typename T>
    static std::vector<T> get_datas_from_bytes(const std::array<char, 1440>& array, int start, int end, int datasize);

    /**
     * @brief 根据工具坐标系ID设置对应的移动范围参数
     * @param tool_id 工具坐标系ID
     */
    void set_parameters_for_tool_coordinate(int tool_id);

private:
    std::shared_ptr<boost::asio::io_context> io_context_;                  ///< IO上下文
    std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;  ///< 工作守护
    std::thread worker_thread_;                                     ///< 工作线程
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_29999_;           ///< 29999端口socket
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_30003_;           ///< 30003端口socket
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_30004_;           ///< 30004端口socket
    std::array<char, 1440> receive_buffer_;                         ///< 接收缓冲区
    bool is_running_;                                               ///< 运行状态标志
    pushed_info pushed_info_;                                        ///< 推送信息
    
    // 坐标系相关
    int user_coordinate_id_;                                         ///< 用户坐标系ID
    int tool_coordinate_id_;                                         ///< 工具坐标系ID
    
    // 移动范围参数
    MoveRangeParams move_range_params_;                              ///< 移动范围参数
    
    // 标准位置参数
    StandardPoseParams standard_pose_params_;                        ///< 标准位置参数
    
    // 标定点管理器
    // std::unique_ptr<CalibRobotPosMgr> calib_robot_pos_mgr_;          ///< 标定点管理器
    
    // 机械手ID
    int robot_id_;                                                   ///< 机械手ID
    
    // 移除ROS日志记录器
    // rclcpp::Logger logger_;
};

} // namespace nova_robot_ctrl

#endif // NOVA_ROBOT_CTRL_H