#ifndef HAND_EYE_CALIB_ROS__CALIB_TEST_NODE_HPP_
#define HAND_EYE_CALIB_ROS__CALIB_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <termios.h>

// 前向声明键盘处理器类
namespace handeyecalib_ros {
class KeyboardHandler;
}

namespace handeyecalib_ros {

/**
 * @class CalibTestNode
 * @brief ROS节点类，用于测试手眼标定功能
 * 
 * 该类实现了测试功能，包括Aruco检测结果处理和标定精度验证。
 */
class CalibTestNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param options ROS节点选项
     */
    explicit CalibTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~CalibTestNode();
    
    /**
     * @brief 运行键盘输入循环（在主线程中调用）
     */
    void runKeyboardLoop();

private:
    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;      ///< 机械臂位姿订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_calib_result_sub_;  ///< Aruco标定结果订阅器

    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_target_pub_;       ///< 机械臂目标位姿发布器

    // 状态变量
    bool robot_connected_;                                                               ///< 机械臂是否已连接
    bool robot_enabled_;                                                                 ///< 机械臂是否已使能
    bool aruco_detection_received_;                                                      ///< 是否收到Aruco检测结果
    bool running_;                                                                      ///< 是否正在运行
    bool waiting_for_user_input_;                                                       ///< 是否正在等待用户输入
    bool aruco_target_pose_recorded_;                                                   ///< 是否已记录Aruco目标位置

    // 互斥锁
    mutable std::mutex robot_pose_mutex_;                                               ///< 机械臂位姿互斥锁
    mutable std::mutex aruco_detection_mutex_;                                          ///< Aruco检测结果互斥锁

    // 数据变量
    geometry_msgs::msg::PoseStamped current_robot_pose_;                                ///< 当前机械臂位姿
    geometry_msgs::msg::Pose aruco_detection_pose_;                                     ///< Aruco检测位置
    geometry_msgs::msg::PoseStamped before_move_pose_;                                 ///< 移动前机械臂位姿
    geometry_msgs::msg::PoseStamped after_move_pose_;                                  ///< 移动后机械臂位姿

    // 参数变量
    std::string robot_pose_topic_;        ///< 机械臂位姿话题名称
    std::string calib_result_topic_;   ///< 标定结果话题名称
    std::string robot_target_topic_;      ///< 机械臂目标话题名称
    int timer_period_ms_;                 ///< 定时器周期（毫秒）

    // 键盘处理器
    std::unique_ptr<KeyboardHandler> keyboard_handler_;

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
     * @brief 初始化键盘处理器
     */
    void initKeyboardHandler();

    /**
     * @brief 机械臂位姿回调函数
     * @param msg 机械臂位姿消息
     */
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Aruco标定结果回调函数
     * @param msg Aruco标定结果消息
     */
    void arucoCalibResultCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief 读取键盘输入
     * @return 按键字符，如果没有按键则返回-1
     */
    int readKeyboardInput();

    /**
     * @brief 处理键盘输入
     */
    void processKeyboardInput();

    /**
     * @brief 移动到Aruco检测位置
     */
    void moveToArucoPosition();
    
    /**
     * @brief 输出标定精度偏差
     */
    void printCalibrationDeviation();
};
}  // namespace handeyecalib_ros

#endif  // HAND_EYE_CALIB_ROS__CALIB_TEST_NODE_HPP_