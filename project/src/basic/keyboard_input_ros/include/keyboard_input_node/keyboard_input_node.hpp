#ifndef KEYBOARD_INPUT_NODE__KEYBOARD_INPUT_NODE_HPP_
#define KEYBOARD_INPUT_NODE__KEYBOARD_INPUT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs_comm/srv/keyboard_control.hpp>
#include <memory>
#include <atomic>
#include <termios.h>

namespace keyboard_input_node {

/**
 * @class KeyboardInputNode
 * @brief 独立的ROS2键盘输入节点
 *
 * 该节点负责读取键盘输入，并通过服务调用机械臂控制节点。
 *
 * 功能特性：
 * - 非阻塞键盘输入读取
 * - 支持普通按键和方向键
 * - 通过服务调用机械臂控制节点
 * - 完全独立，不依赖其他机械臂控制代码
 */
class KeyboardInputNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options ROS节点选项
     */
    explicit KeyboardInputNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数 - 自动恢复终端设置
     */
    ~KeyboardInputNode();

    /**
     * @brief 运行键盘输入循环（阻塞）
     */
    void run();

    /**
     * @brief 停止键盘输入循环
     */
    void stop();

private:
    /**
     * @brief 初始化终端为非阻塞模式
     */
    void initTerminal();

    /**
     * @brief 恢复终端原始设置
     */
    void restoreTerminal();

    /**
     * @brief 读取单个按键（非阻塞）
     * @return 按键ASCII码，无输入时返回-1
     */
    int readKey();

    /**
     * @brief 处理按键并调用服务
     * @param key 按键值
     */
    void processKey(int key);

    /**
     * @brief 调用键盘控制服务
     * @param key_name 按键名称
     */
    void callKeyboardControlService(const std::string& key_name);

    /**
     * @brief 显示使用帮助信息
     */
    void showHelp();

    // ROS2 服务客户端
    rclcpp::Client<custom_msgs_comm::srv::KeyboardControl>::SharedPtr keyboard_control_client_;

    // 终端设置
    struct termios original_termios_;
    bool terminal_settings_saved_;

    // 运行控制
    std::atomic<bool> running_;
    std::atomic<bool> stopped_;
    std::atomic<bool> service_in_progress_;  // 标记服务是否正在处理中

    // 统计信息
    uint64_t key_press_count_;
};

}  // namespace keyboard_input_node

#endif  // KEYBOARD_INPUT_NODE__KEYBOARD_INPUT_NODE_HPP_
