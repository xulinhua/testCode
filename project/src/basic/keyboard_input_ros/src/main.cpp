#include "keyboard_input_node/keyboard_input_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <memory>

namespace keyboard_input_node {

// 全局节点指针，用于信号处理
std::shared_ptr<KeyboardInputNode> g_node = nullptr;
std::atomic<bool> g_shutdown_requested{false};

/**
 * @brief 信号处理函数
 * @param signal 信号编号
 */
void signalHandler(int signal)
{
    if (g_node && !g_shutdown_requested.exchange(true)) {
        RCLCPP_INFO(g_node->get_logger(), "收到信号 %d，正在关闭节点...", signal);
        g_node->stop();
    }
}

}  // namespace keyboard_input_node

/**
 * @brief 主函数
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出码
 */
int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 设置信号处理
    signal(SIGINT, keyboard_input_node::signalHandler);
    signal(SIGTERM, keyboard_input_node::signalHandler);

    try {
        // 创建键盘输入节点
        keyboard_input_node::g_node = std::make_shared<keyboard_input_node::KeyboardInputNode>();

        // 运行节点
        keyboard_input_node::g_node->run();

        // 清理节点（在shutdown之前）
        keyboard_input_node::g_node.reset();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // 关闭ROS2
    rclcpp::shutdown();

    return 0;
}
