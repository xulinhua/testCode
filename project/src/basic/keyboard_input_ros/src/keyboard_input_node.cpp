#include "keyboard_input_node/keyboard_input_node.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <iomanip>

namespace keyboard_input_node {

KeyboardInputNode::KeyboardInputNode(const rclcpp::NodeOptions & options)
: Node("keyboard_input_node", options),
  terminal_settings_saved_(false),
  running_(false),
  stopped_(false),
  service_in_progress_(false),
  key_press_count_(0)
{
    // 创建键盘控制服务客户端
    keyboard_control_client_ = this->create_client<custom_msgs_comm::srv::KeyboardControl>("/keyboard_control");

    // 初始化终端为非阻塞模式
    initTerminal();

    // 显示欢迎信息
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  ROS2 键盘输入节点已启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "服务: /keyboard_control");
    RCLCPP_INFO(this->get_logger(), "");

    // 显示帮助信息
    showHelp();

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "节点已就绪，等待键盘输入...");
    RCLCPP_INFO(this->get_logger(), "按 Ctrl+C 退出");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

KeyboardInputNode::~KeyboardInputNode()
{
    stop();
    restoreTerminal();
    RCLCPP_INFO(this->get_logger(), "键盘输入节点已关闭，共处理 %lu 次按键",
                static_cast<unsigned long>(key_press_count_));
}

void KeyboardInputNode::initTerminal()
{
    // 保存原始终端设置
    if (tcgetattr(STDIN_FILENO, &original_termios_) == 0) {
        terminal_settings_saved_ = true;

        struct termios new_termios = original_termios_;

        // 设置为非阻塞模式
        new_termios.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
        new_termios.c_cc[VMIN] = 0;               // 非阻塞读取
        new_termios.c_cc[VTIME] = 0;              // 无超时

        // 应用新设置
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        // 设置文件描述符为非阻塞
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

void KeyboardInputNode::restoreTerminal()
{
    if (terminal_settings_saved_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        tcflush(STDIN_FILENO, TCIFLUSH);
        terminal_settings_saved_ = false;
    }
}

int KeyboardInputNode::readKey()
{
    fd_set set;
    struct timeval timeout;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    // 10ms 超时
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;

    int ret = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);

    if (ret > 0) {
        unsigned char c;
        ssize_t bytes_read = read(STDIN_FILENO, &c, 1);
        if (bytes_read > 0) {
            return static_cast<int>(c);
        }
    }

    return -1;  // 无输入
}

void KeyboardInputNode::processKey(int key)
{
    std::string key_name;

    // 检查是否是转义序列（方向键等特殊键）
    if (key == 27) {  // ESC
        int key2 = readKey();
        if (key2 == 91) {
            int key3 = readKey();
            if (key3 >= 65 && key3 <= 68) {
                // 方向键序列: ESC [ A/B/C/D
                switch (key3) {
                    case 65: key_name = "UP";         break;  // 上箭头
                    case 66: key_name = "DOWN";       break;  // 下箭头
                    case 67: key_name = "RIGHT";      break;  // 右箭头
                    case 68: key_name = "LEFT";       break;  // 左箭头
                }
            }
        }

        // 如果不是方向键，可能是单独的ESC键
        if (key_name.empty() && key2 == -1) {
            key_name = "ESC";
        }
    }
    // 处理控制字符
    else if (key >= 0 && key <= 31) {
        switch (key) {
            case 3:  key_name = "Ctrl+C";     break;
            case 4:  key_name = "Ctrl+D";     break;
            case 9:  key_name = "TAB";        break;
            case 10: key_name = "ENTER";      break;
            case 13: key_name = "RETURN";     break;
            case 127:key_name = "BACKSPACE";  break;
            default:
                key_name = "Ctrl+" + std::string(1, static_cast<char>(key + 64));
                break;
        }
    }
    // 处理普通可打印字符
    else if (key >= 32) {
        char c = static_cast<char>(key);
        if (c >= 'a' && c <= 'z') {
            key_name = std::string(1, std::toupper(c));
        } else if (c >= 'A' && c <= 'Z') {
            key_name = std::string(1, c);
        } else if (c >= '0' && c <= '9') {
            key_name = std::string(1, c);
        } else {
            switch (c) {
                case ' ': key_name = "SPACE";  break;
                case '+': key_name = "PLUS";   break;
                case '-': key_name = "MINUS";  break;
                case '=': key_name = "EQUAL";  break;
                default:  key_name = std::string(1, c);  break;
            }
        }
    }

    // 调用键盘控制服务
    if (!key_name.empty()) {
        // 如果服务正在处理中，忽略新按键
        if (service_in_progress_) {
            RCLCPP_DEBUG(this->get_logger(), "服务处理中，忽略按键: %s", key_name.c_str());
            return;
        }
        callKeyboardControlService(key_name);
    }
}

void KeyboardInputNode::callKeyboardControlService(const std::string& key_name)
{
    // 标记服务正在处理中
    service_in_progress_ = true;

    // 检查服务是否可用
    if (!keyboard_control_client_->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(this->get_logger(), "键盘控制服务不可用，等待服务启动...");
        service_in_progress_ = false;
        return;
    }

    // 创建服务请求
    auto request = std::make_shared<custom_msgs_comm::srv::KeyboardControl::Request>();
    request->key_name = key_name;

    // 同步发送服务请求并等待响应
    auto future = keyboard_control_client_->async_send_request(request);

    // 等待服务响应，使用较长的超时时间以等待机械臂动作完成
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[按键: %s] %s", key_name.c_str(), response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "[按键: %s] 执行失败: %s", key_name.c_str(), response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "[按键: %s] 服务响应超时", key_name.c_str());
    }

    key_press_count_++;

    // 清空输入缓冲区，丢弃服务处理期间用户按下的所有按键
    tcflush(STDIN_FILENO, TCIFLUSH);

    // 标记服务处理完成
    service_in_progress_ = false;
}

void KeyboardInputNode::showHelp()
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  支持的按键输入");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "方向键:");
    RCLCPP_INFO(this->get_logger(), "  ↑  UP      - 上方向键");
    RCLCPP_INFO(this->get_logger(), "  ↓  DOWN    - 下方向键");
    RCLCPP_INFO(this->get_logger(), "  ←  LEFT    - 左方向键");
    RCLCPP_INFO(this->get_logger(), "  →  RIGHT   - 右方向键");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "字母键:");
    RCLCPP_INFO(this->get_logger(), "  A-Z       - 字母键（自动转大写）");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "数字键:");
    RCLCPP_INFO(this->get_logger(), "  0-9       - 数字键");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "特殊键:");
    RCLCPP_INFO(this->get_logger(), "  SPACE     - 空格键");
    RCLCPP_INFO(this->get_logger(), "  ENTER     - 回车键");
    RCLCPP_INFO(this->get_logger(), "  TAB       - 制表键");
    RCLCPP_INFO(this->get_logger(), "  ESC       - ESC键");
    RCLCPP_INFO(this->get_logger(), "  BACKSPACE - 退格键");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "说明:");
    RCLCPP_INFO(this->get_logger(), "  - 按键会调用 /keyboard_control 服务");
    RCLCPP_INFO(this->get_logger(), "  - 服务类型: custom_msgs_comm/srv/KeyboardControl");
    RCLCPP_INFO(this->get_logger(), "  - 机械臂控制节点响应按键执行相应动作");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "示例服务调用命令:");
    RCLCPP_INFO(this->get_logger(), "  ros2 service call /keyboard_control custom_msgs_comm/srv/KeyboardControl \"{key_name: 'UP'}\"");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void KeyboardInputNode::run()
{
    running_ = true;
    stopped_ = false;

    while (running_ && rclcpp::ok()) {
        // 只有在服务未处理时才读取新按键
        if (!service_in_progress_) {
            // 读取按键
            int key = readKey();

            if (key != -1) {
                // 调试：显示读取到的原始键值
                RCLCPP_DEBUG(this->get_logger(), "读取到键值: %d", key);

                // 处理 Ctrl+C 退出（即使在服务处理中也允许退出）
                if (key == 3) {  // Ctrl+C
                    RCLCPP_INFO(this->get_logger(), "收到退出信号 (Ctrl+C)");
                    running_ = false;
                    break;
                }

                // 处理按键
                processKey(key);
            }
        }

        // 短暂休眠，避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 处理ROS回调（仅在运行状态下）
        if (running_ && rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
        }
    }

    stopped_ = true;
}

void KeyboardInputNode::stop()
{
    running_ = false;
}

}  // namespace keyboard_input_node
