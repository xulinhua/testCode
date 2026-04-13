# 键盘输入节点 (Keyboard Input Node)

独立的ROS2键盘输入节点，用于读取键盘输入并将按键信息发布到ROS话题。

## 功能特性

- 非阻塞键盘输入读取
- 支持普通按键（A-Z, 0-9）
- 支持方向键（上下左右）
- 支持特殊键（SPACE, ENTER, TAB, ESC, BACKSPACE）
- 将按键信息发布到 `/keyboard_input` 话题
- 完全独立，不依赖其他机械臂控制代码

## 编译

```bash
cd /home/hoson/nova_ctrl
colcon build --packages-select keyboard_input_node
```

## 运行

### 方式1: 使用launch文件

```bash
source install/setup.bash
ros2 launch keyboard_input_node keyboard_input.launch.py
```

### 方式2: 直接运行可执行文件

```bash
source install/setup.bash
ros2 run keyboard_input_node keyboard_input_node
```

## 订阅按键事件

在另一个终端中订阅 `/keyboard_input` 话题：

```bash
source install/setup.bash
ros2 topic echo /keyboard_input
```

## 支持的按键

| 按键 | 发布的消息 |
|------|-----------|
| 方向键 ↑ | `UP` |
| 方向键 ↓ | `DOWN` |
| 方向键 ← | `LEFT` |
| 方向键 → | `RIGHT` |
| A-Z | `A` - `Z` (自动转大写) |
| 0-9 | `0` - `9` |
| 空格 | `SPACE` |
| 回车 | `ENTER` |
| TAB | `TAB` |
| ESC | `ESC` |
| 退格 | `BACKSPACE` |

## 示例代码

### 在其他节点中订阅按键

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MyControlNode : public rclcpp::Node
{
public:
    MyControlNode() : Node("my_control_node")
    {
        // 订阅键盘输入话题
        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_input",
            10,
            std::bind(&MyControlNode::keyCallback, this, std::placeholders::_1)
        );
    }

private:
    void keyCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 根据按键执行相应逻辑
        if (msg->data == "UP") {
            RCLCPP_INFO(this->get_logger(), "向上移动");
            // 执行向上移动逻辑
        }
        else if (msg->data == "DOWN") {
            RCLCPP_INFO(this->get_logger(), "向下移动");
            // 执行向下移动逻辑
        }
        else if (msg->data == "LEFT") {
            RCLCPP_INFO(this->get_logger(), "向左移动");
            // 执行向左移动逻辑
        }
        else if (msg->data == "RIGHT") {
            RCLCPP_INFO(this->get_logger(), "向右移动");
            // 执行向右移动逻辑
        }
        // 更多按键处理...
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `key_event_topic` | `/keyboard_input` | 按键事件发布的话题名称 |

## 退出

按 `Ctrl+C` 退出节点。

## 注意事项

1. 该节点需要直接在终端中运行，不能在后台运行（需要读取键盘输入）
2. 运行时会修改终端设置为非阻塞模式，退出时会自动恢复
3. 如果异常退出导致终端显示异常，可以使用 `reset` 命令恢复

## 依赖

- ROS2 (Humble/Foxy/Galactic)
- rclcpp
- std_msgs
