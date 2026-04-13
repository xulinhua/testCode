#include "nova_robot_ctrl/nova_robot_ctrl.h"
#include "nova_robot_ctrl/nova_gripper_ctrl.h"
#include "nova_robot_ctrl/robot_mgr.h" // 添加RobotMgr头文件
// 移除ROS相关头文件
// #include <rclcpp/rclcpp.hpp>
// 添加log_system头文件
#include <iostream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <sys/ioctl.h>
#include <linux/kd.h>

// 全局变量用于存储原始终端设置
static struct termios original_settings;
static bool terminal_settings_saved = false;

// 信号处理函数，确保终端设置被正确恢复
void signal_handler(int signal)
{
    if (terminal_settings_saved)
    {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &original_settings);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }
    // 退出程序
    exit(signal);
}

#define TEST_GRIPPER 1
// 获取终端属性以支持非阻塞键盘输入
class KeyboardReader
{
public:
    KeyboardReader()
    {
        tcgetattr(0, &initial_settings_);
        new_settings_ = initial_settings_;
        new_settings_.c_lflag &= ~ICANON;
        new_settings_.c_lflag &= ~ECHO;
        new_settings_.c_cc[VMIN] = 1;
        new_settings_.c_cc[VTIME] = 0;
        tcsetattr(0, TCSANOW, &new_settings_);
        fcntl(0, F_SETFL, O_NONBLOCK);
    }

    ~KeyboardReader()
    {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &initial_settings_);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }

    int readOne()
    {
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(0, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        int res = select(1, &set, NULL, NULL, &timeout);
        if (res > 0)
        {
            char c;
            if (read(0, &c, 1) > 0)
                return c;
        }
        return -1;
    }

private:
    struct termios initial_settings_;
    struct termios new_settings_;
};

// 新的main函数，使用RobotMgr类简化测试代码
int main(int argc, char **argv)
{
    // 注册信号处理函数，处理Ctrl+C等信号
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 保存原始终端设置
    tcgetattr(0, &original_settings);
    terminal_settings_saved = true;

    // 创建机器人管理对象
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 开始使用RobotMgr进行机器人移动测试..." << std::endl;
    nova_robot_ctrl::RobotMgr robot_mgr;

    // 加载配置文件
    if (!robot_mgr.load_config())
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 加载配置文件失败" << std::endl;
        return -1;
    }
    else
    {
        // 加载成功后设置夹爪串口和ID
        // robot_mgr.set_gripper_port_and_id(gripper_config_.serial_port, gripper_config_.id);
    }
    // 启用机器人系统
    if (!robot_mgr.init())
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 启用机器人系统失败" << std::endl;
        return -1;
    }

    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统已成功启用" << std::endl;

    // 添加当前机械臂ID变量
    int current_robot_id = 0;  // 默认使用机器人ID 0
    
    // 获取当前机械手位姿
    nova_robot_ctrl::Pose current_pose;
    if (robot_mgr.get_current_pose_robot(current_pose, current_robot_id))
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前位置: 位置(" << std::fixed << std::setprecision(3) << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << "), 姿态(" << current_pose.rx << ", " << current_pose.ry << ", " << current_pose.rz << ")" << std::endl;
    }
    else
    {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取当前位置失败" << std::endl;
    }

    // 创建键盘读取器
    KeyboardReader key_reader;

    // 初始化当前位置为初始坐标
    std::vector<double> target_joints = {current_pose.x, current_pose.y, current_pose.z,
                                         current_pose.rx, current_pose.ry, current_pose.rz};
    
    // 夹爪控制变量
    int gripper_position = robot_mgr.get_current_position_gripper();        // 初始位置
    int gripper_speed = 50;          // 初始速度
    int gripper_force = 0;           // 初始力度
    int gripper_abs_position = 2000; // 绝对位置变量，初始值为2000
        
    // 移动步长
    double step_size_normal = 1;                 // 正常点动步长 1mm
    double step_size_large = 10;                 // 大点动步长 10mm
    double current_step_size = step_size_normal; // 当前使用的步长
    bool ctrl_pressed = false;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 使用RobotMgr开始机器人移动测试。控制说明:" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   方向键: 在X/Y平面移动" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   I/K键: 在Z方向移动" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   G: 打开夹爪" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   H: 关闭夹爪" << std::endl;
    // LOG_INFO("  J: 启用夹爪");
    // LOG_INFO("  K: 禁用夹爪");
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   U/O: 调整夹爪位置 (U-减小, O-增大)" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   V/N: 调整夹爪绝对位置 (V-减小, N-增大)" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   P: 获取当前位置" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   Q: 退出" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   T: 切换到下一个机器人ID" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   Current step size - Normal: " << step_size_normal << " mm, Large: " << step_size_large << " mm" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   Hold Ctrl key for large step movement" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   Current gripper position: " << gripper_position << ", speed: " << gripper_speed << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   Current gripper absolute position: " << gripper_abs_position << std::endl;

    bool running = true;
    bool joints_changed = false; // 标记关节角度是否发生变化
    while (running)
    {
        // 读取键盘输入
        int key = key_reader.readOne();
        if (key != -1)
        {
            // 检查是否是ESC序列开始（方向键编码为 27 91 对应的数字）
            if (key == 27)
            {
                // 可能是ESC序列开始，尝试读取后续字符
                int next_key = key_reader.readOne();
                if (next_key == 91)
                { // '[' 字符
                    int arrow_key = key_reader.readOne();

                    // 检查是否是Shift+方向键 (1;2表示Shift修饰)
                    if (arrow_key == 49)
                    { // '1' 字符
                        int semicolon_key = key_reader.readOne();
                        if (semicolon_key == 59)
                        { // ';' 字符
                            int modifier_key = key_reader.readOne();
                            if (modifier_key == 50)
                            { // '2' 字符 (Shift)
                                int actual_arrow_key = key_reader.readOne();
                                // 设置为大步长模式
                                current_step_size = step_size_large;
                                ctrl_pressed = true; // 使用ctrl_pressed变量表示大步长模式

                                joints_changed = false; // 重置标记
                                switch (actual_arrow_key)
                                {
                                case 65: // 上箭头
                                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向上移动(Y+)大步长(Shift+上)机器人 " << current_robot_id << std::endl;
                                    target_joints[1] += current_step_size;
                                    joints_changed = true;
                                    break;
                                case 66: // 下箭头
                                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向下移动(Y-)大步长(Shift+下)机器人 " << current_robot_id << std::endl;
                                    target_joints[1] -= current_step_size;
                                    joints_changed = true;
                                    break;
                                case 67: // 右箭头
                                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向右移动(X+)大步长(Shift+右)机器人 " << current_robot_id << std::endl;
                                    target_joints[0] += current_step_size;
                                    joints_changed = true;
                                    break;
                                case 68: // 左箭头
                                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向左移动(X-)大步长(Shift+左)机器人 " << current_robot_id << std::endl;
                                    target_joints[0] -= current_step_size;
                                    joints_changed = true;
                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        // 普通方向键
                        current_step_size = step_size_normal;
                        ctrl_pressed = false;

                        joints_changed = false; // 重置标记
                        switch (arrow_key)
                        {
                        case 65: // 上箭头
                            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向上移动(Y+)机器人 " << current_robot_id << std::endl;
                            target_joints[1] += current_step_size;
                            joints_changed = true;
                            break;
                        case 66: // 下箭头
                            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向下移动(Y-)机器人 " << current_robot_id << std::endl;
                            target_joints[1] -= current_step_size;
                            joints_changed = true;
                            break;
                        case 67: // 右箭头
                            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向右移动(X+)机器人 " << current_robot_id << std::endl;
                            target_joints[0] += current_step_size;
                            joints_changed = true;
                            break;
                        case 68: // 左箭头
                            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向左移动(X-)机器人 " << current_robot_id << std::endl;
                            target_joints[0] -= current_step_size;
                            joints_changed = true;
                            break;
                        }
                    }
                }
                else
                {
                    // 不是方向键序列，恢复为正常步长
                    current_step_size = step_size_normal;
                    ctrl_pressed = false;
                }
            }
            else
            {
                // 恢复为正常步长
                current_step_size = step_size_normal;
                ctrl_pressed = false;

                joints_changed = false; // 重置标记
                switch (key)
                {
                case 65: // 上箭头
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向上移动(Y+)机器人 " << current_robot_id << std::endl;
                    target_joints[1] += current_step_size;
                    joints_changed = true;
                    break;
                case 66: // 下箭头
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向下移动(Y-)机器人 " << current_robot_id << std::endl;
                    target_joints[1] -= current_step_size;
                    joints_changed = true;
                    break;
                case 67: // 右箭头
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向右移动(X+)机器人 " << current_robot_id << std::endl;
                    target_joints[0] += current_step_size;
                    joints_changed = true;
                    break;
                case 68: // 左箭头
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向左移动(X-)机器人 " << current_robot_id << std::endl;
                    target_joints[0] -= current_step_size;
                    joints_changed = true;
                    break;
                case 'I':  // 大写I，可能是Shift+I
                    // 设置为大步长模式
                    current_step_size = step_size_large;
                    ctrl_pressed = true;
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向上移动(Z+)大步长(Shift+I)机器人 " << current_robot_id << std::endl;
                    target_joints[2] += current_step_size;
                    joints_changed = true;
                    break;
                case 'i':  // 小写i
                    current_step_size = step_size_normal;
                    ctrl_pressed = false;
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向上移动(Z+)机器人 " << current_robot_id << std::endl;
                    target_joints[2] += current_step_size;
                    joints_changed = true;
                    break;
                case 'K':  // 大写K，可能是Shift+K
                    // 设置为大步长模式
                    current_step_size = step_size_large;
                    ctrl_pressed = true;
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向下移动(Z-)大步长(Shift+K)机器人 " << current_robot_id << std::endl;
                    target_joints[2] -= current_step_size;
                    joints_changed = true;
                    break;
                case 'k':  // 小写k
                    current_step_size = step_size_normal;
                    ctrl_pressed = false;
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 向下移动(Z-)机器人 " << current_robot_id << std::endl;
                    target_joints[2] -= current_step_size;
                    joints_changed = true;
                    break;
                
                case 'g':
                case 'G':
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在打开夹爪" << std::endl;
                    if (!robot_mgr.gripper_open(current_robot_id))
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 打开夹爪失败" << std::endl;
                    }
                    else
                    {
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪已成功打开" << std::endl;
                    }
                    break;
                case 'h':
                case 'H':
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在关闭夹爪" << std::endl;
                    if (!robot_mgr.gripper_close(current_robot_id))
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 关闭夹爪失败" << std::endl;
                    }
                    else
                    {
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪已成功关闭" << std::endl;
                    }
                    break;
                case 'o':
                    gripper_position = std::min(gripper_position + 1, 100);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪位置增加到 " << gripper_position << std::endl;
                    if (!robot_mgr.move_gripper(gripper_position, gripper_speed, gripper_force, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 移动夹爪失败" << std::endl;
                    }
                    break;
                case 'O':
                    gripper_position = std::min(gripper_position + 5, 100);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪位置增加到 " << gripper_position << std::endl;
                    if (!robot_mgr.move_gripper(gripper_position, gripper_speed, gripper_force, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 移动夹爪失败" << std::endl;
                    }
                    break;
                case 'u':
                    gripper_position = std::max(gripper_position - 1, 0);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪位置减少到 " << gripper_position << std::endl;
                    if (!robot_mgr.move_gripper(gripper_position, gripper_speed, gripper_force, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 移动夹爪失败" << std::endl;
                    }
                    break;
                case 'U':
                    gripper_position = std::max(gripper_position - 5, 0);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪位置减少到 " << gripper_position << std::endl;
                    if (!robot_mgr.move_gripper(gripper_position, gripper_speed, gripper_force, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 移动夹爪失败" << std::endl;
                    }
                    break;
                case 'p':
                case 'P':
                {
                    nova_robot_ctrl::Pose pose;
                    if (robot_mgr.get_current_pose_robot(pose, current_robot_id))
                    {
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前位置: 位置(" << std::fixed << std::setprecision(3) << pose.x << ", " << pose.y << ", " << pose.z << "), 姿态(" << pose.rx << ", " << pose.ry << ", " << pose.rz << ")" << std::endl;
                    }
                    else
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取当前位置失败" << std::endl;
                    }
                }
                break;
                case 'v':
                case 'V':
                    gripper_abs_position = std::max(gripper_abs_position - 100, 0);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪绝对位置减少到 " << gripper_abs_position << std::endl;
                    if (!robot_mgr.move_by_abs_pos_gripper(gripper_abs_position, 1500, 0, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 按绝对位置移动夹爪失败" << std::endl;
                    }
                    break;
                case 'n':
                case 'N':
                    gripper_abs_position = std::min(gripper_abs_position + 100, 100000);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将夹爪绝对位置增加到 " << gripper_abs_position << std::endl;
                    if (!robot_mgr.move_by_abs_pos_gripper(gripper_abs_position, 1500, 0, current_robot_id).first)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 按绝对位置移动夹爪失败" << std::endl;
                    }
                    break;
                case 'q':
                case 'Q':
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在退出..." << std::endl;
                    running = false;
                    break;
                case 't':
                case 'T':
                {
                    // 获取所有已启用的机器人ID
                    std::vector<int> enable_robots = robot_mgr.get_all_enable_robots_id();
                    if (enable_robots.empty()) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 没有启用的机器人" << std::endl;
                        break;
                    }
                    
                    // 查找当前机器人ID在启用列表中的位置
                    auto it = std::find(enable_robots.begin(), enable_robots.end(), current_robot_id);
                    int next_index = 0;
                    if (it != enable_robots.end()) {
                        // 找到当前ID，切换到下一个ID
                        size_t current_index = std::distance(enable_robots.begin(), it);
                        next_index = (current_index + 1) % enable_robots.size();
                    }
                    
                    int next_robot_id = enable_robots[next_index];
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 将机器人ID从 " << current_robot_id << " 切换到 " << next_robot_id << std::endl;
                    current_robot_id = next_robot_id;
                    
                    // 获取新机械臂的当前位姿
                    nova_robot_ctrl::Pose new_pose;
                    if (robot_mgr.get_current_pose_robot(new_pose, current_robot_id))
                    {
                        target_joints[0] = new_pose.x;
                        target_joints[1] = new_pose.y;
                        target_joints[2] = new_pose.z;
                        target_joints[3] = new_pose.rx;
                        target_joints[4] = new_pose.ry;
                        target_joints[5] = new_pose.rz;
                        
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人 " << current_robot_id << " 的当前位置: 位置(" << std::fixed << std::setprecision(3) << new_pose.x << ", " << new_pose.y << ", " << new_pose.z << "), 姿态(" << new_pose.rx << ", " << new_pose.ry << ", " << new_pose.rz << ")" << std::endl;
                    }
                    else
                    {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取机器人 " << current_robot_id << " 的当前位置失败" << std::endl;
                    }
                    
                    // 重新获取新机器人的夹爪位置
                    gripper_position = robot_mgr.get_current_position_gripper(current_robot_id);
                    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人 " << current_robot_id << " 的夹爪位置: " << gripper_position << std::endl;
                    break;
                }
                default:
                    std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 按下了未知键: " << key << std::endl;
                    break;
                }
            }
        }
        // 只有位置发生变化时才发送位置命令
        if (joints_changed && running)
        {
            if (robot_mgr.servo_p_robot(target_joints, 0.1, 50, 500, current_robot_id))
            {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已发送位置命令到机器人 " << current_robot_id << ": [" << std::fixed << std::setprecision(3) << target_joints[0] << ", " << target_joints[1] << ", " << target_joints[2] << ", " << target_joints[3] << ", " << target_joints[4] << ", " << target_joints[5] << "]" << std::endl;
                joints_changed = false; // 发送完成后重置标记
            }
            else
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发送位置命令到机器人 " << current_robot_id << " 失败" << std::endl;
            }
        }
        // 短暂延时以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 禁用机器人系统
    robot_mgr.release();
    // robot_mgr.disable_gripper();
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统已成功禁用" << std::endl;

    return 0;
}