# Nova Robot Control Package

## 概述

`nova_robot_ctrl` 是一个用于控制Nova机械臂的ROS2功能包。该功能包提供了对Nova机械臂和夹爪的基本控制功能，包括连接管理、关节控制、位姿控制以及夹爪控制等。

## 功能特性

- 机械臂TCP连接管理
- 关节空间伺服控制 (ServoJ)
- 笛卡尔空间伺服控制 (ServoP)
- 关节角度读取
- 机械臂使能控制
- 实时状态信息获取
- 夹爪控制（开合、位置控制）
- SCS系列伺服电机控制
- 键盘控制测试程序

## 代码结构

```
nova_robot_ctrl/
├── include/
│   └── nova_robot_ctrl/
│       ├── nova_robot_ctrl.h      # 机械臂控制类头文件
│       ├── nova_gripper_ctrl.h    # 夹爪控制类头文件
│       ├── scscl.h                # SCSCL伺服控制类头文件
│       ├── sms_sts.h              # SMS/STS伺服控制类头文件
│       ├── protocol_packet_handler.h # SCS协议包处理器头文件
│       └── port_handler.h         # 串口处理器头文件
├── src/
│   ├── nova_robot_ctrl.cpp        # 机械臂控制类实现
│   ├── nova_gripper_ctrl.cpp      # 夹爪控制类实现
│   ├── scscl.cpp                  # SCSCL类实现
│   ├── sms_sts.cpp                # SMS/STS类实现
│   ├── protocol_packet_handler.cpp # SCS协议包处理器实现
│   └── port_handler.cpp           # 串口处理器实现
├── test/
│   └── robot_move_test.cpp        # 键盘控制测试程序
├── CMakeLists.txt                 # 构建配置文件
├── package.xml                    # 包描述文件
└── README.md                      # 说明文档
```

## 依赖项

- ROS2 (Humble或更新版本)
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `yaml_cpp_vendor`
- `file_operate`
- `Boost` (system component)
- `fmt`

## 类说明

### RobotMgr 类

机器人管理类，提供对机械臂和夹爪的统一管理功能：

#### 公共接口

- `RobotMgr()` - 构造函数
- `~RobotMgr()` - 析构函数
- `bool create_default_config(const std::string& config_path)` - 创建默认配置文件
- `bool load_config(const std::string& config_path)` - 加载配置文件

##### 机械手控制接口
- `bool enable_robot()` - 启用机械手
- `void disable_robot()` - 禁用机械手
- `bool get_current_pose_robot(Pose& pose)` - 获取当前机械手位姿
- `bool get_current_position_robot(std::vector<double>& position)` - 获取当前机械手位置
- `bool set_position_robot(const std::vector<double>& position)` - 设置机械手位置

##### 夹爪控制接口
- `bool enable_gripper()` - 启用夹爪
- `bool disable_gripper()` - 要用夹爪
- `bool open_gripper()` - 张开夹爪
- `bool close_gripper()` - 闭合夹爪
- `bool get_gripper_position(int& position)` - 获取夹爪当前位置
- `bool set_gripper_position(int position, int speed = 50, int force = 0)` - 设置夹爪位置
- `bool set_gripper_port_and_id(const std::string& port, int id)` - 设置夹爪串口路径和ID

#### 数据结构

- `RobotConfig` - 机器人配置结构体
  - `ip` - 机器人IP地址
  - `ranges` - 移动范围参数，key为工具坐标系ID，value为{x_min, x_max, y_min, y_max, z_min, z_max}

- `GripperConfig` - 夹爪配置结构体
  - `id` - 夹爪ID
  - `serial_port` - 串口路径
  - `servo_ranges` - 舵机参数范围，{pos_min, pos_max, speed_min, speed_max, force_min, force_max}

#### 配置文件

默认配置文件路径：`install/sys_config/nova_robot_config.yaml`

配置文件格式：
```yaml
robot_config:
  ip: "192.168.5.1"
  tool_0:
    x_min: -400.00
    x_max: 275.00
    y_min: -500.00
    y_max: -165.00
    z_min: 240.00
    z_max: 525.00
  tool_1:
    x_min: -390.00
    x_max: 270.00
    y_min: -510.00
    y_max: -195.00
    z_min: 44.00
    z_max: 350.00
  tool_2:
    x_min: -390.00
    x_max: 270.00
    y_min: -510.00
    y_max: -195.00
    z_min: 44.00
    z_max: 350.00

gripper_config:
  id: 1
  serial_port: "/dev/ttyUSB0"
  servo_params:
    pos_min: 2000
    pos_max: 3000
    speed_min: 0
    speed_max: 4096
    force_min: 0
    force_max: 100
```



### NovaRobotCtrl 类

机械臂控制类，提供以下主要功能：

#### 公共接口

- `NovaRobotCtrl()` - 构造函数
- `~NovaRobotCtrl()` - 析构函数
- `bool open(const std::string& ip)` - 连接机械臂
- `void close()` - 断开连接
- `bool servo_j(const std::vector<double>& joint, float t = 0.1, float aheadtime = 50, float gain = 500)` - 关节空间伺服控制
- `bool get_angle(std::vector<double>& joint)` - 获取当前关节角度
- `bool enable_robot()` - 使能机械臂
- `pushed_info get_pushed_info()` - 获取推送信息
- `bool get_current_pose(Pose& pose)` - 获取当前机械手的实时坐标
- `bool is_connected() const` - 检查连接状态

#### 数据结构

- `pushed_info` - 包含机械臂实时状态信息的结构体
  - `time_stamp` - 时间戳
  - `q_actual` - 实际关节位置
  - `q_d_actual` - 实际关节速度
  - `enable_status` - 使能状态
  - `error_status` - 错误状态

- `Pose` - 包含机械手位置和姿态信息的结构体
  - `x` - X坐标
  - `y` - Y坐标
  - `z` - Z坐标
  - `rx` - Rx旋转角
  - `ry` - Ry旋转角
  - `rz` - Rz旋转角

### NovaGripperCtrl 类

夹爪控制类，提供以下主要功能：

#### 公共接口

- `NovaGripperCtrl(const std::string& port, int id_name, const std::vector<int>& servo_pos)` - 构造函数
- `~NovaGripperCtrl()` - 析构函数
- `bool connect()` - 连接夹爪
- `bool is_connected() const` - 检查连接状态
- `bool ping()` - Ping伺服器
- `bool set_torque_limit(int limit)` - 设置扭矩限制
- `bool set_port_and_id(const std::string& port, int id)` - 设置串口路径和伺服ID
- `void disconnect()` - 断开连接
- `int get_min_position() const` - 获取最小位置
- `int get_max_position() const` - 获取最大位置
- `int get_open_position() const` - 获取打开位置
- `int get_closed_position() const` - 获取闭合位置
- `bool is_open() const` - 检查是否打开
- `bool is_closed() const` - 检查是否闭合
- `int get_current_position() const` - 获取当前位置
- `int get_current_speed() const` - 获取当前速度
- `std::pair<bool, int> move(int position, int speed, int force)` - 移动夹爪
- `std::pair<bool, int> reset_position(int speed = 50, int force = 0)` - 复位夹爪位置到4000
- `std::pair<bool, int> move_by_abs_pos(int position, int speed = 50, int force = 0)` - 通过绝对位置移动夹爪
- `GripperStatus get_status() const` - 获取夹爪完整状态信息

#### 数据结构

- `GripperStatus` - 包含夹爪状态信息的结构体
  - `position` - 当前位置
  - `speed` - 当前速度
  - `load` - 负载
  - `voltage` - 电压
  - `temperature` - 温度
  - `moving` - 是否正在移动
  - `current` - 电流

### SCSCL 类

SCS系列伺服电机控制类，提供以下主要功能：

#### 公共接口

- `SCSCL(int param)` - 构造函数
- `~SCSCL()` - 析构函数
- `int begin(int baudrate, const char* port)` - 初始化串口连接
- `void end()` - 关闭串口连接
- `int ping(int id)` - Ping伺服器
- `int read_pos(int id)` - 读取当前位置
- `int read_speed(int id)` - 读取当前速度
- `int write_pos(int id, int position, int time, int speed)` - 写入位置、时间、速度
- `int write_pos_ex(int id, int position, int speed, int acc)` - 写入位置（扩展版）

### sms_sts 类

SMS/STS系列伺服电机控制类，继承自protocol_packet_handler：

#### 公共接口

- `sms_sts(PortHandler* portHandler)` - 构造函数
- `~sms_sts()` - 析构函数
- `int write_pos_ex(int scs_id, int position, int speed, int acc)` - 写入位置（扩展版）
- `int read_pos(int scs_id, int* position, int* error)` - 读取当前位置
- `int read_speed(int scs_id, int* speed, int* error)` - 读取当前速度
- `int read_pos_speed(int scs_id, int* position, int* speed, int* error)` - 读取位置和速度
- `int read_moving(int scs_id, int* moving, int* error)` - 读取运动状态
- `int sync_write_pos_ex(int scs_id, int position, int speed, int acc)` - 同步写入位置（扩展版）
- `int reg_write_pos_ex(int scs_id, int position, int speed, int acc)` - 寄存器写入位置（扩展版）
- `int reg_action()` - 执行寄存器动作
- `int wheel_mode(int scs_id)` - 设置轮式模式
- `int write_spec(int scs_id, int speed, int acc)` - 写入特殊参数
- `int lock_eprom(int scs_id)` - 锁定EPROM
- `int unlock_eprom(int scs_id)` - 解锁EPROM

### protocol_packet_handler 类

SCS协议包处理器类，处理SCS系列伺服电机的通信协议：

#### 公共接口

- `protocol_packet_handler(PortHandler* portHandler, int protocol_end)` - 构造函数
- `~protocol_packet_handler()` - 析构函数
- `int scs_tohost(int a, int b)` - 将SCS数据转换为主机数据
- `int scs_toscs(int a, int b)` - 将主机数据转换为SCS数据
- `int scs_lobyte(int w)` - 获取数据的低字节
- `int scs_hibyte(int w)` - 获取数据的高字节
- `int scs_loword(int l)` - 获取数据的低字
- `int scs_hiword(int l)` - 获取数据的高字
- `int scs_makeword(int low, int high)` - 合成字数据
- `int scs_makedword(int low, int high)` - 合成双字数据
- `int tx_packet(unsigned char* txpacket)` - 发送数据包
- `int rx_packet(unsigned char* rxpacket)` - 接收数据包
- `int tx_rx_packet(unsigned char* txpacket, unsigned char* rxpacket, int* error)` - 发送并接收数据包
- `int ping(int scs_id, int* error)` - Ping指令
- 各种读写接口（read_tx, read_rx, write_tx_only等）
- 同步读写接口（sync_read_tx, sync_write_tx_only等）

### PortHandler 类

串口处理器类，负责串口通信的底层操作：

#### 公共接口

- `PortHandler(const char* port_name)` - 构造函数
- `~PortHandler()` - 析构函数
- `bool open_port()` - 打开串口
- `void close_port()` - 关闭串口
- `void clear_port()` - 清空串口缓冲区
- `bool set_baud_rate(int baudrate)` - 设置波特率
- `int get_baud_rate()` - 获取当前波特率
- `int get_bytes_available()` - 获取可用字节数
- `int read_port(unsigned char* packet, int length)` - 从串口读取数据
- `int write_port(unsigned char* packet, int length)` - 向串口写入数据
- `void set_packet_timeout(int packet_length)` - 设置数据包超时时间
- `void set_packet_timeout_millis(int msec)` - 设置数据包超时时间（毫秒）
- `bool is_packet_timeout()` - 检查数据包是否超时
- `double get_current_time()` - 获取当前时间
- `double get_time_since_start()` - 获取自开始以来的时间

## 构建说明

1. 确保已在ROS2环境中：
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 进入工作空间目录并构建：
   ```bash
   cd /home/xlh/WorkStationData/project
   colcon build --packages-select nova_robot_ctrl
   ```

3. 激活构建结果：
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 库使用

#### 使用独立控制类

```cpp
#include "nova_robot_ctrl/nova_robot_ctrl.h"
#include "nova_robot_ctrl/nova_gripper_ctrl.h"

// 创建机械臂控制对象
nova_robot_ctrl::NovaRobotCtrl robot_ctrl;

// 连接机械臂
if (robot_ctrl.open("192.168.5.1")) {
    // 使能机械臂
    robot_ctrl.enable_robot();
    
    // 发送关节角度命令
    std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot_ctrl.servo_j(joints);
    
    // 获取当前机械手位姿
    nova_robot_ctrl::Pose current_pose;
    if (robot_ctrl.get_current_pose(current_pose)) 
    {
        // 使用current_pose
        std::cout << "Current position: (" << current_pose.x << ", " 
                  << current_pose.y << ", " << current_pose.z << ")" << std::endl;
    }
}

// 创建夹爪控制对象
std::vector<int> servo_pos = {0, 255};
nova_robot_ctrl::NovaGripperCtrl gripper_ctrl("/dev/ttyUSB0", 1, servo_pos);

// 连接夹爪
if (gripper_ctrl.connect()) {
    // 使用相对位置移动夹爪
    gripper_ctrl.move(50, 50, 0);
    
    // 使用绝对位置移动夹爪
    gripper_ctrl.move_by_abs_pos(2000, 50, 0);
    
    // 复位夹爪位置
    gripper_ctrl.reset_position();
    
    // 获取夹爪状态信息
    nova_robot_ctrl::GripperStatus status = gripper_ctrl.get_status();
    std::cout << "Gripper position: " << status.position << std::endl;
    std::cout << "Gripper speed: " << status.speed << std::endl;
    std::cout << "Gripper voltage: " << status.voltage << std::endl;
}
```

#### 使用RobotMgr统一管理类

```cpp
#include "nova_robot_ctrl/robot_mgr.h"

// 创建机器人管理对象
nova_robot_ctrl::RobotMgr robot_mgr;

// 加载配置文件
if (robot_mgr.load_config("install/sys_config/nova_robot_config.yaml")) {
    // 启用机器人系统
    if (robot_mgr.enable_robot()) {
        // 设置机械手位置
        std::vector<double> position = {100.0, 200.0, 300.0, 0.0, 0.0, 0.0};
        robot_mgr.set_position_robot(position);
        
        // 获取当前机械手位姿
        nova_robot_ctrl::Pose pose;
        if (robot_mgr.get_current_pose_robot(pose)) {
            std::cout << "Current pose: (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
        }
        
        // 控制夹爪
        robot_mgr.set_gripper_position(128, 50, 0);
        
        // 启用夹爪
        robot_mgr.enable_gripper();
        
        // 禁用夹爪
        robot_mgr.disable_gripper();
        
        // 禁用机器人系统
        robot_mgr.disable_robot();
    }
}
```
### 测试程序

运行键盘控制测试程序：

```bash
ros2 run nova_robot_ctrl robot_move_test
```

控制说明：
- 方向键：控制XY平面移动
- I/K键：控制Z轴移动
- G键：打开夹爪
- H键：关闭夹爪
- J键：Ping夹爪连接
- U/O键：调整夹爪位置（U减小，O增大）
- V/N键：调整夹爪绝对位置（V减小，N增大）
- P键：获取当前位姿
- Q键：退出程序

夹爪控制变量：
- gripper_position：相对位置变量，初始值为50
- gripper_speed：相对速度变量，初始值为50
- gripper_force：相对力度变量，初始值为0
- gripper_abs_position：绝对位置变量，初始值为2000

## 日志记录

所有重要操作都会通过ROS2日志系统记录，包括：
- 连接/断开连接事件
- 命令发送和接收
- 错误和警告信息
- 调试信息（在DEBUG级别）

可通过ROS2日志配置调整日志级别。

## 注意事项

1. 确保机械臂IP地址正确配置
2. 确保夹爪串口设备权限正确设置
3. 在使用前务必使能机械臂
4. 注意安全操作，避免机械臂碰撞
5. 夹爪的设置位置与实际反馈位置不一致,测试程序夹爪控制如果存在异常，需要手动矫正夹爪位置的设置范围。即使用V/N键手动验证夹爪设置范围，再修改servo_pos，重新编译运行程序。
6. 初次使用需要修改串口权限：
```
#打开 udev 规则
sudo nano /etc/udev/rules.d/99-ttyUSB.rules
#添加以下语句
SUBSYSTEM=="tty", GROUP="dialout", MODE="0666"

#重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```
## 故障排除

1. **无法连接机械臂**：
   - 检查IP地址是否正确
   - 检查网络连接
   - 确认机械臂电源已开启

2. **无法连接夹爪**：
   - 检查串口设备路径
   - 确认设备权限（可能需要sudo）
   - 检查串口线连接

3. **命令发送失败**：
   - 检查机械臂是否已使能
   - 确认连接状态
   - 查看日志信息获取详细错误

## 许可证

本项目采用Apache License 2.0许可证。

## 更新日志

### 2025-12-05
- 修复了机械臂移动后立即读取位置时返回空值的问题
- 在`get_current_pose`函数中添加了重试机制，最多尝试5次获取位姿，每次重试间隔1000毫秒
- 增加了适当的延迟时间，确保机械臂完全稳定后再读取位置信息
- 修改了`nova_robot_ctrl_ros`节点中的相关函数，增加了移动后的延迟时间
- 在`waitForRobotToStop`函数中增加了2000毫秒的延迟，确保机械臂完全停稳并且位置信息已更新
