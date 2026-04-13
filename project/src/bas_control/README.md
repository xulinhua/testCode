# bas_control 统一部署控制层

## 项目概述

`bas_control` 是具身智能机器人视觉识别系统的**统一部署控制层**，负责整个视觉系统的启动管理、状态监控、任务调度和配置热更新。该系统基于 ROS 2 Humble 框架构建，采用 C++17 标准实现，具有良好的可扩展性和可维护性。

## 系统架构

### 核心组件

```
bas_control/
├── include/bas_control/          # 头文件目录
│   ├── launch_mgr.hpp           # 启动管理器
│   ├── status_monitor.hpp       # 状态监控器
│   ├── task_scheduler.hpp       # 任务调度器
│   ├── config_hot_updater.hpp   # 配置热更新器
│   ├── system_mgr.hpp           # 系统管理器（核心协调类）
│   └── module_info.hpp          # 模块信息定义（数据结构）
├── src/                         # 源文件目录
│   ├── launch_mgr.cpp
│   ├── status_monitor.cpp
│   ├── task_scheduler.cpp
│   ├── config_hot_updater.cpp
│   ├── system_mgr.cpp
│   └── bas_control_node.cpp     # ROS节点主程序
├── config/                      # 配置文件目录
│   ├── bas_control.yaml         # 系统配置文件
│   └── log_config.yaml          # 日志配置文件
├── test/                        # 测试文件目录
│   ├── test_launch_mgr.cpp
│   ├── test_status_monitor.cpp
│   └── test_task_scheduler.cpp
├── scripts/                     # 脚本目录
│   ├── bas_control.service      # systemd服务配置
│   └── ...                      # 其他脚本
├── CMakeLists.txt               # 构建配置文件
├── package.xml                  # ROS包配置文件
└── README.md                    # 项目说明文档
```

### 架构层次

```
┌─────────────────────────────────────────────────────────────┐
│                     bas_control_node                         │
│                    (ROS节点入口层)                            │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                      SystemMgr                               │
│                   (系统管理器 - 核心协调类)                    │
└────────┬──────────┬──────────┬──────────────┬───────────────┘
         │          │          │              │
         ▼          ▼          ▼              ▼
    ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────────┐
    │LaunchMgr│ │ Status  │ │  Task   │ │ ConfigHot    │
    │         │ │Monitor  │ │Scheduler│ │ Updater      │
    │启动管理 │ │状态监控 │ │任务调度 │ │配置热更新    │
    └─────────┘ └─────────┘ └─────────┘ └──────────────┘
         │          │          │              │
         └──────────┴──────────┴──────────────┘
                          │
                          ▼
              ┌──────────────────────┐
              │   bas_operate        │
              │  (外部工具函数库)     │
              └──────────────────────┘
```

## 功能特性

### 核心特性

#### ✨ 动态可执行文件查找（新增）

系统新增动态查找可执行文件功能，无需在配置文件中指定模块的可执行路径：

**实现位置**：`bas_operate/src/file_operate.cpp`

```cpp
// 自动查找模块可执行文件
std::string executable_path = find_ros_executable("bas_sys_config_ros");
// 返回: /home/user/testCode/dev/install/bas_sys_config_ros/lib/bas_sys_config_ros/bas_sys_config_ros_node
```

**查找逻辑**：
```
get_install_dir() → /home/user/testCode/dev/install/
  └─ <package_name>/ → bas_sys_config_ros/
      └─ lib/
          └─ <package_name>/ → bas_sys_config_ros/
              └─ <executable> → bas_sys_config_ros_node
```

**优势**：
- ✅ 零配置：不需要在 YAML 文件中指定可执行路径
- ✅ 环境适配：自动适配不同环境（开发、测试、生产）
- ✅ 易维护：模块安装位置变更时无需修改配置

#### ✨ 真实进程启动（新增）

使用真实的进程启动替代模拟启动：

**实现位置**：`launch_mgr.cpp:532-620`

```cpp
// 1. 查找可执行文件
std::string executable_path = basmodule::find_ros_executable(module_name);

// 2. 启动进程
int pid = basmodule::launch_process(executable_path, arguments, "");

// 3. 记录真实 PID
module_info.pid = pid;
module_info.executable_path = executable_path;
```

**进程管理**：
- `launch_process()` - 启动进程（fork + execvp）
- `stop_process()` - 停止进程（SIGTERM/SIGKILL）
- `is_process_alive()` - 检查进程存活
- `wait_for_process()` - 等待进程退出

#### ✨ 场景驱动启动（新增）

系统根据场景启动模块，而非启动所有模块：

**启动流程**：
```
TaskScheduler 构造
  └─ 设置默认场景（IDLE）
      └─ active_modules = ["bas_sys_config_ros", "cam_mgr_ros"]

startSystem()
  └─ startCoreServices()
      └─ 根据 task_scheduler_->getActiveModules() 启动模块
          └─ 只启动 2 个模块（而非所有 7 个）
```

**资源节约**：
| 场景 | 启动模块数 | 预估CPU | 预估内存 |
|------|----------|--------|---------|
| idle | 2 | 10% | 500MB |
| full | 7 | 70% | 6GB |

---

### 1. 启动管理 (Launch Manager)
- 按依赖关系拓扑排序启动模块
- 模块生命周期管理（启动/停止/重启）
- 启动超时处理和重试机制
- 循环依赖检测
- 模块状态跟踪和回调通知

### 2. 状态监控 (Status Monitor)
- 实时系统状态监控（CPU/内存/GPU/温度）
- 模块健康检查和心跳检测
- 进程存活检查
- 阈值告警和异常检测
- 状态变化回调通知

### 3. 任务调度 (Task Scheduler)
- 基于场景的资源调度
- 场景切换管理（5种预定义场景）
- 模块动态加载/卸载
- 调度历史记录和统计

### 4. 配置热更新 (Config Hot Updater)
- 运行时参数配置更新
- 模型文件热加载
- A/B测试版本切换
- 配置版本管理和回滚
- 配置文件变更监控

### 5. 系统管理 (System Manager)
- 核心协调类，统筹管理所有子组件
- 系统生命周期管理（初始化/启动/停止/重启）
- 组件间依赖注入和连接
- 配置加载和解析
- 系统自检和健康状态评估

## 依赖关系

### 系统依赖
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2 Humble
- C++17 编译器
- CMake 3.8+

### ROS包依赖

| 依赖包 | 说明 |
|--------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `std_msgs` | 标准消息类型 |
| `std_srvs` | 标准服务类型 |
| `builtin_interfaces` | 内置接口 |
| `yaml_cpp_vendor` | YAML解析库 |
| `ament_index_cpp` | ament索引查找 |

### 自定义包依赖

| 依赖包 | 说明 |
|--------|------|
| `bas_operate` | 基础操作库（提供进程管理、资源监控、拓扑排序等工具函数） |
| `data_handler` | 数据处理库 |
| `bas_operate_ros` | ROS操作库 |
| `log_system` | 日志系统框架 |
| `custom_msgs_comm` | 自定义消息和服务类型 |

## 安装配置

### 1. 环境准备

```bash
# 安装ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装依赖包
sudo apt install libyaml-cpp-dev

# 安装编译工具
sudo apt install cmake build-essential
```

### 2. 编译项目

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆项目（假设已克隆到当前位置）
# git clone <repository_url> bas_control

# 返回工作空间根目录
cd ~/ros2_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译项目
colcon build --packages-select bas_control

# 加载环境
source install/setup.bash
```

### 3. 配置文件设置

配置文件位于 `config/bas_control.yaml`，主要配置项包括：

**重要说明**:
- 配置文件在编译时会自动安装到 ROS 2 工作空间的正确位置
- 系统使用 `ament_index_cpp` 自动查找配置文件，无需手动指定绝对路径
- 默认配置文件路径: `<workspace>/install/bas_control/share/bas_control/config/bas_control.yaml`

```yaml
# 系统配置
system:
  platform: "jetson_orin_nx"
  ros_version: "humble"
  config_path: "/config/bas_control"
  # 默认启动场景（可选值：idle, navigation, interaction, manipulation, full）
  # 更改此值可以改变系统启动时默认加载的场景
  default_scene: "idle"

# 模块配置
modules:
  startup_order:
    - bas_sys_config_ros
    - cam_mgr_ros
    - yolo_det
    - face_det
    - hand_gesture_rec
    - ppocr
    - yolo_obb_det
  
  dependencies:
    bas_sys_config_ros: []
    cam_mgr_ros: [bas_sys_config_ros]
    yolo_det: [cam_mgr_ros, bas_sys_config_ros]
    # ... 其他模块依赖

# 场景配置
scenes:
  idle:
    modules: [bas_sys_config_ros, cam_mgr_ros]
    description: "空闲场景，仅启动基础服务"
  navigation:
    modules: [bas_sys_config_ros, cam_mgr_ros, yolo_det, ppocr]
    description: "导航场景，启动检测和OCR服务"
  # ... 其他场景
```

### 4. 默认启动场景配置

**功能说明**：
- 系统启动时自动加载 `default_scene` 指定的场景
- 更改配置文件中的 `default_scene` 值即可改变默认启动场景
- 无需修改代码，支持动态配置

**配置示例**：
```yaml
system:
  # 启动 IDLE 场景（仅基础服务，资源占用最低）
  default_scene: "idle"
  
  # 或启动 NAVIGATION 场景（目标检测+OCR）
  # default_scene: "navigation"
  
  # 或启动 INTERACTION 场景（人脸+手势识别）
  # default_scene: "interaction"
  
  # 或启动 FULL 场景（全功能运行）
  # default_scene: "full"
```

**场景资源占用对比**：
| 场景 | 启动模块数 | 预估CPU | 预估内存 |
|------|----------|--------|---------|
| idle | 2 | 10% | 500MB |
| navigation | 4 | 40% | 3GB |
| interaction | 4 | 35% | 2.5GB |
| manipulation | 4 | 45% | 3.5GB |
| calibration | 4 | 30% | 2GB |
| full | 9 | 80% | 7GB |

## 系统启动流程

### 完整执行流程

```
main()
  │
  ├─ 创建 BasControlNode
  │   │
  │   ├─ SystemMgr 构造
  │   │   │
  │   │   ├─ 加载配置文件（bas_control.yaml）
  │   │   │   └─ 解析 default_scene（默认：idle）
  │   │   │
  │   │   └─ TaskScheduler 构造 ⭐ [步骤1：设置默认场景]
  │   │       │
  │   │       ├─ 从配置文件加载场景配置
  │   │       │   (idle, navigation, interaction, manipulation, full)
  │   │       │
  │   │       ├─ 设置默认启动场景
  │   │       │   current_scene_ = config_.default_scene
  │   │       │   （如配置为 "navigation" 则启动导航场景模块）
  │   │       │
  │   │       └─ LOG: "默认启动场景设置为: navigation"
  │   │
  │   ├─ SystemMgr::initialize()
  │   │   ├─ initializeComponents() - 创建各组件
  │   │   └─ setupComponentConnections() - 建立连接
  │   │
  │   └─ 创建 ROS 接口
  │
  ├─ startSystem() ⭐ [步骤2：启动系统]
  │   │
  │   └─ system_mgr_->start()
  │       │
  │       └─ startCoreServices() ⭐ [步骤3：启动核心服务]
  │           │
  │           ├─ status_monitor_->startMonitoring()
  │           ├─ config_updater_->startMonitoring()
  │           │
  │           └─ 启动场景模块 ⭐ [步骤4：启动默认场景模块]
  │               │
  │               ├─ active_modules = task_scheduler_->getActiveModules()
  │               │   └─ 返回配置的默认场景的模块列表
  │               │   （如 default_scene="navigation"，返回 4 个模块）
  │               │
  │               └─ launch_mgr_->startModules(active_modules)
  │                   │
  │                   ├─ 启动 bas_sys_config_ros
  │                   ├─ 启动 cam_mgr_ros
  │                   ├─ 启动 yolo_det
  │                   └─ 启动 ppocr
  │
  └─ rclcpp::spin(node)
```

### 默认场景设置

**位置**：`src/task_scheduler.cpp:157-168`

```cpp
// TaskScheduler 构造函数
TaskScheduler::TaskScheduler(const ConfigParams& config)
{
    // 从配置文件加载场景...
    
    // ✅ 设置默认启动场景（从配置文件读取）
    std::string default_scene_name = config_.default_scene.empty() ? "idle" : config_.default_scene;
    SceneType default_scene_type = stringToSceneType(default_scene_name);
    
    // 验证默认场景是否存在
    auto it = scenes_.find(default_scene_type);
    if (it == scenes_.end()) {
        LOG_WARN("配置的默认场景 '%s' 不存在，使用 'idle' 场景", default_scene_name.c_str());
        default_scene_type = SceneType::IDLE;
    }
    
    current_scene_ = default_scene_type;
    active_modules_ = it->second.active_modules;
    
    LOG_INFO("默认启动场景设置为: %s", sceneTypeToString(default_scene_type).c_str());
}
```

**说明**：
- 从配置文件读取 `default_scene` 字段确定默认启动场景
- 如果配置的场景不存在，自动回退到 `idle` 场景
- 支持动态配置：修改配置文件后重新启动即可生效

### 启动默认场景模块

**位置**：`src/system_mgr.cpp:553-561`

```cpp
bool SystemMgr::startCoreServices() 
{
    // 启动监控服务...
    
    // ✅ 根据当前场景启动模块
    if (task_scheduler_ && launch_mgr_) {
        std::vector<std::string> active_modules = task_scheduler_->getActiveModules();
        LOG_INFO("根据当前场景启动模块，共 %zu 个", active_modules.size());
        
        if (!launch_mgr_->startModules(active_modules)) {
            LOG_WARN("启动场景模块失败");
            return false;
        }
    }
    
    return true;
}
```

**说明**：
- 自动获取当前场景的活动模块（默认是 IDLE 场景）
- 使用 `find_ros_executable()` 动态查找可执行文件
- 使用 `launch_process()` 真实启动进程

### 启动日志输出示例

```
[INFO] 正在初始化控制节点...
[INFO] 任务调度器已初始化，共 5 个场景
[INFO] 场景: idle, 描述: 空闲场景，仅启动基础服务, 活动模块数: 2
[INFO]   活动模块: [bas_sys_config_ros, cam_mgr_ros]
...
[INFO] 系统初始化成功
[INFO] 正在启动视觉系统...
[INFO] 根据当前场景启动模块，共 2 个  ⭐
[INFO] 正在批量启动模块，共 2 个
[INFO] 正在启动模块: bas_sys_config_ros
[INFO] 模块可执行文件路径: /home/user/.../bas_sys_config_ros_node
[INFO] 进程启动成功，PID: 12345
[INFO] 正在启动模块: cam_mgr_ros
[INFO] 模块可执行文件路径: /home/user/.../cam_mgr_ros_node
[INFO] 进程启动成功，PID: 12346
[INFO] 所有模块启动成功
[INFO] 视觉系统启动成功
```

## 运行方法

### 1. 启动系统

#### 方式一：直接运行（推荐用于测试）

```bash
# 加载ROS 2环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 启动bas_control节点（默认启动 idle 场景）
ros2 run bas_control bas_control_node
```

**启动说明**：
- 系统启动时自动使用默认场景（idle）
- 只启动 2 个基础模块：`bas_sys_config_ros` 和 `cam_mgr_ros`
- 可通过 `switchScene` 服务切换到其他场景

#### 方式二：使用启动脚本（带进程检查）

```bash
# 使用启动脚本（自动检查进程是否运行）
./scripts/start_bas_control.sh              # 前台运行
./scripts/start_bas_control.sh --daemon     # 后台运行
./scripts/start_bas_control.sh --check      # 仅检查状态

# 脚本功能：
# ✅ 自动检查 bas_control_node 是否已在运行
# ✅ 自动加载 ROS2 环境（支持 humble/foxy）
# ✅ 自动检测工作空间路径
# ✅ 显示进程详细信息（PID、运行时长、内存、CPU）
```

#### 方式三：使用监控脚本（守护进程模式）

```bash
# 启动监控守护进程
./scripts/monitor_bas_control.sh --daemon

# 查看监控状态
./scripts/monitor_bas_control.sh --status

# 停止监控守护进程
./scripts/monitor_bas_control.sh --stop

# 监控功能：
# ✅ 每10秒检查一次进程状态
# ✅ 进程崩溃时自动重启
# ✅ 重启频率限制（每小时最多5次）
# ✅ 详细日志记录
```

#### 方式四：使用 systemd 服务（生产环境推荐）

**1. 安装 systemd 服务**：

```bash
# 复制服务配置文件
sudo cp scripts/bas_control.service /etc/systemd/system/

# 修改服务配置（根据实际路径）
sudo nano /etc/systemd/system/bas_control.service
# 修改以下路径：
# - WorkingDirectory: ROS2工作空间路径
# - ExecStartPre: ROS2环境路径
# - ExecStart: 启动命令

# 重新加载 systemd
sudo systemctl daemon-reload

# 启用开机自启
sudo systemctl enable bas_control
```

**2. 使用 systemd 管理服务**：

```bash
# 启动服务
sudo systemctl start bas_control

# 停止服务
sudo systemctl stop bas_control

# 重启服务
sudo systemctl restart bas_control

# 查看服务状态
sudo systemctl status bas_control

# 查看服务日志
sudo journalctl -u bas_control -f

# 查看最近100行日志
sudo journalctl -u bas_control -n 100
```

**systemd 服务优势**：
- ✅ 开机自启动
- ✅ 崩溃自动重启
- ✅ 日志集中管理
- ✅ 资源限制和监控
- ✅ 优雅停止（30秒超时）

**对比总结**：

| 启动方式 | 适用场景 | 进程管理 | 自动重启 | 日志管理 |
|---------|---------|---------|---------|---------|
| 直接运行 | 开发测试 | 手动管理 | ❌ | 终端输出 |
| 启动脚本 | 简单部署 | 脚本检查 | ❌ | 终端输出 |
| 监控脚本 | 长期运行 | 自动监控 | ✅ | 文件日志 |
| systemd | 生产环境 | 系统级管理 | ✅ | journal日志 |

### 2. 系统控制命令

#### 场景切换服务（智能健康检查）

**✨ 新功能：场景切换时自动模块健康检查**

接收到切换场景指令后，系统会自动执行智能健康检查：

1. **场景相同性检查**：判断目标场景是否与当前场景相同
2. **模块健康检查**：遍历当前场景的所有模块，检查运行状态
3. **自动修复机制**：对未正常运行的模块执行自动启动
4. **结果反馈**：记录并上报健康检查和修复结果

```bash
# 查看当前场景和运行状态
ros2 service call /bas/get_status std_srvs/srv/Trigger

# 切换到导航场景（启动目标检测和OCR）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'navigation'}"

# 切换到交互场景（启动人脸和手势识别）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'interaction'}"

# 切换到操作场景（启动精确检测）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'manipulation'}"

# 切换到完整场景（启动所有模块）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'full'}"

# 切换到标定场景（启动标识识别和手眼标定）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'calibration'}"

# 切换回空闲场景（只保留基础服务）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'idle'}"

# 💡 当场景与当前场景相同时，自动检查模块健康状态并修复故障模块
# 示例：如果 cam_mgr_ros 模块崩溃，再次调用 idle 场景切换会自动重启该模块
```

**测试验证示例**：

```bash
# 1. 系统启动（IDLE场景，启动2个模块）
ros2 run bas_control bas_control_node

# 2. 模拟模块故障（手动kill一个模块）
ps aux | grep cam_mgr_ros
kill -9 <cam_mgr_ros_pid>

# 3. 再次调用IDLE场景切换（触发健康检查）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'idle'}"

# 4. 观察日志输出
# [INFO] 已在当前场景中: idle，检查模块健康状态
# [WARN] 发现未运行模块: cam_mgr_ros，将尝试启动
# [INFO] 正在启动模块: cam_mgr_ros
# [INFO] 模块启动成功: cam_mgr_ros
# [INFO] 所有未运行的模块已成功启动
```

#### 场景切换效果

| 切换命令 | 启动模块 | 停止模块 | 总模块数 |
|---------|---------|---------|---------|
| `idle` → `navigation` | yolo_det, ppocr | - | 2 → 4 |
| `idle` → `interaction` | face_det, hand_gesture_rec | - | 2 → 4 |
| `idle` → `calibration` | marker_detect_ros, hand_eye_calib_ros | - | 2 → 4 |
| `navigation` → `idle` | - | yolo_det, ppocr | 4 → 2 |
| `idle` → `full` | yolo_det, face_det, hand_gesture_rec, ppocr, yolo_obb_det, marker_detect_ros, hand_eye_calib_ros | - | 2 → 9 |
| `full` → `idle` | - | yolo_det, face_det, hand_gesture_rec, ppocr, yolo_obb_det, marker_detect_ros, hand_eye_calib_ros | 9 → 2 |

#### 系统生命周期控制

**✨ 新功能：停止系统自动退出进程**

接收到停止指令后，系统会：
1. 停止当前场景下的所有模块
2. 清理系统资源
3. **自动退出 bas_control_node 进程**（新增）

```bash
# 启动系统（如果已停止）
ros2 service call /bas/start_system std_srvs/srv/Trigger

# 停止系统并退出进程（新增：500ms后自动退出）
ros2 service call /bas/stop_system std_srvs/srv/Trigger

# 获取可用场景列表
ros2 service call /bas/get_scenes std_srvs/srv/Trigger

# 健康检查
ros2 service call /bas/health_check std_srvs/srv/SetBool "{data: true}"
```

**停止系统测试示例**：

```bash
# 1. 启动系统并记录PID
ros2 run bas_control bas_control_node &
PID=$!

# 2. 调用停止服务
ros2 service call /bas/stop_system std_srvs/srv/Trigger

# 3. 观察日志输出
# [INFO] 正在停止视觉系统...
# [INFO] 系统停止：停止所有运行中的模块
# [INFO] 发现运行中的模块: bas_sys_config_ros (PID: 12345)
# [INFO] 发现运行中的模块: cam_mgr_ros (PID: 12346)
# ...停止模块...
# [INFO] 系统停止成功，即将退出进程
# [INFO] 系统已停止，退出进程...

# 4. 验证进程退出
sleep 1
ps -p $PID
# 应返回错误（进程已退出）
```

### 3. 状态监控

```bash
# 订阅系统状态话题（1 Hz）
ros2 topic echo /bas/system_status

# 订阅系统信息话题（0.2 Hz）
ros2 topic echo /bas/system_info
```

## API接口

### 服务接口

| 服务名称 | 类型 | 描述 |
|---------|------|------|
| `/bas/start_system` | `std_srvs/srv/Trigger` | 启动系统 |
| `/bas/stop_system` | `std_srvs/srv/Trigger` | 停止系统 |
| `/bas/get_status` | `std_srvs/srv/Trigger` | 获取系统状态 |
| `/bas/switch_scene` | `custom_msgs_comm/srv/SwitchScene` | 切换场景 |
| `/bas/get_scenes` | `std_srvs/srv/Trigger` | 获取可用场景列表 |
| `/bas/health_check` | `std_srvs/srv/SetBool` | 健康检查 |

### 话题接口

| 话题名称 | 类型 | 描述 | 频率 |
|---------|------|------|------|
| `/bas/system_status` | `std_msgs/msg/String` | 系统状态信息 | 1 Hz |
| `/bas/system_info` | `std_msgs/msg/String` | 系统信息 | 0.2 Hz |

## 预定义场景

### 场景类型枚举

系统采用**混合模式**设计场景管理，整合枚举类型安全与字符串扩展能力：

```cpp
enum class SceneType {
    UNKNOWN = 0,      // 未知场景
    IDLE = 1,         // 空闲场景 - 仅启动基础服务
    NAVIGATION = 2,   // 导航场景 - 目标检测+文字识别
    INTERACTION = 3,  // 交互场景 - 人脸+手势识别
    MANIPULATION = 4, // 操作场景 - 精确检测服务
    CALIBRATION = 5,  // 标定场景 - 手眼标定服务
    FULL = 6,         // 完整场景 - 全功能运行
    CUSTOM = 255      // 自定义场景标记 - 支持运行时扩展
};
```

### 场景信息结构体（SceneInfo）

```cpp
struct SceneInfo {
    SceneType type;           // 场景类型枚举
    std::string custom_name;  // 自定义场景名称（仅 type == CUSTOM 时有效）
    
    // 默认构造
    SceneInfo() : type(SceneType::UNKNOWN) {}
    
    // 从枚举类型构造（核心场景）
    explicit SceneInfo(SceneType scene_type) : type(scene_type) {}
    
    // 从字符串构造（自动识别核心/自定义）
    explicit SceneInfo(const std::string& name) 
        : type(stringToSceneType(name)), 
          custom_name(type == SceneType::CUSTOM ? name : "") {}
    
    // 获取场景名称
    std::string getName() const;
    
    // 检查场景类型
    bool isCoreScene() const;
    bool isValid() const;
    
    // 比较运算符（支持 std::map 等容器）
    bool operator==(const SceneInfo& other) const;
    bool operator<(const SceneInfo& other) const;
};
```

**使用示例**：
```cpp
// 核心场景（编译期类型安全）
SceneInfo nav_scene(SceneType::NAVIGATION);
std::cout << nav_scene.getName();      // "navigation"
std::cout << nav_scene.isCoreScene();  // true

// 从字符串自动识别
SceneInfo auto_scene("interaction");   // 自动识别为核心场景
SceneInfo custom("patrol");            // 自动识别为自定义场景

// 显式自定义场景
SceneInfo patrol("patrol", true);      // 显式标记为自定义
```

### 场景配置结构体（SceneConfig）

```cpp
struct SceneConfig {
    SceneType type;         // 场景类型枚举
    std::string name;       // 场景名称，如 "navigation", "interaction"
    std::vector<std::string> active_modules; // 该场景下应激活的模块
    std::map<std::string, std::map<std::string, std::string>> module_params; // 模块特定参数
    std::string description; // 场景描述
    
    // 默认构造
    SceneConfig() : type(SceneType::UNKNOWN) {}
    
    // 从字符串名称构造（自动识别核心/自定义场景）
    SceneConfig(const std::string& scene_name, 
                const std::vector<std::string>& modules)
        : type(stringToSceneType(scene_name)), 
          name(scene_name), 
          active_modules(modules) {}
    
    // 从枚举类型构造（核心场景）
    SceneConfig(SceneType scene_type, 
                const std::vector<std::string>& modules)
        : type(scene_type), 
          name(sceneTypeToString(scene_type)), 
          active_modules(modules) {}
    
    // 获取场景信息
    SceneInfo getSceneInfo() const {
        return (type == SceneType::CUSTOM) ? SceneInfo(name, true) : SceneInfo(type);
    }
    
    // 检查是否为核心场景
    bool isCoreScene() const {
        return type != SceneType::UNKNOWN && type != SceneType::CUSTOM;
    }
};
```

**注意**：内部场景映射使用 `std::map<SceneType, SceneConfig>` 存储场景配置，以 SceneType 枚举为键，而非字符串。

### 设计优势

| 优势 | 说明 |
|------|------|
| **类型安全** | 核心场景通过枚举比较，编译期检查拼写错误 |
| **高性能** | 枚举比较为整数操作，无需字符串比较开销 |
| **可扩展** | 支持从配置文件动态加载自定义场景 |
| **向后兼容** | 提供字符串转换接口，保持接口一致性 |

### 核心场景配置

| 场景名称 | 枚举值 | 激活模块 | 应用场景 |
|---------|--------|---------|---------|
| `idle` | `SceneType::IDLE` | bas_sys_config_ros, cam_mgr_ros | 系统空闲，仅基础服务 |
| `navigation` | `SceneType::NAVIGATION` | + yolo_det, ppocr | 自主导航，目标检测+文字识别 |
| `interaction` | `SceneType::INTERACTION` | + face_det, hand_gesture_rec | 人机交互，人脸+手势识别 |
| `manipulation` | `SceneType::MANIPULATION` | + yolo_det, yolo_obb_det | 机械臂操作，精确检测 |
| `calibration` | `SceneType::CALIBRATION` | + marker_detect_ros, hand_eye_calib_ros | 手眼标定，标识识别 |
| `full` | `SceneType::FULL` | 所有模块 | 全功能运行 |

## 测试验证

### 单元测试

```bash
# 运行所有单元测试
colcon test --packages-select bas_control

# 运行特定测试
colcon test --packages-select bas_control --ctest-args -R test_launch_mgr
colcon test --packages-select bas_control --ctest-args -R test_status_monitor
colcon test --packages-select bas_control --ctest-args -R test_task_scheduler
```

### 测试用例

1. **启动管理测试** (`test_launch_mgr.cpp`)
   - 模块注册/注销测试
   - 启动顺序验证
   - 依赖关系检查
   - 循环依赖检测

2. **状态监控测试** (`test_status_monitor.cpp`)
   - 状态更新测试
   - 资源监控测试
   - 健康检查测试

3. **任务调度测试** (`test_task_scheduler.cpp`)
   - 场景切换测试
   - 模块调度测试
   - 配置验证测试

## 日志系统

本系统集成了 `log_system` 日志框架，使用标准的日志宏：

```cpp
#include "log_system/log_macros.hpp"

// 不同级别的日志输出
LOG_DEBUG("调试信息: %s", debug_info);
LOG_INFO("系统启动完成");
LOG_WARN("警告信息: %d", warning_code);
LOG_ERROR("错误信息: %s", error_msg);
LOG_FATAL("致命错误: %s", fatal_msg);

// 带颜色的日志输出
LOG_INFO(false, logsys::Color::BLUE, "场景: %s", scene_name.c_str());
```

日志配置在 `bas_control.yaml` 的 `logging` 部分进行设置。

## 工具函数库

项目使用外部依赖包 `bas_operate` 提供的工具函数（`basmodule` 命名空间）：

```cpp
namespace basmodule {
    // 系统资源获取
    float get_cpu_usage();
    void get_memory_usage(uint64_t& used, uint64_t& total, float& usage_percent);
    float get_system_temperature();
    
    // 进程管理
    int launch_process(const std::string& path, ...);
    bool stop_process(int pid, bool force);
    bool is_process_alive(int pid);
    bool wait_for_process(int pid, int timeout_ms);
    
    // 依赖关系处理
    std::vector<std::string> topological_sort(...);
    bool has_circular_dependency(...);
    
    // 时间处理
    void sleep_ms(int milliseconds);
    int64_t duration_ms(...);
    std::string format_timestamp(...);
    std::string get_current_timestamp();
    
    // YAML配置
    template<typename T>
    T get_param_from_yaml(const YAML::Node& node, const std::string& key, const T& default_value);
}
```

## 故障排除

### 常见问题

1. **模块启动失败**
   - 检查模块依赖关系
   - 验证launch文件路径
   - 查看系统日志输出

2. **配置加载错误**
   - 验证YAML文件格式
   - 检查配置文件路径
   - 确认必需配置项
   
   **注意**: 从v1.0.0版本开始，配置文件路径通过 `ament_index_cpp` 自动解析。如果遇到"bad file"错误：
   - 确保已成功编译项目: `colcon build --packages-select bas_control`
   - 确保已source工作空间: `source install/setup.bash`

3. **资源监控异常**
   - 检查系统权限设置
   - 验证监控间隔配置
   - 确认资源阈值设置

4. **模块启动卡死/无响应**
   - **症状**: 启动模块时程序卡住，Ctrl+C无法终止
   - **原因**: 死锁问题 - 持有锁期间进行长时间操作
   - **解决方案**: 
     - 从v1.0.0版本开始，采用了**最小化锁范围**的设计方案
     - **关键原则**: 只在访问共享数据时加锁，不在持有锁时进行耗时操作
     - 创建 `updateModuleStatusInternal()` 内部方法（不加锁版本）
   - **验证**: 确保使用最新代码重新编译:
     ```bash
     colcon build --packages-select bas_control
     source install/setup.bash
     ```

### 调试方法

```bash
# 启用详细日志
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[%s] [%s] [%s]: %s"
export RCUTILS_LOG_LEVEL=DEBUG

# 运行节点并查看详细输出
ros2 run bas_control bas_control_node --ros-args --log-level debug
```

## 性能优化

### 系统调优建议

1. **启动优化**
   - 合理设置模块启动顺序
   - 优化依赖关系图
   - 调整启动超时参数

2. **资源管理**
   - 设置合理的资源限制
   - 配置适当的监控间隔
   - 优化内存使用策略

3. **日志配置**
   - 生产环境使用INFO级别
   - 开发环境可启用DEBUG级别
   - 定期清理日志文件

## 开发指南

### 代码规范

- 遵循Google C++ Style Guide
- 使用C++17标准特性
- 保持代码简洁和可读性
- 完善的注释和文档

### 扩展开发

1. **添加新模块**
   ```yaml
   # 在bas_control.yaml中添加
   modules:
     startup_order:
       - new_module_name
     dependencies:
       new_module_name: [cam_mgr_ros, bas_sys_config_ros]
   
   scenes:
     navigation:
       modules: [..., new_module_name]
   ```

2. **扩展场景**
   
   **方式一：配置文件添加（推荐）**
   ```yaml
   # 在bas_control.yaml中添加新场景
   scenes:
     patrol:
       modules: [bas_sys_config_ros, cam_mgr_ros, yolo_det]
       description: "巡逻场景，启动基础检测服务"
   ```
   配置文件中的场景会自动识别为核心场景（名称匹配）或自定义场景。

   **方式二：代码动态添加**
   ```cpp
   // 方式1：使用枚举类型（核心场景，编译期类型安全）
   SceneConfig core_scene(SceneType::NAVIGATION, 
                          {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det", "ppocr"});
   core_scene.description = "导航场景";
   task_scheduler_->addScene(core_scene);
   
   // 方式2：使用字符串名称（自动识别核心/自定义场景）
   SceneConfig auto_scene("patrol", 
                          {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det"});
   auto_scene.description = "自定义巡逻场景";
   // auto_scene.type == SceneType::CUSTOM（未识别的名称自动标记为自定义）
   task_scheduler_->addScene(auto_scene);
   
   // 方式3：显式指定自定义场景
   SceneConfig custom_scene("patrol", 
                            {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det"});
   custom_scene.type = SceneType::CUSTOM;  // 显式标记
   task_scheduler_->addScene(custom_scene);
   ```

3. **场景切换示例**
   ```cpp
   // 切换到核心场景（推荐使用枚举，编译期检查）
   SceneInfo nav_scene(SceneType::NAVIGATION);
   system_mgr_->switchScene(nav_scene.getName());
   
   // 切换到自定义场景
   SceneInfo patrol_scene("patrol", true);  // 显式指定为自定义场景
   system_mgr_->switchScene(patrol_scene.getName());
   
   // 从字符串自动识别
   SceneInfo auto_scene("interaction");  // 自动识别为核心场景
   system_mgr_->switchScene(auto_scene.getName());
   ```

### 回调注册示例

```cpp
// 系统状态回调
system_mgr_->registerSystemStatusCallback(
    [](const SystemStatus& status) {
        std::cout << "系统状态: " << status.getStatusString() << std::endl;
    });

// 场景切换回调
system_mgr_->registerSceneCallback(
    [](const std::string& scene_name, const std::vector<std::string>& modules) {
        std::cout << "场景切换到: " << scene_name << std::endl;
    });

// 模块状态回调
system_mgr_->registerModuleStatusCallback(
    [](const ModuleInfo& module_info) {
        std::cout << "模块 " << module_info.name 
                  << " 状态: " << module_info.getStatusString() << std::endl;
    });
```

## 功能列表

### 核心功能

#### 1. 场景切换智能健康检查

**功能描述**：
接收到切换场景指令后，系统会自动检查当前场景的模块健康状态，对未正常运行的模块执行自动启动。

**主要特性**：
- ✅ 场景相同性检查：判断目标场景是否与当前场景相同
- ✅ 模块健康检查：遍历当前场景的所有模块，检查运行状态
- ✅ 自动修复机制：对未正常运行的模块执行自动启动
- ✅ 结果反馈：记录并上报健康检查和修复结果

**使用场景**：
- 模块崩溃后自动重启
- 系统状态一致性维护
- 故障自动恢复

**测试示例**：
```bash
# 模拟模块故障
kill -9 <cam_mgr_ros_pid>

# 调用相同场景切换（触发健康检查）
ros2 service call /bas/switch_scene custom_msgs_comm/srv/SwitchScene "{scene_name: 'idle'}"

# 日志输出：
# [INFO] 已在当前场景中: idle，检查模块健康状态
# [WARN] 发现未运行模块: cam_mgr_ros，将尝试启动
# [INFO] 模块启动成功
```

---

#### 2. 系统停止退出进程

**功能描述**：
接收到停止指令后，系统会停止所有模块，清理资源，并自动退出 bas_control_node 进程。

**主要特性**：
- ✅ 停止当前场景下的所有模块
- ✅ 清理系统资源（状态监控、配置监控）
- ✅ 500ms延迟后自动退出进程
- ✅ 优雅停止（SIGTERM信号）

**使用场景**：
- 系统维护时一键停止
- 远程控制系统关闭
- 自动化部署脚本

**测试示例**：
```bash
# 停止系统并退出进程
ros2 service call /bas/stop_system std_srvs/srv/Trigger

# 日志输出：
# [INFO] 系统停止成功，即将退出进程
# [INFO] 系统已停止，退出进程...
```

---

#### 3. 默认启动场景配置化

**功能描述**：
支持通过配置文件指定默认启动场景，无需修改代码即可更改启动行为。

**主要特性**：
- ✅ 新增 `default_scene` 配置项
- ✅ 可选值：idle, navigation, interaction, manipulation, full
- ✅ 配置场景不存在时自动回退到 idle 场景

**配置示例**：
```yaml
system:
  default_scene: "navigation"  # 默认启动导航场景

scenes:
  navigation:
    modules: [bas_sys_config_ros, cam_mgr_ros, yolo_det, ppocr]
    description: "导航场景，启动检测和OCR服务"
```

**使用场景**：
- 开发调试：配置 `default_scene: "idle"` 最小化资源占用
- 生产部署：配置 `default_scene: "navigation"` 或其他业务场景

---

#### 4. 场景配置从配置文件加载

**功能描述**：
场景模块配置完全基于配置文件，支持动态添加和修改场景。

**主要特性**：
- ✅ 场景及其模块配置均在 `bas_control.yaml` 中定义
- ✅ 支持动态添加、修改场景配置
- ✅ 配置变更后重新启动即可生效

**配置示例**：
```yaml
scenes:
  idle:
    modules: [bas_sys_config_ros, cam_mgr_ros]
    description: "空闲场景，仅启动基础服务"
  
  navigation:
    modules: [bas_sys_config_ros, cam_mgr_ros, yolo_det, ppocr]
    description: "导航场景，启动检测和OCR服务"
  
  # 可自定义新场景
  patrol:
    modules: [bas_sys_config_ros, cam_mgr_ros, yolo_det]
    description: "巡逻场景，启动基础检测服务"
```

---

### 系统管理功能

#### 5. 动态查找可执行文件

**功能描述**：
无需配置模块路径，系统自动查找 ROS2 包的可执行文件。

**主要特性**：
- ✅ 新增 `find_ros_executable()` 和 `list_ros_executables()` 函数
- ✅ 自动从安装目录查找模块可执行文件
- ✅ 支持不同环境（开发、测试、生产）的自动适配

**优势**：
- 零配置：不需要在 YAML 文件中指定可执行路径
- 环境适配：自动适配不同环境
- 易维护：模块安装位置变更时无需修改配置

---

#### 6. 真实进程启动

**功能描述**：
替换模拟启动，实现真实进程管理。

**主要特性**：
- ✅ 使用 `fork()` + `execvp()` 启动进程
- ✅ 实时跟踪进程状态（PID、启动时间）
- ✅ 支持进程停止和重启功能
- ✅ 通过进程ID（PID）跟踪模块运行状态

**进程管理工具**：
- `launch_process()` - 启动进程（fork + execvp）
- `stop_process()` - 停止进程（SIGTERM/SIGKILL）
- `is_process_alive()` - 检查进程存活
- `wait_for_process()` - 等待进程退出

---

#### 7. 场景驱动启动

**功能描述**：
根据场景启动模块，而非启动所有模块，实现按需加载。

**主要特性**：
- ✅ 默认场景在 TaskScheduler 构造时自动设置（从配置文件读取）
- ✅ `startSystem()` 自动启动默认场景的模块
- ✅ 按需加载，节约系统资源

**资源优化效果**：
| 场景 | 启动模块数 | 预估CPU | 预估内存 |
|------|----------|--------|---------|
| idle | 2 | 10% | 500MB |
| navigation | 4 | 40% | 3GB |
| full | 9 | 80% | 7GB |

---

#### 8. 批量模块管理

**功能描述**：
新增批量启动/停止接口，简化模块管理。

**主要特性**：
- ✅ `startModules()` - 批量启动模块
- ✅ `stopModules()` - 批量停止模块
- ✅ `getModuleStartupOrder()` - 根据依赖关系计算启动顺序

**使用示例**：
```cpp
// 系统启动 - 启动所有模块
launch_mgr->startModules({});

// 场景切换 - 只启动指定模块
launch_mgr->startModules({"cam_mgr_ros", "yolo_det"});

// 系统停止 - 停止所有模块
launch_mgr->stopModules({});
```

---

### 场景管理功能

#### 9. 场景管理混合模式

**功能描述**：
整合枚举类型安全与字符串扩展能力，支持核心场景和自定义场景。

**主要特性**：
- ✅ 核心场景使用枚举类型，提供编译期类型安全
- ✅ 自定义场景使用字符串名称，支持运行时扩展
- ✅ 整合枚举类型安全与字符串扩展能力

**场景类型**：
```cpp
enum class SceneType {
    UNKNOWN = 0,      // 未知场景
    IDLE = 1,         // 空闲场景 - 仅启动基础服务
    NAVIGATION = 2,   // 导航场景 - 目标检测+文字识别
    INTERACTION = 3,  // 交互场景 - 人脸+手势识别
    MANIPULATION = 4, // 操作场景 - 精确检测服务
    CALIBRATION = 5,  // 标定场景 - 手眼标定服务
    FULL = 6,         // 完整场景 - 全功能运行
    CUSTOM = 255      // 自定义场景标记
};
```

**设计优势**：
| 优势 | 说明 |
|------|------|
| **类型安全** | 核心场景通过枚举比较，编译期检查拼写错误 |
| **高性能** | 枚举比较为整数操作，无需字符串比较开销 |
| **可扩展** | 支持从配置文件动态加载自定义场景 |
| **向后兼容** | 提供字符串转换接口，保持接口一致性 |

---

### 部署运维功能

#### 10. systemd 服务管理

**功能描述**：
提供 systemd 服务配置，适用于生产环境的系统级进程管理。

**systemd 服务配置** (`scripts/bas_control.service`)：
- ✅ 开机自启动
- ✅ 崩溃自动重启
- ✅ 日志集中管理（journal）
- ✅ 资源限制和优雅停止

**使用方法**：
```bash
# 安装服务
sudo cp scripts/bas_control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bas_control

# 管理服务
sudo systemctl start bas_control      # 启动
sudo systemctl stop bas_control       # 停止
sudo systemctl restart bas_control    # 重启
sudo systemctl status bas_control     # 查看状态
sudo journalctl -u bas_control -f     # 查看日志
```

**systemd 服务优势**：
| 优势 | 说明 |
|------|------|
| **开机自启动** | 系统启动时自动运行服务 |
| **崩溃自动重启** | Restart=on-failure 自动重启 |
| **日志集中管理** | 使用 journalctl 统一查看日志 |
| **资源限制** | LimitNOFILE、LimitNPROC 等限制 |
| **优雅停止** | KillSignal=SIGTERM，超时30秒 |

---

### 架构设计特性

#### 11. 配置文件路径自动解析

**功能描述**：
使用 `ament_index_cpp` 自动解析配置文件路径，无需手动指定绝对路径。

**主要特性**：
- ✅ 配置文件在编译时自动安装到正确位置
- ✅ 系统使用 `ament_index_cpp` 自动查找配置文件
- ✅ 默认路径: `<workspace>/install/bas_control/share/bas_control/config/bas_control.yaml`

---

#### 12. 死锁避免设计

**功能描述**：
采用最小化锁范围的设计方案，避免持有锁期间进行长时间操作。

**主要特性**：
- ✅ 创建 `updateModuleStatusInternal()` 不加锁版本
- ✅ 最小化锁范围：只在访问共享数据时加锁
- ✅ 不在持有锁时进行耗时操作
- ✅ 清晰的锁职责划分和详细注释
- ✅ 使用普通 `std::mutex` 获得更好性能

**关键原则**：
1. 只在访问共享数据时加锁
2. 不在持有锁时进行耗时操作（sleep、进程启动等）
3. 提供不加锁的内部方法供已持有锁的方法调用

---

## 外部依赖

| 依赖包 | 说明 |
|--------|------|
| `bas_operate` | 基础操作库（提供工具函数） |
| `log_system` | 日志系统框架 |
| `custom_msgs_comm` | 自定义消息通信 |
| `rclcpp` | ROS 2 C++ 客户端库 |
| `ament_index_cpp` | ament索引查找 |
| `yaml_cpp_vendor` | YAML解析库 |

---

## 模块状态订阅机制

### 概述

`bas_control_node` 启动时会自动订阅所有模块的状态话题，实时监控各模块的运行状态。支持多相机架构，自动为每个相机创建对应的模块状态订阅器。

### 系统相机个数读取

启动时自动从配置文件 `sys_cam_config.yaml` 读取相机个数：

```yaml
sys_cam_config:
  cam_num: 3  # 系统相机个数
```

读取流程：
1. 获取 install 目录路径
2. 构建配置文件完整路径
3. 使用 `SysConfig::SysConfigMgr::getSysCamNum()` 读取相机个数
4. 根据相机个数创建对应数量的订阅器

### 话题命名规范

#### 按相机区分的模块

对于需要按相机ID区分子节点的模块（如检测模块），话题命名格式为：
```
/cam{cam_id}/{module_name}/mdl_status_info
```

示例（系统相机个数为3时）：
- `/cam0/cam_mgr_ros/mdl_status_info` - 相机0的相机管理模块状态
- `/cam1/marker_detect_ros/mdl_status_info` - 相机1的标记检测模块状态
- `/cam2/yolo_det/mdl_status_info` - 相机2的YOLO检测模块状态

#### 系统级模块

对于不需要按相机区分的系统级模块，话题命名格式为：
```
/{module_name}/mdl_status_info
```

示例：
- `/bas_sys_config_ros/mdl_status_info` - 系统配置模块状态

### 数据结构

所有模块状态话题使用统一的数据结构 `basros::ModuleStatusInfo`，定义在 `bas_operate_ros/module_status.hpp`：

```cpp
struct ModuleStatusInfo {
    std::string module_name;          ///< 模块名称
    int cam_id;                       ///< 相机ID（-1表示主节点）
    ModuleStatus status;              ///< 当前状态
    std::string status_msg;           ///< 状态信息文本
};
```

### 状态枚举

```cpp
enum class ModuleStatus {
    UNKNOWN,        ///< 未知状态
    STOPPED,        ///< 已停止
    STARTING,       ///< 启动中
    RUNNING,        ///< 正常运行
    RUNNING_PAUSED, ///< 业务暂停
    RELOADING,      ///< 热加载中
    STOPPING,       ///< 停止中
    ERROR,          ///< 错误
    CRASHED         ///< 崩溃
};
```

### JSON消息格式

话题使用 `std_msgs::msg::String` 类型，消息内容为JSON格式：

```json
{
  "module_name": "yolo_det",
  "cam_id": 0,
  "status": "RUNNING",
  "status_msg": "正常工作中"
}
```

### 订阅器数量计算

```
总订阅器数量 = (按相机模块数 × 相机个数) + 系统级模块数
```

示例计算：
- 按相机区分的模块数量：8个
- 系统相机个数：3个
- 系统级模块数量：1个

则：
```
总订阅器数量 = (8 × 3) + 1 = 25个订阅器
```

### 模块状态发布指南

各模块需要实现状态发布功能：

```cpp
#include "bas_operate_ros/module_status.hpp"

// 创建状态信息
basros::ModuleStatusInfo status_info;
status_info.module_name = "yolo_det";
status_info.cam_id = cam_id_;  // 子节点使用相机 ID，主节点使用 -1
status_info.status = basros::ModuleStatus::RUNNING;
status_info.status_msg = "正常工作中";

// 转换为 JSON 并发布
auto msg = std_msgs::msg::String();
msg.data = basros::moduleStatusInfoToJson(status_info);
status_publisher_->publish(msg);
```

### 为什么需要 JSON 序列化？

#### 设计原因

**1. 跨模块标准化通信**
- `bas_control` 作为统一部署控制层，需要监控所有视觉模块的状态
- 不同模块可能由不同语言实现（C++、Python），JSON 是通用标准
- JSON 格式自描述性强，无需额外的接口定义文档

**2. 解耦合设计**
- 发送方（业务模块）和接收方（bas_control）不需要知道彼此的实现细节
- 未来可能有 Python、Java 编写的模块，JSON 确保跨语言兼容性
- 支持异构系统集成，便于系统扩展

**3. 可读性与调试便利**
```bash
# 运维人员可以直接查看原始消息
ros2 topic echo /cam0/yolo_det/mdl_status_info
# 输出：{module_name: "yolo_det", cam_id: 0, status: "RUNNING", ...}

# 日志文件中直接记录 JSON，便于故障诊断
[INFO] 收到模块状态：{"module_name":"yolo_det","cam_id":0,"status":"RUNNING"}
```

**4. Web 界面友好**
- 前端 JavaScript 可以直接解析 JSON
- 便于构建可视化的系统监控界面
- 支持 RESTful API 集成

#### JSON 消息格式

```json
{
  "module_name": "marker_detect_ros",
  "cam_id": 0,
  "status": "RUNNING",
  "status_msg": "正常工作中"
}
```

**字段说明**：
- `module_name` (string): 模块唯一标识名称
- `cam_id` (int): 相机 ID（-1 表示主节点，>=0 表示子节点）
- `status` (string): 模块状态枚举字符串表示
  - 可选值：UNKNOWN, STOPPED, STARTING, RUNNING, RUNNING_PAUSED, RELOADING, STOPPING, ERROR, CRASHED
- `status_msg` (string): 人类可读的状态描述文本

#### 实现机制

**序列化函数** (`bas_operate_ros/module_status.hpp`):
```cpp
inline std::string moduleStatusInfoToJson(const ModuleStatusInfo& info) {
    std::ostringstream oss;
    oss << "{";
    oss << "\"module_name\":\"" << info.module_name << "\",";
    oss << "\"cam_id\":" << info.cam_id << ",";
    oss << "\"status\":\"" << moduleStatusToString(info.status) << "\",";
    oss << "\"status_msg\":\"" << info.status_msg << "\"";
    oss << "}";
    return oss.str();
}
```

**反序列化函数**:
```cpp
inline ModuleStatusInfo jsonToModuleStatusInfo(const std::string& json_str) {
    // 解析 JSON 字符串并返回 ModuleStatusInfo 对象
    // 支持从订阅的消息中恢复模块状态信息
}
```

**优势总结**：
- ✅ **零依赖**：不依赖外部 JSON 库（如 nlohmann/json）
- ✅ **轻量级**：纯手工实现，代码量小
- ✅ **高性能**：无额外内存分配，直接使用 ostringstream
- ✅ **易维护**：代码透明，易于调试和扩展

### 话题名称构建

**子节点（按相机区分）**：
```cpp
std::string topic_name = "/cam" + std::to_string(cam_id_) + "/" + module_name_ + "/mdl_status_info";
```

**主节点（系统级）**：
```cpp
std::string topic_name = "/" + module_name_ + "/mdl_status_info";
```

### 状态刷新周期

建议状态刷新周期：
- **正常情况**：1秒（默认）
- **高频场景**：500毫秒
- **低频场景**：2-5秒

---

## 项目版本

**当前版本**：v1.4.0

本项目采用语义化版本管理，版本格式为：主版本号.次版本号.修订号

---

## 许可证

本项目采用 Apache License 2.0 许可证。

## 联系方式

- 维护者: Developer <developer@example.com>
- 项目主页: <repository_url>
