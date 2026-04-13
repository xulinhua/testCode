# 手眼标定项目 (hand_eye_calib)

## 项目概述

本项目是一个用于具身智能机器人的手眼标定系统，可在Ubuntu 22.04 + ROS Humble环境下运行。项目实现了完整的手眼标定功能，包括数据采集、标定矩阵生成、结果应用和质量分析等模块。

## 项目特点

1. **独立性**: 不依赖ROS环境，可在纯C++环境下编译和运行
2. **模块化设计**: 各功能模块解耦，便于维护和扩展
3. **多模式支持**: 支持眼在手外和眼在手上的标定模式
4. **实时可视化**: 提供实时相机采图显示和Aruco码识别结果显示
5. **质量分析**: 提供标定精度验证和分析功能

## 项目架构

```
graph TB
    A[主程序] --> B[数据采集模块]
    A --> C[标定矩阵生成模块]
    A --> D[标定结果应用模块]
    A --> E[质量分析模块]
    A --> F[可视化模块]
    A --> G[机械臂坐标数据管理模块]
    
    B --> H[Aruco识别模块]
    B --> I[相机SDK模块]
    
    C --> J[矩阵计算]
    D --> K[坐标变换]
    E --> L[精度分析]
    F --> M[图像显示]
    G --> N[坐标点管理]
```

## 功能模块

### 1. 数据采集模块 (CalibDataCollector)
- 实时采集Aruco码位置数据和机械臂位姿数据
- 支持加载本地保存的标定数据
- 提供数据点管理功能
- 支持外部模块调用接口设置机械臂位姿并触发数据采集

### 2. 标定矩阵生成模块 (gen_calib_matrix)
- 计算眼在手外标定矩阵
- 计算眼在手上标定矩阵
- 支持标定结果保存和加载

### 3. 标定结果应用模块 (calib_result_apply)
- 基于标定结果反推机械手实际6D位姿
- 支持坐标变换计算

### 4. 质量分析模块 (calib_quality_analysis)
- 标定精度验证分析
- 重投影误差计算
- 矩阵条件数分析

### 5. 可视化模块 (VisualizationManager)
- 实时相机图像显示
- Aruco码识别结果显示
- 坐标轴绘制

### 6. 机械臂坐标数据管理模块 (CalibRobotPosMgr)
- 生成和管理手眼标定时所需的机械臂坐标数据点
- 支持自定义标准位置
- 提供获取指定索引数据点的功能
- 参照`eye2hand_test/nova_data_collector.py`中第58行到第75行的实现

## 项目结构

```
hand_eye_calib/
├── CMakeLists.txt                 # CMake构建配置
├── package.xml                    # ROS包描述文件
├── README.md                      # 项目说明文档
├── include/hand_eye_calib/        # 头文件目录
│   ├── calib_data_collector.hpp    # 数据采集器头文件
│   ├── calib_robot_pos_mgr.hpp    # 机械臂坐标数据管理器头文件
│   ├── calib_config.hpp           # 配置管理器头文件
│   └── calib_utils.hpp             # 工具函数头文件（包含所有功能）
├── src/                           # 源文件目录
│   ├── main.cpp                   # 主程序
│   ├── calib_data_collector.cpp   # 数据采集器实现
│   ├── calib_robot_pos_mgr.cpp   # 机械臂坐标数据管理器实现
│   ├── calib_config.cpp           # 配置管理器实现
│   └── calib_utils.cpp             # 工具函数实现（包含所有功能）
├── test/                          # 测试文件目录
│   ├── test_calibration.cpp       # 核心算法测试程序
│   ├── test_data_collector.cpp    # 数据采集模块测试程序
│   ├── test_calib_matrix.cpp      # 标定矩阵生成模块测试程序
│   ├── test_calib_application.cpp # 标定结果应用模块测试程序
│   ├── test_calib_analysis.cpp    # 标定质量分析模块测试程序
│   └── test_calib_robot_pos_mgr.cpp  # 机械臂坐标数据管理器测试
├── docs/                          # 文档目录
│   └── calib_robot_pos_mgr.md    # 机械臂坐标数据管理器使用说明
└── config/                        # 配置文件目录
  └── calib_config.yaml            # 标定配置文件
```

## 依赖项

- OpenCV 4.x
- Eigen3
- nlohmann/json
- C++14 或更高版本

## 编译方法

### 在ROS环境下编译

```bash
# 进入ROS工作空间
cd ~/ros2_ws

# 将项目复制到src目录
cp -r /path/to/calib/hand_eye_calib src/

# 编译
colcon build --packages-select hand_eye_calib

# 设置环境变量
source install/setup.bash
```

### 独立编译（非ROS环境）

```bash
# 进入项目目录
cd calib/hand_eye_calib

# 创建构建目录
mkdir build && cd build

# 配置项目
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make

# 运行
./hand_eye_calib --help
```

### 算法说明

本模块采用了与Python版本相同的算法实现，使用伪逆方法计算变换矩阵，避免了原C++版本中由于欠定系统导致的求解失败问题。

算法核心公式：
```
T_cam2base = base_coords * pinv(cam_coords)
```

其中：
- `base_coords`: 机器人基座坐标系下的标记位置 (4×n)
- `cam_coords`: 相机坐标系下的标记位置 (4×n)
- `pinv`: 矩阵伪逆运算

## 使用方法

### 方式一：非ROS环境下的直接运行

### 1. 数据采集模式

```bash
./test_data_collector
```

此模式用于采集标定所需的数据点，包括机械臂位姿和Aruco码位置。

数据采集功能会从配置文件中指定的源数据目录加载已有的标定点数据，
并将新采集的数据保存到配置文件中指定的输出数据目录。

- 源数据目录：`sys_config/source_calib_data`
- 输出数据目录：`output_calib_data`

在源数据目录中，coordinates 子目录存放源标定点数据，src_images 子目录存放源标定时的源图，
aruco_rendered 子目录存放源标定时的渲染图，calibration_result.json 为源标定结果，
base2camera.npy 和 camera2base.npy 分别是源标定转换矩阵。

在输出数据目录中，coordinates 子目录存放新采集并保存的标定点数据，src_images 子目录存放标定时的源图，
aruco_rendered 子目录存放标定时的渲染图，calibration_result.json 为标定结果，
base2camera.npy 和 camera2base.npy 分别是标定转换矩阵。

### 2. 标定矩阵生成模式

```bash
./test_calib_matrix
```

此模式根据采集的数据计算手眼标定矩阵，并保存结果。
注意：此模式需要先运行数据采集模式并保存足够的标定数据点。

### 3. 标定结果应用模式

```bash
./test_calib_application
```

此模式用于应用已生成的标定结果，计算机械手实际位姿。
注意：此模式需要先运行标定矩阵生成模式并生成标定结果。

### 4. 标定质量分析模式

```bash
./test_calib_analysis
```

此模式用于分析标定结果的质量，包括精度评估。
注意：此模式需要先运行标定矩阵生成模式并生成标定结果。

### 5. 机械臂坐标数据管理模式

```bash
./test_calib_robot_pos_mgr
```

此模式用于生成和管理手眼标定时所需的机械臂坐标数据点，参照`eye2hand_test/nova_data_collector.py`中第58行到第75行的实现。

### 运行标定测试程序

项目包含多个专门的测试程序，用于验证各个模块的功能正确性：

1. **核心算法测试程序**：[test_calibration.cpp](test_calibration.cpp)
   用于验证手眼标定算法的正确性。

2. **数据采集模块测试程序**：[test_data_collector.cpp](test/test_data_collector.cpp)
   用于测试数据采集模块的功能。这是执行数据采集的主要程序。

3. **标定矩阵生成模块测试程序**：[test_calib_matrix.cpp](test/test_calib_matrix.cpp)
   用于测试标定矩阵生成模块的功能。这是执行标定矩阵生成的主要程序。

4. **标定结果应用模块测试程序**：[test_calib_application.cpp](test/test_calib_application.cpp)
   用于测试标定结果应用模块的功能。这是执行标定结果应用的主要程序。

5. **标定质量分析模块测试程序**：[test_calib_analysis.cpp](test/test_calib_analysis.cpp)
   用于测试标定质量分析模块的功能。这是执行标定质量分析的主要程序。

6. **机械臂坐标数据管理模块测试程序**：[test_calib_robot_pos_mgr.cpp](test/test_calib_robot_pos_mgr.cpp)
   用于测试机械臂坐标数据管理模块的功能。这是执行机械臂坐标数据管理的主要程序。

编译后，可以通过以下方式运行各个测试程序：

```bash
# 进入构建目录
cd build

# 运行核心算法测试程序
./test_calibration

# 运行数据采集模块测试程序
./test_data_collector

# 运行标定矩阵生成模块测试程序
./test_calib_matrix

# 运行标定结果应用模块测试程序
./test_calib_application

# 运行标定质量分析模块测试程序
./test_calib_analysis

# 运行机械臂坐标数据管理模块测试程序
./test_calib_robot_pos_mgr
```

或者在ROS 2环境中使用以下命令运行：

```
# 构建项目
colcon build --packages-select hand_eye_calib

# source工作空间
source install/setup.bash

# 运行数据采集模块测试程序
ros2 run hand_eye_calib test_data_collector

# 运行标定矩阵生成模块测试程序
ros2 run hand_eye_calib test_calib_matrix

# 运行标定结果应用模块测试程序
ros2 run hand_eye_calib test_calib_application

# 运行标定质量分析模块测试程序
ros2 run hand_eye_calib test_calib_analysis

# 运行机械臂坐标数据管理模块测试程序
ros2 run hand_eye_calib test_calib_robot_pos_mgr
```

或者直接执行：
```
./build/hand_eye_calib/test_calibration
./build/hand_eye_calib/test_data_collector
./build/hand_eye_calib/test_calib_matrix
./build/hand_eye_calib/test_calib_application
./build/hand_eye_calib/test_calib_analysis
./build/hand_eye_calib/test_calib_robot_pos_mgr
```

各个测试程序将使用预设的测试数据执行相应的功能测试。如果实现正确，程序将显示测试结果并输出相关信息。

### 方式二：ROS Humble环境下的运行

在ROS Humble环境下，请使用以下方式运行：

1. 首先构建项目：
```bash
# 返回到ROS工作空间根目录
cd /path/to/your/ros2_workspace

# 构建hand_eye_calib包
colcon build --packages-select hand_eye_calib

# source工作空间
source install/setup.bash
```

2. 运行各模式：

#### 2.1 数据采集模式

```
ros2 run hand_eye_calib test_data_collector
```

此模式用于采集标定所需的数据点，包括机械臂位姿和Aruco码位置。
注意：此模式需要连接相机设备并确保机械臂可以接收位姿数据。

数据采集功能会从配置文件中指定的源数据目录加载已有的标定点数据，
并将新采集的数据保存到配置文件中指定的输出数据目录。

- 源数据目录：`sys_config/source_calib_data`
- 输出数据目录：`output_calib_data`

在源数据目录中，coordinates 子目录存放源标定点数据。
在输出数据目录中，coordinates 子目录存放新采集并保存的标定点数据。

#### 2.2 标定矩阵生成模式

```
ros2 run hand_eye_calib test_calib_matrix
```

此模式根据采集的数据计算手眼标定矩阵，并保存结果。
注意：此模式需要先运行数据采集模式并保存足够的标定数据点。

#### 2.3 标定结果应用模式

```
ros2 run hand_eye_calib test_calib_application
```

此模式用于应用已生成的标定结果，计算机械手实际位姿。
注意：此模式需要先运行标定矩阵生成模式并生成标定结果。

#### 2.4 标定质量分析模式

```
ros2 run hand_eye_calib test_calib_analysis
```

此模式用于分析标定结果的质量，包括精度评估。
注意：此模式需要先运行标定矩阵生成模式并生成标定结果。

#### 2.5 机械臂坐标数据管理模式

```
ros2 run hand_eye_calib test_calib_robot_pos_mgr
```

此模式用于生成和管理手眼标定时所需的机械臂坐标数据点，参照`eye2hand_test/nova_data_collector.py`中第58行到第75行的实现。

#### 2.6 显示帮助信息

```
ros2 run hand_eye_calib hand_eye_calib --help
```

注意：在ROS环境下运行时，确保已正确source工作空间环境变量。

## API接口说明

### CalibDataCollector 类

数据采集模块的核心类，提供了以下主要接口：

#### addCalibrationPoint()
添加标定点数据。

```cpp
bool addCalibrationPoint(const std::vector<double>& robot_pose, 
                        const std::vector<double>& marker_position);
```

参数：
- `robot_pose`: 6维机械臂位姿向量 [x, y, z, rx, ry, rz]
- `marker_position`: 3维标记位置向量 [x, y, z]

返回值：
- `true`: 成功添加标定点
- `false`: 添加失败

#### setCalibrationPoint()
设置指定索引的标定点数据。

```cpp
bool setCalibrationPoint(int index,
                        const std::vector<double>& robot_pose,
                        const std::vector<double>& marker_position);
```

参数：
- `index`: 标定点索引
- `robot_pose`: 6维机械臂位姿向量 [x, y, z, rx, ry, rz]
- `marker_position`: 3维标记位置向量 [x, y, z]

返回值：
- `true`: 成功设置标定点
- `false`: 设置失败

#### getCalibrationData()
获取标定计算所需的数据格式。

```cpp
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> 
getCalibrationData() const;
```

返回值：
- `pair<robot_poses, marker_positions>`：第一个元素是机器人位姿列表，第二个元素是标记位置列表

#### getCalibrationPoints()
获取所有标定点数据。

```cpp
const std::vector<CalibrationPoint>& getCalibrationPoints() const;
```

返回值：
- 标定点数据列表

#### getCalibrationPoint()
获取指定索引的标定点数据。

```cpp
const CalibrationPoint* getCalibrationPoint(int index) const;
```

参数：
- `index`: 标定点索引

返回值：
- 标定点数据指针，如果不存在则返回nullptr

#### saveCalibrationData()
保存标定数据到文件。

```cpp
bool saveCalibrationData(const std::string& save_dir);
```

参数：
- `save_dir`: 保存目录路径

返回值：
- `true`: 保存成功
- `false`: 保存失败

#### loadCalibrationData()
加载标定数据。

```cpp
bool loadCalibrationData(const std::string& data_dir);
```

参数：
- `data_dir`: 数据目录路径

返回值：
- `true`: 加载成功
- `false`: 加载失败

更多API接口请参考 [calib_data_collector.hpp](include/hand_eye_calib/calib_data_collector.hpp) 头文件。

## 配置文件说明

项目使用[YAML](config/calib_config.yaml)格式的配置文件来管理标定参数。配置文件位于[config/calib_config.yaml](config/calib_config.yaml)。

### 配置参数

- `calibration.min_points`: 进行生成手眼标定矩阵的最小标定点个数，标定点总数低于此数值时，标定生成矩阵将无法进行
- `calibration.data_save_dir`: 标定数据保存目录
- `calibration.result_save_dir`: 标定结果保存目录

## 数据格式

### 标定点数据格式

```
{
  "index": 1,
  "timestamp": "2025-11-19T14:18:06",
  "robot_position": {
    "x": -117.1344,
    "y": -344.7371,
    "z": 65.8233,
    "rx": -179.9815,
    "ry": 0.0051,
    "rz": 90.0711
  },
  "marker_position": {
    "x": 0.1548,
    "y": -0.1371,
    "z": 0.8673
  },
  "robot_pose": {
    "x": -117.1344,
    "y": -344.7371,
    "z": 65.8233,
    "rx": -179.9815,
    "ry": 0.0051,
    "rz": 90.0711
  }
}
```

### 标定结果格式

```
{
  "camera2base": {
    "matrix": [[...], [...], [...], [...]],
    "pose": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "rx": 0.0,
      "ry": 0.0,
      "rz": 0.0
    }
  },
  "base2camera": {
    "matrix": [[...], [...], [...], [...]],
    "pose": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "rx": 0.0,
      "ry": 0.0,
      "rz": 0.0
    }
  }
}
```

## 注意事项

1. 确保系统已安装所有依赖项
2. 标定过程中需要保持环境光照稳定
3. 建议使用高对比度的Aruco码标记
4. 采集数据时应覆盖机械臂工作空间的各个区域
5. 至少需要4个标定点才能进行标定计算

## 故障排除

### 编译问题

1. **找不到OpenCV**: 确保已安装OpenCV开发包
   ```bash
   sudo apt install libopencv-dev
   ```

2. **找不到Eigen3**: 确保已安装Eigen3开发包
   ```bash
   sudo apt install libeigen3-dev
   ```

### 运行问题

1. **相机无法初始化**: 检查相机连接和权限
2. **Aruco码无法识别**: 检查标记清晰度和光照条件
3. **标定精度低**: 增加标定点数量，改善采集条件

## 许可证

本项目采用Apache License 2.0许可证，详见 [LICENSE](LICENSE) 文件。

## 联系方式

如有问题或建议，请联系项目维护者。