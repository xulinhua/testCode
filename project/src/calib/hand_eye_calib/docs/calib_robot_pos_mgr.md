# 机械臂坐标数据管理器使用说明

## 功能概述

机械臂坐标数据管理器([CalibRobotPosMgr](file:///192.168.11.129/testCode/project/src/calib/hand_eye_calib/include/hand_eye_calib/calib_robot_pos_mgr.hpp#L26-L84))是hand_eye_calib项目的一个子功能模块，用于生成和管理手眼标定时所需的机械臂坐标数据点。该模块参照了`eye2hand_test/nova_data_collector.py`中第58行到第75行的实现。

## 主要特性

1. 根据标准位置生成一系列机械臂坐标数据点
2. 支持自定义标准位置
3. 提供获取指定索引数据点的功能
4. 符合手眼标定的精度要求

## 坐标点生成规则

生成的坐标点基于以下规则：

1. XY平面偏移点：
   - 中心点：(0, 0)
   - 四角点：(±50mm, ±50mm)
   - 边中点：(±50mm, 0) 和 (0, ±50mm)
   - 内圈点：(±30mm, ±30mm)
   - 内圈边中点：(±30mm, 0) 和 (0, ±30mm)

2. Z轴偏移值：
   - 范围：±150mm
   - 间距：30mm
   - 具体值：[-150, -90, -60, -30, 0, 30]

## 使用方法

### 在代码中使用

```cpp
#include "hand_eye_calib/calib_robot_pos_mgr.hpp"

using namespace handeyecalib;

// 创建管理器实例
CalibRobotPosMgr mgr;

// 设置标准位置
mgr.setStandardPose(100.0, 200.0, 300.0, 0.0, 0.0, 0.0);

// 生成所有数据点
std::vector<RobotPoseData> data_points = mgr.generateDataPoints();

// 获取特定索引的数据点
RobotPoseData point = mgr.getDataPoint(10);
```

### 通过命令行使用

```bash
ros2 run hand_eye_calib test_calib_robot_pos_mgr
```

## 配置参数

项目使用[YAML](../config/calib_config.yaml)格式的配置文件来管理标定参数。配置文件位于[config/calib_config.yaml](../config/calib_config.yaml)。

### 相关配置参数

- `calibration.min_points`: 进行生成手眼标定矩阵的最小标定点个数，标定点总数低于此数值时，标定生成矩阵将无法进行
- `calibration.data_save_dir`: 标定数据保存目录
- `calibration.result_save_dir`: 标定结果保存目录

## 数据结构

### RobotPoseData

```cpp
struct RobotPoseData {
    double x, y, z;      // 位置坐标 (mm)
    double rx, ry, rz;   // 姿态角 (degrees)
};
```

## API参考

### CalibRobotPosMgr

#### 构造函数
```cpp
CalibRobotPosMgr();
```

#### 析构函数
```cpp
~CalibRobotPosMgr();
```

#### generateDataPoints
生成数据采集点列表
```cpp
std::vector<RobotPoseData> generateDataPoints() const;
```

#### getDataPoint
获取指定索引的数据点
```cpp
RobotPoseData getDataPoint(std::size_t index) const;
```

#### getPointCount
获取数据点数量
```cpp
std::size_t getPointCount() const;
```

#### setStandardPose
设置标准位置
```cpp
void setStandardPose(double x, double y, double z, double rx, double ry, double rz);
```

#### getStandardPose
获取标准位置
```cpp
RobotPoseData getStandardPose() const;
```