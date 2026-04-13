# comm_alg

通用算法模块，包含共用的数据结构和工具函数。

## 目录结构

```
comm_alg/
├── CMakeLists.txt
├── include/
│   └── comm_alg/
│       └── comm_structs.hpp
└── src/
```

## 功能特点

- 提供共用的数据结构，如MarkerInfo、CameraIntrinsics、DetectionResult等
- 支持不同算法模块之间的数据交换
- 统一的接口定义，便于扩展和维护

## 安装依赖

- OpenCV 4.0+

## 构建方法

```bash
cd /path/to/workspace
colcon build --packages-select comm_alg
```

## 使用示例

在其他模块的CMakeLists.txt中添加依赖：

```cmake
find_package(comm_alg REQUIRED)

ament_target_dependencies(${PROJECT_NAME} 
  "comm_alg"
  # 其他依赖...
)
```

在代码中使用：

```cpp
#include "comm_alg/comm_structs.hpp"

// 使用共用的数据结构
comm_alg::DetectionResult result;
```