# Stereo Calibration Module (stereo_calib)

双目相机标定模块，支持立体标定、视差图计算、深度图生成和3D重建功能。

## 功能特性

- **双目标定**: 使用棋盘格进行立体相机标定，同时优化内外参
- **图像校正**: 立体校正消除畸变和行对齐
- **视差图计算**: 支持BM和SGBM两种立体匹配算法
- **深度图生成**: 根据视差图计算深度信息
- **3D重建**: 生成点云数据，支持单点3D坐标投影
- **配置文件支持**: 通过YAML文件配置所有参数和路径
- **无头模式**: 支持无图形界面环境运行

## 目录结构

```
stereo_calib/
├── CMakeLists.txt                        # 构建配置
├── package.xml                           # ROS包配置
├── README.md                             # 说明文档
├── src/
│   ├── include/stereo_calib/
│   │   └── stereo_calib.hpp              # 头文件
│   ├── src/
│   │   └── stereo_calib.cpp              # 实现文件
│   ├── test/
│   │   ├── example_stereo_calib.cpp      # 完整示例程序(交互式)
│   │   ├── stereo_calibrate.cpp          # 仅标定程序
│   │   ├── stereo_calib_test.cpp         # 标定+处理测试程序
│   │   └── stereo_process.cpp            # 批量处理程序
│   └── config/
│       └── stereo_calib_params.yaml      # 配置文件
├── data/
│   ├── left/                             # 左相机图像文件夹
│   └── right/                            # 右相机图像文件夹
└── config/                               # 标定结果保存目录
```

## 可执行程序

| 程序名 | 功能 | 适用场景 |
|--------|------|----------|
| `stereo_calibrate` | 仅执行双目标定 | 只需要标定结果 |
| `stereo_calib_test` | 标定+批量处理+保存结果 | 测试和验证 |
| `stereo_calib_example` | 交互式示例(需GUI) | 本地调试 |
| `stereo_process` | 使用已有标定结果处理图像 | 生产环境 |

## 快速开始

### 1. 准备标定图像

将左右相机拍摄的棋盘格图像分别放入对应文件夹：

```
data/left/   - 左相机图像 (如 left_0.jpg, left_1.jpg, ...)
data/right/  - 右相机图像 (如 right_0.jpg, right_1.jpg, ...)
```

**注意**: 
- 左右图像需要成对拍摄，文件名按顺序排列
- 至少需要10-20对不同角度的图像
- 棋盘格应覆盖图像不同区域

### 2. 配置参数

编辑 `src/config/stereo_calib_params.yaml`:

```yaml
stereo_calib_parameters:
  # 棋盘格参数
  board_size: [10, 7]           # 棋盘格内角点数量 (宽 x 高)
  square_size: 0.024125         # 方格边长 (米)
  image_width: 1280             # 图像宽度
  image_height: 720             # 图像高度
  
  # 图像路径
  left_image_folder: "/home/user/code/Dev/src/calib/stereo_calib/data/left"
  right_image_folder: "/home/user/code/Dev/src/calib/stereo_calib/data/right"
  
  # 标定结果保存路径
  calibration_file: "/home/user/code/Dev/src/calib/stereo_calib/config/stereo_calib_result.yaml"
  
  # 输出路径
  output_folder: "/home/user/code/Dev/output/stereo_calib"
  
  # 视差计算参数
  disparity_params:
    num_disparities: 64         # 视差数量 (需为16的倍数)
    block_size: 11              # 匹配块大小 (奇数, 5-21)
    pre_filter_cap: 31          # 预滤波截断值
    min_disparity: 0            # 最小视差
    texture_threshold: 10       # 纹理阈值
    uniqueness_ratio: 15        # 唯一性比率
    speckle_window_size: 100    # 斑点滤波窗口大小
    speckle_range: 32           # 斑点滤波范围
    disp12_max_diff: 1          # 左右一致性检查阈值
    use_sgbm: true              # true: SGBM算法, false: BM算法
```

### 3. 编译

```bash
cd /home/user/code/Dev
colcon build --packages-select stereo_calib
source install/setup.bash
```

### 4. 运行

#### 方式一：仅标定

```bash
ros2 run stereo_calib stereo_calibrate
```

#### 方式二：标定+批量处理（推荐）

```bash
ros2 run stereo_calib stereo_calib_test
```

输出结果：
```
/home/user/code/Dev/output/stereo_calib/
├── rectified_left/     # 校正后的左图
├── rectified_right/    # 校正后的右图
├── disparity/          # 视差图
├── depth/              # 深度图
└── pointcloud/         # 点云(PLY格式)
```

#### 方式三：使用已有标定结果处理图像

```bash
# 使用默认配置文件
ros2 run stereo_calib stereo_process

# 指定配置文件
ros2 run stereo_calib stereo_process --config /path/to/params.yaml

# 覆盖部分参数
ros2 run stereo_calib stereo_process --config params.yaml --output ./my_output

# 处理单张图像
ros2 run stereo_calib stereo_process \
  --calib calib.yaml \
  --left-img left.png \
  --right-img right.png

# 完整手动指定
ros2 run stereo_calib stereo_process \
  --calib /path/to/calib.yaml \
  --left /path/to/left/ \
  --right /path/to/right/ \
  --output /path/to/output/

# 查看帮助
ros2 run stereo_calib stereo_process --help
```

#### 方式四：交互式示例（需GUI）

```bash
# 正常模式
ros2 run stereo_calib stereo_calib_example

# 无头模式（服务器/SSH环境）
ros2 run stereo_calib stereo_calib_example --headless
```

## 标定质量评估

### 正常标定结果示例

```
=== Stereo Calibration Info ===
Image Size: 1280 x 720
Board Size: 10 x 7
Square Size: 0.024125 m
Baseline: 0.0601234 m
Calibrated: Yes

Left Camera Matrix:
[1205.6, 0, 642.3;
 0, 1205.8, 361.2;
 0, 0, 1]

Left Distortion Coefficients:
[-0.02, 0.01, 0, 0, 0]

重投影误差: 0.35 pixels
```

### 质量指标

| 指标 | 正常范围 | 异常值可能原因 |
|------|----------|----------------|
| 重投影误差 | < 1.0 像素 | 棋盘格检测不准确、图像模糊 |
| 焦距 (fx, fy) | 合理值（如800-2000） | 内参优化失败 |
| 畸变系数 | 绝对值 < 0.5 | 图像无明显畸变或优化失败 |
| 基线距离 | 与实际测量接近 | 棋盘格尺寸设置错误 |

### 常见问题

| 问题 | 现象 | 解决方案 |
|------|------|----------|
| 内参矩阵为单位矩阵 | `[1, 0, 0; 0, 1, 0; 0, 0, 1]` | 已修复：同时优化内外参 |
| 重投影误差过大 | > 3 像素 | 检查图像质量、棋盘格参数 |
| GTK后端错误 | `Can't initialize GTK backend` | 使用 `--headless` 参数 |
| 标定失败 | `有效棋盘格图像对数量不足` | 增加图像数量、检查棋盘格可见性 |

## API 接口

### 核心类: `stereo_calib::StereoCalib`

#### 初始化

```cpp
#include "stereo_calib/stereo_calib.hpp"

// 默认构造
stereo_calib::StereoCalib stereo;

// 带参数构造
stereo_calib::StereoCalibParams params;
params.board_size = cv::Size(10, 7);
params.square_size = 0.024125f;
params.image_size = cv::Size(1280, 720);
stereo_calib::StereoCalib stereo(params);
```

#### 标定相关

```cpp
// 从文件夹加载图像进行标定
bool success = stereo.calibrateFromFolders(left_folder, right_folder);

// 从图像向量进行标定
std::vector<cv::Mat> left_images, right_images;
bool success = stereo.calibrateFromImages(left_images, right_images);

// 保存/加载标定结果
stereo.saveCalibrationToYaml("calibration.yaml");
stereo.loadCalibrationFromYaml("calibration.yaml");

// 打印标定信息
stereo.printCalibrationInfo();
```

#### 图像校正

```cpp
// 校正单张图像
cv::Mat rectified_left = stereo.rectifyImage(left_image, true);
cv::Mat rectified_right = stereo.rectifyImage(right_image, false);

// 校正立体图像对
cv::Mat rect_left, rect_right;
stereo.rectifyStereoImages(left_image, right_image, rect_left, rect_right);
```

#### 视差与深度

```cpp
// 设置视差参数
stereo_calib::DisparityParams disp_params;
disp_params.num_disparities = 64;
disp_params.block_size = 11;
disp_params.use_sgbm = true;
stereo.setDisparityParams(disp_params);

// 计算视差图
cv::Mat disparity = stereo.computeDisparity(rect_left, rect_right);

// 视差转深度
cv::Mat depth_map = stereo.disparityToDepth(disparity);

// 可视化
cv::Mat disparity_vis = stereo.visualizeDisparity(disparity);
cv::Mat depth_vis = stereo.visualizeDepth(depth_map);
```

#### 3D重建

```cpp
// 生成点云
cv::Mat point_cloud = stereo.disparityToPointCloud(disparity, rect_left);

// 单点3D投影
cv::Point2f left_point(640, 360);
cv::Point2f right_point(580, 360);  // 同一行
cv::Point3f point_3d;
bool success = stereo.projectPointTo3D(left_point, right_point, point_3d);
// 深度计算: Z = fx * baseline / disparity
```

#### 完整处理流程

```cpp
// 一步完成所有处理
stereo_calib::StereoOutput output = stereo.processStereoImages(
    left_image, 
    right_image, 
    true  // 是否计算点云
);

// 输出结果
output.rectified_left;       // 校正后的左图
output.rectified_right;      // 校正后的右图
output.disparity;            // 视差图 (CV_32F)
output.disparity_normalized; // 可视化视差图 (彩色)
output.depth_map;            // 深度图 (CV_32F, 单位: 米)
output.point_cloud;          // 点云 (CV_32FC3)
output.is_valid;             // 处理是否成功
```

## 数据结构

### StereoCalibParams

| 字段 | 类型 | 说明 |
|------|------|------|
| board_size | cv::Size | 棋盘格内角点数量 |
| square_size | float | 方格边长 (米) |
| image_size | cv::Size | 图像尺寸 |

### StereoCameraIntrinsics

| 字段 | 类型 | 说明 |
|------|------|------|
| camera_matrix_left | cv::Mat | 左相机内参矩阵 (3x3) |
| dist_coeffs_left | cv::Mat | 左相机畸变系数 (5x1) |
| camera_matrix_right | cv::Mat | 右相机内参矩阵 (3x3) |
| dist_coeffs_right | cv::Mat | 右相机畸变系数 (5x1) |
| R | cv::Mat | 右相机相对左相机的旋转矩阵 (3x3) |
| T | cv::Mat | 右相机相对左相机的平移向量 (3x1) |
| E | cv::Mat | 本质矩阵 (3x3) |
| F | cv::Mat | 基础矩阵 (3x3) |
| R1, R2 | cv::Mat | 校正旋转矩阵 (3x3) |
| P1, P2 | cv::Mat | 投影矩阵 (3x4) |
| Q | cv::Mat | 视差到深度映射矩阵 (4x4) |
| baseline | double | 基线距离 (米) |

### StereoOutput

| 字段 | 类型 | 说明 |
|------|------|------|
| rectified_left | cv::Mat | 校正后的左图 |
| rectified_right | cv::Mat | 校正后的右图 |
| disparity | cv::Mat | 视差图 (CV_32F) |
| disparity_normalized | cv::Mat | 归一化视差图 (CV_8U) |
| depth_map | cv::Mat | 深度图 (CV_32F, 米) |
| point_cloud | cv::Mat | 点云 (CV_32FC3) |
| is_valid | bool | 处理是否成功 |

### DisparityParams

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| num_disparities | int | 64 | 视差数量 (16的倍数) |
| block_size | int | 11 | 匹配块大小 (奇数, 5-21) |
| pre_filter_cap | int | 31 | 预滤波截断值 (1-63) |
| pre_filter_size | int | 9 | 预滤波核大小 (BM专用) |
| min_disparity | int | 0 | 最小视差 |
| texture_threshold | int | 10 | 纹理阈值 (BM专用) |
| uniqueness_ratio | int | 15 | 唯一性比率 |
| speckle_window_size | int | 100 | 斑点滤波窗口 |
| speckle_range | int | 32 | 斑点滤波范围 |
| disp12_max_diff | int | 1 | 一致性检查阈值 |
| use_sgbm | bool | true | 使用SGBM算法 |

## 标定结果文件格式

标定结果保存为YAML格式:

```yaml
image_width: 1280
image_height: 720

camera_matrix_left: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
camera_matrix_left_rows: 3
camera_matrix_left_cols: 3

dist_coeffs_left: [k1, k2, p1, p2, k3]
dist_coeffs_left_rows: 5
dist_coeffs_left_cols: 1

camera_matrix_right: [...]
dist_coeffs_right: [...]

R: [...]      # 旋转矩阵 (3x3)
T: [...]      # 平移向量 (3x1)
E: [...]      # 本质矩阵 (3x3)
F: [...]      # 基础矩阵 (3x3)
R1: [...]     # 左校正旋转矩阵 (3x3)
R2: [...]     # 右校正旋转矩阵 (3x3)
P1: [...]     # 左投影矩阵 (3x4)
P2: [...]     # 右投影矩阵 (3x4)
Q: [...]      # 视差到深度映射矩阵 (4x4)

baseline: 0.060
```

## 视差参数调优指南

### BM vs SGBM

| 算法 | 速度 | 精度 | 适用场景 |
|------|------|------|----------|
| BM | 快 | 低 | 实时应用、纹理丰富场景 |
| SGBM | 慢 | 高 | 精度要求高、弱纹理场景 |

### 参数调整建议

```yaml
# 精度优先
disparity_params:
  num_disparities: 128      # 增大视差范围
  block_size: 15            # 增大匹配块
  uniqueness_ratio: 20      # 提高唯一性要求
  use_sgbm: true

# 速度优先
disparity_params:
  num_disparities: 32       # 减小视差范围
  block_size: 7             # 减小匹配块
  use_sgbm: false
```

## 标定最佳实践

### 图像采集

1. **数量**: 至少 15-20 对图像
2. **角度**: 棋盘格应覆盖图像不同位置和角度
3. **距离**: 采集不同工作距离的图像
4. **光照**: 保持均匀光照，避免过曝和阴影
5. **清晰**: 确保图像清晰，无运动模糊

### 棋盘格要求

1. **平整**: 棋盘格应完全平整
2. **对比**: 黑白格对比明显
3. **尺寸**: 根据工作距离选择合适的棋盘格尺寸
4. **测量**: 准确测量方格实际尺寸

### 标定验证

1. 检查重投影误差 (< 1 像素)
2. 检查内参矩阵是否合理
3. 检查基线距离是否与实际接近
4. 使用校正后的图像验证极线对齐

## 依赖

- OpenCV 4.x (需 calib3d, highgui 模块)
- Eigen3
- yaml-cpp
- ROS2 (ament_cmake)

## 参考模块

本模块参考了 `chessboard_alg` 模块的设计风格，采用类似的代码结构和命名规范。

## 更新日志

### v1.1.0 (2026-03-28)
- 修复: 双目标定内参矩阵为单位矩阵的问题
- 新增: `stereo_process` 批量处理程序
- 新增: 无头模式支持 (`--headless`)
- 新增: 配置文件路径自动加载
- 优化: 标定同时优化内外参

### v1.0.0
- 初始版本
- 双目标定功能
- 视差图和深度图计算
- 3D重建功能
