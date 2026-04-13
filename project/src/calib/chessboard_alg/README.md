# Chessboard Algorithm Module

## 模块介绍

`chessboard_alg` 是一个基于 OpenCV 的棋盘格检测和位姿估计模块，用于机器人视觉系统中的相机标定和目标定位。该模块提供了完整的棋盘格检测、位姿估计和相机标定功能，适用于需要精确相机位姿估计的应用场景。

## 功能特点

- ✅ 棋盘格角点检测
- ✅ 棋盘格位姿估计（位置和旋转）
- ✅ 相机内参标定
- ✅ 检测结果可视化
- ✅ 支持自定义棋盘格尺寸
- ✅ 支持自定义棋盘格方格大小

## 目录结构

```
chessboard_alg/
├── CMakeLists.txt        # 构建配置文件
├── package.xml           # ROS 包配置文件
└── src/
    ├── include/chessboard_alg/
    │   └── chessboard_pose_detector.hpp  # 头文件
    ├── src/
    │   └── chessboard_pose_detector.cpp  # 实现文件
    └── example_chessboard_pose.cpp       # 示例代码
```

## 安装依赖

### 核心依赖
- OpenCV 4.0+
- C++17 或更高版本
- CMake 3.10+

### ROS 依赖（可选）
- ROS 2 Humble 或更高版本

## 构建方法

1. **克隆代码**
   ```bash
   cd /path/to/workspace/src
   git clone <repository_url>
   ```

2. **构建项目**
   ```bash
   cd /path/to/workspace
   colcon build --packages-select chessboard_alg
   ```

3. **设置环境**
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 1. 基本使用示例

```cpp
#include <chessboard_alg/chessboard_pose_detector.hpp>
#include <opencv2/opencv.hpp>

int main() {
    // 创建检测器实例（8x6 棋盘格，方格大小 0.025 米）
    chessboard_alg::ChessboardPoseDetector detector(cv::Size(8, 6), 0.025);
    
    // 设置相机内参（如果已知）
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    detector.setCameraIntrinsics(camera_matrix, dist_coeffs);
    
    // 读取图像
    cv::Mat frame = cv::imread("chessboard.jpg");
    
    // 检测并估计位姿
    auto pose_info = detector.detectAndEstimatePose(frame);
    
    // 显示结果
    cv::imshow("Result", frame);
    cv::waitKey(0);
    
    return 0;
}
```

### 2. 相机标定示例

```cpp
#include <chessboard_alg/chessboard_pose_detector.hpp>
#include <opencv2/opencv.hpp>

int main() {
    // 创建检测器实例
    chessboard_alg::ChessboardPoseDetector detector(cv::Size(8, 6), 0.025);
    
    // 加载标定图像
    std::vector<cv::Mat> calibration_images;
    // ... 加载多个棋盘格图像 ...
    
    // 标定相机内参
    cv::Mat camera_matrix, dist_coeffs;
    bool success = detector.calibrateCameraIntrinsics(calibration_images, camera_matrix, dist_coeffs);
    
    if (success) {
        std::cout << "相机标定成功！" << std::endl;
        std::cout << "相机矩阵：" << std::endl << camera_matrix << std::endl;
        std::cout << "畸变系数：" << std::endl << dist_coeffs << std::endl;
    } else {
        std::cout << "相机标定失败！" << std::endl;
    }
    
    return 0;
}
```

## API 参考

### 主要类和结构

#### `ChessboardPoseInfo` 结构

| 字段 | 类型 | 描述 |
|------|------|------|
| `center_2d` | `cv::Point2f` | 棋盘格中心点像素坐标 |
| `position` | `cv::Point3f` | 棋盘格位置（世界坐标系） |
| `rotation` | `cv::Vec3f` | 棋盘格旋转欧拉角（度） |
| `rotation_matrix` | `cv::Matx33f` | 旋转矩阵 |
| `rvec` | `cv::Vec3d` | 旋转向量 |
| `tvec` | `cv::Vec3d` | 平移向量 |
| `corners` | `std::vector<cv::Point2f>` | 检测到的角点 |
| `world_corners` | `std::vector<cv::Point3f>` | 世界坐标系下角点 |
| `distance` | `float` | 相机到棋盘格的距离 |
| `found` | `bool` | 是否检测到棋盘格 |

#### `ChessboardPoseDetector` 类

##### 构造函数
```cpp
ChessboardPoseDetector(cv::Size board_size = cv::Size(8, 6), float square_size = 0.025);
```

##### 核心方法

| 方法 | 描述 | 参数 | 返回值 |
|------|------|------|--------|
| `detectChessboard` | 检测棋盘格角点 | `frame`: 输入图像 | `bool`: 是否检测成功 |
| `estimatePose` | 估计棋盘格位姿 | `frame`: 输入图像 | `ChessboardPoseInfo`: 位姿信息 |
| `detectAndEstimatePose` | 完整检测和估计流程 | `frame`: 输入图像<br>`draw_results`: 是否绘制结果<br>`print_results`: 是否打印结果 | `ChessboardPoseInfo`: 完整检测结果 |
| `calibrateCameraIntrinsics` | 相机内参标定 | `images`: 标定图像集<br>`camera_matrix`: 输出相机矩阵<br>`dist_coeffs`: 输出畸变系数 | `bool`: 标定是否成功 |
| `drawChessboardResults` | 绘制检测结果 | `frame`: 输入图像<br>`pose_info`: 位姿信息<br>`corners`: 角点<br>`draw_axes`: 是否绘制坐标轴 | `cv::Mat`: 绘制结果 |
| `printChessboardResults` | 打印检测结果 | `pose_info`: 位姿信息 | 无 |

##### 配置方法

| 方法 | 描述 | 参数 |
|------|------|------|
| `setBoardSize` | 设置棋盘格尺寸 | `board_size`: 棋盘格内角点尺寸 |
| `setSquareSize` | 设置方格大小 | `square_size`: 方格尺寸（米） |
| `setCameraIntrinsics` | 设置相机内参 | `camera_matrix`: 相机矩阵<br>`dist_coeffs`: 畸变系数 |

## 示例代码

### 运行示例

```bash
cd /path/to/workspace
colcon build --packages-select chessboard_alg
source install/setup.bash
ros2 run chessboard_alg example_chessboard_pose
ros2 run chessboard_alg camera_calibration_example "/home/user/code/Dev/Src_Image copy 3/left"
```

### 示例输出

```
Chessboard detected!
Position: (0.123, -0.045, 1.234)
Rotation: (10.2, -5.3, 2.1) degrees
Distance: 1.234 meters
```

## 注意事项

1. **棋盘格要求**
   - 棋盘格应该有清晰的黑白对比
   - 棋盘格应该完整地出现在图像中
   - 棋盘格应该有足够的光照

2. **相机内参**
   - 为了获得准确的位姿估计，需要提供相机内参
   - 如果没有相机内参，可以使用 `calibrateCameraIntrinsics` 方法进行标定

3. **性能优化**
   - 对于实时应用，可以考虑降低图像分辨率
   - 可以使用多线程来加速处理

4. **错误处理**
   - 检测失败时，`detectChessboard` 会返回 `false`
   - 位姿估计失败时，`estimatePose` 会返回 `found = false` 的 `ChessboardPoseInfo`

## 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 检测不到棋盘格 | 光照不足 | 增加光照 |
| | 棋盘格不完整 | 确保棋盘格完整出现在图像中 |
| | 棋盘格尺寸设置错误 | 检查 `board_size` 参数 |
| 位姿估计不准确 | 相机内参错误 | 重新标定相机 |
| | 方格大小设置错误 | 检查 `square_size` 参数 |
| 标定失败 | 标定图像数量不足 | 至少使用10-15张不同角度的图像 |
| | 标定图像质量差 | 使用清晰、不同角度的图像 |

## 应用场景

- 机器人视觉系统相机标定
- 机器人手眼标定
- 增强现实（AR）应用
- 计算机视觉研究和教育
- 工业自动化视觉引导

## 版本信息

- **版本**: 1.0.0
- **更新日期**: 2026-02-28
- **作者**: Chessboard Algorithm Team

## 许可证

本模块采用 MIT 许可证，详见 LICENSE 文件。