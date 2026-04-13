# cam_demo - 相机管理库调用演示指南

## 项目概述

cam_demo是一个完整的演示项目，专门用于展示如何正确配置和调用cam_manage相机管理库。本项目演示了多相机系统的实际应用，包括ROS2节点发布、数据可视化和配置管理等功能。

## 项目结构说明
### cam_demo详细文件说明
```
cam_demo/src/                    # cam_demo项目根目录
├── CMakeLists.txt              # CMake构建配置文件
│   - 配置cam_manage依赖查找
│   - 设置头文件包含路径
│   - 配置库链接关系
│   - 定义可执行文件构建规则
│   - 配置相机SDK链接
├── package.xml                 # ROS包配置文件
│   - 声明项目元数据
│   - 定义cam_manage依赖关系
│   - 配置ROS2和视觉处理依赖
│   - 设置构建类型和导出规则
├── README.md                   # 项目说明文档
│   - 调用配置指南
│   - 使用示例说明
│   - 故障排除方法
└── src/                        # 源代码目录
    └── cam_demo.cpp           # 演示程序主文件
        - 包含cam_manage头文件
        - 实现ROS2节点类
        - 调用相机管理器接口
        - 发布图像和点云数据
        - 加载和管理相机配置
```

### 关键文件作用
- **CMakeLists.txt**: 配置构建系统，确保正确查找和链接cam_manage库及相关依赖
- **package.xml**: 声明对cam_manage和ROS2组件的依赖关系
- **cam_demo.cpp**: 实际调用cam_manage接口的演示代码，实现完整的ROS2相机节点
- **README.md**: 完整的配置和使用指南
- **sys_config/cam_config.yaml**: 相机参数配置文件

## 调用cam_manage接口的配置步骤

### 1. 构建顺序要求

**必须先构建cam_manage模块，再构建调用项目：**

```bash
# 第一步：构建cam_manage库
colcon build --packages-select cam_manage

# 第二步：构建调用项目（cam_demo）
colcon build --packages-select cam_demo
```

### 2. package.xml配置 - 需要修改的关键部分

在调用项目的`package.xml`中添加cam_manage依赖，**必须添加以下依赖项**：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cam_demo</name>
  <version>0.0.0</version>
  <description>Camera demo package for testing camera management functionality</description>
  <maintainer email="xlh@todo.todo">xlh</maintainer>
  <license>TODO: License declaration</license>

  <!-- 构建工具依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- ROS2核心依赖 -->
  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>ament_index_cpp</depend>
  
  <!-- 视觉处理依赖 -->
  <depend>pcl_ros</depend> 
  <depend>rviz2</depend>
  <depend>visualization_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  
  <!-- ========== 关键修改：添加cam_manage依赖 ========== -->
  <depend>cam_manage</depend>
  <!-- =================================================== -->
  
  <!-- 其他依赖 -->
  <depend>yaml-cpp</depend> 

  <!-- OpenCV依赖 -->
  <build_depend>libopencv-dev</build_depend>
  <exec_depend>libopencv-dev</exec_depend>

  <!-- 启动依赖 -->
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**修改说明**：
- ✅ **必须添加**：`<depend>cam_manage</depend>`依赖项
- 🔄 **如果已有**：确保该依赖项存在且正确
- ⚠️ **注意事项**：该依赖项必须在其他视觉处理依赖之后添加

### 3. CMakeLists.txt配置 - 需要修改的关键部分

在调用项目的`CMakeLists.txt`中配置依赖和链接，**必须进行以下关键修改**：

```cmake
cmake_minimum_required(VERSION 3.8)
project(cam_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== 关键修改：查找依赖包 ==========
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# ========== 关键修改：查找cam_manage包 ==========
find_package(cam_manage REQUIRED)
# ===============================================

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_msgs REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== 关键修改：包含头文件目录 ==========
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cam_manage_INCLUDE_DIRS}  # 关键：包含cam_manage头文件
  ${YAML_CPP_INCLUDE_DIRS}
  /usr/local/include
)
# =============================================

# 创建可执行文件
add_executable(cam_demo 
  src/src/cam_demo.cpp
)

# ========== 关键修改：链接相机SDK库 ==========
find_library(ORBBEC_LIB NAMES OrbbecSDK libOrbbecSDK.so)
if(ORBBEC_LIB)
    target_link_libraries(cam_demo ${ORBBEC_LIB})
endif()

find_library(REALSENSE_LIB NAMES librealsense2.so realsense2)
if(REALSENSE_LIB)
    target_link_libraries(cam_demo /usr/local/lib/librealsense2.so)
endif()
# ============================================

# ========== 关键修改：链接依赖库 ==========
target_link_libraries(cam_demo
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${cam_manage_LIBRARIES}  # 关键：链接cam_manage库
)

ament_target_dependencies(cam_demo 
  rclcpp
  rclcpp_components
  visualization_msgs
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  cam_manage  # 关键：添加cam_manage依赖
  cv_bridge
  image_transport
  yaml-cpp
  realsense2
)
# ==========================================

# 安装配置
install(TARGETS cam_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

**关键修改点总结**：
1. ✅ **查找包**：`find_package(cam_manage REQUIRED)`
2. ✅ **包含头文件**：`include_directories`中添加`${cam_manage_INCLUDE_DIRS}`
3. ✅ **链接库**：`target_link_libraries`中添加`${cam_manage_LIBRARIES}`
4. ✅ **依赖关系**：`ament_target_dependencies`中添加`cam_manage`

### 4. 源代码使用示例 - 需要修改的关键部分

在C++代码中正确使用cam_manage接口，**必须进行以下关键修改**：

```cpp
// ========== 关键修改：包含cam_manage头文件 ==========
#include "cam_manage/cam_manage.hpp"
#include "cam_manage/cam_com_struct.hpp"
// ==================================================

#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// ========== 关键修改：实现ROS2相机节点类 ==========
class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        // 创建发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/points", 10);

        // ========== 关键修改：初始化相机管理器 ==========
        cam_manager_ = &CameraManager::get_instance();
        // ===============================================
        
        // 遍历相机
        get_all_camera();
        // 从配置加载相机
        load_camera_config();
        // 获取内参
        get_cam_intr();
        // 创建定时器获取并发布数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CameraPublisher::publish_data, this));
    }

private:
    // ========== 关键修改：相机管理相关成员变量 ==========
    CameraManager* cam_manager_;
    std::vector<CameraInfo> camera_infos_;
    CameraIntrinsics camera_intrinsics_;
    // ==================================================
    
    // ROS2发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 数据缓存
    cv::Mat* color_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // ========== 关键修改：相机管理相关方法 ==========
    void get_all_camera()
    {
        camera_infos_ = cam_manager_->discover_cameras();
        RCLCPP_INFO(this->get_logger(), "发现 %zu 个相机", camera_infos_.size());
        
        for (size_t i = 0; i < camera_infos_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "相机 %zu: %s (%s)", 
                       i, camera_infos_[i].device_name.c_str(), 
                       camera_infos_[i].device_id.c_str());
        }
    }
    
    void load_camera_config()
    {
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["cameras"]) {
                for (const auto& camera_config : config["cameras"]) {
                    std::string device_id = camera_config["device_id"].as<std::string>();
                    bool enabled = camera_config["enabled"].as<bool>();
                    
                    if (enabled) {
                        cam_manager_->connect_camera(device_id);
                        RCLCPP_INFO(this->get_logger(), "连接相机: %s", device_id.c_str());
                    }
                }
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "配置文件加载失败: %s", e.what());
        }
    }
    
    void get_cam_intr()
    {
        if (!camera_infos_.empty()) {
            if (cam_manager_->get_camera_intrinsics(camera_intrinsics_)) {
                RCLCPP_INFO(this->get_logger(), "相机内参获取成功");
            } else {
                RCLCPP_WARN(this->get_logger(), "相机内参获取失败");
            }
        }
    }
    
    void publish_data()
    {
        // ========== 关键修改：获取相机数据 ==========
        if (cam_manager_->get_color_data(*color_image)) {
            // 发布彩色图像
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *color_image).toImageMsg();
            image_msg->header.stamp = this->now();
            image_msg->header.frame_id = "camera_link";
            image_publisher_->publish(*image_msg);
        }
        
        if (cam_manager_->get_point_cloud_data(cloud)) {
            // 发布点云数据
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.stamp = this->now();
            cloud_msg.header.frame_id = "camera_link";
            pointcloud_publisher_->publish(cloud_msg);
        }
        // ===========================================
    }
    // ==============================================
};
// ===================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 配置文件设置

### 相机配置文件 (sys_config/cam_config.yaml)

```yaml
default_camera:
  camera_type: "realsense"
  device_id: ""
  resolution: "1280x720"
  fps: 30
  auto_exposure: true
  auto_white_balance: true
  get_intrinsics_from_api: true
```

## 运行和测试

### 1. 构建项目
```bash
# 确保工作空间已正确设置
cd /home/user/project

# 构建cam_manage（如果尚未构建）
colcon build --packages-select cam_manage

# 构建cam_demo
colcon build --packages-select cam_demo

# 源化工作空间
source install/setup.sh
```

### 2. 运行演示程序
```bash
# 直接运行
ros2 run cam_demo cam_demo

```

## 常见问题和解决方案

### 1. 构建问题
**问题**: 找不到cam_manage包
**解决**: 确保先构建cam_manage，且路径正确
```bash
colcon build --packages-select cam_manage --symlink-install
```

**问题**: 链接错误，找不到cam_manage_LIBRARIES
**解决**: 检查cam_manage的CMakeLists.txt是否正确导出库
```bash
# 检查cam_manage是否正确安装
ls install/cam_manage/lib/
```

### 2. 运行时问题
**问题**: 相机连接失败
**解决**: 检查相机权限和连接
```bash
# 检查USB设备
lsusb | grep -i "camera\|orbbec\|intel"

# 添加用户权限
sudo usermod -a -G plugdev $USER
```

**问题**: 配置文件找不到
**解决**: 检查配置文件路径和权限
```bash
# 创建配置目录
mkdir -p install/sys_config
cp sys_config/cam_config.yaml install/sys_config/
```

### 3. 数据发布问题
**问题**: 图像或点云数据为空
**解决**: 检查相机初始化和数据获取流程
```bash
# 查看调试信息
ros2 run cam_demo cam_demo --ros-args --log-level DEBUG
```
