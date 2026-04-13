# PointCloud UDP测试指南

## 测试目的

本测试脚本用于验证UDP通信模块的点云数据传输功能，具体测试流程如下：

1. **测试发布**：ROS 2节点发布模拟的PointCloudData数据
2. **UDP传输**：UDP客户端订阅数据并通过UDP发送到服务端
3. **接收验证**：UDP服务端接收并处理数据
4. **端到端验证**：验证整个数据传输链路的完整性

## 测试环境准备

### 1. 编译项目

```bash
cd /home/user/code/Dev
colcon build --packages-select custom_msgs_comm udp_comm_client udp_comm_server
```

### 2. 配置本地测试环境

修改配置文件，将IP地址改为本地回环地址：

#### UDP客户端配置 (`udp_client_params.yaml`)
```yaml
udp_client:
  ros__parameters:
    server_ip: "127.0.0.1"  # 本地回环地址
    server_port: 8888
    client_port: 8889
    # 其他参数保持不变
```

#### UDP服务器配置 (`udp_server_params.yaml`)
```yaml
udp_server:
  ros__parameters:
    udp_port: 8888
    # 其他参数保持不变
    client_ip: "127.0.0.1"  # 本地回环地址
    client_port: 8889
```

## 测试脚本说明

### 脚本功能

- **`test_pointcloud_udp.py`**：ROS 2节点，发布模拟的PointCloudData数据
  - 发布话题：`/camera/pointcloud`
  - 发布频率：2 Hz
  - 点云大小：100个点
  - 数据格式：符合custom_msgs_comm/PointCloudData定义

### 测试数据结构

发布的PointCloudData数据包含以下字段：

| 字段 | 类型 | 值 |
|------|------|------|
| header | Header | 包含frame_id和时间戳 |
| height | uint32 | 1 |
| width | uint32 | 100 |
| fields | PointField[] | x, y, z, intensity字段信息 |
| is_bigendian | boolean | false |
| point_step | uint32 | 32 |
| row_step | uint32 | 3200 |
| data | uint8[] | 点云数据（3200字节） |
| is_dense | boolean | true |

**header字段：**
| 子字段 | 类型 | 值 |
|--------|------|------|
| frame_id | string | "camera_depth_optical_frame" |
| stamp | Time | 当前系统时间 |

## 测试步骤

### 步骤1：启动UDP服务器

在第一个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
ros2 launch udp_comm_server udp_server.launch.py
```

### 步骤2：启动UDP客户端

在第二个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
ros2 launch udp_comm_client udp_client.launch.py
```

### 步骤3：运行测试发布器

在第三个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
python3 src/comm/udp_comm/src/test/test_pointcloud_udp.py
```

### 步骤4：验证测试结果

#### 4.1 发布器日志

查看测试发布器的输出，确认数据已成功发布：

```
[1/20] 发布点云数据
  点数量: 100
  数据大小: 3200 字节
  坐标系: camera_depth_optical_frame
  时间戳: 1708843200.123456
```

#### 4.2 客户端日志

查看UDP客户端的输出，确认数据已被订阅并发送：

```
[INFO] [udp_client]: Received PointCloudData from /camera/pointcloud
[INFO] [udp_client]: Sending pointcloud data to server: 3424 bytes
```

#### 4.3 服务器日志

查看UDP服务器的输出，确认数据已被接收并处理：

```
[INFO] [udp_server]: Received pointcloud data: 3424 bytes
[INFO] [udp_server]: Publishing PointCloudData to /camera/pointcloud
```

#### 4.4 ROS话题验证

使用`ros2 topic echo`查看服务器发布的话题，确认数据传输完整：

```bash
ros2 topic echo /camera/pointcloud
```

## 故障排除

### 常见问题

1. **发布器无法启动**
   - 检查`custom_msgs_comm`是否已正确编译
   - 确认消息类型导入正确

2. **客户端未订阅到数据**
   - 检查客户端订阅的话题名称是否为`/camera/pointcloud`
   - 确认QoS设置是否匹配

3. **服务器未接收到数据**
   - 检查UDP配置文件中的IP和端口设置
   - 确认客户端和服务器都在运行
   - 检查网络连接和防火墙设置

4. **数据格式错误**
   - 确认点云数据的二进制格式与C++版本一致
   - 检查数据大小和字段顺序

### 调试工具

- **网络监控**：`sudo tcpdump -i any udp port 8888`
- **ROS话题**：`ros2 topic list` 和 `ros2 topic echo`
- **节点列表**：`ros2 node list`
- **服务列表**：`ros2 service list`

## 测试结果评估

### 成功标准

1. **发布成功**：测试发布器成功发布所有20个数据包
2. **传输成功**：客户端成功通过UDP发送数据
3. **接收成功**：服务器成功接收并处理数据
4. **发布回ROS**：服务器成功将数据重新发布到ROS话题
5. **数据完整**：传输的数据与原始数据一致

### 预期性能

- **发布延迟**：< 100ms
- **传输延迟**：< 50ms
- **CPU使用率**：< 5%
- **内存使用率**：< 100MB

## 扩展测试

### 1. 压力测试

修改测试脚本中的参数进行压力测试：

```python
self.publish_rate = 10  # 增加发布频率
self.max_count = 100    # 增加发布次数
msg.width = 1000         # 增加点数量
```

### 2. 稳定性测试

长时间运行测试以验证系统稳定性：

```python
self.publish_rate = 1    # 降低频率
self.max_count = 1000    # 大幅增加发布次数
```

### 3. 网络异常测试

模拟网络异常情况：
- 暂时断开网络连接
- 恢复网络连接
- 验证系统是否能够自动恢复

## 注意事项

1. **资源管理**：测试完成后，记得关闭所有终端和进程
2. **端口冲突**：确保端口8888和8889未被其他程序占用
3. **配置恢复**：测试完成后，根据需要恢复原始IP配置
4. **权限问题**：确保脚本有执行权限
5. **依赖检查**：确保所有依赖包已正确安装

## 激光雷达UDP测试

### 测试目的

测试LaserScanData数据通过UDP从客户端传输到服务端的完整流程。

### 测试步骤

#### 步骤1：启动UDP服务器（服务端）

在第一个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
ros2 launch udp_comm_server udp_server.launch.py
```

#### 步骤2：启动UDP客户端（客户端）

在第二个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
ros2 launch udp_comm_client udp_client.launch.py
```

#### 步骤3：运行激光雷达发布器（客户端）

在第三个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
python3 src/comm/udp_comm/src/test/test_laserscan_publisher.py
```

#### 步骤4：运行激光雷达订阅器（服务端）

在第四个终端中：

```bash
cd /home/user/code/Dev
source install/setup.bash
python3 src/comm/udp_comm/src/test/test_laserscan_subscriber.py
```

### 验证测试结果

#### 发布器日志（客户端）

```
[1/50] 发布激光雷达数据
  扫描点数: 361
  角度范围: [-3.14, 3.14] rad
  距离范围: [0.10, 10.00] m
  坐标系: laser_frame
  时间戳: 1708843200.123456
  前5个距离值: ['2.45', '2.47', '2.50', '2.53', '2.55'] m
```

#### 订阅器日志（服务端）

```
[1] 接收到激光雷达数据
  扫描点数: 361
  角度范围: [-3.14, 3.14] rad
  角度增量: 0.0175 rad
  距离范围: [0.10, 10.00] m
  扫描时间: 0.1000 s
  坐标系: laser_frame
  时间戳: 1708843200.123456
  距离统计: min=2.35, max=7.89, avg=5.23 m
  前5个距离值: ['2.45', '2.47', '2.50', '2.53', '2.55'] m
  强度数据点数: 361
  前5个强度值: ['0.392', '0.389', '0.385', '0.382', '0.378']
```

### 测试参数

可在 `test_laserscan_publisher.py` 中修改以下参数：

```python
# 发布频率
self.publish_rate = 10  # Hz

# 发布次数
self.max_count = 50

# 激光雷达参数
self.angle_min = -math.pi      # -180度
self.angle_max = math.pi       # +180度
self.angle_increment = math.pi / 180  # 1度
self.range_min = 0.1           # 最小距离
self.range_max = 10.0          # 最大距离
```

## 测试完成

测试完成后，按Ctrl+C停止所有进程：

1. 停止激光雷达订阅器
2. 停止激光雷达发布器
3. 停止UDP客户端
4. 停止UDP服务器

## 版本信息

- **测试脚本版本**：1.1.0
- **支持ROS版本**：ROS 2 Humble
- **支持操作系统**：Ubuntu 22.04
- **最后更新**：2026-03-25
