在客户端（Jetson）主板上，需要安装psutil库来监控进程：
pip3 install psutil

1. 编译所有包

colcon build --symlink-install
source install/setup.bash
2. 运行方法一：通过单个节点启动

2.1 在x86主板上运行UDP服务端
# x86主板终端1：启动ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# x86主板终端2：启动UDP服务端
ros2 run udp_server udp_server_node
或者通过命令行传递参数
	ros2 run udp_server udp_server_node --ros-args -p client_ip:="192.168.10.61" -p bShowRunInfo:=false
	
2.2 在Jetson Orin NX上运行UDP客户端
# Jetson主板终端1：启动ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Jetson主板终端2：启动UDP客户端
ros2 run udp_client udp_client_node
或者传入x86主板的IP地址（替换为x86主板的实际IP）
	ros2 run udp_client udp_client_node --ros-args -p server_ip:=192.168.10.30

3. 运行方法二：通过launch文件启动

3.1 在x86主板上运行UDP服务端
# x86主板终端1：启动ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
# x86主板终端2：启动UDP服务端
source install/setup.bash
ros2 launch udp_server udp_server.launch.py

3.2 在Jetson Orin NX上运行UDP客户端
# Jetson主板终端1：启动ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
# Jetson主板终端2：启动UDP客户端
ros2 launch udp_client udp_client.launch.py

# 另外可通过launch参数设置相机类型：​​
ros2 launch udp_client udp_client.launch.py camera_type:=Realsense
# 或
ros2 launch udp_client udp_client.launch.py camera_type:=Gemini

4. 在x86主板上查看Jetson的话题

# 查看所有活跃话题
ros2 topic list

# 查看点云话题数据
ros2 topic echo /jetson/pointcloud

# 查看AI坐标话题数据
<!-- ros2 topic echo /jetson/ai_coordinates -->
ros2 topic echo /jetson/det_res

# 查看话题信息
ros2 topic info /jetson/pointcloud
<!-- ros2 topic info /jetson/ai_coordinates -->
ros2 topic info /jetson/det_res

标定流程使用方式

1. 启动标定流程（三种方式任选其一）
​方式A：使用专用启动脚本（最简单）​​
# 在x86服务器端执行
ros2 run udp_server start_calibration

​方式B：通过服务调用（服务器端）​​
# 在x86服务器端执行
ros2 service call /calibration/start std_srvs/srv/Trigger

​方式C：通过服务调用（客户端）​​
# 在Jetson客户端执行
	ros2 service call /calibration_client/start std_srvs/srv/Trigger

验证方式：
启动某个项目：
	例如打开realsense相机，执行如下指令：
# 在x86终端执行如下指令启动Jetson主板上的realsense相机
ros2 run udp_server project_control start camera   
# 在x86终端执行如下指令启动Jetson主板上的相机点云转雷达scan数据的项目
ros2 run udp_server project_control start pcl2laser
# 在x86终端执行如下指令启动Jetson主板上的目标检测的项目
ros2 run udp_server project_control start detection 
# 在x86终端执行如下指令启动Jetson主板上的语音中间调度项目
ros2 run udp_server project_control start cmd_dispatcher

停止某个项目：
# 在x86终端执行如下指令停止Jetson主板上的realsense相机
ros2 run udp_server project_control stop camera
# 在x86终端执行如下指令停止Jetson主板上的相机点云转雷达scan数据的项目
ros2 run udp_server project_control stop pcl2laser
# 在x86终端执行如下指令停止Jetson主板上的目标检测的项目
ros2 run udp_server project_control stop detection
# 在x86终端执行如下指令停止Jetson主板上的语音中间调度项目
ros2 run udp_server project_control stop cmd_dispatcher  

查看某个项目的启停状态：
# 在x86上检查点云转激光雷达项目状态
# 在x86终端执行如下指令查看Jetson主板上的realsense相机运行状态
ros2 run udp_server project_control status camera
# 在x86终端执行如下指令查看Jetson主板上的相机点云转雷达scan数据的项目运行状态
ros2 run udp_server project_control status pcl2laser 
# 在x86终端执行如下指令查看Jetson主板上的目标检测的项目运行状态
ros2 run udp_server project_control status detection
# 在x86终端执行如下指令查看Jetson主板上的语音中间调度项目运行状态
ros2 run udp_server project_control status cmd_dispatcher

验证进程是否真的被终止：
# 在Jetson主板上检查realsense进程
ps aux | grep realsense

2. 标定流程说明
1.​启动标定​：
	•服务器端初始化标定状态
	•发送START_CALIBRATION指令给客户端
	•客户端初始化标定状态
2.​发送第一个标定点​：
•服务器端获取机械手位姿
•发送NEXT_POINT指令给客户端
3.​客户端处理​：
•收到NEXT_POINT指令
•执行Aruco识别
•发送响应（SUCCESS/FAILURE）
4.​服务器响应处理​：
•收到SUCCESS响应：发送下一个点（直到达到100点）
•收到FAILURE响应：终止标定流程
5.
​标定完成​：
•客户端收集100个点后计算变换矩阵
•发送COMPLETED响应给服务器
•服务器端结束标定流程
