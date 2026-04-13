执行流程

# 重新构建
colcon build --packages-select cmd_dispatcher

# 重新运行
source install/setup.bash
ros2 launch cmd_dispatcher cmd_dispatcher.launch.py
