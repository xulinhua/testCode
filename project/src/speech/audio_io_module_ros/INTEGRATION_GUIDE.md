# Audio I/O Module 与 ROS 2 集成指南

## 问题背景

你需要同时构建 `audio_io_module` 和 `audio_io_module_ros` 项目，但遇到了以下问题：

1. `audio_io_module` 是一个纯 Python 项目，需要保持跨平台特性
2. `audio_io_module_ros` 需要调用 `audio_io_module` 的接口
3. ROS 2 无法正确找到 `audio_io_module_ros` 包

## 解决方案

### 1. 项目结构优化

已完成的修改：

#### audio_io_module (纯 Python 包)
- 创建了 `setup.py` 使其成为标准 Python 包
- 添加了 `__init__.py` 使其成为真正的 Python 包
- 保持了跨平台特性，无 ROS 依赖

#### audio_io_module_ros (ROS 2 包)
- 修复了 `setup.py` 配置，使用标准 ROS 2 Python 包结构
- 修改了导入路径，使 ROS 节点能正确找到 `audio_io_module`

### 2. 集成方法

#### 方法一：使用相对路径导入（已实现）

在 `audio_io_module_ros` 的 ROS 节点中，使用相对路径导入 `audio_io_module`：

```python
import sys
import os

# 添加audio_io_module到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

try:
    from audio_io_module.audio_io_manager import AudioIOManager
except ImportError as e:
    print(f"无法导入audio_io_module: {e}")
    sys.exit(1)
```

#### 方法二：创建符号链接（推荐在Linux环境下使用）

在Linux环境下，可以在 `audio_io_module_ros/src/audio_io_module_ros` 目录中创建符号链接：

```bash
cd audio_io_module_ros/src/audio_io_module_ros
ln -s ../../../audio_io_module audio_io_module
```

然后修改导入方式：

```python
# 直接导入，无需修改sys.path
from audio_io_module.audio_io_manager import AudioIOManager
```

### 3. 构建和运行步骤

#### 在Windows环境下：

1. 确保在 `speech` 目录下工作
2. 清理之前的构建：
   ```powershell
   Remove-Item -Recurse -Force build,install,log
   ```

3. 使用colcon构建：
   ```bash
   colcon build
   ```

4. 设置环境并运行：
   ```bash
   source install/setup.bash
   ros2 launch audio_io_module_ros audio_io_launch.py
   ```

### 4. 故障排除

#### 如果仍然遇到"Package 'audio_io_module_ros' not found"错误：

1. 检查环境变量：
   ```bash
   echo $AMENT_PREFIX_PATH
   echo $COLCON_PREFIX_PATH
   ```

2. 确保所有必要的路径都包含在环境变量中

3. 重新构建并确保构建成功：
   ```bash
   colcon build --packages-select audio_io_module_ros
   ```

### 5. 验证集成

测试 `audio_io_module` 是否能正确导入：

```python
# 在Python中测试
import sys
sys.path.insert(0, '/path/to/speech/audio_io_module')
try:
    from audio_io_manager import AudioIOManager
    print("audio_io_module 导入成功！")
except ImportError as e:
    print(f"导入失败: {e}")
```

## 技术架构

这种集成方法具有以下优点：

1. **架构清晰**：`audio_io_module` 保持纯 Python 特性，无 ROS 依赖
2. **跨平台兼容**：在Windows和Linux下都能正常工作
3. **易于维护**：两个项目可以独立开发和测试
4. **ROS 2 标准**：使用标准的 ROS 2 Python 包结构

## 注意事项

1. 确保在构建前清理旧的构建文件
2. 在Windows环境下需要使用PowerShell命令
3. 如果使用符号链接方法，需要确保文件系统支持符号链接
4. 确保 `audio_io_module` 的依赖（如pyaudio）已正确安装

这种集成方案既满足了你的需求，又保持了项目的整洁性和可维护性。