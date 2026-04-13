import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import sys


def generate_launch_description():
    """生成手势识别launch描述文件"""

    # 声明launch参数
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='Realsense',
        description='Camera type: Realsense or Gemini'
    )

    enable_vis_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='True',
        description='Enable visualization window'
    )

    move_thr_arg = DeclareLaunchArgument(
        'move_thr',
        default_value='20',
        description='Move threshold for Wave gesture detection'
    )

    # 从命令行参数获取相机类型（兼容旧的参数格式）
    camera_type = "Realsense"
    if 'camera_type:=Gemini' in sys.argv:
        camera_type = "Gemini"
    elif 'camera_type:=Realsense' in sys.argv:
        camera_type = "Realsense"

    # 手势识别节点
    gesture_rec_node = Node(
        package='gesture_rec',
        executable='gesture_rec_node.py',
        name='gesture_rec',
        output='screen',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'move_thr': LaunchConfiguration('move_thr'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        camera_type_arg,
        enable_vis_arg,
        move_thr_arg,
        gesture_rec_node
    ])
