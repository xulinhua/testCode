import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    # Get the package share directory
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    your_pkg = get_package_share_directory('nova_sim')
    
    package_name = "nova_sim"
    pkg_share = os.path.join(get_package_prefix(package_name), 'share')
    
    # 设置 GAZEBO_MODEL_PATH 环境变量
    os.environ['GAZEBO_MODEL_PATH'] = pkg_share + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')

    # 直接读取 URDF 文件内容
    urdf_path = os.path.join(your_pkg, 'urdf', 'nova_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # Include the empty_world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': '/usr/share/gazebo-11/worlds/empty.world'}.items()
    )
    
    # TF static transform publisher
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # 发布关节状态（驱动 TF 链路）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # 发布机器人描述
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Spawn model from topic
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'nova_robot'
        ],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        joint_state_publisher,
        robot_state_publisher,
        spawn_model
    ])
