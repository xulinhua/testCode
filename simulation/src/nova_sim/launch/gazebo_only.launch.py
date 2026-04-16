import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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

    # 世界文件（默认使用包内 simple.world，便于调 ODE；上层 launch 可覆盖）
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('nova_sim'), 'worlds', 'simple.world']
        ),
        description='Gazebo world file path',
    )

    # URDF 文件参数
    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(your_pkg, 'urdf', 'nova_robot_position.urdf'),
        description='URDF file path'
    )
    spawn_z_arg = DeclareLaunchArgument(
        name='spawn_z',
        default_value='0.06',
        description='Initial Z offset to avoid base-ground penetration',
    )

    # URDF/xacro 路径
    urdf_path = LaunchConfiguration('urdf_file')
    robot_description = Command(['xacro ', urdf_path])

    # Include the empty_world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # TF static transform publisher
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    # 发布机器人描述（先展开 xacro，确保 aruco 宏等内容真正进入模型）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 从 robot_description 话题生成模型，避免 -file 直读导致 xacro 宏未展开
    spawn_model = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-topic', 'robot_description',
             '-entity', 'nova_robot',
             '-z', LaunchConfiguration('spawn_z')],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        urdf_file_arg,
        spawn_z_arg,
        gazebo_launch,
        tf_world_base,
        tf_footprint_base,
        robot_state_publisher,
        spawn_model
    ])
