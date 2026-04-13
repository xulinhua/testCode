from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Get the package share directory
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    your_pkg = get_package_share_directory('x-trainer_asm-0226')

    package_name = "x-trainer_asm-0226"
    pkg_share = os.path.join(get_package_prefix(package_name), 'share')

    os.environ['GAZEBO_MODEL_PATH'] = pkg_share + os.pathsep + os.environ['GAZEBO_MODEL_PATH']

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
    
    # Robot state publisher to publish URDF
    urdf_path = os.path.join(your_pkg, 'urdf', 'x-trainer.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn model from topic
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'x-trainer'
        ],
        output='screen'
    )
    
    # Fake joint calibration - using proper ROS2 topic pub format
    # fake_joint_calibration = ExecuteProcess(
    #     cmd=['ros2', 'topic', 'pub', '--once', '/calibrated', 'std_msgs/msg/Bool', '{data: true}'],
    #     output='screen'
    # )
    
    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        robot_state_publisher,
        spawn_model
        # fake_joint_calibration
    ])