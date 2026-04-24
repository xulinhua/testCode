import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    your_pkg = FindPackageShare('nova_sim_mujoco').find('nova_sim_mujoco')
    urdf_file = LaunchConfiguration('urdf_file').perform(context)

    robot_description_raw = subprocess.check_output(
        ['xacro', urdf_file],
        text=True,
    )

    # Reuse existing URDF and swap ros2_control hardware plugin from Gazebo to MuJoCo.
    robot_description_fixed = robot_description_raw.replace(
        'gazebo_ros2_control/GazeboSystem',
        'mujoco_ros2_control/MujocoSystemInterface',
    )
    robot_description_fixed = robot_description_fixed.replace(
        'package://nova_sim/',
        'package://nova_sim_mujoco/',
    )
    robot_description_fixed = robot_description_fixed.replace(
        '$(find nova_sim)',
        '$(find nova_sim_mujoco)',
    )
    robot_description_fixed = robot_description_fixed.replace(
        '/install/nova_sim/share/nova_sim/',
        '/install/nova_sim_mujoco/share/nova_sim_mujoco/',
    )
    # Keep a full robot description for robot_state_publisher/controller_manager.
    robot_description = {
        'robot_description': ParameterValue(value=robot_description_fixed, value_type=str)
    }

    # MuJoCo MJCF converter uses the same URDF (incl. ArUco boards) so TF and visuals match.
    expanded_urdf_file = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.urdf',
        prefix='nova_sim_mujoco_expanded_',
        delete=False,
    )
    expanded_urdf_file.write(robot_description_fixed)
    expanded_urdf_file.flush()
    expanded_urdf_file.close()

    controller_params = PathJoinSubstitution([FindPackageShare('nova_sim_mujoco'), 'config', 'nova_position.yaml'])
    plugins_params = PathJoinSubstitution(
        [FindPackageShare('mujoco_ros2_control_demos'), 'config', 'mujoco_ros2_control_plugins.yaml']
    )

    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
        output='screen'
    )
    tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    mjcf_converter = Node(
        package='mujoco_ros2_control',
        executable='robot_description_to_mjcf.sh',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--urdf', expanded_urdf_file.name,
            '--mujoco_inputs',
            os.path.join(your_pkg, 'config', 'mujoco_inputs.xml'),
            '--publish_topic', '/mujoco_robot_description',
        ],
    )

    ros2_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': True},
            ParameterFile(controller_params),
            ParameterFile(plugins_params),
        ],
        remappings=[('~/robot_description', '/robot_description')],
        on_exit=Shutdown(),
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controller_params],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', controller_params],
        output='screen',
    )

    # Publish an explicit zero pose once controllers are up, to avoid random startup posture.
    # arm_controller currently controls all 28 joints.
    zero_pose_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/arm_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            "{data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}",
        ],
        output='screen',
    )
    delayed_zero_pose = TimerAction(
        period=4.0,
        actions=[zero_pose_command],
    )

    return [
        tf_world_base,
        tf_footprint_base,
        robot_state_publisher,
        mjcf_converter,
        ros2_control_node,
        joint_state_spawner,
        arm_controller_spawner,
        delayed_zero_pose,
    ]


def generate_launch_description():
    your_pkg = FindPackageShare('nova_sim_mujoco').find('nova_sim_mujoco')
    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(your_pkg, 'urdf', 'nova_robot_position.urdf'),
        description='URDF file path'
    )

    return LaunchDescription([
        urdf_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
