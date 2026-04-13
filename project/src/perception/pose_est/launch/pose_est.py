import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID for PoseEst node'
    )

    color_image_topic_arg = DeclareLaunchArgument(
        'color_image_topic',
        default_value='',
        description='Color image topic name (empty to get from param server)'
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='',
        description='Depth image topic name (empty to get from param server)'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='',
        description='Camera info topic name (empty to get from param server)'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='',
        description='Camera type: realsense, orbbec (used to auto-generate topic names)'
    )

    mesh_path_arg = DeclareLaunchArgument(
        'mesh_path',
        default_value='',
        description='Path to mesh file (.obj)'
    )

    target_name_arg = DeclareLaunchArgument(
        'target_name',
        default_value='object',
        description='Target object name'
    )

    target_class_name_arg = DeclareLaunchArgument(
        'target_class_name',
        default_value='',
        description='Target class name for YOLO segmentation (empty for auto-select)'
    )

    def create_pose_est_node(context):
        camera_id = LaunchConfiguration('camera_id').perform(context)
        camera_type = LaunchConfiguration('camera_type').perform(context)

        # Build parameter dict
        params_dict = {
            'camera_id': int(camera_id),
            'target_name': LaunchConfiguration('target_name'),
            'target_class_name': LaunchConfiguration('target_class_name'),
        }

        if camera_type:
            params_dict['camera_type'] = camera_type
            # Auto-generate topic names based on camera type
            color_image_topic = LaunchConfiguration('color_image_topic').perform(context)
            depth_image_topic = LaunchConfiguration('depth_image_topic').perform(context)
            camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)

            if not color_image_topic or not depth_image_topic or not camera_info_topic:
                if camera_type == 'orbbec':
                    if not color_image_topic:
                        color_image_topic = f'/ob_camera_{camera_id}/color/image_raw'
                    if not depth_image_topic:
                        depth_image_topic = f'/ob_camera_{camera_id}/depth/image_raw'
                    if not camera_info_topic:
                        camera_info_topic = f'/ob_camera_{camera_id}/depth/camera_info'
                elif camera_type == 'realsense':
                    if not color_image_topic:
                        color_image_topic = f'/camera/rs_camera_{camera_id}/color/image_raw'
                    if not depth_image_topic:
                        depth_image_topic = f'/camera/rs_camera_{camera_id}/aligned_depth_to_color/image_raw'
                    if not camera_info_topic:
                        camera_info_topic = f'/camera/rs_camera_{camera_id}/aligned_depth_to_color/camera_info'
                else:
                    if not color_image_topic:
                        color_image_topic = f'/cam_{camera_id}/src_color_image'
                    if not depth_image_topic:
                        depth_image_topic = f'/cam_{camera_id}/src_depth_image'
                    if not camera_info_topic:
                        camera_info_topic = f'/cam_{camera_id}/intrinsics'

            params_dict['color_image_topic'] = color_image_topic
            params_dict['depth_image_topic'] = depth_image_topic
            params_dict['camera_info_topic'] = camera_info_topic
        else:
            color_topic = LaunchConfiguration('color_image_topic').perform(context)
            depth_topic = LaunchConfiguration('depth_image_topic').perform(context)
            cam_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
            if color_topic:
                params_dict['color_image_topic'] = color_topic
            if depth_topic:
                params_dict['depth_image_topic'] = depth_topic
            if cam_info_topic:
                params_dict['camera_info_topic'] = cam_info_topic

        mesh_path = LaunchConfiguration('mesh_path').perform(context)
        if mesh_path:
            params_dict['mesh_path'] = mesh_path

        node_name = f'pose_est_node_{camera_id}'
        pose_est_node = Node(
            package='pose_est',
            executable='pose_est',
            name=node_name,
            output='screen',
            parameters=[params_dict]
        )
        return [pose_est_node]

    pose_est_node_creator = OpaqueFunction(function=create_pose_est_node)

    ld = LaunchDescription()
    ld.add_action(camera_id_arg)
    ld.add_action(color_image_topic_arg)
    ld.add_action(depth_image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(camera_type_arg)
    ld.add_action(mesh_path_arg)
    ld.add_action(target_name_arg)
    ld.add_action(target_class_name_arg)
    ld.add_action(pose_est_node_creator)
    return ld
