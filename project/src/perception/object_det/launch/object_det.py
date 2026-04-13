import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明launch参数
    cam_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='',
        description='Camera ID to use for YOLO detection'
    )

    color_image_topic_arg = DeclareLaunchArgument(
        'color_image_topic',
        default_value='',
        description='Color image topic name (empty to auto-generate from camera_id and camera_type)'
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='',
        description='Depth image topic name (empty to get from param server)'
    )

    cam_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='',
        description='Camera info topic name (empty to get from param server)'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='',
        description='Camera type: realsense, orbbec (used to auto-generate topic names)'
    )

    # 创建节点的函数
    def create_object_det_node(context):
        camera_id = LaunchConfiguration('camera_id').perform(context)
        camera_type = LaunchConfiguration('camera_type').perform(context)

        # 根据相机类型确定实际的话题名称（如果launch文件没有指定）
        if camera_type:
            if camera_type == 'orbbec':
                color_image_topic = f'/ob_camera_{camera_id}/color/image_raw'
                depth_image_topic = f'/ob_camera_{camera_id}/depth/image_raw'
                camera_info_topic = f'/ob_camera_{camera_id}/depth/camera_info'
            elif camera_type == 'realsense':
                color_image_topic = f'/camera/rs_camera_{camera_id}/color/image_raw'
                depth_image_topic = f'/camera/rs_camera_{camera_id}/aligned_depth_to_color/image_raw'
                camera_info_topic = f'/camera/rs_camera_{camera_id}/aligned_depth_to_color/camera_info'
                # color_image_topic = f'/camera/rs_camera_{camera_id}/color/image_rect_raw' #405话题名
                # depth_image_topic = f'/camera/rs_camera_{camera_id}/depth/image_rect_raw'
                # camera_info_topic = f'/camera/rs_camera_{camera_id}/color/camera_info'
            else:
                # 默认使用参数服务器的逻辑话题名
                color_image_topic = f'/cam_{camera_id}/src_color_image'
                depth_image_topic = f'/cam_{camera_id}/src_depth_image'
                camera_info_topic = f'/cam_{camera_id}/intrinsics'

        # 不使用namespace，直接使用完整路径
        node_name = f'object_det_node_{camera_id}'

        # 构建参数字典
        params_dict = {
            'camera_id': LaunchConfiguration('camera_id'),
            'camera_type': LaunchConfiguration('camera_type'),
        }

        # 如果launch文件提供了话题名，直接使用
        if LaunchConfiguration('color_image_topic').perform(context):
            color_image_topic = LaunchConfiguration('color_image_topic').perform(context)
        if LaunchConfiguration('depth_image_topic').perform(context):
            depth_image_topic = LaunchConfiguration('depth_image_topic').perform(context)
        if LaunchConfiguration('camera_info_topic').perform(context):
            camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)

        params_dict['color_image_topic'] = color_image_topic
        params_dict['depth_image_topic'] = depth_image_topic
        params_dict['camera_info_topic'] = camera_info_topic

        object_det_node = Node(
            package='object_det',
            executable='object_det',
            name=node_name,
            output='screen',
            parameters=[params_dict]
        )
        return [object_det_node]

    # 使用OpaqueFunction来动态创建节点
    object_det_node_creator = OpaqueFunction(function=create_object_det_node)

    ld = LaunchDescription()
    ld.add_action(cam_id_arg)
    ld.add_action(camera_type_arg)
    ld.add_action(color_image_topic_arg)
    ld.add_action(depth_image_topic_arg)
    ld.add_action(cam_info_topic_arg)
    ld.add_action(object_det_node_creator)
    return ld
