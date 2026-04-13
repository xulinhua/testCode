from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    生成ArUco检测节点的启动描述
    """
    
    # 声明启动参数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='订阅的图像话题'
    )
    
    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.1',
        description='ArUco标记的实际长度（米）'
    )
    
    aruco_dict_type_arg = DeclareLaunchArgument(
        'aruco_dict_type',
        default_value='10',  # cv::aruco::DICT_5X5_100
        description='ArUco字典类型'
    )
    
    # 创建ArUco检测节点
    marker_detect_node = Node(
        package='marker_detect_ros',
        executable='marker_detect_node',
        name='marker_detect_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'marker_length': LaunchConfiguration('marker_length'),
            'aruco_dict_type': LaunchConfiguration('aruco_dict_type'),
        }]
    )
    
    return LaunchDescription([
        image_topic_arg,
        marker_length_arg,
        aruco_dict_type_arg,
        marker_detect_node,
    ])
