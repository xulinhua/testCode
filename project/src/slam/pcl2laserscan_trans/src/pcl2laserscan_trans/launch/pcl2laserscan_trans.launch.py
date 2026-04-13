import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
import sys

try:
    from ament_index_python.packages import get_package_share_directory
    _config_dir = get_package_share_directory('pcl2laserscan_trans')
    _config_file = os.path.join(_config_dir, 'config', 'pcl_cloud_config.yaml')
except Exception:
    _config_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _config_file = os.path.join(_config_dir, 'config', 'pcl_cloud_config.yaml')

# 导入配置管理类
sys.path.append(os.path.join(_config_dir, 'lib'))
try:
    from pcl2laserscan_trans.pcl_config_manager import PclConfigMgr
except ImportError:
    config_file = _config_file
    
    # 简单的配置加载函数
    def load_config():
        import yaml
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    config = load_config()
    
    # 模拟配置管理类
    class PclConfigMgr:
        def __init__(self):
            self.config = config
        
        def get_system_config(self):
            return self.config['system']
        
        def get_launch_config(self):
            return self.config['launch']
        
        def get_camera_config(self):
            return self.config['camera']
        
        def get_lidar_config(self):
            return self.config['lidar']
        
        def get_laserscan_to_pointcloud_config(self):
            return self.config['laserscan_to_pointcloud']
        
        def get_tf_config(self):
            return self.config['tf']

def generate_launch_description():
    # 使用包 share 目录下的 config（source/install 均正确）
    config_file = _config_file
    config_mgr = PclConfigMgr()
    
    # 获取配置
    system_config = config_mgr.get_system_config()
    launch_config = config_mgr.get_launch_config()
    camera_config = config_mgr.get_camera_config()
    lidar_config = config_mgr.get_lidar_config()
    tf_config = config_mgr.get_tf_config()
    
    # 设置日志目录
    log_dir = system_config['log_dir']
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir
    
    # 定义启动参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value=launch_config.get('default_mode', 'lidar_only'),
        description='启动模式: camera_only, lidar_only, both',
        choices=['camera_only', 'lidar_only', 'both']
    )

    # 话题名参数
    camera_input_topic_arg = DeclareLaunchArgument(
        'camera_input_topic',
        default_value=launch_config.get('camera_input_topic', '/camera/depth/color/points'),
        description='相机输入点云话题'
    )
    
    camera_output_topic_arg = DeclareLaunchArgument(
        'camera_output_topic',
        default_value=launch_config.get('camera_output_topic', '/camera/laser_scan'),
        description='相机输出激光扫描话题'
    )
    
    lidar_input_topic_arg = DeclareLaunchArgument(
        'lidar_input_topic',
        default_value=launch_config.get('lidar_input_topic', '/sensor/lidar_3d/top/point_cloud'),
        description='雷达输入点云话题'
    )
    
    lidar_output_topic_arg = DeclareLaunchArgument(
        'lidar_output_topic',
        default_value=launch_config.get('lidar_output_topic', '/lidar/laser_scan'),
        description='雷达输出激光扫描话题'
    )
    
    # 获取启动参数
    mode = LaunchConfiguration('mode')
    camera_input_topic = LaunchConfiguration('camera_input_topic')
    camera_output_topic = LaunchConfiguration('camera_output_topic')
    lidar_input_topic = LaunchConfiguration('lidar_input_topic')
    lidar_output_topic = LaunchConfiguration('lidar_output_topic')
    
    # 根据模式决定启动哪些节点
    nodes_to_launch = []
    
    # 使用Python表达式创建条件
    # 注意：这里使用PythonExpression来正确解析字符串值
    camera_condition = IfCondition(
        PythonExpression([
            "'", mode, "' == 'camera_only' or '" , mode, "' == 'both'"
        ])
    )
    
    lidar_condition = IfCondition(
        PythonExpression([
            "'", mode, "' == 'lidar_only' or '" , mode, "' == 'both'"
        ])
    )
    
    # 创建相机参数（话题名直接使用 YAML 配置，确保生效）
    camera_params = {
        'log_dir': log_dir,
        'cloud_topic': launch_config['camera_input_topic'],
        'target_frame': camera_config['target_frame'],
        'min_height': camera_config['min_height'],
        'max_height': camera_config['max_height'],
        'bSaveLogInfo2Files': camera_config['bSaveLogInfo2Files'],
        'bOutputToTerminal': camera_config['bOutputToTerminal'],
        'range_min': camera_config['range_min'],
        'range_max': camera_config['range_max'],
        'angle_min': camera_config['angle_min'],
        'angle_max': camera_config['angle_max'],
        'angle_increment': camera_config['angle_increment'],
        'voxel_leaf_size': camera_config['voxel_leaf_size'],
        'sor_mean_k': camera_config['sor_mean_k'],
        'sor_stddev_mul_thresh': camera_config['sor_stddev_mul_thresh']
    }
    
    # 相机节点：订阅 cloud_topic（参数），发布 camera_scan（remap 为配置中的输出话题）
    nodes_to_launch.append(
        Node(
            package='pcl2laserscan_trans',
            executable='camera_pcl2laserscan_node',
            name='camera_pcl2laserscan',
            output='screen',
            parameters=[camera_params],
            remappings=[
                ('camera_scan', launch_config['camera_output_topic'])
            ],
            condition=camera_condition
        )
    )
    
    # 相机TF发布器
    # 是否由本 launch 发布 TF（仿真/驱动已发布时设为 false）
    publish_camera_tf = tf_config.get('publish_camera_tf', True)
    publish_lidar_tf = tf_config.get('publish_lidar_tf', True)

    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            str(tf_config['camera']['x']),
            str(tf_config['camera']['y']),
            str(tf_config['camera']['z']),
            str(tf_config['camera']['roll']),
            str(tf_config['camera']['pitch']),
            str(tf_config['camera']['yaw']),
            tf_config['camera']['parent_frame'],
            tf_config['camera']['child_frame']
        ],
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(
            PythonExpression([
                "('", mode, "' == 'camera_only' or '", mode, "' == 'both') and ", str(publish_camera_tf), ""
            ])
        )
    )
    
    # 雷达TF发布器
    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=[
            str(tf_config['lidar']['x']),
            str(tf_config['lidar']['y']),
            str(tf_config['lidar']['z']),
            str(tf_config['lidar']['roll']),
            str(tf_config['lidar']['pitch']),
            str(tf_config['lidar']['yaw']),
            tf_config['lidar']['parent_frame'],
            tf_config['lidar']['child_frame']
        ],
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(
            PythonExpression([
                "('", mode, "' == 'lidar_only' or '", mode, "' == 'both') and ", str(publish_lidar_tf), ""
            ])
        )
    )
    
    # 添加节点到列表
    nodes_to_launch.extend([camera_tf_publisher, lidar_tf_publisher])

    # 激光雷达节点（当模式为lidar_only或both时启动）
    # 激光雷达参数
    lidar_params = {
        'target_frame': lidar_config['target_frame'],
        'min_height': lidar_config['min_height'],
        'max_height': lidar_config['max_height'],
        'range_min': lidar_config['range_min'],
        'tilt_compensation_angle': lidar_config['tilt_compensation_angle'],
        'tilt_axis': lidar_config['tilt_axis'],
        'debug': lidar_config['debug'],
        'filter_mean_k': lidar_config['filter_mean_k'],
        'filter_stddev': lidar_config['filter_stddev'],
        'voxel_leaf_size': lidar_config['voxel_leaf_size']
    }
    
    # 雷达节点（点云→激光扫描）：直接订阅雷达点云话题，发布激光扫描
    lidar_pcl2laserscan_node = Node(
        package='pcl2laserscan_trans',
        executable='lidar_pcl2laserscan_node',
        name='lidar_pcl2laserscan',
        output='screen',
        parameters=[lidar_params],
        remappings=[
            ('cloud_in', launch_config['lidar_input_topic']),
            ('laser_scan', launch_config['lidar_output_topic'])
        ],
        emulate_tty=True,
        condition=lidar_condition
    )

    nodes_to_launch.append(lidar_pcl2laserscan_node)
    
    return LaunchDescription([
        # 启动模式参数
        mode_arg,
        # 话题名参数
        camera_input_topic_arg,
        camera_output_topic_arg,
        lidar_input_topic_arg,
        lidar_output_topic_arg,
        # 所有节点
        *nodes_to_launch
    ])