import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

configurable_parameters = [
    {'name': 'camera_name', 'default': 'stereo_camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': 'camera', 'description': 'namespace for camera'},
    {'name': 'left_config_file', 'default': '', 'description': 'left camera config yaml file'},
    {'name': 'right_config_file', 'default': '', 'description': 'right camera config yaml file'},
    {'name': 'calibration_file_path', 'default': '', 'description': 'stereo calibration file path'},
    {'name': 'log_level', 'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name': 'output', 'default': 'screen', 'description': 'node output [screen|log]'},
    {'name': 'enable_pointcloud', 'default': 'false', 'description': 'enable pointcloud generation'},
    {'name': 'pointcloud_topic', 'default': 'points', 'description': 'pointcloud topic name'},
    {'name': 'enable_sync', 'default': 'false', 'description': 'enable sync mode for left and right images'},
    {'name': 'sync_threshold_ms', 'default': '50', 'description': 'sync threshold in milliseconds'},
    {'name': 'publish_tf', 'default': 'true', 'description': 'enable/disable publishing static TF'},
    {'name': 'tf_publish_rate', 'default': '10.0', 'description': 'rate in Hz for publishing dynamic TF'},
    {'name': 'base_frame_id', 'default': 'stereo_camera_link', 'description': 'Root frame of the sensors transform tree'},
    {'name': 'left_frame_id', 'default': 'left_camera_link', 'description': 'left camera frame ID'},
    {'name': 'right_frame_id', 'default': 'right_camera_link', 'description': 'right camera frame ID'},
    {'name': 'enable_rectified', 'default': 'true', 'description': 'publish rectified images'},
    {'name': 'enable_raw', 'default': 'false', 'description': 'publish raw images'},
    {'name': 'image_width', 'default': '640', 'description': 'image width'},
    {'name': 'image_height', 'default': '480', 'description': 'image height'},
    {'name': 'framerate', 'default': '30', 'description': 'camera frame rate'},
    {'name': 'auto_exposure', 'default': 'true', 'description': 'enable auto exposure'},
    {'name': 'exposure_time', 'default': '100', 'description': 'manual exposure time'},
    {'name': 'gain', 'default': '50', 'description': 'camera gain'},
    {'name': 'camera_id', 'default': '0', 'description': 'camera device ID'},
    {'name': 'camera_id_right', 'default': '1', 'description': 'right camera device ID'},
    {'name': 'mono_mode', 'default': 'true', 'description': 'run in mono mode (single camera)'},
    {'name': 'algorithm_type', 'default': 'bm', 'description': 'depth algorithm type [bm|sgbm]'},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    if not path_to_yaml or path_to_yaml == "''":
        return {}
    try:
        with open(path_to_yaml, "r") as f:
            return yaml.load(f, Loader=yaml.SafeLoader)
    except (FileNotFoundError, yaml.YAMLError) as e:
        print(f"Warning: Could not load YAML file {path_to_yaml}: {e}")
        return {}

def launch_setup(context, params):
    # 获取参数值的辅助函数
    def get_param(name, default=''):
        try:
            value = LaunchConfiguration(name).perform(context)
            return value if value is not None else default
        except:
            return default
    
    # 安全转换函数
    def safe_int(value, default=0):
        try:
            if value and str(value).strip() not in ["''", "none", "None"]:
                return int(value)
        except:
            pass
        return default
    
    def safe_bool(value, default=False):
        try:
            if value and str(value).strip() not in ["''", "none", "None"]:
                return str(value).lower() == 'true'
        except:
            pass
        return default
    
    def safe_float(value, default=0.0):
        try:
            if value and str(value).strip() not in ["''", "none", "None"]:
                return float(value)
        except:
            pass
        return default
    
    def safe_str(value, default=''):
        if value and str(value).strip() not in ["''", "none", "None"]:
            return str(value).strip()
        return default
    
    # 获取所有参数
    left_config_file = get_param('left_config_file')
    right_config_file = get_param('right_config_file')
    calibration_file_path = get_param('calibration_file_path')
    camera_name = get_param('camera_name', 'stereo_camera')
    camera_namespace = get_param('camera_namespace', 'camera')
    output = get_param('output', 'screen')
    log_level = get_param('log_level', 'info')
    
    camera_id = get_param('camera_id', '0')
    camera_id_right = get_param('camera_id_right', '1')
    
    image_width = get_param('image_width', '640')
    image_height = get_param('image_height', '480')
    framerate = get_param('framerate', '30')
    
    mono_mode = safe_bool(get_param('mono_mode', 'true'), True)
    algorithm_type = safe_str(get_param('algorithm_type', 'bm'))

    left_params_from_file = yaml_to_dict(left_config_file)
    right_params_from_file = yaml_to_dict(right_config_file)

    # 构建节点参数
    node_params = {}
    
    # 设备ID列表
    if mono_mode:
        node_params['video_device_id'] = [safe_int(camera_id, 0), safe_int(camera_id, 0)]
    else:
        node_params['video_device_id'] = [safe_int(camera_id, 0), safe_int(camera_id_right, 1)]
    
    # 图像尺寸
    node_params['image_size'] = [safe_int(image_width, 640), safe_int(image_height, 480)]
    
    # 基础参数
    node_params.update({
        'fps': safe_int(framerate, 30),
        'camera_name': safe_str(camera_name, 'stereo_camera'),
        'camera_namespace': safe_str(camera_namespace, 'camera'),
        'algorithm_type': safe_str(algorithm_type, 'bm'),
    })
    
    # 标定文件
    if calibration_file_path:
        node_params['calibration_file_path'] = calibration_file_path
    
    # 单目模式特殊设置
    if mono_mode:
        node_params.update({
            'enable_pointcloud': False,
            'enable_sync': False,
        })
    
    # 处理可选参数
    optional_params = [
        'auto_exposure', 'exposure_time', 'gain',
        'enable_rectified', 'enable_raw', 'enable_pointcloud',
        'enable_sync', 'sync_threshold_ms',
        'base_frame_id', 'left_frame_id', 'right_frame_id',
        'publish_tf', 'tf_publish_rate',
        'pointcloud_topic'
    ]
    
    for param_name in optional_params:
        value = get_param(param_name)
        if value:
            if value.lower() in ['true', 'false']:
                node_params[param_name] = value.lower() == 'true'
            elif value.replace('.', '').isdigit():
                node_params[param_name] = float(value) if '.' in value else int(value)
            else:
                node_params[param_name] = value

    # YAML配置文件参数
    if left_params_from_file:
        node_params['left_camera_config'] = left_params_from_file
    if right_params_from_file and not mono_mode:
        node_params['right_camera_config'] = right_params_from_file

    # 构建日志消息（保留基本的状态信息）
    log_messages = [
        LogInfo(msg=f"🚀 Starting CSI Camera ({'MONO' if mono_mode else 'STEREO'})"),
        LogInfo(msg=f"📷 Camera: {camera_name}, IDs: {camera_id if mono_mode else f'{camera_id}/{camera_id_right}'}"),
        LogInfo(msg=f"📐 Resolution: {image_width}x{image_height} @ {framerate}fps"),
    ]

    return log_messages + [
        Node(
            package="csi_cam_service",
            executable="stereo_cam_node",
            namespace=camera_namespace,
            name=camera_name,
            parameters=[node_params],
            output=output,
            arguments=['--ros-args', '--log-level', log_level],
            emulate_tty=True,
        )
    ]

def generate_launch_description():
    # 设置默认配置文件路径
    default_left_config = os.path.join(
        get_package_share_directory("csi_cam_service"), "config", "left.yaml"
    )
    default_right_config = os.path.join(
        get_package_share_directory("csi_cam_service"), "config", "right.yaml"
    )

    launch_args = declare_configurable_parameters(configurable_parameters)
    
    # 添加配置文件参数
    launch_args.append(
        DeclareLaunchArgument(
            'left_config_file',
            default_value=default_left_config if os.path.exists(default_left_config) else '',
            description='left camera config yaml file'
        )
    )
    
    launch_args.append(
        DeclareLaunchArgument(
            'right_config_file',
            default_value=default_right_config if os.path.exists(default_right_config) else '',
            description='right camera config yaml file'
        )
    )

    return LaunchDescription(
        launch_args + [
            OpaqueFunction(function=launch_setup, kwargs={'params': set_configurable_parameters(configurable_parameters)})
        ]
    )
