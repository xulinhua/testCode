from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    @brief DDS服务统一启动文件
    @details 支持服务端和客户端模式启动，通过mode参数控制
    @note 在Jetson主板上使用mode=server，在x86主板上使用mode=client
    """
    
    # 定义启动参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='server',
        description='运行模式: server(服务端) 或 client(客户端)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='DDS服务配置文件路径'
    )
    
    # 创建DDS服务节点
    dds_service = Node(
        package='dds_comm',
        executable='dds_service',
        name='dds_service',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'config_file': LaunchConfiguration('config_file')
        }]
    )
    
    return LaunchDescription([
        mode_arg,
        config_file_arg,
        dds_service
    ])