#!/usr/bin/env python3
"""
Launch only YOLO detection and grasping nodes
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # YOLO Object Detector Node
    yolo_detector = Node(
        package='yolo_object_detection',
        executable='yolo_object_detector',
        name='yolo_object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolov5s.pt',
            'confidence_threshold': 0.5,
            'camera_topic': '/top_camera/image_raw',
            'depth_topic': '/top_camera/depth/image_raw',
            'camera_info_topic': '/top_camera/camera_info',
            'target_classes': ['cup', 'bottle', 'bowl', 'apple', 'orange', 'banana', 'cell phone', 'book']
        }],
        remappings=[
            ('/top_camera/image_raw', '/top_camera/image_raw'),
            ('/top_camera/depth/image_raw', '/top_camera/depth/image_raw'),
            ('/top_camera/camera_info', '/top_camera/camera_info')
        ]
    )

    # Grasp Executor Node
    grasp_executor = Node(
        package='grasp_executor',
        executable='grasp_executor',
        name='grasp_executor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pre_grasp_height': 0.1,
            'grasp_height': 0.02,
            'retract_height': 0.15,
            'arm_group_name': 'arm',
            'gripper_group_name': 'gripper',
            'end_effector_link': 'end_effector_link'
        }]
    )

    # Optional: RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=["-d", "/home/hoson/xtrainer_moveit_config/config/moveit.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),

        # Launch perception and grasping nodes
        yolo_detector,
        grasp_executor,

        # Optional: RViz
        rviz_node
    ])