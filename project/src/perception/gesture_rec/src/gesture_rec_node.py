#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# 初始化MediaPipe相关类
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# 导入手势识别算法模块
from gesture_detector import GestureDetector, HAND_CONNECTIONS


class GestureRecognitionNode(Node):
    """手势识别ROS2节点"""

    def __init__(self):
        super().__init__('gesture_recognition_node')

        # 声明参数
        self.declare_parameter('camera_type', 'Realsense')
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('move_thr', 50)

        # 获取参数
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        self.move_thr = self.get_parameter('move_thr').get_parameter_value().integer_value

        # 根据相机类型设置订阅话题
        if self.camera_type == "Gemini":
            self.color_image_topic_ = "/camera/color/image_raw"
        else:  # Realsense
            self.color_image_topic_ = "/camera/camera/color/image_raw"

        self.get_logger().info(f'Camera type: {self.camera_type}')
        self.get_logger().info(f'Subscribing to: {self.color_image_topic_}')

        # 初始化cv_bridge
        self.bridge = CvBridge()

        # 初始化MediaPipe HandLandmarker
        self._init_hand_landmarker()

        # 初始化手势识别器（独立算法模块）
        self.gesture_detector = GestureDetector(move_thr=self.move_thr)

        # 创建图像订阅者
        sensor_data_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.color_image_topic_,
            self.image_callback,
            sensor_data_qos
        )

        # 创建手势识别结果发布者
        self.gesture_pub = self.create_publisher(
            String,
            '/gesture_res',
            10
        )

        # 创建Wave手势指令发布者
        self.wave_command_pub = self.create_publisher(
            String,
            '/gesture_command',
            10
        )

        # 用于避免重复触发Wave指令
        self.wave_triggered = False
        self.wave_cooldown_frames = 30  # Wave指令冷却帧数（约1秒）
        self.wave_cooldown_counter = 0

        # 创建带标注图像的发布者（用于RViz2可视化）
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/gesture_image',
            1
        )

        # FPS计算
        self.prev_time = 0
        self.frame_count = 0

        self.get_logger().info('Gesture Recognition Node initialized successfully')

    def _init_hand_landmarker(self):
        """初始化MediaPipe HandLandmarker"""
        try:
            # 获取模型文件路径
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory('gesture_recognition')
            model_path = os.path.join(package_path, 'model', 'hand_landmarker.task')

            if not os.path.exists(model_path):
                self.get_logger().error(f'Model file not found: {model_path}')
                raise FileNotFoundError(f'Model file not found: {model_path}')

            # 创建HandLandmarker
            options = HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                num_hands=2,
                min_hand_detection_confidence=0.7,
                min_hand_presence_confidence=0.7,
                min_tracking_confidence=0.7
            )
            self.detector = HandLandmarker.create_from_options(options)
            self.get_logger().info(f'MediaPipe HandLandmarker loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize HandLandmarker: {e}')
            raise

    def draw_landmarks(self, image, landmarks, connections):
        """绘制手部关键点和连接线"""
        height, width = image.shape[:2]

        # 绘制连接线
        for connection in connections:
            start_idx, end_idx = connection
            if start_idx < len(landmarks) and end_idx < len(landmarks):
                start = landmarks[start_idx]
                end = landmarks[end_idx]
                cv2.line(image, (int(start.x * width), int(start.y * height)),
                         (int(end.x * width), int(end.y * height)), (0, 255, 0), 2)

        # 绘制关键点
        for landmark in landmarks:
            cv2.circle(image, (int(landmark.x * width), int(landmark.y * height)), 5, (255, 0, 0), -1)

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换ROS图像消息为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 转换为MediaPipe Image格式
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

            # 检测手部
            results = self.detector.detect(mp_image)

            if results.hand_landmarks:
                for idx, hand_landmarks in enumerate(results.hand_landmarks):
                    # 绘制手部关键点和连接线
                    self.draw_landmarks(frame, hand_landmarks, HAND_CONNECTIONS)

                    # 获取手的左右标签
                    handedness = results.handedness[idx]
                    hand_label = handedness[0].display_name

                    # 使用手势识别器处理关键点，传递图像尺寸和手部信息
                    gesture_result = self.gesture_detector.process_hand_landmarks(
                        hand_landmarks,
                        idx,
                        frame.shape[1],  # image_width
                        frame.shape[0],  # image_height
                        hand_info={'handedness': hand_label}  # 传递左右手信息
                    )

                    if gesture_result:
                        gesture_str = gesture_result['gesture']
                        hand_local = gesture_result['hand_local']
                        wrist = hand_local[0]
                        palm_facing_angle = gesture_result['palm_facing_angle']

                        # 在图像上显示手势名称和调试信息
                        cv2.putText(frame, f"{hand_label} Hand: {gesture_str}", (int(wrist[0]), int(wrist[1]) - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(frame, f"angle: {palm_facing_angle:.1f}", (int(wrist[0]), int(wrist[1]) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

                        # 打印识别结果
                        if gesture_str:
                            self.get_logger().info(f'Gesture Detected: {hand_label} -> {gesture_str}')

                            # 发布手势识别结果
                            gesture_msg = String()
                            gesture_msg.data = gesture_str
                            self.gesture_pub.publish(gesture_msg)

                            # Wave手势特殊处理：触发指令
                            if gesture_str == "Wave" and not self.wave_triggered:
                                wave_command_msg = String()
                                wave_command_msg.data = "wave_greeting"  # 挥手问好指令
                                self.wave_command_pub.publish(wave_command_msg)
                                self.get_logger().info('Wave gesture detected: Sending wave_greeting command')
                                self.wave_triggered = True
                                self.wave_cooldown_counter = self.wave_cooldown_frames
                            elif gesture_str != "Wave":
                                self.wave_triggered = False

            # 计算并显示FPS
            current_time = time.time()
            if self.prev_time != 0:
                fps = 1 / (current_time - self.prev_time)
                self.frame_count += 1
                if self.frame_count % 30 == 0:  # 每30帧打印一次
                    self.get_logger().info(f'FPS: {fps:.2f}')
            else:
                fps = 0
            self.prev_time = current_time
            cv2.putText(frame, f"FPS: {fps:.2f}", (frame.shape[1] - 150, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

            # 发布带标注的图像（始终发布以支持RViz2可视化）
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.annotated_image_pub.publish(annotated_msg)

            # Wave指令冷却处理
            if self.wave_triggered:
                self.wave_cooldown_counter -= 1
                if self.wave_cooldown_counter <= 0:
                    self.wave_triggered = False
                    self.wave_cooldown_counter = 0

            # 如果启用了可视化，显示图像窗口
            if self.enable_visualization:
                cv2.imshow('Hand Gesture Recognition', frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    gesture_recognition_node = GestureRecognitionNode()

    try:
        rclpy.spin(gesture_recognition_node)
    except KeyboardInterrupt:
        gesture_recognition_node.get_logger().info('Node stopped by user')
    finally:
        gesture_recognition_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
