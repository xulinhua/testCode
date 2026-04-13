#!/usr/bin/env python3
"""
Test script for YOLO + Grasping system
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import cv2
import numpy as np

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image, '/top_camera/image_raw', self.image_callback, 10)

        # Subscribe to detected objects
        self.objects_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.objects_callback, 10)

        self.bridge = cv2.CvBridge()
        self.image_count = 0
        self.detection_count = 0

        self.get_logger().info('System Tester Started')

    def image_callback(self, msg):
        """Check if we're receiving images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_count += 1

            if self.image_count % 30 == 0:  # Log every 30 frames
                self.get_logger().info(f'Received {self.image_count} camera frames')

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def objects_callback(self, msg):
        """Check if we're detecting objects"""
        self.detection_count += 1
        num_objects = len(msg.poses)

        self.get_logger().info(
            f'Detection #{self.detection_count}: Found {num_objects} objects')

        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f'  Object {i+1}: Position ({pose.position.x:.3f}, '
                f'{pose.position.y:.3f}, {pose.position.z:.3f})')

    def run_test(self):
        """Run the test sequence"""
        self.get_logger().info('Starting YOLO + Grasping System Test')
        self.get_logger().info('=====================================')

        # Test duration (seconds)
        test_duration = 60
        start_time = self.get_clock().now()

        while rclpy.ok():
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9

            if elapsed >= test_duration:
                break

            # Log status every 10 seconds
            if int(elapsed) % 10 == 0 and elapsed > 0:
                self.get_logger().info(
                    f'Test Progress: {int(elapsed)}/{test_duration}s - '
                    f'Images: {self.image_count}, Detections: {self.detection_count}')

            rclpy.spin_once(self, timeout_sec=1.0)

        # Print test summary
        self.get_logger().info('Test Summary')
        self.get_logger().info('=============')
        self.get_logger().info(f'Total Duration: {elapsed:.1f} seconds')
        self.get_logger().info(f'Camera Frames: {self.image_count}')
        self.get_logger().info(f'Object Detections: {self.detection_count}')
        self.get_logger().info(f'FPS: {self.image_count/elapsed:.2f}')
        self.get_logger().info(f'Detection Rate: {self.detection_count/elapsed:.2f} Hz')

def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()

    try:
        tester.run_test()
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()