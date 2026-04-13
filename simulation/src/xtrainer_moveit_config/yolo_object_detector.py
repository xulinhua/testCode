#!/usr/bin/env python3
"""
YOLO Object Detection Node for ROS2
Detects objects using YOLOv5 and publishes their poses
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class YOLOObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # You can use yolov5m.pt or yolov11.pt for better accuracy

        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.detected_objects_pub = self.create_publisher(
            PoseArray, '/detected_objects', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/yolo_debug_image', 10)

        # Storage for latest data
        self.depth_image = None
        self.camera_info = None
        self.camera_frame = "camera_link"

        # Target classes for grasping (COCO dataset classes)
        self.target_classes = {
            'cup': 41, 'bottle': 39, 'bowl': 41, 'apple': 47, 'orange': 49,
            'banana': 46, 'cell phone': 67, 'book': 73, 'laptop': 63,
            'mouse': 64, 'keyboard': 66, 'remote': 65, 'teddy bear': 77, 'kite':33
        }

        self.get_logger().info('YOLO Object Detector initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_info = msg

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            # Debug: print depth image statistics
            if self.depth_image is not None:
                min_depth = np.min(self.depth_image)
                max_depth = np.max(self.depth_image)
                mean_depth = np.mean(self.depth_image)
                self.get_logger().info(f'Depth image stats - min: {min_depth}, max: {max_depth}, mean: {mean_depth}')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def image_callback(self, msg):
        """Process RGB image and detect objects"""
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera info...')
            return

        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLO detection - optimized for Gazebo environment
            results = self.model(cv_image, conf=0.25, iou=0.3, imgsz=640)

            # Debug: print total detections
            total_detections = 0
            for result in results:
                if result.boxes is not None:
                    total_detections += len(result.boxes)
            if total_detections > 0:
                self.get_logger().info(f'Total detections: {total_detections}')

            # Process detections
            detected_poses = self.process_detections(results)

            # Publish detected objects
            if detected_poses:
                self.get_logger().info(f'Publishing {len(detected_poses)} detected objects to /detected_objects')
                pose_array = PoseArray()
                pose_array.header = Header()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = "base_link"
                pose_array.poses = detected_poses
                self.detected_objects_pub.publish(pose_array)
            else:
                self.get_logger().info('No valid 3D poses computed')

            # Publish debug image
            debug_image = self.draw_detections(cv_image, results)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def process_detections(self, results):
        """Process YOLO detections and convert to 3D poses"""
        detected_poses = []

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get class name
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]

                    # Check if this is a target class
                    # Temporary: show all detections for debugging
                    if class_id not in self.target_classes.values():
                        self.get_logger().info(f'Detected non-target object: {class_name} (class_id: {class_id})')
                        continue

                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()

                    self.get_logger().info(f'Target object: {class_name}, confidence: {confidence:.2f}, bbox: [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]')

                    # Calculate center point
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Get 3D position
                    self.get_logger().info(f'Getting 3D position for pixel: ({center_x}, {center_y})')
                    pose_3d = self.get_3d_position(center_x, center_y)

                    if pose_3d:
                        detected_poses.append(pose_3d)
                        self.get_logger().info(
                            f'Detected {class_name} at ({pose_3d.position.x:.3f}, '
                            f'{pose_3d.position.y:.3f}, {pose_3d.position.z:.3f}) '
                            f'with confidence {confidence:.2f}')

        return detected_poses

    def get_3d_position(self, pixel_x, pixel_y):
        """Convert pixel coordinates to 3D position using depth image"""
        self.get_logger().info(f'get_3d_position called with pixel: ({pixel_x}, {pixel_y})')

        if self.depth_image is None:
            self.get_logger().warning('Depth image is None')
            return None

        try:
            # Get depth value at pixel location
            depth_value = self.depth_image[pixel_y, pixel_x]
            self.get_logger().info(f'Depth value at ({pixel_x}, {pixel_y}): {depth_value}')

            # Convert depth to meters
            # Handle invalid depth values (all 1s indicates sensor issue)
            if depth_value == 1:
                self.get_logger().warning('Depth sensor appears to be returning invalid values (all 1s)')
                # Use a fixed estimated distance for demonstration
                depth_meters = 0.5  # 50cm estimated distance
            elif depth_value > 0 and depth_value <= 10:
                depth_meters = depth_value
            else:
                depth_meters = depth_value / 1000.0
            self.get_logger().info(f'Depth in meters: {depth_meters}')

            # Check for invalid depth values
            if depth_meters <= 0.1 or depth_meters > 10.0:
                self.get_logger().warning(f'Invalid depth value: {depth_meters} meters')
                return None

            # Get camera intrinsic parameters
            fx = self.camera_info.k[0]  # focal length x
            fy = self.camera_info.k[4]  # focal length y
            cx = self.camera_info.k[2]  # principal point x
            cy = self.camera_info.k[5]  # principal point y

            # Convert pixel coordinates to camera coordinates
            camera_x = float((pixel_x - cx) * depth_meters / fx)
            camera_y = float((pixel_y - cy) * depth_meters / fy)
            camera_z = float(depth_meters)

            self.get_logger().info(f'Camera coordinates: x={camera_x:.3f}, y={camera_y:.3f}, z={camera_z:.3f}')

            # Debug: 手动验证计算
            self.get_logger().info(f'Manual check:')
            self.get_logger().info(f'  Pixel: ({pixel_x}, {pixel_y})')
            self.get_logger().info(f'  Depth: {depth_meters:.3f}m')
            self.get_logger().info(f'  Camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')
            self.get_logger().info(f'  Expected object position: (0.41, -0.36, 0.06)')

            # Create pose in camera frame
            camera_pose = Pose()
            camera_pose.position.x = camera_x
            camera_pose.position.y = camera_y
            camera_pose.position.z = camera_z
            camera_pose.orientation.w = 1.0  # No rotation

            # Transform to base_link frame using TF
            try:
                self.get_logger().info('Looking up TF transform...')
                transform = self.tf_buffer.lookup_transform(
                    'base_link', self.camera_frame, rclpy.time.Time())

                # Create a point with the camera coordinates
                camera_point = geometry_msgs.msg.PointStamped()
                camera_point.header.frame_id = self.camera_frame
                camera_point.header.stamp = self.get_clock().now().to_msg()
                camera_point.point.x = camera_x
                camera_point.point.y = camera_y
                camera_point.point.z = camera_z

                # Transform the point
                base_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)

                # Create a pose from the transformed point
                base_pose = Pose()
                base_pose.position.x = base_point.point.x
                base_pose.position.y = base_point.point.y
                base_pose.position.z = base_point.point.z
                base_pose.orientation.w = 1.0

                self.get_logger().info(f'Successfully computed 3D pose: x={base_pose.position.x:.3f}, y={base_pose.position.y:.3f}, z={base_pose.position.z:.3f}')
                return base_pose

            except Exception as e:
                self.get_logger().error(f'TF transform error: {e}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error getting 3D position: {e}')
            return None

    def draw_detections(self, cv_image, results):
        """Draw bounding boxes and labels on image"""
        debug_image = cv_image.copy()

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                    # Get class name and confidence
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    confidence = box.conf[0].cpu().numpy()

                    # Check if target class
                    if class_id in self.target_classes.values():
                        color = (0, 255, 0)  # Green for target objects
                    else:
                        color = (0, 0, 255)  # Red for non-target objects

                    # Draw bounding box
                    cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)

                    # Draw label
                    label = f'{class_name}: {confidence:.2f}'
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    cv2.rectangle(debug_image, (x1, y1 - label_size[1] - 10),
                                 (x1 + label_size[0], y1), color, -1)
                    cv2.putText(debug_image, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return debug_image

def main(args=None):
    rclpy.init(args=args)
    detector = YOLOObjectDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()