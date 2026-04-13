#!/usr/bin/env python3
"""
Simplified YOLO + Grasping Demo
"""
import rclpy
from rclpy.node import Node
import threading
import time

def main():
    rclpy.init()

    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Create YOLO detector
    from yolo_object_detector import YOLOObjectDetector
    detector = YOLOObjectDetector()
    executor.add_node(detector)

    # Create grasp executor
    from grasp_executor import GraspExecutor
    grasp_executor = GraspExecutor()
    executor.add_node(grasp_executor)

    # Start spinner thread
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    try:
        print("YOLO + Grasping Demo Started")
        print("==========================")
        print("1. Make sure Gazebo is running with objects")
        print("2. Make sure camera is publishing images")
        print("3. YOLO will detect objects automatically")
        print("4. Grasp executor will attempt to pick up detected objects")
        print("5. Press Ctrl+C to stop")
        print("")

        while rclpy.ok():
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nShutting down demo...")

    finally:
        executor.shutdown()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()