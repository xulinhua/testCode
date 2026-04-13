#!/usr/bin/env python3
"""
Grasp Execution Node for ROS2
Receives detected object poses and executes grasping using MoveIt
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import tf2_ros
import tf2_geometry_msgs
import math
import time

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')

        # MoveIt Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # IK Service Client
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.objects_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.objects_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/grasp_status', 10)

        # Gripper control
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        # Storage
        self.current_joint_states = None
        self.is_executing = False
        self.target_object = None

        # Grasp parameters
        self.pre_grasp_height = 0.1  # 10cm above object
        self.grasp_height = 0.02     # 2cm above object for actual grasp
        self.retract_height = 0.15   # 15cm lift after grasp

        self.get_logger().info('Grasp Executor initialized')

    def joint_states_callback(self, msg):
        """Store current joint states"""
        self.current_joint_states = msg

    def objects_callback(self, msg):
        """Handle detected objects"""
        if self.is_executing:
            self.get_logger().info('Already executing grasp, ignoring new objects')
            return

        if len(msg.poses) == 0:
            return

        # Select the first detected object for now
        # You can add more sophisticated selection logic here
        target_pose = msg.poses[0]

        self.get_logger().info('Object detected, starting grasp sequence')
        self.execute_grasp_sequence(target_pose)

    def execute_grasp_sequence(self, object_pose):
        """Execute complete grasp sequence"""
        self.is_executing = True
        self.publish_status('Starting grasp sequence')

        try:
            # Step 1: Move to pre-grasp position
            if not self.move_to_pre_grasp(object_pose):
                raise Exception('Failed to reach pre-grasp position')

            # Step 2: Move to grasp position
            if not self.move_to_grasp_position(object_pose):
                raise Exception('Failed to reach grasp position')

            # Step 3: Close gripper
            if not self.close_gripper():
                raise Exception('Failed to close gripper')

            # Step 4: Retract with object
            if not self.retract_with_object():
                raise Exception('Failed to retract')

            self.publish_status('Grasp sequence completed successfully')
            self.get_logger().info('Grasp sequence completed successfully')

        except Exception as e:
            self.publish_status(f'Grasp failed: {str(e)}')
            self.get_logger().error(f'Grasp sequence failed: {e}')

        finally:
            self.is_executing = False

    def move_to_pre_grasp(self, object_pose):
        """Move to pre-grasp position above object"""
        self.publish_status('Moving to pre-grasp position')

        # Calculate pre-grasp pose
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = 'base_link'
        pre_grasp_pose.header.stamp = self.get_clock().now().to_msg()
        pre_grasp_pose.pose.position.x = object_pose.position.x
        pre_grasp_pose.pose.position.y = object_pose.position.y
        pre_grasp_pose.pose.position.z = object_pose.position.z + self.pre_grasp_height

        # Set orientation for grasping (gripper pointing down)
        pre_grasp_pose.pose.orientation.x = 0.0
        pre_grasp_pose.pose.orientation.y = 1.0
        pre_grasp_pose.pose.orientation.z = 0.0
        pre_grasp_pose.pose.orientation.w = 0.0

        return self.move_to_pose(pre_grasp_pose, 'pre_grasp')

    def move_to_grasp_position(self, object_pose):
        """Move down to grasp position"""
        self.publish_status('Moving to grasp position')

        # Calculate grasp pose
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position.x = object_pose.position.x
        grasp_pose.pose.position.y = object_pose.position.y
        grasp_pose.pose.position.z = object_pose.position.z + self.grasp_height

        # Same orientation as pre-grasp
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = 1.0
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 0.0

        return self.move_to_pose(grasp_pose, 'grasp')

    def retract_with_object(self):
        """Retract arm after successful grasp"""
        self.publish_status('Retracting with object')

        # Get current end-effector position
        try:
            current_pose = self.get_current_ee_pose()
            if current_pose is None:
                return False

            # Move straight up
            retract_pose = PoseStamped()
            retract_pose.header.frame_id = 'base_link'
            retract_pose.header.stamp = self.get_clock().now().to_msg()
            retract_pose.pose.position.x = current_pose.pose.position.x
            retract_pose.pose.position.y = current_pose.pose.position.y
            retract_pose.pose.position.z = current_pose.pose.position.z + self.retract_height

            # Keep same orientation
            retract_pose.pose.orientation = current_pose.pose.orientation

            return self.move_to_pose(retract_pose, 'retract')

        except Exception as e:
            self.get_logger().error(f'Retract failed: {e}')
            return False

    def move_to_pose(self, target_pose, plan_id):
        """Move to target pose using MoveIt"""
        try:
            # Wait for MoveIt action server
            if not self.move_group_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('MoveIt action server not available')
                return False

            # Create motion plan request
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = 'arm'  # Use your arm group name
            goal_msg.request.max_acceleration_scaling_factor = 0.5
            goal_msg.request.max_velocity_scaling_factor = 0.5
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.allowed_planning_time = 10.0

            # Set target constraint
            constraint = Constraints()
            constraint.name = f'{plan_id}_constraint'

            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = 'base_link'
            pos_constraint.link_name = 'end_effector_link'  # Use your end-effector link name
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            pos_constraint.weight = 1.0

            # Orientation constraint
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = 'base_link'
            orient_constraint.link_name = 'end_effector_link'  # Use your end-effector link name
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0

            constraint.position_constraints.append(pos_constraint)
            constraint.orientation_constraints.append(orient_constraint)
            goal_msg.request.goal_constraints.append(constraint)

            # Send goal and wait for result
            future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error('Goal rejected')
                return False

            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()

            if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f'Motion to {plan_id} completed successfully')
                return True
            else:
                self.get_logger().error(f'Motion failed with error code: {result.result.error_code.val}')
                return False

        except Exception as e:
            self.get_logger().error(f'Motion planning failed: {e}')
            return False

    def get_current_ee_pose(self):
        """Get current end-effector pose"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'end_effector_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except Exception as e:
            self.get_logger().error(f'Failed to get current EE pose: {e}')
            return None

    def close_gripper(self):
        """Close gripper to grasp object"""
        self.publish_status('Closing gripper')

        try:
            # Send gripper command
            gripper_cmd = Float64MultiArray()
            gripper_cmd.data = [0.5]  # Adjust based on your gripper

            self.gripper_pub.publish(gripper_cmd)

            # Wait for gripper to close
            time.sleep(2.0)

            self.get_logger().info('Gripper closed')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to close gripper: {e}')
            return False

    def open_gripper(self):
        """Open gripper to release object"""
        self.publish_status('Opening gripper')

        try:
            # Send gripper command
            gripper_cmd = Float64MultiArray()
            gripper_cmd.data = [0.0]  # Open position

            self.gripper_pub.publish(gripper_cmd)

            # Wait for gripper to open
            time.sleep(2.0)

            self.get_logger().info('Gripper opened')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to open gripper: {e}')
            return False

    def publish_status(self, status):
        """Publish grasp status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    executor = GraspExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()