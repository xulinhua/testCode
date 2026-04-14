#!/usr/bin/env python3
"""
MoveIt2 IK + ros2_control executor for nova arms.

Input topic:
  /nova_arm_goal (std_msgs/String, JSON)

JSON schema:
{
  "arm_id": 0,
  "target_pose": {
    "frame_id": "base_link",
    "position": [0.35, -0.10, 0.25],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "gripper": {
    "mode": "open" | "close" | "width",
    "value": 0.06
  },
  "execute_order": "pose_then_gripper" | "gripper_then_pose" | "sync"
}
"""

import json
from copy import deepcopy
from typing import Dict, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class MoveIt2ArmExecutor(Node):
    def __init__(self) -> None:
        super().__init__("moveit2_arm_executor")

        self.arm_groups = {
            0: "j1_arm",
            1: "j2_arm",
            2: "j3_arm",
            3: "j4_arm",
        }
        self.ee_links = {
            0: "J1_6",
            1: "J2_6",
            2: "J3_6",
            3: "J4_6",
        }
        self.arm_prefix = {
            0: "J1_",
            1: "J2_",
            2: "J3_",
            3: "J4_",
        }
        self.control_joint_order = [
            "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint",
            "J1_7_joint", "J1_8_joint",
            "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint",
            "J2_7_joint", "J2_8_joint",
            "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint", "J3_6_joint",
            "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint",
        ]
        self.gripper_joints = {
            0: ("J1_7_joint", "J1_8_joint"),
            1: ("J2_7_joint", "J2_8_joint"),
        }
        # Tunable and explicit, so physical opening direction can be calibrated quickly.
        self.gripper_open = {
            0: (0.08, 0.08),
            1: (0.08, 0.08),
        }
        self.gripper_close = {
            0: (0.0, 0.0),
            1: (0.0, 0.0),
        }

        self.current_joint_map: Dict[str, float] = {}
        self.last_joint_state_time = self.get_clock().now()

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        # arm_controller is JointGroupPositionController, command topic is /arm_controller/commands
        self.command_pub = self.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.goal_sub = self.create_subscription(String, "/nova_arm_goal", self.on_goal, 10)

        self.get_logger().info("Waiting for /compute_ik service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("MoveIt2 arm executor ready. Subscribe /nova_arm_goal")

    def on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_map[name] = float(msg.position[i])
        self.last_joint_state_time = self.get_clock().now()

    def on_goal(self, msg: String) -> None:
        try:
            goal = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON on /nova_arm_goal: {exc}")
            return

        try:
            arm_id = int(goal["arm_id"])
            pose_cfg = goal["target_pose"]
            frame_id = pose_cfg.get("frame_id", "base_link")
            pos = pose_cfg["position"]
            quat = pose_cfg["orientation"]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Bad goal payload: {exc}")
            return

        if arm_id not in self.arm_groups:
            self.get_logger().error(f"Unsupported arm_id={arm_id}, expect 0/1/2/3")
            return

        if not self.current_joint_map:
            self.get_logger().error("No /joint_states received yet, cannot seed IK.")
            return

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        target_arm_joints = self.compute_ik(arm_id, pose)
        if target_arm_joints is None:
            self.get_logger().error(f"IK failed for arm_id={arm_id}")
            return

        order = goal.get("execute_order", "pose_then_gripper")
        gripper_cfg = goal.get("gripper")

        if order == "gripper_then_pose":
            if gripper_cfg is not None:
                self.execute_gripper(arm_id, gripper_cfg, time_s=0.8)
            self.execute_arm(arm_id, target_arm_joints, time_s=2.0)
        elif order == "sync":
            self.execute_arm_with_optional_gripper(arm_id, target_arm_joints, gripper_cfg, time_s=2.0)
        else:
            self.execute_arm(arm_id, target_arm_joints, time_s=2.0)
            if gripper_cfg is not None:
                self.execute_gripper(arm_id, gripper_cfg, time_s=0.8)

    def compute_ik(self, arm_id: int, pose: PoseStamped) -> Dict[str, float] | None:
        req = GetPositionIK.Request()
        ikr = PositionIKRequest()
        ikr.group_name = self.arm_groups[arm_id]
        ikr.ik_link_name = self.ee_links[arm_id]
        ikr.pose_stamped = pose
        ikr.timeout = Duration(seconds=0.5).to_msg()
        ikr.avoid_collisions = True

        state = RobotState()
        seed = JointState()
        seed.name = list(self.current_joint_map.keys())
        seed.position = [self.current_joint_map[n] for n in seed.name]
        state.joint_state = seed
        ikr.robot_state = state

        # Keep non-target arms near current posture to stabilize IK on shared model.
        constraints = Constraints()
        target_prefix = self.arm_prefix[arm_id]
        for name, pos in self.current_joint_map.items():
            if name.endswith("_joint") and not name.startswith(target_prefix):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = float(pos)
                jc.tolerance_above = 0.02
                jc.tolerance_below = 0.02
                jc.weight = 0.2
                constraints.joint_constraints.append(jc)
        ikr.constraints = constraints

        req.ik_request = ikr
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("No IK response from /compute_ik")
            return None

        res = future.result()
        if res.error_code.val != 1:
            self.get_logger().error(f"IK failed, MoveIt code={res.error_code.val}")
            return None

        out = {}
        for i, name in enumerate(res.solution.joint_state.name):
            if i < len(res.solution.joint_state.position):
                out[name] = float(res.solution.joint_state.position[i])
        return out

    def resolve_gripper_targets(self, arm_id: int, cfg: dict) -> Tuple[str, str, float, float] | None:
        if arm_id not in self.gripper_joints:
            self.get_logger().warn(f"arm_id={arm_id} has no gripper, skip gripper command")
            return None
        j0, j1 = self.gripper_joints[arm_id]
        mode = str(cfg.get("mode", "open")).lower()
        if mode == "open":
            p0, p1 = self.gripper_open[arm_id]
        elif mode == "close":
            p0, p1 = self.gripper_close[arm_id]
        elif mode == "width":
            width = float(cfg.get("value", 0.06))
            width = max(0.0, min(0.1, width))
            ratio = width / 0.1
            o0, o1 = self.gripper_open[arm_id]
            c0, c1 = self.gripper_close[arm_id]
            p0 = c0 + (o0 - c0) * ratio
            p1 = c1 + (o1 - c1) * ratio
        else:
            self.get_logger().error(f"Unsupported gripper mode={mode}")
            return None
        return j0, j1, float(p0), float(p1)

    def execute_arm(self, arm_id: int, ik_solution: Dict[str, float], time_s: float) -> None:
        cmd = deepcopy(self.current_joint_map)
        prefix = self.arm_prefix[arm_id]
        for name, pos in ik_solution.items():
            if name.startswith(prefix) and name in cmd:
                cmd[name] = pos
        self.publish_command(cmd)
        self.get_logger().info(f"Arm {arm_id} command sent.")

    def execute_gripper(self, arm_id: int, gripper_cfg: dict, time_s: float) -> None:
        resolved = self.resolve_gripper_targets(arm_id, gripper_cfg)
        if resolved is None:
            return
        j0, j1, p0, p1 = resolved
        cmd = deepcopy(self.current_joint_map)
        cmd[j0] = p0
        cmd[j1] = p1
        self.publish_command(cmd)
        self.get_logger().info(f"Arm {arm_id} gripper command sent: {j0}={p0:.4f}, {j1}={p1:.4f}")

    def execute_arm_with_optional_gripper(
        self, arm_id: int, ik_solution: Dict[str, float], gripper_cfg: dict | None, time_s: float
    ) -> None:
        cmd = deepcopy(self.current_joint_map)
        prefix = self.arm_prefix[arm_id]
        for name, pos in ik_solution.items():
            if name.startswith(prefix) and name in cmd:
                cmd[name] = pos

        if gripper_cfg is not None:
            resolved = self.resolve_gripper_targets(arm_id, gripper_cfg)
            if resolved is not None:
                j0, j1, p0, p1 = resolved
                cmd[j0] = p0
                cmd[j1] = p1
        self.publish_command(cmd)
        self.get_logger().info(f"Arm {arm_id} synced arm+gripper trajectory sent.")

    def publish_command(self, command_map: Dict[str, float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(command_map.get(j, 0.0)) for j in self.control_joint_order]
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveIt2ArmExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
