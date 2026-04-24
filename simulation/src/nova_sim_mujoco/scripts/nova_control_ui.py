#!/usr/bin/env python3
"""
Nova control UI (Tkinter):
1) Joint mode: publish all joints to /arm_controller/commands
2) Pose mode: publish calib_sim_mujoco/ArmPose to /nova_target_arm_pose (与 arm_id 同帧，避免分话题错序)
"""

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from calib_sim_mujoco.msg import ArmPose
from std_msgs.msg import Float64MultiArray, Int32, String


JOINT_ORDER = [
    "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint", "J1_8_joint",
    "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint", "J2_7_joint", "J2_8_joint",
    "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint", "J3_6_joint",
    "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint",
]


class NovaControlUI:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("nova_control_ui")
        self.cmd_pub = self.node.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)
        self.arm_id_pub = self.node.create_publisher(Int32, "/nova_arm_id", 10)
        self.arm_pose_pub = self.node.create_publisher(ArmPose, "/nova_target_arm_pose", 10)
        self.gripper_pub = self.node.create_publisher(String, "/nova_gripper_goal", 10)

        self.root = tk.Tk()
        self.root.title("Nova Arm Control UI")
        self.root.geometry("980x760")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.joint_tab = ttk.Frame(notebook)
        self.pose_tab = ttk.Frame(notebook)
        notebook.add(self.joint_tab, text="Joint Control")
        notebook.add(self.pose_tab, text="Pose Control")

        self.joint_vars = []
        self.build_joint_tab()
        self.build_pose_tab()

    def build_joint_tab(self) -> None:
        hdr = ttk.Frame(self.joint_tab)
        hdr.pack(fill=tk.X, padx=8, pady=6)
        ttk.Label(hdr, text="All-joint mode: publish 28 positions to /arm_controller/commands").pack(side=tk.LEFT)

        canvas = tk.Canvas(self.joint_tab)
        scrollbar = ttk.Scrollbar(self.joint_tab, orient="vertical", command=canvas.yview)
        frame = ttk.Frame(canvas)
        frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0), pady=6)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 8), pady=6)

        for idx, name in enumerate(JOINT_ORDER):
            row = ttk.Frame(frame)
            row.grid(row=idx, column=0, sticky="ew", padx=4, pady=2)
            ttk.Label(row, text=f"{idx:02d} {name}", width=24).pack(side=tk.LEFT)
            v = tk.StringVar(value="0.0")
            self.joint_vars.append(v)
            ttk.Entry(row, textvariable=v, width=12).pack(side=tk.LEFT, padx=4)
            # quick buttons
            ttk.Button(row, text="+0.1", command=lambda i=idx: self.bump_joint(i, +0.1)).pack(side=tk.LEFT, padx=2)
            ttk.Button(row, text="-0.1", command=lambda i=idx: self.bump_joint(i, -0.1)).pack(side=tk.LEFT, padx=2)

        btns = ttk.Frame(self.joint_tab)
        btns.pack(fill=tk.X, padx=8, pady=8)
        ttk.Button(btns, text="Publish Once", command=self.publish_joint_command).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Set All Zero", command=self.set_all_zero).pack(side=tk.LEFT, padx=4)

    def build_pose_tab(self) -> None:
        body = ttk.Frame(self.pose_tab)
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

        self.arm_id_var = tk.StringVar(value="0")
        self.frame_id_var = tk.StringVar(value="base_link")
        self.px_var = tk.StringVar(value="0.35")
        self.py_var = tk.StringVar(value="-0.10")
        self.pz_var = tk.StringVar(value="0.25")
        self.qx_var = tk.StringVar(value="0.0")
        self.qy_var = tk.StringVar(value="0.0")
        self.qz_var = tk.StringVar(value="0.0")
        self.qw_var = tk.StringVar(value="1.0")

        self.gripper_mode_var = tk.StringVar(value="none")
        self.gripper_width_var = tk.StringVar(value="0.05")

        row = 0
        ttk.Label(body, text="Pose mode: /nova_target_arm_pose (ArmPose)").grid(row=row, column=0, columnspan=4, sticky="w")
        row += 1

        ttk.Label(body, text="arm_id (0/1/2/3)").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        ttk.Entry(body, textvariable=self.arm_id_var, width=12).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="frame_id").grid(row=row, column=2, sticky="e", padx=6)
        ttk.Entry(body, textvariable=self.frame_id_var, width=16).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Label(body, text="x").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        ttk.Entry(body, textvariable=self.px_var, width=12).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="y").grid(row=row, column=2, sticky="e", padx=6)
        ttk.Entry(body, textvariable=self.py_var, width=12).grid(row=row, column=3, sticky="w")
        row += 1
        ttk.Label(body, text="z").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        ttk.Entry(body, textvariable=self.pz_var, width=12).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(body, text="qx").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        ttk.Entry(body, textvariable=self.qx_var, width=12).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="qy").grid(row=row, column=2, sticky="e", padx=6)
        ttk.Entry(body, textvariable=self.qy_var, width=12).grid(row=row, column=3, sticky="w")
        row += 1
        ttk.Label(body, text="qz").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        ttk.Entry(body, textvariable=self.qz_var, width=12).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="qw").grid(row=row, column=2, sticky="e", padx=6)
        ttk.Entry(body, textvariable=self.qw_var, width=12).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Separator(body, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=8)
        row += 1

        ttk.Label(body, text="Gripper").grid(row=row, column=0, sticky="e", padx=6, pady=4)
        mode_combo = ttk.Combobox(
            body,
            textvariable=self.gripper_mode_var,
            values=["none", "open", "close", "width"],
            state="readonly",
            width=10,
        )
        mode_combo.grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="width(m)").grid(row=row, column=2, sticky="e", padx=6)
        ttk.Entry(body, textvariable=self.gripper_width_var, width=12).grid(row=row, column=3, sticky="w")
        row += 1

        btns = ttk.Frame(body)
        btns.grid(row=row, column=0, columnspan=4, sticky="w", pady=12)
        ttk.Button(btns, text="Send Pose", command=self.send_pose_goal).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Send Gripper Only", command=self.send_gripper_only).pack(side=tk.LEFT, padx=4)

    def bump_joint(self, idx: int, delta: float) -> None:
        try:
            v = float(self.joint_vars[idx].get())
        except ValueError:
            v = 0.0
        self.joint_vars[idx].set(f"{v + delta:.4f}")

    def set_all_zero(self) -> None:
        for v in self.joint_vars:
            v.set("0.0")

    def publish_joint_command(self) -> None:
        try:
            values = [float(v.get()) for v in self.joint_vars]
        except ValueError:
            messagebox.showerror("Input Error", "Joint value must be numeric.")
            return
        msg = Float64MultiArray()
        msg.data = values
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("Published /arm_controller/commands")

    def send_pose_goal(self) -> None:
        try:
            arm_id = int(self.arm_id_var.get())
            px = float(self.px_var.get())
            py = float(self.py_var.get())
            pz = float(self.pz_var.get())
            qx = float(self.qx_var.get())
            qy = float(self.qy_var.get())
            qz = float(self.qz_var.get())
            qw = float(self.qw_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "arm_id / pose values must be numeric.")
            return
        if arm_id not in (0, 1, 2, 3):
            messagebox.showerror("Input Error", "arm_id must be 0,1,2,3.")
            return

        out = ArmPose()
        out.arm_id = arm_id
        out.pose.header.frame_id = self.frame_id_var.get().strip() or "base_link"
        out.pose.header.stamp = self.node.get_clock().now().to_msg()
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        self.arm_pose_pub.publish(out)

        mode = self.gripper_mode_var.get()
        if mode != "none":
            self.publish_gripper_mode(mode)
        self.node.get_logger().info(f"Sent pose goal for arm_id={arm_id}")

    def publish_gripper_mode(self, mode: str) -> None:
        msg = String()
        if mode == "width":
            try:
                width = float(self.gripper_width_var.get())
            except ValueError:
                messagebox.showerror("Input Error", "gripper width must be numeric.")
                return
            msg.data = f"width:{width:.4f}"
        else:
            msg.data = mode
        self.gripper_pub.publish(msg)

    def send_gripper_only(self) -> None:
        try:
            arm_id = int(self.arm_id_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "arm_id must be numeric.")
            return
        mode = self.gripper_mode_var.get()
        if mode == "none":
            messagebox.showerror("Input Error", "Please choose gripper mode.")
            return
        id_msg = Int32()
        id_msg.data = arm_id
        self.arm_id_pub.publish(id_msg)
        self.publish_gripper_mode(mode)
        self.node.get_logger().info(f"Sent gripper={mode} for arm_id={arm_id}")

    def on_close(self) -> None:
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    ui = NovaControlUI()
    ui.run()


if __name__ == "__main__":
    main()
