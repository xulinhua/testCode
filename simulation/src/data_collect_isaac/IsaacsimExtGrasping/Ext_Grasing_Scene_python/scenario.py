# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class ScenarioTemplate:
    """
    场景控制模板接口。

    这是 NVIDIA 示例扩展里留下的结构，用于把“UI 构建”和“仿真更新逻辑”分开。
    当前抓取数据采集主流程没有启用 ExampleScenario，但保留它方便以后加入机械臂/夹爪运动。
    """

    def __init__(self):
        pass

    def setup_scenario(self):
        """加载场景对象后调用，用于初始化控制状态。"""
        pass

    def teardown_scenario(self):
        """场景结束或重置时调用，用于释放引用、清空状态。"""
        pass

    def update_scenario(self):
        """每个物理步调用，用于更新机器人或物体运动。"""
        pass


import numpy as np
from isaacsim.core.utils.types import ArticulationAction

"""
这个场景模版展示了如何使用Python来控制IsaacSim中的场景。
场景模版的主要功能是让机器人运动起来。它可以让机器人运动到指定的位置，或者让机器人按照指定的轨迹运动。

场景模版的实现主要是基于IsaacSim的API。在这个模版中，我们使用了两个主要的API：
1. Articulation：机器人运动的核心API。它可以让机器人按照指定的动作进行运动。
2. Prim：场景中的物体的主要API。它可以让我们在场景中添加物体，并对其进行控制。

在这个模版中，我们使用了两个主要的功能：
1. 运动机器人：这个功能让机器人按照指定的轨迹运动。
2. 移动物体：这个功能让物体按照指定的轨迹移动。

在这个模版中，我们使用了两个主要的变量：
1. joint_index：机器人的关节索引。
2. joint_time：机器人的关节时间。

在这个模版中，我们使用了两个主要的函数：
1. derive_sinusoid_params：这个函数用来计算机器人的关节目标位置和速度。
2. update_sinusoidal_joint_path：这个函数用来更新机器人的关节目标位置和速度。
在这个模版中，我们使用了两个主要的类：
1. ExampleScenario：场景模版的主要类。
2. ScenarioTemplate：场景模版的父类。
在这个模版中，我们使用了两个主要的场景：
1. ExampleScenario：这个场景模版的具体实现。
2. ScenarioTemplate：场景模版的父类。   


This scenario takes in a robot Articulation and makes it move through its joint DOFs.
Additionally, it adds a cuboid prim to the stage that moves in a circle around the robot.

The particular framework under which this scenario operates should not be taken as a direct
recomendation to the user about how to structure their code.  In the simple example put together
in this template, this particular structure served to improve code readability and separate
the logic that runs the example from the UI design.
"""


class ExampleScenario(ScenarioTemplate):
    """
    示例场景：让一个 Articulation 的关节按正弦轨迹运动，同时让物体绕圆运动。

    本项目当前没有从 UIBuilder 调用它；如果后续要加入机械臂抓取动作，
    可以把这里作为“每个 physics step 更新控制指令”的参考。
    """

    def __init__(self):
        # 场景中被控制的物体和机器人引用，由 setup_scenario() 注入。
        self._object = None
        self._articulation = None

        self._running_scenario = False

        self._time = 0.0  # s

        self._object_radius = 0.5  # m
        self._object_height = 0.5  # m
        self._object_frequency = 0.25  # Hz

        self._joint_index = 0
        self._max_joint_speed = 4  # rad/sec
        self._lower_joint_limits = None
        self._upper_joint_limits = None

        self._joint_time = 0
        self._path_duration = 0
        self._calculate_position = lambda t, x: 0
        self._calculate_velocity = lambda t, x: 0

    def setup_scenario(self, articulation, object_prim):
        """绑定机器人和物体，并把机器人关节初始化到下限附近。"""
        self._articulation = articulation
        self._object = object_prim

        self._initial_object_position = self._object.get_world_pose()[0]
        self._initial_object_phase = np.arctan2(self._initial_object_position[1], self._initial_object_position[0])
        self._object_radius = np.linalg.norm(self._initial_object_position[:2])

        self._running_scenario = True

        self._joint_index = 0
        self._lower_joint_limits = articulation.dof_properties["lower"]
        self._upper_joint_limits = articulation.dof_properties["upper"]

        # 先把机器人关节移动到下限附近，保证正弦轨迹从可控起点开始。
        epsilon = 0.001
        articulation.set_joint_positions(self._lower_joint_limits + epsilon)

        self._derive_sinusoid_params(0)

    def teardown_scenario(self):
        """恢复初始状态，避免下一次运行沿用旧对象和旧时间。"""
        self._time = 0.0
        self._object = None
        self._articulation = None
        self._running_scenario = False

        self._joint_index = 0
        self._lower_joint_limits = None
        self._upper_joint_limits = None

        self._joint_time = 0
        self._path_duration = 0
        self._calculate_position = lambda t, x: 0
        self._calculate_velocity = lambda t, x: 0

    def update_scenario(self, step: float):
        """按物理步长推进示例场景。"""
        if not self._running_scenario:
            return

        self._time += step

        # 让示例物体以初始半径绕原点做圆周运动。
        x = self._object_radius * np.cos(self._initial_object_phase + self._time * self._object_frequency * 2 * np.pi)
        y = self._object_radius * np.sin(self._initial_object_phase + self._time * self._object_frequency * 2 * np.pi)
        z = self._initial_object_position[2]

        self._object.set_world_pose(np.array([x, y, z]))

        self._update_sinusoidal_joint_path(step)

    def _derive_sinusoid_params(self, joint_index: int):
        """为指定关节计算正弦目标位置/速度函数。"""
        # 从当前关节下限出发，在上下限之间做半周期余弦插值。
        start_position = self._lower_joint_limits[joint_index]

        P_max = self._upper_joint_limits[joint_index] - start_position
        V_max = self._max_joint_speed
        T = P_max * np.pi / V_max

        # T is the expected time of the joint path

        self._path_duration = T
        self._calculate_position = (
            lambda time, path_duration: start_position
            + -P_max / 2 * np.cos(time * 2 * np.pi / path_duration)
            + P_max / 2
        )
        self._calculate_velocity = lambda time, path_duration: V_max * np.sin(2 * np.pi * time / path_duration)

    def _update_sinusoidal_joint_path(self, step):
        """更新当前关节目标；一个关节走完后切换到下一个关节。"""
        self._joint_time += step

        if self._joint_time > self._path_duration:
            self._joint_time = 0
            self._joint_index = (self._joint_index + 1) % self._articulation.num_dof
            self._derive_sinusoid_params(self._joint_index)

        joint_position_target = self._calculate_position(self._joint_time, self._path_duration)
        joint_velocity_target = self._calculate_velocity(self._joint_time, self._path_duration)

        action = ArticulationAction(
            np.array([joint_position_target]),
            np.array([joint_velocity_target]),
            joint_indices=np.array([self._joint_index]),
        )
        self._articulation.apply_action(action)
