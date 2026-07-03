# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import random
import numpy as np
import asyncio
import time
import json


import omni.timeline
import omni.ui as ui
import omni.replicator.core as rep
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
# from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton

from isaacsim.gui.components.ui_utils import btn_builder, str_builder, cb_builder, combo_cb_scrolling_frame_builder, multi_cb_builder, combo_cb_dropdown_builder, multi_dropdown_builder
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style, dropdown_builder, combo_cb_str_builder   
from isaacsim.storage.native import get_assets_root_path
from omni.usd import StageEventType
from pxr import Sdf, UsdLux, UsdGeom,Gf, UsdPhysics, PhysxSchema, Usd

from omni.isaac.core.utils.semantics import add_update_semantics, get_semantics
# from omni.isaac.core.utils.semantics import add_labels
from .scenario import ExampleScenario
from .global_variables import class_names, num_classes, meshOptions
from .lx_writer import LxWriter
from .util import generate_camera_pose
# from .layouts import CategorySelector


class UIBuilder:
    """
    插件主要业务类。

    负责三件事：
    1. 构建左侧插件 UI，读取物体数量、生成范围、相机采集参数等输入。
    2. 在 Stage 中创建基础场景：地面、灯光、相机、桌子、随机物体和物理碰撞。
    3. 调用 Replicator + LxWriter 逐帧采集 RGB、深度、语义分割和点云数据。

    extension.py 只负责生命周期和事件转发，本类才是项目的核心逻辑。
    """

    def __init__(self):
        # Frames 是可折叠 UI 区块，Isaac 示例模板中常用来做分组面板。
        self.frames = []
        # 使用 UIElementWrapper 创建的控件需要在 cleanup() 中释放，避免热重载后残留回调。
        self.wrapped_ui_elements = []

        # Timeline 控制仿真播放/暂停/停止；物体下落和 Replicator 逐帧采集都依赖时间推进。
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

        self.stage = omni.usd.get_context().get_stage()
        # UI 控件索引表：后续通过 key 读取用户输入值或控制按钮状态。
        self._buttons = {}
        self._line_edit = {}
        # 场景对象句柄。load_world() 创建，collect_data() 使用。
        self.camera = None
        self.render_product = None
        self.ground = None
        self.light = None
        # self.object 保存 /World/obj_i 这些容器 Prim 路径，用于采集前记录物体位姿。
        self.object = []
        # work_dir 指向扩展根目录；采集输出写入 work_dir/data_log/<时间戳>/。
        self.work_dir = None
        self.camera_name = None
        # 类别配置来自 global_variables.py。_classes_select 会随复选框变化。
        self._num_classes = num_classes
        self._class_names = class_names.copy()
        self._classes_select = class_names.copy()
        # self.world = None
        # World 是 Isaac Core 的场景/物理容器，这里指定 60Hz 物理步长。
        self.world = World(physics_dt=1.0 / 60.0)
        # Replicator Writer。采集前 attach，采集完成后 detach，避免重复写帧。
        self.writter = None

        

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """菜单打开后的回调。

        当前没有额外逻辑，保留这个函数是为了兼容 extension.py 的通用模板。
        """
        pass

    def on_timeline_event(self, event):
        """Timeline 播放/暂停/停止回调。

        Args:
            event (omni.timeline.TimelineEventType): 时间轴事件。
        """
        pass
        # if event.type == int(omni.timeline.TimelineEventType.STOP):
        #     # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
        #     # For complete robustness, the user should resolve those edge cases here
        #     # In general, for extensions based off this template, there is no value to having the user click the play/stop
        #     # button instead of using the Load/Reset/Run buttons provided.
        #     self._scenario_state_btn.reset()
        #     self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """每个物理步的回调。

        目前采集流程没有逐物理步控制，函数保留为后续实现夹爪、机械臂或稳定检测的扩展点。

        Args:
            step (float): 当前物理步长。
        """
        pass

    def on_stage_event(self, event):
        """Stage 事件回调。

        Args:
            event (omni.usd.StageEventType): Stage 打开、关闭等事件。
        """
        pass
        
        # if event.type == int(StageEventType.OPENED):
        #     # If the user opens a new stage, the extension should completely reset
        #     self._reset_extension()

    def cleanup(self):
        """
        Stage 关闭或扩展热重载时调用。

        通过 Isaac UI wrapper 创建的控件内部可能注册了回调，这里统一 cleanup。
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def add_category_to_list(self, state, category_name):
        """
        当复选框状态改变时调用此函数。

        Args:
            state (bool): 复选框的状态，True 表示选中，False 表示取消选中。
            category_name (str): 类别名称。
        """
        print(state)
        print(category_name)
        if state:  # 被选中 → 添加
            if category_name not in self._classes_select:
                self._classes_select.append(category_name)
                print(f"✅ 已添加类别: {category_name}")
            else:
                print(f"ℹ️  类别 {category_name} 已存在，无需重复添加。")
        else:  # 被取消 → 删除
            if category_name in self._classes_select:
                self._classes_select.remove(category_name)
                print(f"❌ 已删除类别: {category_name}")
            else:
                print(f"ℹ️  类别 {category_name} 本就不在列表中，无需删除。")
        print(f"当前选中的类别: {self._classes_select}")

    # def add_category_to_list(self, state, idx):
    #     print(f"当前选中的类别: {idx}")

    def checkbox_dropdown_builder(self):
        """
        创建一个下拉菜单，其中包含所有类别，并添加复选框以选择类别。
        :return:
        """
        with ui.VStack(style=get_style(), spacing=5, height=0):
            # 正确捕获 idx，忽略 state
            callbacks = [
                # lambda state, idx=idx: self.add_category_to_list(state, idx)
                lambda state, idx=idx: self.add_category_to_list(state, self._class_names[idx])
                for idx in range(len(self._class_names))
            ]

            # mb = multi_cb_builder(
            #     type='multi_checkbox',
            #     count=num_classes,
            #     text=class_names,
            #     default_val=[True] * num_classes,
            #     tooltip=[''] * (num_classes+1),  # 修正：数量一致
            #     on_clicked_fn=callbacks,
            # )

            for i in range(self._num_classes):
                cb_builder(
                    label=self._class_names[i],
                    type="checkbox",
                    default_val=True,
                    tooltip="",
                    on_clicked_fn=callbacks[i]
                )


    def build_ui(self):
        """
        构建插件 UI。

        这个函数在窗口每次重新打开时都会执行，因此只负责创建控件和绑定回调；
        真实场景对象不要在这里创建，避免打开/关闭窗口造成重复加载。
        """

        selection_controls_frame = CollapsableFrame("Select Plane", collapsed=False)

        with selection_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    # 要随机生成的小物体数量，不包含桌子 obj_mz。
                    # self._plane_dropdown = ui.Dropdown(label="Plane", options=["XY", "XZ", "YZ"])
                    but_dict = {
                        "label": "Number of objects",
                        "type": "stringfield",
                        "default_val": "10",
                        "tooltip": "number of object generated on scene",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["num_obj"] = str_builder(**but_dict)

                    print(self._classes_select)

                self.checkbox_dropdown_builder()

                # with ui.VStack(style=get_style(), spacing=5, height=0):
                #     but_dict = {
                #         "label": "load scene",
                #         "type": "button",
                #         "text": "Load Scene",
                #         "tooltip": "Load World and Task",
                #         "on_clicked_fn": self._setup_scene,
                #     }
                #     self._buttons["load_scene"] = btn_builder(**but_dict)
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    # 随机生成物体的 XY 范围；Z 在 load_world() 中固定抬高后让物体自然下落。
                    but_dict = {
                        "label": "Spawn x min",
                        "type": "stringfield",
                        "default_val": "-0.3",
                        "tooltip": "Spawn x min",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["spawn_x_min"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "Spawn x max",
                        "type": "stringfield",
                        "default_val": "0.3",
                        "tooltip": "Spawn x max",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["spawn_x_max"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "Spawn y min",
                        "type": "stringfield",
                        "default_val": "-0.3",
                        "tooltip": "number of object generated on scene",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["spawn_y_min"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "Spawn y max",
                        "type": "stringfield",
                        "default_val": "0.3",
                        "tooltip": "Spawn y max",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["spawn_y_max"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "load scene",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Load World and Task",
                        "on_clicked_fn": self.load_world,
                        # "on_clicked_fn": self._setup_scene,
                    }
                    self._buttons["load_scene"] = btn_builder(**but_dict)

                with ui.VStack(style=get_style(), spacing=5, height=0):
                    # 采集参数：相机从目标上方圆柱区域随机采样位置和朝向。
                    but_dict = {
                        "label": "Number of shots",
                        "type": "stringfield",
                        "default_val": "20",
                        "tooltip": "Number of shots",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["num_of_shots"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "cylinder_radius",
                        "type": "stringfield",
                        "default_val": "0.5",
                        "tooltip": "cylinder_radius",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                    }
                    self._line_edit["cylinder_radius"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "cylinder_height",
                        "type": "stringfield",
                        "default_val": "4",
                        "tooltip": "cylinder_height",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                        }
                    self._line_edit["cylinder_height"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "obj_x",
                        "type": "stringfield",
                        "default_val": "0",
                        "tooltip": "obj_x",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                        }
                    self._line_edit["obj_x"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "obj_y",
                        "type": "stringfield",
                        "default_val": "0",
                        "tooltip": "obj_y",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                        }
                    self._line_edit["obj_y"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "upper_cylinder_height",
                        "type": "stringfield",
                        "default_val": "0.5",
                        "tooltip": "upper_cylinder_height",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                        }
                    self._line_edit["upper_cylinder_height"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "obj_radius",
                        "type": "stringfield",
                        "default_val": "0.25",
                        "tooltip": "obj_radius",
                        "on_clicked_fn": None,
                        "use_folder_picker": False,
                        "read_only": False
                        }
                    self._line_edit["obj_radius"] = str_builder(**but_dict)

                    but_dict = {
                        "label": "collect data",
                        "type": "button",
                        "text": "Start",
                        "tooltip": "Start data collection",
                        "on_clicked_fn": self.collection_on_button_click,
                        # "on_clicked_fn": None,
                    }
                    self._buttons["collect_data"] = btn_builder(**but_dict)


        # run_scenario_frame = CollapsableFrame("Run Scenario")

        # with run_scenario_frame:
        #     with ui.VStack(style=get_style(), spacing=5, height=0):
        #         self._scenario_state_btn = StateButton(
        #             "Run Scenario",
        #             "RUN",
        #             "STOP",
        #             on_a_click_fn=self._on_run_scenario_a_text,
        #             on_b_click_fn=self._on_run_scenario_b_text,
        #             physics_callback_fn=self._update_scenario,
        #         )
        #         self._scenario_state_btn.enabled = False
        #         self.wrapped_ui_elements.append(self._scenario_state_btn)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        """模板初始化入口。当前项目没有使用示例机械臂场景，所以保持空实现。"""
        pass
    #     self._articulation = None
    #     self._cuboid = None
    #     self._scenario = ExampleScenario()

    def clear_scene(self):
        """
        清除当前 Stage 中的所有 Prim（除了根节点 /）。
        注意：不会删除引用的资产，只删除当前 Layer 中的 Prim。
        """
        # 获取所有顶级子 Prim（/World, /Cams, /Render 等）
        root_prim = self.stage.GetPseudoRoot()
        children = list(root_prim.GetChildren())  # 转为列表，避免迭代时修改问题

        for child in children:
            # 获取 Prim 的路径
            path = child.GetPath()

            # ⚠️ 可选：跳过某些关键 Prim（如 PhysicsScene, Render Settings）
            # 如果你不确定，可以清空所有
            # 这里我们删除所有，add_default_ground_plane 会重建必要的
            try:
                self.stage.RemovePrim(path)
                print(f"🗑️ Removed: {path}")
            except Exception as e:
                print(f"Failed to remove {path}: {e}")

    def create_new_world(self):
        """
        删除当前场景，创建一个全新的 Stage 和 World。
        """

        # 1. ✅ 清理旧的 World（如果存在）
        if self.world is not None:
            if self.world.is_playing():
                self.world.stop()  # 停止仿真
            self.world.clear()  # 清理内部状态
            self.world = None  # 解引用

        # 1. 获取当前 USD Context
        usd_context = omni.usd.get_context()

        # 2. 关闭当前 Stage（这会清空一切）
        usd_context.new_stage()  # ⚡ 关键：new_stage() 会创建一个空的 Stage
        print("✅ Cleared everything and created a new empty stage.")

        # 3. 创建新的 World 实例（它会绑定到新的 Stage）
        # self.world = World(stage_units_in_meters=1.0)  # 可自定义参数
        self.world = World(physics_dt=1.0 / 60.0)
        # self.world.initialize()
        print("✅ New World created on fresh stage.")

    def load_world(self):
        """
        加载并初始化采集场景。

        流程：
        1. 创建/复用地面、灯光、相机和 render_product。
        2. 从 data/raw_data/mz/mz.usd 加载桌子模型。
        3. 根据 UI 选择的类别，从 data/raw_data/<类别>/ 中随机挑选 USD 模型。
        4. 给桌子和物体添加语义标签、刚体、质量和碰撞近似。
        5. reset World，让物理系统接管物体下落。
        """

        # 清除场景
        # self.clear_scene()
        # self.world = World(physics_dt=1.0 / 60.0)
        # self.create_new_world()
        # self.create_new_world_async()
        self.cleanup()

        # 默认地面来自 Isaac Core；加语义标签后，分割和点云语义里能识别为 ground。
        self.ground = self.world.scene.add_default_ground_plane()
        # ground_usd_path = get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd"
        # self.ground = rep.create.from_usd(ground_usd_path, semantics=[('class', 'ground')])
        # self.ground = rep.create_from_usd(ground_usd_path)
        # 追加标签（不覆盖已有）
        # add_labels(self.ground, ["ground"], "class", overwrite=True)
        add_update_semantics(self.ground.prim, "ground", "select_classes")
        # _ = get_semantics(self.ground.prim)
        self.stage = omni.usd.get_context().get_stage()

        if self.light is None:
            # 创建一个平行光，给 RGB 图像提供稳定照明。
            light_path = Sdf.Path("/World/DefaultLight")
            if not self.stage.GetPrimAtPath(light_path):
                self.light = UsdLux.DistantLight.Define(self.stage, light_path)
                self.light.CreateIntensityAttr(3000)
                self.light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.9))
                self.light.CreateAngleAttr(0.53)
                rot_z = Gf.Rotation(Gf.Vec3d.ZAxis(), 45)
                rot_y = Gf.Rotation(Gf.Vec3d.YAxis(), -30)
                xform = self.light.AddRotateXYZOp()

                # xform.Set(Gf.Vec3f(rot_y * rot_z).Decompose(Gf.Vec3d.XAxis(),
                #                                             Gf.Vec3d.YAxis(),
                #                                             Gf.Vec3d.ZAxis()))
                # 1. 计算组合旋转
                combined_rot = rot_y * rot_z

                # 2. 转换为 3x3 旋转矩阵
                # 注意：SetRotate 可能需要 GfQuat* 或 GfRotation 作为输入来设置矩阵的旋转部分
                mat = Gf.Matrix4d(1.0)  # 创建单位 4x4 矩阵
                mat.SetRotate(combined_rot)  # 应用组合旋转
                rot_mat3 = mat.ExtractRotationMatrix()  # 提取 3x3 旋转部分

                # 3. 从 3x3 矩阵手动计算 XYZ 顺序欧拉角 (度)
                import math
                m = rot_mat3
                epsilon = 1e-6
                sy = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])

                if sy > epsilon:
                    # No gimbal lock
                    euler_x_rad = math.atan2(m[2][1], m[2][2])
                    euler_y_rad = math.atan2(-m[2][0], sy)
                    euler_z_rad = math.atan2(m[1][0], m[0][0])
                else:
                    # Gimbal lock
                    euler_x_rad = math.atan2(-m[1][2], m[1][1])
                    euler_y_rad = math.atan2(-m[2][0], sy)  # sy is close to zero
                    euler_z_rad = 0.0

                # 转换为度数
                euler_angles_deg = Gf.Vec3f(
                    math.degrees(euler_x_rad),
                    math.degrees(euler_y_rad),
                    math.degrees(euler_z_rad)
                )

                # 4. 设置 xform
                xform.Set(euler_angles_deg)
                print(f"成功设置灯光旋转 (XYZ degrees): {euler_angles_deg}")

        if self.camera is None:
            # Replicator 相机和 render_product 是数据采集入口。
            # 分辨率必须与 get_camera_intrinsics() 中的宽高保持一致。
            self.camera = rep.create.camera(position=(0, 0, 1))
            self.render_product = rep.create.render_product(self.camera, (640, 480))
            # self.render_product = rep.create.render_product(self.camera, (1024, 1024))


        # 生成桌子。路径使用相对当前 Python 文件的位置，避免写死某台机器的绝对路径。
        prim_path = f"/World/obj_mz"
        parent_folder = os.path.split(__file__)[0]
        table_usd_path = os.path.abspath(os.path.join(parent_folder, "..", "data", "raw_data", "mz", "mz.usd"))

        # 添加到场景
        added_obj = add_reference_to_stage(
            usd_path=table_usd_path,
            prim_path=prim_path
        )
        add_update_semantics(added_obj, "mz", "select_classes")

        # 桌子固定在世界原点，作为物体下落后的支撑面。
        self.set_obj_position(prim_path, (0, 0, 0))


        # 1. 获取物体的 Prim
        obj_prim = self.stage.GetPrimAtPath(prim_path)
        if not obj_prim:
            print(f"❌ 错误：无法找到 Prim {prim_path}，跳过物理属性添加。")
        xform = UsdGeom.Xform(obj_prim)
        xform.AddScaleOp().Set(Gf.Vec3d(2.5, 2.5, 2.5))
        # xform.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))

        # --- 给桌子添加刚体、质量和碰撞属性 ---
        #  应用 Physics Rigid Body API
        # 检查是否已存在
        if not UsdPhysics.RigidBodyAPI(obj_prim):
            applied_rb_api = UsdPhysics.RigidBodyAPI.Apply(obj_prim)

        #应用 Mass API 并设置质量
        # 检查是否已存在 MassAPI
        mass_api = UsdPhysics.MassAPI(obj_prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

        if mass_api:
            # 检查质量属性是否存在，如果不存在则创建并设置
            mass_attr = mass_api.GetMassAttr()
            if not mass_attr:
                mass_attr = mass_api.CreateMassAttr()
            # 设置质量值 (例如 1.0 kg)
            mass_attr.Set(10)

        # 递归给桌子内部所有 Mesh 子节点添加碰撞；否则物体会穿过桌面。
        num_collisions = self.apply_collision_to_mesh_children(obj_prim)

        # 从 UI 输入读取随机生成参数。这里失败后直接返回，避免用错误参数继续生成场景。
        try:
            item_count = self._line_edit['num_obj'].get_value_as_int()
        except:
            print("Invalid num_obj")
            return 1
        try:
            x_min = self._line_edit['spawn_x_min'].get_value_as_float()
        except:
            print("Invalid x min")
        try:
            x_max = self._line_edit['spawn_x_max'].get_value_as_float()
        except:
            print("Invalid x max")
        try:
            y_min = self._line_edit['spawn_y_min'].get_value_as_float()
        except:
            print("Invalid y min")
        try:
            y_max = self._line_edit['spawn_y_max'].get_value_as_float()
        except:
            print("Invalid y max")
        if self.ground is None:
            ground_usd_path = get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd"
            self.ground = rep.create.from_usd(ground_usd_path, semantics=[('select_classes', 'ground')])
        if len(self.object):
            # 如果之前已经加载过场景，先删除旧的 obj_i，避免重复对象混在 Stage 里。
            for i in range(len(self.object)):
                prim_path = '/World/obj_{}'.format(i)
                if self.stage.GetPrimAtPath(prim_path):
                    self.stage.RemovePrim(prim_path)

            # 置空obj列表
            self.object = []

        parent_folder = os.path.split(__file__)[0]
        # 物体资源根目录：data/raw_data/<类别名>/<模型>.usd。
        data_folder = os.path.join(parent_folder, "..", "data", "raw_data")

        # 生成每个类别的完整路径
        class_paths = [
            os.path.join(data_folder, class_name) for class_name in self._classes_select
        ]

        # 打印调试
        for path in class_paths:
            print(f"类别路径: {path}")

        # 扩展根目录。collector_folder_prep() 会在这里创建 data_log/<时间戳>/。
        self.work_dir = os.path.join(parent_folder, "..", "")
        os.makedirs(self.work_dir, exist_ok=True)

        # 创建 data_log 目录（如果不存在）
        data_log_path = os.path.join(self.work_dir, "data_log")
        if not os.path.exists(data_log_path):
            os.mkdir(data_log_path)

        # 收集所有选中类别下的 USD 文件完整路径，后面随机抽样生成。
        all_usd_paths = []

        for class_path in class_paths:
            if not os.path.exists(class_path):
                print(f"警告：类别路径不存在，跳过 {class_path}")
                continue
            for file in os.listdir(class_path):
                if file.lower().endswith(('.usd', '.usda', '.usdc')):
                    full_path = os.path.join(class_path, file)
                    all_usd_paths.append(full_path)

        # 检查是否找到模型
        if not all_usd_paths:
            print("❌ 错误：未找到任何 USD 模型文件！请检查 data/raw_data 下的类别目录。")
            return

        print(f"✅ 共找到 {len(all_usd_paths)} 个 USD 模型")

        # === 随机生成物体 ===
        for i in range(item_count):
            # 随机选择一个模型
            obj_path = random.choice(all_usd_paths)
            prim_path = f"/World/obj_{i}"


            # 添加到场景
            added_obj = add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)

            # 添加语义标签。这里使用文件名去掉后缀作为类别名，所以模型文件名要与 class_names 对齐。
            semantic_label = obj_path.split("/")[-1].rsplit('.', 1)[0]
            if semantic_label in self._classes_select:
                add_update_semantics(added_obj, semantic_label, "select_classes")
            else:
                print(f"❌ 错误：{obj_path} 的语义标签 {semantic_label} 不在类别列表中，跳过添加语义标签。")
                continue

            self.object.append(prim_path)  # 存储路径可能比存储 add_reference_to_stage 的返回值更通用

            # 随机位置
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = 2  # 初始高度抬高，让物体在物理仿真中自然落到桌面上。
            self.set_obj_position(prim_path, (x, y, z))

            # --- 添加刚体和碰撞属性，使物体参与物理仿真 ---
            # 1. 获取物体的 Prim
            obj_prim = self.stage.GetPrimAtPath(prim_path)
            if not obj_prim:
                print(f"❌ 错误：无法找到 Prim {prim_path}，跳过物理属性添加。")
                # continue

            # 2. 应用 Physics Rigid Body API
            # 检查是否已存在
            if not UsdPhysics.RigidBodyAPI(obj_prim):
                applied_rb_api = UsdPhysics.RigidBodyAPI.Apply(obj_prim)

            # 3. (可选) 应用 Mass API 并设置质量
            # 检查是否已存在 MassAPI
            mass_api = UsdPhysics.MassAPI(obj_prim)
            if not mass_api:
                mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

            if mass_api:
                # 检查质量属性是否存在，如果不存在则创建并设置
                mass_attr = mass_api.GetMassAttr()
                if not mass_attr:
                    mass_attr = mass_api.CreateMassAttr()
                # 设置质量值 (例如 1.0 kg)
                mass_attr.Set(0.1)

            # 4. 应用 Physics Collision API (使用 MeshCollisionAPI)
            # applied_collision_api = UsdPhysics.MeshCollisionAPI.Apply(obj_prim)

            # 递归给模型内部 Mesh 添加碰撞近似。
            num_collisions = self.apply_collision_to_mesh_children(obj_prim)

            if num_collisions == 0:
                print(f"❌ {prim_path} 及其子级中未找到任何可添加碰撞的几何体，请检查模型结构")

        # 重置 World，让刚刚添加的对象进入物理系统；之后用户可播放仿真让物体落稳。
        self.world.reset()

    # =====================================================
    # 3. 为所有子级 Mesh 添加 MeshCollisionAPI
    # =====================================================
    @staticmethod
    def apply_collision_to_mesh_children(parent_prim):
        """
        给 parent_prim 及其所有几何子节点添加碰撞。

        USD 模型通常是一个 Xform 容器，真正的 Mesh 在子级；只给顶层 Prim 加碰撞不一定生效。
        当前碰撞近似使用 convexHull，速度和稳定性比较适合数据采集场景。

        Args:
            parent_prim: 要递归处理的 USD Prim。

        Returns:
            int: 成功添加碰撞的几何体数量
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("❌ 无法获取 Stage")
            return 0

        applied_count = 0

        # 递归遍历所有后代
        for child_prim in Usd.PrimRange(parent_prim):
            # 检查是否是支持的几何类型
            if not (child_prim.IsA(UsdGeom.Mesh) or
                    child_prim.IsA(UsdGeom.Cube) or
                    child_prim.IsA(UsdGeom.Sphere) or
                    child_prim.IsA(UsdGeom.Cylinder) or
                    child_prim.IsA(UsdGeom.Capsule)):
                continue

            # ✅ 检查是否是 Mesh 并有顶点数据（仅对 Mesh 有意义）
            if child_prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(child_prim)
                points = mesh.GetPointsAttr().Get()
                if not points or len(points) == 0:
                    print(f"⚠️  {child_prim.GetPath()} 是 Mesh 但无顶点数据，跳过")
                    # continue

            #
            # UsdPhysics.CollisionAPI.Apply(child_prim)
            # child_prim.GetAttribute("physxCollision:approximationShape").Set("convexHull")
            # # UsdPhysics.MeshCollisionAPI.Apply(child_prim)
            # print(f"🔧 为 {child_prim.GetPath()} 应用 CollisionAPI")

            # 1. 应用 MeshCollisionAPI
            # meshcollsion_api 需要先collision_api后再设置
            collisionAPI = UsdPhysics.CollisionAPI.Apply(child_prim)
            meshcollsion_api = UsdPhysics.MeshCollisionAPI.Apply(child_prim)
            meshcollsion_api.GetApproximationAttr().Set(meshOptions[2])
            # print(f"✅ Collision added to {child_prim.GetPath()} with Convex Hull")

            applied_count += 1
        return applied_count

    @staticmethod
    def setup_xform(prim_path):
        """清空指定 Prim 的旧 transform op，并创建一个新的 translate op。"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xform(prim)
        # 清除所有现有操作
        xform.ClearXformOpOrder()
        # 添加一个平移操作
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        return translate_op

    def set_obj_position(self, prim_path, position):
        """设置 Prim 的世界平移位置。position 为 (x, y, z)。"""
        translate_op = self.setup_xform(prim_path)
        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
    # def _load_scene(self):
    #     """
    #     This function is called when the user clicks the Reset Button.
    #     It clears the current stage and creates a new one with a light and a ground plane.
    #     """
    #     # ✅ 使用 World 来统一管理 stage
    #     world = World.instance()
    #     if world is None:
    #         # world = World(stage_units_in_meters=1.0)
    #         world = World()
    #
    #     # # 🔁 reset 会自动创建新 stage
    #     # world.reset()
    #     # print("New USD stage created")
    #
    #     # # ✅ 在 reset 之后添加所有内容
    #     self._add_light_to_stage()
    #     self._add_ground_plane_to_stage()

    @staticmethod
    def get_camera_intrinsics(prim_path):
        """
        返回当前采集分辨率对应的相机内参矩阵 K。

        注意：这里使用固定相机参数和固定分辨率 640x480。
        如果 render_product 分辨率或 Isaac 相机焦距/孔径改变，需要同步更新这里。
        prim_path 参数目前未使用，保留是为了以后从 USD Camera Prim 动态读取参数。
        """

        width = 640  # 默认宽度
        height = 480  # 默认高度
        # 获取物理相机参数（毫米单位）
        focal_length = 24
        horiz_aperture = 20.955
        vert_aperture = 15.2908

        # 计算像素焦距
        fx = (focal_length * width) / horiz_aperture
        fy = (focal_length * height) / vert_aperture

        # 计算主点（通常为图像中心）
        cx = width / 2.0
        cy = height / 2.0

        # 构建内参矩阵 (K)
        K = [[fx, 0, cx],
             [0, fy, cy],
             [0, 0, 1]]

        return K

    def collector_folder_prep(self, folder_name):
        """创建一次采集任务的输出目录：data_log/<folder_name>/。"""
        try:
            print(self.work_dir + "/data_log/{}".format(folder_name))
        except:
            pass
        os.mkdir(self.work_dir + "/data_log/{}".format(folder_name))
        abs_folder_path = self.work_dir + "/data_log/{}".format(folder_name)
        return abs_folder_path

    def get_current_item_info(self):
        """
        读取当前场景中每个随机物体的世界位姿。

        返回的数据会写入 info.json。这里记录的是 obj_i 下实际模型子 Prim 的世界变换，
        不是外层 /World/obj_i 容器的局部 transform，因此更接近可见几何体的真实位姿。
        """
        item_data = []
        # physics = omni.physx.get_physx_scene_query_interface()
        # timeline = omni.timeline.get_timeline_interface()
        # rigid_body_api = omni.physx.get_rigid_body_interface()
        # physx = omni.physx.get_physx_scene_query_interface()
        # if hasattr(physx, 'get_rigid_body_position') and hasattr(physx, 'get_rigid_body_rotation'):
        #     position = physx.get_rigid_body_position(prim_path)
        #     rotation = physx.get_rigid_body_rotation(prim_path)  # 返回carb.Float4, 顺序为(x, y, z, w)
        #     # 转换为(w, x, y, z)
        #     quat = (rotation.w, rotation.x, rotation.y, rotation.z)
        #     return list(position), list(quat)
        for i in range(len(self.object)):
            prim_path = '/World/obj_{}'.format(i)
            prim = self.stage.GetPrimAtPath(prim_path)
            prim_name = prim.GetName()
            # print(prim.GetAllChildrenNames())
            # rigid_body = UsdPhysics.RigidBodyAPI(prim)
            time = Usd.TimeCode.Default()
            # position = physx.get_rigid_body_position(prim_path)
            # rotation = physx.get_rigid_body_rotation(prim_path)
            # transform = rigid_body.get_rigid_pose()
            # pose = rigid_body_api.get_rigid_body_pose(rigid_body)
            # print(dir(rigid_body))
            # print(dir(prim))
            # print(prim.GetAllChildrenNames())

            # _ = prim.GetAllChildrenNames()
            filtered = [
                name for name in prim.GetAllChildrenNames()
                if any(cls in name for cls in self._classes_select)
            ]
            # 模型导入后通常结构为 /World/obj_i/<模型名>，这里定位实际带类别名的子 Prim。
            # currect_prim_path = filtered[0]
            currect_prim_path = '{}/{}'.format(prim_path, filtered[0])
            # currect_prim_path = '{}/{}'.format(prim_path, prim.GetAllChildrenNames()[1])
            current_prim = self.stage.GetPrimAtPath(currect_prim_path)
            # position = rigid_body.GetTargetPositionAttr().Get(time)
            # orientation = rigid_body.GetTargetRotationAttr().Get(time)

            xform = UsdGeom.Xformable(current_prim)

            world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)

            translation: Gf.Vec3d = world_transform.ExtractTranslation()
            rotation: Gf.Rotation = world_transform.ExtractRotation()
            quat: Gf.Quatd = rotation.GetQuat()
            quat_numpy = np.array([quat.GetReal(), *quat.GetImaginary()])
            # rr = {"name": prim.GetAllChildrenNames()[0], "translation": np.array(translation).tolist(), "quat": np.array(quat_numpy).tolist()}
            rr = {"name": filtered[0], "translation": np.array(translation).tolist(), "quat": np.array(quat_numpy).tolist()}
            # print(rr)
            item_data.append(rr)
        return item_data

    # ✅ 2. 在 async 函数中
    async def generate_frames(self, shots_count, cylinder):
        """
        逐帧随机生成相机位姿并推进 Replicator。

        每一帧都会调用 generate_camera_pose()，在给定圆柱采样区域内生成相机位置，
        并让相机朝向 obj_x/obj_y 附近的目标区域。
        """
        for i in range(shots_count):
            print(f'---------------------------------------- Frame {i}')

            # ✅ 定义这一帧要做什么
            with rep.trigger.on_frame():
                with self.camera:
                    print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
                    # pose = generate_camera_pose(
                    #     cylinder_radius=0.5,
                    #     cylinder_height=5,
                    #     obj_xy=np.array([0, 0]),
                    #     upper_cylinder_height=0.5,
                    #     obj_radius=0.25
                    # )
                    pose = generate_camera_pose(
                        cylinder_radius=cylinder["cylinder_radius"],
                        cylinder_height=cylinder["cylinder_height"],
                        obj_xy=np.array([cylinder["obj_x"], cylinder["obj_y"]]),
                        upper_cylinder_height=cylinder["upper_cylinder_height"],
                        obj_radius=cylinder["obj_radius"]
                    )
                    print("Camera Position:", pose["camera_position"])
                    print("Quaternion (x, y, z, w):", pose["quaternion"])
                    print("Euler Angles (pitch, yaw, roll):", pose["euler_angles"])

                    rot = pose['euler_angles'].copy()  # 建议 copy
                    rot[1] -= 90
                    rep.modify.pose(position=pose['camera_position'], rotation=rot)

            # ✅ 推进一帧（真正执行上面定义的逻辑）
            await rep.orchestrator.step_async(
                rt_subframes=8,
                delta_time=None,
                pause_timeline=False
            )

    async def collect_data(self):
        """
        执行一次完整数据采集。

        输出目录：
            data_log/<当前时间>/

        输出内容：
            info.json: 相机内参和物体位姿
            rgb_*.png: RGB 图像
            distance_to_camera_*.npy: 深度
            semantic_segmentation_*.png/json: 语义分割及标签映射
            pointcloud_*.npy / lx_data_*.pcd: 点云及 PCD 文件
        """

        print("✅ collect_data 开始执行")  # 加日志
        if (not len(self.object)):
            print("No object found!")
            return 1
        if self.work_dir is None:
            print("work dir not define, init scene first")
            return 1
        self.stage = omni.usd.get_context().get_stage()
        try:
            shots_count = self._line_edit['num_of_shots'].get_value_as_int()
        except:
            print("Invalid num_obj")
            return 1


        # 采集开始前记录物体稳定后的世界位姿，便于后续标注和抓取位姿分析。
        current_info = self.get_current_item_info()
        # current_info = {}
        folder_name = time.strftime("%Y_%m_%d_%H_%M_%S")
        abs_folder_path = self.collector_folder_prep(folder_name)
        camera_info = UIBuilder.get_camera_intrinsics(self.camera_name)
        info = {}
        info['camera_info'] = camera_info
        info['objects'] = current_info
        with open('{}/info.json'.format(abs_folder_path), 'w') as fp:
            json.dump(info, fp, indent=4)
        # writter = rep.WriterRegistry.get("LxWriter")
        # with rep.trigger.on_frame(num_frames=shots_count):
        if self.writter:
            self.writter.detach()

        # 使用自定义 LxWriter，它在 BasicWriter 基础上额外输出 open3d PCD。
        self.writter = rep.WriterRegistry.get("LxWriter")
        # writter = rep.WriterRegistry.get("BasicWriter")
        # 先 detach
        # writter.detach()

        self.writter.initialize(
            output_dir=abs_folder_path, pointcloud=True, rgb=True,
            distance_to_camera=True, semantic_segmentation=True,
            semantic_types=["select_classes"]
        )
        self.writter.attach([self.render_product])

        # 相机生成位姿的参数。注意这里目前使用 get_value_as_int()，
        # 如果 UI 默认值包含小数，后续建议改成 get_value_as_float()。
        cylinder = {}
        try:
            cylinder["cylinder_radius"] = self._line_edit['cylinder_radius'].get_value_as_int()
        except:
            print("Invalid cylinder_radius")
            return 1
        try:
            cylinder["cylinder_height"] = self._line_edit['cylinder_height'].get_value_as_int()
        except:
            print("Invalid cylinder_height")
            return 1
        try:
            cylinder["obj_x"] = self._line_edit['obj_x'].get_value_as_int()
        except:
            print("Invalid obj_x")
            return 1
        try:
            cylinder["obj_y"] = self._line_edit['obj_y'].get_value_as_int()
        except:
            print("Invalid obj_y")
            return 1
        try:
            cylinder["upper_cylinder_height"] = self._line_edit['upper_cylinder_height'].get_value_as_int()
        except:
            print("Invalid upper_cylinder_height")
            return 1
        try:
            cylinder["obj_radius"] = self._line_edit['obj_radius'].get_value_as_int()
        except:
            print("Invalid obj_radius")
            return 1

        await self.generate_frames(shots_count, cylinder)

        self.writter.detach()
        print("all step is done")

    def collection_on_button_click(self):
        """Start 按钮回调。通过 _is_collecting 防止重复点击启动多个采集任务。"""
        if getattr(self, "_is_collecting", False):
            print("⚠️ 正在采集中，请勿重复点击")
            return

        self._is_collecting = True
        asyncio.ensure_future(self._run_collection())

    async def _run_collection(self):
        """异步采集任务包装器，确保采集失败时也能复位 _is_collecting。"""
        try:
            await self.collect_data()
        finally:
            self._is_collecting = False

    def _setup_scene(self):
        """
        这个函数是由Load Button的setup_scene_fn回调函数调用的。
        在用户点击Load Button时，会创建一个新的World实例，然后调用这个函数。
        用户现在可以将他们的资产加载到Stage上，并将它们添加到World场景中。
        在这个例子中，显式地加载了一个新的Stage，并重新加载了所有资产。
        如果用户依赖于热重载功能，并且不希望在每次加载资产时都重新加载，
        他们可以在这里进行检查，以查看他们所需的资产是否已经在Stage上，
        并避免重新加载任何内容，除非它们已经存在。
        
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.

        In this example, a new stage is loaded explicitly, and all assets are reloaded.
        If the user is relying on hot-reloading and does not want to reload assets every time,
        they may perform a check here to see if their desired assets are already on the stage,
        and avoid loading anything if they are.  In this case, the user would still need to add
        their assets to the World (which has low overhead).  See commented code section in this function.
        """
        # 清理原有场景并新建带光线和地面的基础场景
        # self._reset_scene()
        
        # self._load_scene()
        # print("Loading Scene")
        # Load the UR10e
        self.world.scene.add_default_ground_plane()

        robot_prim_path = "/ur10e"
        
        obj_path = ""
        
        # path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        parent_folder = os.path.split(__file__)[0]
        path_to_robot_usd = os.path.abspath(os.path.join(parent_folder, "..", "data", "raw_data", "sqm", "sqm.usd"))

        # Do not reload assets when hot reloading.  This should only be done while extension is under development.
        # if not is_prim_path_valid(robot_prim_path):
        #     create_new_stage()
        #     add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        # else:
        #     print("Robot already on Stage")

        # 强制清除原有场景
        # create_new_stage()
        # self._add_light_to_stage()

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)

        obj_path = path_to_robot_usd
        prim_path = robot_prim_path

        # 添加到场景
        added_obj = add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
        # 将添加的对象存入列表 (如果 self.object 是用来存储这些引用的)
        # self.object.append(added_obj)
        # 或者直接使用 prim_path 操作，通常更可靠
        # self.object.append(prim_path)  # 存储路径可能比存储 add_reference_to_stage 的返回值更通用

        # --- 添加刚体和碰撞属性 ---
        # 1. 获取物体的 Prim
        obj_prim = self.stage.GetPrimAtPath(prim_path)
        if not obj_prim:
            print(f"❌ 错误：无法找到 Prim {prim_path}，跳过物理属性添加。")

        # 2. 应用 Physics Rigid Body API
        # 检查是否已存在
        if not UsdPhysics.RigidBodyAPI(obj_prim):
            applied_rb_api = UsdPhysics.RigidBodyAPI.Apply(obj_prim)
            if applied_rb_api:
                print(f"  ✅ 已应用 Physics RigidBody API 到 {prim_path}")
            else:
                print(f"  ⚠️ 无法为 {prim_path} 应用 Physics RigidBody API。")
                # 可以选择跳过这个物体
                # continue
        else:
            print(f"  ℹ️  {prim_path} 已存在 Physics RigidBody API。")

        # 3. (可选) 应用 Mass API 并设置质量
        # 检查是否已存在 MassAPI
        mass_api = UsdPhysics.MassAPI(obj_prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

        if mass_api:
            # 检查质量属性是否存在，如果不存在则创建并设置
            mass_attr = mass_api.GetMassAttr()
            if not mass_attr:
                mass_attr = mass_api.CreateMassAttr()
            # 设置质量值 (例如 1.0 kg)
            mass_attr.Set(1.0)
            print(f"  ✅ 已设置 {prim_path} 的质量为 1.0 kg")
        else:
            print(f"  ⚠️ 无法为 {prim_path} 应用或获取 Mass API。")

        # 4. 应用 Physics Collision API (使用 MeshCollisionAPI)
        # 检查是否已存在任何碰撞API

        applied_collision_api = UsdPhysics.MeshCollisionAPI.Apply(obj_prim)
        if applied_collision_api:
            print(f"  ✅ 已应用 Mesh Collision API 到 {prim_path}")
        else:
            print(f"  ⚠️ 无法为 {prim_path} 应用 Mesh Collision API。")




    # def _setup_scenario(self):
    #     """
    #     This function is attached to the Load Button as the setup_post_load_fn callback.
    #     The user may assume that their assets have been loaded by their setup_scene_fn callback, that
    #     their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
    #     provided prim in a circle around the robot.
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True
    #     self._reset_btn.enabled = True

    # def _reset_scenario(self):
    #     self._scenario.teardown_scenario()
    #     self._scenario.setup_scenario(self._articulation, self._cuboid)

    # def _on_post_reset_btn(self):
    #     """
    #     This function is attached to the Reset Button as the post_reset_fn callback.
    #     The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     They may also assume that objects that were added to the World.Scene have been moved to their default positions.
    #     I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True

    # def _update_scenario(self, step: float):
    #     """This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the physics_callback_fn argument.
    #     This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
    #     When the b_text "STOP" is pressed, the physics callback is removed.

    #     Args:
    #         step (float): The dt of the current physics step
    #     """
    #     self._scenario.update_scenario(step)

    # def _on_run_scenario_a_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_a_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "RUN".

    #     This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
    #     the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
    #     through the left-hand UI toolbar.
    #     """
    #     self._timeline.play()

    # def _on_run_scenario_b_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_b_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "STOP"

    #     Pausing the timeline on b_text is not strictly necessary for this example to run.
    #     Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
    #     the robot will stop getting new commands and the cube will stop updating without needing to
    #     pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
    #     forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
    #     this example prettier, but if curious, the user should observe what happens when this line is removed.
    #     """
    #     self._timeline.pause()

    # def _reset_extension(self):
    #     """This is called when the user opens a new stage from self.on_stage_event().
    #     All state should be reset.
    #     """
    #     self._on_init()
    #     self._reset_ui()

    # def _reset_ui(self):
        # self._scenario_state_btn.reset()
        # self._scenario_state_btn.enabled = False
        # self._reset_btn.enabled = False
        
    

    
