# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from pxr import UsdPhysics

# 扩展窗口标题，同时也作为菜单项名称使用。
EXTENSION_TITLE = "Ext_Grasing_Scene"

# 扩展描述，主要用于日志和扩展信息展示。
EXTENSION_DESCRIPTION = "Grasping Scene Data Capture"

# 当前支持的数据类别数量。新增类别时，需要同步修改 class_names。
num_classes = 4

# 类别名必须与 data/raw_data/<类别名>/ 目录以及 USD 文件语义标签逻辑保持一致。
# load_world() 会根据用户勾选的类别到这些目录下查找 .usd/.usda/.usdc 模型。
class_names = ['bwb', 'sqm', 'yida', 'dingshuji']


# USD/PhysX 支持的 Mesh 碰撞近似方式。
# ui_builder.apply_collision_to_mesh_children() 当前使用 meshOptions[2]，即 convexHull。
# 如果物体形状复杂但采集时只需要稳定下落，convexHull 通常比原始三角网格更稳更快。
meshOptions = [
    UsdPhysics.Tokens.none,
    UsdPhysics.Tokens.meshSimplification,
    UsdPhysics.Tokens.convexHull,
    UsdPhysics.Tokens.convexDecomposition,
    UsdPhysics.Tokens.boundingSphere,
    UsdPhysics.Tokens.boundingCube
]

# class_select = []



