# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
# Isaac Sim 会根据 config/extension.toml 中的 [[python.module]] 配置导入本包。
# 这里使用通配导入把 extension.py 中的 Extension 类暴露给 Omniverse 扩展系统。
# 一般不要在这里放业务逻辑，避免扩展扫描阶段就执行耗时操作。
from .extension import *
