# -*- coding: utf-8 -*-
"""OmniGraph 创建/销毁辅助。"""

from __future__ import annotations


def destroy_omni_graph(graph_path: str) -> None:
    """删除指定路径上的 OmniGraph ActionGraph prim。

    Isaac Sim 5 无 ``destroy_graph`` API，通过 ``DeletePrimsCommand`` 删除图根节点。

    Args:
        graph_path: 例如 ``/NovaGraspNet/CameraGraph/cam0``。
    """
    try:
        import omni.usd
        from omni.usd.commands import DeletePrimsCommand

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        prim = stage.GetPrimAtPath(graph_path)
        if prim and prim.IsValid():
            DeletePrimsCommand([graph_path]).do()
    except Exception as exc:
        print(f"destroy_omni_graph({graph_path}): {exc}")
