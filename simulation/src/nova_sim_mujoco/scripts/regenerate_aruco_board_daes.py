#!/usr/bin/env python3
"""Regenerate aruco_6x6_1000_id*.dae thin boxes for MuJoCo (watertight volume + UV)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

H = 0.001


def build_box():
    m = trimesh.creation.box(extents=[0.1, 0.1, H])
    m.apply_translation([0, 0, -H / 2])
    assert m.is_watertight and m.volume > 1e-10
    return m


def uv_top_plus_z(x: float, y: float) -> tuple[float, float]:
    """Marker face (+Z): 与旧单面平面视觉对齐（相对之前 box 实现矫正左右反）。"""
    u = (x + 0.05) / 0.1
    v = (y + 0.05) / 0.1
    return float(u), float(v)


def uv_bottom_minus_z(x: float, y: float) -> tuple[float, float]:
    """底面（-Z 外法向）：从板下往上看时左右与顶面一致。"""
    u = (0.05 - x) / 0.1
    v = (y + 0.05) / 0.1
    return float(u), float(v)


def corner_uv(pos: np.ndarray, normal: np.ndarray) -> tuple[float, float]:
    x, y, _z = float(pos[0]), float(pos[1]), float(pos[2])
    nx, ny, nz = float(normal[0]), float(normal[1]), float(normal[2])
    if nz > 0.5:
        return uv_top_plus_z(x, y)
    if nz < -0.5:
        return uv_bottom_minus_z(x, y)
    return 0.5, 0.5


def write_dae(mid: int, out: Path, base: trimesh.Trimesh) -> None:
    pos_list: list[tuple[float, float, float]] = []
    nrm_list: list[tuple[float, float, float]] = []
    uv_list: list[tuple[float, float]] = []
    for tri in base.faces:
        pts = base.vertices[tri]
        e1, e2 = pts[1] - pts[0], pts[2] - pts[0]
        fn = np.cross(e1, e2)
        fn = fn / np.linalg.norm(fn)
        for vi in tri:
            p = base.vertices[vi]
            pos_list.append(tuple(float(c) for c in p))
            nrm_list.append(tuple(float(c) for c in fn))
            uv_list.append(corner_uv(p, fn))
    pos_flat = "\n            ".join(" ".join(f"{c:.10g}" for c in p) for p in pos_list)
    nrm_flat = "\n            ".join(" ".join(f"{c:.10g}" for c in n) for n in nrm_list)
    uv_flat = "\n            ".join(" ".join(f"{c:.10g}" for c in uv) for uv in uv_list)
    pstr = " ".join(f"{i} {i} {i}" for i in range(len(pos_list)))
    png = f"6x6_1000-{mid}.png"
    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>regenerate_aruco_board_daes.py id{mid}</authoring_tool>
    </contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_images>
    <image id="aruco_png" name="aruco_png">
      <init_from>{png}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="aruco_fx">
      <profile_COMMON>
        <newparam sid="aruco_surface">
          <surface type="2D">
            <init_from>aruco_png</init_from>
          </surface>
        </newparam>
        <newparam sid="aruco_sampler">
          <sampler2D>
            <source>aruco_surface</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <diffuse>
              <texture texture="aruco_sampler" texcoord="UVMap"/>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="aruco_mat" name="aruco_mat">
      <instance_effect url="#aruco_fx"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="aruco_slab" name="aruco_slab">
      <mesh>
        <source id="aruco_positions">
          <float_array id="aruco_positions_array" count="{len(pos_list) * 3}">
            {pos_flat}
          </float_array>
          <technique_common>
            <accessor source="#aruco_positions_array" count="{len(pos_list)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="aruco_normals">
          <float_array id="aruco_normals_array" count="{len(nrm_list) * 3}">
            {nrm_flat}
          </float_array>
          <technique_common>
            <accessor source="#aruco_normals_array" count="{len(nrm_list)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="aruco_uvs">
          <float_array id="aruco_uvs_array" count="{len(uv_list) * 2}">
            {uv_flat}
          </float_array>
          <technique_common>
            <accessor source="#aruco_uvs_array" count="{len(uv_list)}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="aruco_vertices">
          <input semantic="POSITION" source="#aruco_positions"/>
        </vertices>
        <triangles material="aruco_mat" count="12">
          <input semantic="VERTEX" source="#aruco_vertices" offset="0"/>
          <input semantic="NORMAL" source="#aruco_normals" offset="1"/>
          <input semantic="TEXCOORD" source="#aruco_uvs" offset="2" set="0"/>
          <p>{pstr}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="aruco_node" name="aruco_node" type="NODE">
        <instance_geometry url="#aruco_slab">
          <bind_material>
            <technique_common>
              <instance_material symbol="aruco_mat" target="#aruco_mat"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
'''
    out.write_text(xml, encoding="utf-8")


def main() -> None:
    mesh_dir = Path(__file__).resolve().parent.parent / "meshes"
    base = build_box()
    for mid in (0, 1, 2, 3):
        write_dae(mid, mesh_dir / f"aruco_6x6_1000_id{mid}.dae", base)
        print("wrote", mesh_dir / f"aruco_6x6_1000_id{mid}.dae")


if __name__ == "__main__":
    main()
