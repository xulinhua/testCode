#!/usr/bin/env python3
"""URDF -> USD via Isaac Sim URDF importer (headless)."""

from __future__ import annotations

import argparse
import os
import sys


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert URDF to USD using Isaac Sim")
    p.add_argument("--urdf", required=True, help="Input URDF path")
    p.add_argument("--usd", required=True, help="Output USD path")
    p.add_argument("--merge-fixed-joints", action="store_true", default=False)
    p.add_argument("--fix-base", action="store_true", default=False)
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    urdf_path = os.path.abspath(args.urdf)
    dest_path = os.path.abspath(args.usd)

    if not os.path.isfile(urdf_path):
        print(f"URDF not found: {urdf_path}", file=sys.stderr)
        return 1
    os.makedirs(os.path.dirname(dest_path) or ".", exist_ok=True)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": True,
            "create_new_stage": True,
            "extra_args": ["--enable", "isaacsim.asset.importer.urdf"],
        }
    )

    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")

        import omni.kit.commands
        from pxr import Usd

        for _ in range(8):
            simulation_app.update()

        _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.import_inertia_tensor = True
        import_config.merge_fixed_joints = args.merge_fixed_joints
        import_config.fix_base = args.fix_base
        import_config.make_default_prim = True

        print(f"Importing URDF: {urdf_path}")
        print(f"Writing USD: {dest_path}")
        robot_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path=dest_path,
        )
        for _ in range(8):
            simulation_app.update()

        if not os.path.isfile(dest_path):
            print(f"ERROR: USD not created: {dest_path}", file=sys.stderr)
            return 1

        stage = Usd.Stage.Open(dest_path)
        if stage is None:
            print(f"ERROR: cannot open USD: {dest_path}", file=sys.stderr)
            return 1

        default_prim = stage.GetDefaultPrim()
        print(f"OK robot_path={robot_path}")
        print(f"OK default_prim={default_prim.GetPath() if default_prim else '(none)'}")
        print(f"OK bytes={os.path.getsize(dest_path)}")
        return 0
    except Exception as exc:
        import traceback

        traceback.print_exc()
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
