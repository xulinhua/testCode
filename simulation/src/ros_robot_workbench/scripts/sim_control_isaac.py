#!/usr/bin/env python3
"""Isaac Sim headless simulation control (play/pause/step/reset/load USD)."""

from __future__ import annotations

import argparse
import os
import sys


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Isaac Sim control commands")
    p.add_argument(
        "--cmd",
        required=True,
        choices=["play", "pause", "step", "reset", "load_scene"],
        help="Control command",
    )
    p.add_argument("--usd", default="", help="USD scene path for load_scene")
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True, "create_new_stage": True})

    try:
        import omni.timeline
        import omni.usd
        from pxr import Usd

        timeline = omni.timeline.get_timeline_interface()

        if args.cmd == "load_scene":
            usd_path = os.path.abspath(args.usd)
            if not usd_path or not os.path.isfile(usd_path):
                print(f"USD not found: {usd_path}", file=sys.stderr)
                return 1
            omni.usd.get_context().open_stage(usd_path)
            print(f"Loaded USD: {usd_path}")
            return 0

        if args.cmd == "play":
            timeline.play()
            print("play")
        elif args.cmd == "pause":
            timeline.pause()
            print("pause")
        elif args.cmd == "step":
            timeline.pause()
            # advance one frame
            app_update = getattr(simulation_app, "update", None)
            if callable(app_update):
                app_update()
            print("step")
        elif args.cmd == "reset":
            timeline.stop()
            stage = omni.usd.get_context().get_stage()
            if stage:
                stage.SetEditTarget(stage.GetRootLayer())
            timeline.play()
            print("reset")
        return 0
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
