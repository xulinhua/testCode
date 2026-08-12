# -*- coding: utf-8 -*-
"""Camera orbit for hand-held-like calibration capture.

Always looking exactly at board center makes image fingerprints (board area /
center / tilt) almost invariant under azimuth, so auto-capture saturates at a
handful of frames. This motion adds:
  - wider distance / elevation excursion
  - oscillating look-at offset on the board plane (moves board in image)
  - mild camera roll (extra tilt diversity)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from ..global_variables import (
    DEFAULT_AZIM_RATE_DEG,
    DEFAULT_DIST_MAX,
    DEFAULT_DIST_MIN,
    DEFAULT_DIST_PERIOD_S,
    DEFAULT_ELEV_MAX_DEG,
    DEFAULT_ELEV_MIN_DEG,
    DEFAULT_ELEV_PERIOD_S,
    DEFAULT_LOOKAT_OFFSET_M,
    DEFAULT_LOOKAT_PERIOD_S,
    DEFAULT_ROLL_AMP_DEG,
    DEFAULT_ROLL_PERIOD_S,
)


@dataclass
class OrbitParams:
    dist_min: float = DEFAULT_DIST_MIN
    dist_max: float = DEFAULT_DIST_MAX
    elev_min_deg: float = DEFAULT_ELEV_MIN_DEG
    elev_max_deg: float = DEFAULT_ELEV_MAX_DEG
    azim_rate_deg: float = DEFAULT_AZIM_RATE_DEG
    dist_period_s: float = DEFAULT_DIST_PERIOD_S
    elev_period_s: float = DEFAULT_ELEV_PERIOD_S
    # Off-center aim on board plane (meters) — key for cx/cy diversity
    lookat_offset_m: float = DEFAULT_LOOKAT_OFFSET_M
    lookat_period_s: float = DEFAULT_LOOKAT_PERIOD_S
    # Camera roll amplitude (degrees)
    roll_amp_deg: float = DEFAULT_ROLL_AMP_DEG
    roll_period_s: float = DEFAULT_ROLL_PERIOD_S
    enabled: bool = True


class CameraOrbitMotion:
    """Spherical orbit with look-at offset + roll for calibration diversity."""

    def __init__(self, params: OrbitParams | None = None):
        self.params = params or OrbitParams()
        self._t = 0.0
        self._azim0_deg = 0.0

    def reset(self, azim0_deg: float = 0.0) -> None:
        self._t = 0.0
        self._azim0_deg = float(azim0_deg)

    def set_params(self, params: OrbitParams) -> None:
        self.params = params

    def step(
        self, dt: float, target: Tuple[float, float, float]
    ) -> Tuple[
        Tuple[float, float, float],
        Tuple[float, float, float],
        float,
        float,
        float,
        float,
    ]:
        """Advance dt seconds.

        Returns:
            eye_xyz, lookat_xyz, distance, elev_deg, azim_deg, roll_deg
        """
        if self.params.enabled:
            self._t += max(0.0, float(dt))
        t = self._t
        p = self.params

        dist_mid = 0.5 * (p.dist_min + p.dist_max)
        dist_amp = 0.5 * max(0.0, p.dist_max - p.dist_min)
        elev_mid = 0.5 * (p.elev_min_deg + p.elev_max_deg)
        elev_amp = 0.5 * max(0.0, p.elev_max_deg - p.elev_min_deg)

        wd = 2.0 * math.pi / max(1e-3, p.dist_period_s)
        we = 2.0 * math.pi / max(1e-3, p.elev_period_s)
        wo = 2.0 * math.pi / max(1e-3, p.lookat_period_s)
        wr = 2.0 * math.pi / max(1e-3, p.roll_period_s)

        # Slightly phase-shifted sinusoids so the path is denser / less periodic
        distance = dist_mid + dist_amp * math.sin(wd * t)
        elev_deg = elev_mid + elev_amp * math.sin(we * t + 0.7)
        azim_deg = self._azim0_deg + p.azim_rate_deg * t
        roll_deg = p.roll_amp_deg * math.sin(wr * t + 1.1)

        # Look-at wanders on the board plane (XY) in a Lissajous pattern so the
        # board drifts across the image — this is what auto-capture fingerprints.
        off = max(0.0, float(p.lookat_offset_m))
        look_x = target[0] + off * math.sin(wo * t)
        look_y = target[1] + off * math.sin(wo * t * 1.37 + 0.9)
        look_z = target[2] + 0.35 * off * math.sin(wo * t * 0.81 + 0.4)
        lookat = (look_x, look_y, look_z)

        elev = math.radians(elev_deg)
        azim = math.radians(azim_deg)
        # Z-up: elevation from horizontal plane, orbit around board center
        # (not lookat) so distance stays meaningful for board size in image.
        x = target[0] + distance * math.cos(elev) * math.cos(azim)
        y = target[1] + distance * math.cos(elev) * math.sin(azim)
        z = target[2] + distance * math.sin(elev)
        return (x, y, z), lookat, distance, elev_deg, azim_deg, roll_deg
