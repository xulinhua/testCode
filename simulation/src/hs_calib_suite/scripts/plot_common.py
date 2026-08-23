#!/usr/bin/env python3
"""Shared helpers for Tier4 intrinsics statistics plots."""

from __future__ import annotations

import json
from typing import List, Sequence

import numpy as np


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def apply_tier4_style() -> None:
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "axes.edgecolor": "black",
            "axes.labelcolor": "black",
            "xtick.color": "black",
            "ytick.color": "black",
            "text.color": "black",
            "font.size": 8,
            "axes.titlesize": 7,
        }
    )


_STAGE_SHORT_NAMES = {
    "Training": "Training",
    "Pre rejection inliers": "Pre rej.",
    "Subsampled": "Subsampled",
    "Post rejection inliers": "Post rej.",
    "Evaluation": "Evaluation",
    "Inliers": "Inliers",
}


def short_stage_name(name: str) -> str:
    return _STAGE_SHORT_NAMES.get(name, name)


def set_compact_title(ax, *lines: str, fontsize: float = 6.5, pad: float = 4.0) -> None:
    text = "\n".join(line for line in lines if line)
    ax.set_title(text, fontsize=fontsize, pad=pad, loc="center")


def finalize_grid_figure(
    fig,
    *,
    top: float = 0.90,
    wspace: float = 0.95,
    hspace: float = 0.58,
) -> None:
    """Leave room for multi-line titles, colorbars, and column headers."""
    fig.subplots_adjust(
        left=0.04,
        right=0.995,
        top=top,
        bottom=0.07,
        wspace=wspace,
        hspace=hspace,
    )


def add_column_headers(
    fig, axes_row, stage_names: Sequence[str], *, gap: float = 0.055
) -> None:
    """Stage name once per column (above top row), avoids long subplot titles."""
    for col, name in enumerate(stage_names):
        ax = axes_row[col]
        pos = ax.get_position()
        fig.text(
            pos.x0 + pos.width * 0.5,
            pos.y1 + gap,
            short_stage_name(name),
            ha="center",
            va="bottom",
            fontsize=8,
            fontweight="bold",
            transform=fig.transFigure,
        )


def collection_figsize(ncols: int) -> tuple[float, float]:
    return (max(4.5 * ncols, 14.0), 10.8)


def shannon_entropy(grid: np.ndarray) -> float:
    g = grid.astype(np.float64).ravel()
    total = g.sum()
    if total <= 0:
        return 0.0
    p = g[g > 0] / total
    return float(-np.sum(p * np.log2(p)))


def bin_points_pixel(
    points: Sequence[Sequence[float]], width: int, height: int, cells: int
) -> np.ndarray:
    grid = np.zeros((cells, cells), dtype=np.float64)
    if width <= 0 or height <= 0:
        return grid
    for pt in points:
        if len(pt) < 2:
            continue
        x, y = float(pt[0]), float(pt[1])
        ix = min(cells - 1, max(0, int(x / width * cells)))
        iy = min(cells - 1, max(0, int(y / height * cells)))
        grid[iy, ix] += 1.0
    return grid


def plt_colorbar(im, ax) -> None:
    import matplotlib.pyplot as plt

    cbar = plt.colorbar(im, ax=ax, fraction=0.042, pad=0.02)
    cbar.ax.tick_params(labelsize=6)


def plot_pixel_heatmap(
    ax, grid: np.ndarray, width: int, height: int, stage_name: str, *, title_pad: float = 4.0
) -> None:
    ent = shannon_entropy(grid)
    extent = [0, width, height, 0]
    masked = np.ma.masked_where(grid <= 0, grid)
    im = ax.imshow(
        masked,
        origin="upper",
        extent=extent,
        cmap="jet",
        aspect="auto",
        interpolation="nearest",
    )
    set_compact_title(ax, "pixel heatmap", f"entropy={ent:.2f}", pad=title_pad)
    ax.set_xlabel("x (px)", fontsize=7)
    ax.set_ylabel("y (px)", fontsize=7)
    ax.tick_params(labelsize=6)
    plt_colorbar(im, ax)


def plot_rotation_heatmap(
    ax, frames: Sequence[dict], angle_res: float, max_tilt: float, stage_name: str
) -> None:
    axs = [float(f.get("angle_x", 0.0)) for f in frames if f.get("has_z", True)]
    ays = [float(f.get("angle_y", 0.0)) for f in frames if f.get("has_z", True)]
    if not axs:
        set_compact_title(ax, "rotation heatmap", "empty")
        return
    half = max(5.0, max_tilt)
    step = max(1.0, angle_res)
    bins = max(4, int(round(2.0 * half / step)))
    h, xe, ye = np.histogram2d(
        axs, ays, bins=bins, range=[[-half, half], [-half, half]]
    )
    ent = shannon_entropy(h)
    extent = [xe[0], xe[-1], ye[0], ye[-1]]
    masked = np.ma.masked_where(h.T <= 0, h.T)
    im = ax.imshow(
        masked,
        origin="lower",
        extent=extent,
        cmap="jet",
        aspect="auto",
        interpolation="nearest",
    )
    set_compact_title(ax, "rotation heatmap", f"entropy={ent:.2f}")
    ax.set_xlabel("angle_x (deg)", fontsize=7)
    ax.set_ylabel("angle_y (deg)", fontsize=7)
    ax.tick_params(labelsize=6)
    plt_colorbar(im, ax)


def plot_z_histogram(ax, frames: Sequence[dict], z_bins: int, stage_name: str) -> None:
    zs = [float(f.get("z", 0.0)) for f in frames if f.get("has_z", False)]
    set_compact_title(ax, "z histogram")
    if not zs:
        ax.text(0.5, 0.5, "no pose", ha="center", va="center", transform=ax.transAxes)
        return
    ax.hist(
        zs,
        bins=max(4, z_bins),
        color="#4a90d9",
        edgecolor="black",
        linewidth=0.6,
    )
    ax.set_xlabel("z (m)", fontsize=7)
    ax.set_ylabel("count", fontsize=7)
    ax.tick_params(labelsize=6)


def aggregate_rms_grid(
    points: Sequence[dict],
    key_u: str,
    key_v: str,
    width: int,
    height: int,
    cells: int,
    stat: str,
    max_tilt: float = 40.0,
) -> np.ndarray:
    buckets: List[List[List[float]]] = [
        [[] for _ in range(cells)] for _ in range(cells)
    ]
    for p in points:
        u = float(p.get(key_u, 0.0))
        v = float(p.get(key_v, 0.0))
        err = float(p.get("err", 0.0))
        if width > 0 and height > 0:
            ix = min(cells - 1, max(0, int(u / width * cells)))
            iy = min(cells - 1, max(0, int(v / height * cells)))
        else:
            half = max(5.0, max_tilt)
            ix = min(cells - 1, max(0, int((u + half) / (2.0 * half) * cells)))
            iy = min(cells - 1, max(0, int((v + half) / (2.0 * half) * cells)))
        buckets[iy][ix].append(err)

    out = np.full((cells, cells), np.nan, dtype=np.float64)
    for iy in range(cells):
        for ix in range(cells):
            vals = buckets[iy][ix]
            if not vals:
                continue
            if stat == "mean":
                out[iy, ix] = float(np.mean(vals))
            else:
                out[iy, ix] = float(np.std(vals)) if len(vals) > 1 else 0.0
    return out


def plot_rms_heatmap(ax, grid: np.ndarray, extent, stage_name: str, metric: str) -> None:
    masked = np.ma.masked_invalid(grid)
    masked = np.ma.masked_where(masked <= 0, masked)
    im = ax.imshow(
        masked,
        origin="upper" if extent[2] > extent[3] else "lower",
        extent=extent,
        cmap="jet",
        aspect="auto",
        interpolation="nearest",
    )
    set_compact_title(ax, f"{short_stage_name(stage_name)}", f"rms error ({metric})")
    ax.tick_params(labelsize=6)
    plt_colorbar(im, ax)
