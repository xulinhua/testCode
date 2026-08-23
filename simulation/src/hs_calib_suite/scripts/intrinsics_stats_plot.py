#!/usr/bin/env python3
"""Tier4-style intrinsics collection statistics (matplotlib Agg backend)."""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def grid_from_samples(samples, cells: int) -> np.ndarray:
    g = np.zeros((cells, cells), dtype=np.int32)
    for s in samples:
        cx = float(s.get("cx", 0.5))
        cy = float(s.get("cy", 0.5))
        ix = min(cells - 1, max(0, int(cx * cells)))
        iy = min(cells - 1, max(0, int(cy * cells)))
        g[iy, ix] += 1
    return g


def plot_rotation_heatmap(ax, samples, angle_res: int, max_tilt: float) -> None:
    ax.set_facecolor("#0c1018")
    if not samples:
        ax.set_title("Rotation heatmap (empty)", color="#d0d8e4")
        return
    axs = [float(s.get("angle_x", 0.0)) for s in samples]
    ays = [float(s.get("angle_y", 0.0)) for s in samples]
    bins = max(4, int(max_tilt / max(angle_res, 1)))
    h, xe, ye, im = ax.hist2d(axs, ays, bins=bins, cmap="inferno")
    ax.set_xlabel("angle_x (deg)", color="#b8c4d4")
    ax.set_ylabel("angle_y (deg)", color="#b8c4d4")
    ax.set_title("Rotation heatmap", color="#d0d8e4")
    ax.tick_params(colors="#9aa8b8")
    for spine in ax.spines.values():
        spine.set_color("#4a5668")
    cbar = plt.colorbar(im, ax=ax, fraction=0.046)
    cbar.ax.yaxis.set_tick_params(color="#9aa8b8")
    plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#9aa8b8")


def plot_centroid_scatter(ax, samples, title: str) -> None:
    ax.set_facecolor("#0c1018")
    if not samples:
        ax.set_title(title + " (empty)")
        return
    xs = [float(s.get("cx", 0.5)) for s in samples]
    ys = [float(s.get("cy", 0.5)) for s in samples]
    ax.scatter(
        xs, ys, s=42, c="#5aff9a", edgecolors="#0a1410", linewidths=0.8, alpha=0.92)
    ax.set_xlim(0, 1)
    ax.set_ylim(1, 0)
    ax.set_xlabel("cx (norm)")
    ax.set_ylabel("cy (norm)")
    ax.set_title(title, color="#d0d8e4")
    ax.set_aspect("equal")
    ax.tick_params(colors="#9aa8b8")
    for spine in ax.spines.values():
        spine.set_color("#4a5668")


def plot_occupancy(ax, grid: np.ndarray, title: str) -> None:
    ax.set_facecolor("#0c1018")
    im = ax.imshow(grid, origin="upper", cmap="inferno", interpolation="nearest")
    ax.set_title(title, color="#d0d8e4")
    cbar = plt.colorbar(im, ax=ax, fraction=0.046)
    cbar.ax.yaxis.set_tick_params(color="#9aa8b8")
    plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#9aa8b8")


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: intrinsics_stats_plot.py <input.json> <output.png>", file=sys.stderr)
        return 2
    data = load(sys.argv[1])
    cells = int(data.get("heatmap_cells", 16))
    angle_res = int(data.get("rotation_heatmap_angle_res", 10))
    max_tilt = float(data.get("max_tilt_deg", 45.0))
    train = data.get("training", [])
    eval_ = data.get("evaluation", [])

    fig, axes = plt.subplots(2, 2, figsize=(10, 8), facecolor="#121820")
    plot_rotation_heatmap(axes[0, 0], train + eval_, angle_res, max_tilt)
    plot_centroid_scatter(axes[0, 1], train, "Training centroids")
    plot_occupancy(axes[1, 0], grid_from_samples(train, cells), "Training occupancy")
    plot_occupancy(axes[1, 1], grid_from_samples(eval_, cells), "Evaluation occupancy")
    fig.suptitle(
        f"Collection stats — train={len(train)} eval={len(eval_)}",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=120)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
