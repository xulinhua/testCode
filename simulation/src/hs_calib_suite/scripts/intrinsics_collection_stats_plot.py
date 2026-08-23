#!/usr/bin/env python3
"""Tier4 Calibration data statistics — 5 columns × 3 rows."""

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from plot_common import (  # noqa: E402
    add_column_headers,
    apply_tier4_style,
    bin_points_pixel,
    collection_figsize,
    finalize_grid_figure,
    load_json,
    plot_pixel_heatmap,
    plot_rotation_heatmap,
    plot_z_histogram,
)


def main() -> int:
    if len(sys.argv) < 3:
        print(
            "usage: intrinsics_collection_stats_plot.py <input.json> <output.png>",
            file=sys.stderr,
        )
        return 2
    apply_tier4_style()
    data = load_json(sys.argv[1])
    width = int(data.get("image_width", 640))
    height = int(data.get("image_height", 480))
    cells = int(data.get("heatmap_cells", 16))
    angle_res = float(data.get("rotation_angle_res", 10))
    max_tilt = float(data.get("max_tilt_deg", 40))
    z_bins = int(data.get("z_bins", 12))
    stages = data.get("stages", [])
    ncols = max(1, len(stages))
    fig, axes = plt.subplots(
        3, ncols, figsize=collection_figsize(ncols), facecolor="white"
    )
    if ncols == 1:
        axes = axes.reshape(3, 1)
    stage_names = []
    for col, stage in enumerate(stages):
        name = stage.get("name", f"Stage {col}")
        stage_names.append(name)
        pts = stage.get("pixel_points", [])
        frames = stage.get("frames", [])
        grid = bin_points_pixel(pts, width, height, cells)
        plot_pixel_heatmap(axes[0, col], grid, width, height, name, title_pad=2.0)
        plot_rotation_heatmap(axes[1, col], frames, angle_res, max_tilt, name)
        plot_z_histogram(axes[2, col], frames, z_bins, name)
    fig.suptitle("Calibration data statistics", fontsize=13, y=0.98)
    finalize_grid_figure(fig, top=0.84, wspace=1.05, hspace=0.62)
    add_column_headers(fig, axes[0, :], stage_names, gap=0.062)
    fig.savefig(sys.argv[2], dpi=120, facecolor="white")
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
