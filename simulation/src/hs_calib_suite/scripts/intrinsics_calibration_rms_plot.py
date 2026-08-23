#!/usr/bin/env python3
"""Tier4 Calibration result statistics — RMS mean/std heatmaps."""

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from plot_common import (  # noqa: E402
    aggregate_rms_grid,
    apply_tier4_style,
    finalize_grid_figure,
    load_json,
    plot_rms_heatmap,
)


def main() -> int:
    if len(sys.argv) < 3:
        print(
            "usage: intrinsics_calibration_rms_plot.py <input.json> <output.png>",
            file=sys.stderr,
        )
        return 2
    apply_tier4_style()
    data = load_json(sys.argv[1])
    width = int(data.get("image_width", 640))
    height = int(data.get("image_height", 480))
    cells = int(data.get("viz_pixel_cells", 16))
    max_tilt = float(data.get("viz_max_tilt_deg", 40))
    sets = data.get("sets", [])
    nrows = max(1, len(sets))
    fig, axes = plt.subplots(nrows, 4, figsize=(17.5, 3.8 * nrows), facecolor="white")
    if nrows == 1:
        axes = axes.reshape(1, 4)
    pix_extent = [0, width, height, 0]
    rot_extent = [-max_tilt, max_tilt, max_tilt, -max_tilt]
    for row, item in enumerate(sets):
        name = item.get("name", f"Set {row}")
        pts = item.get("points", [])
        mean_pix = aggregate_rms_grid(pts, "u", "v", width, height, cells, "mean")
        std_pix = aggregate_rms_grid(pts, "u", "v", width, height, cells, "std")
        mean_rot = aggregate_rms_grid(
            pts, "angle_x", "angle_y", 0, 0, cells, "mean", max_tilt
        )
        std_rot = aggregate_rms_grid(
            pts, "angle_x", "angle_y", 0, 0, cells, "std", max_tilt
        )
        plot_rms_heatmap(axes[row, 0], mean_pix, pix_extent, name, "mean")
        plot_rms_heatmap(axes[row, 1], std_pix, pix_extent, name, "std")
        plot_rms_heatmap(axes[row, 2], mean_rot, rot_extent, name, "mean")
        plot_rms_heatmap(axes[row, 3], std_rot, rot_extent, name, "std")
    fig.suptitle("Calibration result statistics", fontsize=12, y=0.992)
    finalize_grid_figure(fig, top=0.90, wspace=0.75, hspace=0.50)
    fig.savefig(sys.argv[2], dpi=120, facecolor="white", bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
