#!/usr/bin/env python3
"""Tier4 Calibration result statistics vs single-shot calibration."""

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from plot_common import apply_tier4_style, finalize_grid_figure, load_json  # noqa: E402


def plot_bar_panel(ax, rows: list, title: str, rotate_labels: bool) -> None:
    ax.set_title(title)
    if not rows:
        ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes)
        return
    rows = sorted(rows, key=lambda r: float(r.get("calibrated_rms", 0.0)))
    indices = [int(r.get("index", i)) for i, r in enumerate(rows)]
    cal = [float(r["calibrated_rms"]) for r in rows]
    single = [float(r["singleshot_rms"]) for r in rows]
    x = np.arange(len(rows))
    w = 0.38
    ax.bar(x - w / 2, cal, width=w, label="Calibrated intrinsics", color="#4a90d9")
    ax.bar(
        x + w / 2,
        single,
        width=w,
        label="Single-shot intrinsics (lower bound)",
        color="#f5a623",
    )
    ax.set_xticks(x)
    ax.set_xticklabels(
        [str(i) for i in indices],
        rotation=90 if rotate_labels else 0,
        fontsize=7,
    )
    ax.set_ylabel("RMS (px)")
    ax.legend(loc="upper left", fontsize=7)
    ax.grid(axis="y", alpha=0.3, color="#cccccc")


def main() -> int:
    if len(sys.argv) < 3:
        print(
            "usage: intrinsics_calibration_bars_plot.py <input.json> <output.png>",
            file=sys.stderr,
        )
        return 2
    apply_tier4_style()
    data = load_json(sys.argv[1])
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), facecolor="white")
    plot_bar_panel(axes[0], data.get("training", []), "Training", False)
    plot_bar_panel(axes[1], data.get("inliers", []), "Inliers", False)
    plot_bar_panel(axes[2], data.get("evaluation", []), "Evaluation", True)
    fig.suptitle(
        "Calibration result statistics vs single-shot calibration", fontsize=12, y=0.985
    )
    finalize_grid_figure(fig, top=0.90, wspace=0.2, hspace=0.45)
    fig.savefig(sys.argv[2], dpi=120, facecolor="white", bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
