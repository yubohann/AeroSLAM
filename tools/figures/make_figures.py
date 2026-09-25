#!/usr/bin/env python3
"""Generate publication-style figures for AeroSLAM.

Figures follow a top-venue aesthetic: serif fonts, Okabe-Ito colour-blind-safe
palette, thin strokes, no chart junk. Both PDF (vector) and PNG (300 dpi) are
written to docs/media/figures/.

    python3 tools/figures/make_figures.py
"""

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
RESULTS = os.path.join(ROOT, "tools", "benchmark", "results")
OUT = os.path.join(ROOT, "docs", "media", "figures")

BLUE = "#0072B2"
ORANGE = "#E69F00"
GREEN = "#009E73"
VERMILLION = "#D55E00"
GRAY = "#666666"

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8.5,
    "axes.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "figure.dpi": 300,
    "savefig.bbox": "tight",
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
    "axes.facecolor": "white",
})


def load(name):
    with open(os.path.join(RESULTS, name)) as handle:
        return json.load(handle)


def box(ax, x, y, w, h, text, edge, face="white", fontsize=8):
    """Kept for the band boxes; node boxes use fitted_text (auto-sized)."""
    patch = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06,rounding_size=0.08",
                           linewidth=0.9, edgecolor=edge, facecolor=face)
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, linespacing=1.35)
    return (x, y, w, h)


def fitted_text(ax, cx, cy, text, edge, fontsize=7.5):
    """Node box: the border is drawn around the text, so it can never overlap."""
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize,
            linespacing=1.45, zorder=3,
            bbox=dict(boxstyle="round,pad=0.38,rounding_size=0.12",
                      linewidth=0.9, edgecolor=edge, facecolor="white"))


def arrow(ax, start, end, style="-|>", color=GRAY):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=9,
                                 linewidth=0.9, color=color,
                                 shrinkA=1.0, shrinkB=1.0))


def figure_architecture():
    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6.1)
    ax.axis("off")

    # Aerial panel: centred stack, borders fitted to the text
    ax.add_patch(FancyBboxPatch((0.15, 1.95), 4.55, 4.35,
                                boxstyle="round,pad=0.08", linewidth=0.7,
                                edgecolor=BLUE, facecolor="#F5F9FC"))
    ax.text(0.35, 5.95, "Aerial stack  (src/uav)", fontsize=8.5, color=BLUE, weight="bold")
    cx = 2.42
    fitted_text(ax, cx, 5.35, "hector_quadrotor\ncontrol · EKF · actions", BLUE)
    fitted_text(ax, cx, 4.30, "livox simulation + bridge\nPointCloud → CustomMsg", BLUE)
    fitted_text(ax, cx, 3.25, "FAST-LIO\nLiDAR-inertial odometry", BLUE)
    fitted_text(ax, cx, 2.20, "EGO-Planner\nB-spline planning", BLUE)
    arrow(ax, (cx, 5.00), (cx, 4.68))
    arrow(ax, (cx, 3.95), (cx, 3.62))
    arrow(ax, (cx, 2.90), (cx, 2.57))
    # control loop: EGO -> hector, drawn outside the stack
    ax.annotate("", xy=(0.75, 5.35), xytext=(0.75, 2.20),
                arrowprops=dict(arrowstyle="-|>", linewidth=0.9, color=GRAY,
                                connectionstyle="arc3,rad=0.45"))
    ax.text(0.34, 3.75, "command/pose", fontsize=6.2, color=GRAY, rotation=90,
            ha="center", va="center")

    # Ground panel
    ax.add_patch(FancyBboxPatch((5.30, 1.95), 4.55, 4.35,
                                boxstyle="round,pad=0.08", linewidth=0.7,
                                edgecolor=GREEN, facecolor="#F4FAF7"))
    ax.text(5.50, 5.95, "Ground stack  (src/ground)", fontsize=8.5, color=GREEN, weight="bold")
    gx = 7.58
    fitted_text(ax, gx, 5.35, "Global planning\nA* · Hybrid A* · Sunshine", GREEN, fontsize=7.2)
    fitted_text(ax, gx, 4.30, "Local planning / control\nHLP+MPC+corridor · iLQR · MPPI", GREEN, fontsize=7.2)
    fitted_text(ax, gx, 3.25, "Trajectory optimization\nRDP · L-BFGS · min-snap", GREEN, fontsize=7.2)
    fitted_text(ax, gx, 2.20, "Costmap layers\nESDF · Voronoi · reachability · social", GREEN, fontsize=7.2)
    arrow(ax, (gx, 5.00), (gx, 4.68))
    arrow(ax, (gx, 3.95), (gx, 3.62))
    arrow(ax, (gx, 2.90), (gx, 2.57))

    # Common / tooling band
    box(ax, 0.15, 0.80, 9.70, 0.62,
        "Common (src/common):  geodesy WGS-84 ⇄ UTM · geographic_msgs · uuid_msgs · teleop",
        GRAY, face="#FAFAFA", fontsize=7.6)
    box(ax, 0.15, 0.10, 9.70, 0.50,
        "Tooling:  unified scripts · Docker · CI · benchmark suite · frontier modules (pinned, patched)",
        GRAY, face="#FAFAFA", fontsize=7.2)

    path = os.path.join(OUT, "fig1_architecture")
    fig.savefig(path + ".png")
    fig.savefig(path + ".pdf")
    plt.close(fig)
    print("wrote", path)


def figure_benchmarks():
    runs = [
        ("ab_no_deskew.json", "No de-skew\n#1", BLUE),
        ("run_01.json", "No de-skew\n#2", BLUE),
        ("run_02.json", "No de-skew\n#3", BLUE),
        ("ab_linear_deskew.json", "Linear\nde-skew", ORANGE),
    ]
    data = [(label, color, load(name)) for name, label, color in runs]

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7),
                             gridspec_kw={"width_ratios": [1.45, 1.0]})

    ax = axes[0]
    positions = range(len(data))
    width = 0.36
    rmse = [d["position_rmse_m"] for _, _, d in data]
    aligned = [d["aligned_rmse_m"] for _, _, d in data]
    colors = [color for _, color, _ in data]
    bars1 = ax.bar([p - width / 2 for p in positions], rmse, width,
                   color=colors, edgecolor="white", linewidth=0.4, label="position RMSE")
    bars2 = ax.bar([p + width / 2 for p in positions], aligned, width,
                   color=colors, alpha=0.45, edgecolor="white", linewidth=0.4,
                   hatch="///", label="aligned RMSE")
    for bars in (bars1, bars2):
        for bar in bars:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.008,
                    "%.3f" % bar.get_height(), ha="center", va="bottom", fontsize=6.2)
    mean_rmse = sum(rmse[:3]) / 3.0
    ax.axhline(mean_rmse, color=GRAY, linewidth=0.7, linestyle="--")
    ax.set_xticks(list(positions))
    ax.set_xticklabels([label for label, _, _ in data], fontsize=7)
    ax.set_ylabel("position error (m)")
    ax.set_ylim(0, 0.48)
    ax.legend(frameon=True, framealpha=0.95, edgecolor="none", fontsize=6.3,
              loc="upper left", bbox_to_anchor=(0.02, 0.98), borderaxespad=0.0,
              handlelength=1.2, labelspacing=0.4)
    ax.set_title("(a) LIO drift vs. ground truth, 40-45 s flights", fontsize=8.5, pad=6)

    ground = load("ground_02.json")
    ax = axes[1]
    labels = ["time to goal\n(s)", "path length\n(m)", "final error\n(m)"]
    values = [ground["time_s"], ground["path_length_m"], ground["final_distance_m"]]
    bars = ax.bar(range(3), values, 0.55, color=GREEN, edgecolor="white", linewidth=0.5)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.25,
                "%.2f" % value, ha="center", va="bottom", fontsize=6.8)
    ax.axhline(0.5, color=VERMILLION, linewidth=0.7, linestyle=":")
    ax.text(2.45, 1.6, "tolerance 0.5 m", ha="right", va="bottom",
            fontsize=6.0, color=VERMILLION)
    ax.set_xticks(range(3))
    ax.set_xticklabels(labels, fontsize=7)
    ax.set_ylim(0, 17)
    ax.set_title("(b) ground navigation, test2 world", fontsize=8.5)

    fig.tight_layout()
    path = os.path.join(OUT, "fig2_benchmarks")
    fig.savefig(path + ".png")
    fig.savefig(path + ".pdf")
    plt.close(fig)
    print("wrote", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    figure_architecture()
    figure_benchmarks()


if __name__ == "__main__":
    main()
