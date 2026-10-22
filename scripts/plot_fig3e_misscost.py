#!/usr/bin/env python3
"""Figure 3e (analysis): post-mispredict u-op-cache misses are few but expensive.

Per suite, two 100%-stacked columns from the 6Ki no-pf baseline: the
steady/post-mispredict split of u-op-cache MISS COUNTS vs the split of
BACKEND-IDLE CYCLES.  The "xN" annotation is the implied per-miss idle cost
of a post-mispredict miss relative to a steady one.  zen5_final platform.
CVP-1 splits from the wrap-free W50M+I50M dumps.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (recovery% of misses, recovery% of idle cycles)
DATA = {
    "Qualcomm": (19.8, 57.7),
    "Google": (13.9, 58.9),
    "CVP-1": (7.4, 67.4),
}

HUES = ["#2a78d6", "#eb6834", "#1baf7a"]
TINT_ALPHA = 0.30
INK = "#0b0b0b"
INK_2ND = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"

plt.rcParams.update({
    "font.size": 8,
    "axes.linewidth": 0.6,
    "hatch.linewidth": 0.6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, ax = plt.subplots(figsize=(3.35, 2.25), dpi=300)

W = 0.32
positions, labels = [], []
for i, (suite, (rec_miss, rec_idle)) in enumerate(DATA.items()):
    hue = HUES[i % len(HUES)]
    for k, rec in enumerate((rec_miss, rec_idle)):
        x = i - W / 2 - 0.02 + k * (W + 0.04)
        steady = 100.0 - rec
        ax.bar(x, steady, W, color=hue, zorder=3)
        ax.bar(x, rec, W, bottom=steady, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
        ax.bar(x, rec, W, bottom=steady, facecolor="none", hatch="///",
               edgecolor=hue, linewidth=0.0, zorder=4)
        if rec >= 12:
            ax.annotate(f"{rec:.0f}%", (x, steady + rec / 2), ha="center",
                        va="center", fontsize=6.8, color=INK)
        else:
            ax.annotate(f"{rec:.0f}%", (x - W / 2 - 0.03, 100 - rec / 2),
                        ha="right", va="center", fontsize=6.4, color=INK_2ND)
        positions.append(x)
        labels.append("uop\nmisses" if k == 0 else "backend\nidle")
    ax.annotate(suite, (i, -34), ha="center", va="top", fontsize=7.5,
                color=INK, annotation_clip=False)

ax.set_xlim(-0.65, len(DATA) - 1 + 0.65)
ax.set_xticks(positions)
ax.set_xticklabels(labels, fontsize=6.2, color=INK_2ND, linespacing=1.05)
ax.set_ylim(0, 105)
ax.set_yticks([0, 25, 50, 75, 100])
ax.yaxis.set_tick_params(labelsize=7, colors=MUTED, width=0.6)
ax.xaxis.set_tick_params(length=0)
ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(BASELINE)

legend_handles = [
    Patch(facecolor=INK_2ND, label="steady-state"),
    Patch(facecolor="#f0efec", hatch="////", edgecolor=MUTED, linewidth=0.6,
          label="post-mispredict"),
]
fig.legend(handles=legend_handles, fontsize=6.8, frameon=False, ncol=2,
           loc="lower center", bbox_to_anchor=(0.56, -0.006),
           handlelength=1.3, handletextpad=0.5, columnspacing=1.2)

fig.tight_layout(pad=0.4, rect=(0, 0.09, 1, 1))
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig3e_misscost.{ext}")
print("wrote figs/fig3e_misscost.{pdf,png}")
