#!/usr/bin/env python3
"""Figure 3d (evaluation): backend-idle with trPref and UCP added (no L1I pf).

1x3 small multiples (one per suite): backend-idle cycles (% of runtime) for
6K / 24K u-op caches, 6K+UCP-noind (banks4), 6K+trPref, and the cold-miss
ideal, stacked steady-state (solid) vs post-mispredict recovery (tint+hatch).
zen5_final platform; CVP-1 = 143 headroom subset.  6K/UCP/chain columns from
the Sep 2026 matched-ROI dumps (qualcomm/google I150M, CVP W50M+I50M);
CVP 24K/ideal still I150M (understates idle -- refresh when I50M dumps exist).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

CONFIGS = ["6K", "24K", "6K\n+UCP", "6K\n+trPref", "ideal"]

# suite -> hue + [(total_pct, steady_frac), ...] in CONFIGS order
DATA = {
    "Qualcomm": {
        "hue": "#2a78d6",
        "idle": [(6.54, 0.423), (5.41, 0.487), (5.07, 0.362), (3.03, 0.428), (0.75, 0.238)],
    },
    "Google": {
        "hue": "#eb6834",
        "idle": [(5.37, 0.411), (5.00, 0.421), (3.88, 0.335), (4.23, 0.394), (1.99, 0.336)],
    },
    "CVP-1": {
        "hue": "#1baf7a",
        # 6K/UCP/chain wrap-free W50M+I50M; 24K/ideal still I150M (TODO)
        "idle": [(3.87, 0.326), (2.12, 0.404), (2.63, 0.257), (2.93, 0.316), (0.73, 0.299)],
    },
}
YMAX = 7.4

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

fig, axes = plt.subplots(1, 3, figsize=(5.6, 1.95), dpi=300, sharey=True)

for col, (suite, d) in enumerate(DATA.items()):
    hue = d["hue"]
    ax = axes[col]
    if d["idle"] is None:
        ax.annotate("data\npending", (0.5, 0.45), xycoords="axes fraction",
                    ha="center", va="center", fontsize=7.5, color=MUTED,
                    linespacing=1.3)
        bars = []
    else:
        bars = d["idle"]
    for i, entry in enumerate(bars):
        if entry is None:
            ax.annotate("pend.", (i, 0.35), ha="center", va="bottom",
                        fontsize=6.3, color=MUTED)
            continue
        total, steady_frac = entry
        steady = total * steady_frac
        rec = total - steady
        ax.bar(i, steady, 0.62, color=hue, zorder=3)
        ax.bar(i, rec, 0.62, bottom=steady, facecolor=hue,
               alpha=TINT_ALPHA, zorder=3)
        ax.bar(i, rec, 0.62, bottom=steady, facecolor="none", hatch="///",
               edgecolor=hue, linewidth=0.0, zorder=4)
        ax.annotate(f"{total:.1f}", (i, total), xytext=(0, 1.5),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=6.5, color=INK)
    ax.set_ylim(0, YMAX)
    ax.set_xlim(-0.6, len(CONFIGS) - 0.4)
    ax.set_xticks(range(len(CONFIGS)))
    ax.set_xticklabels(CONFIGS, fontsize=6.2, color=INK, linespacing=1.05)
    ax.yaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)
    if col == 0:
        ax.set_ylabel("Backend idle\n(% runtime)", fontsize=7.5, color=INK,
                      linespacing=1.2)
    ax.set_title(suite, fontsize=7.5, color=INK, pad=3)

legend_handles = [
    Patch(facecolor=INK_2ND, label="steady-state"),
    Patch(facecolor="#f0efec", hatch="////", edgecolor=MUTED, linewidth=0.6,
          label="post-mispredict"),
]
fig.legend(handles=legend_handles, fontsize=6.8, frameon=False, ncol=2,
           loc="lower center", bbox_to_anchor=(0.55, -0.01),
           handlelength=1.3, handletextpad=0.5, columnspacing=1.2)

fig.tight_layout(pad=0.4, w_pad=0.7, rect=(0, 0.09, 1, 1))
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig3d_chain_loss.{ext}")
print("wrote figs/fig3d_chain_loss.{pdf,png}")
