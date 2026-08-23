#!/usr/bin/env python3
"""Figure 1c (evaluation): UCP against the u-op cache's potential (no-pf).

Per suite: 6Ki realized (solid), 6Ki+UCP-noind (mid tint, backslash hatch),
cold-miss ideal (light tint, slash hatch), all over the no-u-op-cache baseline.
UCP rows are the 4-bank realistic-banking builds.  zen5_final platform.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (realized_6ki, ucp_noind, ideal, n)
DATA = {
    "Qualcomm": (0.927, 2.450, 16.892, 76),
    "Google": (0.278, 2.229, 6.107, 43),
    "CVP-1": (2.994, 3.756, 10.216, 143),
}

HUES = ["#2a78d6", "#eb6834", "#1baf7a"]
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

fig, ax = plt.subplots(figsize=(3.35, 2.3), dpi=300)

suites = list(DATA.keys())
W = 0.24

for i, suite in enumerate(suites):
    realized, ucp, potential, n = DATA[suite]
    hue = HUES[i % len(HUES)]
    xr, xu, xp = i - W - 0.03, i, i + W + 0.03
    ax.bar(xr, realized, W, color=hue, zorder=3)
    ax.bar(xu, ucp, W, facecolor=hue, alpha=0.55, zorder=3)
    ax.bar(xu, ucp, W, facecolor="none", hatch="\\\\\\", edgecolor=hue,
           linewidth=0.0, zorder=4)
    ax.bar(xp, potential, W, facecolor=hue, alpha=0.30, zorder=3)
    ax.bar(xp, potential, W, facecolor="none", hatch="///", edgecolor=hue,
           linewidth=0.0, zorder=4)
    for x, v in ((xr, realized), (xu, ucp), (xp, potential)):
        ax.annotate(f"+{v:.2f}%" if v < 10 else f"+{v:.1f}%", (x, v),
                    xytext=(0, 2), textcoords="offset points", ha="center",
                    va="bottom", fontsize=6.6, color=INK)
    ax.annotate(f"{100 * ucp / potential:.0f}% of\npotential", (xu, ucp),
                xytext=(0, 12), textcoords="offset points", ha="center",
                va="bottom", fontsize=6.4, color=INK_2ND, linespacing=1.1)

ax.set_xlim(-0.55, len(suites) - 1 + 0.55)
ax.set_xticks(range(len(suites)))
ax.set_xticklabels([f"{s}\n({DATA[s][3]} traces)" for s in suites],
                   fontsize=7.5, color=INK, linespacing=1.2)
ax.set_ylabel("Geomean speedup\nover no µ-op cache (%)", fontsize=8, color=INK,
              linespacing=1.2)
ax.set_ylim(0, 19)
ax.set_yticks([0, 5, 10, 15])
ax.yaxis.set_tick_params(labelsize=7, colors=MUTED, width=0.6)
ax.xaxis.set_tick_params(length=0)
ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(BASELINE)

legend_handles = [
    Patch(facecolor=INK_2ND, label="6K µ-ops cache"),
    Patch(facecolor="#b9b7b0", hatch="\\\\\\\\", edgecolor=INK_2ND, linewidth=0.6,
          label="6K + UCP"),
    Patch(facecolor="#f0efec", hatch="////", edgecolor=MUTED, linewidth=0.6,
          label="ideal µ-op cache"),
]
ax.legend(handles=legend_handles, fontsize=6.6, frameon=False, loc="upper right",
          handlelength=1.2, handletextpad=0.5, borderaxespad=0.1,
          labelspacing=0.35)

fig.tight_layout(pad=0.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig1c_ucp.{ext}")
print("wrote figs/fig1c_ucp.{pdf,png}")
