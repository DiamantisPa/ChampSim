#!/usr/bin/env python3
"""Figure 1d (evaluation): chain against the u-op cache's potential (no-pf).

Per suite: 6Ki realized (solid), 6Ki+UCP-noind (mid tint, backslash hatch),
cold-miss ideal (light tint, slash hatch), all over the no-u-op-cache baseline.
Chain at the final realistic configuration (~45KB).  zen5_final platform.
CVP-1 uses the wrap-free W50M+I100M runs (traces ~100M instr).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (realized_6ki, ucp_noind, chain, ideal, n)
DATA = {
    "Qualcomm": (0.927, 2.450, 4.643, 16.892, 76),
    "Google": (0.278, 2.229, 1.279, 6.107, 43),
    "CVP-1": (2.955, 3.870, 4.552, 10.448, 143),
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
W = 0.185

for i, suite in enumerate(suites):
    realized, ucp, chain, potential, n = DATA[suite]
    hue = HUES[i % len(HUES)]
    step = W + 0.025
    xr, xu, xc, xp = i - 1.5 * step, i - 0.5 * step, i + 0.5 * step, i + 1.5 * step
    ax.bar(xr, realized, W, color=hue, zorder=3)
    ax.bar(xu, ucp, W, facecolor=hue, alpha=0.55, zorder=3)
    ax.bar(xu, ucp, W, facecolor="none", hatch="\\\\\\", edgecolor=hue,
           linewidth=0.0, zorder=4)
    ax.bar(xc, chain, W, facecolor=hue, alpha=0.80, zorder=3)
    ax.bar(xc, chain, W, facecolor="none", hatch="xx", edgecolor="white",
           linewidth=0.0, zorder=4)
    ax.bar(xp, potential, W, facecolor=hue, alpha=0.30, zorder=3)
    ax.bar(xp, potential, W, facecolor="none", hatch="///", edgecolor=hue,
           linewidth=0.0, zorder=4)
    for x, v in ((xr, realized), (xu, ucp), (xc, chain), (xp, potential)):
        ax.annotate(f"+{v:.2f}%" if v < 10 else f"+{v:.1f}%", (x, v),
                    xytext=(0, 2), textcoords="offset points", ha="center",
                    va="bottom", fontsize=6.6, color=INK)
    ax.annotate(f"{100 * chain / potential:.0f}% of\npotential",
                (xc, max(chain, ucp)), xytext=(0, 13),
                textcoords="offset points", ha="center", va="bottom",
                fontsize=6.4, color=INK_2ND, linespacing=1.1)

ax.set_xlim(-0.55, len(suites) - 1 + 0.55)
ax.set_xticks(range(len(suites)))
ax.set_xticklabels([f"{s}\n({DATA[s][4]} traces)" for s in suites],
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
    Patch(facecolor=INK_2ND, hatch="xx", edgecolor="white", linewidth=0.6,
          label="6K + trPref"),
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
    fig.savefig(f"figs/fig1d_chain.{ext}")
print("wrote figs/fig1d_chain.{pdf,png}")
