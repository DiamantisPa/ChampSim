#!/usr/bin/env python3
"""Figure 1f (analysis): L1I prefetchers cut demand L1I misses, not u-op misses.

Two panels, four configs per suite (no-pf / EIP / D-JOLT / FNL-MMA):
left = demand L1I MPKI (LOAD row), right = u-op cache MPKI.  zen5_final;
CVP-1 from the matched W50M+I50M set, qualcomm/google from I150M.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> [no-pf, EIP, D-JOLT, FNL-MMA]
L1I = {
    "Qualcomm": [37.47, 25.76, 33.87, 28.26],
    "Google": [24.98, 16.07, 14.04, 15.46],
    "CVP-1": [15.72, 11.09, 10.48, 11.69],
}
MUOP = {
    "Qualcomm": [330.5, 327.9, 329.5, 328.1],
    "Google": [232.2, 230.9, 230.6, 230.2],
    "CVP-1": [139.4, 138.7, 138.6, 138.7],
}

HUES = ["#2a78d6", "#eb6834", "#1baf7a"]
HATCH = [None, "///", "\\\\\\", "xx"]
TINT_ALPHA = 0.45
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

fig, axes = plt.subplots(1, 2, figsize=(5.6, 2.15), dpi=300)

for ax, data, title, ymax, fmt in (
    (axes[0], L1I, "L1I cache MPKI", 43, "{:.1f}"),
    (axes[1], MUOP, "µ-op cache MPKI", 375, "{:.0f}"),
):
    W = 0.185
    step = W + 0.025
    for i, (suite, vals) in enumerate(data.items()):
        hue = HUES[i % len(HUES)]
        for k, v in enumerate(vals):
            x = i + (k - 1.5) * step
            if k == 0:
                ax.bar(x, v, W, color=hue, zorder=3)
            else:
                ax.bar(x, v, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
                ax.bar(x, v, W, facecolor="none", hatch=HATCH[k], edgecolor=hue,
                       linewidth=0.0, zorder=4)
            ax.annotate(fmt.format(v), (x, v), xytext=(0, 2),
                        textcoords="offset points", ha="center", va="bottom",
                        fontsize=5.2, color=INK)
    ax.set_xlim(-0.55, len(data) - 1 + 0.55)
    ax.set_xticks(range(len(data)))
    ax.set_xticklabels(list(data.keys()), fontsize=7, color=INK)
    ax.set_title(title, fontsize=7.8, color=INK, pad=3)
    ax.set_ylim(0, ymax)
    ax.yaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)

legend_handles = [
    Patch(facecolor=INK_2ND, label="no L1I pf."),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ EIP"),
    Patch(facecolor="#b9b7b0", hatch="\\\\\\\\", edgecolor=INK_2ND, linewidth=0.6,
          label="+ D-JOLT"),
    Patch(facecolor="#b9b7b0", hatch="xxxx", edgecolor=INK_2ND, linewidth=0.6,
          label="+ FNL-MMA"),
]
fig.legend(handles=legend_handles, fontsize=6.6, frameon=False, ncol=4,
           loc="lower center", bbox_to_anchor=(0.53, -0.01),
           handlelength=1.2, handletextpad=0.5, columnspacing=1.2)

fig.tight_layout(pad=0.4, w_pad=1.2, rect=(0, 0.08, 1, 1))
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig1f_mpki_layers.{ext}")
print("wrote figs/fig1f_mpki_layers.{pdf,png}")
