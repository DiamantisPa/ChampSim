#!/usr/bin/env python3
"""Figure 1e (analysis): L1I prefetching does not change u-op-cache hit rate.

Per suite: u-op-cache hit rate on the 6Ki baseline with no prefetcher and
with three L1I prefetchers (EIP-4Ke, D-JOLT, FNL-MMA); max delta +0.26pp
while IPC moves up to +2.8%.  zen5_final; CVP-1 rows from the matched
W50M+I50M set (wrap-free), qualcomm/google from I150M.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (no-pf, EIP, D-JOLT, FNL-MMA)  [%]
DATA = {
    "Qualcomm": (66.95, 67.21, 67.05, 67.19),
    "Google": (76.78, 76.91, 76.94, 76.98),
    "CVP-1": (86.06, 86.13, 86.14, 86.13),
}

HUES = ["#2a78d6", "#eb6834", "#1baf7a"]
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

fig, ax = plt.subplots(figsize=(3.35, 2.1), dpi=300)

W = 0.185
HATCH = [None, "///", "\\\\\\", "xx"]
for i, (suite, vals) in enumerate(DATA.items()):
    hue = HUES[i % len(HUES)]
    step = W + 0.025
    for k, v in enumerate(vals):
        x = i + (k - 1.5) * step
        if k == 0:
            ax.bar(x, v, W, color=hue, zorder=3)
        else:
            ax.bar(x, v, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
            ax.bar(x, v, W, facecolor="none", hatch=HATCH[k], edgecolor=hue,
                   linewidth=0.0, zorder=4)
        ax.annotate(f"{v:.1f}", (x, v), xytext=(0, 2),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=5.6, color=INK, rotation=0)

ax.set_xlim(-0.55, len(DATA) - 1 + 0.55)
ax.set_xticks(range(len(DATA)))
ax.set_xticklabels(list(DATA.keys()), fontsize=7.5, color=INK)
ax.set_ylabel("µ-op cache hit rate (%)", fontsize=8, color=INK)
ax.set_ylim(0, 100)
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
    Patch(facecolor=INK_2ND, label="no L1I pf."),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ EIP"),
    Patch(facecolor="#b9b7b0", hatch="\\\\\\\\", edgecolor=INK_2ND, linewidth=0.6,
          label="+ D-JOLT"),
    Patch(facecolor="#b9b7b0", hatch="xxxx", edgecolor=INK_2ND, linewidth=0.6,
          label="+ FNL-MMA"),
]
ax.legend(handles=legend_handles, fontsize=6.4, frameon=False, ncol=4,
          loc="lower left", bbox_to_anchor=(0.0, 1.01), handlelength=1.2,
          handletextpad=0.5, columnspacing=1.4, borderaxespad=0.0)

fig.tight_layout(pad=0.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig1e_mpki_pf.{ext}")
print("wrote figs/fig1e_mpki_pf.{pdf,png}")
