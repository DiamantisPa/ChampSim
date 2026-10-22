#!/usr/bin/env python3
"""Figure 3f2 (analysis): fate of UCP installs (panel 3 of fig3f).

Left: u-op cache MPKI, 6K baseline vs 6K+UCP-noind (banks4), no L1I pf.
Middle: percent reduction in u-op-cache misses by class (steady-state vs
post-mispredict recovery), from the loss-dump miss counts x class splits.
Right: fate of UCP's newly allocated u-op-cache windows (completed
lifetimes, from the unused-evicted stat), 100%-stacked: useful (>=1
demand hit before eviction) vs evicted unused.  Installs of already-
resident windows (~90% of install traffic) are excluded.
UCP removes <=8% of even its target (recovery) class, ~0-2% of steady
misses, and >=93% of its new allocations die unused; its IPC gains come
from miss COST, not miss COUNT.
zen5_final; CVP-1 wrap-free W50M+I50M 143-subset (all complete),
qualcomm/google I150M; UCP = noind banks4 rebuild with unused-evicted stat.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (6K MPKI, +UCP MPKI)
MPKI = {
    "Qualcomm": (330.5, 327.0),
    "Google": (232.2, 226.1),
    "CVP-1": (139.4, 137.9),
}
# suite -> (% steady-miss reduction, % post-mispredict-miss reduction)
# from avg per-simpoint miss counts x recovery%:
#   qc 49,569,403@19.8% -> 49,047,108@19.4%; gg 34,828,274@13.9% ->
#   33,912,540@13.3%; cvp 6,969,352@7.4% -> 6,895,812@6.9% (143-subset,
#   Sep 4 rebuild -- all 143 simpoints complete)
REDUCTION = {
    "Qualcomm": (0.6, 3.1),
    "Google": (2.0, 6.8),
    "CVP-1": (0.5, 7.7),
}
# suite -> % of completed new-allocation lifetimes that were demand-hit
# (aggregator "accuracy % of completed window lifetimes", per-simpoint mean):
#   qc useful 101,201 vs unused 2,920,503; gg 133,589 vs 1,941,268;
#   cvp143 31,667 vs 690,952
ACCURACY = {
    "Qualcomm": 3.4,
    "Google": 6.9,
    "CVP-1": 4.7,
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

fig, axu = plt.subplots(figsize=(2.4, 2.3), dpi=300)

# --- third panel: fate of UCP's new allocations -------------------------
# 100%-stacked: evicted-unused (tint+hatch, dominant) at the bottom, useful
# (solid sliver) on top; the sliver's value is printed above the bar
BW = 0.5
for i, (suite, v) in enumerate(ACCURACY.items()):
    hue = HUES[i % len(HUES)]
    unused = 100.0 - v
    axu.bar(i, unused, BW, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
    axu.bar(i, unused, BW, facecolor="none", hatch="///", edgecolor=hue,
            linewidth=0.0, zorder=4)
    axu.bar(i, v, BW, bottom=unused, color=hue, zorder=3)
    axu.annotate(f"{unused:.1f}", (i, unused / 2), ha="center", va="center",
                 fontsize=6.0, color=INK)
    axu.annotate(f"{v:.1f}", (i, 100), xytext=(0, 2),
                 textcoords="offset points", ha="center", va="bottom",
                 fontsize=5.8, color=INK)
axu.set_xticks(range(len(ACCURACY)))
axu.set_xticklabels(list(ACCURACY.keys()), fontsize=6.4, color=INK)
axu.set_ylim(0, 145)
axu.set_yticks([0, 25, 50, 75, 100])
axu.set_title("UCP installs (%)", fontsize=7.8, color=INK, pad=3)
axu.legend(handles=[
    Patch(facecolor=INK_2ND, label="useful (demand hit)"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="evicted unused"),
], fontsize=6.2, frameon=False, loc="upper left", handlelength=1.2,
    handletextpad=0.5, borderaxespad=0.2, labelspacing=0.35)

for ax in (axu,):
    ax.set_xlim(-0.65, len(MPKI) - 1 + 0.65)
    ax.yaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)


fig.tight_layout(pad=0.4, w_pad=1.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig3f2_ucp_installs.{ext}", bbox_inches="tight", pad_inches=0.02)
print("wrote figs/fig3f2_ucp_installs.{pdf,png}")
