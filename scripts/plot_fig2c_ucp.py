#!/usr/bin/env python3
"""Figure 2c (evaluation): capacity sweep with UCP-noind (4-bank) at 6K u-ops.

Per suite (hue matches Fig. 1): capacity sweep 6-24K u-ops (solid line),
cold-miss-ideal ceiling (dashed), chain at 6K u-ops (star).  All geomean
speedup over the shared no-u-op-cache baseline (zen5_final, no-pf runs).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

CAPS = [6, 8, 12, 16, 24]  # K u-ops

# suite -> dict(sweep=[% at each cap], ideal=%, chain=% @6K, n=traces)
DATA = {
    "Qualcomm": dict(sweep=[0.927, 1.118, 1.472, 1.703, 1.982], ucp=2.450, n=76),
    "Google": dict(sweep=[0.278, 0.318, 0.410, 0.446, 0.505], ucp=2.229, n=43),
    "CVP-1": dict(sweep=[2.955, 3.505, 4.149, 4.415, 4.648], ucp=3.870, n=143),  # wrap-free W50+I100
}

HUES = {"Qualcomm": "#2a78d6", "Google": "#eb6834", "CVP-1": "#1baf7a"}
INK = "#0b0b0b"
INK_2ND = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"

plt.rcParams.update({
    "font.size": 8,
    "axes.linewidth": 0.6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, ax = plt.subplots(figsize=(3.35, 2.5), dpi=300)

for suite, d in DATA.items():
    hue = HUES[suite]
    ax.plot(CAPS, d["sweep"], color=hue, linewidth=1.4, marker="o",
            markersize=3.2, zorder=3)
    ucp_x = {"Qualcomm": 5.75, "Google": 6.0, "CVP-1": 6.25}[suite]
    ax.scatter([ucp_x], [d["ucp"]], marker="D", s=42, color=hue,
               edgecolor="white", linewidth=0.7, zorder=5)
    # identity by placement, ink by text tokens
    ax.annotate(suite, (CAPS[-1], d["sweep"][-1]), xytext=(4, 0),
                textcoords="offset points", ha="left", va="center",
                fontsize=7, color=INK)

ax.set_xscale("log", basex=2)
ax.set_xlim(5.4, 31)
ax.set_xticks(CAPS)
ax.set_xticklabels([f"{c}K" for c in CAPS], fontsize=7)
ax.minorticks_off()
ax.set_xlabel("µ-op cache capacity (µ-ops)", fontsize=8, color=INK)
ax.set_ylabel("Geomean speedup\nover no µ-op cache (%)", fontsize=8, color=INK,
              linespacing=1.2)
ax.set_ylim(0, 5.4)
ax.set_yticks([0, 1, 2, 3, 4, 5])
ax.yaxis.set_tick_params(labelsize=7, colors=MUTED, width=0.6)
ax.xaxis.set_tick_params(labelsize=7, colors=MUTED, width=0.6, length=2)
ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(BASELINE)

from matplotlib.lines import Line2D
ax.legend(handles=[
    Line2D([], [], color=INK_2ND, linewidth=1.4, marker="o", markersize=3.2,
           label="µ-op cache, 6→24K µ-ops"),
    Line2D([], [], color=INK_2ND, marker="D", markersize=6, linestyle="none",
           markeredgecolor="white", markeredgewidth=0.6,
           label="UCP @ 6K µ-ops (9 KB)"),
], fontsize=6.8, frameon=False, loc="upper left", handlelength=1.6,
   handletextpad=0.5, borderaxespad=0.1, labelspacing=0.4)

fig.tight_layout(pad=0.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig2c_ucp.{ext}")
print("wrote figs/fig2c_ucp.{pdf,png}")
