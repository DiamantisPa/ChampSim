#!/usr/bin/env python3
"""Figure 2b (motivation): capacity sweep under an EP-4Ke L1I prefetcher.

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
    "Qualcomm": dict(sweep=[1.207, 1.396, 1.706, 1.906, 2.145], ideal=12.863,
                     chain=2.827, n=76),
    "Google": dict(sweep=[0.400, 0.421, 0.508, 0.534, 0.569], ideal=5.028,
                   chain=0.852, n=43),
    "CVP-1": dict(sweep=[3.803, 4.400, 4.938, 5.140, 5.305], ideal=7.603,
                  chain=5.056, n=143),
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

fig.tight_layout(pad=0.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig2b_capacity_ep4ke.{ext}")
print("wrote figs/fig2b_capacity_ep4ke.{pdf,png}")
