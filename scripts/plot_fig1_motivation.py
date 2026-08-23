#!/usr/bin/env python3
"""Figure 1 (motivation): the u-op cache realizes a small fraction of its own
supply-side potential on datacenter-class traces.

Per suite (one hue per suite): realized 6Ki-DIB bar (solid, left) and
cold-miss-ideal bar (light tint + hatch, right), both over the shared
no-u-op-cache baseline (zen5_final platform, no-pf runs, geomean).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (realized_6ki_pct, potential_coldmiss_pct or None, n_traces)
DATA = {
    "Qualcomm": (0.927, 16.892, 76),
    "Google": (0.278, 6.107, 43),
    "CVP-1": (2.994, 10.216, 143),
}

# reference palette: categorical slots 1-3 (validated all-pairs); tint via alpha
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

fig, ax = plt.subplots(figsize=(3.35, 2.3), dpi=300)

suites = list(DATA.keys())
W = 0.34  # bar width; 0.04 gap between the pair ~ the 2px spacer rule

for i, suite in enumerate(suites):
    realized, potential, n = DATA[suite]
    hue = HUES[i % len(HUES)]
    xr, xp = i - W / 2 - 0.02, i + W / 2 + 0.02
    ax.bar(xr, realized, W, color=hue, zorder=3)
    ax.annotate(f"+{realized:.2f}%", (xr, realized), xytext=(0, 2),
                textcoords="offset points", ha="center", va="bottom",
                fontsize=7.5, color=INK)
    if potential is not None:
        ax.bar(xp, potential, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
        ax.bar(xp, potential, W, facecolor="none", hatch="///",
               edgecolor=hue, linewidth=0.0, zorder=4)
        ax.annotate(f"+{potential:.1f}%", (xp, potential), xytext=(0, 2),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=7.5, color=INK)
        ax.annotate(f"{100 * realized / potential:.1f}% of\npotential",
                    (xr, realized), xytext=(0, 13),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=6.8, color=INK_2ND, linespacing=1.1)
    else:
        ax.annotate("ideal:\npending", (xp, 0), xytext=(0, 4),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=6.8, color=MUTED, linespacing=1.1, zorder=5)

ax.set_xlim(-0.55, len(suites) - 1 + 0.55)
ax.set_xticks(range(len(suites)))
ax.set_xticklabels([f"{s}\n({DATA[s][2]} traces)" for s in suites],
                   fontsize=7.5, color=INK, linespacing=1.2)
ax.set_ylabel("Geomean speedup\nover no µ-op cache (%)", fontsize=8, color=INK, linespacing=1.2)
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

# neutral-gray proxies: hue belongs to the suite, fill style to the config
legend_handles = [
    Patch(facecolor=INK_2ND, label="6K µ-ops cache"),
    Patch(facecolor="#f0efec", hatch="////", edgecolor=MUTED, linewidth=0.6,
          label="ideal µ-op cache"),
]
ax.legend(handles=legend_handles, fontsize=7, frameon=False, loc="upper right",
          handlelength=1.2, handletextpad=0.5, borderaxespad=0.1)

fig.tight_layout(pad=0.4)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig1_motivation.{ext}")
print("wrote figs/fig1_motivation.{pdf,png}")
