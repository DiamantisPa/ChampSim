#!/usr/bin/env python3
"""Figure 4 (evaluation headline): trPref vs UCP speedup, both prefetch regimes.

Two panels, shared y-axis: geomean IPC speedup over the matched 6K-u-op-cache
baseline of each regime -- left without an L1I prefetcher, right under EP-4Ke.
Latest matched-ROI runs (Sep 2026, warmup-train enabled, UCP = noind banks4):
qualcomm/google W50M+I150M, CVP-1 143-subset W50M+I50M (same-ROI pairs).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (+UCP %, +trPref %)
NOPF = {
    "Qualcomm": (1.554, 3.683),
    "Google": (1.961, 1.006),
    "CVP-1": (1.082, 1.355),
}
EP4KE = {
    "Qualcomm": (-0.243, 1.602),
    "Google": (1.253, 0.439),
    "CVP-1": (0.415, 1.004),
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

fig, axes = plt.subplots(1, 2, figsize=(5.6, 2.1), dpi=300, sharey=True)

W = 0.3
STEP = W + 0.05
for ax, data, title in ((axes[0], NOPF, "no L1I prefetcher"),
                        (axes[1], EP4KE, "with EIP")):
    for i, (suite, (ucp, chain)) in enumerate(data.items()):
        hue = HUES[i % len(HUES)]
        x_ucp = i - STEP / 2
        ax.bar(x_ucp, ucp, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
        ax.bar(x_ucp, ucp, W, facecolor="none", hatch="///", edgecolor=hue,
               linewidth=0.0, zorder=4)
        ax.bar(i + STEP / 2, chain, W, color=hue, zorder=3)
        for x, v in ((x_ucp, ucp), (i + STEP / 2, chain)):
            va = "bottom" if v >= 0 else "top"
            dy = 2 if v >= 0 else -2
            ax.annotate(f"{v:.2f}", (x, v), xytext=(0, dy),
                        textcoords="offset points", ha="center", va=va,
                        fontsize=6.0, color=INK)
    ax.set_xlim(-0.6, len(data) - 1 + 0.6)
    ax.set_xticks(range(len(data)))
    ax.set_xticklabels(list(data.keys()), fontsize=7, color=INK)
    ax.set_title(title, fontsize=7.8, color=INK, pad=3)
    ax.axhline(0, color=BASELINE, linewidth=0.6, zorder=2)
    ax.yaxis.set_tick_params(labelsize=7, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)

axes[0].set_ylim(-0.6, 4.2)
axes[0].set_ylabel("Geomean speedup (%)", fontsize=7.5, color=INK)
axes[0].legend(handles=[
    Patch(facecolor=INK_2ND, label="+ trace prefetch"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ UCP"),
], fontsize=6.4, frameon=False, loc="upper right", handlelength=1.2,
    handletextpad=0.5, borderaxespad=0.2, labelspacing=0.35)

fig.tight_layout(pad=0.4, w_pad=0.9)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig4_speedup.{ext}", bbox_inches="tight", pad_inches=0.02)
print("wrote figs/fig4_speedup.{pdf,png}")
