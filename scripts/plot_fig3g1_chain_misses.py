#!/usr/bin/env python3
"""Figure 3g1 (analysis): fig3g without the dispatch-starve panel.

Left: uop cache MPKI, 6K baseline vs +UCP-noind (banks4) vs +trPref.
Middle: percent reduction in uop-cache MISSES by class (steady vs
post-mispredict), per mechanism, from loss-dump miss counts x class splits:
  base qc 49,569,403@19.8%rec, gg 34,828,274@13.9%, cvp 6,969,352@7.4%
  UCP  qc 49,047,108@19.4%,    gg 33,912,540@13.3%, cvp 6,895,812@6.9%
  chn  qc 34,006,675@11.9%,    gg 28,571,779@10.6%, cvp 5,156,109@5.8%
Right: 100%-stacked fate of completed install lifetimes (useful = >=1 demand
hit before eviction) -- UCP = newly allocated uop-cache windows; trPref =
staged buffer windows (its actual uop-cache insertions are 100% used by
construction, promote-on-hit).
zen5_final; CVP-1 wrap-free W50M+I50M 143-subset, qualcomm/google I150M;
warmup-train enabled runs (Sep 2026).
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# suite -> (6K MPKI, +UCP MPKI, +trPref MPKI)
MPKI = {
    "Qualcomm": (330.5, 327.0, 226.7),
    "Google": (232.2, 226.1, 190.5),
    "CVP-1": (139.4, 137.9, 103.1),
}
# suite -> ((UCP steady red%, UCP rec red%), (trPref steady red%, trPref rec red%))
REDUCTION = {
    "Qualcomm": ((0.6, 3.1), (24.6, 58.8)),
    "Google": ((2.0, 6.8), (14.8, 37.4)),
    "CVP-1": ((0.5, 7.7), (24.7, 42.0)),
}
# suite -> (UCP useful %, trPref useful %)  [completed install lifetimes]
USEFUL = {
    "Qualcomm": (3.4, 58.0),
    "Google": (6.9, 56.9),
    "CVP-1": (4.7, 77.3),
}

HUES = ["#2a78d6", "#eb6834", "#1baf7a"]
TINT_ALPHA = 0.45
INK = "#0b0b0b"
INK_2ND = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
SUCP_COLOR = "#f39c34"  # light orange: highlights our mechanism's labels

plt.rcParams.update({
    "font.size": 8,
    "axes.linewidth": 0.6,
    "hatch.linewidth": 0.6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, (axl, axr, axu) = plt.subplots(
    1, 3, figsize=(7.6, 2.4), dpi=300,
    gridspec_kw={"width_ratios": [1.05, 1.2, 0.9]})

# --- left panel: MPKI, 6K vs +UCP vs +trPref ----------------------------
W = 0.22
STEP = W + 0.03
HATCH = [None, "\\\\\\", "///"]
for i, (suite, vals) in enumerate(MPKI.items()):
    hue = HUES[i % len(HUES)]
    for k, v in enumerate(vals):
        x = i + (k - 1) * STEP
        if k == 0:
            axl.bar(x, v, W, color=hue, zorder=3)
        else:
            axl.bar(x, v, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
            axl.bar(x, v, W, facecolor="none", hatch=HATCH[k], edgecolor=hue,
                    linewidth=0.0, zorder=4)
        axl.annotate(f"{v:.0f}", (x, v), xytext=(0, 2),
                     textcoords="offset points", ha="center", va="bottom",
                     fontsize=5.6, color=INK)
axl.set_xticks(range(len(MPKI)))
axl.set_xticklabels(list(MPKI.keys()), fontsize=7, color=INK)
axl.set_ylim(0, 385)
axl.set_title("uop cache MPKI", fontsize=7.8, color=INK, pad=3)
leg = axl.legend(handles=[
    Patch(facecolor=INK_2ND, label="6K uops cache"),
    Patch(facecolor="#b9b7b0", hatch="\\\\\\\\", edgecolor=INK_2ND,
          linewidth=0.6, label="+ UCP"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ SUCP"),
], fontsize=5.9, frameon=False, loc="upper right", handlelength=1.2,
    handletextpad=0.45, borderaxespad=0.2, labelspacing=0.3)

# --- middle panel: miss reduction by class and mechanism ----------------
BW = 0.155
PAIR_OFF = 0.21  # pair centers at suite x -/+ PAIR_OFF
positions, labels = [], []
for i, (suite, (ucp_pair, chn_pair)) in enumerate(REDUCTION.items()):
    hue = HUES[i % len(HUES)]
    for m, pair in enumerate((ucp_pair, chn_pair)):
        cx = i + (m - 0.5) * 2 * PAIR_OFF
        for k, v in enumerate(pair):
            x = cx + (k - 0.5) * (BW + 0.01)
            if k == 0:
                axr.bar(x, v, BW, color=hue, zorder=3)
            else:
                axr.bar(x, v, BW, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
                axr.bar(x, v, BW, facecolor="none", hatch="///",
                        edgecolor=hue, linewidth=0.0, zorder=4)
            axr.annotate(f"{v:.1f}", (x, v), xytext=(0, 1.5),
                         textcoords="offset points", ha="center",
                         va="bottom", fontsize=5.4, color=INK)
        positions.append(cx)
        labels.append("UCP" if m == 0 else "SUCP")
    axr.annotate(suite, (i, -9.5), ha="center", va="top", fontsize=7,
                 color=INK, annotation_clip=False)
axr.set_xticks(positions)
axr.set_xticklabels(labels, fontsize=5.8, color=INK_2ND)
axr.set_ylim(0, 68)
for t in leg.get_texts():
    if "SUCP" in t.get_text():
        t.set_fontproperties(t.get_fontproperties().copy())  # legend texts share one FontProperties
        t.set_color(SUCP_COLOR)
        t.set_fontweight("bold")

axr.set_title("uop miss reduction (%)", fontsize=7.8, color=INK, pad=3)
axr.legend(handles=[
    Patch(facecolor=INK_2ND, label="steady-state"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND,
          linewidth=0.6, label="post-mispredict"),
], fontsize=5.9, frameon=False, loc="upper right", handlelength=1.2,
    handletextpad=0.45, borderaxespad=0.2, labelspacing=0.3)

# --- right panel: 100%-stacked install breakdown, UCP vs trPref ---------
# evicted-unused (tint+hatch) at the bottom, useful (solid) on top; a useful
# share too thin to label inside gets its value above the bar
BW3 = 0.32
positions, labels = [], []
for i, (suite, (ucp, chn)) in enumerate(USEFUL.items()):
    hue = HUES[i % len(HUES)]
    for m, v in enumerate((ucp, chn)):
        x = i + (m - 0.5) * 0.42
        unused = 100.0 - v
        axu.bar(x, unused, BW3, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
        axu.bar(x, unused, BW3, facecolor="none", hatch="///",
                edgecolor=hue, linewidth=0.0, zorder=4)
        axu.bar(x, v, BW3, bottom=unused, color=hue, zorder=3)
        if v >= 15:
            axu.annotate(f"{v:.1f}", (x, unused + v / 2), ha="center",
                         va="center", fontsize=5.8, color="white")
        else:
            axu.annotate(f"{v:.1f}", (x, 100), xytext=(0, 2),
                         textcoords="offset points", ha="center", va="bottom",
                         fontsize=5.6, color=INK)
        axu.annotate(f"{unused:.1f}", (x, unused / 2), ha="center",
                     va="center", fontsize=5.6, color=INK)
        positions.append(x)
        labels.append("UCP" if m == 0 else "SUCP")
    axu.annotate(suite, (i, -16), ha="center", va="top", fontsize=7,
                 color=INK, annotation_clip=False)
axu.set_xticks(positions)
axu.set_xticklabels(labels, fontsize=5.8, color=INK_2ND)
axu.set_ylim(0, 135)
axu.set_yticks([0, 25, 50, 75, 100])
axu.set_title("install breakdown (%)", fontsize=7.8, color=INK, pad=3)
axu.legend(handles=[
    Patch(facecolor=INK_2ND, label="useful (demand hit)"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="evicted unused"),
], fontsize=5.9, frameon=False, loc="upper left", handlelength=1.2,
    handletextpad=0.45, borderaxespad=0.2, labelspacing=0.3)

for ax in (axr, axu):
    for t in ax.get_xticklabels():
        if t.get_text() == "SUCP":
            t.set_color(SUCP_COLOR)
            t.set_fontweight("bold")

for ax in (axl, axr, axu):
    ax.set_xlim(-0.65, len(MPKI) - 1 + 0.65)
    ax.yaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)

fig.tight_layout(pad=0.4, w_pad=1.2)
import os

os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig3g1_chain_misses.{ext}", bbox_inches="tight",
                pad_inches=0.02)
print("wrote figs/fig3g1_chain_misses.{pdf,png}")
