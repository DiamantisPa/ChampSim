#!/usr/bin/env python3
"""Figure 3g (analysis): trPref repairs the u-op supply; UCP discounts it.

Left: u-op cache MPKI, 6K baseline vs +UCP-noind (banks4) vs +trPref.
Middle: percent reduction in u-op-cache MISSES by class (steady vs
post-mispredict), per mechanism, from loss-dump miss counts x class splits:
  base qc 49,569,403@19.8%rec, gg 34,828,274@13.9%, cvp 6,969,352@7.4%
  UCP  qc 49,047,108@19.4%,    gg 33,912,540@13.3%, cvp 6,895,812@6.9%
  chn  qc 34,006,675@11.9%,    gg 28,571,779@10.6%, cvp 5,156,109@5.8%
Right: 100%-stacked fate of completed install lifetimes (useful = >=1 demand
hit before eviction) -- UCP = newly allocated u-op-cache windows; trPref =
staged buffer windows (its actual u-op-cache insertions are 100% used by
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
# same shape, dispatch-starve (shallow stall) CYCLE reduction by class (from
# starve counts x splits: base qc 28,319,696@68.9%st, gg 35,937,656@71.0%,
# cvp 2,179,631@62.2%; UCP qc 26,538,950@69.5%, gg 32,487,191@71.7%,
# cvp 1,886,765@63.5%; chn qc 17,581,458@77.3%, gg 28,260,990@73.0%,
# cvp 1,526,915@61.6%)
STARVE_REDUCTION = {
    "Qualcomm": ((5.5, 8.1), (30.3, 54.7)),
    "Google": ((8.7, 11.8), (19.1, 26.8)),
    "CVP-1": ((11.6, 16.4), (30.6, 28.8)),
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

plt.rcParams.update({
    "font.size": 8,
    "axes.linewidth": 0.6,
    "hatch.linewidth": 0.6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, (axl, axr, axm, axu) = plt.subplots(
    1, 4, figsize=(9.8, 2.4), dpi=300,
    gridspec_kw={"width_ratios": [1.05, 1.2, 1.2, 0.9]})

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
axl.set_title("µ-op cache MPKI", fontsize=7.8, color=INK, pad=3)
axl.legend(handles=[
    Patch(facecolor=INK_2ND, label="6K µ-ops cache"),
    Patch(facecolor="#b9b7b0", hatch="\\\\\\\\", edgecolor=INK_2ND,
          linewidth=0.6, label="+ UCP"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ trace prefetch"),
], fontsize=5.9, frameon=False, loc="upper right", handlelength=1.2,
    handletextpad=0.45, borderaxespad=0.2, labelspacing=0.3)

# --- class-reduction panels (misses, then backend-idle cycles) ----------
def class_reduction_panel(ax, data, title, with_legend):
    BW = 0.155
    PAIR_OFF = 0.21  # pair centers at suite x -/+ PAIR_OFF
    positions, labels = [], []
    for i, (suite, (ucp_pair, chn_pair)) in enumerate(data.items()):
        hue = HUES[i % len(HUES)]
        for m, pair in enumerate((ucp_pair, chn_pair)):
            cx = i + (m - 0.5) * 2 * PAIR_OFF
            for k, v in enumerate(pair):
                x = cx + (k - 0.5) * (BW + 0.01)
                if k == 0:
                    ax.bar(x, v, BW, color=hue, zorder=3)
                else:
                    ax.bar(x, v, BW, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
                    ax.bar(x, v, BW, facecolor="none", hatch="///",
                           edgecolor=hue, linewidth=0.0, zorder=4)
                ax.annotate(f"{v:.1f}", (x, v), xytext=(0, 1.5),
                            textcoords="offset points", ha="center",
                            va="bottom", fontsize=5.4, color=INK)
            positions.append(cx)
            labels.append("UCP" if m == 0 else "trPref")
        ax.annotate(suite, (i, -9.5), ha="center", va="top", fontsize=7,
                    color=INK, annotation_clip=False)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=5.8, color=INK_2ND)
    ax.set_ylim(0, 68)
    ax.set_title(title, fontsize=7.8, color=INK, pad=3)
    if with_legend:
        ax.legend(handles=[
            Patch(facecolor=INK_2ND, label="steady-state"),
            Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND,
                  linewidth=0.6, label="post-mispredict"),
        ], fontsize=5.9, frameon=False, loc="upper right", handlelength=1.2,
            handletextpad=0.45, borderaxespad=0.2, labelspacing=0.3)


class_reduction_panel(axr, REDUCTION, "µ-op miss reduction (%)", True)
class_reduction_panel(axm, STARVE_REDUCTION, "dispatch-starve reduction (%)", True)

# --- right panel: 100%-stacked install breakdown, UCP vs trPref ---------
# useful (solid) at bottom, evicted-unused (tint+hatch) on top; a useful
# share too thin to label inside gets its value above the bar
BW3 = 0.32
positions, labels = [], []
for i, (suite, (ucp, chn)) in enumerate(USEFUL.items()):
    hue = HUES[i % len(HUES)]
    for m, v in enumerate((ucp, chn)):
        x = i + (m - 0.5) * 0.42
        unused = 100.0 - v
        # evicted-unused at the bottom, useful stacked on top
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
        labels.append("UCP" if m == 0 else "trPref")
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

for ax in (axl, axr, axm, axu):
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
    fig.savefig(f"figs/fig3g_chain_misses.{ext}", bbox_inches="tight",
                pad_inches=0.02)
print("wrote figs/fig3g_chain_misses.{pdf,png}")
