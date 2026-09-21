#!/usr/bin/env python3
"""Figure 5 (evaluation): geomean speedup + per-simpoint distribution.

Left: geomean IPC speedup over the matched 6K baseline per suite (computed
from the same per-simpoint ratios as the right panel).
Right: one dot per simpoint, sorted ascending per suite/mechanism;
x = simpoint percentile (unlabeled) so suites of different sizes share one
axis; trPref = filled dots, UCP = x marks.

Data: the per-workload scatter output of aggregate_results.py.  For each
suite and mechanism, generate one file with e.g.
    python3 aggregate_results.py <mech_dir> --baseline <6ki_dir> \
        [--include subset.txt] --scatter > <data_dir>/qualcomm_trpref.txt
and place them in the data dir (default results/scatter/, or pass a dir as
argv[1]) named {qualcomm,google,cvp}_{trpref,ucp}.txt.  Only the
"  <name>  +x.xx%  ipc ..." scatter lines are parsed; missing files are
skipped with a warning so partial data still renders.
No-L1I-prefetcher regime; zen5_final; CVP-1 = 143 headroom subset W50M+I50M,
qualcomm/google W50M+I150M (Sep 2026 matched-ROI runs, warmup-train on).
"""
import math
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

DATA_DIR = sys.argv[1] if len(sys.argv) > 1 else "results/scatter"

SUITES = [("qualcomm", "Qualcomm"), ("google", "Google"), ("cvp", "CVP-1")]
MECHS = [("ucp", "+ UCP"), ("trpref", "+ trace prefetch")]

RE_ROW = re.compile(r'^\s+\S+\s+([+-]?\d+(?:\.\d+)?)%\s+ipc\s', re.M)


def load(suite, mech):
    path = os.path.join(DATA_DIR, f"{suite}_{mech}.txt")
    if not os.path.exists(path):
        print(f"[warn] missing {path} -- skipping")
        return None
    with open(path, errors="ignore") as f:
        vals = [float(x) for x in RE_ROW.findall(f.read())]
    if not vals:
        print(f"[warn] no scatter rows parsed in {path} -- skipping")
        return None
    return sorted(vals)


def geomean_pct(vals):
    return 100.0 * (math.exp(sum(math.log1p(v / 100.0) for v in vals) / len(vals)) - 1.0)


DATA = {label: {m: load(key, m) for m, _ in MECHS} for key, label in SUITES}

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

fig, (axg, axs) = plt.subplots(1, 2, figsize=(7.0, 2.3), dpi=300,
                               gridspec_kw={"width_ratios": [0.75, 1.5]})

# --- left panel: geomean bars -------------------------------------------
W = 0.3
STEP = W + 0.05
for i, (suite, hue) in enumerate(zip(DATA, HUES)):
    for m, (mech, _) in enumerate(MECHS):
        vals = DATA[suite][mech]
        if vals is None:
            continue
        v = geomean_pct(vals)
        x = i + (m - 0.5) * STEP
        if mech == "ucp":
            axg.bar(x, v, W, facecolor=hue, alpha=TINT_ALPHA, zorder=3)
            axg.bar(x, v, W, facecolor="none", hatch="///", edgecolor=hue,
                    linewidth=0.0, zorder=4)
        else:
            axg.bar(x, v, W, color=hue, zorder=3)
        va = "bottom" if v >= 0 else "top"
        axg.annotate(f"{v:.2f}", (x, v), xytext=(0, 2 if v >= 0 else -2),
                     textcoords="offset points", ha="center", va=va,
                     fontsize=6.0, color=INK)
axg.set_xlim(-0.6, len(DATA) - 1 + 0.6)
axg.set_xticks(range(len(DATA)))
axg.set_xticklabels(list(DATA.keys()), fontsize=7, color=INK)
axg.set_ylabel("Geomean speedup (%)", fontsize=7.5, color=INK)
axg.axhline(0, color=BASELINE, linewidth=0.6, zorder=2)
axg.legend(handles=[
    Patch(facecolor=INK_2ND, label="+ trace prefetch"),
    Patch(facecolor="#b9b7b0", hatch="////", edgecolor=INK_2ND, linewidth=0.6,
          label="+ UCP"),
], fontsize=6.0, frameon=False, loc="upper right", handlelength=1.2,
    handletextpad=0.5, borderaxespad=0.2, labelspacing=0.35)

# --- right panel: sorted per-simpoint speedup (one dot per simpoint) ----
for i, (suite, hue) in enumerate(zip(DATA, HUES)):
    for mech in ("ucp", "trpref"):
        vals = DATA[suite][mech]
        if vals is None:
            continue
        n = len(vals)
        xs = [100.0 * (k + 0.5) / n for k in range(n)]
        if mech == "trpref":
            axs.plot(xs, vals, linestyle="none", marker="o", markersize=3.2,
                     markerfacecolor=hue, markeredgewidth=0.0, zorder=4)
        else:
            axs.plot(xs, vals, linestyle="none", marker="x", markersize=2.9,
                     markeredgecolor=hue, markeredgewidth=0.7, alpha=0.85,
                     zorder=3)
axs.axhline(0, color=BASELINE, linewidth=0.6, zorder=2)
axs.set_xlim(0, 100)
axs.set_xticks([])
axs.set_xlabel("Simpoints", fontsize=7, color=INK)
axs.set_ylabel("Speedup (%)", fontsize=7.5, color=INK)
suite_handles = [Line2D([], [], color=h, marker="s", linestyle="none",
                        markersize=4, label=s) for s, h in zip(DATA, HUES)]
style_handles = [
    Line2D([], [], color=INK_2ND, marker="o", linestyle="none", markersize=4,
           label="trace prefetch"),
    Line2D([], [], markeredgecolor=INK_2ND, marker="x", linestyle="none",
           markersize=4, markeredgewidth=0.9, label="UCP"),
]
axs.legend(handles=suite_handles + style_handles, fontsize=6.0, frameon=False,
           loc="upper left", ncol=2, handlelength=1.6, handletextpad=0.5,
           borderaxespad=0.2, labelspacing=0.35, columnspacing=1.0)

for ax in (axg, axs):
    ax.yaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6)
    ax.xaxis.set_tick_params(labelsize=6.5, colors=MUTED, width=0.6, length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)
axg.xaxis.set_tick_params(labelsize=7, colors="#0b0b0b")

fig.tight_layout(pad=0.4, w_pad=1.3)
os.makedirs("figs", exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(f"figs/fig5_simpoints.{ext}", bbox_inches="tight",
                pad_inches=0.02)
print("wrote figs/fig5_simpoints.{pdf,png}")
