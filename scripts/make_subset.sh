#!/bin/bash
# Build a frontend-bound workload subset list (UCP-style: ideal-uop-cache speedup
# >= threshold) from per-workload .out files, then re-aggregate result dirs on it.
#
# Usage:
#   scripts/make_subset.sh <ideal_dir> <baseline_dir> [threshold] > subset.txt
#   python3 scripts/aggregate_results.py <any_result_dir> --include subset.txt
#
#   <ideal_dir>    e.g. results/.../muop_ideal_new_backend_W50M_I150M
#   <baseline_dir> e.g. results/.../no_muop_new_backend_W50M_I150M
#   [threshold]    minimum ideal speedup, default 1.05 (>=5% headroom)
#
# Emits one simpoint name per line (names with ideal/baseline IPC both present
# and ideal_IPC / baseline_IPC >= threshold).  Diagnostics go to stderr.

set -u
IDEAL_DIR=${1:?usage: make_subset.sh <ideal_dir> <baseline_dir> [threshold]}
BASE_DIR=${2:?usage: make_subset.sh <ideal_dir> <baseline_dir> [threshold]}
THRESH=${3:-1.05}

total=0
kept=0
for f in "$IDEAL_DIR"/*.out; do
  n=$(basename "$f" .out)
  b="$BASE_DIR/$n.out"
  [ -f "$b" ] || { echo "[skip] no baseline for $n" >&2; continue; }
  i_ipc=$(grep -m1 '^CPU 0 cumulative IPC:' "$f" | grep -oP 'IPC:\s+\K[0-9.]+')
  b_ipc=$(grep -m1 '^CPU 0 cumulative IPC:' "$b" | grep -oP 'IPC:\s+\K[0-9.]+')
  [ -n "$i_ipc" ] && [ -n "$b_ipc" ] || { echo "[skip] no ROI IPC for $n" >&2; continue; }
  total=$((total + 1))
  if awk -v i="$i_ipc" -v b="$b_ipc" -v t="$THRESH" 'BEGIN { exit !(b > 0 && i/b >= t) }'; then
    echo "$n"
    kept=$((kept + 1))
  fi
done
echo "[subset] kept $kept of $total workloads (ideal speedup >= $THRESH)" >&2
