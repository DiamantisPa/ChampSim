#!/bin/bash
# Sweep the ALT trace-fill walk knobs on one trace and print a comparison table.
#
# Runs: off (baseline), window (instant-install ceiling), and alt at each walk
# delay, all with the stall builder and a given trace-store capacity.  Reports
# hit rate, ROI IPC, and the trace-alt stats (triggers / dropped / installed /
# late-misses / useful-hits) so timing (late) and pollution (useful/installed)
# are visible side by side.
#
# Usage:
#   scripts/alt_walk_sweep.sh <binary> <trace> [warmup] [insts]
# Env overrides:
#   DELAYS="0 5 20"  STORE=16384  WALK_MAX=2  WALK_WIDTH=1  WALK_WAIT=0
#   ROB=64  DEPTH=144  MIN_OCC=1
#   OUTDIR=<dir for .out files>   (default: ./alt_sweep_out)
#
# Example:
#   scripts/alt_walk_sweep.sh bin/muop_4kb_stall_segmentation \
#       ~/traces/google_traces/merced_0000.champsim.gz 2000000 12000000

set -u
BIN=${1:?usage: alt_walk_sweep.sh <binary> <trace> [warmup] [insts]}
TRACE=${2:?usage: alt_walk_sweep.sh <binary> <trace> [warmup] [insts]}
WARMUP=${3:-2000000}
INSTS=${4:-12000000}

DELAYS=${DELAYS:-"0 5 20"}
STORE=${STORE:-16384}
WALK_MAX=${WALK_MAX:-2}
WALK_WIDTH=${WALK_WIDTH:-1}
WALK_WAIT=${WALK_WAIT:-0}
ROB=${ROB:-64}
DEPTH=${DEPTH:-144}
MIN_OCC=${MIN_OCC:-1}
OUTDIR=${OUTDIR:-./alt_sweep_out}

mkdir -p "$OUTDIR"

run() { # fill_mode delay outfile
  env PROMETHEUS_TRACE=stall PROMETHEUS_TRACE_FILL="$1" \
      PROMETHEUS_STALL_MIN_OCC="$MIN_OCC" PROMETHEUS_STALL_ROB="$ROB" PROMETHEUS_STALL_DEPTH="$DEPTH" \
      PROMETHEUS_TRACE_STORE="$STORE" PROMETHEUS_WALK_DELAY="$2" PROMETHEUS_WALK_MAX="$WALK_MAX" \
      PROMETHEUS_WALK_WIDTH="$WALK_WIDTH" PROMETHEUS_WALK_WAIT="$WALK_WAIT" \
      "$BIN" -w "$WARMUP" -i "$INSTS" "$TRACE" >"$3" 2>&1
}

hitrate() { grep -m1 "uop-cache hit rate:" "$1" | grep -oP "hit rate: \K[0-9.]+"; }
roi_ipc() { grep -m1 "^CPU 0 cumulative IPC:" "$1" | grep -oP "IPC: \K[0-9.]+"; }
swmpki()  { grep -m1 "uop-cache mode-switch stalls:" "$1" | grep -oP "MPKI:\s+\K[0-9.]+"; }
altline() { grep -m1 "trace-alt:" "$1" | sed 's/.*trace-alt: //'; }

echo "binary=$BIN trace=$(basename "$TRACE") warmup=$WARMUP insts=$INSTS"
echo "store=$STORE walk_max=$WALK_MAX width=$WALK_WIDTH wait=$WALK_WAIT rob=$ROB depth=$DEPTH min_occ=$MIN_OCC"
echo

run off    0 "$OUTDIR/off.out" &
run window 0 "$OUTDIR/window.out" &
for d in $DELAYS; do
  run alt "$d" "$OUTDIR/alt_d${d}.out" &
done
wait

printf "%-14s %-8s %-9s %-8s %s\n" "config" "hit%" "ROI-IPC" "swMPKI" "trace-alt"
printf "%-14s %-8s %-9s %-8s %s\n" "off"    "$(hitrate "$OUTDIR/off.out")"    "$(roi_ipc "$OUTDIR/off.out")"    "$(swmpki "$OUTDIR/off.out")"    "-"
printf "%-14s %-8s %-9s %-8s %s\n" "window" "$(hitrate "$OUTDIR/window.out")" "$(roi_ipc "$OUTDIR/window.out")" "$(swmpki "$OUTDIR/window.out")" "- (instant ceiling)"
for d in $DELAYS; do
  f="$OUTDIR/alt_d${d}.out"
  printf "%-14s %-8s %-9s %-8s %s\n" "alt delay=$d" "$(hitrate "$f")" "$(roi_ipc "$f")" "$(swmpki "$f")" "$(altline "$f")"
done
