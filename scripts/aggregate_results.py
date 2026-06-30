#!/usr/bin/env python3
"""Aggregate ChampSim regression outputs (<path>/<simpoint>.out) and print averages.

Parses the ROI stats from each .out file -- IPC, u-op-cache hit rate, switch MPKI,
and the frontend-loss decomposition (misses + stall-cycles[upper] + backend-idle[tight],
each split steady vs recovery) -- and reports per-simpoint averages over a directory.

Usage:
    python3 aggregate_results.py <dir> [<dir> ...] [--pattern '*.out'] [--include names.txt]
    # e.g. python3 aggregate_results.py results/.../muop_4kb_new_backend_W50M_I150M
"""
import argparse
import glob
import math
import os
import re

# ROI stats block line (NOT the "Warmup ..."/"Heartbeat ..."/"Simulation finished ..." lines)
RE_IPC = re.compile(r'^CPU \d+ cumulative IPC:\s+([\d.]+)\s+instructions:\s+(\d+)\s+cycles:\s+(\d+)', re.M)
RE_HIT = re.compile(r'uop-cache hit rate:\s+([\d.]+)%', re.M)
RE_SWITCH = re.compile(r'uop-cache mode-switch stalls:\s+\d+\s+MPKI:\s+([\d.]+)', re.M)
RE_MISS = re.compile(r'frontend-loss misses: steady (\d+) recovery (\d+) \(of (\d+) total', re.M)
RE_UPPER = re.compile(r'frontend-loss stall-cycles \(upper\): steady (\d+) recovery (\d+) switch (\d+) \| total (\d+)', re.M)
RE_TIGHT = re.compile(r'frontend-loss backend-idle \(tight\): steady (\d+) recovery (\d+) \| total (\d+)', re.M)


def parse_file(path):
    """Return a dict of metrics for one .out file, or None if it has no ROI IPC."""
    with open(path, errors='ignore') as f:
        txt = f.read()
    m = RE_IPC.search(txt)
    if not m:
        return None  # incomplete / crashed run
    d = {'ipc': float(m.group(1)), 'instrs': int(m.group(2)), 'cycles': int(m.group(3))}

    h = RE_HIT.search(txt)
    d['hit'] = float(h.group(1)) if h else None
    s = RE_SWITCH.search(txt)
    d['switch_mpki'] = float(s.group(1)) if s else None

    mm = RE_MISS.search(txt)
    if mm:
        d['miss_steady'], d['miss_recovery'], d['miss_total'] = int(mm.group(1)), int(mm.group(2)), int(mm.group(3))
    up = RE_UPPER.search(txt)
    if up:
        d['up_steady'], d['up_recovery'], d['up_switch'], d['up_total'] = (int(up.group(i)) for i in range(1, 5))
    ti = RE_TIGHT.search(txt)
    if ti:
        d['ti_steady'], d['ti_recovery'], d['ti_total'] = int(ti.group(1)), int(ti.group(2)), int(ti.group(3))
    return d


def amean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float('nan')


def gmean(xs):
    xs = [x for x in xs if x is not None and x > 0]
    return math.exp(sum(math.log(x) for x in xs) / len(xs)) if xs else float('nan')


def pct(num, den):
    return 100.0 * num / den if den else float('nan')


def aggregate(path, pattern, include):
    files = sorted(glob.glob(os.path.join(path, pattern)))
    rows, skipped = [], 0
    for fp in files:
        name = os.path.splitext(os.path.basename(fp))[0]
        if include is not None and name not in include:
            continue
        d = parse_file(fp)
        if d is None:
            skipped += 1
            continue
        d['name'] = name
        rows.append(d)

    print(f"\n=== {path} ===")
    print(f"simpoints: {len(rows)}  (skipped {skipped} with no ROI IPC)")
    if not rows:
        return

    print(f"IPC:          arith-mean {amean([r['ipc'] for r in rows]):.4f}   geomean {gmean([r['ipc'] for r in rows]):.4f}")
    print(f"hit rate:     {amean([r['hit'] for r in rows]):.2f}%")
    print(f"switch MPKI:  {amean([r['switch_mpki'] for r in rows]):.3f}")

    # frontend-loss decomposition: average each simpoint's percentages (equal weight),
    # and also report the summed (cycle-weighted) split.
    have_fe = [r for r in rows if 'ti_total' in r]
    if have_fe:
        miss_share = amean([pct(r['miss_recovery'], r['miss_total']) for r in have_fe if r.get('miss_total')])
        up_pct = amean([pct(r['up_total'], r['cycles']) for r in have_fe])
        up_share = amean([pct(r['up_recovery'], r['up_steady'] + r['up_recovery']) for r in have_fe if (r['up_steady'] + r['up_recovery'])])
        ti_pct = amean([pct(r['ti_total'], r['cycles']) for r in have_fe])
        ti_share = amean([pct(r['ti_recovery'], r['ti_total']) for r in have_fe if r['ti_total']])
        # cycle-weighted (summed) tight split across all simpoints
        sum_ti_s = sum(r['ti_steady'] for r in have_fe)
        sum_ti_r = sum(r['ti_recovery'] for r in have_fe)
        print("--- frontend-loss decomposition (avg over simpoints) ---")
        print(f"  misses:                  recovery {miss_share:.1f}% of total misses")
        print(f"  dispatch-starve (upper): {up_pct:.2f}% of cycles   (recovery {up_share:.1f}% of steady+recovery)")
        print(f"  backend-idle   (tight):  {ti_pct:.2f}% of cycles   (recovery {ti_share:.1f}% of total)")
        print(f"  tight, cycle-weighted:   recovery {pct(sum_ti_r, sum_ti_s + sum_ti_r):.1f}% of total backend-idle cycles")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+', help='directory(ies) containing <simpoint>.out files')
    ap.add_argument('--pattern', default='*.out', help="glob for result files (default '*.out')")
    ap.add_argument('--include', help='file with one simpoint name per line; restrict to these')
    args = ap.parse_args()

    include = None
    if args.include:
        with open(args.include) as f:
            include = {line.strip() for line in f if line.strip()}
        print(f"[include] {len(include)} simpoint name(s); analysis restricted to these")

    for d in args.dirs:
        aggregate(d, args.pattern, include)


if __name__ == '__main__':
    main()
