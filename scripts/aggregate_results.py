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
RE_MISS = re.compile(r'frontend-loss misses: steady (\d+) \(traced (\d+)\) recovery (\d+) \(traced (\d+)\) \(of (\d+) total', re.M)
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
        d['miss_steady'], d['miss_steady_traced'] = int(mm.group(1)), int(mm.group(2))
        d['miss_recovery'], d['miss_recovery_traced'] = int(mm.group(3)), int(mm.group(4))
        d['miss_total'] = int(mm.group(5))
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

    have_fe = [r for r in rows if 'ti_total' in r]
    if not have_fe:
        return
    print("\n--- frontend-loss decomposition (averaged per simpoint) ---")

    # u-op-cache misses (absolute count + steady/recovery split + trace-fill ceiling)
    miss_abs = amean([r['miss_total'] for r in have_fe])
    miss_rec = amean([pct(r['miss_recovery'], r['miss_total']) for r in have_fe if r.get('miss_total')])
    miss_ste = amean([pct(r['miss_steady'], r['miss_total']) for r in have_fe if r.get('miss_total')])
    tr_ste = amean([pct(r['miss_steady_traced'], r['miss_steady']) for r in have_fe if r.get('miss_steady')])
    tr_rec = amean([pct(r['miss_recovery_traced'], r['miss_recovery']) for r in have_fe if r.get('miss_recovery')])
    print(f"muop cache misses:               {miss_abs:,.0f}")
    print(f"  recovery %:                    {miss_rec:.1f}%")
    print(f"  steady %:                      {miss_ste:.1f}%")
    print(f"  trace-covered (fill ceiling):  steady {tr_ste:.1f}%   recovery {tr_rec:.1f}%")

    # dispatch-starve (upper bound): steady+recovery dispatch-starvation cycles
    up_abs = amean([r['up_steady'] + r['up_recovery'] for r in have_fe])
    up_sw = amean([r['up_switch'] for r in have_fe])
    up_pct = amean([pct(r['up_steady'] + r['up_recovery'], r['cycles']) for r in have_fe])
    up_ste = amean([pct(r['up_steady'], r['up_steady'] + r['up_recovery']) for r in have_fe if (r['up_steady'] + r['up_recovery'])])
    up_rec = amean([pct(r['up_recovery'], r['up_steady'] + r['up_recovery']) for r in have_fe if (r['up_steady'] + r['up_recovery'])])
    print(f"dispatch-starve (upper):         {up_abs:,.0f} cycles   ({up_pct:.2f}% of runtime; +{up_sw:,.0f} switch)")
    print(f"  steady %:                      {up_ste:.1f}%")
    print(f"  recovery %:                    {up_rec:.1f}%")

    # backend-idle (tight bound): ROB fully empty in build mode
    ti_abs = amean([r['ti_total'] for r in have_fe])
    ti_pct = amean([pct(r['ti_total'], r['cycles']) for r in have_fe])
    ti_ste = amean([pct(r['ti_steady'], r['ti_total']) for r in have_fe if r['ti_total']])
    ti_rec = amean([pct(r['ti_recovery'], r['ti_total']) for r in have_fe if r['ti_total']])
    print(f"backend-idle (tight):            {ti_abs:,.0f} cycles   ({ti_pct:.2f}% of runtime)")
    print(f"  steady %:                      {ti_ste:.1f}%")
    print(f"  recovery %:                    {ti_rec:.1f}%")


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
