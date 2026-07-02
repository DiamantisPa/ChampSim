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
RE_STG = re.compile(r'trace-stg: loop (\d+) function (\d+) dedup-hits (\d+) entangled (\d+) '
                    r'dropped: bad-layout (\d+) overflow (\d+) short (\d+) stored-uops (\d+)', re.M)
RE_STG_COV = re.compile(r'trace-stg coverage: unique IPs (\d+)/(\d+) \([\d.-]+%\) dynamic uops (\d+)/(\d+)', re.M)
RE_STALL = re.compile(r'trace-stall: traces (\d+) dedup-hits (\d+) stored-uops (\d+)', re.M)
RE_STALL_COV = re.compile(r'trace-stall coverage: unique IPs (\d+)/(\d+) \([\d.-]+%\) dynamic uops (\d+)/(\d+)', re.M)
BKT_LABELS = [16, 32, 64, 128, 256, 512, 1024]
_bkt = ' '.join(fr'{n}=([\d.-]+)%' for n in BKT_LABELS)
RE_STALL_OCC = re.compile(r'trace-stall top-by-occurrence \(cum% dyn-weight\): ' + _bkt, re.M)
RE_STALL_COVB = re.compile(r'trace-stall top-by-coverage \(cum% dyn-weight\): ' + _bkt, re.M)
_bktv = ' '.join(fr'{n}=([\d.-]+)' for n in BKT_LABELS)  # avg-occ / avg-len (no % suffix)
RE_OCC_AVGOCC = re.compile(r'trace-stall top-by-occurrence avg-occ: ' + _bktv, re.M)
RE_OCC_AVGLEN = re.compile(r'trace-stall top-by-occurrence avg-len: ' + _bktv, re.M)
RE_COV_AVGOCC = re.compile(r'trace-stall top-by-coverage avg-occ: ' + _bktv, re.M)
RE_COV_AVGLEN = re.compile(r'trace-stall top-by-coverage avg-len: ' + _bktv, re.M)


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

    stg = RE_STG.search(txt)
    if stg:
        d['stg_traces'] = int(stg.group(1)) + int(stg.group(2))  # loop + function = distinct traces
        d['stg_dedup'] = int(stg.group(3))
        d['stg_dropped'] = int(stg.group(5)) + int(stg.group(6)) + int(stg.group(7))
        d['stg_stored_uops'] = int(stg.group(8))
    cov = RE_STG_COV.search(txt)
    if cov:
        d['stg_uniq_cov'], d['stg_uniq_seen'] = int(cov.group(1)), int(cov.group(2))
        d['stg_dyn_cov'], d['stg_dyn_tot'] = int(cov.group(3)), int(cov.group(4))

    st = RE_STALL.search(txt)
    if st:
        d['stall_traces'], d['stall_dedup'], d['stall_stored_uops'] = int(st.group(1)), int(st.group(2)), int(st.group(3))
    stc = RE_STALL_COV.search(txt)
    if stc:
        d['stall_uniq_cov'], d['stall_uniq_seen'] = int(stc.group(1)), int(stc.group(2))
        d['stall_dyn_cov'], d['stall_dyn_tot'] = int(stc.group(3)), int(stc.group(4))

    def _floats(m):
        out = []
        for x in m.groups():
            try:
                out.append(float(x))
            except ValueError:
                out.append(None)
        return out

    oc = RE_STALL_OCC.search(txt)
    if oc:
        d['stall_occ_bkt'] = _floats(oc)
    cb = RE_STALL_COVB.search(txt)
    if cb:
        d['stall_cov_bkt'] = _floats(cb)
    for key, rx in (('s_oo', RE_OCC_AVGOCC), ('s_ol', RE_OCC_AVGLEN), ('s_co', RE_COV_AVGOCC), ('s_cl', RE_COV_AVGLEN)):
        mm2 = rx.search(txt)
        if mm2:
            d[key] = _floats(mm2)
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

    # trace capture (stager): how many distinct traces segmentation built + coverage
    # (only shown when the stager actually ran, i.e. it observed u-ops)
    have_stg = [r for r in rows if r.get('stg_dyn_tot', 0) > 0 or r.get('stg_traces', 0) > 0]
    if have_stg:
        print("\n--- trace capture (stager) ---")
        print(f"traces captured:   {amean([r['stg_traces'] for r in have_stg]):,.0f}   "
            f"(dedup re-hits {amean([r['stg_dedup'] for r in have_stg]):,.0f}, dropped {amean([r['stg_dropped'] for r in have_stg]):,.0f})")
        print(f"stored u-ops:      {amean([r['stg_stored_uops'] for r in have_stg]):,.0f}")
        cov_have = [r for r in have_stg if 'stg_dyn_tot' in r]
        if cov_have:
            dyn = amean([pct(r['stg_dyn_cov'], r['stg_dyn_tot']) for r in cov_have if r['stg_dyn_tot']])
            uniq = amean([pct(r['stg_uniq_cov'], r['stg_uniq_seen']) for r in cov_have if r['stg_uniq_seen']])
            print(f"coverage:          {dyn:.1f}% of dynamic u-ops   ({uniq:.1f}% of unique IPs)")

    # trace capture (stall segmenter): traces built at costly ROB-empty build-mode stalls
    # (only shown when the stall segmenter actually ran)
    have_stall = [r for r in rows if r.get('stall_dyn_tot', 0) > 0 or r.get('stall_traces', 0) > 0]
    if have_stall:
        print("\n--- trace capture (stall segmenter) ---")
        print(f"traces captured:   {amean([r['stall_traces'] for r in have_stall]):,.0f}   "
            f"(dedup re-hits {amean([r['stall_dedup'] for r in have_stall]):,.0f})")
        print(f"stored u-ops:      {amean([r['stall_stored_uops'] for r in have_stall]):,.0f}")
        cov_have = [r for r in have_stall if 'stall_dyn_tot' in r]
        if cov_have:
            dyn = amean([pct(r['stall_dyn_cov'], r['stall_dyn_tot']) for r in cov_have if r['stall_dyn_tot']])
            uniq = amean([pct(r['stall_uniq_cov'], r['stall_uniq_seen']) for r in cov_have if r['stall_uniq_seen']])
            print(f"coverage:          {dyn:.1f}% of dynamic u-ops   ({uniq:.1f}% of unique IPs)")
        def _colavg(key):
            rr = [r[key] for r in have_stall if key in r]
            return [amean([row[i] for row in rr]) for i in range(len(BKT_LABELS))] if rr else None

        occ_avg, cov_avg = _colavg('stall_occ_bkt'), _colavg('stall_cov_bkt')
        if occ_avg:
            def _row(vals, unit=''):
                return "  ".join(f"{n}={v:.1f}{unit}" for n, v in zip(BKT_LABELS, vals))
            oo, ol, co, cl = _colavg('s_oo'), _colavg('s_ol'), _colavg('s_co'), _colavg('s_cl')
            print("top-N traces, ranked by OCCURRENCE:")
            print("  cum% dyn-weight: " + _row(occ_avg, '%'))
            if oo:
                print("  avg occurrence:  " + _row(oo))
                print("  avg length:      " + _row(ol))
            print("top-N traces, ranked by COVERAGE:")
            print("  cum% dyn-weight: " + _row(cov_avg, '%'))
            if co:
                print("  avg occurrence:  " + _row(co))
                print("  avg length:      " + _row(cl))

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
