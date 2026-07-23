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
RE_STALL = re.compile(r'trace-stall: traces (\d+) \(of (\d+) candidates\) dedup-hits (\d+) stored-uops (\d+)(?: l1i-gated (\d+))?', re.M)
RE_STALL_COV = re.compile(r'trace-stall coverage: unique IPs (\d+)/(\d+) \([\d.-]+%\) dynamic uops (\d+)/(\d+)', re.M)
RE_STORE = re.compile(r'trace-store: evictions (\d+) conflict-evictions (\d+) occupancy (\d+)', re.M)
BKT_LABELS = [16, 32, 64, 128, 256, 512, 1024]
_bkt = ' '.join(fr'{n}=([\d.-]+)%' for n in BKT_LABELS)
RE_STALL_OCC = re.compile(r'trace-stall top-by-occurrence \(cum% dyn-weight\): ' + _bkt, re.M)
RE_STALL_COVB = re.compile(r'trace-stall top-by-coverage \(cum% dyn-weight\): ' + _bkt, re.M)
RE_STALL_COSTC = re.compile(r'trace-stall top-by-cost \(cum% stall-cycles\): ' + _bkt + r'(?: \(total (\d+)\))?', re.M)
RE_STALL_COSTW = re.compile(r'trace-stall top-by-cost \(cum% dyn-weight\): ' + _bkt, re.M)
RE_OCC_COSTC = re.compile(r'trace-stall top-by-occurrence \(cum% stall-cycles\): ' + _bkt, re.M)
RE_COV_COSTC = re.compile(r'trace-stall top-by-coverage \(cum% stall-cycles\): ' + _bkt, re.M)
_bktv = ' '.join(fr'{n}=([\d.-]+)' for n in BKT_LABELS)  # avg-occ / avg-len (no % suffix)
RE_OCC_AVGOCC = re.compile(r'trace-stall top-by-occurrence avg-occ: ' + _bktv, re.M)
RE_OCC_AVGLEN = re.compile(r'trace-stall top-by-occurrence avg-len: ' + _bktv, re.M)
RE_COV_AVGOCC = re.compile(r'trace-stall top-by-coverage avg-occ: ' + _bktv, re.M)
RE_COV_AVGLEN = re.compile(r'trace-stall top-by-coverage avg-len: ' + _bktv, re.M)
RE_CST_AVGOCC = re.compile(r'trace-stall top-by-cost avg-occ: ' + _bktv, re.M)
RE_CST_AVGLEN = re.compile(r'trace-stall top-by-cost avg-len: ' + _bktv, re.M)
RE_ALT = re.compile(r'trace-alt: triggers (\d+) dropped (\d+) installed-windows (\d+) late-misses (\d+)(?: useful-hits (\d+))?(?: wait-cycles (\d+))?'
                    r'(?: lines-issued (\d+) line-stalls (\d+))?', re.M)
RE_CHAIN = re.compile(r'trace-chain: unencodable (\d+) truncated (\d+) buffer-hits (\d+) buffer-evicted-unused (\d+)'
                      r'(?: filtered (\d+))? slack\(4/8/16/32/64/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)

RE_UCPPORT = re.compile(r'trace-ucp: cond (\d+) cond-misses (\d+) h2p-marked (\d+) h2p-marked-misses (\d+) paths (\d+) '
                        r'stops\(sat/ind/btbmiss/maxip\): (\d+) (\d+) (\d+) (\d+)(?: ind-walked (\d+))?', re.M)

# UCP_ISCA24 artifact output format (their profiler printer; ROI IPC line is standard)
RE_UCP_HIT = re.compile(r'^UOP_CACHE_HIT:\s+([\d.]+)', re.M)
RE_UCP_SWITCH = re.compile(r'^SWITCH_STALLS_MPKI:\s+([\d.]+)', re.M)
RE_UCP_L1I = re.compile(r'^L1I Hit Rate:\s+([\d.]+)', re.M)
RE_UCP_PREF = re.compile(r'^total_window_pref:\s+(\d+)', re.M)
RE_UCP_PREF_HITS = re.compile(r'^hits_from_pref:\s+(\d+)', re.M)
RE_UCP_H2P_COV = re.compile(r'^h2p_coverage:\s+([\d.]+)', re.M)
RE_UCP_H2P_ACC = re.compile(r'^h2p_accuracy:\s+([\d.]+)', re.M)
# standard ChampSim L1I prefetch-traffic line (both printers emit it)
RE_L1I_PREF = re.compile(r'L1I PREFETCH\s+REQUESTED:\s+(\d+)\s+ISSUED:\s+(\d+)\s+USEFUL:\s+(\d+)\s+USELESS:\s+(\d+)', re.M)


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

    # fall back to the UCP_ISCA24 artifact's output format
    if d['hit'] is None:
        h = RE_UCP_HIT.search(txt)
        if h:
            d['hit'] = float(h.group(1))
            d['is_ucp'] = True
    if d['switch_mpki'] is None:
        s = RE_UCP_SWITCH.search(txt)
        if s:
            d['switch_mpki'] = float(s.group(1))
    if d.get('is_ucp'):
        for key, rx, cast in (('ucp_l1i_hit', RE_UCP_L1I, float), ('ucp_pref', RE_UCP_PREF, int),
                              ('ucp_pref_hits', RE_UCP_PREF_HITS, int),
                              ('ucp_h2p_cov', RE_UCP_H2P_COV, float), ('ucp_h2p_acc', RE_UCP_H2P_ACC, float)):
            mu = rx.search(txt)
            if mu:
                d[key] = cast(mu.group(1))
    lp = RE_L1I_PREF.search(txt)
    if lp:
        d['l1i_pref_issued'], d['l1i_pref_useful'] = int(lp.group(2)), int(lp.group(3))

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
        d['stall_traces'], d['stall_candidates'] = int(st.group(1)), int(st.group(2))
        d['stall_dedup'], d['stall_stored_uops'] = int(st.group(3)), int(st.group(4))
        d['stall_l1i_gated'] = int(st.group(5)) if st.group(5) else None
    stc = RE_STALL_COV.search(txt)
    if stc:
        d['stall_uniq_cov'], d['stall_uniq_seen'] = int(stc.group(1)), int(stc.group(2))
        d['stall_dyn_cov'], d['stall_dyn_tot'] = int(stc.group(3)), int(stc.group(4))
    sto = RE_STORE.search(txt)
    if sto:
        d['store_evict'], d['store_conflict'], d['store_occ'] = int(sto.group(1)), int(sto.group(2)), int(sto.group(3))

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
    okc = RE_OCC_COSTC.search(txt)
    if okc:
        d['stall_occ_cost_bkt'] = _floats(okc)
    ckc = RE_COV_COSTC.search(txt)
    if ckc:
        d['stall_cov_cost_bkt'] = _floats(ckc)
    kc = RE_STALL_COSTC.search(txt)
    if kc:
        d['stall_cost_bkt'] = _floats(kc)[:len(BKT_LABELS)]  # 8th group is the optional total
        d['stall_cost_total'] = int(kc.group(len(BKT_LABELS) + 1)) if kc.group(len(BKT_LABELS) + 1) else None
    kw = RE_STALL_COSTW.search(txt)
    if kw:
        d['stall_costw_bkt'] = _floats(kw)
    al = RE_ALT.search(txt)
    if al:
        d['alt_triggers'], d['alt_drops'] = int(al.group(1)), int(al.group(2))
        d['alt_windows'], d['alt_late'] = int(al.group(3)), int(al.group(4))
        d['alt_useful'] = int(al.group(5)) if al.group(5) else None
        d['alt_wait'] = int(al.group(6)) if al.group(6) else None
        d['alt_lines'] = int(al.group(7)) if al.group(7) else None
        d['alt_line_stalls'] = int(al.group(8)) if al.group(8) else None
    ch = RE_CHAIN.search(txt)
    if ch:
        d['chain_unenc'], d['chain_trunc'] = int(ch.group(1)), int(ch.group(2))
        d['chain_hits'], d['chain_evict'] = int(ch.group(3)), int(ch.group(4))
        d['chain_filtered'] = int(ch.group(5)) if ch.group(5) else None
        d['chain_slack'] = [int(ch.group(i)) for i in range(6, 12)]
    up2 = RE_UCPPORT.search(txt)
    if up2:
        (d['ucpp_cond'], d['ucpp_cond_miss'], d['ucpp_marked'], d['ucpp_marked_miss'], d['ucpp_paths'],
         d['ucpp_stop_sat'], d['ucpp_stop_ind'], d['ucpp_stop_btbmiss'], d['ucpp_stop_maxip']) = (int(up2.group(i)) for i in range(1, 10))
        d['ucpp_ind_walked'] = int(up2.group(10)) if up2.group(10) else 0
    for key, rx in (('s_oo', RE_OCC_AVGOCC), ('s_ol', RE_OCC_AVGLEN), ('s_co', RE_COV_AVGOCC), ('s_cl', RE_COV_AVGLEN),
                    ('s_ko', RE_CST_AVGOCC), ('s_kl', RE_CST_AVGLEN)):
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


def load_rows(path, pattern, include):
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
    return rows, skipped


def aggregate(path, pattern, include, baseline=None, scatter=False):
    rows, skipped = load_rows(path, pattern, include)

    print(f"\n=== {path} ===")
    print(f"simpoints: {len(rows)}  (skipped {skipped} with no ROI IPC)")
    if not rows:
        return

    print(f"IPC:          arith-mean {amean([r['ipc'] for r in rows]):.4f}   geomean {gmean([r['ipc'] for r in rows]):.4f}")
    print(f"hit rate:     {amean([r['hit'] for r in rows]):.2f}%")
    print(f"switch MPKI:  {amean([r['switch_mpki'] for r in rows]):.3f}")

    # per-workload speedup vs a baseline directory (matched by simpoint name)
    if baseline:
        pairs = [(r['name'], r['ipc'], baseline[r['name']]) for r in rows if baseline.get(r['name'], 0) > 0]
        missing = len(rows) - len(pairs)
        if pairs:
            g = gmean([ipc / b for _, ipc, b in pairs])
            note = f", {missing} unmatched" if missing else ""
            print(f"vs baseline:  geomean speedup {100 * (g - 1):+.3f}%   ({len(pairs)} workloads{note})")
            if scatter:
                print("per-workload speedup (sorted):")
                for name, ipc, b in sorted(pairs, key=lambda p: p[1] / p[2], reverse=True):
                    print(f"  {name:<40s} {100 * (ipc / b - 1):+7.2f}%   ipc {ipc:.4f}  base {b:.4f}")

    # UCP artifact runs: their prefetch/H2P counters (parsed from their output format)
    have_ucp = [r for r in rows if r.get('is_ucp')]
    if have_ucp:
        print("\n--- UCP artifact stats ---")
        l1i = [r['ucp_l1i_hit'] for r in have_ucp if 'ucp_l1i_hit' in r]
        if l1i:
            print(f"L1I hit rate:      {amean(l1i):.2f}%")
        pref = [r['ucp_pref'] for r in have_ucp if 'ucp_pref' in r]
        if pref and amean(pref) > 0:
            hits = [r.get('ucp_pref_hits') for r in have_ucp if r.get('ucp_pref_hits') is not None]
            line = f"window prefetches: {amean(pref):,.0f}"
            if hits:
                acc = amean([pct(r['ucp_pref_hits'], r['ucp_pref']) for r in have_ucp
                             if r.get('ucp_pref_hits') is not None and r.get('ucp_pref')])
                line += f"   muop hits from prefetched entries: {amean(hits):,.0f} ({acc:.1f}% of prefetched)"
            print(line)
        cov = [r['ucp_h2p_cov'] for r in have_ucp if 'ucp_h2p_cov' in r]
        acc = [r['ucp_h2p_acc'] for r in have_ucp if 'ucp_h2p_acc' in r]
        if cov or acc:
            print(f"H2P detector:      coverage {amean(cov):.1f}%   accuracy {amean(acc):.1f}%")

    # UCP port runs (our reimplementation): H2P detector quality + walk stop reasons
    have_up = [r for r in rows if r.get('ucpp_cond', 0) > 0]
    if have_up:
        print("\n--- UCP port (our framework) ---")
        cov = amean([pct(r['ucpp_marked_miss'], r['ucpp_cond_miss']) for r in have_up if r['ucpp_cond_miss']])
        acc = amean([pct(r['ucpp_marked_miss'], r['ucpp_marked']) for r in have_up if r['ucpp_marked']])
        mrate = amean([pct(r['ucpp_marked'], r['ucpp_cond']) for r in have_up if r['ucpp_cond']])
        print(f"H2P detector:      coverage {cov:.1f}%   accuracy {acc:.1f}%   (marked {mrate:.1f}% of conditionals)")
        paths = amean([r['ucpp_paths'] for r in have_up])
        stops = [amean([r[k] for r in have_up]) for k in ('ucpp_stop_sat', 'ucpp_stop_ind', 'ucpp_stop_btbmiss', 'ucpp_stop_maxip')]
        print(f"alt paths:         {paths:,.0f}   stops: sat {stops[0]:,.0f}  indirect {stops[1]:,.0f}  btb-miss {stops[2]:,.0f}  max-ip {stops[3]:,.0f}")
        iw = amean([r.get('ucpp_ind_walked', 0) for r in have_up])
        if iw > 0:
            print(f"Alt-Ind:           {iw:,.0f} indirects walked through on a predicted target")

    # L1I prefetch traffic (any run that issued prefetches: UCP alt path, L1I prefetchers)
    have_lp = [r for r in rows if r.get('l1i_pref_issued', 0) > 0]
    if have_lp:
        iss = amean([r['l1i_pref_issued'] for r in have_lp])
        acc = amean([pct(r['l1i_pref_useful'], r['l1i_pref_issued']) for r in have_lp if r['l1i_pref_issued']])
        print(f"L1I prefetches:    issued {iss:,.0f}   useful {amean([r['l1i_pref_useful'] for r in have_lp]):,.0f} ({acc:.1f}%)   [{len(have_lp)}/{len(rows)} workloads]")

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
        cand = amean([r.get('stall_candidates', r['stall_traces']) for r in have_stall])
        kept = amean([r['stall_traces'] for r in have_stall])
        print(f"traces captured:   {kept:,.0f}   of {cand:,.0f} candidates ({pct(kept, cand):.1f}% kept after filter)   "
            f"(dedup re-hits {amean([r['stall_dedup'] for r in have_stall]):,.0f})")
        gated = [r['stall_l1i_gated'] for r in have_stall if r.get('stall_l1i_gated') is not None]
        if gated and amean(gated) > 0:
            print(f"l1i-gated out:     {amean(gated):,.0f} costly stretches (no L1I miss observed)")
        print(f"stored u-ops:      {amean([r['stall_stored_uops'] for r in have_stall]):,.0f}")
        have_store = [r for r in have_stall if 'store_evict' in r]
        if have_store:
            ev = amean([r['store_evict'] for r in have_store])
            cf = amean([pct(r['store_conflict'], r['store_evict']) for r in have_store if r['store_evict']])
            print(f"store:             occupancy {amean([r['store_occ'] for r in have_store]):,.0f}   "
                  f"evictions {ev:,.0f} ({cf:.1f}% conflict = while underfull)")
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
            occ_cost, cov_cost = _colavg('stall_occ_cost_bkt'), _colavg('stall_cov_cost_bkt')
            print("top-N traces, ranked by OCCURRENCE:")
            print("  cum% dyn-weight: " + _row(occ_avg, '%'))
            if occ_cost:
                print("  cum% stall-cyc:  " + _row(occ_cost, '%'))
            if oo:
                print("  avg occurrence:  " + _row(oo))
                print("  avg length:      " + _row(ol))
            print("top-N traces, ranked by COVERAGE:")
            print("  cum% dyn-weight: " + _row(cov_avg, '%'))
            if cov_cost:
                print("  cum% stall-cyc:  " + _row(cov_cost, '%'))
            if co:
                print("  avg occurrence:  " + _row(co))
                print("  avg length:      " + _row(cl))
        cost_avg, costw_avg = _colavg('stall_cost_bkt'), _colavg('stall_costw_bkt')
        if cost_avg:
            def _row2(vals, unit=''):
                return "  ".join(f"{n}={v:.1f}{unit}" for n, v in zip(BKT_LABELS, vals))
            ko, kl = _colavg('s_ko'), _colavg('s_kl')
            tot = amean([r['stall_cost_total'] for r in have_stall if r.get('stall_cost_total') is not None])
            print("top-N traces, ranked by ROB-STALL COST:")
            print(f"  cum% stall-cyc:  " + _row2(cost_avg, '%') + (f"   (avg total {tot:,.0f} cycles)" if not math.isnan(tot) else ""))
            if costw_avg:
                print("  cum% dyn-weight: " + _row2(costw_avg, '%'))
            if ko:
                print("  avg occurrence:  " + _row2(ko))
                print("  avg length:      " + _row2(kl))

    # ALT fill mode (metadata trace cache + timed pre-decode walk), shown when active
    have_alt = [r for r in rows if r.get('alt_triggers', 0) > 0]
    if have_alt:
        print("\n--- alt-trigger walk (metadata pre-decode) ---")
        trig = amean([r['alt_triggers'] for r in have_alt])
        drop = amean([r['alt_drops'] for r in have_alt])
        wins = amean([r['alt_windows'] for r in have_alt])
        late = amean([r['alt_late'] for r in have_alt])
        print(f"walks launched:    {trig:,.0f}   (dropped {drop:,.0f} = {pct(drop, trig + drop):.1f}% of trigger hits)")
        print(f"windows installed: {wins:,.0f}")
        print(f"late misses:       {late:,.0f}   (walk in flight but too slow)")
        useful = [r['alt_useful'] for r in have_alt if r.get('alt_useful') is not None]
        if useful:
            acc = amean([pct(r['alt_useful'], r['alt_windows']) for r in have_alt if r.get('alt_useful') is not None and r['alt_windows']])
            print(f"useful hits:       {amean(useful):,.0f}   (walk accuracy {acc:.1f}% of installed windows)")
        waits = [r['alt_wait'] for r in have_alt if r.get('alt_wait') is not None]
        if waits:
            wpct = amean([pct(r['alt_wait'], r['cycles']) for r in have_alt if r.get('alt_wait') is not None and r['cycles']])
            print(f"wait cycles:       {amean(waits):,.0f}   ({wpct:.2f}% of runtime; hit-under-fill stalls)")
        lns = [r['alt_lines'] for r in have_alt if r.get('alt_lines') is not None]
        if lns and amean(lns) > 0:
            lst = amean([r['alt_line_stalls'] for r in have_alt if r.get('alt_line_stalls') is not None])
            print(f"L1I lines issued:  {amean(lns):,.0f}   (walk line-stall cycles {lst:,.0f})")
        have_chain = [r for r in have_alt if r.get('chain_hits') is not None and (r['chain_hits'] > 0 or r.get('chain_unenc', 0) > 0)]
        if have_chain:
            print(f"chain: buffer hits {amean([r['chain_hits'] for r in have_chain]):,.0f}   "
                  f"evicted-unused {amean([r['chain_evict'] for r in have_chain]):,.0f}   "
                  f"unencodable {amean([r['chain_unenc'] for r in have_chain]):,.0f}   truncated {amean([r['chain_trunc'] for r in have_chain]):,.0f}")
            filt = [r['chain_filtered'] for r in have_chain if r.get('chain_filtered') is not None]
            if filt and amean(filt) > 0:
                print(f"chain filtered:    {amean(filt):,.0f} windows already resident (not staged)")
            sl = [amean([r['chain_slack'][i] for r in have_chain]) for i in range(6)]
            tot = sum(sl)
            if tot > 0:
                print("chain slack (probe->demand):  " + "  ".join(f"{lbl}={100*v/tot:.0f}%" for lbl, v in zip(['<=4', '<=8', '<=16', '<=32', '<=64', '>64'], sl)))

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
    ap.add_argument('--baseline', help='baseline result directory; adds per-dir geomean speedup vs it')
    ap.add_argument('--scatter', action='store_true', help='with --baseline: print per-workload speedups')
    args = ap.parse_args()

    include = None
    if args.include:
        with open(args.include) as f:
            include = {line.strip() for line in f if line.strip()}
        print(f"[include] {len(include)} simpoint name(s); analysis restricted to these")

    baseline = None
    if args.baseline:
        base_rows, base_skipped = load_rows(args.baseline, args.pattern, include)
        baseline = {r['name']: r['ipc'] for r in base_rows}
        print(f"[baseline] {args.baseline}: {len(baseline)} workloads (skipped {base_skipped})")

    for d in args.dirs:
        aggregate(d, args.pattern, include, baseline=baseline, scatter=args.scatter)


if __name__ == '__main__':
    main()
