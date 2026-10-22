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
RE_LOST = re.compile(r'frontend-loss lost-slots \(mid\): steady (\d+) recovery (\d+) \| total (\d+) slots', re.M)
DISPATCH_WIDTH = 8  # zen5_final dispatch width, for slot -> cycle-equivalent conversion
RE_STG = re.compile(r'trace-stg: loop (\d+) function (\d+) dedup-hits (\d+) entangled (\d+) '
                    r'dropped: bad-layout (\d+) overflow (\d+) short (\d+) stored-uops (\d+)', re.M)
RE_STG_COV = re.compile(r'trace-stg coverage: unique IPs (\d+)/(\d+) \([\d.-]+%\) dynamic uops (\d+)/(\d+)', re.M)
RE_STALL = re.compile(r'trace-stall: traces (\d+) \(of (\d+) candidates\) dedup-hits (\d+) stored-uops (\d+)(?: l1i-gated (\d+))?(?: marks-dropped (\d+))?', re.M)
RE_STALL_COV = re.compile(r'trace-stall coverage: unique IPs (\d+)/(\d+) \([\d.-]+%\) dynamic uops (\d+)/(\d+)', re.M)
RE_TAGS = re.compile(r'trace-store-tags: lookups (\d+) alias-hits (\d+) alias-overwrites (\d+)', re.M)
RE_STORE = re.compile(r'trace-store: evictions (\d+) conflict-evictions (\d+) occupancy (\d+)(?: windows (\d+) shared (\d+))?', re.M)
RE_EVENT = re.compile(r'trace-event: probes (\d+) probe-hits (\d+) traces (\d+) \(of (\d+) candidates\) dedup-hits (\d+)', re.M)
RE_MISSCLS = re.compile(r'chain-miss-classes: in-flight (\d+) entry (\d+) mid-trace (\d+)', re.M)
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
RE_ALT = re.compile(r'trace-alt: triggers (\d+) dropped (\d+) installed-windows (\d+) late-misses (\d+)(?: useful-hits (\d+))?'
                    r'(?: unused-evicted (\d+))?(?: wait-cycles (\d+))?'
                    r'(?: lines-issued (\d+) line-stalls (\d+))?', re.M)
RE_CHAIN = re.compile(r'trace-chain: unencodable (\d+) truncated (\d+) buffer-hits (\d+) buffer-evicted-unused (\d+)'
                      r'(?: filtered (\d+))?(?: conf-suppressed (\d+))?'
                      r'(?: evicted-then-missed (\d+) age\(256/1K/4K/16K/inf\): (\d+) (\d+) (\d+) (\d+) (\d+))?'
                      r' slack\(4/8/16/32/64/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)

RE_DELTAS = re.compile(r'trace-chain-deltas: seq (\d+) short (\d+) long (\d+) far (\d+)'
                       r'(?: traces: single (\d+) seq-only (\d+) with-short (\d+) with-long (\d+))?(?: far-truncated (\d+))?', re.M)
# packed-format widths from the knob echo (absent on older binaries -> 2/8/16 assumed)
RE_FMT = re.compile(r'meta_bits=(\d+) short_bits=(\d+) long_bits=(\d+)(?: kind_bits=(\d+) far_truncate=(\d+))?')
RE_NEED = re.compile(r'trace-chain-need: bits\(16/24/32/48/64/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) '
                     r'windows\(2/3/4/6/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) trunc-windows (\d+)', re.M)
RE_REACH = re.compile(r'trace-chain-reach: transfers\(4K/128K/256K/512K/1M/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) '
                      r'traces\(none/4K/128K/256K/512K/1M/inf\): (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_PROMOTE = re.compile(r'trace-chain-promote: buffer-rehits (\d+)', re.M)
RE_PROMOTE_KNOB = re.compile(r'buffer_promote=(\d+)')
RE_SHADOW = re.compile(r'trace-chain-shadow: mispredicts (\d+) past-mispredict walks (\d+) lines (\d+) staged-windows (\d+) buffer-hits (\d+)', re.M)
RE_SQUASH_KNOB = re.compile(r'probe_squash=(\d+)')
RE_WP = re.compile(r'trace-wrong-path: cursor-stops (\d+) shadows (\d+) addresses (\d+) fetched (\d+) uop-misses (\d+) fetch-cycles (\d+) '
                   r'decoder-busy-cycles (\d+) store-hits (\d+) ucp-triggers (\d+) walks (\d+) flushed (\d+) lines (\d+) staged (\d+) buffer-hits (\d+)'
                   r'(?: l1i-lines (\d+) line-stall-cycles (\d+) backend-full-cycles (\d+))?(?: stops (\d+) uop-fills (\d+))?', re.M)
RE_WP_KNOB = re.compile(r'wrong_path=(\w+)')
RE_PEEK = re.compile(r'trace-peek: probes (\d+) store-hits (\d+)', re.M)
RE_RK = re.compile(r'trace-recovery-key: mispredicts (\d+) filed (\d+) probes (\d+) hits (\d+) walks (\d+) buffer-hits (\d+)', re.M)
RE_RK_KNOB = re.compile(r'recovery_key=(\d) recovery_ghr=(\d+) recovery_h2p=(\d)')
RE_VP = re.compile(r'trace-vpred: lookups (\d+) matches (\d+) fires (\d+) store-hits (\d+) walks (\d+) walks-at-mispredict (\d+) '
                   r'buffer-hits (\d+) buffer-hits-at-mispredict (\d+)', re.M)
RE_VPT = re.compile(r'trace-vpred-train: events (\d+) after-mispredict (\d+) correct (\d+) correct-after-mispredict (\d+) wrong (\d+) '
                    r'no-provider (\d+) none (\d+) allocs (\d+) alloc-fail (\d+)', re.M)
RE_VP_KNOB = re.compile(r'vpred=(\d) vpred_tables=(\d+) vpred_entries=(\d+) vpred_hist=(\d+) vpred_kbytes=([\d.]+)')
RE_SUCC = re.compile(r'trace-succession (steady|after-mispredict): addr\(next/near/4K/jump\) (\d+) (\d+) (\d+) (\d+) '
                     r'gap\(0/1-8/9-64/65-256/more\) (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_CHFOLLOW = re.compile(r'trace-chain-follow: links (\d+) hops (\d+) found (\d+) walks\(hop1/2/3/4\+\) (\d+) (\d+) (\d+) (\d+) '
                         r'buffer-hits\(hop1/2/3/4\+\) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_CHAIN_KNOB = re.compile(r'chain_depth=(\d+) chain_window=(\d+)')
VP_KEYS = ('vp_lookups', 'vp_matches', 'vp_fires', 'vp_store_hits', 'vp_walks', 'vp_walks_mp', 'vp_bhits', 'vp_bhits_mp')
VPT_KEYS = ('vp_train', 'vp_train_mp', 'vp_correct', 'vp_correct_mp', 'vp_wrong', 'vp_noprov', 'vp_none', 'vp_allocs', 'vp_alloc_fail')
RE_PDIAG = re.compile(r'trace-peek-diag: mispredicts (\d+) with-trace (\d+)', re.M)
RE_PDIAG_KEY = re.compile(r'trace-peek-diag key (\S+): fires (\d+) useful (\d+) wrong (\d+)', re.M)
WP_KEYS = ('wp_cursor_stops', 'wp_shadows', 'wp_addresses', 'wp_fetched', 'wp_uop_misses', 'wp_fetch_cycles', 'wp_decoder_busy',
           'wp_store_hits', 'wp_ucp_triggers', 'wp_walks', 'wp_flushed', 'wp_lines', 'wp_staged', 'wp_buffer_hits',
           'wp_l1i_lines', 'wp_line_stalls', 'wp_full', 'wp_stops', 'wp_uop_fills')
RE_DBITS = re.compile(r'trace-chain-deltabits\(<=3\.\.16,>16\): ((?:\d+ ){14}\d+)', re.M)
RE_READMIT = re.compile(r'trace-stall-readmit: evicted-recaptures (\d+) readmitted (\d+)', re.M)
RE_MPOPEN = re.compile(r'trace-stall-mpopen: opened (\d+) traces (\d+)', re.M)
RE_SPLIT = re.compile(r'trace-stall-split: continuations (\d+) inherited-costly (\d+) stored (\d+)', re.M)
RE_MP = re.compile(r'uop-miss-profile: windows (\d+) misses (\d+) stall-cycles (\d+) other-stall-cycles (\d+)', re.M)
RE_MP_TOP = re.compile(r'uop-miss-profile top-by-cost\(1K/2K/4K/8K\) cum% (\S+) (\S+) (\S+) (\S+)  top-by-misses\(1K/2K/4K/8K\) cum% (\S+) (\S+) (\S+) (\S+)', re.M)
RE_MP_TR = re.compile(r'uop-miss-profile traced-by-segmenter: all (\S+)%  top1K (\S+)%  top2K (\S+)%  top4K (\S+)%  top8K (\S+)%', re.M)
RE_ORACLE_KNOB = re.compile(r'uop_oracle=(\d+)')
RE_TORACLE_KNOB = re.compile(r'trace_oracle=(\d+)')
RE_OINSTALL_KNOB = re.compile(r'oracle_install=(\d)')
RE_SUCP_KB = re.compile(r'SUCP total ([\d.]+) KB')
RE_FUP = re.compile(r'ftq-uop-prefetch: walks (\d+)', re.M)
RE_FDIP = re.compile(r'ftq-fdip: issued (\d+) dropped (\d+)(?: wrong-path (\d+))?', re.M)
RE_TAX = re.compile(r'stall-taxonomy tight \(.*?\): ([\d ]+) \| other: not-looked-up (\d+) uop-hit (\d+)', re.M)
RE_L2I = re.compile(r'l2-instr: perfect-hits (\d+) emissary-mark-events (\d+) l2-marks (\d+) diverted-evictions (\d+) high-priority-evicted (\d+)', re.M)
RE_RP = re.compile(r'resteer-prefetch: lookups (\d+) hits (\d+) prefetches (\d+) dropped (\d+) records (\d+)', re.M)
RE_RP_KB = re.compile(r'rp_storage_kb=([\d.]+)')
# trace_trigger_diag: class order matches O3_CPU::TD_*
TD_LABELS = ['unclassified', 'uncovered', 'not-stored', 'side-door', 'not-probed', 'dropped', 'resident-at-probe', 'late', 'staged-evicted', 'walked-gone',
             'cold', 'past-depth', 'never-costly', 'costly-once', 'costly-repeat']
_td10 = r'((?:\d+ )*\d+)'  # the class list (10 classes on older binaries, 15 with the window history)
RE_TD_PROBES = re.compile(r'trace-trigger-diag probes\(resident/conf/cap/dedup/launched\): (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_TD_ARR = re.compile(r'trace-trigger-diag arrivals (\d+) resident (\d+) staged (\d+) missed-by-class: ' + _td10, re.M)
RE_TD_LEAD = re.compile(r'trace-trigger-diag lead\(8/16/32/64/128/inf\) served: ' + ' '.join([r'(\d+)'] * 6) + ' missed: ' + ' '.join([r'(\d+)'] * 6), re.M)
RE_TD_MISS = re.compile(r'trace-trigger-diag misses-by-class: ' + _td10, re.M)
RE_TD_TIGHT = re.compile(r'trace-trigger-diag empty-rob-cycles-by-class: ' + _td10 + r' other (\d+)', re.M)
RE_TD_UPPER = re.compile(r'trace-trigger-diag starve-cycles-by-class: ' + _td10 + r' other (\d+)', re.M)
RE_TD_POS = re.compile(r'trace-trigger-diag staged-position\(0/1/2/3/4\+\) used: (\d+) (\d+) (\d+) (\d+) (\d+) unused: (\d+) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_TD_STAGE = re.compile(r'trace-trigger-diag staged (\d+) late (\d+) evicted-unused: late (\d+) on-time (\d+)', re.M)
RE_TD_REC = re.compile(r'trace-trigger-diag recovery-misses-by-class: ' + _td10, re.M)
RE_TD_HT = re.compile(r'trace-trigger-diag empty-rob-cycles-by-run-head: ' + _td10 + r' other (\d+)', re.M)
RE_TD_HU = re.compile(r'trace-trigger-diag starve-cycles-by-run-head: ' + _td10 + r' other (\d+)', re.M)
RE_COST_BIG = re.compile(r'trace-stall top-by-cost-big \(cum% stall-cycles\): 2048=([\d.-]+)% 4096=([\d.-]+)% 8192=([\d.-]+)%', re.M)
RE_RUN = re.compile(r'trace-run: cut-at-jump (\d+) cut-at-limit (\d+) inherited-costly (\d+) continuation-traces-stored (\d+)', re.M)
RE_LEAD = re.compile(r'trace-stall-lead: keyed (\d+) self (\d+) traces (\d+) conflicts (\d+) dist\(4K/64K/1M/far\) (\d+) (\d+) (\d+) (\d+)', re.M)
RE_STARVE = re.compile(r'trace-stall-starve: offset-windows\(0/1/2/3/4-7/8\+/past-depth/before/after\): ((?:\d+ ){8}\d+)', re.M)
RE_UCPPORT = re.compile(r'trace-ucp: cond (\d+) cond-misses (\d+) h2p-marked (\d+) h2p-marked-misses (\d+) paths (\d+) '
                        r'stops\(sat/ind/btbmiss/maxip\): (\d+) (\d+) (\d+) (\d+)(?: ind-walked (\d+))?(?: bank-conflicts (\d+))?', re.M)

# UCP_ISCA24 artifact output format (their profiler printer; ROI IPC line is standard)
RE_UCP_HIT = re.compile(r'^UOP_CACHE_HIT:\s+([\d.]+)', re.M)
RE_UCP_SWITCH = re.compile(r'^SWITCH_STALLS_MPKI:\s+([\d.]+)', re.M)
RE_UCP_L1I = re.compile(r'^L1I Hit Rate:\s+([\d.]+)', re.M)
RE_UCP_PREF = re.compile(r'^total_window_pref:\s+(\d+)', re.M)
RE_UCP_PREF_HITS = re.compile(r'^hits_from_pref:\s+(\d+)', re.M)
RE_UCP_H2P_COV = re.compile(r'^h2p_coverage:\s+([\d.]+)', re.M)
RE_UCP_H2P_ACC = re.compile(r'^h2p_accuracy:\s+([\d.]+)', re.M)
# standard ChampSim L1I prefetch-traffic line (both printers emit it)
RE_L1I_TOT = re.compile(r'L1I LOAD\s+ACCESS:\s+(\d+)\s+HIT:\s+(\d+)\s+MISS:\s+(\d+)', re.M)  # LOAD excludes prefetches but INCLUDES SUCP/UCP walk lines
# demand fetch lines that missed L1I (fetch-side, walk lines excluded), steady/recovery + summed wait
RE_DEMAND_L1I = re.compile(r'frontend-loss l1i-missed-fetches: steady (\d+) recovery (\d+) wait-cycles steady (\d+) recovery (\d+)', re.M)
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
    lt = RE_L1I_TOT.search(txt)
    if lt:
        d['l1i_access'], d['l1i_miss'] = int(lt.group(1)), int(lt.group(3))
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
    dl1 = RE_DEMAND_L1I.search(txt)
    if dl1:
        d['dl1_steady'], d['dl1_recovery'], d['dl1_wait_steady'], d['dl1_wait_recovery'] = (int(dl1.group(i)) for i in range(1, 5))
    ls = RE_LOST.search(txt)
    if ls:
        d['ls_steady'], d['ls_recovery'], d['ls_total'] = int(ls.group(1)), int(ls.group(2)), int(ls.group(3))

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
        d['stall_marks_dropped'] = int(st.group(6)) if st.group(6) else None
    stc = RE_STALL_COV.search(txt)
    if stc:
        d['stall_uniq_cov'], d['stall_uniq_seen'] = int(stc.group(1)), int(stc.group(2))
        d['stall_dyn_cov'], d['stall_dyn_tot'] = int(stc.group(3)), int(stc.group(4))
    sto = RE_STORE.search(txt)
    if sto:
        d['store_evict'], d['store_conflict'], d['store_occ'] = int(sto.group(1)), int(sto.group(2)), int(sto.group(3))
        if sto.group(4):
            d['store_win'], d['store_shared'] = int(sto.group(4)), int(sto.group(5))
    tg = RE_TAGS.search(txt)
    if tg:
        d['store_lookups'], d['store_alias_hits'], d['store_alias_over'] = int(tg.group(1)), int(tg.group(2)), int(tg.group(3))
    ev = RE_EVENT.search(txt)
    if ev:
        d['ev_probes'], d['ev_hits'] = int(ev.group(1)), int(ev.group(2))
        d['ev_traces'], d['ev_cand'], d['ev_dedup'] = int(ev.group(3)), int(ev.group(4)), int(ev.group(5))
    mc = RE_MISSCLS.search(txt)
    if mc:
        d['mc_inflight'], d['mc_entry'], d['mc_midtrace'] = int(mc.group(1)), int(mc.group(2)), int(mc.group(3))

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
        d['alt_unused_evicted'] = int(al.group(6)) if al.group(6) else None
        d['alt_wait'] = int(al.group(7)) if al.group(7) else None
        d['alt_lines'] = int(al.group(8)) if al.group(8) else None
        d['alt_line_stalls'] = int(al.group(9)) if al.group(9) else None
    ch = RE_CHAIN.search(txt)
    if ch:
        d['chain_unenc'], d['chain_trunc'] = int(ch.group(1)), int(ch.group(2))
        d['chain_hits'], d['chain_evict'] = int(ch.group(3)), int(ch.group(4))
        d['chain_filtered'] = int(ch.group(5)) if ch.group(5) else None
        d['chain_conf_suppressed'] = int(ch.group(6)) if ch.group(6) else None
        d['chain_evicted_missed'] = int(ch.group(7)) if ch.group(7) else None
        d['chain_evicted_age'] = [int(ch.group(i)) for i in range(8, 13)] if ch.group(7) else None
        d['chain_slack'] = [int(ch.group(i)) for i in range(13, 19)]
    dl = RE_DELTAS.search(txt)
    if dl:
        d['chain_deltas'] = [int(dl.group(i)) for i in range(1, 5)]
        if dl.group(5):
            d['chain_trace_kinds'] = [int(dl.group(i)) for i in range(5, 9)]
        d['chain_far_truncated'] = int(dl.group(9)) if dl.group(9) else None
    fm = RE_FMT.search(txt)
    if fm:
        d['fmt_meta_bits'], d['fmt_short'], d['fmt_long'] = int(fm.group(1)), int(fm.group(2)), int(fm.group(3))
        d['fmt_kind'] = int(fm.group(4)) if fm.group(4) else 2
        d['fmt_far_trunc'] = int(fm.group(5)) if fm.group(5) else 0
    nd = RE_NEED.search(txt)
    if nd:
        d['chain_need_bits'] = [int(nd.group(i)) for i in range(1, 7)]
        d['chain_need_windows'] = [int(nd.group(i)) for i in range(7, 12)]
        d['chain_trunc_windows'] = int(nd.group(12))
    pr = RE_PROMOTE.search(txt)
    if pr:
        d['chain_buf_rehits'] = int(pr.group(1))
    sh = RE_SHADOW.search(txt)
    if sh:
        (d['shadow_mispredicts'], d['shadow_walks'], d['shadow_lines'],
         d['shadow_windows'], d['shadow_hits']) = (int(sh.group(i)) for i in range(1, 6))
    wp = RE_WP.search(txt)
    if wp:
        for i, k in enumerate(WP_KEYS):
            d[k] = int(wp.group(i + 1) or 0)  # level-3 fields are absent in older logs
    wk = RE_WP_KNOB.search(txt)
    if wk:
        d['wrong_path_name'] = wk.group(1)  # 0-3 or 'ucp' (= level 1 + fetch-block rule)
        d['wrong_path'] = {'ucp': 1, 'ucp2': 3}.get(wk.group(1)) or int(wk.group(1))
    pk_ = RE_PEEK.search(txt)
    if pk_:
        d['peek_probes'], d['peek_hits'] = int(pk_.group(1)), int(pk_.group(2))
    rk = RE_RK.search(txt)
    if rk:
        (d['rk_mp'], d['rk_filed'], d['rk_probes'], d['rk_hits'], d['rk_walks'], d['rk_bhits']) = (int(rk.group(i)) for i in range(1, 7))
    cf = RE_CHFOLLOW.search(txt)
    if cf:
        d['chf'] = [int(cf.group(i)) for i in range(1, 12)]
    ck = RE_CHAIN_KNOB.search(txt)
    if ck:
        d['chain_knob'] = (int(ck.group(1)), int(ck.group(2)))
    for sm in RE_SUCC.finditer(txt):
        d['succ_' + ('mp' if sm.group(1) != 'steady' else 'st')] = [int(sm.group(i)) for i in range(2, 11)]
    vp = RE_VP.search(txt)
    if vp:
        for i, k in enumerate(VP_KEYS):
            d[k] = int(vp.group(i + 1))
    vpt = RE_VPT.search(txt)
    if vpt:
        for i, k in enumerate(VPT_KEYS):
            d[k] = int(vpt.group(i + 1))
    vpk = RE_VP_KNOB.search(txt)
    if vpk:
        d['vp_knob'] = (int(vpk.group(1)), int(vpk.group(2)), int(vpk.group(3)), int(vpk.group(4)), float(vpk.group(5)))
    rkk = RE_RK_KNOB.search(txt)
    if rkk:
        d['rk_knob'] = (int(rkk.group(1)), int(rkk.group(2)), int(rkk.group(3)))
    pdg = RE_PDIAG.search(txt)
    if pdg:
        d['pdiag_mp'], d['pdiag_wt'] = int(pdg.group(1)), int(pdg.group(2))
        d['pdiag_keys'] = {m.group(1): (int(m.group(2)), int(m.group(3)), int(m.group(4))) for m in RE_PDIAG_KEY.finditer(txt)}
    sk = RE_SQUASH_KNOB.search(txt)
    if sk:
        d['probe_squash'] = int(sk.group(1))
    pk = RE_PROMOTE_KNOB.search(txt)
    d['buffer_promote'] = int(pk.group(1)) if pk else 1
    db = RE_DBITS.search(txt)
    if db:
        d['chain_delta_bits'] = [int(x) for x in db.group(1).split()]  # index 0 = <=3 bits ... 13 = 16 bits, 14 = >16
    rc = RE_REACH.search(txt)
    if rc:
        d['chain_reach_xfer'] = [int(rc.group(i)) for i in range(1, 7)]
        d['chain_reach_trace'] = [int(rc.group(i)) for i in range(7, 14)]  # none, then 6 reach buckets
    mp = RE_MP.search(txt)
    if mp:
        d['mp'] = [int(mp.group(i)) for i in range(1, 5)]
        mt = RE_MP_TOP.search(txt)
        d['mp_top'] = _floats(mt) if mt else None
        mr = RE_MP_TR.search(txt)
        d['mp_tr'] = _floats(mr) if mr else None
    ok_ = RE_ORACLE_KNOB.search(txt)
    if ok_:
        d['uop_oracle'] = int(ok_.group(1))
    to_ = RE_TORACLE_KNOB.search(txt)
    if to_:
        d['trace_oracle'] = int(to_.group(1))
    fu_ = RE_FUP.search(txt)
    if fu_:
        d['fup_walks'] = int(fu_.group(1))
    skb = RE_SUCP_KB.search(txt)
    if skb:
        d['sucp_kb'] = float(skb.group(1))
    tx_ = RE_TAX.search(txt)
    if tx_:
        d['tax'] = [int(x) for x in tx_.group(1).split()] + [int(tx_.group(2)), int(tx_.group(3))]
    l2_ = RE_L2I.search(txt)
    if l2_:
        d['l2i'] = [int(x) for x in l2_.groups()]
    rp_ = RE_RP.search(txt)
    if rp_:
        d['rp'] = [int(x) for x in rp_.groups()]
    rk_ = RE_RP_KB.search(txt)
    if rk_ and float(rk_.group(1)) > 0:
        d['rp_kb'] = float(rk_.group(1))
    fd_ = RE_FDIP.search(txt)
    if fd_:
        d['fdip'] = [int(fd_.group(1)), int(fd_.group(2)), int(fd_.group(3) or 0)]
    oi_ = RE_OINSTALL_KNOB.search(txt)
    d['oracle_install'] = int(oi_.group(1)) if oi_ else 0
    tda = RE_TD_ARR.search(txt)
    if tda:
        g = tda.groups()
        d['td_arr'] = [int(g[0]), int(g[1]), int(g[2])]  # arrivals, resident, staged
        d['td_arr_miss'] = [int(x) for x in g[3].split()]  # by class
        d['td_probes'] = [int(x) for x in RE_TD_PROBES.search(txt).groups()]
        tst = RE_TD_STAGE.search(txt)
        if tst:
            d['td_stage'] = [int(x) for x in tst.groups()]
        tps = RE_TD_POS.search(txt)
        if tps:
            d['td_pos'] = [int(x) for x in tps.groups()]
        ld = [int(x) for x in RE_TD_LEAD.search(txt).groups()]
        d['td_lead_served'], d['td_lead_missed'] = ld[:6], ld[6:]
        # class lists; the cost lists carry the "other" cycles as one extra last element
        for key, rx in (('td_miss', RE_TD_MISS), ('td_rec', RE_TD_REC), ('td_tight', RE_TD_TIGHT), ('td_upper', RE_TD_UPPER),
                        ('td_head_tight', RE_TD_HT), ('td_head_upper', RE_TD_HU)):
            mt_ = rx.search(txt)
            if mt_:
                d[key] = [int(x) for x in mt_.group(1).split()] + ([int(mt_.group(2))] if mt_.lastindex == 2 else [])
    cbg = RE_COST_BIG.search(txt)
    if cbg:
        d['cost_big'] = _floats(cbg)
    rn = RE_RUN.search(txt)
    if rn:
        d['run_jump'], d['run_full'], d['run_inh'], d['run_stored'] = (int(rn.group(i)) for i in range(1, 5))
    sp = RE_SPLIT.search(txt)
    if sp:
        d['split_cont'], d['split_inh'], d['split_stored'] = int(sp.group(1)), int(sp.group(2)), int(sp.group(3))
    mo = RE_MPOPEN.search(txt)
    if mo:
        d['stall_mp_opened'], d['stall_mp_traces'] = int(mo.group(1)), int(mo.group(2))
    ld = RE_LEAD.search(txt)
    if ld:
        d['stall_lead_keyed'], d['stall_lead_self'], d['stall_lead_traces'], d['stall_lead_conflicts'] = (int(ld.group(i)) for i in range(1, 5))
        d['stall_lead_dist'] = [int(ld.group(i)) for i in range(5, 9)]
    ra = RE_READMIT.search(txt)
    if ra:
        d['stall_lost_recap'], d['stall_readmits'] = int(ra.group(1)), int(ra.group(2))
    sv = RE_STARVE.search(txt)
    if sv:
        d['stall_starve_offset'] = [int(x) for x in sv.group(1).split()]
    up2 = RE_UCPPORT.search(txt)
    if up2:
        (d['ucpp_cond'], d['ucpp_cond_miss'], d['ucpp_marked'], d['ucpp_marked_miss'], d['ucpp_paths'],
         d['ucpp_stop_sat'], d['ucpp_stop_ind'], d['ucpp_stop_btbmiss'], d['ucpp_stop_maxip']) = (int(up2.group(i)) for i in range(1, 10))
        d['ucpp_ind_walked'] = int(up2.group(10)) if up2.group(10) else 0
        d['ucpp_bank_conf'] = int(up2.group(11)) if up2.group(11) else 0
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


_STRIP_EXTS = ('.out', '.txt', '.log', '.gz', '.xz', '.champsimtrace', '.champsim', '.trace',
               '_ap')  # cvp1ap runs suffix the trace name with _ap; subset lists don't


def canon_name(name):
    """Canonical simpoint name: drop any trailing run/trace/compression suffixes
    so include lists, output files, and baseline files all key the same way."""
    changed = True
    while changed:
        changed = False
        for ext in _STRIP_EXTS:
            if name.endswith(ext) and len(name) > len(ext):
                name = name[:-len(ext)]
                changed = True
    return name


def load_rows(path, pattern, include):
    files = sorted(glob.glob(os.path.join(path, pattern)))
    rows, skipped, inc_skipped = [], 0, 0
    for fp in files:
        name = canon_name(os.path.basename(fp))
        if include is not None and name not in include:
            inc_skipped += 1
            continue
        d = parse_file(fp)
        if d is None:
            skipped += 1
            continue
        d['name'] = name
        rows.append(d)
    return rows, skipped, inc_skipped


def aggregate(path, pattern, include, baseline=None, scatter=False):
    rows, skipped, inc_skipped = load_rows(path, pattern, include)

    print(f"\n=== {path} ===")
    if include is not None:
        print(f"[include] kept {len(rows)} simpoint(s), skipped {inc_skipped} not in list")
        if not rows and inc_skipped:
            files = sorted(glob.glob(os.path.join(path, pattern)))[:3]
            print("[include] NO MATCHES - derived file names look like:")
            for fp in files:
                print(f"[include]   file: {os.path.basename(fp)!r} -> {canon_name(os.path.basename(fp))!r}")
            for n in sorted(include)[:3]:
                print(f"[include]   list: {n!r}")
    print(f"simpoints: {len(rows)}  (skipped {skipped} with no ROI IPC)")
    if not rows:
        return

    print(f"IPC:          arith-mean {amean([r['ipc'] for r in rows]):.4f}   geomean {gmean([r['ipc'] for r in rows]):.4f}")
    print(f"hit rate:     {amean([r['hit'] for r in rows]):.2f}%")
    print(f"switch MPKI:  {amean([r['switch_mpki'] for r in rows]):.3f}")
    mm = [r for r in rows if r.get('miss_total') and r.get('instrs')]
    if mm:
        print(f"muop-$ MPKI:  {amean([1000.0 * r['miss_total'] / r['instrs'] for r in mm]):.1f}")
    l1 = [r for r in rows if r.get('l1i_miss') is not None and r.get('instrs')]
    if l1:
        print(f"L1I MPKI:     {amean([1000.0 * r['l1i_miss'] / r['instrs'] for r in l1]):.3f}   (incl. walk lines)")
    dl = [r for r in rows if r.get('dl1_steady') is not None and r.get('instrs')]
    if dl:
        # demand-only: fetch lines that missed L1I, and how long they waited (the latency a line prefetch hides)
        s_, r_ = amean([r['dl1_steady'] for r in dl]), amean([r['dl1_recovery'] for r in dl])
        ws, wr = amean([r['dl1_wait_steady'] for r in dl]), amean([r['dl1_wait_recovery'] for r in dl])
        print(f"demand L1I misses: steady {s_:,.0f} lines (avg wait {ws / s_ if s_ else float('nan'):.0f} cyc, total {ws:,.0f})   "
              f"recovery {r_:,.0f} lines (avg wait {wr / r_ if r_ else float('nan'):.0f} cyc, total {wr:,.0f})")

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

    # u-op-cache miss-cost profile (PROMETHEUS_MISS_PROFILE / _MISS_DUMP): windows ranked by the empty-ROB
    # cycles charged to their misses, and how much of that the stall segmenter's traces contain
    have_mp = [r for r in rows if r.get('mp')]
    if have_mp:
        w, ms, c, o = (amean([r['mp'][i] for r in have_mp]) for i in range(4))
        print(f"\n--- u-op-cache miss-cost profile ---")
        print(f"missing windows:   {w:,.0f}   misses {ms:,.0f}   empty-ROB cycles charged to a miss {c:,.0f}   (other empty-ROB cycles {o:,.0f})")
        tops = [r['mp_top'] for r in have_mp if r.get('mp_top')]
        if tops:
            v = [amean([x[i] for x in tops]) for i in range(8)]
            print(f"cost held by the costliest      1K {v[0]:.1f}%  2K {v[1]:.1f}%  4K {v[2]:.1f}%  8K {v[3]:.1f}%   windows")
            print(f"cost held by the most-missing   1K {v[4]:.1f}%  2K {v[5]:.1f}%  4K {v[6]:.1f}%  8K {v[7]:.1f}%   windows")
        trs = [r['mp_tr'] for r in have_mp if r.get('mp_tr')]
        if trs:
            v = [amean([x[i] for x in trs]) for i in range(5)]
            print(f"in segmenter traces: all cost {v[0]:.1f}%   of the costliest 1K {v[1]:.1f}%  2K {v[2]:.1f}%  4K {v[3]:.1f}%  8K {v[4]:.1f}%")
    have_or = [r for r in rows if r.get('uop_oracle')]
    if have_or:
        have_to = [r for r in have_or if r.get('trace_oracle')]
        how = "filled on demand, taking u-op-cache capacity" if have_or[0].get('oracle_install') else "hit outside the u-op cache"
        if have_to:
            # trace oracle: every window of the N costliest segmenter traces always hits
            print(f"trace oracle:      {amean([r['trace_oracle'] for r in have_to]):,.0f} traces "
                  f"({amean([r['uop_oracle'] for r in have_to]):,.0f} windows) always hit ({how})")
        else:
            print(f"u-op-cache oracle: {amean([r['uop_oracle'] for r in have_or]):,.0f} windows always hit ({how})")

    have_fu = [r for r in rows if r.get('fup_walks')]
    if have_fu:
        print(f"u-op FDIP:         {amean([r['fup_walks'] for r in have_fu]):,.0f} single-window walks launched at the cursor")
    have_kb = [r for r in rows if r.get('sucp_kb')]
    if have_kb:
        print(f"SUCP storage (est.): {amean([r['sucp_kb'] for r in have_kb]):.1f} KB")
    have_tax = [r for r in rows if r.get('tax')]
    if have_tax:
        # u-op-miss starve cycles (ROB empty), summed over workloads, by first touch and by the level that served the bytes
        tot = [sum(r['tax'][i] for r in have_tax) for i in range(22)]
        S = max(sum(tot), 1)
        cyc = sum(r['cycles'] for r in have_tax)
        seen = [tot[i] + tot[10 + i] for i in range(5)]
        first = sum(tot[5:10]) + sum(tot[15:20])
        rec = sum(tot[10:20])
        print(f"stall taxonomy:    u-op-miss empty-ROB cycles {100 * S / max(cyc, 1):.1f}% of all cycles  (recovery {100 * rec / S:.0f}% of them)")
        print(f"  code seen before: L1I {100 * seen[0] / S:.1f}%  L2 {100 * seen[1] / S:.1f}%  LLC {100 * seen[2] / S:.1f}%  DRAM {100 * seen[3] / S:.1f}%   "
              f"| first-touch {100 * first / S:.1f}% | redirect bubble {100 * tot[20] / S:.1f}%  u-op hit in flight {100 * tot[21] / S:.1f}%")
    have_l2i = [r for r in rows if r.get('l2i')]
    if have_l2i:
        v = [amean([r['l2i'][i] for r in have_l2i]) for i in range(5)]
        print(f"L2 instruction:    perfect-hits {v[0]:,.0f}   EMISSARY mark events {v[1]:,.0f}  L2 lines marked {v[2]:,.0f}  evictions diverted {v[3]:,.0f}  high-priority evicted {v[4]:,.0f}")
    have_rp = [r for r in rows if r.get('rp')]
    if have_rp:
        v = [amean([r['rp'][i] for r in have_rp]) for i in range(5)]
        kb = [r['rp_kb'] for r in have_rp if r.get('rp_kb')]
        print(f"resteer prefetch:  lookups {v[0]:,.0f}  trigger hits {v[1]:,.0f} ({pct(v[1], v[0]):.1f}%)  L1I prefetches {v[2]:,.0f}  dropped {v[3]:,.0f}  "
              f"filed {v[4]:,.0f}" + (f"  table {amean(kb):.1f} KB" if kb else ''))
    have_fd = [r for r in rows if r.get('fdip')]
    if have_fd:
        print(f"FDIP (cursor):     L1I line prefetches issued {amean([r['fdip'][0] for r in have_fd]):,.0f}   "
              f"dropped (prefetch queue full) {amean([r['fdip'][1] for r in have_fd]):,.0f}   wrong-path {amean([r['fdip'][2] for r in have_fd]):,.0f}")

    # trace_trigger_diag: why stored traces did not serve their misses (cost-weighted)
    have_td = [r for r in rows if r.get('td_arr')]
    if have_td:
        print("\n--- trigger diagnostic: why stored traces did not serve ---")
        pr = [amean([r['td_probes'][i] for r in have_td]) for i in range(5)]
        print(f"cursor probes at stored entries: launched {pr[4]:,.0f}   already walking {pr[3]:,.0f}   "
              f"resident (no walk) {pr[0]:,.0f}   dropped: walk cap {pr[2]:,.0f} / confidence {pr[1]:,.0f}")
        ncls = min(len(r['td_miss']) for r in have_td)
        arr = [amean([r['td_arr'][i] for r in have_td]) for i in range(3)]
        am = [amean([r['td_arr_miss'][i] for r in have_td]) for i in range(ncls)]
        miss_arr = sum(am)
        print(f"demand at a stored entry: {arr[0]:,.0f}   already resident {pct(arr[1], arr[0]):.1f}%   "
              f"served by the walk {pct(arr[2], arr[0]):.1f}%   missed {pct(miss_arr, arr[0]):.1f}%")
        print("  missed because:   " + "  ".join(f"{TD_LABELS[i]} {pct(am[i], miss_arr):.1f}%" for i in range(1, ncls) if am[i] > 0))
        ls_ = [amean([r['td_lead_served'][i] for r in have_td]) for i in range(6)]
        lm_ = [amean([r['td_lead_missed'][i] for r in have_td]) for i in range(6)]
        lb = ['<=8', '<=16', '<=32', '<=64', '<=128', '>128']
        print("  probe->demand lead, walked entries served: " + "  ".join(f"{l}={pct(v, sum(ls_)):.0f}%" for l, v in zip(lb, ls_))
              + f"   [{sum(ls_):,.0f}]")
        print("  probe->demand lead, walked entries missed: " + "  ".join(f"{l}={pct(v, sum(lm_)):.0f}%" for l, v in zip(lb, lm_))
              + f"   [{sum(lm_):,.0f}]")
        hs = [r for r in have_td if r.get('td_stage')]
        if hs:
            st_ = [amean([r['td_stage'][i] for r in hs]) for i in range(4)]
            unused = st_[2] + st_[3]
            print(f"staged windows:    {st_[0]:,.0f}   staged after demand had already missed them (late) {pct(st_[1], st_[0]):.1f}%   "
                  f"evicted unused {pct(unused, st_[0]):.1f}% = late {pct(st_[2], st_[0]):.1f}% + on time but never demanded {pct(st_[3], st_[0]):.1f}%")
        hp = [r for r in have_td if r.get('td_pos')]
        if hp:
            pv = [amean([r['td_pos'][i] for r in hp]) for i in range(10)]
            print("  used share by position in the walk: " + "  ".join(f"{l}={pct(pv[i], pv[i] + pv[5 + i]):.0f}% of {pv[i] + pv[5 + i]:,.0f}"
                                                                  for i, l in enumerate(['1st', '2nd', '3rd', '4th', '5th+'])))
        # cost views: charged to the class of the miss dispatch waits for, or of the FIRST miss of its miss run
        for key, lbl in (('td_miss', 'u-op-cache misses'), ('td_tight', 'empty-ROB cyc, waited-for miss'),
                         ('td_head_tight', 'empty-ROB cyc, run-head miss'), ('td_upper', 'starve cyc, waited-for miss'),
                         ('td_head_upper', 'starve cyc, run-head miss')):
            hk = [r for r in have_td if r.get(key)]
            if not hk:
                continue
            v = [amean([r[key][i] for r in hk]) for i in range(len(hk[0][key]))]
            tot = sum(v)
            # classes that never occur are left out (e.g. 'uncovered' once the window history splits it)
            parts = [f"{TD_LABELS[i]} {pct(v[i], tot):.1f}%" for i in range(ncls) if v[i] > 0]
            if len(v) > ncls:
                parts.append(f"hit/other {pct(v[ncls], tot):.1f}%")
            print(f"{lbl + ':':32s}" + "  ".join(parts) + f"   [{tot:,.0f}]")
        hr = [r for r in have_td if r.get('td_rec')]
        if hr:
            mi = [amean([r['td_miss'][i] for r in hr]) for i in range(ncls)]
            rc = [amean([r['td_rec'][i] for r in hr]) for i in range(ncls)]
            print(f"{'share in recovery (per cause):':32s}" + "  ".join(f"{TD_LABELS[i]} {pct(rc[i], mi[i]):.0f}%" for i in range(1, ncls) if mi[i] > 0)
                  + f"   [all misses {pct(sum(rc), sum(mi)):.0f}%]")

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
        bc = amean([r.get('ucpp_bank_conf', 0) for r in have_up])
        if bc > 0:
            print(f"BTB banking:       {bc:,.0f} walk-install stalls on demand bank conflicts")

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
        md = [r for r in have_stall if r.get('stall_marks_dropped') is not None]
        if md:
            md_abs = amean([r['stall_marks_dropped'] for r in md])
            md_den = [r for r in md if r.get('ti_total')]
            md_pct = amean([pct(r['stall_marks_dropped'], r['ti_total']) for r in md_den]) if md_den else float('nan')
            print(f"marks dropped:     {md_abs:,.0f} tight-stall cycles with no open stretch ({md_pct:.1f}% of tight cycles)")
        ras = [r for r in have_stall if r.get('stall_lost_recap') is not None]
        if ras:
            lr = amean([r['stall_lost_recap'] for r in ras])
            rd = amean([r['stall_readmits'] for r in ras])
            dd = amean([r['stall_dedup'] for r in ras])
            print(f"evicted re-captures: {lr:,.0f} costly re-captures of evicted traces ({pct(lr, dd):.1f}% of re-captures)   re-admitted {rd:,.0f}")
        rns = [r for r in have_stall if r.get('run_jump') or r.get('run_full')]
        if rns:
            # trace_run_format: traces are sequential-window runs, cut at jumps
            print(f"run format:        cut at a jump {amean([r['run_jump'] for r in rns]):,.0f}   at the window limit {amean([r['run_full'] for r in rns]):,.0f}   "
                  f"inherited costly {amean([r['run_inh'] for r in rns]):,.0f}   continuation traces stored {amean([r['run_stored'] for r in rns]):,.0f} "
                  f"({pct(amean([r['run_stored'] for r in rns]), amean([r['stall_traces'] for r in rns])):.1f}% of stored traces)")
        sps = [r for r in have_stall if r.get('split_cont') and not r.get('run_jump')]
        if sps:
            # trace_stall_split: miss runs past the depth continue as chained segments
            print(f"split segments:    {amean([r['split_cont'] for r in sps]):,.0f} continuations   "
                  f"{amean([r['split_inh'] for r in sps]):,.0f} inherited the run's costly verdict   "
                  f"stored {amean([r['split_stored'] for r in sps]):,.0f} traces "
                  f"({pct(amean([r['split_stored'] for r in sps]), amean([r['stall_traces'] for r in sps])):.1f}% of stored traces)")
        mos = [r for r in have_stall if r.get('stall_mp_opened')]
        if mos:
            # trace_stall_mispredict_open: stretches opened at mispredicted branches (recorded correct path)
            print(f"mispredict-opened: {amean([r['stall_mp_opened'] for r in mos]):,.0f} stretches   "
                  f"stored {amean([r['stall_mp_traces'] for r in mos]):,.0f} traces "
                  f"({pct(amean([r['stall_mp_traces'] for r in mos]), amean([r['stall_traces'] for r in mos])):.1f}% of stored traces)")
        lds = [r for r in have_stall if r.get('stall_lead_keyed')]
        if lds:
            # trace_stall_trigger_lead: stretches stored under the pc checked L cycles before them
            lk, ls = amean([r['stall_lead_keyed'] for r in lds]), amean([r['stall_lead_self'] for r in lds])
            lt = amean([r['stall_lead_traces'] for r in lds])
            dist = [amean([r['stall_lead_dist'][i] for r in lds]) for i in range(4)]
            print(f"lead-keyed: {lk:,.0f} stretches ({pct(lk, lk + ls):.1f}% had a key that old)   stored {lt:,.0f} traces "
                  f"({pct(lt, amean([r['stall_traces'] for r in lds])):.1f}% of stored)   "
                  f"conflicts {amean([r['stall_lead_conflicts'] for r in lds]):,.0f} re-captures   "
                  f"key-to-entry <=4K/64K/1M/far: {' / '.join(f'{pct(x, sum(dist)):.0f}%' for x in dist)}")
        svs = [r['stall_starve_offset'] for r in have_stall if r.get('stall_starve_offset') is not None]
        if svs:
            so = [amean([x[i] for x in svs]) for i in range(9)]
            stot = sum(so)
            if stot > 0:
                # where in a costly stretch dispatch first starves: win0 = the entry window; before = an
                # earlier stretch's instructions starved dispatch; after = the closing hit or later
                lbls = ['win0', 'win1', 'win2', 'win3', 'win4-7', 'win8+', 'past-depth', 'before', 'after']
                print("first starve at:   " + "  ".join(f"{l}={100 * v / stot:.1f}%" for l, v in zip(lbls, so))
                      + f"   [{stot:,.0f} costly stretches]")
        print(f"stored u-ops:      {amean([r['stall_stored_uops'] for r in have_stall]):,.0f}")
        have_store = [r for r in have_stall if 'store_evict' in r]
        if have_store:
            ev = amean([r['store_evict'] for r in have_store])
            cf = amean([pct(r['store_conflict'], r['store_evict']) for r in have_store if r['store_evict']])
            print(f"store:             occupancy {amean([r['store_occ'] for r in have_store]):,.0f}   "
                  f"evictions {ev:,.0f} ({cf:.1f}% conflict = while underfull)")
            have_ov = [r for r in have_store if r.get('store_win')]
            if have_ov:
                ovp = amean([pct(r['store_shared'], r['store_win']) for r in have_ov if r['store_win']])
                print(f"fragmentation:     windows inserted {amean([r['store_win'] for r in have_ov]):,.0f}   "
                      f"already in another trace {amean([r['store_shared'] for r in have_ov]):,.0f} ({ovp:.1f}% overlap)")
        tgs = [r for r in have_stall if r.get('store_alias_hits')]
        if tgs:
            lk = amean([r['store_lookups'] for r in tgs])
            ah = amean([r['store_alias_hits'] for r in tgs])
            ao = amean([r['store_alias_over'] for r in tgs])
            print(f"partial tags:      alias hits {ah:,.0f} of {lk:,.0f} probes ({100 * ah / lk:.4f}%)   alias overwrites {ao:,.0f}")
        have_mc = [r for r in have_stall if 'mc_midtrace' in r and r.get('miss_total')]
        if have_mc:
            infl = amean([pct(r['mc_inflight'], r['miss_total']) for r in have_mc])
            ent = amean([pct(r['mc_entry'], r['miss_total']) for r in have_mc])
            mid = amean([pct(r['mc_midtrace'], r['miss_total']) for r in have_mc])
            print(f"miss classes:      in-flight {amean([r['mc_inflight'] for r in have_mc]):,.0f} ({infl:.1f}%)   "
                  f"entry {amean([r['mc_entry'] for r in have_mc]):,.0f} ({ent:.1f}%)   "
                  f"MID-TRACE {amean([r['mc_midtrace'] for r in have_mc]):,.0f} ({mid:.1f}%)   [% of all muop misses]")
        have_ev = [r for r in have_stall if r.get('ev_probes', 0) > 0]
        if have_ev:
            hitp = amean([pct(r['ev_hits'], r['ev_probes']) for r in have_ev if r['ev_probes']])
            print(f"mispredict trig:   probes {amean([r['ev_probes'] for r in have_ev]):,.0f}   "
                  f"walk-hits {amean([r['ev_hits'] for r in have_ev]):,.0f} ({hitp:.1f}%)   "
                  f"event-traces {amean([r['ev_traces'] for r in have_ev]):,.0f} of {amean([r['ev_cand'] for r in have_ev]):,.0f} "
                  f"(dedup {amean([r['ev_dedup'] for r in have_ev]):,.0f})")
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
            cbs = [r['cost_big'] for r in have_stall if r.get('cost_big')]
            if cbs:
                print("  cum% stall-cyc:  " + "  ".join(f"{n}={amean([x[i] for x in cbs]):.1f}%" for i, n in enumerate([2048, 4096, 8192])))
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
        unused = [r['alt_unused_evicted'] for r in have_alt if r.get('alt_unused_evicted') is not None]
        if unused:
            # accuracy over COMPLETED window lifetimes: useful / (useful + evicted-unused);
            # windows still resident (bit set, not yet hit or evicted) are excluded
            lacc = amean([pct(r['alt_useful'], r['alt_useful'] + r['alt_unused_evicted']) for r in have_alt
                          if r.get('alt_useful') is not None and r.get('alt_unused_evicted') is not None
                          and (r['alt_useful'] + r['alt_unused_evicted'])])
            print(f"unused evicted:    {amean(unused):,.0f}   (accuracy {lacc:.1f}% of completed window lifetimes)")
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
            mode = have_chain[0].get('buffer_promote', 1)
            if mode != 1:
                rh = [r['chain_buf_rehits'] for r in have_chain if r.get('chain_buf_rehits') is not None]
                desc = {0: "no u-op-cache fill, buffer serves repeats", 2: "fill at LRU"}[mode]
                line = f"buffer promote:    mode {mode} ({desc})"
                if mode == 0 and rh:
                    line += f"   re-hits {amean(rh):,.0f} of {amean([r['chain_hits'] for r in have_chain]):,.0f} buffer hits"
                print(line)
            shs = [r for r in have_chain if r.get('shadow_walks') is not None]
            if shs:
                # deep probe past a mispredicted branch: the supply there is the correct path a real FTQ
                # would not hold yet.  Squash 0 = legacy (flagged; their buffer hits are oracle benefit),
                # 1 = squashed at the misprediction (lines already fetched stay in the L1I)
                sq = shs[0].get('probe_squash', 0)
                w_ = amean([r['shadow_walks'] for r in shs])
                verb = "squashed" if sq == 1 else "flagged"
                line = (f"past-mispredict:   {w_:,.0f} walks {verb} ({pct(w_, amean([r['alt_triggers'] for r in shs])):.1f}% of walks)   "
                        f"lines {amean([r['shadow_lines'] for r in shs]):,.0f}   staged windows {amean([r['shadow_windows'] for r in shs]):,.0f}")
                if sq == 0:
                    sh_h = amean([r['shadow_hits'] for r in shs])
                    line += f"   buffer hits {sh_h:,.0f} ({pct(sh_h, amean([r['chain_hits'] for r in shs])):.1f}% of buffer hits = oracle)"
                print(line)
            filt = [r['chain_filtered'] for r in have_chain if r.get('chain_filtered') is not None]
            if filt and amean(filt) > 0:
                print(f"chain filtered:    {amean(filt):,.0f} windows already resident (not staged)")
            sup = [r['chain_conf_suppressed'] for r in have_chain if r.get('chain_conf_suppressed') is not None]
            if sup and amean(sup) > 0:
                launched = amean([r['alt_triggers'] for r in have_chain])
                print(f"conf suppressed:   {amean(sup):,.0f} walk launches withheld by launch confidence "
                      f"({pct(amean(sup), amean(sup) + launched):.1f}% of would-be launches)")
            etm = [r for r in have_chain if r.get('chain_evicted_missed') is not None]
            if etm:
                n_etm = amean([r['chain_evicted_missed'] for r in etm])
                share = amean([pct(r['chain_evicted_missed'], r['chain_evict']) for r in etm if r['chain_evict']])
                ages = [amean([r['chain_evicted_age'][i] for r in etm]) for i in range(5)]
                tot = sum(ages)
                # short ages (<= a few K cycles) = what a larger FIFO would have held; long ages = the
                # window's NEXT recurrence (a re-walk's job, not the buffer's)
                dist = "  ".join(f"{lbl}={100 * v / tot:.0f}%" for lbl, v in zip(['<=256', '<=1K', '<=4K', '<=16K', '>16K'], ages)) if tot else "-"
                print(f"evicted->missed:   {n_etm:,.0f} demand misses on windows the buffer evicted unused "
                      f"({share:.1f}% of evicted-unused); eviction->miss age: {dist}")
            dls = [r['chain_deltas'] for r in have_chain if r.get('chain_deltas') is not None]
            if dls:
                mix = [amean([x[i] for x in dls]) for i in range(4)]
                tot = sum(mix)
                if tot > 0:
                    # packed-manifest record costs from the run's own format (kind + delta widths)
                    r0 = have_chain[0]
                    kb, sb, lb = r0.get('fmt_kind', 2), r0.get('fmt_short', 8), r0.get('fmt_long', 16)
                    if not r0.get('fmt_meta_bits'):
                        kb, sb, lb = 2, 8, 16  # legacy run: price as the default packed format
                    avg_bits = (kb * mix[0] + (kb + sb) * mix[1] + (kb + lb) * mix[2]) / max(1e-9, mix[0] + mix[1] + mix[2])
                    ftr = [r['chain_far_truncated'] for r in have_chain if r.get('chain_far_truncated') is not None]
                    far_note = f"truncated at, prefix kept: {amean(ftr):,.0f} traces" if ftr and amean(ftr) > 0 else "rejected"
                    print(f"delta mix:         seq {100 * mix[0] / tot:.0f}%  short {100 * mix[1] / tot:.0f}%  long {100 * mix[2] / tot:.0f}%  "
                          f"far {100 * mix[3] / tot:.1f}% ({far_note})   -> packed avg {avg_bits:.1f} bits/transfer "
                          f"(kind {kb}b, short {sb}b, " + (f"long {lb}b" if kb == 2 and lb > 0 else "no long class") + ") vs 16 flat")
            tks = [r['chain_trace_kinds'] for r in have_chain if r.get('chain_trace_kinds') is not None]
            if tks:
                kinds = [amean([x[i] for x in tks]) for i in range(4)]
                ktot = sum(kinds)
                if ktot > 0:
                    print(f"trace kinds:       single-window {100 * kinds[0] / ktot:.0f}%  sequential-only {100 * kinds[1] / ktot:.0f}%  "
                          f"with short jump {100 * kinds[2] / ktot:.0f}%  with long jump {100 * kinds[3] / ktot:.0f}%   [of stored manifests]")
            nds = [r for r in have_chain if r.get('chain_need_bits') is not None]
            if nds:
                nb = [amean([r['chain_need_bits'][i] for r in nds]) for i in range(6)]
                nw = [amean([r['chain_need_windows'][i] for r in nds]) for i in range(5)]
                tb, tw = sum(nb), sum(nw)
                if tb > 0:
                    # cumulative: share of captured stretches that a slot field of N bits stores WHOLE
                    cum = 0.0
                    parts = []
                    for lbl, v in zip(['16', '24', '32', '48', '64'], nb[:5]):
                        cum += v
                        parts.append(f"{lbl}b={100 * cum / tb:.0f}%")
                    print("stored whole by field width (cum):  " + "  ".join(parts) + f"  >64b={100 * nb[5] / tb:.0f}%")
                if tw > 0:
                    cum = 0.0
                    parts = []
                    for lbl, v in zip(['<=2', '<=3', '<=4', '<=6'], nw[:4]):
                        cum += v
                        parts.append(f"{lbl}={100 * cum / tw:.0f}%")
                    print("stretch windows (cum, depth-capped): " + "  ".join(parts) + f"  >6={100 * nw[4] / tw:.0f}%")
                tw_lost = amean([r['chain_trunc_windows'] for r in nds])
                trunc_n = amean([r['chain_trunc'] for r in nds])
                if trunc_n > 0:
                    print(f"truncation loss:   {tw_lost:,.0f} windows dropped from {trunc_n:,.0f} truncated traces ({tw_lost / trunc_n:.1f} per truncated trace)")
            rcs = [r for r in have_chain if r.get('chain_reach_trace') is not None]
            if rcs:
                rt = [amean([r['chain_reach_trace'][i] for r in rcs]) for i in range(7)]
                rx = [amean([r['chain_reach_xfer'][i] for r in rcs]) for i in range(6)]
                ttot, xtot = sum(rt), sum(rx)
                if ttot > 0:
                    # cumulative share of captured stretches kept WHOLE if the widest delta were N
                    # (rt[0] = no taken transfer at all, always kept)
                    cum = rt[0]
                    parts = []
                    for lbl, v in zip(['4KB(8b)', '128KB(13b)', '256KB(14b)', '512KB(15b)', '1MB(16b)'], rt[1:6]):
                        cum += v
                        parts.append(f"{lbl}={100 * cum / ttot:.1f}%")
                    print("kept by max delta reach (cum):  " + "  ".join(parts) + f"   [{100 * rt[0] / ttot:.0f}% have no jump; {100 * rt[6] / ttot:.1f}% beyond 1MB]")
                if xtot > 0:
                    print("taken transfers by |delta|:     " + "  ".join(f"{lbl}={100 * v / xtot:.1f}%" for lbl, v in
                                                                      zip(['<=4KB', '<=128KB', '<=256KB', '<=512KB', '<=1MB', '>1MB'], rx)))
            dbs = [r['chain_delta_bits'] for r in have_chain if r.get('chain_delta_bits') is not None]
            if dbs:
                hb = [amean([x[i] for x in dbs]) for i in range(15)]  # hb[k] = transfers needing (k+3) bits (k=0: <=3, k=14: >16)
                tot = sum(hb)
                if tot > 0:
                    # within +-4KB (<= 8 signed bits): distribution by exact width (32B windows: n bits = +-2^(n+4) B)
                    short_tot = sum(hb[:6])
                    if short_tot > 0:
                        cum, parts = 0.0, []
                        for k, lbl in enumerate(['3b/128B', '4b/256B', '5b/512B', '6b/1KB', '7b/2KB', '8b/4KB']):
                            cum += hb[k]
                            parts.append(f"{lbl}={100 * cum / short_tot:.0f}%")
                        print(f"jumps <=4KB by bits (cum):   " + "  ".join(parts) + f"   [{100 * short_tot / tot:.0f}% of taken jumps]")
                    cum, parts = 0.0, []
                    for n in range(3, 17):
                        cum += hb[n - 3]
                        if n in (6, 8, 10, 12, 13, 14, 15, 16):
                            parts.append(f"{n}b={100 * cum / tot:.1f}%")
                    print(f"all taken jumps by bits (cum): " + "  ".join(parts) + f"   >16b={100 * hb[14] / tot:.1f}%")
            sl = [amean([r['chain_slack'][i] for r in have_chain]) for i in range(6)]
            tot = sum(sl)
            if tot > 0:
                print("chain slack (probe->demand):  " + "  ".join(f"{lbl}={100*v/tot:.0f}%" for lbl, v in zip(['<=4', '<=8', '<=16', '<=32', '<=64', '>64'], sl)))

    # trace_wrong_path: 1 = cursor stops at mispredicts, 2 = + synthetic wrong path holding
    # BTB banks / decoders, 3 = + SUCP/UCP prefetching from the wrong path (flushed at resolution)
    have_wp = [r for r in rows if r.get('wp_addresses') is not None and r.get('wrong_path', 0) >= 1]
    if have_wp:
        lvl = have_wp[0].get('wrong_path_name', have_wp[0]['wrong_path'])
        print(f"\n--- wrong path (trace_wrong_path {lvl}) ---")
        m = {k: amean([r[k] for r in have_wp]) for k in WP_KEYS}
        if m['wp_cursor_stops'] > 0:
            print(f"cursor stops:      {m['wp_cursor_stops']:,.0f} mispredictions seen at the cursor (no probing past them)")
        if m['wp_shadows'] > 0:
            cyc = [pct(r['wp_decoder_busy'], r['cycles']) for r in have_wp if r.get('cycles')]
            print(f"wrong paths:       {m['wp_shadows']:,.0f}   addresses {m['wp_addresses']:,.0f}   fetched {m['wp_fetched']:,.0f} "
                  f"({pct(m['wp_uop_misses'], m['wp_fetched']):.1f}% u-op-cache misses)")
            print(f"decoders held:     {m['wp_decoder_busy']:,.0f} cycles ({pct(m['wp_decoder_busy'], m['wp_fetch_cycles']):.1f}% of wrong-path cycles, "
                  f"{amean(cyc) if cyc else float('nan'):.2f}% of runtime) -- walks could not pre-decode")
        if m['wp_l1i_lines'] > 0:
            # level 3: wrong-path fetch requests lines, waits on them, and stops at a full backend
            print(f"wrong-path fetch:  {m['wp_l1i_lines']:,.0f} L1I lines   {pct(m['wp_fetched'], m['wp_shadows']) / 100:,.0f} instrs per wrong path   "
                  f"waiting on a line {pct(m['wp_line_stalls'], m['wp_fetch_cycles']):.1f}% / backend full {pct(m['wp_full'], m['wp_fetch_cycles']):.1f}% of wrong-path cycles")
        if m['wp_stops'] > 0 or m['wp_uop_fills'] > 0:
            # ucp2: exact wrong path -- how often it ran into code the trace never executed
            print(f"exact wrong path:  {pct(m['wp_stops'], m['wp_shadows']):.1f}% of wrong paths ended at unseen code / unknown target   "
                  f"u-op-cache fills {m['wp_uop_fills']:,.0f} (wrong-path pollution)")
        if m['wp_walks'] > 0 or m['wp_store_hits'] > 0 or m['wp_ucp_triggers'] > 0:
            print(f"wrong-path walks:  {m['wp_walks']:,.0f} (store hits {m['wp_store_hits']:,.0f}, UCP H2P {m['wp_ucp_triggers']:,.0f})   "
                  f"flushed {m['wp_flushed']:,.0f}   lines {m['wp_lines']:,.0f}   staged {m['wp_staged']:,.0f}   buffer hits {m['wp_buffer_hits']:,.0f}")

    # trace_oracle_peek (diagnostic ceiling) and trace_peek_diag (next-trace predictability at mispredictions)
    have_pk = [r for r in rows if r.get('peek_probes', 0) > 0]
    if have_pk:
        print(f"\n--- oracle peek (diagnostic ceiling) ---")
        print(f"peek probes:       {amean([r['peek_probes'] for r in have_pk]):,.0f}   store hits {amean([r['peek_hits'] for r in have_pk]):,.0f}   "
              f"(walk use: see past-mispredict row)")
    have_rk = [r for r in rows if r.get('rk_probes', 0) > 0]
    if have_rk:
        kn = have_rk[0].get('rk_knob', (1, 0, 0))
        m = {k: amean([r[k] for r in have_rk]) for k in ('rk_mp', 'rk_filed', 'rk_probes', 'rk_hits', 'rk_walks', 'rk_bhits')}
        print(f"\n--- recovery key (ghr {kn[1]} bits, h2p gate {kn[2]}) ---")
        print(f"filed:             {m['rk_filed']:,.0f} of {m['rk_mp']:,.0f} mispredictions ({pct(m['rk_filed'], m['rk_mp']):.1f}%) had a stored trace after them")
        print(f"probes:            {m['rk_probes']:,.0f}   hits {m['rk_hits']:,.0f}   walks launched {m['rk_walks']:,.0f}   "
              f"buffer hits {m['rk_bhits']:,.0f} ({m['rk_bhits'] / m['rk_walks'] if m['rk_walks'] else float('nan'):.2f} per walk)")
    have_cf = [r for r in rows if r.get('chf') is not None and r['chf'][1] > 0]
    if have_cf:
        kn = have_cf[0].get('chain_knob', (0, 0))
        v = [amean([r['chf'][i] for r in have_cf]) for i in range(11)]
        print(f"\n--- trace chaining (depth {kn[0]}, window {kn[1] if kn[1] else 'any'}) ---")
        print(f"successor links:   {v[0]:,.0f} written   hops {v[1]:,.0f}   successor stored {v[2]:,.0f} ({pct(v[2], v[1]):.1f}%)")
        print(f"walks by hop:      " + "  ".join(f"{h}: {v[3 + i]:,.0f}" for i, h in enumerate(['1', '2', '3', '4+'])))
        print(f"buffer hits by hop:" + "  ".join(f" {h}: {v[7 + i]:,.0f}" for i, h in enumerate(['1', '2', '3', '4+'])))
    have_sc = [r for r in rows if r.get('succ_st') is not None and sum(r['succ_st'][:4]) + sum(r.get('succ_mp', [0] * 9)[:4]) > 0]
    if have_sc:
        print("\n--- trace succession: consecutive costly stretches A -> B ---")
        for key, lbl in (('succ_st', 'steady'), ('succ_mp', 'after a mispredict')):
            v = [amean([r[key][i] for r in have_sc if r.get(key)]) for i in range(9)]
            na, ng = sum(v[:4]), sum(v[4:])
            if na > 0:
                print(f"{lbl:19s} {na:>10,.0f} pairs   B starts at A's next instr {pct(v[0], na):.1f}%  near (<=256B) {pct(v[1], na):.1f}%  "
                      f"same 4KB {pct(v[2], na):.1f}%  jump {pct(v[3], na):.1f}%   |  gap 0 {pct(v[4], ng):.1f}%  1-8 {pct(v[5], ng):.1f}%  "
                      f"9-64 {pct(v[6], ng):.1f}%  65-256 {pct(v[7], ng):.1f}%  >256 {pct(v[8], ng):.1f}% instrs")
    have_vp = [r for r in rows if r.get('vp_lookups', 0) > 0]
    if have_vp:
        kn = have_vp[0].get('vp_knob', (1, 0, 0, 0, 0.0))
        m = {k: amean([r[k] for r in have_vp]) for k in VP_KEYS + VPT_KEYS}
        print(f"\n--- trace value predictor ({kn[1]} tables x {kn[2]} entries, history up to {kn[3]} bits, {kn[4]:.1f} KB) ---")
        print(f"predictions:       {m['vp_lookups']:,.0f}   matched {m['vp_matches']:,.0f} ({pct(m['vp_matches'], m['vp_lookups']):.1f}%)   "
              f"fired {m['vp_fires']:,.0f}   trace in store {m['vp_store_hits']:,.0f}   walks launched {m['vp_walks']:,.0f} "
              f"({m['vp_walks_mp']:,.0f} at a misprediction)")
        print(f"buffer hits:       {m['vp_bhits']:,.0f} ({m['vp_bhits_mp']:,.0f} from walks fired at a misprediction)")
        print(f"training:          {m['vp_train']:,.0f} costly stretches ({m['vp_train_mp']:,.0f} after a misprediction)   "
              f"correct {m['vp_correct']:,.0f} ({pct(m['vp_correct'], m['vp_train']):.1f}%; after a misprediction "
              f"{pct(m['vp_correct_mp'], m['vp_train_mp']):.1f}%)   wrong {m['vp_wrong']:,.0f}   no prediction {m['vp_noprov']:,.0f}")
        print(f"                   matched predictions with no costly stretch after {m['vp_none']:,.0f}   "
              f"allocations {m['vp_allocs']:,.0f} (failed {m['vp_alloc_fail']:,.0f})")
    have_pd = [r for r in rows if r.get('pdiag_mp', 0) > 0]
    if have_pd:
        mp, wt = amean([r['pdiag_mp'] for r in have_pd]), amean([r['pdiag_wt'] for r in have_pd])
        print(f"\n--- next-trace predictability at mispredictions (trace_peek_diag) ---")
        print(f"mispredictions:    {mp:,.0f}   followed by a stored trace {wt:,.0f} ({pct(wt, mp):.1f}%)")
        print(f"  {'key':10s} {'fires':>12s} {'useful':>10s} {'wrong':>10s} {'coverage':>9s} {'accuracy':>9s}   [coverage = useful / followed-by-trace; accuracy = useful / fires]")
        for k in have_pd[0]['pdiag_keys']:
            f_ = amean([r['pdiag_keys'][k][0] for r in have_pd])
            u_ = amean([r['pdiag_keys'][k][1] for r in have_pd])
            w_ = amean([r['pdiag_keys'][k][2] for r in have_pd])
            print(f"  {k:10s} {f_:12,.0f} {u_:10,.0f} {w_:10,.0f} {pct(u_, wt):8.1f}% {pct(u_, f_):8.1f}%")

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

    # lost dispatch slots (top-down middle bound): slots the frontend failed to
    # fill in build mode while the ROB had room; cycle-equiv = slots / width
    have_ls = [r for r in have_fe if r.get('ls_total') is not None]
    if have_ls:
        ls_abs = amean([r['ls_total'] for r in have_ls])
        ls_pct = amean([pct(r['ls_total'] / DISPATCH_WIDTH, r['cycles']) for r in have_ls if r['cycles']])
        ls_ste = amean([pct(r['ls_steady'], r['ls_total']) for r in have_ls if r['ls_total']])
        ls_rec = amean([pct(r['ls_recovery'], r['ls_total']) for r in have_ls if r['ls_total']])
        print(f"lost dispatch slots (mid):       {ls_abs:,.0f} slots   ({ls_pct:.2f}% of runtime as cycle-equiv)")
        print(f"  steady %:                      {ls_ste:.1f}%")
        print(f"  recovery %:                    {ls_rec:.1f}%")


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
            # tolerate scatter-style lists: take the first token, drop any path
            include = {canon_name(os.path.basename(line.split()[0])) for line in f if line.strip()}
        print(f"[include] {len(include)} simpoint name(s); analysis restricted to these")

    baseline = None
    if args.baseline:
        base_rows, base_skipped, _ = load_rows(args.baseline, args.pattern, include)
        baseline = {r['name']: r['ipc'] for r in base_rows}
        print(f"[baseline] {args.baseline}: {len(baseline)} workloads (skipped {base_skipped})")

    for d in args.dirs:
        aggregate(d, args.pattern, include, baseline=baseline, scatter=args.scatter)


if __name__ == '__main__':
    main()
