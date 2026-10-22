#!/usr/bin/env python3
"""Where the u-op-cache headroom is: front-end loss summed over a suite, split into
recovery (from a misprediction redirect to the next u-op-cache hit) and steady state.

usage: scripts/fe_loss_breakdown.py <results dir> [<results dir> ...]
  e.g. scripts/fe_loss_breakdown.py results/prefetch_segmentation/smt_qualcomm/champsim_zen5_fb96fu_{base,rtt64,ideal}_W50M_I150M

Only workloads present (with an IPC line) in every directory are summed.
A second table splits the recovery (empty-ROB) stall by the class of the mispredicted branch,
over the directories whose runs print the recovery-by-branch line.
"""
import os
import re
import sys

PATTERNS = {
    'cycles': r'CPU 0 cumulative IPC: [\d.]+ instructions: \d+ cycles: (\d+)',
    'idle': r'frontend-loss backend-idle \(tight\): steady (\d+) recovery (\d+)',
    'stall': r'frontend-loss stall-cycles \(upper\): steady (\d+) recovery (\d+)',
    'miss': r'frontend-loss misses: steady (\d+) \(traced \d+\) recovery (\d+)',
}


CLASSES = ('cond-H2P', 'cond-confident', 'indirect', 'return', 'direct(BTB-miss)', 'other')
REC = re.compile(r'recovery-by-branch \(mispredictions / empty-ROB recovery cycles\):' +
                 ''.join(r'\s+' + re.escape(c) + r' (\d+) / (\d+)' for c in CLASSES))


def parse(path):
    try:
        with open(path, errors='replace') as f:
            s = f.read()
    except OSError:
        return None
    out = {}
    for k, p in PATTERNS.items():
        m = re.search(p, s)
        if m is None:
            return None
        out[k] = tuple(int(g) for g in m.groups())
    m = REC.search(s)
    if m is not None:
        out['rec'] = tuple(int(g) for g in m.groups())
    return out


def main(dirs):
    runs = {d: {} for d in dirs}
    for d in dirs:
        for name in os.listdir(d):
            r = parse(os.path.join(d, name))
            if r is not None:
                runs[d][name] = r
    common = set.intersection(*(set(v) for v in runs.values())) if runs else set()
    print(f'{len(common)} workloads in common\n')
    print(f'{"run":48s} {"cycles":>9s}  {"ROB-empty u-op-miss stall":>27s}  {"dispatch-starved (upper)":>26s}  {"u-op misses":>21s}')
    print(f'{"":48s} {"":>9s}  {"steady":>9s} {"recovery":>9s} {"rec%":>6s}  {"steady":>9s} {"recovery":>9s} {"rec%":>5s}  {"steady":>9s} {"recovery":>9s}')
    base = None
    for d in dirs:
        t = {k: [0, 0] for k in ('idle', 'stall', 'miss')}
        cyc = 0
        for w in common:
            r = runs[d][w]
            cyc += r['cycles'][0]
            for k in t:
                t[k][0] += r[k][0]
                t[k][1] += r[k][1]
        pct = lambda a, b: 100.0 * b / (a + b) if a + b else 0.0
        label = os.path.basename(os.path.normpath(d))
        line = (f'{label[:48]:48s} {cyc / 1e6:8.1f}M  {t["idle"][0] / 1e6:8.2f}M {t["idle"][1] / 1e6:8.2f}M {pct(*t["idle"]):5.1f}%  '
                f'{t["stall"][0] / 1e6:8.2f}M {t["stall"][1] / 1e6:8.2f}M {pct(*t["stall"]):4.1f}%  {t["miss"][0] / 1e6:8.2f}M {t["miss"][1] / 1e6:8.2f}M')
        if base is None:
            base = cyc
        else:
            line += f'   saves {(base - cyc) / 1e6:.1f}M cycles'
        print(line)

    rdirs = [d for d in dirs if all('rec' in runs[d][w] for w in common)]
    if not rdirs or not common:
        return
    print(f'\nrecovery empty-ROB cycles by mispredicted-branch class (M cycles; mispredicts in K; vs {os.path.basename(os.path.normpath(rdirs[0]))})')
    print(f'{"run":48s} ' + ' '.join(f'{c:>17s}' for c in CLASSES))
    first = None
    for d in rdirs:
        mis = [0] * len(CLASSES)
        cyc = [0] * len(CLASSES)
        for w in common:
            r = runs[d][w]['rec']
            for i in range(len(CLASSES)):
                mis[i] += r[2 * i]
                cyc[i] += r[2 * i + 1]
        label = os.path.basename(os.path.normpath(d))
        tot = sum(cyc) or 1
        print(f'{label[:48]:48s} ' + ' '.join(f'{cyc[i] / 1e6:7.2f}M {100 * cyc[i] / tot:4.1f}%  ' for i in range(len(CLASSES))) + f'  total {tot / 1e6:.2f}M')
        if first is None:
            first = cyc
            print(f'{"  mispredicts / cycles per mispredict":48s} ' + ' '.join(f'{mis[i] / 1e3:7.0f}K {cyc[i] / max(mis[i], 1):5.1f}c ' for i in range(len(CLASSES))))
        else:
            print(f'{"  removed vs first":48s} ' + ' '.join(f'{100 * (first[i] - cyc[i]) / max(first[i], 1):+7.1f}%        ' for i in range(len(CLASSES))))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
