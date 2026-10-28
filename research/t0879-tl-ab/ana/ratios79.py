#!/usr/bin/env python3
"""Paired-within-round ratios vs TL 600 with t confidence intervals.

usage: ratios79.py AB79_JSON [OUT_JSON]
Rounds follow sched.txt: indices 0-14 -> round idx//5, 15 = LEARN (paired with
the 600 block at index 16), 16-25 -> round (idx-1)//5. Partial rounds (missing
the 600 block or any arm) are dropped from the ratios.
"""
import json
import math
import sys

import numpy as np

T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262}
ARMS = ['400', '450', '500', '600', '800']
METRICS = ['valid_s', 'caps_s', 'guard_s', 'unique_ac', 'fpa', 'crc_fail_pct', 'yield_pct', 'core1_pct',
           'demod_us', 'lone', 'jumps', 'pos_fixes', 'duty', 'edges_s']


def round_of(idx):
    if idx < 15:
        return idx // 5
    if idx == 15:
        return 'L'
    return (idx - 1) // 5


def ci(logs):
    """Mean ratio and 95 % CI from per-round log ratios."""
    x = np.array(logs, float)
    n = len(x)
    m = float(x.mean())
    if n < 2:
        return dict(n=n, r=math.exp(m), lo=None, hi=None)
    h = T975.get(n - 1, 1.96) * float(x.std(ddof=1)) / math.sqrt(n)
    return dict(n=n, r=math.exp(m), lo=math.exp(m - h), hi=math.exp(m + h))


def get(b, k):
    if k.startswith('bin'):
        i = int(k[3:])
        return b['pos_bins_h'][i] if b.get('pos_bins_h') else None
    if k.startswith('acbin'):
        i = int(k[5:])
        return b['ac_bins'][i] if b.get('ac_bins') else None
    if k.startswith('range'):
        return b.get('range_pct', {}).get(k[5:])
    if k.startswith('acmax'):
        return b.get('acmax_pct', {}).get(k[5:])
    if k.startswith('raw_'):
        return (b.get('raw') or {}).get(k[4:])
    return b.get(k)


def main():
    blocks = json.load(open(sys.argv[1]))
    rounds = {}
    for b in blocks:
        idx = int(b['tag'][1:4])
        b['idx'] = idx
        b['round'] = round_of(idx)
        rounds.setdefault(b['round'], {})[b['tl']] = b
    full = {r: v for r, v in rounds.items() if r != 'L' and all(a in v for a in ARMS)}
    nbins = len(blocks[0].get('pos_bins_h') or [])
    keys = METRICS + ['range50', 'range90', 'range99', 'acmax50', 'acmax90'] + \
        ['bin%d' % i for i in range(nbins)] + ['acbin%d' % i for i in range(nbins)] + \
        ['raw_duty', 'raw_duty_sig', 'raw_duty_q', 'raw_edges_s', 'raw_edges_s_sig', 'raw_edges_s_q']
    res = {}
    for arm in ARMS + ['LEARN']:
        res[arm] = {}
        for k in keys:
            logs = []
            if arm == 'LEARN':
                pairs = [(rounds.get('L', {}).get('LEARN'), blocks_by_idx(blocks, 16))]
            else:
                pairs = [(v[arm], v['600']) for v in full.values()]
            for a, z in pairs:
                if not a or not z:
                    continue
                x, y = get(a, k), get(z, k)
                if x is None or y is None:
                    continue
                if x <= 0 or y <= 0:
                    # Counts can be zero; add 0.5 to both (continuity) for count-like keys.
                    x, y = x + 0.5, y + 0.5
                logs.append(math.log(x / y))
            if logs:
                res[arm][k] = ci(logs)
    # Per-arm means of raw values (not ratios) for tables.
    means = {}
    for arm in ARMS + ['LEARN']:
        bs = [v[arm] for v in full.values()] if arm != 'LEARN' else [b for b in blocks if b['tl'] == 'LEARN']
        means[arm] = {k: (float(np.mean([get(b, k) for b in bs if get(b, k) is not None]))
                          if any(get(b, k) is not None for b in bs) else None) for k in keys}
        means[arm]['n'] = len(bs)
        means[arm]['tl_offset_mv'] = float(np.median([b['tl_offset_mv'] for b in bs])) if bs else None
    out = dict(rounds=sorted(str(r) for r in full), ratios=res, means=means,
               partial=sorted(str(r) for r in rounds if r not in full and r != 'L'))
    json.dump(out, open(sys.argv[2] if len(sys.argv) > 2 else 'ratios79.json', 'w'), indent=1)
    print('full rounds:', out['rounds'], 'partial:', out['partial'])
    for k in ['valid_s', 'unique_ac', 'fpa', 'crc_fail_pct', 'lone', 'jumps', 'core1_pct', 'duty', 'range90',
              'range99', 'acmax90']:
        row = []
        for arm in ARMS + ['LEARN']:
            c = res[arm].get(k)
            if not c:
                row.append('%16s' % '-')
            elif c['lo'] is None:
                row.append('%16s' % ('%.3f' % c['r']))
            else:
                row.append('%16s' % ('%.3f [%.3f,%.3f]' % (c['r'], c['lo'], c['hi'])))
        print('%-13s' % k, ' '.join(row))


def blocks_by_idx(blocks, idx):
    for b in blocks:
        if b['idx'] == idx:
            return b
    return None


if __name__ == '__main__':
    main()
