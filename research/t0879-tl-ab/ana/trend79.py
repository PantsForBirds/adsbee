#!/usr/bin/env python3
"""Arm effects vs TL 600 with a smooth traffic trend removed (weighted least squares on log values).

usage: trend79.py AB79_JSON OUT_JSON [MAX_HOURS]
Model: log y_block = arm effect + f(time), where f is linear within the first sitting (16:30-19:00) and a
linear spline with knots every 2 h within the second sitting (22:30-09:30). Weights = blocks' aircraft count
(night blocks with ~15 aircraft count less than day blocks with ~170). 95 % CIs from the residual variance
(t, df = n - params). MAX_HOURS keeps only blocks that start within that many hours of the first block.
"""
import json
import math
import sys
from datetime import datetime

import numpy as np

from ratios79 import get

TQ = {d: v for d, v in zip(range(1, 31), [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228,
                                          2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086,
                                          2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042])}
ARMS = ['400', '450', '500', '800', 'LEARN']
BINS = [(0, 1), (1, 2), (2, 3), (3, 8)]


def value(b, k):
    if k[:3] == 'acm' and k[3:].isdigit():  # merged per-aircraft distance bins
        lo, hi = BINS[int(k[3:])]
        return sum(b['ac_bins'][lo:hi])
    if k[:2] == 'pr' and k[2:].isdigit():
        return b['pr_bins'][int(k[2:])]
    if k.startswith('posm'):
        lo, hi = BINS[int(k[4:])]
        return sum(b['pos_bins_h'][lo:hi])
    return get(b, k)


def design(blocks):
    t0 = min(datetime.fromisoformat(b['start']) for b in blocks)
    t = np.array([(datetime.fromisoformat(b['start']) - t0).total_seconds() / 3600 for b in blocks])
    seg2 = t > 4.5  # after the 19:00-22:30 pause
    cols, names = [], []
    for a in ARMS:
        cols.append(np.array([b['tl'] == a for b in blocks], float))
        names.append(a)
    s1 = (~seg2).astype(float)
    cols += [s1, s1 * t]
    if seg2.any():
        s2 = seg2.astype(float)
        ts = t[seg2].min()
        cols += [s2, s2 * (t - ts)]
        for k in np.arange(ts + 2, t.max() - 0.5, 2):
            cols.append(s2 * np.maximum(0, t - k))
    X = np.column_stack(cols)
    keep = X.any(axis=0)
    return X[:, keep], [n for n, k in zip(names + ['t'] * (X.shape[1] - len(names)), keep) if True], t


def fit(blocks, key, w):
    y = []
    for b in blocks:
        v = value(b, key)
        if v is None or (v <= 0 and key not in COUNTS):
            y.append(None)
        else:
            y.append(math.log(v + 0.5) if key in COUNTS else math.log(v))
    ok = np.array([v is not None for v in y])
    if ok.sum() < 8:
        return {}
    bl = [b for b, o in zip(blocks, ok) if o]
    X, _, _ = design(bl)
    present = [a for a in ARMS if any(b['tl'] == a for b in bl)]
    X = X[:, [i for i in range(X.shape[1]) if i >= len(ARMS) or ARMS[i] in present]]
    Y = np.array([v for v in y if v is not None])
    W = np.sqrt(np.array(w)[ok])
    beta, *_ = np.linalg.lstsq(X * W[:, None], Y * W, rcond=None)
    res = (Y - X @ beta) * W
    df = len(Y) - X.shape[1]
    s2 = float(res @ res) / max(df, 1)
    cov = s2 * np.linalg.pinv((X * W[:, None]).T @ (X * W[:, None]))
    out = {}
    for i, a in enumerate(present):
        h = TQ.get(df, 1.96) * math.sqrt(cov[i, i])
        out[a] = dict(r=math.exp(beta[i]), lo=math.exp(beta[i] - h), hi=math.exp(beta[i] + h), df=df)
    return out


COUNTS = {'unique_ac', 'lone', 'lone_ap', 'jumps', 'pos_fixes', 'acm0', 'acm1', 'acm2', 'acm3'}
KEYS = ['valid_s', 'caps_s', 'guard_s', 'unique_ac', 'fpa', 'crc_fail_pct', 'yield_pct', 'core1_pct', 'demod_us',
        'lone', 'lone_ap', 'jumps', 'pos_fixes', 'duty', 'edges_s', 'range50', 'range90', 'range99', 'acmax50',
        'acmax90', 'acm0', 'acm1', 'acm2', 'acm3', 'posm0', 'posm1', 'posm2', 'posm3', 'raw_duty_q', 'raw_duty_sig']


def main():
    blocks = json.load(open(sys.argv[1]))
    if len(sys.argv) > 3:
        t0 = min(datetime.fromisoformat(b['start']) for b in blocks)
        blocks = [b for b in blocks
                  if (datetime.fromisoformat(b['start']) - t0).total_seconds() / 3600 < float(sys.argv[3])]
    w = [max(b['unique_ac'], 1) for b in blocks]
    res = {k: fit(blocks, k, w) for k in KEYS}
    for i in range(5):
        res['pr%d' % i] = fit(blocks, 'pr%d' % i, [max(b['pr_n'][i], 0.01) for b in blocks])
    KEYS.extend('pr%d' % i for i in range(5))
    json.dump(dict(n=len(blocks), fx=res), open(sys.argv[2], 'w'), indent=1)
    print('blocks', len(blocks))
    for k in KEYS:
        print('%-13s' % k, ' '.join('%22s' % ('%.3f [%.3f,%.3f]' % (res[k][a]['r'], res[k][a]['lo'], res[k][a]['hi'])
                                               if a in res[k] else '-') for a in ARMS))


if __name__ == '__main__':
    main()
