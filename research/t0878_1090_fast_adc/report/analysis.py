"""Crossing points (50 % / 90 %) of every success curve, calibrated to dBm so that today's chain (TL 600, the live
default) has its 90 % point at -84 dBm. Writes summary.json next to this file."""
import json, os, numpy as np
D = '/workspace/agents/worker-6/t0878/data'
CAL_90_DBM = -84.0
CAL_KEY = 'A600'


def load(path):
    if not os.path.exists(path): return None
    return json.load(open(path))


def curve(d, key):
    s = sorted(d['data'], key=float)
    return [(float(x), d['data'][x][key] / d['data'][x]['n']) for x in s if key in d['data'][x]]


def cross(pts, level):
    """First SNR where the curve reaches level (linear interpolation), None if never."""
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if y0 < level <= y1:
            return x0 + (level - y0) / (y1 - y0) * (x1 - x0)
    return None


def summarize(d, keys=None):
    out = {}
    if d is None: return out
    allk = sorted({k for v in d['data'].values() for k in v if k != 'n' and not k.endswith('_match')})
    for k in (keys or allk):
        p = curve(d, k)
        out[k] = dict(s50=cross(p, .5), s90=cross(p, .9), plateau=float(np.mean([y for x, y in p[-4:]])) if p else None)
    return out


if __name__ == '__main__':
    res = {}
    for name in ('main', 'a_sweep', 'flash', 'iq', 'rate100'):
        res[name] = summarize(load(f'{D}/{name}.json'))
    for v in ('fl0', 'fl6', 'flm7', 'v5'):
        for name in ('main', 'a_sweep', 'flash'):
            res[f'{v}/{name}'] = summarize(load(f'{D}/{v}/{name}.json'))
    off = CAL_90_DBM - res['main'][CAL_KEY]['s90']
    res['cal'] = dict(offset_db=off, noise_dbm_in_band=off, nf_db=off + 174 - 10 * np.log10(23.94e6))
    json.dump(res, open(os.path.join(os.path.dirname(__file__), 'summary.json'), 'w'), indent=1)
    print('cal', res['cal'])
    for name in res:
        if name == 'cal': continue
        for k, v in res[name].items():
            if v['s50'] is not None or v['s90'] is not None:
                f = lambda x: f'{x:6.2f}' if x is not None else '   -  '
                print(f'{name:14s} {k:22s} s50 {f(v["s50"])} s90 {f(v["s90"])} plateau {v["plateau"]:.3f}')
