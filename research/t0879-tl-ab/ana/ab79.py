#!/usr/bin/env python3
"""ab79.py DATA_DIR [SETTLE_S]: t-0879 pi-0 TL-offset A/B from a copy of ~/t0879/runs -> ab79.json + table.

Per block (console log of the lab image):
  #LS (10 s): captures (begins), guard aborts (G), mid aborts (M), queue drops (O), no message (N), CRC fail (C),
              clean (K), fixed (F), retry rescues (R), address/parity (A), demod time, packets, valid, nf/TL.
  #LP (per decoded packet into the dictionary): valid flag, corrected bits, DF, bits -> ICAO, positions.
  #LDUTY / #LRAW: comparator HI duty and edges (lraw.py splits on-signal vs between frames).
Ratios vs TL 600 are paired within each round (each round holds every arm once), geometric mean with t CI.
Range: DF17 airborne positions (global CPR pairs, then local relative to the last fix). There is no receiver
position in the unit's settings, so the center is estimated from the data (private, never printed)."""
import glob, json, math, os, re, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lraw

D = sys.argv[1]
SETTLE = float(sys.argv[2]) if len(sys.argv) > 2 else 60
PRIV = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'priv')
T = [12.71, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131, 2.120]
PR_EDGES = [0, 15, 30, 45, 60, 400]
LS_COLS = ('seq ms begins G M B O P N C K F R A demods demod_us demod_us_max qhwm ring_full dump_full packets valid '
           'nf_mv tl_offset_mv tl_mv learning learn_temp_mv').split()

# ---------------------------------------------------------------- Mode S
POLY = 0xFFF409


def crc24(bits_int, nbits):
    """Mode S parity over the first nbits-24 bits; returns the 24-bit remainder."""
    data = bits_int >> 24
    n = nbits - 24
    rem = 0
    for i in range(n - 1, -1, -1):
        b = (data >> i) & 1
        top = (rem >> 23) & 1
        rem = ((rem << 1) & 0xFFFFFF)
        if b ^ top:
            rem ^= POLY & 0xFFFFFF
    return rem


def icao_of(df, v, nbits):
    if df in (11, 17, 18):
        return (v >> (nbits - 32)) & 0xFFFFFF
    # Address/parity: AP = parity XOR address.
    return (v & 0xFFFFFF) ^ crc24(v, nbits)


# ---------------------------------------------------------------- CPR
NZ = 15


def nl(lat):
    if abs(lat) >= 87:
        return 1
    if lat == 0:
        return 59
    a = 1 - math.cos(math.pi / (2 * NZ))
    b = math.cos(math.pi / 180 * abs(lat)) ** 2
    return int(math.floor(2 * math.pi / math.acos(1 - a / b)))


def cpr_global(e, o, last_odd):
    """e, o: (lat_cpr, lon_cpr) as fractions. Returns (lat, lon) or None."""
    dlat0, dlat1 = 360 / 60, 360 / 59
    j = math.floor(59 * e[0] - 60 * o[0] + 0.5)
    lat0 = dlat0 * ((j % 60) + e[0])
    lat1 = dlat1 * ((j % 59) + o[0])
    if lat0 >= 270: lat0 -= 360
    if lat1 >= 270: lat1 -= 360
    if nl(lat0) != nl(lat1):
        return None
    lat = lat1 if last_odd else lat0
    n = nl(lat)
    m = math.floor(e[1] * (n - 1) - o[1] * n + 0.5)
    ni = max(n - (1 if last_odd else 0), 1)
    lon = (360 / ni) * ((m % ni) + (o[1] if last_odd else e[1]))
    if lon >= 180: lon -= 360
    return lat, lon


def cpr_local(c, odd, ref):
    dlat = 360 / (59 if odd else 60)
    j = math.floor(ref[0] / dlat) + math.floor(0.5 + (ref[0] % dlat) / dlat - c[0])
    lat = dlat * (j + c[0])
    n = max(nl(lat) - (1 if odd else 0), 1)
    dlon = 360 / n
    m = math.floor(ref[1] / dlon) + math.floor(0.5 + (ref[1] % dlon) / dlon - c[1])
    return lat, dlon * (m + c[1])


def nm(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 3440.065 * math.asin(math.sqrt(min(1, h)))


class Tracks:
    """Airborne position decoding; counts fixes that disagree with the track."""

    def __init__(self):
        self.cpr = {}   # icao -> {0: (t, c), 1: (t, c)}
        self.fix = {}   # icao -> (t, lat, lon)
        self.jumps = 0

    def add(self, t, icao, c, odd):
        """Returns (lat, lon) or None."""
        d = self.cpr.setdefault(icao, {})
        d[odd] = (t, c)
        f = self.fix.get(icao)
        pos = None
        if f and t - f[0] < 120:
            pos = cpr_local(c, odd, (f[1], f[2]))
        elif 0 in d and 1 in d and abs(d[0][0] - d[1][0]) < 10:
            pos = cpr_global(d[0][1], d[1][1], odd)
            if pos and f is None:
                pass
        if pos is None:
            return None
        if f and t - f[0] < 120:
            lim = 2.0 + 0.2 * (t - f[0])  # 720 kt plus slack.
            if nm(pos, (f[1], f[2])) > lim:
                self.jumps += 1
                # Re-anchor with a global pair if one is fresh; else drop this fix.
                if 0 in d and 1 in d and abs(d[0][0] - d[1][0]) < 10:
                    g = cpr_global(d[0][1], d[1][1], odd)
                    if g and nm(g, pos) < 1:
                        self.fix[icao] = (t, *g)
                        return g
                return None
        self.fix[icao] = (t, *pos)
        return pos


# ---------------------------------------------------------------- parsing
def read_log(path):
    for l in open(path, errors='replace'):
        p = l.rstrip('\n').split(' ', 1)
        if len(p) < 2:
            continue
        try:
            t = float(p[0])
        except ValueError:
            continue
        yield t, p[1]


def parse_block(path, t_from):
    ls, lp, duty, raw_lines, notes = [], [], [], [], {}
    for t, s in read_log(path):
        if s.startswith('TL_OFFSET='):
            notes.setdefault('tl_offset', []).append(s)
        elif s.startswith('CONFIG_HASH=') or s.startswith('FIRMWARE_RP2040='):
            notes.setdefault('hash', []).append(s)
        if t < t_from:
            continue
        if s.startswith('#LS,'):
            v = s.split(',')[1:]
            if len(v) == len(LS_COLS):
                ls.append(dict(zip(LS_COLS, map(int, v)), t=t))
        elif s.startswith('#LDUTY,'):
            n, h, e, us = map(int, s.split(',')[1:])
            duty.append((h / n, e / (us * 1e-6)))
        elif s.startswith('#LRAW,') or s.startswith('#LRW,'):
            raw_lines.append(s)
    return ls, lp, duty, raw_lines, notes


def parse_lp(s):
    v = s.split(',')
    if len(v) != 6:
        return None
    _, mlat, valid, corr, df, bits = v
    nbits = len(bits) * 4
    if nbits not in (56, 112):
        return None
    return int(valid), int(corr), int(df), int(bits, 16), nbits


def tl_of(tag):
    return tag.split('-', 1)[1]


def main():
    runs = os.path.join(D, 'runs')
    blocks = {}
    for l in open(os.path.join(runs, 'ab.log')):
        m = re.match(r'(\S+) block (\S+) (start|logging|done)', l)
        if m:
            blocks.setdefault(m.group(2), {})[m.group(3)] = m.group(1)
    out = []
    all_pos = []
    for tag in sorted(blocks):
        path = os.path.join(runs, tag + '.log')
        if 'done' not in blocks[tag] or not os.path.exists(path):
            continue
        first = next(read_log(path))[0]
        ls, _, duty, raw_lines, notes = parse_block(path, first + SETTLE)
        if len(ls) < int(os.environ.get('MIN_LS', 30)):
            continue
        secs = sum(1 for _ in ls) * 10.0
        g = lambda k: sum(r[k] for r in ls)
        # Packets.
        tr = Tracks()
        ac = {}
        dfs = {}
        pos_d = []
        npk = nvalid = 0
        for t, s in read_log(path):
            if t < first + SETTLE or not s.startswith('#LP,'):
                continue
            p = parse_lp(s)
            if not p:
                continue
            valid, corr, df, v, nbits = p
            npk += 1
            if not valid:
                continue
            nvalid += 1
            icao = icao_of(df, v, nbits)
            a = ac.setdefault(icao, dict(n=0, es=0, ap=0, dfs=set(), maxr=0.0, pos=0, pm=0, pt0=None, pt1=None))
            a['n'] += 1
            a['dfs'].add(df)
            dfs[df] = dfs.get(df, 0) + 1
            if df in (17, 18):
                a['es'] += 1
                me = (v >> 24) & ((1 << 56) - 1)
                tc = me >> 51
                if 9 <= tc <= 18 or 20 <= tc <= 22:
                    a['pm'] += 1
                    a['pt0'] = t if a['pt0'] is None else a['pt0']
                    a['pt1'] = t
                    odd = (me >> 34) & 1
                    c = (((me >> 17) & 0x1FFFF) / 131072, (me & 0x1FFFF) / 131072)
                    pos = tr.add(t, icao, c, odd)
                    if pos:
                        pos_d.append((icao, pos))
                        a['pos'] += 1
            else:
                a['ap'] += 1
        all_pos += [p for _, p in pos_d]
        shots = [lraw.split(us, b) for us, b in lraw.parse(raw_lines)]
        rs = lraw.summarize(shots) if shots else {}
        caps = g('begins')
        r = dict(tag=tag, tl=tl_of(tag), secs=secs, start=blocks[tag]['logging'],
                 valid_s=nvalid / secs, packets_s=npk / secs, ls_valid_s=g('valid') / secs,
                 caps_s=caps / secs, guard_s=g('G') / secs, mid_s=g('M') / secs, qdrop=g('O'),
                 crc_fail_pct=100 * g('C') / max(1, g('C') + g('K') + g('F') + g('R') + g('A')),
                 nomsg_s=g('N') / secs, fixed_s=g('F') / secs, retry_s=g('R') / secs,
                 yield_pct=100 * (g('K') + g('F') + g('R') + g('A')) / max(1, caps),
                 core1_pct=100 * g('demod_us') / (secs * 1e6), demod_us=g('demod_us') / max(1, g('demods')),
                 nf_mv=float(np.mean([x['nf_mv'] for x in ls])), tl_mv=float(np.mean([x['tl_mv'] for x in ls])),
                 tl_offset_mv=float(np.median([x['tl_offset_mv'] for x in ls])),
                 unique_ac=sum(1 for a in ac.values() if a['n'] >= 2), lone=sum(1 for a in ac.values() if a['n'] == 1),
                 lone_ap=sum(1 for a in ac.values() if a['n'] == 1 and a['ap'] == 1),
                 jumps=tr.jumps, pos_fixes=len(pos_d),
                 duty=float(np.mean([d for d, e in duty])) if duty else None,
                 edges_s=float(np.mean([e for d, e in duty])) if duty else None,
                 raw=rs, hashes=sorted(set(notes.get('hash', []))))
        r['fpa'] = r['valid_s'] * secs / max(1, r['unique_ac'])
        r['_pos'] = pos_d
        r['_ac'] = {k: dict(n=a['n'], es=a['es'], pos=a['pos'], pm=a['pm'], span=(a['pt1'] - a['pt0']) if a['pm'] else 0) for k, a in ac.items()}
        out.append(r)
    # Receiver center (private): median of positions, then refine to center of the inner 50 % by distance.
    if all_pos:
        P = np.array(all_pos)
        c = np.median(P, axis=0)
        os.makedirs(PRIV, exist_ok=True)
        json.dump(dict(center=list(map(float, c)), n=len(P)), open(os.path.join(PRIV, 'center.json'), 'w'))
        for r in out:
            d = np.array([nm(c, p) for _, p in r['_pos']]) if r['_pos'] else np.array([])
            r['range_pct'] = {str(q): float(np.percentile(d, q)) for q in (50, 90, 99)} if len(d) else {}
            per_ac = {}
            for icao, p in r['_pos']:
                per_ac[icao] = max(per_ac.get(icao, 0), nm(c, p))
            m = np.array(list(per_ac.values()))
            r['acmax_pct'] = {str(q): float(np.percentile(m, q)) for q in (50, 90)} if len(m) else {}
            edges = [0, 25, 50, 75, 100, 125, 150, 200, 400]
            r['pos_bins_h'] = (np.histogram(d, edges)[0] * 3600 / r['secs']).tolist()
            r['ac_bins'] = np.histogram(m, edges)[0].tolist()
            r['bin_edges'] = edges
            # Position-message rate per aircraft (nominal 2/s), by median distance: decode probability vs range.
            dl = {}
            for icao, p in r['_pos']:
                dl.setdefault(icao, []).append(nm(c, p))
            pr = [[0, 0.0, 0] for _ in PR_EDGES[:-1]]
            for icao, ds in dl.items():
                a = r['_ac'][icao]
                if a['pm'] < 10 or a['span'] < 60:
                    continue
                k = int(np.searchsorted(PR_EDGES, np.median(ds), side='right')) - 1
                if 0 <= k < len(pr):
                    pr[k][0] += a['pm']; pr[k][1] += a['span']; pr[k][2] += 1
            r['pr_bins'] = [x[0] / x[1] if x[1] else None for x in pr]
            r['pr_n'] = [x[2] for x in pr]
    for r in out:
        r.pop('_pos', None)
        r.pop('_ac', None)
    json.dump(out, open(os.path.join(D, 'ab79.json'), 'w'), indent=1, default=str)
    cols = ['tag', 'tl_offset_mv', 'valid_s', 'caps_s', 'guard_s', 'crc_fail_pct', 'unique_ac', 'fpa', 'lone', 'jumps',
            'core1_pct', 'duty', 'nf_mv']
    print(' '.join('%12s' % c[:12] for c in cols))
    for r in out:
        print(' '.join('%12s' % (('%.3f' % r[c]) if isinstance(r.get(c), float) else r.get(c)) for c in cols),
              ' range p50/90/99', {k: round(v) for k, v in r.get('range_pct', {}).items()})


if __name__ == '__main__':
    main()
