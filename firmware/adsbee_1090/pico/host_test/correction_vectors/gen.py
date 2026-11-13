#!/usr/bin/env python3
"""Frame sets for the aircraft dictionary correction harness.

Bench sets (wrongx, nbr, ghost, ecc) are the #398/#420 bench generators (signal generator sessions), same seeds and rng
call order, with the radio replaced by timestamps (1 frame per ms, ROUND_MS per round). --check DIR compares them with
the frames a recorded session sent (DIR/wrongx.jsonl, nbr.jsonl, ghost.jsonl, ecc.jsonl).
New sets add positions and velocities (the bench sets carry idents and replies only) and enough frames to see rare
miscorrections.

Line format: '<ms> <role> <hex> [<original>]'
  P  clean frame from a tracked aircraft (sets the truth)
  B  frame that must not change a tracked aircraft (untracked, random, or 3+ bit errors)
  G  a tracked aircraft's frame with 1 or 2 bit errors; <original> is its clean frame (a fix to it is a good update)
'# set <name>' starts a set (fresh dictionary).

Usage: python3 gen.py OUT [--check DIR] [--bench-only]   (STRESS_ROUNDS=400 sets the new sets' rounds)
Then:  CORRECTION_VECTORS=OUT host_test --gtest_filter='AircraftDictionaryCorrections.*'"""
import json, math, os, random, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import msenc as F
from msenc import encode_airborne_position

# ---- Bench encoders, without the bench address check ----
GEN = 0x1FFF409
def crc24(h):
    v = int(h, 16) << 24; n = len(h) * 4 + 24
    for i in range(n - 1, 23, -1):
        if v >> i & 1: v ^= GEN << (i - 24)
    return v & 0xFFFFFF
CH = '#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######'
def df17_ident(icao, df=17, ca=5):
    me = 4 << 51
    for i, c in enumerate(('BEE%03X' % (icao & 0xFFF)).ljust(8)): me |= CH.index(c) << (42 - 6 * i)
    head = '%02X%06X%014X' % ((df << 3) | ca, icao, me); return head + '%06X' % crc24(head)
def ap(head, icao): return head + '%06X' % (crc24(head) ^ icao)
def ac13(alt_ft):
    n = (alt_ft + 1000) // 25
    return ((n >> 5) << 7) | (((n >> 4) & 1) << 5) | (1 << 4) | (n & 0xF)
def df0(icao, alt=10000): return ap('%08X' % ((0 << 27) | ac13(alt)), icao)
def df4(icao, alt=10000): return ap('%08X' % ((4 << 27) | ac13(alt)), icao)
def df5(icao, sq=0o1200):
    a, b, c, d = (sq >> 9) & 7, (sq >> 6) & 7, (sq >> 3) & 7, sq & 7
    bits = [c & 1, a & 1, (c >> 1) & 1, (a >> 1) & 1, (c >> 2) & 1, (a >> 2) & 1, 0, (b) & 1, d & 1, (b >> 1) & 1, (d >> 1) & 1, (b >> 2) & 1, (d >> 2) & 1]
    idf = 0
    for x in bits: idf = (idf << 1) | x
    return ap('%08X' % ((5 << 27) | idf), icao)
def df11(icao, ca=5): head = '%02X%06X' % ((11 << 3) | ca, icao); return head + '%06X' % crc24(head)
def df16(icao, alt=10000): return ap('%08X%014X' % ((16 << 27) | ac13(alt), 0x58000000000000), icao)
def df20(icao, alt=10000):
    mb = 0x20 << 48
    for i, c in enumerate(('BEE%03X' % (icao & 0xFFF)).ljust(8)): mb |= CH.index(c) << (42 - 6 * i)
    return ap('%08X%014X' % ((20 << 27) | ac13(alt), mb), icao)
def df21(icao, sq=0o1200):
    h = df5(icao, sq)[:8]; h = '%08X' % ((int(h, 16) & 0x07FFFFFF) | (21 << 27))
    mb = 0x20 << 48
    for i, c in enumerate(('BEE%03X' % (icao & 0xFFF)).ljust(8)): mb |= CH.index(c) << (42 - 6 * i)
    return ap(h + '%014X' % mb, icao)
def flipbits(h, bits):
    n = len(h) * 4; v = int(h, 16)
    for b in bits: v ^= 1 << (n - 1 - b)
    return '%0*X' % (len(h), v)

ROUND_MS = 3000
OUT = []
def header(name): OUT.append('# set ' + name)
def emit(t, role, h, orig=None): OUT.append('%d %s %s%s' % (t, role, h, ' ' + orig if orig else ''))
def block(t0, frames):
    """frames: list of (role, hex, orig) sent 1 per ms from t0; returns the next free time."""
    for i, (r, h, o) in enumerate(frames): emit(t0 + i, r, h, o)
    return t0 + len(frames)

# ---- bench sets ----
def wrongx(rounds, prime_alt):
    rng = random.Random(914)
    T = [0xBEE080 + i for i in range(64)]; U = [0xBEE0C0 + i for i in range(64)]
    APF = [df0, df4, df5, df16, df20, df21]
    def rnd_frame():
        df = rng.choice([0, 4, 5, 11, 16, 17, 20, 21]); nbits = 112 if df >= 16 else 56
        v = (df << (nbits - 5)) | rng.getrandbits(nbits - 5)
        return '%0*X' % (nbits // 4, v)
    t = 1000; cats = []
    if prime_alt:   # "DF4 bursts first": the tracked aircraft's altitude and squawk are known.
        t = block(t, [('P', df17_ident(k), None) for k in T] + [('P', f(k), None) for k in T for f in (df4, df5)]) + 500
    for rnd in range(rounds):
        refresh = [df17_ident(k) for k in T]
        cat = []
        for i in range(48): u = U[rng.randrange(64)]; o = APF[i % 6](u); cat.append(('untracked_ap', None, o))
        for i in range(64):
            k = T[rng.randrange(64)]; o = [df0, df4, df5, df20, df21][i % 5](k); n = len(o) * 4
            cat.append(('ap_bad', o, flipbits(o, rng.sample(range(5, n), rng.randint(3, 6)))))
        for i in range(48):
            o = df17_ident(T[rng.randrange(64)]); cat.append(('es_bad', o, flipbits(o, rng.sample(range(5, 112), rng.randint(3, 8)))))
        for i in range(32):
            o = df11(T[rng.randrange(64)]); cat.append(('df11_bad', o, flipbits(o, rng.sample(range(5, 56), rng.randint(3, 6)))))
        for i in range(64): cat.append(('random', None, rnd_frame()))
        seq = [c[2] for c in cat]; rng.shuffle(seq)
        cats.append(cat)
        t0 = 1000 + rnd * ROUND_MS + (3000 if prime_alt else 0)
        t = block(t0, [('P', h, None) for h in refresh]) + 500
        block(t, [('B', h, None) for h in seq])
    return cats

def nbr(rounds, prime_alt):
    rng = random.Random(9140)
    T = [0xBEE080 + i for i in range(64)]; N = [t ^ 0x40 for t in T] + [t ^ 0x80 for t in T]
    APF = [df0, df4, df5, df16, df20, df21]
    seqs = []
    t = 1000
    if prime_alt:
        t = block(t, [('P', df17_ident(k), None) for k in T] + [('P', f(k), None) for k in T for f in (df4, df5)]) + 500
    for rnd in range(rounds):
        refresh = [df17_ident(k) for k in T]
        seq = [APF[i % 6](N[rng.randrange(len(N))]) for i in range(256)]; rng.shuffle(seq)
        seqs.append(seq)
        t0 = 1000 + rnd * ROUND_MS + (3000 if prime_alt else 0)
        t = block(t0, [('P', h, None) for h in refresh]) + 500
        block(t, [('B', h, None) for h in seq])
    return seqs

def ghost(rounds):
    rng = random.Random(988)
    T = [0xBEE000 + i for i in range(16)]; U = [0xBEE0E0 + i for i in range(16)]
    APF = [df0, df4, df5, df16, df20, df21, df0, df4]
    def pos(k, n): return encode_airborne_position('%06X' % k, F.SITE[0] + 0.05 * (k & 15) + 0.00027 * (n // 2), F.SITE[1] + 0.3, 10000, n % 2 == 1)
    n = 0; cats = []
    for rnd in range(rounds):
        warm = []
        for k in T: warm += [pos(k, n), pos(k, n + 1), df4(k), df5(k)]
        n += 2
        seq = []; cat = []
        for i, k in enumerate(T):
            for j in range(8):
                seq.append(('P', pos(k, n + j), None)); o = APF[j](U[i]); nb = 1 + (i + j + rnd) % 2
                e = flipbits(o, rng.sample(range(5, len(o) * 4), nb)); seq.append(('B', e, None)); cat.append((nb, o, e))
        n += 8
        cats.append(cat)
        t = block(1000 + rnd * ROUND_MS, [('P', h, None) for h in warm]) + 500
        block(t, seq)
    return cats

def ecc(rounds):
    rng = random.Random(774)
    T = [0xBEE0E0 + i for i in range(8)]; U = [0xBEE0F8 + i for i in range(8)]
    def pos(k, rnd, odd, dlat=0.0, alt=10000):
        return encode_airborne_position('%06X' % k, F.SITE[0] + 0.05 * (k & 7) + 0.0003 * rnd + dlat, F.SITE[1] + 0.3, alt, odd)
    def vel(k, ew=-150): return F.df17_velocity('%06X' % k, ew_kt=ew)
    cats = []
    for rnd in range(rounds):
        clean, cat = [], []
        for k in T:
            clean += [pos(k, rnd, False), pos(k, rnd, True), vel(k), df4(k, 10000)]
        warm = clean[:]
        for k in T:
            for _ in range(2): o = vel(k, -153); cat.append(('es1', o, flipbits(o, rng.sample(range(5, 112), 1))))
            for _ in range(4): o = vel(k, -152); cat.append(('es2', o, flipbits(o, rng.sample(range(5, 112), 2))))
            for _ in range(3): o = pos(k, rnd, rng.random() < 0.5, 0.0001, 10025); cat.append(('pos2', o, flipbits(o, rng.sample(range(5, 112), 2))))
            for _ in range(4): o = df4(k, 10050); cat.append(('ap1', o, flipbits(o, rng.sample(range(5, 56), 1))))
            for _ in range(3): o = df4(k, 10075); cat.append(('ap2', o, flipbits(o, rng.sample(range(5, 56), 2))))
            for _ in range(2): o = vel(k, -151); cat.append(('es6', o, flipbits(o, rng.sample(range(5, 112), 6))))
        for u in U:
            for _ in range(6): o = df4(u, 10100 + 100 * rng.randrange(0, 6)); cat.append(('untracked_ap', o, o))
            for _ in range(4): o = vel(u, -152); cat.append(('untracked_es2', o, flipbits(o, rng.sample(range(5, 112), 2))))
        cl = set(clean)
        roles = {}
        for kind, o, s in cat:
            roles[s] = ('G', o) if kind in ('es1', 'es2', 'pos2', 'ap1', 'ap2') else ('B', None)
        seq = clean + [c[2] for c in cat]
        rng.shuffle(seq)
        cats.append(cat)
        t = block(1000 + rnd * ROUND_MS, [('P', h, None) for h in warm]) + 500
        block(t, [('P', h, None) if h in cl else (roles[h][0], h, roles[h][1]) for h in seq])
    return cats

# ---- new sets ----
EW_KT, NS_KT = -150, 120
def spread_addresses(n, seed, min_bits=6):
    """n random addresses at least min_bits apart, so no reply with a few bit errors matches another tracked one."""
    rng = random.Random(seed); out = []
    while len(out) < n:
        a = rng.getrandbits(24)
        if all(bin(a ^ b).count('1') >= min_bits for b in out): out.append(a)
    return out
def track_pos(k, t_ms, n_tracked=64):
    """Position of tracked aircraft k at t_ms: flying with the velocity its TC19 frames give (150 kt W, 120 kt N)."""
    s = t_ms / 1000.0
    lat = F.SITE[0] + 0.02 * (k % n_tracked) + NS_KT * 0.514444 * s / 111320.0
    lon = F.SITE[1] + 0.3 + EW_KT * 0.514444 * s / (111320.0 * math.cos(math.radians(F.SITE[0])))
    return lat, lon

def es_stress(name, seed, rounds, positions_known, kinds, nerr=(3, 8)):
    """64 tracked aircraft (refresh: even + odd position, velocity, ident, DF4, DF5 each round unless positions are not
    known, then ident, DF4 and DF5 only) and 256 of their DF17 frames per round with nerr random bit errors (never in the
    DF field)."""
    rng = random.Random(seed)
    T = spread_addresses(64, seed)   # Host only: no radio, any address.
    alt = {k: 9000 + 100 * (i % 30) for i, k in enumerate(T)}
    def pos(k, t, odd): la, lo = track_pos(T.index(k), t); return encode_airborne_position('%06X' % k, la, lo, alt[k], odd)
    def vel(k): return F.df17_velocity('%06X' % k, ew_kt=EW_KT, ns_kt=NS_KT)
    header(name)
    for rnd in range(rounds):
        t = 1000 + rnd * ROUND_MS
        refresh = []
        for k in T:
            if positions_known: refresh += [pos(k, t, False), pos(k, t, True), vel(k)]
            refresh += [df17_ident(k), df4(k, alt[k]), df5(k, 0o1200 + T.index(k))]
        t = block(t, [('P', h, None) for h in refresh]) + 10
        seq = []
        for i in range(256):
            k = T[rng.randrange(64)]; kind = kinds[i % len(kinds)]
            o = {'pos': lambda: pos(k, t + i, rng.random() < 0.5), 'vel': lambda: vel(k), 'ident': lambda: df17_ident(k)}[kind]()
            seq.append(('B', flipbits(o, rng.sample(range(5, 112), rng.randint(*nerr))), None))
        block(t, seq)

def ap_stress(name, seed, rounds):
    """64 tracked aircraft with known altitude and squawk; 256 of their replies per round (DF0/4/5/16/20/21) with 3-6
    random bit errors."""
    rng = random.Random(seed)
    T = spread_addresses(64, seed)
    alt = {k: 9000 + 100 * (i % 30) for i, k in enumerate(T)}
    sq = {k: 0o1200 + i for i, k in enumerate(T)}
    APF = [lambda k: df0(k, alt[k]), lambda k: df4(k, alt[k]), lambda k: df5(k, sq[k]), lambda k: df16(k, alt[k]),
           lambda k: df20(k, alt[k]), lambda k: df21(k, sq[k])]
    header(name)
    for rnd in range(rounds):
        t = 1000 + rnd * ROUND_MS
        refresh = []
        for k in T: refresh += [df17_ident(k), df4(k, alt[k]), df5(k, sq[k])]
        t = block(t, [('P', h, None) for h in refresh]) + 10
        seq = []
        for i in range(256):
            k = T[rng.randrange(64)]; o = APF[i % 6](k)
            seq.append(('B', flipbits(o, rng.sample(range(5, len(o) * 4), rng.randint(3, 6))), None))
        block(t, seq)

def neighbors(name, seed, rounds, n_tracked, per_tracked, alt_offsets):
    """Clean replies from untracked aircraft exactly 1 address bit from a tracked one, altitude offset from the
    tracked aircraft's by one of alt_offsets, same squawk. n_tracked * per_tracked neighbors, all replying each round."""
    rng = random.Random(seed)
    T = spread_addresses(n_tracked, seed, 6)
    alt = {k: 9000 + 100 * (i % 30) for i, k in enumerate(T)}
    sq = {k: 0o1200 + i for i, k in enumerate(T)}
    N = [(k, k ^ (1 << b)) for k in T for b in range(per_tracked)]
    header(name)
    for rnd in range(rounds):
        t = 1000 + rnd * ROUND_MS
        refresh = []
        for k in T: refresh += [df17_ident(k), df4(k, alt[k]), df5(k, sq[k])]
        t = block(t, [('P', h, None) for h in refresh]) + 10
        seq = []
        for k, u in N:
            f = rng.choice(['df0', 'df4', 'df5', 'df16', 'df20', 'df21'])
            off = rng.choice(alt_offsets)
            h = {'df0': df0, 'df4': df4, 'df16': df16, 'df20': df20}[f](u, alt[k] + off) if f not in ('df5', 'df21') \
                else {'df5': df5, 'df21': df21}[f](u, sq[k])
            seq.append(('B', h, None))
        rng.shuffle(seq)
        block(t, seq)

def good(name, seed, rounds, positions_known, nerr):
    """Cost side: 64 tracked aircraft; per round 256 of their current frames (position, velocity, ident, DF4, DF5) with
    exactly nerr bit errors (never in the DF field). Each is a good update when fixed back to its original."""
    rng = random.Random(seed)
    T = spread_addresses(64, seed)
    alt = {k: 9000 + 100 * (i % 30) for i, k in enumerate(T)}
    sq = {k: 0o1200 + i for i, k in enumerate(T)}
    def pos(k, t, odd): la, lo = track_pos(T.index(k), t); return encode_airborne_position('%06X' % k, la, lo, alt[k], odd)
    def vel(k): return F.df17_velocity('%06X' % k, ew_kt=EW_KT, ns_kt=NS_KT)
    header(name)
    for rnd in range(rounds):
        t = 1000 + rnd * ROUND_MS
        refresh = []
        for k in T:
            if positions_known: refresh += [pos(k, t, False), pos(k, t, True), vel(k)]
            refresh += [df17_ident(k), df4(k, alt[k]), df5(k, sq[k])]
        t = block(t, [('P', h, None) for h in refresh]) + 10
        seq = []
        for i in range(256):
            k = T[rng.randrange(64)]
            kind = ['pos', 'pos', 'vel', 'ident', 'df4', 'df5'][i % 6]
            o = {'pos': lambda: pos(k, t + i, rng.random() < 0.5), 'vel': lambda: vel(k), 'ident': lambda: df17_ident(k),
                 'df4': lambda: df4(k, alt[k]), 'df5': lambda: df5(k, sq[k])}[kind]()
            seq.append(('G', flipbits(o, rng.sample(range(5, len(o) * 4), nerr)), o))
        block(t, seq)

def check(rec_dir, cats, test, key):
    if rec_dir:
        print(test, 'matches', check_recorded(cats, os.path.join(rec_dir, test + '.jsonl'), key), 'recorded rounds',
              file=sys.stderr)

def check_recorded(cats, path, key):
    """The regenerated frames equal the ones the bench sent."""
    rec = [json.loads(l) for l in open(path) if '"step"' in l]
    for r, c in zip(rec, cats):
        want = r[key]
        got = c if key == 'seq' else [list(x) for x in c]
        assert got == [list(x) if isinstance(x, (list, tuple)) else x for x in want], path
    return len(rec)

if __name__ == '__main__':
    out = sys.argv[1]
    REC = sys.argv[sys.argv.index('--check') + 1] if '--check' in sys.argv else None
    ROUNDS = 8   # As in the recorded bench sessions.
    header('wrongx_noalt'); c = wrongx(ROUNDS, False); check(REC, c, 'wrongx', 'cat')
    header('wrongx_alt'); wrongx(ROUNDS, True)
    header('nbr_noalt'); s = nbr(ROUNDS, False); check(REC, s, 'nbr', 'seq')
    header('nbr_alt'); nbr(ROUNDS, True)
    header('ghost'); c = ghost(ROUNDS); check(REC, c, 'ghost', 'cat')
    header('ecc'); c = ecc(ROUNDS); check(REC, c, 'ecc', 'cat')
    if '--bench-only' not in sys.argv:
        R = int(os.environ.get('STRESS_ROUNDS', '400'))
        es_stress('es_bad_tracked', 1224, R, True, ['pos', 'pos', 'vel', 'ident'])
        es_stress('es_bad_no_position', 1225, R, False, ['pos'])
        ap_stress('ap_bad', 1226, R)
        neighbors('nbr1_16x8_alt+-275', 1227, 40, 16, 8, [-275, -200, -100, -25, 25, 100, 200, 275])
        neighbors('nbr1_128x1_alt+-275', 1228, 40, 128, 1, [-275, -200, -100, -25, 25, 100, 200, 275])
        good('good1_tracked', 1229, 100, True, 1)
        good('good2_tracked', 1230, 100, True, 2)
        good('good1_no_position', 1231, 100, False, 1)
        good('good2_no_position', 1232, 100, False, 2)
    open(out, 'w').write('\n'.join(OUT) + '\n')
    print(len(OUT), 'lines', file=sys.stderr)
