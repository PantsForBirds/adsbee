"""Two overlapping DF17 frames: A first, B starts 8-120 us after A, B = A + delta dB. Both well above noise.
overlap.py OUT.json ROWS_PER_DELTA [LA_dB] [TL_offset_mV]"""
import sys, json, numpy as np, multiprocessing as mp
import model as M, fwdemod as F
NOISE = json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json'))); NF = NOISE['nf_mean']
FS_ADC = 10; NB = 8
THR = float(min((int(t) for t, v in NOISE['soft'][str(FS_ADC)].items() if v <= 1000), default=60))
N = 262 * M.SPU; STA = 6 * M.SPU
LA = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
OFF = int(sys.argv[4]) if len(sys.argv) > 4 else 1200

def frame_env(rng, start):
    b = M.df17_bits(rng)
    e = M.envelope(M.chips_of(b), float(start), N, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240, 0.0)
    return b, e

def same(x, b): return x is not None and np.array_equal(x, b)

def batch(args):
    delta, R, seed = args
    rng = np.random.default_rng(seed)
    t = np.arange(N) / M.FS
    sig = np.zeros((R, N), complex); A = []; B = []; DT = []
    for r in range(R):
        dt = rng.uniform(8, 120); DT.append(dt)
        ba, ea = frame_env(rng, STA); bb, eb = frame_env(rng, STA + int(dt * M.SPU))
        A.append(ba); B.append(bb)
        for e, lvl in ((ea, LA), (eb, LA + delta)):
            fo = rng.uniform(-2e5, 2e5); ph = rng.uniform(0, 2 * np.pi)
            sig[r] += 10 ** (lvl / 20) * e * np.exp(1j * (2 * np.pi * fo * t + ph))
    det = M.detector(M.add_noise_filter(sig, rng))
    c = M.comparator_chain(det, NF, OFF)
    x, ph, stride = M.adc_path(det, FS_ADC, NB, rng); spc = FS_ADC // 2
    res = dict(a_A=0, a_B=0, h_A=0, h_B=0, b_A=0, b_B=0, b_trig=0, a_caps=0)
    bins = {}
    for r in range(R):
        c48 = c[r, ::5]; P = np.concatenate([[0.0], np.cumsum(x[r])])
        caps = []; tt = STA // 5 - 96; last = -10 ** 9
        while True:
            m, e = M.matcher_from(c48, tt, len(c48) - 1500, M.PATTERN)
            if m < 0: break
            if m - last >= 118 * 48: caps.append(m); last = m
            tt = m + 1
        res['a_caps'] += len(caps)
        okA = okB = hA = hB = False
        for m in caps:
            st, bb = F.fw_demod(c48[m + 2::4])
            if st == 'ok': okA |= same(bb, A[r]); okB |= same(bb, B[r])
            est = ((m + 22) * 5 - ph[r]) // stride; ob = np.zeros(112, np.uint8)
            if M.adc_demod(P, est - spc, est + spc, spc, M.SYN112, ob) == 1:
                hA |= same(ob, A[r]); hB |= same(ob, B[r])
        bo = np.zeros((8, 112), np.uint8); so = np.zeros(8, np.int64)
        nd, ntr = M.soft_search_all(P, (STA - 2 * M.SPU) // stride, len(P) - 240 * spc, spc, THR, M.SYN112, bo, so)
        bA = any(same(bo[i], A[r]) for i in range(nd)); bB = any(same(bo[i], B[r]) for i in range(nd))
        for k, v in (('a_A', okA), ('a_B', okB), ('h_A', hA), ('h_B', hB), ('b_A', bA), ('b_B', bB)): res[k] += int(v)
        res['b_trig'] += ntr
        db = 'early' if DT[r] < 64 else 'late'
        d = bins.setdefault(db, dict(n=0, a_A=0, a_B=0, b_A=0, b_B=0))
        d['n'] += 1
        for k, v in (('a_A', okA), ('a_B', okB), ('b_A', bA), ('b_B', bB)): d[k] += int(v)
    return delta, R, res, bins

if __name__ == '__main__':
    out, rows = sys.argv[1], int(sys.argv[2])
    deltas = list(range(-20, 21, 2)); R = 40
    jobs = [(float(d), R, 7000 + 100 * d + k) for d in deltas for k in range(rows // R)]
    tot = {}
    with mp.Pool(3) as p:
        for d, n, res, bins in p.imap_unordered(batch, jobs):
            T = tot.setdefault(d, dict(n=0, bins={}))
            T['n'] += n
            for k, v in res.items(): T[k] = T.get(k, 0) + v
            for b, dd in bins.items():
                tb = T['bins'].setdefault(b, {})
                for k, v in dd.items(): tb[k] = tb.get(k, 0) + v
    json.dump(dict(LA=LA, off=OFF, thr=THR, data={str(k): v for k, v in sorted(tot.items())}), open(out, 'w'), indent=1)
    for d in sorted(tot):
        T = tot[d]; n = T['n']
        print(d, n, ' '.join(f'{k}={T[k]/n:.2f}' for k in ('a_A', 'a_B', 'h_A', 'h_B', 'b_A', 'b_B', 'b_trig', 'a_caps')))
