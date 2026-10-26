"""Multi-comparator 'flash' front ends sampled at 12 Msps by PIO: K comparators on the detector output with fixed
thresholds at noise floor + T dB (no tracking LEVEL), sample value = number of comparators HI (0..K). Software soft
preamble + amplitude demod as in run.py. Thresholds for the soft preamble are calibrated per variant on noise.
flash.py OUT.json ROWS"""
import sys, json, numpy as np, multiprocessing as mp
import model as M
NOISE = json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json'))); NF = NOISE['nf_mean']
N_SAMP = 132 * M.SPU; ST0 = 6 * M.SPU; FSF = 12; STRIDE = 20; SPC = 6
VARS = {'lvl300': 'L300', 'lvl600': 'L600', 'lvl900': 'L900', 'lvl1200': 'L1200', 'c1_3dB': [3], 'c1_6dB': [6], 'c2_3_8dB': [3, 8], 'c3_2_5_10dB': [2, 5, 10], 'c3_3_7_12dB': [3, 7, 12]}

def quant(det, th_db, rng):
    if isinstance(th_db, str):   # today's comparator (FAST vs tracking LEVEL, TL offset in mV), 1-bit
        v = M.comparator_chain(det, NF, int(th_db[1:])).astype(float)
        ph = rng.integers(0, STRIDE, det.shape[0]); n = (det.shape[1] - STRIDE) // STRIDE
        return np.stack([v[r, ph[r]:ph[r] + n * STRIDE:STRIDE] for r in range(det.shape[0])]), ph
    v = M.lp1(det, M.FAST_FC)
    ph = rng.integers(0, STRIDE, det.shape[0]); n = (det.shape[1] - STRIDE) // STRIDE
    x = np.stack([v[r, ph[r]:ph[r] + n * STRIDE:STRIDE] for r in range(det.shape[0])])
    return sum((x > NF + t * M.SLOPE_MV_DB).astype(float) for t in th_db), ph

def calib(rng, rate=1000.0):
    thr = {}; sc_all = {k: [] for k in VARS}; T = 0
    for b in range(40):
        det = M.detector(M.add_noise_filter(np.zeros((50, N_SAMP), complex), rng))
        for name, th in VARS.items():
            x, ph = quant(det, th, rng)
            for r in range(50):
                P = np.concatenate([[0.0], np.cumsum(x[r])]); s = M.preamble_scores(P, 1500 // STRIDE, len(P) - 20 * SPC, SPC)
                k = np.flatnonzero(s > 0); last = -99
                for kk in k:
                    if kk - last > SPC: sc_all[name].append(s[kk:kk + SPC].max())
                    last = kk
        T += 50 * (N_SAMP - 1500) / M.FS
    for name in VARS:
        s = np.sort(sc_all[name])[::-1]; nmax = int(rate * T)
        thr[name] = float(s[nmax]) + 1e-9 if nmax < len(s) else 0.0
    return thr

THR = None

def batch(args):
    snr, R, seed = args; rng = np.random.default_rng(seed)
    bits, envs = [], []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240,
                               rng.uniform(-.05, .05) * 240))
    det = M.detector(M.add_noise_filter(M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng), rng))
    res = {}; ob = np.zeros(112, np.uint8)
    for name, th in VARS.items():
        x, ph = quant(det, th, rng); ok = 0
        for r in range(R):
            P = np.concatenate([[0.0], np.cumsum(x[r])])
            st, t, ntr = M.soft_search(P, (ST0 - 2 * M.SPU) // STRIDE, (ST0 + 2 * M.SPU) // STRIDE, SPC, THR[name], M.SYN112, ob)
            ok += st == 1 and np.array_equal(ob, bits[r])
        res[name] = ok
    return snr, R, res

if __name__ == '__main__':
    out, rows = sys.argv[1], int(sys.argv[2])
    THR = calib(np.random.default_rng(5)); print('thr', THR)
    jobs = [(float(s), 50, 4000 + int(10 * (s + 10)) + k) for s in np.arange(-2, 21, 1) for k in range(rows // 50)]
    tot = {}
    with mp.Pool(3) as p:
        for snr, n, res in p.imap_unordered(batch, jobs):
            d = tot.setdefault(snr, {'n': 0}); d['n'] += n
            for k, v in res.items(): d[k] = d.get(k, 0) + int(v)
    json.dump(dict(thr=THR, data={str(k): v for k, v in sorted(tot.items())}), open(out, 'w'), indent=1)
    for s in sorted(tot): print(s, ' '.join(f'{k}={v/tot[s]["n"]:.3f}' for k, v in tot[s].items() if k != 'n'))
