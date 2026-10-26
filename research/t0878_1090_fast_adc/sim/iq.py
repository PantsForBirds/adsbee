"""(c) I/Q sampling instead of the log detector: complex baseband after the SAWs, channel lowpass (+-BW), sampled at
FS_IQ, magnitude in dB (20log10|x|, scaled to mV with the detector slope so the same thresholds machinery works) or
linear magnitude for the bit decisions. Same SNR reference as run.py (noise power in the SAW cascade = 1).
iq.py OUT.json ROWS"""
import sys, json, numpy as np, multiprocessing as mp
from scipy import signal as ss
import model as M
N_SAMP = 132 * M.SPU; ST0 = 6 * M.SPU
VARS = {'iq10_bw3_log': (10, 3e6, 'log'), 'iq10_bw3_lin': (10, 3e6, 'lin'), 'iq10_bw5_log': (10, 5e6, 'log'),
        'iq20_bw5_lin': (20, 5e6, 'lin')}

def sample(y, fs, bw, rng):
    sos = ss.butter(4, bw, fs=M.FS, output='sos'); z = ss.sosfilt(sos, y, axis=1)
    st = int(240 / fs); ph = rng.integers(0, st, y.shape[0]); n = (y.shape[1] - st) // st
    z = np.stack([z[r, ph[r]:ph[r] + n * st:st] for r in range(y.shape[0])])
    return z, ph, st

def to_metric(z, kind):
    p = np.abs(z) ** 2 + 1e-6
    return M.SLOPE_MV_DB * 10 * np.log10(p) if kind == 'log' else np.abs(z)

def calib(rng, rate=1000.0):
    """Preamble threshold per variant for <= rate false triggers/s on noise (score on log magnitude always)."""
    thr = {}
    for name, (fs, bw, kind) in VARS.items():
        scs = []; T = 0
        for b in range(40):
            y = M.add_noise_filter(np.zeros((50, N_SAMP), complex), rng)
            z, ph, st = sample(y, fs, bw, rng); x = to_metric(z, 'log'); spc = fs // 2
            for r in range(50):
                P = np.concatenate([[0.0], np.cumsum(x[r])]); s = M.preamble_scores(P, 1500 // st, len(P) - 20 * spc, spc)
                k = np.flatnonzero(s > 30); last = -99; peaks = []
                for kk in k:
                    if kk - last > spc: peaks.append(s[kk:kk + spc].max())
                    last = kk
                scs += peaks
            T += 50 * (N_SAMP - 1500) / M.FS
        scs = np.sort(scs)[::-1]; nmax = int(rate * T)
        thr[name] = float(scs[nmax]) if nmax < len(scs) else 30.0
    return thr

THR = None

def batch(args):
    snr, R, seed = args; rng = np.random.default_rng(seed)
    bits, envs = [], []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240,
                               rng.uniform(-.05, .05) * 240))
    y = M.add_noise_filter(M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng), rng)
    res = {}; ob = np.zeros(112, np.uint8)
    for name, (fs, bw, kind) in VARS.items():
        z, ph, st = sample(y, fs, bw, rng); xl = to_metric(z, 'log'); xm = to_metric(z, kind); spc = fs // 2; ok = 0
        for r in range(R):
            Pl = np.concatenate([[0.0], np.cumsum(xl[r])]); Pm = np.concatenate([[0.0], np.cumsum(xm[r])])
            lo = (ST0 - 2 * M.SPU) // st; hi = (ST0 + 2 * M.SPU) // st
            sc = M.preamble_scores(Pl, lo, hi + spc, spc); k = 0; found = False
            while k < hi - lo and not found:
                if sc[k] > THR[name]:
                    kb = k + int(np.argmax(sc[k:k + spc])); t = lo + kb
                    if M.adc_demod(Pm, t + 16 * spc - spc // 2, t + 16 * spc + spc // 2, spc, M.SYN112, ob) == 1:
                        found = np.array_equal(ob, bits[r])
                        break
                    k = kb + spc
                else: k += 1
            ok += found
        res[name] = ok
    return snr, R, res

if __name__ == '__main__':
    out, rows = sys.argv[1], int(sys.argv[2])
    THR = calib(np.random.default_rng(5)); print('thr', THR)
    jobs = [(float(s), 50, 9000 + int(10 * s) + k) for s in np.arange(-2, 15, 1) for k in range(rows // 50)]
    tot = {}
    with mp.Pool(3) as p:
        for snr, n, res in p.imap_unordered(batch, jobs):
            d = tot.setdefault(snr, {'n': 0}); d['n'] += n
            for k, v in res.items(): d[k] = d.get(k, 0) + int(v)
    json.dump(dict(thr=THR, data={str(k): v for k, v in sorted(tot.items())}), open(out, 'w'), indent=1)
    for s in sorted(tot): print(s, ' '.join(f'{k}={v/tot[s]["n"]:.3f}' for k, v in tot[s].items() if k != 'n'))
