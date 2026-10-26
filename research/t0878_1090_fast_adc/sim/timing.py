"""MLAT timing and RSSI per frame: today's comparator mid-bit-edge estimate (port of DataStart48MHzCounts) vs an ADC
template fit. Uses frames that decode in both chains; truth = known data start. timing.py OUT.json ROWS"""
import sys, json, numpy as np, multiprocessing as mp
import model as M, fwdemod as F
NF = json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json')))['nf_mean']; OFF = 900
N_SAMP = 132 * M.SPU; ST0 = 6 * M.SPU; DATA0 = ST0 + 16 * M.CHIP

def fw_time(s, bits, o):
    """DataStart48MHzCounts: start of data in 48 MHz counts after the first sample."""
    fs = rs = fb = rb = 0
    for i in range(24):
        w = s[o + 12 * i + 3:o + 12 * i + 9].sum()
        if bits[i]: fs += w; fb += 1
        else: rs += w; rb += 1
    if fb == 0 or rb == 0: return o * 4 + 3
    return o * 4 + 2 * (fs / fb - rs / rb)

def adc_time(x, bits, est, stride, spc):
    """Correlate the ADC samples with the decoded chip pattern (+-1 for on/off chips, 0 elsewhere), on a 240 MHz grid
    of trial starts; parabolic peak. Returns the data start in 240 MHz samples (row coordinates)."""
    ch = np.repeat(np.where(np.r_[np.c_[bits, 1 - bits].ravel()] > 0, 1.0, -1.0), M.CHIP)
    L = len(ch); best = None
    taus = np.arange(est - 2 * stride, est + 2 * stride + 1)
    S = np.empty(len(taus))
    for k, tau in enumerate(taus):
        # sample i is at row time ph + i*stride (folded into est by caller): index into template
        idx = np.arange(len(x)) * stride - tau
        ok = (idx >= 0) & (idx < L)
        S[k] = np.dot(x[ok], ch[idx[ok]])
    k = int(np.argmax(S))
    if 0 < k < len(S) - 1:
        d = 0.5 * (S[k - 1] - S[k + 1]) / (S[k - 1] - 2 * S[k] + S[k + 1])
    else: d = 0
    return taus[k] + d

def edge_time(x, ts, bits, t0, iters=2, nchips=112):
    """Refine a coarse data start t0 (row coords) from mid-level crossings of the ADC samples at the chip boundaries
    with a transition (first nchips chips); rising and falling edges averaged so pulse width and rise/fall
    asymmetry cancel. Returns (t, hi_level_mV)."""
    on = np.r_[np.c_[bits, 1 - bits].ravel()].astype(bool)
    hi = lo = 0.0
    for _ in range(iters):
        cidx = np.floor((ts - t0) / M.CHIP).astype(int)
        frac = (ts - t0) / M.CHIP - cidx
        inner = (frac > 0.25) & (frac < 0.75) & (cidx >= 0) & (cidx < 224)
        hi = np.median(x[inner & on[np.clip(cidx, 0, 223)]]); lo = np.median(x[inner & ~on[np.clip(cidx, 0, 223)]])
        mid = (hi + lo) / 2
        er, ef = [], []
        for k in range(1, nchips):
            if on[k - 1] == on[k]: continue
            te = t0 + k * M.CHIP
            i = int(np.searchsorted(ts, te - 60))
            while i + 1 < len(ts) and ts[i] < te + 60:
                a, b = x[i] - mid, x[i + 1] - mid
                if (on[k] and a < 0 <= b) or (not on[k] and a >= 0 > b):
                    tc = ts[i] + (ts[i + 1] - ts[i]) * a / (a - b)
                    (er if on[k] else ef).append(tc - te); break
                i += 1
        if er and ef: t0 = t0 + (np.mean(er) + np.mean(ef)) / 2
    return t0, hi


def batch(args):
    snr, R, seed = args; rng = np.random.default_rng(seed)
    bits, envs = [], []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240, 0.0))
    det = M.detector(M.add_noise_filter(M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng), rng))
    c = M.comparator_chain(det, NF, OFF)
    out = dict(fw=[], adc8=[], adc20=[], rssi8=[], rssi_lp=[])
    lp = M.lp1(det, 1 / (2 * np.pi * 10e-6))   # RSSI pin: 10k / 1 nF
    X = {fs: M.adc_path(det, fs, 8, rng) for fs in (8, 20)}
    for r in range(R):
        c48 = c[r, ::5]; m, e = M.matcher_from(c48, ST0 // 5 - 96, ST0 // 5 + 96, M.PATTERN)
        if m < 0: continue
        s = c48[m + 2::4]; st, bb = F.fw_demod(s)
        if st != 'ok' or not np.array_equal(bb, bits[r]): continue
        # find the offset the firmware used: best confidence among 2..7 as in fw_demod
        conf = {o: np.abs(F.margins_fast(s, o, 12)).sum() for o in range(2, 10)}
        o = max(range(2, 8), key=lambda q: (conf[q], -q))
        smp = lp[r, min(N_SAMP - 1, m * 5 + 43 * M.SPU)]
        out['rssi_lp'].append(float((NF + (smp - NF) / 0.53) / M.SLOPE_MV_DB - snr))   # r1090_rssi.hh method
        t48 = (m + 2) + fw_time(s, bits[r], o)          # 48 MHz counts from row start
        out['fw'].append(t48 * 5 - DATA0)              # 240 MHz samples error (bias removed later)
        for fs in (8, 20):
            x, ph, stride = X[fs]; spc = fs // 2
            xr = x[r] - np.median(x[r])
            est = DATA0 - ph[r]
            tc = adc_time(xr, bits[r], est, stride, spc) + ph[r]
            ts = ph[r] + np.arange(len(x[r])) * stride
            t, hi = edge_time(x[r], ts, bits[r], tc)
            out[f'adc{fs}'].append(t - DATA0)
            if fs == 8:
                out['rssi8'].append(float(hi / M.SLOPE_MV_DB - snr))   # pulse level, dB above noise power
    return snr, out

if __name__ == '__main__':
    outp, rows = sys.argv[1], int(sys.argv[2])
    jobs = [(float(s), 50, 500 + int(s) * 10 + k) for s in (10, 12, 15, 20, 30, 40) for k in range(rows // 50)]
    tot = {}
    with mp.Pool(3) as p:
        for snr, o in p.imap_unordered(batch, jobs):
            d = tot.setdefault(snr, {k: [] for k in o})
            for k, v in o.items(): d[k] += list(v)
    res = {}
    for s in sorted(tot):
        d = tot[s]; res[str(s)] = {k: dict(n=len(v), bias=float(np.mean(v)) if v else None,
                                      rms_ns=float(np.std(v) / 0.24) if not k.startswith('rssi') and v else None,
                                      sd_db=float(np.std(v)) if k.startswith('rssi') and v else None) for k, v in d.items()}
        print(s, {k: (v['n'], round(v['rms_ns'], 1) if v['rms_ns'] else None, round(v['sd_db'], 2) if v['sd_db'] else None,
                      round(v['bias'], 2) if v['bias'] is not None else None) for k, v in res[str(s)].items()})
    json.dump(res, open(outp, 'w'), indent=1)
