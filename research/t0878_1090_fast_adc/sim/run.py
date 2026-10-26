"""Frame success vs SNR for today's chain (TL offset sweep) and ADC variants, paired on the same noisy detector output.
run.py OUT.json ROWS_PER_SNR [snr_lo snr_hi step] [video_fc_MHz]"""
import sys, json, numpy as np, multiprocessing as mp
import model as M, fwdemod as F

OFFS = [0, 300, 600, 900, 1200, 1500, 1800]
FSS = [8, 10, 12, 20]
NS = [6, 8, 10]
EXTRA_N = [None, 1, 2, 3, 4]       # at 10 Msps; N=1: 1-bit ADC, threshold at the noise mean (sanity)
HYB = [(10, 8)]
RELAX = [(900, 6), (900, 8), (600, 8), (1200, 8), (1200, 10)]   # (TL offset, mismatch budget) PIO trigger + ADC demod
N_SAMP = 132 * M.SPU
ST0 = 6 * M.SPU
NOISE = json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json')))
NF = NOISE['nf_mean']
RATE = float(__import__('os').environ.get('RATE', 1000))   # soft preamble false triggers/s allowed on noise


def thr_for(fs, rate=RATE):
    d = NOISE['soft'][str(fs)]
    for t in sorted(d, key=lambda k: int(k)):
        if d[t] <= rate: return float(t)
    return float(max(int(k) for k in d))


THR = {fs: thr_for(fs) for fs in FSS}
VIDEO = M.VIDEO_FC


def batch(args):
    snr, R, seed = args
    rng = np.random.default_rng(seed)
    bits, envs = [], []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240,
                               rng.uniform(-.05, .05) * 240))
    sig = M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng)
    det = M.detector(M.add_noise_filter(sig, rng), VIDEO)
    res = {}
    matches = {}
    for o in OFFS:
        c = M.comparator_chain(det, NF, o)
        ok = 0; mt = 0; ms = []
        for r in range(R):
            c48 = c[r, ::5]
            m, e = M.matcher_from(c48, ST0 // 5 - 96, ST0 // 5 + 96, M.PATTERN)
            ms.append(m)
            if m < 0: continue
            mt += 1
            st, bb = F.fw_demod(c48[m + 2::4])
            ok += st == 'ok' and np.array_equal(bb, bits[r])
        res[f'A{o}'] = ok; res[f'A{o}_match'] = mt; matches[o] = ms
    mrel = {}
    for (o, bud) in RELAX:
        c = M.comparator_chain(det, NF, o); mrel[(o, bud)] = []
        for r in range(R):
            m, e = M.matcher_from(c[r, ::5], ST0 // 5 - 96, ST0 // 5 + 96, M.PATTERN, bud)
            mrel[(o, bud)].append(m)
    ob = np.zeros(112, np.uint8)
    variants = [(fs, n) for fs in FSS for n in NS] + [(10, n) for n in EXTRA_N]
    for fs, n in variants:
        if n == 1:
            x, ph, stride = M.adc_path(det, fs, None, rng); x = (x > NF).astype(float)
        else:
            x, ph, stride = M.adc_path(det, fs, n, rng)
        spc = fs // 2
        ok = 0; okh = {o: 0 for o in OFFS}; okc = {2: 0}; okr = {b: 0 for b in RELAX}
        for r in range(R):
            P = np.concatenate([[0.0], np.cumsum(x[r])])
            lo = (ST0 - 2 * M.SPU) // stride; hi = (ST0 + 2 * M.SPU) // stride
            thr = THR[fs] if n != 1 else 0.25
            st, t, ntr = M.soft_search(P, lo, hi, spc, thr, M.SYN112, ob)
            ok += st == 1 and np.array_equal(ob, bits[r])
            if (fs, n) == (8, 8):
                for stp in (2,):
                    st, t, ntr = M.soft_search(P, lo, hi, spc, THR[fs], M.SYN112, ob, stp, THR[fs] - 15)
                    okc[stp] += st == 1 and np.array_equal(ob, bits[r])
            if (fs, n) in HYB:
                for b in RELAX:
                    m = mrel[b][r]
                    if m < 0: continue
                    est = ((m + 22) * 5 - ph[r]) // stride
                    okr[b] += M.adc_demod(P, est - spc, est + spc, spc, M.SYN112, ob) == 1 and np.array_equal(ob, bits[r])
                for o in OFFS:
                    m = matches[o][r]
                    if m < 0: continue
                    est = ((m + 22) * 5 - ph[r]) // stride
                    st2 = M.adc_demod(P, est - spc, est + spc, spc, M.SYN112, ob)
                    okh[o] += st2 == 1 and np.array_equal(ob, bits[r])
        res[f'B{fs}_{n}'] = ok
        if (fs, n) == (8, 8): res['B8_8_step2'] = okc[2]
        if (fs, n) in HYB:
            for b in RELAX: res[f'R{fs}_{n}_{b[0]}_{b[1]}'] = okr[b]
        if (fs, n) in HYB:
            for o in OFFS: res[f'H{fs}_{n}_{o}'] = okh[o]
    return snr, R, res


if __name__ == '__main__':
    out = sys.argv[1]; rows = int(sys.argv[2])
    lo, hi, step = (float(a) for a in sys.argv[3:6]) if len(sys.argv) > 5 else (0, 24, 1)
    if len(sys.argv) > 6:
        VIDEO = float(sys.argv[6]) * 1e6
    snrs = np.arange(lo, hi + 1e-9, step)
    R = 50
    jobs = [(float(s), R, int(1000 * (s + 10)) + k) for s in snrs for k in range(rows // R)]
    tot = {}
    with mp.Pool(3) as p:
        for snr, n, res in p.imap_unordered(batch, jobs):
            d = tot.setdefault(snr, {'n': 0})
            d['n'] += n
            for k, v in res.items(): d[k] = d.get(k, 0) + int(v)
    json.dump(dict(thr=THR, nf=NF, rate=RATE, video=VIDEO, data={str(k): v for k, v in sorted(tot.items())}),
              open(out, 'w'), indent=1)
    for s in sorted(tot):
        d = tot[s]; print(s, d['n'], ' '.join(f'{k}={d[k]/d["n"]:.2f}' for k in d if k != 'n' and not k.endswith('match')))
