"""Today's chain only (1-bit comparator, PIO matcher, firmware demod with #398 retry): TL offset x mismatch budget.
a_sweep.py OUT.json ROWS"""
import sys, json, numpy as np, multiprocessing as mp
import model as M, fwdemod as F
NF = json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json')))['nf_mean']
N_SAMP = 132 * M.SPU; ST0 = 6 * M.SPU
CFG = [(o, b) for o in (0, 300, 450, 600, 750, 900, 1200) for b in (4,)] + [(o, b) for o in (300, 600, 900) for b in (6, 8, 10, 12)]

def batch(args):
    snr, R, seed = args; rng = np.random.default_rng(seed)
    bits, envs = [], []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240,
                               rng.uniform(-.05, .05) * 240))
    det = M.detector(M.add_noise_filter(M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng), rng))
    res = {}
    for o in sorted({c[0] for c in CFG}):
        c = M.comparator_chain(det, NF, o)
        for (oo, bud) in CFG:
            if oo != o: continue
            ok = 0
            for r in range(R):
                c48 = c[r, ::5]; m, e = M.matcher_from(c48, ST0 // 5 - 96, ST0 // 5 + 96, M.PATTERN, bud)
                if m < 0: continue
                st, bb = F.fw_demod(c48[m + 2::4]); ok += st == 'ok' and np.array_equal(bb, bits[r])
            res[f'A{o}_b{bud}'] = ok
    return snr, R, res

if __name__ == '__main__':
    out, rows = sys.argv[1], int(sys.argv[2])
    jobs = [(float(s), 50, 3000 + int(10 * (s + 10)) + k) for s in np.arange(0, 21, 1) for k in range(rows // 50)]
    tot = {}
    with mp.Pool(3) as p:
        for snr, n, res in p.imap_unordered(batch, jobs):
            d = tot.setdefault(snr, {'n': 0}); d['n'] += n
            for k, v in res.items(): d[k] = d.get(k, 0) + int(v)
    json.dump(dict(data={str(k): v for k, v in sorted(tot.items())}), open(out, 'w'), indent=1)
    for s in sorted(tot): print(s, ' '.join(f'{k}={v/tot[s]["n"]:.2f}' for k, v in tot[s].items() if k != 'n'))
