"""Model counterpart of real_err.py: e_on vs e_off per frame for TL offset 600 and several (video fc, detector floor)
settings. model_err.py OUT.json"""
import sys, json, numpy as np, multiprocessing as mp
sys.path.insert(0, '/workspace/agents/worker-6/t0878/sim')
import model as M, softdemod_t0870 as sd
N_SAMP = 132 * M.SPU; ST0 = 6 * M.SPU
SETS = [(8e6, 3), (5e6, 3), (8e6, 0), (8e6, 6)]

def run(args):
    vfc, floor, snr, seed = args
    M.DET_FLOOR_DB = floor
    rng = np.random.default_rng(seed)
    # noise floor mean for this setting
    det0 = M.detector(M.add_noise_filter(np.zeros((8, N_SAMP), complex), rng), vfc); nf = det0[:, 1000:].mean()
    R = 40; bits = []; envs = []
    for r in range(R):
        b = M.df17_bits(rng); bits.append(b)
        envs.append(M.envelope(M.chips_of(b), float(ST0), N_SAMP, rng.uniform(.05, .1) * 240, rng.uniform(.05, .2) * 240, 0.0))
    det = M.detector(M.add_noise_filter(M.frontend(np.array(envs), np.full(R, 10 ** (snr / 20)), rng), rng), vfc)
    c = M.comparator_chain(det, nf, 600); out = []; nm = 0
    for r in range(R):
        c48 = c[r, ::5]; m, e = M.matcher_from(c48, ST0 // 5 - 96, ST0 // 5 + 96, M.PATTERN)
        if m < 0: continue
        nm += 1
        s = c48[m + 2::4][:43 * 32]; res = sd.demod(s)
        if not res or not res['crc_ok']: continue
        o, n, b = res['offset'], res['len'], res['bits']
        x = s[o:o + 12 * n].reshape(n, 2, 6)[:, :, 1:5]
        on = np.where(b[:, None] == 1, x[:, 0], x[:, 1]); off = np.where(b[:, None] == 1, x[:, 1], x[:, 0])
        out.append((float(1 - on.mean()), float(off.mean()), snr))
    return vfc, floor, snr, out, nm / R

if __name__ == '__main__':
    jobs = [(v, f, float(s), 100 * int(s) + k) for (v, f) in SETS for s in range(4, 34, 3) for k in range(3)]
    res = {}
    with mp.Pool(3) as p:
        for vfc, floor, snr, out, mr in p.imap_unordered(run, jobs):
            d = res.setdefault(f'{vfc/1e6:.0f}MHz_{floor}dB', {'pts': [], 'match': {}})
            d['pts'] += out; d['match'].setdefault(str(snr), []).append(mr)
    json.dump(res, open(sys.argv[1], 'w'))
    for k, d in res.items():
        a = np.array(d['pts'])
        print(k, 'match rate by snr', {s: round(float(np.mean(v)), 2) for s, v in sorted(d['match'].items(), key=lambda t: float(t[0]))})
