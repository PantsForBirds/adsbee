"""Noise-only comparator edges/s and PIO noise candidates at TL 600 / 1200 for (video fc, detector floor) settings."""
import sys, json, numpy as np
sys.path.insert(0, '/workspace/agents/worker-6/t0878/sim')
import model as M
rng = np.random.default_rng(3); n = 132 * 240; R = 30
out = {}
for floor in (-20, -10, -6, -3, 0, 3):
    for vfc in (8e6, 5e6, 3e6):
        M.DET_FLOOR_DB = floor
        det = M.detector(M.add_noise_filter(np.zeros((R, n), complex), rng), vfc); nf = det[:, 1000:].mean(); sd = det[:, 1000:].std()
        row = {'sd_mV': float(sd)}
        for off in (300, 600, 900, 1200):
            c = M.comparator_chain(det, nf, off)
            row[off] = float(((np.diff(c[:, 1000:].astype(int), axis=1) == 1).sum()) / (R * (n - 1000) / M.FS))
        out[f'{vfc/1e6:.0f}MHz_{floor}dB'] = row
        print(f'{vfc/1e6:.0f}MHz floor {floor:3d} dB  noise sd {sd:5.1f} mV  edges/s ' + ' '.join(f'{o}:{row[o]:9.0f}' for o in (300, 600, 900, 1200)), flush=True)
json.dump(out, open('val/chatter.json', 'w'), indent=1)
