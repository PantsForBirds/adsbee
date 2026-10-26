"""Per-frame sample error rates in real t-0870 captures (TL offset 600): on-chip LO fraction vs off-chip HI fraction,
inner 4 samples of each chip, for frames that decode with a clean CRC."""
import sys, json, gzip, numpy as np
sys.path.insert(0, '/workspace/agents/worker-6/t0878/sim')
import softdemod_t0870 as sd
out = []
for path in sys.argv[2:]:
    op = gzip.open(path, 'rt', errors='replace') if path.endswith('.gz') else open(path, errors='replace')
    for l in op:
        p = l.split(' ', 1)
        if len(p) != 2 or not p[1].startswith('#OVD,'): continue
        v = p[1].strip().split(',')[1:]
        if len(v) != 13 or len(v[12]) != 8 * int(v[11]): continue
        s = sd.words_to_samples(v[12]); r = sd.demod(s)
        if not r or not r['crc_ok'] or r['df'] not in (11, 17): continue
        o, n, b = r['offset'], r['len'], r['bits']
        x = s[o:o + 12 * n].reshape(n, 2, 6)[:, :, 1:5]
        on = np.where(b[:, None] == 1, x[:, 0], x[:, 1]); off = np.where(b[:, None] == 1, x[:, 1], x[:, 0])
        out.append((float(1 - on.mean()), float(off.mean()), int(v[10]), n, int(r['flipped'])))
        if len(out) >= 40000: break
json.dump(out, open(sys.argv[1], 'w'))
a = np.array([(e[0], e[1]) for e in out]); print(len(out), 'median e_on %.4f e_off %.4f' % tuple(np.median(a, 0)))
