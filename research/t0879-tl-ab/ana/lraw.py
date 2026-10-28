"""Comparator timeline from AT+LABRAW dumps: split HI duty and rising edges into on-signal vs between-frames.

A HI run >= LONG_US is pulse-like (Mode S pulses are 0.5/1.0 us; noise runs are tens of ns). Samples from PRE_US
before to POST_US after any pulse-like run are 'on-signal'; the rest is 'between frames'."""
import numpy as np

LONG_US, PRE_US, POST_US = 0.35, 10.0, 125.0


def parse(lines):
    """lines: console lines (without the time prefix). Yields (us, bits uint8 array)."""
    cur = None
    for l in lines:
        if l.startswith('#LRAW,'):
            _, n, us = l.strip().split(',')
            cur = [int(n), int(us), []]
        elif l.startswith('#LRW,') and cur is not None:
            cur[2].append(l.strip().split(',')[2])
            if sum(len(h) for h in cur[2]) >= cur[0] * 8:
                h = ''.join(cur[2])[:cur[0] * 8]
                b = np.unpackbits(np.frombuffer(bytes.fromhex(h), dtype=np.uint8))  # MSB first = first read.
                yield cur[1], b
                cur = None


def runs(b):
    """HI runs: (start, length) in samples."""
    d = np.diff(np.concatenate(([0], b.astype(np.int8), [0])))
    s = np.flatnonzero(d == 1)
    e = np.flatnonzero(d == -1)
    return s, e - s


def split(us, b):
    n = len(b)
    ts = us / n  # us per sample
    s, L = runs(b)
    sig = np.zeros(n, bool)
    for st, ln in zip(s[L * ts >= LONG_US], L[L * ts >= LONG_US]):
        sig[max(0, st - int(PRE_US / ts)):min(n, st + ln + int(POST_US / ts))] = True
    rises = np.zeros(n, bool)
    rises[s] = True
    if b[0]:
        rises[0] = False  # Run already HI at the start is not an edge.
    q = ~sig
    return dict(ts=ts, n=n, n_sig=int(sig.sum()), n_q=int(q.sum()), hi_sig=int(b[sig].sum()), hi_q=int(b[q].sum()),
                r_sig=int(rises[sig].sum()), r_q=int(rises[q].sum()), hi=int(b.sum()), r=int(rises.sum()),
                qruns=L[q[s]] * ts)


def summarize(shots):
    """shots: list of split() dicts. Duty and edges/s overall, on-signal, between-frames."""
    tot = {k: sum(x[k] for x in shots) for k in ('n', 'n_sig', 'n_q', 'hi_sig', 'hi_q', 'r_sig', 'r_q', 'hi', 'r')}
    ts = np.mean([x['ts'] for x in shots])
    f = lambda a, b: a / b if b else float('nan')
    qr = np.concatenate([x['qruns'] for x in shots]) if shots else np.array([])
    return dict(shots=len(shots), ns_per_read=ts * 1e3, seconds=tot['n'] * ts * 1e-6,
                sig_frac=f(tot['n_sig'], tot['n']),
                duty=f(tot['hi'], tot['n']), duty_sig=f(tot['hi_sig'], tot['n_sig']), duty_q=f(tot['hi_q'], tot['n_q']),
                edges_s=f(tot['r'], tot['n'] * ts * 1e-6), edges_s_sig=f(tot['r_sig'], tot['n_sig'] * ts * 1e-6),
                edges_s_q=f(tot['r_q'], tot['n_q'] * ts * 1e-6),
                qrun_med_ns=float(np.median(qr) * 1e3) if len(qr) else float('nan'))


if __name__ == '__main__':
    # Self test: noise spikes + one 112-bit frame.
    rng = np.random.default_rng(1)
    ts = 0.04; n = 512 * 32
    b = np.zeros(n, np.uint8)
    for st in rng.integers(0, n - 3, 300): b[st:st + rng.integers(1, 3)] = 1
    t0 = 100 / ts
    for c in [0, 2, 7, 9] + [16 + 2 * i + rng.integers(0, 2) for i in range(112)]:
        a = int(t0 + c * 0.5 / ts); b[a:a + int(0.5 / ts)] = 1
    hexs = np.packbits(b).tobytes().hex()
    lines = [f'#LRAW,512,{int(n * ts)}'] + [f'#LRW,{i},{hexs[i * 8:(i + 32) * 8]}' for i in range(0, 512, 32)]
    sh = [split(us, bb) for us, bb in parse(lines)]
    print(summarize(sh))
