"""Reference receivers for the dB budget (t-0877), same frames and noise as sim.py.

ideal(): genie timing, energy per chip (integrate |r|^2 over the chip), PPM bit = larger chip, CRC fix of up to 2
of the 6 weakest bits. Optional filter before it. This is the floor for a non-coherent PPM receiver.

lr2021(): LR2021-like OOK receiver: 3.08 MHz channel filter, envelope sampled once per chip at the chip centre,
16-chip preamble pattern detected with hard chip decisions (threshold at half the pulse amplitude, AGC assumed
ideal, zero chip errors allowed), Manchester data bits by comparing the two chip samples. NF is a parameter.
"""
import numpy as np
import sim

CS = int(sim.FS * 0.5e-6)


def _rx(p_dbm, nf_db, filt, nt, rng, lead=2000):
    ns = lead + int(124e-6 * sim.FS)
    ns += (-ns) % 64
    frames = [sim.make_frame(rng) for _ in range(nt)]
    amp = np.sqrt(10 ** (p_dbm / 10))
    sig = np.zeros((nt, ns), complex)
    for k, b in enumerate(frames):
        sig[k] = amp * sim.envelope(b, ns, lead, 5) * np.exp(1j * rng.uniform(0, 2 * np.pi))
    n0 = 10 ** ((sim.KT + nf_db) / 10)
    var = n0 * sim.FS
    x = sig + (rng.standard_normal((nt, ns)) + 1j * rng.standard_normal((nt, ns))) * np.sqrt(var / 2)
    if filt:
        H = sim.filt_response(filt, ns)
        x = np.fft.ifft(np.fft.fft(x, axis=1) * H, axis=1)
    return x, frames, lead


def _crc_fix_ok(margins, bits_tx):
    bits = (margins > 0).astype(np.uint8)
    err = np.nonzero(bits != bits_tx)[0]
    if len(err) == 0:
        return True
    if len(err) > 2:
        return False
    weak = np.argsort(np.abs(margins))[:6]
    return all(e in weak for e in err)


def ideal(p_dbm, nt, rng, nf_db=1.0, filt=None, gd=0, coherent=True):
    """coherent=True: matched filter (complex sum over the chip, then |.|^2), the non-coherent PPM optimum.
    coherent=False: energy detector (sum of |r|^2 over the chip) behind the given filter."""
    x, frames, lead = _rx(p_dbm, nf_db, filt, nt, rng)
    ok = 0
    for k, b in enumerate(frames):
        s = lead + 16 * CS + gd
        seg = x[k, s:s + 224 * CS].reshape(224, CS)
        ch = np.abs(seg.sum(1)) ** 2 if coherent else (np.abs(seg) ** 2).sum(1)
        m = ch[0::2] - ch[1::2]
        ok += _crc_fix_ok(m, b)
    return ok / nt


def lr2021(p_dbm, nt, rng, nf_db=5.0, filt='bw3.08', gd=None, allow_pre_err=0):
    x, frames, lead = _rx(p_dbm, nf_db, filt, nt, rng)
    env = np.abs(x)
    amp = np.sqrt(10 ** (p_dbm / 10))
    if gd is None:
        # filter group delay ~ peak of the step response; find best sampling point on a noiseless pulse
        H = sim.filt_response(filt, 4096)
        imp = np.zeros(4096); imp[100:100 + CS] = 1
        y = np.abs(np.fft.ifft(np.fft.fft(imp) * H))
        gd = int(np.argmax(y)) - 100 - CS // 2
    pre = np.array([1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0])
    ok = 0
    for k, b in enumerate(frames):
        c = lead + CS // 2 + gd
        smp = env[k, c:c + 240 * CS:CS]
        chips = (smp[:16] > amp / 2).astype(int)
        if np.sum(chips != pre) > allow_pre_err:
            continue
        d = smp[16:240]
        m = d[0::2] - d[1::2]
        ok += _crc_fix_ok(m, b)
    return ok / nt


def p90_of(fn, levels, **kw):
    s = np.array([fn(L, **kw) for L in levels])
    for i in range(1, len(levels)):
        if s[i - 1] < 0.9 <= s[i]:
            return levels[i - 1] + (0.9 - s[i - 1]) / (s[i] - s[i - 1]) * (levels[i] - levels[i - 1]), s
    return np.nan, s
