# Copied read-only from worker-2 t0870 (Python port of mode_s_soft_demod.cc).
"""Python port of firmware/adsbee_1090/pico/application/mode_s_soft_demod.cc (DemodulateBits), plus erasure-aware
variants for the overlap experiments. Samples are a numpy uint8 array of 0/1 at 12 samples per bit."""
import numpy as np

SPB, SPC = 12, 6
GEN = 0x1FFF409


def words_to_samples(hexstr):
    b = bytes.fromhex(hexstr)
    words = [int.from_bytes(b[i:i + 4], 'big') for i in range(0, len(b), 4)]
    out = np.zeros(32 * len(words), np.uint8)
    for i, w in enumerate(words):
        for k in range(32):
            out[32 * i + k] = (w >> (31 - k)) & 1
    return out


def crc24(bits):
    """Remainder of the message bits (list/array of 0/1) incl. parity: 0 for a good DF17."""
    r = 0
    for b in bits:
        r = (r << 1) | int(b)
        if r & 0x1000000:
            r ^= GEN
    return r & 0xFFFFFF


SYN = {}
def syndromes(n):
    if n not in SYN:
        s, out = 1, []
        for i in range(n):
            out.append(s); s <<= 1
            if s & 0x1000000: s ^= GEN
        SYN[n] = out[::-1]  # SYN[n][bit_index]
    return SYN[n]


def margins(samples, offset, nbits, mask=None):
    """Per bit: HI count in chip 1 minus chip 2. With a mask (1 = sample unknown), counts are scaled to the known
    samples of each chip, and a chip with no known samples counts as 0.5 x 6."""
    m = np.zeros(nbits, float)
    for i in range(nbits):
        a = offset + i * SPB
        c1, c2 = samples[a:a + SPC], samples[a + SPC:a + SPB]
        if len(c2) < SPC:
            return m[:i]
        if mask is None:
            m[i] = int(c1.sum()) - int(c2.sum())
        else:
            k1, k2 = 1 - mask[a:a + SPC], 1 - mask[a + SPC:a + SPB]
            v1 = (c1 * k1).sum() / k1.sum() * 6 if k1.sum() else None
            v2 = (c2 * k2).sum() / k2.sum() * 6 if k2.sum() else None
            if v1 is None and v2 is None: m[i] = 0
            elif v1 is None: m[i] = 6 - 2 * v2   # Only chip 2 known: HI there means 0, LO means 1.
            elif v2 is None: m[i] = 2 * v1 - 6
            else: m[i] = v1 - v2
    return m


def demod(samples, mask=None, offsets=range(2, 8), max_flip=2, weak_max=2, max_weak=6, fix_all=False):
    """Returns dict(bits, len, offset, df, crc_ok, flipped) or None. Mirrors DemodulateBits; fix_all also tries the CRC
    fix on address/parity formats against a known address (not used by the firmware)."""
    best, best_conf, best_ties = 5, -1, 99
    for o in offsets:
        m = margins(samples, o, 12, mask)
        if len(m) < 12: continue
        conf, ties = np.abs(m).sum(), int((m == 0).sum())
        if conf > best_conf:
            best, best_conf, best_ties = o, conf, ties
    if best_ties > 3:
        return None
    m = margins(samples, best, 112, mask)
    if len(m) < 8: return None
    bits = (m > 0).astype(np.uint8)
    n = 56 if bits[0] == 0 else 112
    if len(m) < n: return None
    bits, m = bits[:n], m[:n]
    df = int(''.join(map(str, bits[:5])), 2)
    res = dict(bits=bits, len=n, offset=best, df=df, crc_ok=None, flipped=0, margins=m)
    if df not in (11, 17, 18) and not fix_all:
        return res
    r = crc24(bits)
    ok = r == 0 or (df == 11 and r < 0x80)
    if ok:
        res['crc_ok'] = True; return res
    conf = np.abs(m)
    weak = [i for lvl in range(weak_max + 1) for i in range(n) if conf[i] <= lvl and conf[i] > lvl - 1][:max_weak]
    syn = syndromes(n)
    for a in weak:
        if syn[a] == r:
            bits[a] ^= 1; res.update(crc_ok=True, flipped=1); return res
    if max_flip >= 2:
        for i, a in enumerate(weak):
            for b in weak[i + 1:]:
                if syn[a] ^ syn[b] == r:
                    bits[a] ^= 1; bits[b] ^= 1; res.update(crc_ok=True, flipped=2); return res
    res['crc_ok'] = False
    return res


def bits_hex(bits):
    return '%0*X' % (len(bits) // 4, int(''.join(map(str, bits)), 2))


def pulse_mask(bits, offset, nsamples, start, widen=1):
    """Sample mask (1 = covered) of the comparator pulses of a decoded frame, in the frame of a capture whose sample 0
    is `start` samples after the decoded capture's sample 0. Each PPM bit has one 6-sample chip HI; widen extends it by
    `widen` samples on both sides (pulse width varies with level)."""
    mask = np.zeros(nsamples, np.uint8)
    for i, b in enumerate(bits):
        a = offset + i * SPB + (0 if b else SPC) - start
        lo, hi = max(0, a - widen), min(nsamples, a + SPC + widen)
        if hi > lo: mask[lo:hi] = 1
    return mask
