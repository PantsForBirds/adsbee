"""Firmware 1-bit soft demod with the #398 offset retry (branch t0872 mode_s_soft_demod.cc), on top of the t-0870
Python port's bit margins (verified there against the firmware on 20,604 dumps)."""
import numpy as np
import softdemod_t0870 as sd

MIN_OFF, MAX_OFF, MAX_RETRY_OFF, MAX_SHIFT, MAX_RETRIES, MAX_TIES = 2, 7, 9, 2, 3, 3
CAPTURE = 43 * 32
AP = (0, 4, 5, 16, 20, 21)


def margins_fast(s, o, nbits):
    n = (len(s) - o) // 12
    nbits = min(nbits, n)
    x = s[o:o + nbits * 12].reshape(nbits, 12).astype(int)
    return x[:, :6].sum(1) - x[:, 6:].sum(1)


def decode_at(s, o):
    m = margins_fast(s, o, 112)
    if len(m) < 8: return 'none', None
    bits = (m > 0).astype(np.uint8)
    n = 56 if bits[0] == 0 else 112
    if len(m) < n: return 'none', None
    bits, m = bits[:n].copy(), m[:n]
    df = int(''.join(map(str, bits[:5])), 2)
    if df not in (11, 17, 18): return 'nocrc', bits
    r = sd.crc24(bits)
    if r == 0 or (df == 11 and r < 0x80): return 'ok', bits
    conf = np.abs(m)
    weak = [i for lvl in range(3) for i in range(n) if conf[i] == lvl][:6]
    syn = sd.syndromes(n)
    for a in weak:
        if syn[a] == r:
            bits[a] ^= 1; return 'ok', bits
    for i, a in enumerate(weak):
        for b in weak[i + 1:]:
            if syn[a] ^ syn[b] == r:
                bits[a] ^= 1; bits[b] ^= 1; return 'ok', bits
    return 'fail', bits


def fw_demod(s, retry=True):
    """Returns (status, bits): status 'ok' if the CRC passes after fixes/retries."""
    s = s[:CAPTURE]
    conf, ties = {}, {}
    for o in range(MIN_OFF, MAX_RETRY_OFF + 1):
        m = margins_fast(s, o, 12)
        conf[o], ties[o] = int(np.abs(m).sum()), int((m == 0).sum())
    best, bc = 5, 0; bt = 12
    for o in range(MIN_OFF, MAX_OFF + 1):
        if conf[o] > bc: best, bc, bt = o, conf[o], ties[o]
    if bt > MAX_TIES: return 'none', None
    st, bits = decode_at(s, best)
    if st == 'ok' or not retry: return st, bits
    chip_later = st == 'none' or (st == 'nocrc' and int(''.join(map(str, bits[:5])), 2) not in AP)
    tried = {best}
    for _ in range(MAX_RETRIES):
        cand, cc = 0, 0
        for o in range(MIN_OFF, MAX_RETRY_OFF + 1):
            if abs(o - best) > MAX_SHIFT and not (chip_later and o == best + 6): continue
            if o not in tried and ties[o] <= MAX_TIES and conf[o] > cc: cand, cc = o, conf[o]
        if cand == 0: break
        tried.add(cand)
        st2, b2 = decode_at(s, cand)
        if st2 == 'ok': return 'ok', b2
    return st, bits
