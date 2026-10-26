"""Mode S receive chain model for t-0878: today's 1090U chain (AD8313 log detector -> MCP6566 comparator against a
tracking LEVEL -> PIO preamble matcher -> 12 Msps 1-bit capture -> firmware soft demod with the #398 offset retry)
versus an N-bit ADC on the detector output with amplitude demod.

Complex baseband at FS = 240 MHz (a common multiple of 48, 12, 8, 10 and 20 MHz). Units: signal and noise powers are
relative to the noise power after the SAW cascade (= 1), detector output in mV.
"""
import os
import numpy as np
import numba as nb
from scipy import signal as ss

FS = 240e6
SPU = 240            # samples per microsecond
CHIP = 120           # samples per 0.5 us chip
GEN = 0x1FFF409

# ---- Front end constants (1090U rev D schematic, AD8313 datasheet, r1090_rssi.hh calibration) ----
SLOPE_MV_DB = 17.6   # measured detector slope (r1090_rssi.hh kSlopeMVPerdBx10)
DET_FLOOR_DB = float(os.environ.get('DET_FLOOR_DB', 3.0))  # AD8313 floor (~-64 dBm at its input) vs in-band noise (~-67 dBm):
# +3 dB nominal. From BLB01 gain at 1.1 GHz (2 x ~18.5 dB) minus 2 SAWs, and from real units at TL 600 showing almost no
# comparator chatter (t-0878 val/). Swept 0 and +6 dB; -7 dB models 10 dB more LNA gain.
DET_SAT_DB = 56.0    # detector saturates ~-44 dBm at the antenna, noise ~-100 dBm: 56 dB over noise
VIDEO_FC = float(os.environ.get('VIDEO_FC', 8e6))  # AD8313 small-signal response 40 ns 10-90 % (datasheet) -> ~8.8 MHz
FAST_FC = 1 / (2 * np.pi * 1e3 * 3e-12)      # R11 1k / C17 3 pF
LEVEL_TAU = (10e3 * 100e3 / 110e3) * 200e-12  # R13 10k || R9 100k with C16 200 pF = 1.82 us
CMP_HYST_MV = 3.5    # MCP6566 typical input hysteresis

# TFS1090F: flat +-7 MHz, -3 dB ~30 MHz; the cascade of two is 23.7 MHz wide at -3 dB (t-0061 SAW review,
# /library/2026-10-08-narrow-saw-filter-2). Each modelled as a 3rd-order Butterworth lowpass at 13.7 MHz (baseband).
_saw1 = ss.butter(3, 13.7e6, fs=FS, output='sos')
SAW_SOS = np.vstack([_saw1, _saw1])                 # SAW1 and SAW2 in cascade
_imp = np.zeros(20000); _imp[0] = 1
SAW_NOISE_GAIN = float(np.sum(ss.sosfilt(SAW_SOS, _imp) ** 2))   # white noise power gain per unit input variance
NOISE_BW_HZ = SAW_NOISE_GAIN * FS                   # equivalent noise bandwidth of the cascade (complex baseband)


def df17_bits(rng):
    b = np.zeros(112, np.uint8)
    b[:5] = [1, 0, 0, 0, 1]; b[5:8] = [1, 0, 1]
    b[8:88] = rng.integers(0, 2, 80)
    r = crc24(b[:88].tolist() + [0] * 24)
    for k in range(24):
        b[88 + k] = (r >> (23 - k)) & 1
    return b


def crc24(bits):
    r = 0
    for x in bits:
        r = (r << 1) | int(x)
        if r & 0x1000000:
            r ^= GEN
    return r & 0xFFFFFF


def _syn(n):
    s, out = 1, []
    for i in range(n):
        out.append(s); s <<= 1
        if s & 0x1000000: s ^= GEN
    return np.array(out[::-1], np.int64)
SYN112, SYN56 = _syn(112), _syn(56)


def chips_of(bits):
    """Chip on/off sequence: 16 preamble chips then 2 per bit."""
    pre = np.zeros(16, np.uint8); pre[[0, 2, 7, 9]] = 1
    d = np.zeros(2 * len(bits), np.uint8)
    d[0::2] = bits; d[1::2] = 1 - bits
    return np.concatenate([pre, d])


@nb.njit(cache=True)
def envelope(chips, start, n, tr, tf, width_err):
    """Trapezoid pulses (DO-260B / Annex 10: rise 0.05-0.1 us, decay 0.05-0.2 us, 10-90 %), 50 % points on the chip
    edges, consecutive on-chips merged into one pulse. width_err stretches each pulse's end (samples)."""
    env = np.zeros(n)
    Tr = tr / 0.8; Tf = tf / 0.8
    k = 0; nc = len(chips)
    while k < nc:
        if chips[k] == 0:
            k += 1; continue
        j = k
        while j + 1 < nc and chips[j + 1] == 1:
            j += 1
        s = start + k * 120.0; e = start + (j + 1) * 120.0 + width_err
        a = int(max(0, s - Tr / 2)); b = int(min(n, e + Tf / 2 + 1))
        for t in range(a, b):
            up = (t - (s - Tr / 2)) / Tr
            dn = ((e + Tf / 2) - t) / Tf
            v = min(1.0, up, dn)
            if v > 0:
                env[t] = max(env[t], v)
        k = j + 1
    return env


def frontend(rows_env, amps, rng, fo_max=200e3):
    """rows_env: (R, n) unit envelopes (already summed per row if several frames: pass complex if you need phases).
    Returns detector output det (R, n) in mV relative to 10log10(noise)=0 -> 0 mV."""
    R, n = rows_env.shape
    t = np.arange(n) / FS
    fo = rng.uniform(-fo_max, fo_max, (R, 1)); ph = rng.uniform(0, 2 * np.pi, (R, 1))
    sig = (amps[:, None] * rows_env) * np.exp(1j * (2 * np.pi * fo * t + ph))
    return sig


def add_noise_filter(sig, rng, noise=True):
    R, n = sig.shape
    if noise:
        sd = np.sqrt(1.0 / SAW_NOISE_GAIN / 2)
        x = sig + sd * (rng.standard_normal((R, n)) + 1j * rng.standard_normal((R, n)))
    else:
        x = sig
    return ss.sosfilt(SAW_SOS, x, axis=1)


def detector(y, video_fc=VIDEO_FC):
    p = np.abs(y) ** 2 + 10 ** (DET_FLOOR_DB / 10)
    v = SLOPE_MV_DB * 10 * np.log10(p)
    v = np.minimum(v, SLOPE_MV_DB * DET_SAT_DB)
    return lp1(v, video_fc)


def lp1(v, fc, init=None):
    a = np.exp(-2 * np.pi * fc / FS)
    zi = None
    if init is None:
        init = v[:, :1]
    zi = (init * a)
    out, _ = ss.lfilter([1 - a], [1, -a], v, axis=1, zi=zi)
    return out


@nb.njit(cache=True)
def comparator(fast, level, hyst):
    n = len(fast); out = np.zeros(n, np.uint8); st = 0
    for i in range(n):
        d = fast[i] - level[i]
        if st == 0 and d > hyst / 2: st = 1
        elif st == 1 and d < -hyst / 2: st = 0
        out[i] = st
    return out


def comparator_chain(det, nf_mv, offset_mv):
    """FAST (R11/C17) vs LEVEL = RC(1.82 us) of (det*10/11 + bias/11), bias = noise floor (ADC) + TL offset."""
    fast = lp1(det, FAST_FC)
    bias = nf_mv + offset_mv
    src = det * (100 / 110) + bias * (10 / 110)
    fc = 1 / (2 * np.pi * LEVEL_TAU)
    level = lp1(src, fc, init=np.full((det.shape[0], 1), nf_mv * 100 / 110 + bias * 10 / 110))
    out = np.empty(det.shape, np.uint8)
    for r in range(det.shape[0]):
        out[r] = comparator(fast[r], level[r], CMP_HYST_MV)
    return out


# PIO preamble matcher (capture.pio), at 48 MHz. Pattern chips 0..14: 1 = HI, 0 = LO, -1 = skip.
PATTERN = np.array([1, -1, 1, -1, 0, 0, -1, 1, -1, 1, -1, 0, 0, 0, 0], np.int8)


@nb.njit(cache=True)
def matcher_from(c48, t0, t_end, pattern, budget=4, nsm=4):
    """Round-robin matchers from cycle t0. Returns the cycle at which the first match sets the demod pin, or -1.
    Also returns the number of candidate edges examined."""
    free = np.zeros(nsm, np.int64)
    for k in range(nsm): free[k] = t0
    h = 0; t = t0; n = len(c48); edges = 0
    while t < t_end:
        if free[h] > t: t = free[h]
        # wait 0 then wait 1: next rising edge at or after t
        while t < n - 1 and c48[t] != 0: t += 1
        while t < n - 1 and c48[t] == 0: t += 1
        if t >= t_end or t >= n - 400: return -1, edges
        e = t; edges += 1
        y = budget; delay = 0; ok = True; tc = e + 4
        for c in range(15):
            pc = pattern[c]
            if pc >= 0:
                for s in range(4):
                    ts = tc + 5 + 4 * s + delay
                    if c48[ts] != pc:
                        if y == 0:
                            ok = False; break
                        y -= 1; delay += 1
                if not ok:
                    free[h] = tc + 5 + 4 * 4 + delay + 3
                    break
            tc += 24
        if ok:
            return tc + delay, edges
        h = (h + 1) % nsm
        t = e + 2   # next matcher hunts for the next edge after this one
    return -1, edges


# ---------------- ADC amplitude demod (dump1090/readsb style) ----------------
@nb.njit(cache=True)
def _crc(bits, n):
    r = 0
    for i in range(n):
        r = (r << 1) | bits[i]
        if r & 0x1000000: r ^= 0x1FFF409
    return r & 0xFFFFFF


@nb.njit(cache=True)
def adc_decode_at(P, o, spc, syn, out_bits):
    """Bits from prefix sums P at data start o (samples). Returns status 0 none, 1 crc ok, 2 crc fail;
    out_bits gets the bits. Fix: up to 2 flips among the 6 least confident bits."""
    n = 112
    if o + 2 * n * spc >= len(P) or o < 0: return 0
    m = np.empty(n)
    for i in range(n):
        a = o + 2 * i * spc
        m[i] = (P[a + spc] - P[a]) - (P[a + 2 * spc] - P[a + spc])
        out_bits[i] = 1 if m[i] > 0 else 0
    if out_bits[0] == 0:
        n = 56
    r = _crc(out_bits, n)
    if r == 0: return 1
    sy = syn if n == 112 else syn[56:]
    idx = np.argsort(np.abs(m[:n]))[:6]
    for a in range(6):
        if sy[idx[a]] == r:
            out_bits[idx[a]] ^= 1; return 1
    for a in range(6):
        for b in range(a + 1, 6):
            if sy[idx[a]] ^ sy[idx[b]] == r:
                out_bits[idx[a]] ^= 1; out_bits[idx[b]] ^= 1; return 1
    return 2


@nb.njit(cache=True)
def adc_demod(P, lo, hi, spc, syn, out_bits, retries=3, shift=-1):
    """Timing search over data starts lo..hi (first 12 bits' confidence), decode, then retry the next most confident
    offsets within `shift` samples of the pick (default spc//2) after a CRC fail. Returns status as adc_decode_at."""
    if shift < 0: shift = max(1, spc // 2)
    nof = hi - lo + 1
    conf = np.zeros(nof)
    for k in range(nof):
        o = lo + k
        if o < 0 or o + 24 * spc >= len(P): conf[k] = -1; continue
        s = 0.0
        for i in range(12):
            a = o + 2 * i * spc
            s += abs((P[a + spc] - P[a]) - (P[a + 2 * spc] - P[a + spc]))
        conf[k] = s
    best = np.argmax(conf)
    st = adc_decode_at(P, lo + best, spc, syn, out_bits)
    if st == 1 or retries == 0: return st
    tried = np.zeros(nof, np.bool_); tried[best] = True
    for r in range(retries):
        cb = -1.0; kb = -1
        for k in range(nof):
            if not tried[k] and abs(k - best) <= shift and conf[k] > cb:
                cb = conf[k]; kb = k
        if kb < 0: break
        tried[kb] = True
        tmp = out_bits.copy()
        s2 = adc_decode_at(P, lo + kb, spc, syn, tmp)
        if s2 == 1:
            out_bits[:] = tmp; return 1
    return st


@nb.njit(cache=True)
def preamble_scores(P, lo, hi, spc):
    """Soft preamble score at each start t: min(pulse chips 0,2,7,9) - mean(quiet chips 4,5,11..14), chip means."""
    out = np.full(hi - lo, -1e9)
    for k in range(hi - lo):
        t = lo + k
        if t < 0 or t + 16 * spc >= len(P): continue
        c = np.empty(16)
        for j in range(16):
            c[j] = (P[t + (j + 1) * spc] - P[t + j * spc]) / spc
        q = (c[4] + c[5] + c[11] + c[12] + c[13] + c[14]) / 6
        p = min(min(c[0], c[2]), min(c[7], c[9]))
        out[k] = p - q
    return out


@nb.njit(cache=True)
def soft_search(P, lo, hi, spc, thr, syn, out_bits, step=1, thr_coarse=-1e9):
    """Full software receiver on the ADC stream: scan starts lo..hi; at each score above thr, refine within one chip
    to the best score, decode at data start = t + 16 chips (+-spc/2). Returns (status, t_found, ntriggers).
    step > 1: the scan looks only at every step-th start (score > thr_coarse), then refines at full resolution
    within +-step and requires thr there (the CPU-cheap scan)."""
    sc = preamble_scores(P, lo, hi + spc, spc)
    k = 0; ntrig = 0
    n = hi - lo
    if step > 1:
        while k < n:
            if k % step == 0 and sc[k] > thr_coarse:
                a = max(0, k - step); b = min(n, k + step + 1)
                kb = a
                for j in range(a, b):
                    if sc[j] > sc[kb]: kb = j
                if sc[kb] > thr:
                    ntrig += 1
                    t = lo + kb
                    st = adc_demod(P, t + 16 * spc - spc // 2, t + 16 * spc + spc // 2, spc, syn, out_bits)
                    if st == 1:
                        return 1, t, ntrig
                    k = kb + spc
                    continue
            k += 1
        return 0, -1, ntrig
    while k < n:
        if sc[k] > thr:
            ntrig += 1
            kb = k
            for j in range(k, min(k + spc, n)):
                if sc[j] > sc[kb]: kb = j
            t = lo + kb
            st = adc_demod(P, t + 16 * spc - spc // 2, t + 16 * spc + spc // 2, spc, syn, out_bits)
            if st == 1:
                return 1, t, ntrig
            k = kb + spc   # next candidate at least a chip later
        else:
            k += 1
    return 0, -1, ntrig


def quantize(v, nbits, lo_mv, hi_mv):
    if nbits is None or nbits <= 0: return v
    q = (hi_mv - lo_mv) / (2 ** nbits)
    c = np.clip(np.floor((v - lo_mv) / q), 0, 2 ** nbits - 1)
    return lo_mv + (c + 0.5) * q


def adc_path(det, fs_msps, nbits, rng, aa_frac=0.4, lo_db=-15, hi_db=DET_SAT_DB + 2):
    """Anti-alias (2-pole Butterworth at aa_frac*fs), sample at fs with a random phase per row, quantize."""
    sos = ss.butter(2, aa_frac * fs_msps * 1e6, fs=FS, output='sos')
    v = ss.sosfilt(sos, det, axis=1)
    stride = int(round(240 / fs_msps))
    R = det.shape[0]
    ph = rng.integers(0, stride, R)
    n = (det.shape[1] - stride) // stride
    x = np.stack([v[r, ph[r]:ph[r] + n * stride:stride] for r in range(R)])
    return quantize(x, nbits, SLOPE_MV_DB * lo_db, SLOPE_MV_DB * hi_db), ph, stride


@nb.njit(cache=True)
def soft_search_all(P, lo, hi, spc, thr, syn, bits_out, starts_out, stronger_mv=3 * 17.6):
    """As soft_search, but keeps searching after a decode (from one chip after its preamble), so a stronger frame
    that starts inside another is still found. While inside a decoded frame, a candidate needs its weakest preamble
    pulse stronger than that frame's preamble pulses by stronger_mv (readsb-style), which stops the frame's own
    data from retriggering. Returns (n_decodes, n_triggers)."""
    sc = preamble_scores(P, lo, hi + spc, spc)
    k = 0; ntrig = 0; nd = 0; n = hi - lo
    busy_until = -1; busy_level = 0.0
    ob = np.zeros(112, np.uint8)
    while k < n and nd < bits_out.shape[0]:
        if sc[k] > thr:
            kb = k
            for j in range(k, min(k + spc, n)):
                if sc[j] > sc[kb]: kb = j
            t = lo + kb
            pmin = 1e9
            for c in (0, 2, 7, 9):
                v = (P[t + (c + 1) * spc] - P[t + c * spc]) / spc
                if v < pmin: pmin = v
            if t < busy_until and pmin < busy_level + stronger_mv:
                k = kb + spc
                continue
            ntrig += 1
            st = adc_demod(P, t + 16 * spc - spc // 2, t + 16 * spc + spc // 2, spc, syn, ob)
            if st == 1:
                bits_out[nd, :] = ob; starts_out[nd] = t; nd += 1
                nb_ = 112 if ob[0] == 1 else 56
                busy_until = t + (16 + 2 * nb_) * spc; busy_level = pmin
            k = kb + spc
        else:
            k += 1
    return nd, ntrig
