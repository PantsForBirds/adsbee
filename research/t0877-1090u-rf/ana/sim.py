"""Time-domain Monte Carlo model of the ADSBee 1090U receive chain (t-0877).

antenna noise + Mode S frame -> SAW cascade -> AD8313 log detector (log of envelope power, 2-pole video) ->
FAST (R11/C17) and LEVEL (R13/R9/C16, D7) nets -> MCP6566 comparator with hysteresis -> 48 MHz PIO preamble
matcher ring (4 SMs, capture.pio) -> 12 Msps sampler -> soft demod (port of mode_s_soft_demod.cc) with the
#398 offset retry.

All powers are referred to the antenna connector (dBm). The detector law comes from the firmware calibration
(r1090_rssi.hh: 17.6 mV/dB, 1137 mV at -80 dBm CW): V = 17.6 mV * (P + 144.6).
"""
import numpy as np, numba as nb
import softdemod as sd

FS = 96e6                    # complex baseband sample rate for the analog part
SUB = 4                      # ODE sub-steps per sample (dt = 2.6 ns)
SLOPE = 17.6                 # mV/dB
V0DBM = -144.6               # dBm where V = 0
KT = -174.0                  # dBm/Hz at 290 K

# TFS1090F |S21| loss in dB vs offset from 1090 MHz (MHz), from the t-0061 extraction (results.json) plus the
# cascade -3 dB points 1077.9 / 1101.6 MHz.
TFS = np.array([[-112, 48.9], [-60, 62.2], [-30, 42.9], [-25, 49.8], [-15, 6.41], [-12.1, 3.08], [-10, 2.41],
                [-6, 1.45], [0, 1.58], [6, 1.88], [10, 2.36], [11.6, 3.08], [15, 9.30], [25, 58.4], [30, 61.0],
                [60, 69.7]])
TA3003 = np.array([[-112, 55], [-30, 55], [-15, 47.3], [-10, 32.3], [-6, 7.37], [-5.5, 4.2], [0, 2.81], [5.8, 4.2],
                   [6, 5.2], [10, 27.8], [15, 60.4], [25, 64.7], [60, 55]])


def saw_db(table, f_mhz):
    return np.interp(f_mhz, table[:, 0], table[:, 1]) - np.interp(0, table[:, 0], table[:, 1])


def filt_response(kind, n):
    """Complex baseband frequency response on the FFT grid (linear phase)."""
    f = np.fft.fftfreq(n, 1 / FS) / 1e6
    if kind == 'tfs2':
        db = 2 * saw_db(TFS, f)
    elif kind == 'tfs+ta3003':
        db = saw_db(TFS, f) + saw_db(TA3003, f)
    elif kind == 'ta3003x2':
        db = 2 * saw_db(TA3003, f)
    elif kind.startswith('gauss'):           # gaussN: Gaussian with -3 dB BW of N MHz
        bw = float(kind[5:])
        db = 3.0 * (2 * f / bw) ** 2
    elif kind.startswith('bw'):              # bwN: 4th-order Butterworth, -3 dB BW N MHz (two-sided)
        bw = float(kind[2:])
        db = 10 * np.log10(1 + (2 * f / bw) ** 8)
    else:
        raise ValueError(kind)
    return 10 ** (-db / 20)


def enbw(kind, n=1 << 16):
    h = filt_response(kind, n)
    return np.sum(np.abs(h) ** 2) * FS / n / 1e6   # MHz (|H(0)| = 1)


def crc_bits(data):
    bits = list(data) + [0] * 24
    r = sd.crc24(bits)
    return np.array(list(data) + [(r >> (23 - i)) & 1 for i in range(24)], np.uint8)


def make_frame(rng, df=17):
    if df == 17:
        head = [1, 0, 0, 0, 1, 1, 0, 1]   # DF17, CA5
        data = head + list(rng.integers(0, 2, 80))
        return crc_bits(data)
    raise ValueError


def envelope(bits, n, start, ramp=5):
    """Mode S amplitude envelope (peak 1) at FS: preamble pulses at 0, 1.0, 3.5, 4.5 us, data from 8 us."""
    cs = int(FS * 0.5e-6)   # samples per chip
    a = np.zeros(n)
    chips = [0, 2, 7, 9] + [16 + 2 * i + (0 if b else 1) for i, b in enumerate(bits)]
    for c in chips:
        s = start + c * cs
        a[s:s + cs] = 1.0
    if ramp:
        k = np.ones(ramp) / ramp
        a = np.convolve(a, k, mode='same')
    return a


@nb.njit(cache=True)
def analog(p_mw, vb, r11, c17, r13, r9, c16, has_d7, is_d7, nvt, tau_v, vf0, vl0, vd0, hyst, vos, tau_c):
    """p_mw: [trials, samples] instantaneous power (mW, antenna referred) incl. detector noise.
    Returns comparator output at 96 MHz [trials, samples] (uint8), and FAST/LEVEL traces of trial 0."""
    nt, ns = p_mw.shape
    out = np.zeros((nt, ns), np.uint8)
    trf = np.zeros(ns); trl = np.zeros(ns); trd = np.zeros(ns)
    dt = 1.0 / FS / SUB
    a_v = dt / (tau_v + dt)
    for k in range(nt):
        v1 = vd0; v2 = vd0; vf = vf0; vl = vl0; q = 0; dc = vf0 - vl0
        a_c = dt / (tau_c + dt)
        for i in range(ns):
            vin = SLOPE * (10 * np.log10(p_mw[k, i]) - V0DBM) * 1e-3   # V
            vbk = vb[k, i]
            for s in range(SUB):
                v1 += a_v * (vin - v1)                 # AD8313 2-pole video (implicit Euler)
                v2 += a_v * (v1 - v2)
                # Implicit Euler on FAST/LEVEL with D7 linearised (one Newton step around the previous state).
                gf = 1.0 / r11; gl = 1.0 / r13 + 1.0 / r9
                cf = c17 / dt; cl = c16 / dt
                if has_d7:
                    x = (vf - vl) / nvt
                    if x > 40.0:
                        x = 40.0
                    ex = np.exp(x)
                    idd = is_d7 * (ex - 1.0)
                    gd = is_d7 * ex / nvt
                else:
                    idd = 0.0; gd = 0.0
                # Linear system: [cf+gf+gd, -gd; -gd, cl+gl+gd] [vf'; vl'] = rhs
                ieq = idd - gd * (vf - vl)
                r1 = cf * vf + gf * v2 - ieq
                r2 = cl * vl + v2 / r13 + vbk / r9 + ieq
                a11 = cf + gf + gd; a12 = -gd; a21 = -gd; a22 = cl + gl + gd
                det = a11 * a22 - a12 * a21
                vf = (r1 * a22 - a12 * r2) / det
                vl = (a11 * r2 - a21 * r1) / det
                dc += a_c * (vf - vl - dc)            # comparator input bandwidth (single pole)
            d = dc - vos
            if q == 0 and d > hyst / 2:
                q = 1
            elif q == 1 and d < -hyst / 2:
                q = 0
            out[k, i] = q
            if k == 0:
                trf[i] = vf; trl[i] = vl; trd[i] = v2
    return out, trf, trl, trd


# capture.pio chip pattern for chips 0-14: 1 = expect HI, 0 = expect LO, -1 = skip
PAT = np.array([1, -1, 1, -1, 0, 0, -1, 1, -1, 1, -1, 0, 0, 0, 0], np.int64)


@nb.njit(cache=True)
def check(c, te, budget):
    """Runs one preamble_matcher candidate from rising edge te (48 MHz index). Returns (matched, t_end)."""
    n = c.shape[0]
    t = te + 4
    y = budget
    for chip in range(15):
        e = PAT[chip]
        if e < 0:
            t += 24
            continue
        cyc = t + 5
        extra = 0
        for s in range(4):
            if cyc >= n:
                return False, n
            v = c[cyc]
            if (e == 1 and v == 0) or (e == 0 and v == 1):
                if y == 0:
                    return False, cyc + 1
                y -= 1
                cyc += 1
                extra += 1
            cyc += 4
        t += 24 + extra
    return True, t


@nb.njit(cache=True)
def check_maj(c, te, max_bad_chips):
    """Variant: a checked chip passes when >= 3 of its 4 samples are right; up to max_bad_chips may fail."""
    n = c.shape[0]
    t = te + 4
    bad = 0
    for chip in range(15):
        e = PAT[chip]
        if e >= 0:
            good = 0
            for s in range(4):
                cyc = t + 5 + 4 * s
                if cyc >= n:
                    return False, n
                if c[cyc] == e:
                    good += 1
            if good < 3:
                bad += 1
                if bad > max_bad_chips:
                    return False, t + 18
        t += 24
    return True, t


@nb.njit(cache=True)
def matcher_ring(c, t_start, budget, busy_after_match, mode=0):
    """4 preamble matchers in round robin. Returns arrays of (t_edge, t_match) for matched candidates,
    and the number of candidates."""
    n = c.shape[0]
    free = np.zeros(4, np.int64)
    free[:] = t_start
    tok = t_start
    k = 0
    m_edge = np.full(64, -1, np.int64); m_t = np.full(64, -1, np.int64)
    nm = 0; ncand = 0
    while True:
        t = max(tok, free[k] + 3)
        while t < n and c[t] != 0:
            t += 1
        while t < n and c[t] != 1:
            t += 1
        if t >= n - 1:
            break
        te = t
        ncand += 1
        tok = te + 1
        if mode == 1:
            ok, tend = check_maj(c, te, budget)
        else:
            ok, tend = check(c, te, budget)
        if ok:
            if nm < 64:
                m_edge[nm] = te; m_t[nm] = tend; nm += 1
            free[k] = tend + busy_after_match
        else:
            free[k] = tend
        k = (k + 1) % 4
    return m_edge[:nm], m_t[:nm], ncand


def demod398(samples):
    """Firmware demod plus the #398 retry: on a CRC fail try up to 3 other offsets within 2 samples of the best
    (most confident first, 2..9)."""
    r = sd.demod(samples)
    if r is None or r['crc_ok'] or r['crc_ok'] is None:
        return r
    best = r['offset']
    cands = []
    for o in range(max(2, best - 2), min(9, best + 2) + 1):
        if o == best:
            continue
        m = sd.margins(samples, o, 12)
        cands.append((-np.abs(m).sum(), o))
    for _, o in sorted(cands)[:3]:
        r2 = sd.demod(samples, offsets=[o])
        if r2 is not None and r2['crc_ok']:
            return r2
    return r


DEFAULT = dict(nf_db=1.0, pdet_dbm=-112.0, filt='tfs2', tau_v=40e-9 / 3.36 * 1.0,
               r11=5.1e3, c17=3e-12, r13=10e3, r9=100e3, c16=200e-12, d7=True, is_d7=2.5e-9, nvt=1.8 * 0.02585,
               hyst=3.3e-3, vos=0.0, tau_c=10e-9, tl_offset=0.600, pwm_ripple=0.031, pwm_hz=25e3, budget=4, ramp=5,
               fc_mhz=0.0, cw_dbm=None, cw_mhz=0.0, cw_phase=None, lead_us=24.0, guard_us=118.0, cfo_jitter=0.0)


def rev(name):
    """Front-end versions (settings.hh): V1 = rev <=D (R11 1k, no D7), V2 = rev E (5.1k), V3 = rev F+ (5.1k + D7)."""
    p = dict(DEFAULT)
    if name == 'V1':
        p.update(r11=1e3, d7=False)
    elif name == 'V2':
        p.update(d7=False)
    return p


_nf_cache = {}


def cw(p, rng, nt, ns):
    """In-band CW (bench signal-generator LO leakage, or a clock harmonic), constant phase within a trial."""
    if p.get('cw_dbm') is None:
        return 0.0
    t = np.arange(ns) / FS
    ph = rng.uniform(0, 2 * np.pi, (nt, 1)) if p.get('cw_phase') is None else p['cw_phase']
    return np.sqrt(10 ** (p['cw_dbm'] / 10)) * np.exp(1j * (2 * np.pi * p['cw_mhz'] * 1e6 * t[None, :] + ph))


def noise_floor_v(p, rng, n=1 << 15):
    """Mean detector voltage with no signal (what the firmware's RSSI noise floor measures)."""
    key = (p['nf_db'], p['pdet_dbm'], p['filt'], p.get('cw_dbm'), p.get('cw_mhz'))
    if key in _nf_cache:
        return _nf_cache[key]
    pw = noise_power(p, rng, 8, n, cw(p, rng, 8, n))
    v = SLOPE * (10 * np.log10(pw) - V0DBM) * 1e-3
    # video filtering does not change the mean
    _nf_cache[key] = float(v.mean())
    return _nf_cache[key]


def noise_power(p, rng, nt, ns, sig=None):
    """Instantaneous power (mW) after the SAW cascade, plus wideband detector noise."""
    n0 = 10 ** ((KT + p['nf_db']) / 10)               # mW/Hz
    var = n0 * FS                                      # complex sample variance
    x = (rng.standard_normal((nt, ns)) + 1j * rng.standard_normal((nt, ns))) * np.sqrt(var / 2)
    if sig is not None:
        x = x + sig
    H = filt_response(p['filt'], ns)
    x = np.fft.ifft(np.fft.fft(x, axis=1) * H, axis=1)
    pd = 10 ** (p['pdet_dbm'] / 10)
    w = (rng.standard_normal((nt, ns)) + 1j * rng.standard_normal((nt, ns))) * np.sqrt(pd / 2)
    return np.abs(x + w) ** 2


def run_level(p, p_dbm, nt, rng, return_traces=False):
    """nt frames at pulse power p_dbm. Returns dict with success count and diagnostics."""
    lead = int(p['lead_us'] * 1e-6 * FS)
    ns = lead + int(130e-6 * FS)
    ns += (-ns) % 64
    frames = [make_frame(rng) for _ in range(nt)]
    amp = np.sqrt(10 ** (p_dbm / 10))
    sig = np.zeros((nt, ns), complex)
    tvec = np.arange(ns) / FS
    for k, b in enumerate(frames):
        env = envelope(b, ns, lead, p['ramp'])
        ph = rng.uniform(0, 2 * np.pi)
        fc = p['fc_mhz'] * 1e6
        sig[k] = amp * env * np.exp(1j * (ph + 2 * np.pi * fc * tvec))
    sig = sig + cw(p, rng, nt, ns)
    pw = noise_power(p, rng, nt, ns, sig)
    nf = noise_floor_v(p, rng)
    vb0 = nf + p['tl_offset']
    ph = rng.uniform(0, 1, nt)[:, None]
    tri = 2 * np.abs(((tvec[None, :] * p['pwm_hz'] + ph) % 1.0) - 0.5) - 0.5   # -0.5..0.5 triangle
    vb = vb0 + p['pwm_ripple'] * tri
    gl = 1 / p['r13'] + 1 / p['r9']
    vl0 = (nf / p['r13'] + vb0 / p['r9']) / gl
    comp, trf, trl, trd = analog(pw, vb, p['r11'], p['c17'], p['r13'], p['r9'], p['c16'], p['d7'], p['is_d7'],
                                 p['nvt'], p['tau_v'], nf, vl0, nf, p['hyst'], p['vos'], p['tau_c'])
    c48 = comp[:, ::2]                                 # PIO samples at 48 MHz
    t_frame48 = lead // 2
    ok = 0; nomatch = 0; blocked = 0; crcfail = 0; ncand = 0
    for k in range(nt):
        if p.get('genie'):
            t0 = t_frame48 + 4 + 360 + 3
            smp = c48[k, t0:t0 + 4 * 1400:4].astype(np.uint8)
            r = demod398(smp)
            ok += int(r is not None and bool(r['crc_ok']) and r['len'] == 112 and np.array_equal(r['bits'], frames[k]))
            continue
        # start the ring a little into the lead-in so LEVEL has settled
        me, mt, nc = matcher_ring(c48[k], int(4e-6 * 48e6), p['budget'], int(44e-6 * 48e6), p.get('mmode', 0))
        ncand += nc
        if len(me) == 0:
            nomatch += 1
            continue
        first = 0
        te = me[first]
        # captures that start in the lead-in noise and block the frame through the guard
        if te < t_frame48 - 48 and (t_frame48 - te) < p['guard_us'] * 48:
            blocked += 1
            continue
        tm = mt[first]
        t0 = tm + 3 + rng.integers(0, 2)
        smp = c48[k, t0:t0 + 4 * 1400:4].astype(np.uint8)
        r = demod398(smp)
        if r is not None and r['crc_ok'] and r['len'] == 112 and np.array_equal(r['bits'], frames[k]):
            ok += 1
        else:
            crcfail += 1
    res = dict(p_dbm=p_dbm, n=nt, ok=ok, nomatch=nomatch, blocked=blocked, fail=crcfail, cand_per_us=ncand / nt /
               ((ns / FS - 4e-6) * 1e6), nf_v=nf)
    if return_traces:
        res['traces'] = (trd, trf, trl, comp[0])
    return res


def false_match_rate(p, rng, ms=5.0, nt=8):
    """Matches per second on noise alone, and candidate edges per second."""
    ns = int(ms * 1e-3 * FS / nt)
    ns += (-ns) % 64
    pw = noise_power(p, rng, nt, ns, cw(p, rng, nt, ns))
    nf = noise_floor_v(p, rng)
    vb0 = nf + p['tl_offset']
    tvec = np.arange(ns) / FS
    tri = 2 * np.abs(((tvec[None, :] * p['pwm_hz']) % 1.0) - 0.5) - 0.5
    vb = vb0 + p['pwm_ripple'] * np.repeat(tri, nt, 0)
    gl = 1 / p['r13'] + 1 / p['r9']
    vl0 = (nf / p['r13'] + vb0 / p['r9']) / gl
    comp, *_ = analog(pw, vb, p['r11'], p['c17'], p['r13'], p['r9'], p['c16'], p['d7'], p['is_d7'], p['nvt'],
                      p['tau_v'], nf, vl0, nf, p['hyst'], p['vos'], p['tau_c'])
    c48 = comp[:, ::2]
    nm = 0; nc = 0
    for k in range(nt):
        me, mt, ncand = matcher_ring(c48[k], 200, p['budget'], int(44e-6 * 48e6), p.get('mmode', 0))
        nm += len(me); nc += ncand
    secs = nt * (ns / FS - 200 / 48e6)
    return nm / secs, nc / secs, float(comp.mean())


def curve(p, levels, nt, seed=1):
    rng = np.random.default_rng(seed)
    return [run_level(p, L, nt, rng) for L in levels]


def p90(res, target=0.9):
    """Interpolated level (dBm) where success crosses target."""
    L = np.array([r['p_dbm'] for r in res]); s = np.array([r['ok'] / r['n'] for r in res])
    o = np.argsort(L); L, s = L[o], s[o]
    for i in range(1, len(L)):
        if s[i - 1] < target <= s[i]:
            return L[i - 1] + (target - s[i - 1]) / (s[i] - s[i - 1]) * (L[i] - L[i - 1])
    return np.nan


def noise_stats(p, rng, us=400.0, nt=8):
    """Comparator duty and HI run lengths at 12 Msps on noise alone (compare with live-sky capture tails)."""
    ns = int(us * 1e-6 * FS); ns += (-ns) % 64
    pw = noise_power(p, rng, nt, ns, cw(p, rng, nt, ns))
    nf = noise_floor_v(p, rng)
    vb0 = nf + p['tl_offset']
    vb = np.full((nt, ns), vb0)
    gl = 1 / p['r13'] + 1 / p['r9']
    vl0 = (nf / p['r13'] + vb0 / p['r9']) / gl
    comp, *_ = analog(pw, vb, p['r11'], p['c17'], p['r13'], p['r9'], p['c16'], p['d7'], p['is_d7'], p['nvt'],
                      p['tau_v'], nf, vl0, nf, p['hyst'], p['vos'], p['tau_c'])
    c12 = comp[:, int(5e-6 * FS)::8]
    hist = np.zeros(60)
    for row in c12:
        x = np.diff(np.concatenate([[0], row, [0]]).astype(int))
        L = np.where(x == -1)[0] - np.where(x == 1)[0]
        for l in L: hist[min(l, 59)] += 1
    return float(c12.mean()), hist
