"""Builds ../report/index.html from the model outputs (t-0877)."""
import json, re, os, numpy as np
from svg import Axes, legend, esc

R = '/workspace/agents/worker-5/t0877/'
J = lambda f: json.load(open(R + f))
sweep = J('sweep3.json'); refs = J('refs.json'); casc = J('cascade.json'); duty = J('duty.json')
real = J('data_realnoise.json'); tr = J('traces.json'); genie = J('genie.json'); match = J('match.json'); scope = J('scope.json')


def best(name):
    rows = [r for r in sweep if r['name'] == name and r['p90'] == r['p90']]
    b = min(rows, key=lambda r: r['p90'])
    return b['p90'], b['tl']


def curve(name, tl):
    for r in sweep:
        if r['name'] == name and abs(r['tl'] - tl) < 1e-6:
            return r
    raise KeyError((name, tl))


def plateau(r):
    c = [s for L, s, *_ in r['curve'] if L >= -84]
    return float(np.mean(c))


# CW (bench-like) curves from the t6 log
cwc = {}
for line in open(R + 't6_cw.log'):
    m = re.match(r'(\S+) p90 (\S+) duty (\S+) nf (\S+) (.*)', line)
    if m:
        pts = [tuple(map(float, re.match(r'(-?\d+):([\d.]+)/', t).groups())) for t in m.group(5).split()]
        cwc[m.group(1)] = dict(p90=float(m.group(2)), nf=float(m.group(4)), pts=pts)

for nm, key in [('cw-95', 'cw-95_600'), ('cw-92', 'cw-92_600')]:
    r = curve(nm, 0.6); cwc[key] = dict(p90=r['p90'], nf=r['nf'], pts=[(a, b) for a, b, *_ in r['curve']])
cwc['cw-92_500'] = dict(p90=curve('cw-92', 0.5)['p90'])
cwc['cw-95_500'] = dict(p90=curve('cw-95', 0.5)['p90'])
b_base, tl_base = best('base')
p600 = plateau(curve('base', 0.6))
p800 = plateau(curve('base', 0.8))
g_best = min(g[1] for g in genie)
VAR = ['base', 'C17_10p', 'C17_22p', 'X2_TA3003', 'X2_TA3003+C17_22p', 'fastcomp', 'revD_V1', 'NF+3dB', 'maj0']
B = {v: best(v) if any(r['name'] == v and r['p90'] == r['p90'] for r in sweep) else (float('nan'), None) for v in VAR}

# ---------------------------------------------------------------- chart 1: ladder
rows = [
    ('Matched-filter PPM, NF 0.8 dB (theory floor)', refs['matched'], 'var(--s3)', 'model, ideal'),
    ('Energy detector behind 2x TFS1090F (23.7 MHz)', refs['energy_tfs2'], 'var(--s3)', 'model, ideal'),
    ('1090U slicer + soft demod, perfect preamble timing', g_best, 'var(--s1)', 'model'),
    ('1090U full chain, TL offset 800 mV, thermal noise', b_base, 'var(--s1)', 'model'),
    ('1090U + in-band CW at -95 dBm (bench-like), TL 600', cwc['cw-95_600']['p90'], 'var(--s1)', 'model'),
    ('1090U bench, TL 500 (t-0827)', -84.3, 'var(--s2)', 'measured'),
    ('1090U bench, TL 600 (t-0827)', -83.6, 'var(--s2)', 'measured'),
    ('m1421 (LR2021) model, NF 5.5 dB, 3.08 MHz', refs['lr2021_nf5.5'], 'var(--s1)', 'model'),
    ('m1421 spec (bench)', -89.0, 'var(--s2)', 'measured'),
]
h = 34 * len(rows) + 50
ax = Axes(760, h, (-102, -80), (len(rows) - 0.4, -0.6), ml=330, mb=36, mt=8)
for v in range(-102, -79, 2):
    ax.parts.append(f'<line x1="{ax.x(v):.1f}" x2="{ax.x(v):.1f}" y1="{ax.mt}" y2="{h - ax.mb}" class="grid"/>')
    ax.parts.append(f'<text x="{ax.x(v):.1f}" y="{h - ax.mb + 16}" class="tick" text-anchor="middle">{v}</text>')
ax.parts.append(f'<text x="{(ax.ml + 760) / 2}" y="{h - 4}" class="lab" text-anchor="middle">90 % message point, dBm at the antenna port (left is better)</text>')
for i, (n, v, c, kind) in enumerate(rows):
    y = ax.y(i)
    ax.parts.append(f'<text x="{ax.ml - 10}" y="{y + 4:.1f}" class="tick" text-anchor="end">{esc(n)}</text>')
    ax.parts.append(f'<line x1="{ax.x(-80):.1f}" x2="{ax.x(v):.1f}" y1="{y:.1f}" y2="{y:.1f}" class="stem"/>')
    ax.point(v, i, c, f'{n}: {v:.1f} dBm ({kind})', shape='circle' if kind != 'measured' else 'diamond', r=6)
    ax.parts.append(f'<text x="{ax.x(v) + 10:.1f}" y="{y - 8:.1f}" class="dl" text-anchor="start">{v:.1f}</text>')
ladder = ax.svg('Ladder of 90 % points') + legend([('model', 'var(--s1)', 0), ('model, ideal receiver', 'var(--s3)', 0),
                                                   ('measured', 'var(--s2)', 0)])

# ---------------------------------------------------------------- chart 2: success curves
ax = Axes(760, 330, (-98, -76), (0, 100))
ax.grid(range(-98, -75, 2), range(0, 101, 20), 'Pulse power at antenna port (dBm)', 'Frames decoded (%)', yfmt=lambda v: f'{v}')
ax.hline(90, label='90 %')
c6 = curve('base', 0.6)['curve']; c8 = curve('base', tl_base)['curve']
gz = [g for g in genie if g[1] == g_best][0]
ser = [
    ('Model, TL 600 (default), thermal', [(a, 100 * b) for a, b, *_ in c6], 'var(--s1)', None),
    (f'Model, TL {int(tl_base * 1000)}, thermal', [(a, 100 * b) for a, b, *_ in c8], 'var(--s3)', None),
    ('Model, TL 600, CW -95 dBm in band', [(a, 100 * b) for a, b in cwc['cw-95_600']['pts']], 'var(--s5)', None),
    (f'Model, perfect preamble timing, TL {int(gz[0] * 1000)}', [(a, 100 * b) for a, b in gz[2]], 'var(--s4)', '5 4'),
]
for n, pts, c, d in ser:
    ax.line(pts, c, n, dash=d, fmt=lambda x, y: f'{x:.0f} dBm, {y:.0f} %')
for lvl, pct, t in [(-84.2, 50, 'p50'), (-83.6, 90, 'p90'), (-82.9, 99, 'p99')]:
    ax.point(lvl, pct, 'var(--s2)', f'Bench 1090U, TL 600, DF17 {t}: {lvl} dBm (t-0827)')
curves = ax.svg('Decode rate vs level') + legend([(n, c, d) for n, _, c, d in ser] + [('Bench 1090U, TL 600 (t-0827)', 'var(--s2)', 0)])

# ---------------------------------------------------------------- chart 3: noise duty vs TL
ax = Axes(500, 300, (0.2, 1.6), (0, 35))
ax.grid([0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6], range(0, 36, 5), 'TL offset setting (V)', 'Comparator HI on noise (%)')
pts = sorted((float(k), 100 * v['duty']) for k, v in duty.items())
ax.line(pts, 'var(--s1)', 'Model', fmt=lambda x, y: f'{x:.2f} V, {y:.1f} %')
ax.point(0.6, 100 * real['mean'], 'var(--s2)', f"Live sky, attic 1090U (t-0870 captures, n={real['frames']}): {100 * real['mean']:.1f} % (TL assumed at default 600)")
ax.text(0.6, 100 * real['mean'], f"live sky {100 * real['mean']:.1f} %", dx=10, dy=-6)
ax.vline(tl_base, label='model optimum')
dutych = ax.svg('Noise duty vs TL') + legend([('Model (V3 front end, thermal noise)', 'var(--s1)', 0), ('Measured, live sky', 'var(--s2)', 0)])

# run-length comparison
mh = np.array(duty['0.6']['hist']); rh = np.array(real['runhist']); rh = rh / rh.sum()
ax = Axes(500, 300, (0.4, 6.6), (0, 85), mb=40)
ax.grid(range(1, 7), range(0, 81, 20), 'HI run length (samples at 12 Msps)', 'Share of HI runs (%)')
for k in range(1, 7):
    for j, (arr, c, n) in enumerate([(mh, 'var(--s1)', 'model'), (rh, 'var(--s2)', 'live sky')]):
        x0 = ax.x(k - 0.32 + j * 0.34); x1 = ax.x(k - 0.32 + j * 0.34 + 0.3)
        y = ax.y(100 * arr[k]); y0 = ax.y(0)
        ax.parts.append(f'<path d="M{x0:.1f},{y0:.1f} L{x0:.1f},{y + 3:.1f} Q{x0:.1f},{y:.1f} {x0 + 3:.1f},{y:.1f} L{x1 - 3:.1f},{y:.1f} '
                        f'Q{x1:.1f},{y:.1f} {x1:.1f},{y + 3:.1f} L{x1:.1f},{y0:.1f} Z" fill="{c}"><title>{n}, {k} samples: {100 * arr[k]:.1f} %</title></path>')
runch = ax.svg('HI run lengths') + legend([('Model, TL 600', 'var(--s1)', 0), ('Live sky', 'var(--s2)', 0)])

# ---------------------------------------------------------------- chart 4: traces
def tracechart(key, title):
    d = tr[key]; t = d['t']
    ax = Axes(380, 260, (-3, 14), (0.5, 1.35), ml=48)
    ax.grid(range(-2, 15, 2), [0.6, 0.8, 1.0, 1.2], 'Time from first preamble pulse (µs)', 'Volts')
    ax.line(list(zip(t, d['det'])), 'var(--s4)', 'AD8313 out', markers=False, width=1)
    ax.line(list(zip(t, d['fast'])), 'var(--s1)', 'FAST', markers=False, width=1.5)
    ax.line(list(zip(t, d['level'])), 'var(--s2)', 'LEVEL (TL)', markers=False, width=2)
    comp = [(a, 0.55 + 0.08 * c) for a, c in zip(t, d['comp'])]
    ax.line(comp, 'var(--text-2)', 'comparator out', markers=False, width=1, step=True)
    return f'<figure><figcaption>{esc(title)}</figcaption>' + ax.svg(title) + '</figure>'


traces = ('<div class="pair">' + tracechart('tl600', 'TL 600 mV, -88 dBm frame') + tracechart('tl1000', 'TL 1000 mV, same frame and noise')
          + '</div>' + legend([('AD8313 out', 'var(--s4)', 0), ('FAST (comparator +)', 'var(--s1)', 0), ('LEVEL (comparator -)', 'var(--s2)', 0),
                              ('comparator out (scaled)', 'var(--text-2)', 0)]))

# ---------------------------------------------------------------- chart 5: p90 vs TL per variant
hv = [('base', 'Rev F+ as built (C17 3 pF)', 'var(--s1)', None), ('C17_10p', 'C17 10 pF', 'var(--s3)', None),
      ('C17_22p', 'C17 22 pF', 'var(--s4)', None), ('X2_TA3003', 'X2 = TA3003 (11.8 MHz)', 'var(--s2)', '5 4'),
      ('X2_TA3003+C17_22p', 'X2 = TA3003 + C17 22 pF', 'var(--s5)', '5 4')]
ax = Axes(760, 320, (0.25, 1.15), (-84, -96))
ax.grid([0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1], range(-96, -83, 2), 'TL offset setting (V)', '90 % point (dBm), up is better')
for k, n, c, d in hv:
    pts = sorted((r['tl'], r['p90'] if r['p90'] == r['p90'] else None) for r in sweep if r['name'] == k)
    ax.line(pts, c, n, dash=d, fmt=lambda x, y: f'TL {x:.2f} V: {y:.1f} dBm')
ax.vline(0.6, label='default 600')
varch = ax.svg('90% point vs TL') + legend([(n, c, d) for _, n, c, d in hv]) + \
    '<p class="note">Missing points: the decode rate never reached 90 % (noise keeps the preamble matchers busy, see section 4).</p>'

# ---------------------------------------------------------------- tables
crow = ''.join(f"<tr><td>{esc(r['stage'])}</td><td>{r['gain']:+.2f}</td><td>{r['nf']:.2f}</td><td>{esc(r['bw'])}</td>"
               f"<td>{r['cum_gain']:.1f}</td><td>{r['cum_nf']:.2f}</td><td class='src'>{esc(r['src'])}</td></tr>" for r in casc['rows'])


def f1(v):
    return '—' if v != v else f'{v:.1f}'


def vrow(k, label, cost, effort, note):
    v, tl = B[k]
    gain = v - b_base
    return (f"<tr><td>{label}</td><td>{f1(v)}</td><td>{'—' if tl is None else int(tl * 1000)}</td>"
            f"<td>{'—' if gain != gain else f'{-gain:+.1f}'}</td><td>{cost}</td><td>{effort}</td><td>{note}</td></tr>")


tmpl = open(R + 'ana/report_tpl.html').read()
vals = dict(
    LADDER=ladder, CURVES=curves, DUTY=dutych, RUNS=runch, TRACES=traces, VARS=varch, CASC=crow,
    NF=f"{casc['nf_fe']:.2f}", GAIN=f"{casc['gain_fe']:.1f}", FENOISE=f"{casc['fe_noise_ant_dbm']:.1f}",
    DETNOISE=f"{casc['det_noise_ant_dbm']:.1f}", DETPEN=f"{casc['det_penalty_db']:.2f}",
    MATCHED=f"{refs['matched']:.1f}", ENERGY=f"{refs['energy_tfs2']:.1f}", ENERGY_TA=f"{refs['energy_ta3003']:.1f}",
    GENIE=f"{g_best:.1f}", BASE=f"{b_base:.1f}", TLBASE=str(int(tl_base * 1000)),
    CW95=f"{cwc['cw-95_600']['p90']:.1f}", CW95NF=f"{cwc['cw-95_600']['nf']:.3f}", CW92=f"{cwc['cw-92_600']['p90']:.1f}", CW95_500=f"{cwc['cw-95_500']['p90']:.1f}",
    CW92_500=f"{cwc['cw-92_500']['p90']:.1f}",
    LR=f"{refs['lr2021_nf5.5']:.1f}", LR078=f"{refs['lr2021_nf0.78']:.1f}",
    P600=f"{100 * p600:.0f}", P800=f"{100 * p800:.0f}", REALDUTY=f"{100 * real['mean']:.1f}", MODDUTY=f"{100 * duty['0.6']['duty']:.1f}",
    REALN=str(real['frames']),
    D_BW=f"{refs['energy_tfs2'] - refs['matched']:.1f}", D_SLICE=f"{g_best - refs['energy_tfs2']:.1f}",
    D_PRE=f"{b_base - g_best:.1f}", D_CW=f"{-83.6 - b_base:.1f}", MISS=f"{100 - 100 * p600:.0f}", AHEAD=f"{refs['lr2021_nf5.5'] - b_base:.1f}",
    VROWS=''.join([
        vrow('C17_22p', 'C17 3 pF → 22 pF (FAST low-pass 10 → 1.4 MHz), TL retuned', '$0', 'rework 1 cap', 'Optimum TL is sharp (±100 mV costs up to 4 dB): run the TL learner per unit. Adds edge walk, check MLAT.'),
        vrow('C17_10p', 'C17 3 pF → 10 pF, TL retuned', '$0', 'rework 1 cap', 'Smaller gain, flatter TL optimum, less edge walk.'),
        vrow('X2_TA3003', 'X2 TFS1090F → Tai-Saw TA3003AA3112', 'quote', 'part swap, not stocked', 'No catalog stock (t-0061). Also +30 dB rejection at 1080 MHz.'),
        vrow('X2_TA3003+C17_22p', 'Both of the above', 'quote', 'part swap', 'Best hardware case modeled.'),
        vrow('fastcomp', 'MCP6566 → push-pull fast comparator (e.g. TLV3601)', '+$3.5', 'swap U1, drop R10', 'No gain: the comparator is not the limit. Its slowness is a mild extra low-pass.'),
        vrow('revD_V1', 'Rev D network (R11 1 kΩ, no D7) for reference', '—', '—', 'Older boards: slightly worse, same TL behavior.'),
        vrow('NF+3dB', 'Front-end NF 3 dB worse (what an LNA problem would cost)', '—', '—', 'For scale: each dB of NF is ~0.9 dB of sensitivity.'),
        vrow('maj0', 'PIO matcher: per-chip 3-of-4 majority, no bad chips', '$0', 'PIO rewrite', 'Firmware; small gain. The ~4 dB to the timing-genie case needs a soft preamble correlator.'),
    ]),
    MATCHTAB=''.join(f"<tr><td>{k.split('/')[0]} Ω ‖ {k.split('/')[1]} pF</td><td>{v['gain']:.1f}</td><td>{v['rl']:.1f}</td><td>{v['fpk']:.0f} MHz, {v['gpk']:.1f} dB</td></tr>"
                     for k, v in match.items() if k.split('/')[0] in ('250', '330', '450') and k.split('/')[1] in ('1.5', '2.5')),
    FASTRMS=f"{scope['fast_rms_mV']:.0f}", DETRMS=f"{scope['det_rms_mV']:.0f}", LEVRMS=f"{scope['level_rms_mV']:.0f}",
    NFV=f"{scope['nf_v']:.3f}",
)
for k, v in vals.items():
    tmpl = tmpl.replace('{{' + k + '}}', v)
left = re.findall(r'\{\{\w+\}\}', tmpl)
assert not left, left
os.makedirs(R + 'report', exist_ok=True)
open(R + 'report/index.html', 'w').write(tmpl)
print('ok', len(tmpl), {k: vals[k] for k in ['BASE', 'TLBASE', 'GENIE', 'P600', 'P800', 'CW95', 'LR']})
