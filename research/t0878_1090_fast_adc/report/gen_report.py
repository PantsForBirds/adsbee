"""Builds report/index.html for t-0878 from ../data/*.json and summary.json (run analysis.py first)."""
import json, os, html, numpy as np
import svgchart as C

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, '..', 'data')
S = json.load(open(os.path.join(HERE, 'summary.json')))
OFF = S['cal']['offset_db']


def J(p):
    p = os.path.join(D, p)
    return json.load(open(p)) if os.path.exists(p) else None


def curve(d, key):
    s = sorted(d['data'], key=float)
    return [(float(x) + OFF, d['data'][x][key] / d['data'][x]['n']) for x in s if key in d['data'][x]]


def pt(src, key, lvl):
    v = S.get(src, {}).get(key)
    if not v or v[lvl] is None: return None
    return v[lvl] + OFF


TODAY = ('main', 'A600')
OPTS = [  # key, source, label, group
    ('fw_b8', ('a_sweep', 'A600_b8'), 'PIO mismatch budget 4 → 8', 'Firmware only'),
    ('fw_soft600', ('flash', 'lvl600'), 'Comparator stream + software preamble, TL 600', 'Firmware only'),
    ('fw_soft300', ('flash', 'lvl300'), 'Comparator stream + software preamble, TL 300', 'Firmware only'),
    ('hyb', ('main', 'R10_8_600_8'), 'ADC demod, PIO trigger (TL 600, budget 8)', 'New hardware'),
    ('flash3', ('flash', 'c3_2_5_10dB'), '3 comparators (2-bit flash), 12 Msps, soft', 'New hardware'),
    ('adc8_10', ('main', 'B10_8'), '8-bit ADC, 10 Msps, soft preamble', 'New hardware'),
    ('adc8_20', ('main', 'B20_8'), '8-bit ADC, 20 Msps, soft preamble', 'New hardware'),
    ('iq', ('iq', 'iq10_bw3_lin'), 'I/Q 10 Msps, ±3 MHz filter, no log detector', 'New hardware'),
]
t50, t90 = pt(*TODAY, 's50'), pt(*TODAY, 's90')
G = {}
for k, src, lab, grp in OPTS:
    a50, a90 = pt(*src, 's50'), pt(*src, 's90')
    G[k] = dict(label=lab, group=grp, p50=a50, p90=a90, g50=(t50 - a50) if a50 else None, g90=(t90 - a90) if a90 else None)

f = lambda x, d=1: '–' if x is None else f'{x:.{d}f}'
fp = lambda x, d=1: '–' if x is None else f'{x:+.{d}f}'

# ---------- Fig 1: success vs dBm ----------
main, flash, asw, iq = J('main.json'), J('flash.json'), J('a_sweep.json'), J('iq.json')
fig1 = C.line_chart([
    dict(name='Today (TL 600)', color='--s1', pts=curve(main, 'A600')),
    dict(name='FW: budget 8', color='--s4', pts=curve(asw, 'A600_b8')),
    dict(name='FW: soft preamble, TL 300', color='--s3', pts=curve(flash, 'lvl300')),
    dict(name='8-bit ADC 10 Msps', color='--s2', pts=curve(main, 'B10_8')),
], 'Input power (dBm, calibrated: today 90 % at −84)', 'DF17 frames decoded', (-98, -78), (0, 1),
    list(range(-98, -77, 2)), [0, .25, .5, .75, .9, 1], ypct=True, hlines=[(0.9, '90 %'), (0.5, '50 %')])

# ---------- Fig 2: gain bars ----------
rows = [(G[k]['label'], G[k]['g90'] or 0, '--s3' if G[k]['group'] == 'Firmware only' else '--s2') for k, *_ in OPTS]
fig2 = C.hbar(rows, 'dB gain at the 90 % point vs today', 12, label_w=330)

# ---------- ADC bits x rate table ----------
def g90(src, key):
    p = pt(src, key, 's90'); return (t90 - p) if p is not None else None
bits_rows = ''
for fs in (8, 10, 12, 20):
    bits_rows += f'<tr><td>{fs} Msps</td>' + ''.join(f'<td>{fp(g90("main", f"B{fs}_{n}"))}</td>' for n in (6, 8, 10)) + '</tr>'
low_bits = f'3-bit {fp(g90("main", "B10_3"))} dB, 4-bit {fp(g90("main", "B10_4"))} dB, unquantized {fp(g90("main", "B10_None"))} dB'

# ---------- TL / budget sweep table ----------
def a_row(o):
    cells = ''
    for b in (4, 6, 8, 10, 12):
        k = f'A{o}_b{b}'; v = S['a_sweep'].get(k)
        if not v: cells += '<td></td>'; continue
        pl = v['plateau']; g = (t90 - (v['s90'] + OFF)) if v['s90'] is not None else None
        cells += f'<td>{fp(g)}{"" if pl > .97 else f" <span class=warn>({pl*100:.0f} % max)</span>"}</td>'
    return f'<tr><td>{o} mV</td>{cells}</tr>'
tl_rows = ''.join(a_row(o) for o in (300, 600, 900))
tl4 = ' · '.join(f'{o}: {fp((t90 - (S["a_sweep"][f"A{o}_b4"]["s90"] + OFF)) if S["a_sweep"].get(f"A{o}_b4", {}).get("s90") else None)}'
                 for o in (300, 450, 600, 750, 900, 1200))
noise_pio = json.load(open(os.path.join(D, 'noise_pio.json'))) if os.path.exists(os.path.join(D, 'noise_pio.json')) else None

# ---------- overlap ----------
ov = J('overlap.json'); dh = J('delta_hist.json')
ovd = ov['data']; deltas = sorted(ovd, key=float)
fr = lambda d, k: ovd[d][k] / ovd[d]['n']
fig3 = C.line_chart([
    dict(name='Today: first frame', color='--s1', pts=[(float(d), fr(d, 'a_A')) for d in deltas]),
    dict(name='Today: second frame', color='--s1', dash=True, pts=[(float(d), fr(d, 'a_B')) for d in deltas]),
    dict(name='ADC: first frame', color='--s2', pts=[(float(d), fr(d, 'b_A')) for d in deltas]),
    dict(name='ADC: second frame', color='--s2', dash=True, pts=[(float(d), fr(d, 'b_B')) for d in deltas]),
], 'Second frame level minus first (dB)', 'Frames decoded', (-20, 20), (0, 1), list(range(-20, 21, 4)),
    [0, .25, .5, .75, 1], ypct=True)
centers = [(e0 + e1) / 2 for e0, e1 in zip(dh['edges'], dh['edges'][1:])]
w = dict(zip([round(c) for c in centers], dh['hist']))
ex_today = sum(w.get(round(float(d)), 0) * (fr(d, 'a_A') + fr(d, 'a_B')) for d in deltas)
ex_adc = sum(w.get(round(float(d)), 0) * (fr(d, 'b_A') + fr(d, 'b_B')) for d in deltas)
ex_hyb = sum(w.get(round(float(d)), 0) * (fr(d, 'h_A') + fr(d, 'h_B')) for d in deltas)
POIS_H, FRAMES_H = 3257, 329883   # t-0870: expected 2-aircraft overlaps/h and good frames/h at ~120 captures/s
rec_adc = POIS_H * (ex_adc - ex_today); rec_hyb = POIS_H * (ex_hyb - ex_today)
trig = np.mean([ovd[d]['b_trig'] / ovd[d]['n'] for d in deltas])

# ---------- timing / RSSI ----------
tm = J('timing.json')
tm_rows = ''
for s in sorted(tm, key=float):
    r = tm[s]
    if not r['fw']['n']: continue
    tm_rows += (f'<tr><td>{float(s)+OFF:.0f} dBm</td><td>{f(r["fw"]["rms_ns"])}</td><td>{f(r["adc8"]["rms_ns"])}</td>'
                f'<td>{f(r["adc20"]["rms_ns"])}</td><td>{f(r["rssi_lp"]["sd_db"], 2)}</td><td>{f(r["rssi8"]["sd_db"], 2)}</td></tr>')
ks = [s for s in sorted(tm, key=float) if tm[s]['fw']['n']]
drift = {k: (tm[ks[-1]][k]['bias'] - tm[ks[0]][k]['bias']) / 0.24 for k in ('fw', 'adc8', 'adc20')}
lvl_span = float(ks[-1]) - float(ks[0])

# ---------- sensitivity ----------
def sens_row(name, v, note):
    def gg(src, key, ref=None):
        a = S.get(f'{v}/{src}' if v else src, {}).get(key, {})
        r = ref if ref is not None else S.get(f'{v}/main' if v else 'main', {}).get('A600', {}).get('s90')
        return (r - a['s90']) if a.get('s90') is not None and r is not None else None
    ref = None if v != 'flm7' else S['main']['A600']['s90']   # LNA case: vs nominal today, same antenna-referred noise
    cells = [gg('a_sweep', 'A600_b8', ref), gg('flash', 'lvl300', ref), gg('flash', 'c3_2_5_10dB', ref), gg('main', 'B10_8', ref)]
    today = None
    if v == 'flm7':
        best = [S['flm7/a_sweep'].get(f'A{o}_b4', {}).get('s90') for o in (300, 450, 600, 750, 900, 1200)]
        best = [b for b in best if b is not None]
        today = (ref - min(best)) if best else None
    return (f'<tr><td>{name}</td><td>{note}</td>' + (f'<td>{fp(today)}</td>' if v == 'flm7' else '<td>0 (ref)</td>')
            + ''.join(f'<td>{fp(c)}</td>' for c in cells) + '</tr>')
sens = ''.join(sens_row(*r) for r in [
    ('Nominal', '', 'floor +3 dB, video 8 MHz'), ('Floor 0 dB', 'fl0', 'detector floor = in-band noise'),
    ('Floor +6 dB', 'fl6', 'less RF gain / weaker LNAs'), ('Video 5 MHz', 'v5', 'slower detector / loading'),
    ('+10 dB LNA gain', 'flm7', 'floor −7 dB; vs nominal today')])

noise = json.load(open(os.path.join(D, '..', 'noise.json')))
nf_db = S['cal']['nf_db']

hl = G['adc8_10']; fw = G['fw_soft300']; b8 = G['fw_b8']

PAGE = open(os.path.join(HERE, 'template.html')).read()
for k, v in dict(
        FIG1=fig1, FIG2=fig2, FIG3=fig3, BITS_ROWS=bits_rows, LOW_BITS=low_bits, TL_ROWS=tl_rows, TL4=tl4, TM_ROWS=tm_rows,
        SENS=sens,
        T50=f(t50), T90=f(t90), OFFDB=f(OFF), NF=f(nf_db),
        ADC_G90=f(hl['g90']), ADC_G50=f(hl['g50']), ADC_P90=f(hl['p90']), ADC_P50=f(hl['p50']),
        FW_G90=f(fw['g90']), FW_G50=f(fw['g50']), FW_P90=f(fw['p90']),
        B8_G90=f(b8['g90']), B8_G50=f(b8['g50']),
        FW600_G90=f(G['fw_soft600']['g90']), HYB_G90=f(G['hyb']['g90']), FL3_G90=f(G['flash3']['g90']),
        IQ_G90=f(G['iq']['g90']), ADC20_G90=f(G['adc8_20']['g90']),
        OPT_ROWS=''.join(f'<tr><td>{G[k]["group"]}</td><td>{G[k]["label"]}</td><td>{f(G[k]["p50"])}</td><td>{f(G[k]["p90"])}</td>'
                         f'<td>{fp(G[k]["g50"])}</td><td>{fp(G[k]["g90"])}</td></tr>' for k, *_ in OPTS),
        REC_ADC=f'{rec_adc:,.0f}', REC_ADC_PCT=f(100 * rec_adc / FRAMES_H, 2), REC_HYB=f'{rec_hyb:,.0f}',
        EX_TODAY=f(ex_today, 2), EX_ADC=f(ex_adc, 2), TRIG=f(trig, 0),
        DRIFT_FW=f(drift['fw'], 0), DRIFT_A8=f(drift['adc8'], 0), DRIFT_A20=f(drift['adc20'], 0), LVLSPAN=f(lvl_span, 0),
        THR10=f(json.load(open(os.path.join(D, 'main.json')))['thr']['10'], 0),
).items():
    PAGE = PAGE.replace('{{' + k + '}}', str(v))
open(os.path.join(HERE, 'index.html'), 'w').write(PAGE)
print('wrote index.html', len(PAGE))
print({k: (round(v['g50'], 1) if v['g50'] else None, round(v['g90'], 1) if v['g90'] else None) for k, v in G.items()})
print('overlap', ex_today, ex_adc, rec_adc, rec_hyb, trig, 'drift', drift)
