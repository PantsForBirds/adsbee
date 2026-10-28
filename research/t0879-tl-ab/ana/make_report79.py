#!/usr/bin/env python3
"""Build the t-0879 report from ab79.json, ratios79.json (paired), trend79.json (trend-adjusted) and concl.html.

usage: make_report79.py DATA_DIR OUT_HTML
"""
import json
import math
import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svg import Axes, legend, esc  # noqa: E402

D, OUT = sys.argv[1], sys.argv[2]
HERE = os.path.dirname(os.path.abspath(__file__))
blocks = json.load(open(os.path.join(D, 'ab79.json')))
P = json.load(open(os.path.join(D, 'ratios79.json')))
FX = json.load(open(os.path.join(D, 'trend79.json')))['fx']
ARMS = ['400', '450', '500', '600', '800']
OTH = ['400', '450', '500', '800']
COL = {'400': 'var(--s1)', '450': 'var(--s2)', '500': 'var(--s3)', '600': 'var(--s4)', '800': 'var(--s5)',
       'LEARN': 'var(--s6)'}
mean = P['means']
learn_tl = mean['LEARN']['tl_offset_mv']


def yticks(lo, hi):
    span = hi - lo
    step = 0.05 if span < 0.4 else 0.1 if span < 1 else 0.25 if span < 2.5 else 0.5
    return [round(step * i, 3) for i in range(math.ceil(lo / step), math.floor(hi / step) + 1)]


def whisker(ax, x, c, color):
    ax.parts.append(f'<line x1="{ax.x(x):.1f}" x2="{ax.x(x):.1f}" y1="{ax.y(c["lo"]):.1f}" y2="{ax.y(c["hi"]):.1f}" '
                    f'stroke="{color}" stroke-width="2"/>')


def ci_txt(c):
    return f'{c["r"]:.2f} [{c["lo"]:.2f}, {c["hi"]:.2f}]'


def ratio_chart(key, title, clip=(0.4, 2.5)):
    fx = FX[key]
    pts = [(int(a), fx[a]) for a in OTH if a in fx] + [(600, dict(r=1, lo=1, hi=1))]
    pts.sort()
    lc = fx.get('LEARN')
    vals = [v for _, c in pts for v in (c['lo'], c['hi'])]
    lo, hi = max(clip[0], min(vals) - 0.03), min(clip[1], max(vals) + 0.03)
    ax = Axes(440, 230, (350, 850), (lo, hi), ml=52)
    ax.grid([400, 450, 500, 600, 800], yticks(lo, hi), 'TL offset (mV)', 'vs TL 600', yfmt=lambda v: f'{v:.2f}')
    ax.hline(1.0)
    for a, c in pts:
        whisker(ax, a, dict(lo=max(lo, c['lo']), hi=min(hi, c['hi'])), 'var(--s1)')
    ax.line([(a, c['r']) for a, c in pts], 'var(--s1)', title, fmt=lambda a, r: f'TL {a}: ' + ci_txt(fx[str(a)])
            if str(a) in fx else 'reference')
    if lc and lo <= lc['r'] <= hi:
        ax.point(learn_tl, lc['r'], 'var(--s6)', f'learner ({learn_tl:.0f} mV, one block): ' + ci_txt(lc))
    return f'<figure><figcaption>{esc(title)}</figcaption>{ax.svg(title)}</figure>'


def bin_chart(prefix, labels, title, ylabel):
    nb = len(labels)
    ax = Axes(900, 260, (-0.5, nb - 0.5), (0, 2), ml=52)
    vals = [v for a in OTH for i in range(nb) if a in FX[f'{prefix}{i}'] for v in (FX[f'{prefix}{i}'][a]['r'],)]
    ax.yr = (0, min(2.5, max(vals + [1.2]) + 0.2))
    ax.grid(range(nb), yticks(*ax.yr), title, ylabel, xfmt=lambda i: labels[i], yfmt=lambda v: f'{v:.2f}')
    ax.hline(1.0)
    for k, a in enumerate(OTH):
        off = (k - 1.5) * 0.09
        for i in range(nb):
            c = FX[f'{prefix}{i}'].get(a)
            if not c:
                continue
            whisker(ax, i + off, dict(lo=max(0, c['lo']), hi=min(ax.yr[1], c['hi'])), COL[a])
            ax.point(i + off, min(c['r'], ax.yr[1]), COL[a], f'TL {a}, {labels[i]}: ' + ci_txt(c), shape='c', r=5)
    return ax.svg(title) + legend([(f'TL {a}', COL[a], 0) for a in OTH])


# ---- ratio grid (trend-adjusted)
grid = [ratio_chart('valid_s', 'Good frames/s'), ratio_chart('unique_ac', 'Unique aircraft'),
        ratio_chart('pr1', 'Position msgs/s per aircraft, 15–30 nm (strong)'),
        ratio_chart('range99', 'Range, 99th pct of position fixes'),
        ratio_chart('crc_fail_pct', 'CRC fail %'), ratio_chart('lone_ap', 'ICAOs seen once (address/parity)', clip=(0.2, 6))]
ratio_grid = '<div class="pair">' + ''.join(grid) + '</div>' + legend(
    [('ratio vs 600, 95% CI', 'var(--s1)', 0), ('learner, one block', 'var(--s6)', 0)])

prchart = bin_chart('pr', ['0–15 nm', '15–30 nm', '30–45 nm', '45–60 nm', '60+ nm'],
                    'aircraft by median distance', 'position msgs/s vs 600')
acchart = bin_chart('acm', ['0–25 nm', '25–50 nm', '50–75 nm', '75+ nm'], 'farthest position decoded per aircraft',
                    'aircraft vs 600')

# ---- duty and edges
pts_q = [(int(a), 100 * mean[a]['raw_duty_q']) for a in ARMS]
pts_s = [(int(a), 100 * mean[a]['raw_duty_sig']) for a in ARMS]
pts_e = [(int(a), mean[a]['edges_s'] / 1e3) for a in ARMS]
ax = Axes(440, 230, (350, 850), (0, 26), ml=48)
ax.grid([400, 450, 500, 600, 800], [0, 5, 10, 15, 20, 25], 'TL offset (mV)', 'comparator HI %')
ax.line(pts_s, 'var(--s2)', 'on signal', fmt=lambda a, v: f'TL {a}: {v:.1f}%')
ax.line(pts_q, 'var(--s1)', 'between frames', fmt=lambda a, v: f'TL {a}: {v:.1f}%')
ax.point(learn_tl, 100 * mean['LEARN']['raw_duty_q'], 'var(--s6)',
         f'learner {learn_tl:.0f} mV, between frames: {100 * mean["LEARN"]["raw_duty_q"]:.1f}%')
ax.point(600, 0.87, 'var(--s3)', 'pi-1 bench unit at TL 600 (t-0875): 0.87%', shape='c', r=5)
ax.text(600, 0.87, 'pi-1: 0.87%', dy=-6)
dutych = ax.svg('Comparator duty') + legend([('on signal', 'var(--s2)', 0), ('between frames', 'var(--s1)', 0),
                                             ('learner', 'var(--s6)', 0), ('pi-1, TL 600', 'var(--s3)', 0)])
ax = Axes(440, 230, (350, 850), (0, 1400), ml=52)
ax.grid([400, 450, 500, 600, 800], [0, 250, 500, 750, 1000, 1250], 'TL offset (mV)', 'rising edges/s (thousands)')
ax.line(pts_e, 'var(--s1)', 'edges/s', fmt=lambda a, v: f'TL {a}: {v:.0f}k/s')
ax.point(600, 63, 'var(--s3)', 'pi-1 bench unit at TL 600: 63k/s', shape='c', r=5)
ax.text(600, 63, 'pi-1: 63k', dy=-6)
edgech = ax.svg('Edge rate')

# ---- timeline
t0 = min(datetime.fromisoformat(b['start']) for b in blocks)
tl = [((datetime.fromisoformat(b['start']) - t0).total_seconds() / 3600, b['valid_s'], b['tl']) for b in blocks]
ax = Axes(900, 240, (0, 17.5), (0, 200), ml=48)
ax.grid(range(0, 18, 2), [0, 50, 100, 150, 200], f'hours from {t0:%H:%M} UTC {t0:%b %-d} (gap = worker-1 window)',
        'good frames/s', xfmt=lambda h: f'{h}')
for t, v, a in tl:
    ax.point(t + 0.25, v, COL[a], f'{"learner" if a == "LEARN" else "TL " + a} at +{t:.1f} h: {v:.0f}/s', shape='c', r=5)
timech = ax.svg('Good frames/s per block') + legend([(f'TL {a}' if a != 'LEARN' else 'learner', COL[a], 0)
                                                     for a in ARMS + ['LEARN']])

# ---- tables
def pct(c):
    return f'{(c["r"] - 1) * 100:+.0f}% <span class="ci">({(c["lo"] - 1) * 100:+.0f}, {(c["hi"] - 1) * 100:+.0f})</span>'


def sig(c):
    return c['lo'] > 1 or c['hi'] < 1


TROWS = [('valid_s', 'Good frames/s'), ('unique_ac', 'Unique aircraft (≥2 frames)'), ('fpa', 'Frames per aircraft'),
         ('caps_s', 'Captures/s'), ('guard_s', 'Guard drops/s'), ('crc_fail_pct', 'CRC fail %'),
         ('yield_pct', 'Good frames / captures'), ('pr0', 'Position rate, 0–15 nm'), ('pr1', 'Position rate, 15–30 nm'),
         ('pr2', 'Position rate, 30–45 nm'), ('range90', 'Range p90, all fixes'), ('range99', 'Range p99, all fixes'),
         ('acm2', 'Aircraft reaching 50–75 nm'), ('acm3', 'Aircraft reaching 75+ nm'),
         ('posm2', 'Position fixes at 50–75 nm'), ('lone', 'ICAOs seen once (all)'), ('lone_ap', 'ICAOs seen once (address/parity)'),
         ('jumps', 'Track disagreements (CPR jumps)'), ('core1_pct', 'Demod CPU (core 1 busy)'),
         ('demod_us', 'CPU per demod'), ('duty', 'Comparator HI duty'), ('edges_s', 'Rising edges/s')]
rrows = []
for key, t in TROWS:
    cells = ''
    for a in OTH + ['LEARN']:
        c = FX[key].get(a)
        cells += f'<td class="{"sig" if c and sig(c) else ""}">{pct(c) if c else "–"}</td>'
    rrows.append(f'<tr><td>{t}</td>{cells}</tr>')
ratio_table = ''.join(rrows)

prow = []
for key, t in [('valid_s', 'Good frames/s'), ('unique_ac', 'Unique aircraft'), ('crc_fail_pct', 'CRC fail %'),
               ('range99', 'Range p99'), ('lone', 'ICAOs seen once'), ('duty', 'Comparator HI duty')]:
    prow.append(f'<tr><td>{t}</td>' + ''.join(f'<td>{pct(P["ratios"][a][key])}</td>' for a in OTH) + '</tr>')
paired_table = ''.join(prow)

rows = []
for a in ARMS + ['LEARN']:
    m = mean[a]
    lab = f'learner ({m["tl_offset_mv"]:.0f})' if a == 'LEARN' else a
    f = lambda k, fmt='%.1f': (fmt % m[k]) if m.get(k) is not None else '–'  # noqa: E731
    rows.append(f'<tr><td>{lab}</td><td>{m["n"]}</td><td>{f("valid_s", "%.0f")}</td><td>{f("unique_ac", "%.0f")}</td>'
                f'<td>{f("crc_fail_pct", "%.2f")}</td><td>{f("caps_s", "%.0f")}</td><td>{f("guard_s", "%.1f")}</td>'
                f'<td>{100 * m["raw_duty_q"]:.1f} / {100 * m["raw_duty_sig"]:.1f}</td><td>{m["edges_s"] / 1e3:.0f}k</td>'
                f'<td>{f("lone")}</td><td>{f("jumps")}</td><td>{f("core1_pct", "%.2f")}</td></tr>')
abs_table = ''.join(rows)

tpl = open(os.path.join(HERE, 'report79_tpl.html')).read()
concl = open(os.path.join(HERE, 'concl.html')).read()
sub = dict(CONCL=concl, RATIOS=ratio_grid, PR=prchart, BINS=acchart, DUTY=dutych, EDGES=edgech, TIME=timech,
           ABS=abs_table, RTAB=ratio_table, PTAB=paired_table, NROUNDS=str(len(P['rounds'])), LEARNTL=f'{learn_tl:.0f}',
           NBLOCKS=str(len(blocks)))
for k, v in sub.items():
    tpl = tpl.replace('{{' + k + '}}', v)
assert '{{' not in tpl, tpl[tpl.index('{{'):tpl.index('{{') + 40]
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, 'w').write(tpl)
print('wrote', OUT, len(tpl))
