"""Tiny inline-SVG chart helpers for the t-0878 report: line charts and horizontal bars, colored by CSS tokens
(--s1..--s6) so light/dark themes swap in one place. Every point carries a <title> tooltip."""
import html

W, H = 680, 340
ML, MR, MT, MB = 56, 190, 16, 44


def _fmt(v):
    return f'{v:g}'


def line_chart(series, xlabel, ylabel, xlim, ylim, xticks, yticks, refs=(), labels_at=None, w=W, h=H, ypct=False,
               hlines=()):
    """series: list of dict(name, color (css var), pts [(x,y)], dash=False). refs: [(x, text)] vertical refs."""
    pw, ph = w - ML - MR, h - MT - MB
    sx = lambda x: ML + (x - xlim[0]) / (xlim[1] - xlim[0]) * pw
    sy = lambda y: MT + ph - (y - ylim[0]) / (ylim[1] - ylim[0]) * ph
    o = [f'<svg viewBox="0 0 {w} {h}" role="img" class="chart" xmlns="http://www.w3.org/2000/svg">']
    for t in yticks:
        o.append(f'<line x1="{ML}" x2="{ML+pw}" y1="{sy(t):.1f}" y2="{sy(t):.1f}" class="grid"/>')
        lab = f'{t*100:.0f}%' if ypct else _fmt(t)
        o.append(f'<text x="{ML-8}" y="{sy(t)+4:.1f}" class="tick" text-anchor="end">{lab}</text>')
    for t in xticks:
        o.append(f'<text x="{sx(t):.1f}" y="{MT+ph+18}" class="tick" text-anchor="middle">{_fmt(t)}</text>')
    o.append(f'<line x1="{ML}" x2="{ML+pw}" y1="{MT+ph}" y2="{MT+ph}" class="axis"/>')
    o.append(f'<text x="{ML+pw/2}" y="{h-6}" class="axl" text-anchor="middle">{html.escape(xlabel)}</text>')
    o.append(f'<text x="14" y="{MT+ph/2}" class="axl" text-anchor="middle" transform="rotate(-90 14 {MT+ph/2})">'
             f'{html.escape(ylabel)}</text>')
    for y, txt in hlines:
        o.append(f'<line x1="{ML}" x2="{ML+pw}" y1="{sy(y):.1f}" y2="{sy(y):.1f}" class="ref"/>')
        o.append(f'<text x="{ML+4}" y="{sy(y)-4:.1f}" class="reft">{html.escape(txt)}</text>')
    for x, txt in refs:
        o.append(f'<line x1="{sx(x):.1f}" x2="{sx(x):.1f}" y1="{MT}" y2="{MT+ph}" class="ref"/>')
        o.append(f'<text x="{sx(x)+4:.1f}" y="{MT+12}" class="reft">{html.escape(txt)}</text>')
    lab_y = []
    for s in series:
        pts = [(x, y) for x, y in s['pts'] if xlim[0] <= x <= xlim[1]]
        if not pts: continue
        d = ' '.join(f'{"M" if i == 0 else "L"}{sx(x):.1f},{sy(max(ylim[0], min(ylim[1], y))):.1f}' for i, (x, y) in enumerate(pts))
        dash = ' stroke-dasharray="6 4"' if s.get('dash') else ''
        o.append(f'<path d="{d}" fill="none" stroke="var({s["color"]})" stroke-width="2"{dash} stroke-linejoin="round"/>')
        for x, y in pts:
            yv = f'{y*100:.1f}%' if ypct else f'{y:.3g}'
            o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(max(ylim[0], min(ylim[1], y))):.1f}" r="7" class="hit">'
                     f'<title>{html.escape(s["name"])}: {xlabel.split(" (")[0]} {x:g} → {yv}</title></circle>')
        # direct label at the right end (or at labels_at x)
        lx, ly = pts[-1]
        if labels_at is not None:
            cand = [p for p in pts if abs(p[0] - labels_at) < 1e-9]
            if cand: lx, ly = cand[0]
        yy = sy(max(ylim[0], min(ylim[1], ly)))
        while any(abs(yy - q) < 13 for q in lab_y): yy += 13
        lab_y.append(yy)
        o.append(f'<circle cx="{sx(lx):.1f}" cy="{sy(max(ylim[0], min(ylim[1], ly))):.1f}" r="4" fill="var({s["color"]})" class="ring"/>')
        o.append(f'<text x="{sx(xlim[1])+8:.1f}" y="{yy+4:.1f}" class="dl">'
                 f'<tspan fill="var({s["color"]})">■</tspan> {html.escape(s["name"])}</text>')
    o.append('</svg>')
    return '\n'.join(o)


def hbar(rows, unit, xmax, w=W, color='--s1', label_w=250, fmt='{:+.1f}'):
    """rows: [(label, value, note)]."""
    rh = 30; h = rh * len(rows) + 30; pw = w - label_w - 70
    o = [f'<svg viewBox="0 0 {w} {h}" role="img" class="chart" xmlns="http://www.w3.org/2000/svg">']
    sx = lambda v: label_w + max(0, v) / xmax * pw
    o.append(f'<line x1="{label_w}" x2="{label_w}" y1="4" y2="{h-24}" class="axis"/>')
    for i, (lab, v, note) in enumerate(rows):
        y = 6 + i * rh
        c = note if isinstance(note, str) and note.startswith('--') else color
        o.append(f'<text x="{label_w-8}" y="{y+16}" class="tick" text-anchor="end">{html.escape(lab)}</text>')
        o.append(f'<rect x="{label_w}" y="{y+4}" width="{max(1, sx(v)-label_w):.1f}" height="{rh-12}" rx="4" fill="var({c})">'
                 f'<title>{html.escape(lab)}: {fmt.format(v)} {unit}</title></rect>')
        o.append(f'<text x="{sx(v)+6:.1f}" y="{y+16}" class="val">{fmt.format(v)} dB</text>')
    o.append(f'<text x="{label_w+pw/2}" y="{h-6}" class="axl" text-anchor="middle">{html.escape(unit)}</text>')
    o.append('</svg>')
    return '\n'.join(o)
