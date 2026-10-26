"""Tiny inline-SVG chart helpers (themed with CSS custom properties defined in the report)."""
import html

SER = ['var(--s1)', 'var(--s2)', 'var(--s3)', 'var(--s4)', 'var(--s5)', 'var(--s6)']


def esc(s):
    return html.escape(str(s))


class Axes:
    def __init__(self, w, h, xr, yr, ml=56, mr=16, mt=14, mb=40):
        self.w, self.h, self.xr, self.yr = w, h, xr, yr
        self.ml, self.mr, self.mt, self.mb = ml, mr, mt, mb
        self.parts = []

    def x(self, v):
        return self.ml + (v - self.xr[0]) / (self.xr[1] - self.xr[0]) * (self.w - self.ml - self.mr)

    def y(self, v):
        return self.mt + (1 - (v - self.yr[0]) / (self.yr[1] - self.yr[0])) * (self.h - self.mt - self.mb)

    def grid(self, xt, yt, xlabel, ylabel, xfmt=str, yfmt=str):
        p = self.parts
        for v in yt:
            p.append(f'<line x1="{self.ml}" x2="{self.w - self.mr}" y1="{self.y(v):.1f}" y2="{self.y(v):.1f}" class="grid"/>')
            p.append(f'<text x="{self.ml - 6}" y="{self.y(v) + 4:.1f}" class="tick" text-anchor="end">{esc(yfmt(v))}</text>')
        for v in xt:
            p.append(f'<text x="{self.x(v):.1f}" y="{self.h - self.mb + 16}" class="tick" text-anchor="middle">{esc(xfmt(v))}</text>')
        p.append(f'<line x1="{self.ml}" x2="{self.w - self.mr}" y1="{self.h - self.mb}" y2="{self.h - self.mb}" class="axis"/>')
        p.append(f'<text x="{(self.ml + self.w - self.mr) / 2:.1f}" y="{self.h - 6}" class="lab" text-anchor="middle">{esc(xlabel)}</text>')
        p.append(f'<text x="12" y="{(self.mt + self.h - self.mb) / 2:.1f}" class="lab" text-anchor="middle" '
                 f'transform="rotate(-90 12 {(self.mt + self.h - self.mb) / 2:.1f})">{esc(ylabel)}</text>')

    def line(self, pts, color, name, dash=None, markers=True, fmt=lambda x, y: f'{x}, {y}', width=2, step=False):
        pts = [(a, b) for a, b in pts if b is not None]
        if not pts:
            return
        if step:
            d = f'M{self.x(pts[0][0]):.1f},{self.y(pts[0][1]):.1f}'
            for (a0, b0), (a, b) in zip(pts, pts[1:]):
                d += f' L{self.x(a):.1f},{self.y(b0):.1f} L{self.x(a):.1f},{self.y(b):.1f}'
        else:
            d = 'M' + ' L'.join(f'{self.x(a):.1f},{self.y(b):.1f}' for a, b in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ''
        self.parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da} stroke-linejoin="round"/>')
        if markers:
            for a, b in pts:
                self.parts.append(f'<circle cx="{self.x(a):.1f}" cy="{self.y(b):.1f}" r="4" fill="{color}" stroke="var(--surface)" '
                                  f'stroke-width="2"><title>{esc(name)}: {esc(fmt(a, b))}</title></circle>')

    def point(self, a, b, color, title, shape='diamond', r=6):
        X, Y = self.x(a), self.y(b)
        if shape == 'diamond':
            self.parts.append(f'<path d="M{X},{Y - r} L{X + r},{Y} L{X},{Y + r} L{X - r},{Y} Z" fill="{color}" stroke="var(--surface)" '
                              f'stroke-width="2"><title>{esc(title)}</title></path>')
        else:
            self.parts.append(f'<circle cx="{X:.1f}" cy="{Y:.1f}" r="{r}" fill="{color}" stroke="var(--surface)" stroke-width="2">'
                              f'<title>{esc(title)}</title></circle>')

    def text(self, a, b, s, cls='dl', anchor='start', dx=6, dy=4):
        self.parts.append(f'<text x="{self.x(a) + dx:.1f}" y="{self.y(b) + dy:.1f}" class="{cls}" text-anchor="{anchor}">{esc(s)}</text>')

    def vline(self, a, cls='ref', label=None):
        self.parts.append(f'<line x1="{self.x(a):.1f}" x2="{self.x(a):.1f}" y1="{self.mt}" y2="{self.h - self.mb}" class="{cls}"/>')
        if label:
            self.parts.append(f'<text x="{self.x(a) + 4:.1f}" y="{self.mt + 10}" class="tick">{esc(label)}</text>')

    def hline(self, b, cls='ref', label=None):
        self.parts.append(f'<line x1="{self.ml}" x2="{self.w - self.mr}" y1="{self.y(b):.1f}" y2="{self.y(b):.1f}" class="{cls}"/>')
        if label:
            self.parts.append(f'<text x="{self.w - self.mr - 2}" y="{self.y(b) - 4:.1f}" class="tick" text-anchor="end">{esc(label)}</text>')

    def svg(self, title):
        return (f'<svg viewBox="0 0 {self.w} {self.h}" role="img" aria-label="{esc(title)}" class="chart">'
                + ''.join(self.parts) + '</svg>')


def legend(items):
    return '<div class="legend">' + ''.join(
        f'<span><i style="background:{c}{";height:2px;border-top:2px dashed " + c + ";background:none" if d else ""}"></i>{esc(n)}</span>'
        for n, c, d in items) + '</div>'
