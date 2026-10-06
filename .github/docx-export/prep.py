"""Adjust a copy of a docx so LibreOffice lays it out like Word.

usage: prep.py IN.docx OUT.docx
"""
import functools, re, subprocess, sys, zipfile

ZWSP = "\u200b"
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
SECT_RE = re.compile(r"<w:sectPr\b[^>]*?(?:/>|>.*?</w:sectPr>)", re.S)


def fix_breaks(xml):
    # Word breaks lines at any space. LibreOffice (Unicode line breaking) keeps a space together with a
    # following "/", ")", "!" etc., so "A / B" can't break before "/". A zero-width space allows it.
    def text(m):
        t = re.sub(r" (?=[/)\]}!?;:,.])", " " + ZWSP, m.group(2))
        # Word also breaks after a hyphen inside URLs and paths, where LibreOffice doesn't.
        t = re.sub(r"(?<=[A-Za-z])-(?=[A-Za-z])", "-" + ZWSP, t) if "/" in t else t
        return m.group(1) + t + m.group(3)
    return re.sub(r"(<w:t(?: [^>]*)?>)([^<]*)(</w:t>)", text, xml)


def explicit_headers(doc, parts):
    """Give every section explicit header/footer references.

    Word inherits each header/footer type from the previous section, and a type defined in no earlier
    section is blank. LibreOffice falls back to the default header/footer instead (e.g. a title page
    shows the default footer where Word shows none), so missing types get an empty part.
    """
    even = "w:evenAndOddHeaders" in parts["word/settings.xml"].decode()
    rels = parts["word/_rels/document.xml.rels"].decode()
    types = parts["[Content_Types].xml"].decode()
    empty = {}

    def empty_part(kind):  # kind: header | footer
        if kind not in empty:
            rid, name = f"rIdDocxPrep{kind}", f"{kind}DocxPrep.xml"
            tag = "w:hdr" if kind == "header" else "w:ftr"
            parts[f"word/{name}"] = (
                f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n<{tag} xmlns:w="{W_NS}">'
                f'<w:p><w:pPr><w:spacing w:before="0" w:after="0" w:line="20" w:lineRule="exact"/>'
                f'<w:rPr><w:sz w:val="2"/></w:rPr></w:pPr></w:p></{tag}>'
            ).encode()
            nonlocal rels, types
            rels = rels.replace("</Relationships>", f'<Relationship Id="{rid}" Type="{REL_NS}/{kind}" '
                                f'Target="{name}"/></Relationships>')
            types = types.replace("</Types>", f'<Override PartName="/word/{name}" ContentType="application/'
                                  f'vnd.openxmlformats-officedocument.wordprocessingml.{kind}+xml"/></Types>')
            empty[kind] = rid
        return empty[kind]

    current = {}

    def sect(m):
        sp = m.group(0)
        refs = re.findall(r'<w:(header|footer)Reference\b[^>]*?w:type="(\w+)"[^>]*?r:id="([^"]+)"[^>]*/>', sp)
        refs += [(k, t, r) for k, r, t in
                 re.findall(r'<w:(header|footer)Reference\b[^>]*?r:id="([^"]+)"[^>]*?w:type="(\w+)"[^>]*/>', sp)]
        for kind, typ, rid in refs:
            current[(kind, typ)] = rid
        want = ["default"] + (["first"] if "<w:titlePg" in sp and "<w:titlePg w:val=\"0\"" not in sp else []) \
            + (["even"] if even else [])
        new = "".join(
            f'<w:{kind}Reference w:type="{typ}" r:id="{current.get((kind, typ)) or empty_part(kind)}"/>'
            for kind in ("header", "footer") for typ in want)
        body = re.sub(r"<w:(header|footer)Reference\b[^>]*/>", "", sp)
        if body.endswith("/>"):  # <w:sectPr .../>
            return body[:-2] + ">" + new + "</w:sectPr>"
        head = re.match(r"<w:sectPr\b[^>]*>", body).group(0)
        return head + new + body[len(head):]

    doc = SECT_RE.sub(sect, doc)
    parts["word/_rels/document.xml.rels"] = rels.encode()
    parts["[Content_Types].xml"] = types.encode()
    return doc


def page_break_to_section_break(doc):
    """A paragraph holding only a page break, right before the paragraph that ends a section, followed by
    a continuous section: Word starts the next section on the new page, with that section's first-page
    header/footer. LibreOffice keeps the page in the old section. Make the next section start on a new
    page instead of the break."""
    pb = r'<w:p\b[^>]*>(?:<w:pPr>(?:(?!</w:pPr>).)*</w:pPr>)?<w:r\b[^>]*>(?:<w:rPr>(?:(?!</w:rPr>).)*</w:rPr>)?' \
         r'<w:br w:type="page"/></w:r></w:p>'
    end = r'<w:p\b[^>]*><w:pPr>(?:(?!</w:pPr>|<w:r\b).)*<w:sectPr\b(?:(?!</w:p>).)*</w:pPr></w:p>'
    out, pos = [], 0
    for m in re.finditer(f"({pb})({end})", doc, flags=re.S):
        nxt = SECT_RE.search(doc, m.end())
        if not nxt or '<w:type w:val="continuous"/>' not in nxt.group(0) or m.start() < pos:
            continue
        out += [doc[pos:m.start()], m.group(2), doc[m.end():nxt.start()],
                nxt.group(0).replace('<w:type w:val="continuous"/>', '<w:type w:val="nextPage"/>')]
        pos = nxt.end()
    return "".join(out) + doc[pos:]


def toc_tab_size(doc):
    """TOC entries have a 12pt tab between number and title. Word keeps the 11pt line height there;
    LibreOffice makes those lines taller (0.1pt each, enough to shift TOC page breaks)."""
    def para(m):
        p = m.group(0)
        if not re.search(r'<w:pStyle w:val="TOC\d"/>', p):
            return p
        return re.sub(r'(<w:r>|<w:r [^>]*>)<w:rPr>((?:(?!</w:rPr>).)*)</w:rPr>(<w:tab/></w:r>)',
                      lambda r: r.group(1) + "<w:rPr>" + re.sub(r"<w:sz(?:Cs)? w:val=\"\d+\"/>", "", r.group(2))
                      + "</w:rPr>" + r.group(3), p)
    return re.sub(r"<w:p\b(?:(?!</w:p>).)*</w:p>", para, doc, flags=re.S)


LABEL_FONTS = {"Poppins": "Docx Poppins Label", "Courier New": "Docx Courier Label"}


def label_fonts(numbering):
    """Body-size list numbers and bullets use font variants without descent (see make_fonts.py)."""
    def lvl(m):
        l = m.group(0)
        sz = re.search(r'<w:sz w:val="(\d+)"', l)
        if not sz or int(sz.group(1)) <= 22:
            for font, label in LABEL_FONTS.items():
                l = re.sub(rf'(w:(?:ascii|hAnsi)=)"{font}"', rf'\1"{label}"', l)
        return l
    return re.sub(r"<w:lvl\b.*?</w:lvl>", lvl, numbering, flags=re.S)


def font_names(xml):
    """LibreOffice's built-in replacement table maps Courier New to Liberation Mono when that is
    installed, before fontconfig is asked, so name the substitute (make_fonts.py) directly."""
    return re.sub(r'(w:(?:ascii|hAnsi|cs|eastAsia)=)"Courier New"', r'\1"Docx Courier New"', xml)


def page_break_bullets(doc):
    """A list paragraph holding only a page break shows no bullet in Word. LibreOffice shows one at the
    bottom of the page; turn the numbering off there."""
    def para(m):
        p = m.group(0)
        runs = re.sub(r"<w:pPr>.*?</w:pPr>", "", p, flags=re.S)
        if "<w:numPr>" not in p or 'w:type="page"' not in runs or re.search(r"<w:(?:t|drawing|pict|sym|tab)\b", runs):
            return p
        return re.sub(r"<w:numPr>.*?</w:numPr>", '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="0"/></w:numPr>', p,
                      flags=re.S)
    return re.sub(r"<w:p\b(?:(?!</w:p>).)*</w:p>", para, doc, flags=re.S)


def cell_margins(doc, styles):
    """LibreOffice leaves slightly less room for text in a table cell than Word, so a word that just
    fits in Word wraps. Narrow the right cell margin by 0.1pt."""
    cut = 2  # twips
    style_right = {}
    for m in re.finditer(r'<w:style [^>]*w:type="table"[^>]*w:styleId="([^"]+)".*?</w:style>', styles, re.S):
        r = re.search(r'<w:tblCellMar>(?:(?!</w:tblCellMar>).)*<w:(?:right|end) w:w="(\d+)"', m.group(0), re.S)
        based = re.search(r'<w:basedOn w:val="([^"]+)"', m.group(0))
        style_right[m.group(1)] = (int(r.group(1)) if r else None, based.group(1) if based else None)

    def right_of(sid):
        while sid in style_right:
            r, sid2 = style_right[sid]
            if r is not None:
                return r
            sid = sid2
        return 108  # Word's default (TableNormal)

    def tblpr(m):
        p = m.group(0)
        r = re.search(r'(<w:tblCellMar>(?:(?!</w:tblCellMar>).)*<w:(?:right|end) w:w=")(\d+)"', p, re.S)
        if r:
            return p[:r.start(2)] + str(max(0, int(r.group(2)) - cut)) + p[r.end(2):]
        sid = re.search(r'<w:tblStyle w:val="([^"]+)"', p)
        right = right_of(sid.group(1) if sid else "TableNormal")
        mar = f'<w:tblCellMar><w:right w:w="{max(0, right - cut)}" w:type="dxa"/></w:tblCellMar>'
        # tblCellMar comes before tblLook in the schema
        if "<w:tblLook" in p:
            return p.replace("<w:tblLook", mar + "<w:tblLook", 1)
        return p.replace("</w:tblPr>", mar + "</w:tblPr>")

    doc = re.sub(r"<w:tblPr>.*?</w:tblPr>", tblpr, doc, flags=re.S)
    return re.sub(r'(<w:tcMar>(?:(?!</w:tcMar>).)*<w:(?:right|end) w:w=")(\d+)"',
                  lambda m: m.group(1) + str(max(0, int(m.group(2)) - cut)) + '"', doc, flags=re.S)


def table_indent(doc):
    """In Word 2007-2010 compatibility mode a table's text lines up with the margin and its left border
    sits one cell margin further out. LibreOffice does this only for tables with a style or an indent,
    so give the others an explicit zero indent."""
    def tblpr(m):
        p = m.group(0)
        if "<w:tblInd" in p or "<w:tblStyle" in p:
            return p
        for after in ("<w:tblCellSpacing", "<w:jc ", "<w:tblW "):
            i = p.find(after)
            if i >= 0:
                j = p.find("/>", i) + 2
                return p[:j] + '<w:tblInd w:w="0" w:type="dxa"/>' + p[j:]
        return p.replace("<w:tblPr>", '<w:tblPr><w:tblInd w:w="0" w:type="dxa"/>', 1)
    return re.sub(r"<w:tblPr>.*?</w:tblPr>", tblpr, doc, flags=re.S)


def theme_east_asia(xml, theme):
    """Theme East Asian fonts: these docs' themes leave the "ea" typeface empty, so Word uses the
    Simplified Chinese script font (DengXian). LibreOffice falls back to an unrelated font; name the
    font directly."""
    fonts = {}
    for kind in ("major", "minor"):
        m = re.search(rf"<a:{kind}Font>(.*?)</a:{kind}Font>", theme, re.S)
        if not m:
            continue
        ea = re.search(r'<a:ea typeface="([^"]*)"', m.group(1))
        hans = re.search(r'<a:font script="Hans" typeface="([^"]*)"', m.group(1))
        name = (ea and ea.group(1)) or (hans and hans.group(1))
        if name:
            fonts[kind + "EastAsia"] = name

    def rfonts(m):
        tag = m.group(0)
        theme_attr = re.search(r'w:eastAsiaTheme="(\w+)"', tag)
        if not theme_attr or theme_attr.group(1) not in fonts:
            return tag
        tag = re.sub(r'\s+w:eastAsia="[^"]*"', "", tag)
        return tag.replace(theme_attr.group(0), f'w:eastAsia="{fonts[theme_attr.group(1)]}"')
    return re.sub(r"<w:rFonts\b[^>]*/>", rfonts, xml)


SYMBOLS = re.compile("([\u2600-\u27bf\u2b00-\u2bff]+)")


def symbol_runs(xml):
    """Word shows symbols missing from Calibri (check marks) in Segoe UI Symbol, and that font's height
    counts for the line. LibreOffice's glyph fallback doesn't change line height, so put the symbols in
    their own runs with the font named."""
    def run(m):
        r = m.group(0)
        t = re.search(r"(<w:t(?: [^>]*)?>)([^<]*)</w:t>", r)
        if not t or not SYMBOLS.search(t.group(2)) or "<w:sym " in r:
            return r
        rpr = re.search(r"<w:rPr>(.*?)</w:rPr>", r[:t.start()], re.S)
        inner = rpr.group(1) if rpr else ""
        style = re.match(r"\s*(<w:rStyle [^>]*/>)?(.*)", inner, re.S)
        sym_inner = (style.group(1) or "") + '<w:rFonts w:ascii="Segoe UI Symbol" w:hAnsi="Segoe UI Symbol"/>' \
            + re.sub(r"<w:rFonts\b[^>]*/>", "", style.group(2))
        open_r = re.match(r"<w:r\b[^>]*>", r).group(0)
        out = []
        for i, part in enumerate(SYMBOLS.split(t.group(2))):
            if not part:
                continue
            props = f"<w:rPr>{sym_inner}</w:rPr>" if i % 2 else (f"<w:rPr>{inner}</w:rPr>" if rpr else "")
            out.append(f'{open_r}{props}<w:t xml:space="preserve">{part}</w:t></w:r>')
        return "".join(out)
    # Only simple runs: properties and one text element
    return re.sub(r"<w:r\b[^>]*>(?:<w:rPr>(?:(?!</w:rPr>).)*</w:rPr>)?<w:t(?: [^>]*)?>[^<]*</w:t></w:r>", run, xml, flags=re.S)


def style_spacing(styles):
    """styleId -> (before, after) in twips, following basedOn and the document defaults."""
    defaults = re.search(r"<w:pPrDefault>.*?</w:pPrDefault>", styles, re.S)
    own, based = {}, {}
    for m in re.finditer(r'<w:style [^>]*w:type="paragraph"[^>]*w:styleId="([^"]+)".*?</w:style>', styles, re.S):
        sp = re.search(r"<w:spacing [^>]*/>", m.group(0))
        own[m.group(1)] = sp.group(0) if sp else ""
        b = re.search(r'<w:basedOn w:val="([^"]+)"', m.group(0))
        based[m.group(1)] = b.group(1) if b else None

    def value(sid, attr):
        seen = set()
        while sid and sid not in seen:
            seen.add(sid)
            v = re.search(rf'w:{attr}="(\d+)"', own.get(sid, ""))
            if v:
                return int(v.group(1))
            sid = based.get(sid)
        v = re.search(rf'<w:spacing [^>]*w:{attr}="(\d+)"', defaults.group(0)) if defaults else None
        return int(v.group(1)) if v else 0
    return lambda sid: (value(sid, "before"), value(sid, "after"))


def after_columns(doc, styles):
    """After a continuous section break that ends a multi-column section, Word places the next paragraph
    about 8pt higher than LibreOffice (measured on the datasheet title pages). That matches dropping the
    section-break paragraph's space after from the next paragraph's space before."""
    spacing = style_spacing(styles)
    end = re.compile(r'<w:p\b[^>]*><w:pPr>((?:(?!</w:pPr>).)*<w:sectPr\b(?:(?!</w:sectPr>).)*'
                     r'<w:cols [^>]*w:num="[2-9]"(?:(?!</w:sectPr>).)*</w:sectPr>)</w:pPr></w:p>(<w:p\b[^>]*>)', re.S)
    out, pos = [], 0
    for m in end.finditer(doc):
        nxt = SECT_RE.search(doc, m.end())
        if not nxt or '<w:type w:val="continuous"/>' not in nxt.group(0):
            continue
        sect_style = re.search(r'<w:pStyle w:val="([^"]+)"', m.group(1))
        cut = spacing(sect_style.group(1) if sect_style else "Normal")[1]
        ppr = re.match(r"<w:pPr>(?:(?!</w:pPr>).)*</w:pPr>", doc[m.end():], re.S)
        if not ppr or "<w:spacing " in ppr.group(0):
            continue  # direct spacing: leave it
        style = re.search(r'<w:pStyle w:val="([^"]+)"', ppr.group(0))
        before = spacing(style.group(1) if style else "Normal")[0]
        new_ppr = ppr.group(0).replace("<w:pPr>", "<w:pPr>", 1)
        # w:spacing goes after pStyle, keepNext, keepLines, pageBreakBefore, framePr, widowControl,
        # numPr, suppressLineNumbers, pBdr, shd, tabs, suppressAutoHyphens
        anchor = re.search(r"<w:(?:ind|contextualSpacing|jc|outlineLvl|rPr|sectPr)\b|</w:pPr>", new_ppr)
        new_ppr = new_ppr[:anchor.start()] + f'<w:spacing w:before="{max(0, before - cut)}"/>' + new_ppr[anchor.start():]
        out += [doc[pos:m.end()], new_ppr]
        pos = m.end() + len(ppr.group(0))
    return "".join(out) + doc[pos:]


class Styles:
    """Run and paragraph properties from styles.xml (basedOn chains, document defaults, theme fonts)."""

    def __init__(self, styles, theme):
        self.styles, self.default_para = {}, "Normal"
        for m in re.finditer(r'<w:style [^>]*w:type="(\w+)"[^>]*w:styleId="([^"]+)".*?</w:style>', styles, re.S):
            based = re.search(r'<w:basedOn w:val="([^"]+)"', m.group(0))
            self.styles[m.group(2)] = (m.group(0), based.group(1) if based else None)
            if m.group(1) == "paragraph" and 'w:default="1"' in m.group(0)[:m.group(0).find(">")]:
                self.default_para = m.group(2)
        d = re.search(r"<w:docDefaults>.*?</w:docDefaults>", styles, re.S)
        self.defaults = d.group(0) if d else ""
        self.theme = {}
        for kind in ("major", "minor"):
            f = re.search(rf'<a:{kind}Font><a:latin typeface="([^"]*)"', theme)
            if f:
                self.theme[kind + "HAnsi"] = f.group(1)

    def chain(self, sid):
        seen = []
        while sid in self.styles and sid not in seen:
            seen.append(sid)
            sid = self.styles[sid][1]
        return [self.styles[s][0] for s in seen]

    @staticmethod
    def first(xmls, tag, pattern):
        """First match of pattern inside <tag>...</tag> of the first xml that has it."""
        for x in xmls:
            for block in re.findall(rf"<w:{tag}>(.*?)</w:{tag}>", x, re.S):
                m = re.search(pattern, block)
                if m:
                    return m
        return None


@functools.lru_cache(maxsize=None)
def hb_font(family, bold, italic):
    """HarfBuzz font for a Word font name, as fontconfig substitutes it (as LibreOffice will)."""
    try:
        import uharfbuzz as hb
        path = subprocess.run(["fc-match", "-f", "%{file}", f"{family}:weight={200 if bold else 80}"
                               f":slant={100 if italic else 0}"], capture_output=True, text=True, check=True).stdout
        with open(path, "rb") as f:
            return hb.Font(hb.Face(hb.Blob(f.read())))
    except Exception:
        return None


def shaped_width(font, text, size):
    """Advance width in twips of text in font at size (half-points), with kerning and ligatures."""
    import uharfbuzz as hb
    buf = hb.Buffer()
    buf.add_str(text)
    buf.guess_segment_properties()
    hb.shape(font, buf)
    if any(i.codepoint == 0 for i in buf.glyph_infos):
        return None  # glyph missing: LibreOffice would use a fallback font
    return sum(p.x_advance for p in buf.glyph_positions) / font.face.upem * size * 10


def text_width(runs, styles, pstyle):
    """Width in twips of the runs' text on one line, or None if it can't be measured."""
    segs = []
    for r in runs:
        rpr = re.search(r"<w:rPr>(.*?)</w:rPr>", r, re.S)
        rpr = rpr.group(1) if rpr else ""
        if re.search(r"<w:(?:tab|ptab|br|cr|drawing|pict|object|sym|fldChar|instrText|footnoteReference|"
                     r"endnoteReference)\b", r) or re.search(r"<w:(?:caps|smallCaps|spacing|w|vertAlign|position)\b",
                                                             rpr):
            return None
        text = "".join(re.findall(r"<w:t(?: [^>]*)?>([^<]*)</w:t>", r)).replace(ZWSP, "")
        if not text:
            continue
        text = text.replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&").replace("&quot;", '"')
        rstyle = re.search(r'<w:rStyle w:val="([^"]+)"', rpr)
        xmls = [f"<w:rPr>{rpr}</w:rPr>"] + (styles.chain(rstyle.group(1)) if rstyle else []) \
            + styles.chain(pstyle) + [styles.defaults]
        font = styles.first(xmls, "rPr", r'<w:rFonts [^>]*?w:(ascii(?:Theme)?)="([^"]+)"')
        if not font:
            return None
        family = styles.theme.get(font.group(2)) if font.group(1) == "asciiTheme" else font.group(2)
        sz = styles.first(xmls, "rPr", r'<w:sz w:val="(\d+)"')

        def on(tag):
            m = styles.first(xmls, "rPr", rf'<w:{tag}(?: w:val="(\w+)")?/>')
            return bool(m) and m.group(1) not in ("0", "false", "off")
        font = hb_font(family, on("b"), on("i")) if family else None
        if not font or not sz:
            return None
        segs.append([font, text, int(sz.group(1))])
    if segs:
        segs[-1][1] = segs[-1][1].rstrip(" ")
    widths = [shaped_width(*seg) for seg in segs]
    return None if None in widths else sum(widths)


def one_line_widows(doc, styles):
    """LibreOffice moves a paragraph back to the previous page only if it fits there as currently
    formatted. A one-line paragraph that LibreOffice once formatted at the top of the next page beside a
    header object (the bee) has two lines there, and widow/orphan control keeps it from splitting, so it
    stays on that page (1090 title page). Widow control has no effect on a paragraph Word sets on one
    line, so turn it off for body paragraphs that measure one line. Measured with the substitute fonts
    and HarfBuzz; trailing spaces don't count."""
    tables, depth, start = [], 0, 0  # spans of tables and text boxes
    for m in re.finditer(r"</?w:(?:tbl|txbxContent)>", doc):
        if m.group(0)[1] != "/":
            depth += 1
            start = m.start() if depth == 1 else start
        else:
            depth -= 1
            if depth == 0:
                tables.append((start, m.end()))
    sects = [(m.start(), m.group(0)) for m in SECT_RE.finditer(doc)]

    def avail(pos):
        sp = next((s for p, s in sects if p >= pos), sects[-1][1] if sects else "")
        w = re.search(r'<w:pgSz [^>]*w:w="(\d+)"', sp)
        lm = re.search(r'<w:pgMar [^>]*w:left="(\d+)"', sp)
        rm = re.search(r'<w:pgMar [^>]*w:right="(\d+)"', sp)
        if not (w and lm and rm) or 'w:equalWidth="0"' in sp:
            return None
        cols = re.search(r'<w:cols [^>]*w:num="(\d+)"', sp)
        n = int(cols.group(1)) if cols else 1
        space = re.search(r'<w:cols [^>]*w:space="(\d+)"', sp)
        space = int(space.group(1)) if space else 720
        return (int(w.group(1)) - int(lm.group(1)) - int(rm.group(1)) - (n - 1) * space) / n

    out, pos, ti = [], 0, 0
    for m in re.finditer(r"<w:p\b[^>]*>((?:(?!</w:p>).)*)</w:p>", doc, re.S):
        while ti < len(tables) and tables[ti][1] <= m.start():
            ti += 1
        if ti < len(tables) and tables[ti][0] <= m.start() or "<w:p " in m.group(1) or "<w:p>" in m.group(1):
            continue
        ppr = re.match(r"<w:pPr>.*?</w:pPr>", m.group(1), re.S)
        ppr = ppr.group(0) if ppr else ""
        if "<w:widowControl" in ppr or "<w:numPr>" in ppr or "<w:framePr" in ppr:
            continue
        pstyle = re.search(r'<w:pStyle w:val="([^"]+)"', ppr)
        pstyle = pstyle.group(1) if pstyle else styles.default_para
        pxmls = [ppr] + styles.chain(pstyle) + [styles.defaults]
        if styles.first(pxmls, "pPr", r"<w:(?:numPr|widowControl)\b"):
            continue
        width = avail(m.start())
        if width is None:
            continue
        ind = styles.first(pxmls, "pPr", r"<w:ind [^>]*/>")
        for attr, sign in (("(?:left|start)", -1), ("(?:right|end)", -1), ("firstLine", -1), ("hanging", 1)):
            v = re.search(rf'w:{attr}="(-?\d+)"', ind.group(0)) if ind else None
            width += sign * int(v.group(1)) if v else 0
        runs = re.findall(r"<w:r\b[^>]*>.*?</w:r>", m.group(1)[len(ppr):], re.S)
        text = text_width(runs, styles, pstyle) if runs else None
        if not text or text > width - 10:  # 0.5pt to spare
            continue
        p = m.group(0)
        if ppr:
            # widowControl follows pStyle, keepNext, keepLines, pageBreakBefore, framePr
            anchor = re.search(r"<w:(?!pStyle|keepNext|keepLines|pageBreakBefore|framePr)\w+\b|</w:pPr>", ppr[7:])
            i = m.start(1) - m.start() + 7 + anchor.start()
        else:
            i = m.start(1) - m.start()
        new = '<w:widowControl w:val="0"/>' if ppr else '<w:pPr><w:widowControl w:val="0"/></w:pPr>'
        out += [doc[pos:m.start()], p[:i], new, p[i:]]
        pos = m.end()
    return "".join(out) + doc[pos:]


def main(src, dst):
    with zipfile.ZipFile(src) as zin:
        infos = zin.infolist()
        parts = {i.filename: zin.read(i.filename) for i in infos}
    for name in list(parts):
        if re.fullmatch(r"word/(document|header\d*|footer\d*|footnotes|endnotes)\.xml", name):
            parts[name] = symbol_runs(fix_breaks(parts[name].decode())).encode()
    doc = parts["word/document.xml"].decode()
    doc = page_break_to_section_break(doc)
    doc = toc_tab_size(doc)
    doc = page_break_bullets(doc)
    doc = after_columns(doc, parts["word/styles.xml"].decode())
    doc = explicit_headers(doc, parts)
    doc = table_indent(doc)
    doc = cell_margins(doc, parts["word/styles.xml"].decode())
    theme = parts.get("word/theme/theme1.xml", b"").decode()
    doc = one_line_widows(doc, Styles(parts["word/styles.xml"].decode(), theme))
    parts["word/document.xml"] = doc.encode()
    for name in list(parts):
        if re.fullmatch(r"word/(document|styles|numbering|header\d*|footer\d*|footnotes|endnotes)\.xml", name):
            parts[name] = theme_east_asia(parts[name].decode(), theme).encode()
    if "word/numbering.xml" in parts:
        parts["word/numbering.xml"] = label_fonts(parts["word/numbering.xml"].decode()).encode()
    for name in list(parts):
        if re.fullmatch(r"word/(document|styles|numbering|header\d*|footer\d*|footnotes|endnotes)\.xml", name):
            parts[name] = font_names(parts[name].decode()).encode()
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for i in infos:
            zout.writestr(i, parts.pop(i.filename))
        for name, data in parts.items():
            zout.writestr(name, data)


if __name__ == "__main__":
    main(*sys.argv[1:3])
