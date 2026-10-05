"""Adjust a copy of a docx so LibreOffice lays it out like Word.

usage: prep.py IN.docx OUT.docx
"""
import re, sys, zipfile

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
                f'<w:p><w:pPr><w:spacing w:after="0" w:line="240" w:lineRule="auto"/></w:pPr></w:p></{tag}>'
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


def label_fonts(numbering):
    """Body-size list numbers in Poppins use a variant without descent (see make_fonts.py)."""
    def lvl(m):
        l = m.group(0)
        sz = re.search(r'<w:sz w:val="(\d+)"', l)
        if sz and int(sz.group(1)) <= 22:
            l = re.sub(r'(w:(?:ascii|hAnsi)=)"Poppins"', r'\1"Docx Poppins Label"', l)
        return l
    return re.sub(r"<w:lvl\b.*?</w:lvl>", lvl, numbering, flags=re.S)


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
    doc = explicit_headers(doc, parts)
    doc = table_indent(doc)
    doc = cell_margins(doc, parts["word/styles.xml"].decode())
    parts["word/document.xml"] = doc.encode()
    theme = parts.get("word/theme/theme1.xml", b"").decode()
    for name in list(parts):
        if re.fullmatch(r"word/(document|styles|numbering|header\d*|footer\d*|footnotes|endnotes)\.xml", name):
            parts[name] = theme_east_asia(parts[name].decode(), theme).encode()
    if "word/numbering.xml" in parts:
        parts["word/numbering.xml"] = label_fonts(parts["word/numbering.xml"].decode()).encode()
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for i in infos:
            zout.writestr(i, parts.pop(i.filename))
        for name, data in parts.items():
            zout.writestr(name, data)


if __name__ == "__main__":
    main(*sys.argv[1:3])
