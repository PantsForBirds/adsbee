"""Build substitute fonts for Word fonts that have no metric-compatible free clone.

usage: make_fonts.py SRCDIR OUTDIR

SRCDIR holds the free source fonts (see docx-export.yml). Each substitute keeps the source glyphs and
takes the line metrics of the Word font it replaces, so lines get the same height as in Word:

- Symbol: URW Standard Symbols PS (Adobe Symbol widths) with a symbol cmap (U+F020-F0FF), for list
  bullets such as U+F0B7. Metrics of Word's SymbolMT.
- Docx Consolas: Inconsolata at the width where its advance matches Consolas, Consolas metrics.
- Docx Poppins Label: Poppins for body-size list numbers, without descent (see prep.py).
- Docx Segoe Symbol: DejaVu Sans symbols (check marks), Segoe UI Symbol metrics.
- Docx DengXian, Docx SimSun: Droid Sans Fallback (full-width punctuation), DengXian / SimSun metrics.
"""
import sys
from fontTools import subset
from fontTools.ttLib import TTFont
from fontTools.ttLib.tables._c_m_a_p import cmap_format_4
from fontTools.varLib.instancer import instantiateVariableFont


def set_names(font, family, style="Regular"):
    name = font["name"]
    ps = (family + "-" + style).replace(" ", "")
    for nid in (1, 2, 3, 4, 6, 16, 17, 21, 22, 25):
        name.removeNames(nameID=nid)
    for nid, val in ((1, family), (2, style), (3, ps), (4, f"{family} {style}"), (6, ps)):
        name.setName(val, nid, 3, 1, 0x409)
        name.setName(val, nid, 1, 0, 0)
    if "CFF " in font:
        cff = font["CFF "].cff
        cff.fontNames = [ps]
        top = cff.topDictIndex[0]
        top.FullName, top.FamilyName = f"{family} {style}", family


def set_vmetrics(font, asc, desc):
    """Ascent and descent in em, written to hhea, typo and win alike."""
    upem = font["head"].unitsPerEm
    asc, desc = round(asc * upem), round(desc * upem)
    hhea, os2 = font["hhea"], font["OS/2"]
    hhea.ascent, hhea.descent, hhea.lineGap = asc, -desc, 0
    os2.sTypoAscender, os2.sTypoDescender, os2.sTypoLineGap = asc, -desc, 0
    os2.usWinAscent, os2.usWinDescent = asc, desc


def symbol(src, out):
    f = TTFont(src)
    # Standard Symbols PS maps Symbol-encoding positions as code points (0x20-0xFF).
    sym = cmap_format_4(4)
    sym.platformID, sym.platEncID, sym.language = 3, 0, 0
    sym.cmap = {0xF000 + k: v for k, v in f.getBestCmap().items() if k < 0x100}
    f["cmap"].tables = [sym]
    os2 = f["OS/2"]
    os2.ulCodePageRange1, os2.ulCodePageRange2 = 1 << 31, 0  # symbol character set
    os2.usFirstCharIndex, os2.usLastCharIndex = min(sym.cmap), max(sym.cmap)
    set_vmetrics(f, 2059 / 2048, 450 / 2048)
    set_names(f, "Symbol")
    f.save(out)


def consolas(src, out):
    target = 1126 / 2048  # Consolas advance, em
    lo, hi = 100.0, 200.0
    for _ in range(30):  # advance grows with wdth
        wdth = (lo + hi) / 2
        f = instantiateVariableFont(TTFont(src), {"wdth": wdth, "wght": 400})
        adv = f["hmtx"]["a"][0] / f["head"].unitsPerEm
        if abs(adv - target) < 0.0005:
            break
        lo, hi = (wdth, hi) if adv < target else (lo, wdth)
    for t in ("STAT", "MVAR"):
        if t in f:
            del f[t]
    set_vmetrics(f, 1884 / 2048, 514 / 2048)
    set_names(f, "Docx Consolas")
    f.save(out)


def shim(src, out, family, asc, desc, unicodes=None):
    f = TTFont(src)
    if unicodes:
        opts = subset.Options()
        opts.name_IDs = ["*"]
        opts.notdef_outline = True
        sub = subset.Subsetter(opts)
        sub.populate(unicodes=unicodes)
        sub.subset(f)
    set_vmetrics(f, asc, desc)
    set_names(f, family)
    f.save(out)


if __name__ == "__main__":
    src, out = sys.argv[1:3]
    symbol(f"{src}/StandardSymbolsPS.otf", f"{out}/DocxSymbol.otf")
    consolas(f"{src}/Inconsolata[wdth,wght].ttf", f"{out}/DocxConsolas.ttf")
    shim(f"{src}/Poppins-Regular.ttf", f"{out}/DocxPoppinsLabel.ttf", "Docx Poppins Label", 1135 / 1000, 0)
    symbols = [*range(0x2190, 0x2200), *range(0x2600, 0x27C0), *range(0x2B00, 0x2C00)]
    shim(f"{src}/DejaVuSans.ttf", f"{out}/DocxSegoeSymbol.ttf", "Docx Segoe Symbol", 2210 / 2048, 514 / 2048,
         symbols)
    shim(f"{src}/DroidSansFallbackFull.ttf", f"{out}/DocxDengXian.ttf", "Docx DengXian", 1659 / 2048, 475 / 2048)
    shim(f"{src}/DroidSansFallbackFull.ttf", f"{out}/DocxSimSun.ttf", "Docx SimSun", 220 / 256, 36 / 256)
