#!/usr/bin/env python3
"""C5VRX-5 by Twotoz and the C5VRX contributors: menu font generator.

The menu raster has 384 x 56 logical pixels on a nearly square picture
area, so one logical pixel is about six times taller than wide. A glyph
row therefore reads like a thick horizontal stroke, and a vertical stem
needs three pixels to look like the same weight. The fonts below are
drawn for that grid: a 5-row caps font for text and an 11-row display font
for hero values, derived from the text font so both share one design.

  python3 tools/menu_font.py           write main/menu_font.h
  python3 tools/menu_font.py --check   fail when main/menu_font.h is stale
"""
import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / "main" / "menu_font.h"

# Text font: 5 rows, '#' ink, '.' paper. Every row of a glyph has the
# glyph's width. Lower case renders as upper case.
TEXT = {
    " ": ["....."] * 5,
    "A": [".#########.", "###.....###", "###########", "###.....###", "###.....###"],
    "B": ["##########.", "###.....###", "##########.", "###.....###", "##########."],
    "C": [".##########", "###........", "###........", "###........", ".##########"],
    "D": ["##########.", "###.....###", "###.....###", "###.....###", "##########."],
    "E": ["##########", "###.......", "########..", "###.......", "##########"],
    "F": ["##########", "###.......", "########..", "###.......", "###......."],
    "G": [".##########", "###........", "###...#####", "###.....###", ".#########."],
    "H": ["###.....###", "###.....###", "###########", "###.....###", "###.....###"],
    "I": ["###"] * 5,
    "J": [".......###", ".......###", ".......###", "###....###", ".########."],
    "K": ["###.....###", "###...###..", "#######....", "###...###..", "###.....###"],
    "L": ["###.......", "###.......", "###.......", "###.......", "##########"],
    "M": ["####.......####", "###.##...##.###", "###..#####..###", "###.........###",
          "###.........###"],
    "N": ["####.....###", "###.##...###", "###..##..###", "###...##.###", "###.....####"],
    "O": [".#########.", "###.....###", "###.....###", "###.....###", ".#########."],
    "P": ["##########.", "###.....###", "##########.", "###........", "###........"],
    "Q": [".#########.", "###.....###", "###.....###", "###....####", ".##########"],
    "R": ["##########.", "###.....###", "##########.", "###....###.", "###.....###"],
    "S": [".##########", "###........", ".#########.", "........###", "##########."],
    "T": ["###########", "....###....", "....###....", "....###....", "....###...."],
    "U": ["###.....###", "###.....###", "###.....###", "###.....###", ".#########."],
    "V": ["###.....###", "###.....###", ".###...###.", "..###.###..", "...#####..."],
    "W": ["###.........###", "###.........###", "###..#####..###", "###.##...##.###",
          ".####.....####."],
    "X": ["###.....###", ".###...###.", "...#####...", ".###...###.", "###.....###"],
    "Y": ["###.....###", ".###...###.", "...#####...", "....###....", "....###...."],
    "Z": ["###########", "......###..", "....###....", "..###......", "###########"],
    "0": [".########.", "###....###", "###....###", "###....###", ".########."],
    "1": ["..#####...", "....###...", "....###...", "....###...", "..#######."],
    "2": ["#########.", ".......###", ".########.", "###.......", "##########"],
    "3": ["#########.", ".......###", "..#######.", ".......###", "#########."],
    "4": ["###....###", "###....###", "##########", ".......###", ".......###"],
    "5": ["##########", "###.......", "#########.", ".......###", "#########."],
    "6": [".#########", "###.......", "#########.", "###....###", ".########."],
    "7": ["##########", ".......###", "......###.", ".....###..", "....###..."],
    "8": [".########.", "###....###", ".########.", "###....###", ".########."],
    "9": [".########.", "###....###", ".#########", ".......###", "#########."],
    ".": ["...", "...", "...", "...", "###"],
    ",": ["...", "...", "...", "###", ".##"],
    ":": ["...", "###", "...", "###", "..."],
    "-": ["......", "......", "######", "......", "......"],
    "+": [".........", "...###...", "#########", "...###...", "........."],
    "/": ["......###", ".....###.", "...###...", ".###.....", "###......"],
    "%": ["###....###", "###...###.", "....###...", ".###...###", "###....###"],
    "!": ["###", "###", "###", "...", "###"],
    "?": ["#########.", ".......###", "...######.", "..........", "...###...."],
    "(": ["..###", "###..", "###..", "###..", "..###"],
    ")": ["###..", "..###", "..###", "..###", "###.."],
    "<": ["......###", "...###...", "###......", "...###...", "......###"],
    ">": ["###......", "...###...", "......###", "...###...", "###......"],
    "'": ["###", "###", "...", "...", "..."],
    "=": [".........", "#########", ".........", "#########", "........."],
    "_": [".........", ".........", ".........", ".........", "#########"],
    "*": [".....", ".###.", ".###.", ".###.", "....."],   # bullet
}

# Display glyphs that read better drawn by hand than scaled.
DISPLAY_OVERRIDE = {}

TEXT_ROWS = 5
DISPLAY_ROWS = 11
DISPLAY_X = 2           # display font pixels per text font pixel, horizontally
DISPLAY_BOLD = 2        # extra stem width: doubled 3-pixel stems still read thin


def bitmap(rows):
    return [[1 if c == "#" else 0 for c in r] for r in rows]


def display_glyph(rows):
    """Scale a text glyph to the display font. Text rows map to display
    rows 2/2/2/3/2 (top, upper counter, bar, lower counter, base), so every
    stroke keeps one weight; columns double and stems gain DISPLAY_BOLD
    pixels unless that would close a counter. Where a run moves sideways
    from one text row to the next (a diagonal), the display rows in
    between slide the run's edges linearly, so diagonals step smoothly
    instead of in blocks."""
    def runs(row):
        out, x = [], 0
        while x < len(row):
            if row[x] == "#":
                s = x
                while x < len(row) and row[x] == "#":
                    x += 1
                out.append((s * DISPLAY_X, x * DISPLAY_X))
            else:
                x += 1
        bold = []
        for i, (a, b) in enumerate(out):
            room = out[i + 1][0] - b if i + 1 < len(out) else DISPLAY_BOLD + 4
            bold.append((a, b + DISPLAY_BOLD if room >= DISPLAY_BOLD + 4 else b))
        return bold

    src = [runs(r) for r in rows]
    width = len(rows[0]) * DISPLAY_X + DISPLAY_BOLD
    # Display rows: (text row, position inside the row, -0.5..+0.5).
    split = [2, 2, 2, 3, 2]
    places = []
    for k, n in enumerate(split):
        places += [(k, (i + 0.5) / n - 0.5) for i in range(n)]
    assert len(places) == DISPLAY_ROWS

    def moving(a, b):
        # Same run count, every run overlapping a partner of about its
        # width (a stroke that shifts, not a bar that ends), some moved.
        return (len(a) == len(b) and a != b and
                all(p[0] < q[1] and q[0] < p[1] and
                    abs((p[1] - p[0]) - (q[1] - q[0])) <= DISPLAY_X for p, q in zip(a, b)))

    out = []
    for k, t in places:
        here = src[k]
        other = k + (1 if t > 0 else -1)
        spans = here
        if 0 <= other < len(src) and t != 0 and moving(here, src[other]):
            f = abs(t)
            spans = [(round(p[0] + (q[0] - p[0]) * f), round(p[1] + (q[1] - p[1]) * f))
                     for p, q in zip(here, src[other])]
        row = ["."] * width
        for a, b in spans:
            for x in range(a, b):
                row[x] = "#"
        out.append("".join(row))
    return out


def check_font(font, rows, name):
    for ch, g in font.items():
        assert len(g) == rows, f"{name} {ch!r}: {len(g)} rows"
        assert len({len(r) for r in g}) == 1, f"{name} {ch!r}: ragged rows"
        assert len(g[0]) <= 32, f"{name} {ch!r}: wider than 32"


def masks(rows):
    """Row bit masks, bit 0 = leftmost pixel."""
    return [sum(1 << i for i, c in enumerate(r) if c == "#") for r in rows]


def emit_font(name, font, rows, ctype):
    lines = [f"static const menu_glyph{rows}_t {name}[64] = {{"]
    for code in range(32, 96):
        ch = chr(code)
        g = font.get(ch, font["?"])
        m = ", ".join(f"0x{v:0{8 if ctype == 'uint32_t' else 4}x}" for v in masks(g))
        label = "space" if ch == " " else ch.replace("\\", "backslash")
        lines.append(f"    {{{len(g[0])}, {{{m}}}}},   /* {label} */")
    lines.append("};")
    return "\n".join(lines)


def generate():
    check_font(TEXT, TEXT_ROWS, "text")
    display = {ch: display_glyph(g) for ch, g in TEXT.items()}
    display[" "] = ["." * 8] * DISPLAY_ROWS
    for ch, g in DISPLAY_OVERRIDE.items():
        width = max(len(r) for r in g)
        display[ch] = [r.ljust(width, ".") for r in g]
    check_font(display, DISPLAY_ROWS, "display")
    return "\n".join([
        "/* C5VRX-5 by Twotoz and the C5VRX contributors: menu fonts.",
        " * Generated by tools/menu_font.py; edit the glyph art there. Row masks,",
        " * bit 0 = leftmost logical pixel; ASCII 32..95, lower case maps to upper. */",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define MENU_TEXT_ROWS {TEXT_ROWS}u",
        f"#define MENU_DISPLAY_ROWS {DISPLAY_ROWS}u",
        f"typedef struct {{ uint8_t width; uint16_t rows[{TEXT_ROWS}]; }} menu_glyph{TEXT_ROWS}_t;",
        f"typedef struct {{ uint8_t width; uint32_t rows[{DISPLAY_ROWS}]; }} menu_glyph{DISPLAY_ROWS}_t;",
        "",
        emit_font("s_menu_text_font", TEXT, TEXT_ROWS, "uint16_t"),
        "",
        emit_font("s_menu_display_font", display, DISPLAY_ROWS, "uint32_t"),
        "",
    ])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--show", metavar="TEXT", help="print TEXT in both fonts")
    a = ap.parse_args()
    if a.show:
        display = {ch: display_glyph(g) for ch, g in TEXT.items()}
        display.update(DISPLAY_OVERRIDE)
        for font, rows in ((TEXT, TEXT_ROWS), (display, DISPLAY_ROWS)):
            for y in range(rows):
                print(" ".join(font.get(c, font["?"])[y] for c in a.show.upper()).replace(".", " "))
            print()
        return 0
    text = generate()
    if a.check:
        if OUT.read_text(encoding="utf-8") != text:
            print("main/menu_font.h is stale: run python3 tools/menu_font.py", file=sys.stderr)
            return 1
        print("menu_font.h current")
        return 0
    OUT.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
