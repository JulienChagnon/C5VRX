#!/usr/bin/env python3
"""C5VRX-5 by Twotoz and the C5VRX contributors: menu preview.

Shows menu frames as a goggle would: the UI rows written by
test/test_menu_ui.c (one PGM per frame, one DAC sample per column, one
UI row per line) are placed in a PAL picture at their real position
(prefix 652 samples, field line 28, five lines per row), low-passed to a
typical receiver luma bandwidth and scaled to a 4:3 (or 16:9 stretched)
screen. Needs numpy and Pillow.

  cc ... test/test_menu_ui.c main/menu_ui.c -o t && ./t frames/
  python3 tools/menu_preview.py frames/ previews/ [--wide] [--sheet]
"""
import argparse
import pathlib

import numpy as np
from PIL import Image

RATE = 40e6
ACTIVE_START = 414          # PAL active video 10.35 us after 0H
ACTIVE_SAMPLES = 2080       # 52 us
PREFIX = 652                # menu_raster.h MENU_PAL_PREFIX_BYTES
FIRST_LINE = 28 - 25        # field lines from active start to the UI
ROW_LINES = 5               # MENU_PAL_UI_Y_REPEAT
FIELD_LINES = 287           # active lines per field
BLACK, WHITE = 20, 60


def read_pgm(path):
    data = path.read_bytes()
    parts = data.split(maxsplit=4)
    assert parts[0] == b"P5"
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(parts[4][: w * h], dtype=np.uint8).reshape(h, w)


def lowpass(rows, mhz):
    sigma = 0.1325 / (mhz * 1e6) * RATE     # Gaussian, -3 dB at `mhz`
    r = int(np.ceil(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.pad(rows, ((0, 0), (r, r)), mode="edge")
    return np.stack([np.convolve(row, k, mode="valid") for row in pad])


def frame(ui, mhz):
    field = np.full((FIELD_LINES, ACTIVE_SAMPLES), BLACK, dtype=float)
    ui = lowpass(ui.astype(float), mhz)
    x0 = PREFIX - ACTIVE_START
    for row in range(ui.shape[0]):
        y = FIRST_LINE + row * ROW_LINES
        field[y:y + ROW_LINES, x0:x0 + ui.shape[1]] = ui[row]
    lines = np.repeat(field, 2, axis=0)                 # interlaced frame
    return np.clip((lines - BLACK) / (WHITE - BLACK), 0, 1)


def to_image(img, wide, height):
    width = height * (16 if wide else 4) // (9 if wide else 3)
    pic = Image.fromarray((img * 255).astype(np.uint8), "L")
    return pic.resize((width, height), Image.LANCZOS)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("frames", type=pathlib.Path)
    ap.add_argument("out", type=pathlib.Path)
    ap.add_argument("--wide", action="store_true", help="16:9 stretched goggle")
    ap.add_argument("--mhz", type=float, default=4.2, help="luma bandwidth")
    ap.add_argument("--height", type=int, default=576)
    ap.add_argument("--sheet", action="store_true", help="also write sheet.png with every frame")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    shots = []
    for path in sorted(a.frames.glob("*.pgm")):
        pic = to_image(frame(read_pgm(path), a.mhz), a.wide, a.height)
        pic.save(a.out / (path.stem + ".png"))
        shots.append(pic)
    if a.sheet and shots:
        cols = 3
        w, h = shots[0].size
        sheet = Image.new("L", (cols * w + (cols + 1) * 8, ((len(shots) + cols - 1) // cols) * (h + 8) + 8), 90)
        for i, s in enumerate(shots):
            sheet.paste(s, (8 + (i % cols) * (w + 8), 8 + (i // cols) * (h + 8)))
        sheet.save(a.out / "sheet.png")
    print(f"{len(shots)} previews in {a.out}")


if __name__ == "__main__":
    main()
