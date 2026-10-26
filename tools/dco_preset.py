#!/usr/bin/env python3
# C5VRX-5 by Twotoz and the C5VRX contributors: build main/dco_preset.h, the
# filter and DC base map in the firmware, from measured boards.
#
# On each board, VTX off, run menu CALIBRATE / console F, then console C (or
# the web "Download calibration report") and save the log. Then:
#
#   python3 tools/dco_preset.py board1.log [board2.log ...] > main/dco_preset.h
#   python3 tools/dco_preset.py --check main/dco_preset.h LOG [LOG ...]
#
# Rows used: valid OK, not a template, of each preset filter class:
#   0 = NORMAL  (code 56, skirt 32, BW40)
#   1 = NARROW  (code 63, skirt 24, BW20)
#   2 = WIDE    (code 62, skirt  8, BW40)
#   3 = XNARROW (code 63, skirt  0, BW20)
# Several dumps of one board are first reduced to that board's median per
# gain; the preset is the median over boards, so repeated captures of one
# board count as one board.
import json
import re
import statistics
import sys

FILTERS = [
    (56, 32, 0),  # 0: NORMAL
    (63, 24, 1),  # 1: NARROW
    (62, 8, 0),   # 2: WIDE
    (63, 0, 1),   # 3: XNARROW
]
FILTER_NAMES = ["NORMAL", "NARROW", "WIDE", "XNARROW"]
DEFAULT_METRICS = [
    (13125, 13167),
    (11875, 11978),
    (16875, 16339),
    (10625, 11727),
]
DEFAULT_CAL_NBW = 12363
LINE = re.compile(r"CALDUMP_LINE n=\d+ (\{.*\})")


def dumps(path):
    """Yield (board_id, records) per CALDUMP_BEGIN..END block of a log."""
    recs, board = None, None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if "CALDUMP_BEGIN" in line:
                recs, board = [], None
            elif "CALDUMP_END" in line and recs is not None:
                yield board, recs
                recs = None
            elif recs is not None:
                m = LINE.search(line)
                if m:
                    r = json.loads(m.group(1))
                    if r["type"] == "device":
                        board = r["board_id"]
                    recs.append(r)


def median_int(values):
    return int(round(statistics.median(values)))


def build(paths):
    per_board = {c: {} for c in range(len(FILTERS))}      # class -> board -> gain -> ([i], [q])
    filt_metrics = {c: {} for c in range(len(FILTERS))}   # class -> board -> ([width], [nbw])
    context = set()
    for p in paths:
        for board, recs in dumps(p):
            active = next((r for r in recs if r["type"] == "active"), None)
            live = active and (active["filter_code"], active["filter_skirt"], int(active["filter_bw20"])) == FILTERS[0]
            dump_rows = {}
            for r in recs:
                if r["type"] == "filter":
                    c = r.get("class", -1)
                    if (0 <= c < len(FILTERS) and r.get("valid")
                            and (r["code"], r["skirt"], int(r["bw20"])) == FILTERS[c]
                            and r.get("width_khz", 0) > 0 and r.get("nbw_khz", 0) > 0):
                        m = filt_metrics[c].setdefault(board, ([], []))
                        m[0].append(r["width_khz"])
                        m[1].append(r["nbw_khz"])
                    continue
                if r["type"] != "dc_row" or r["valid"] != 1 or r["template"]:
                    continue
                header = (r.get("code"), r.get("skirt"), int(r.get("bw20", 0)))
                c = next((idx for idx, f in enumerate(FILTERS)
                          if header == f or (idx == 0 and header[0] == -1 and live)), -1)
                if c < 0:
                    continue
                context.add((r["iq_scale"], r["table_version"]))
                # Deduplicate within one CALDUMP block (slot -1 vs saved slot 0..5).
                if (c, r["gain"]) not in dump_rows or r["slot"] >= 0:
                    dump_rows[(c, r["gain"])] = (r["i"], r["q"])
            for (c, g), (i_val, q_val) in dump_rows.items():
                rows = per_board[c].setdefault(board, {}).setdefault(g, ([], []))
                rows[0].append(i_val)
                rows[1].append(q_val)
    all_boards = sorted({b for c in per_board.values() for b in c})
    if not all_boards:
        raise SystemExit("no valid own dc_row of the preset filters found (run F, then C)")
    if len(context) != 1:
        raise SystemExit(f"dumps mix IQ scale / table versions: {sorted(context)}")
    gains = sorted({g for c in per_board.values() for b in c.values() for g in b})
    lo, hi = gains[0], gains[-1]
    tables = []
    for c in range(len(FILTERS)):
        src = per_board[c]
        fallback = per_board[1] if FILTERS[c][2] and per_board[1] else per_board[0]
        table = []
        for g in range(lo, hi + 1):
            boards = [b[g] for b in src.values() if g in b]
            if not boards:
                boards = [b[g] for b in fallback.values() if g in b]
            if not boards:
                table.append((-1, -1, 0))
                continue
            table.append((median_int([median_int(b[0]) for b in boards]),
                          median_int([median_int(b[1]) for b in boards]), len(boards)))
        tables.append(table)
    metrics = []
    for c in range(len(FILTERS)):
        if filt_metrics[c]:
            w = median_int([median_int(v[0]) for v in filt_metrics[c].values()])
            n = median_int([median_int(v[1]) for v in filt_metrics[c].values()])
            metrics.append((w, n))
        else:
            metrics.append(DEFAULT_METRICS[c])
    return all_boards, context.pop(), lo, hi, metrics, tables


def header(paths):
    boards, (iq_scale, version), lo, hi, metrics, tables = build(paths)
    out = [
        "/* C5VRX-5 by Twotoz and the C5VRX contributors: filter and DC base map in the firmware.",
        f" * Generated by tools/dco_preset.py from {len(boards)} measured board(s)",
        f" * ({', '.join(boards)}); do not edit by hand. Median filter widths and I/Q codes",
        " * per gain of the 4 preset filter classes (NORMAL, NARROW, WIDE, XNARROW), VTX off.",
        " * Held for a row without own codes of this board; F/B replaces it. Not calibration evidence. */",
        "#pragma once",
        "#include <stdint.h>",
        f"#define DCO_PRESET_BOARDS   {len(boards)}u",
        f"#define DCO_PRESET_CLASSES  {len(FILTERS)}u",
        f"#define DCO_PRESET_LO       {lo}u",
        f"#define DCO_PRESET_HI       {hi}u",
        f"#define DCO_PRESET_IQ_SCALE {iq_scale}u",
        f"#define DCO_PRESET_VERSION  {version}u",
        f"#define DCO_PRESET_CAL_NBW_KHZ {DEFAULT_CAL_NBW}u",
        "typedef struct {",
        "    int8_t code;",
        "    uint8_t skirt, bw20;",
        "    uint16_t width_khz, nbw_khz;",
        "} dco_filter_preset_t;",
        "/* 0=NORMAL, 1=NARROW, 2=WIDE, 3=XNARROW. */",
        "static const dco_filter_preset_t c5vrx5_filter_preset[DCO_PRESET_CLASSES] = {",
    ]
    for c, (code, skirt, bw20) in enumerate(FILTERS):
        w, n = metrics[c]
        out.append(f"    {{{code}, {skirt}, {bw20}, {w}, {n}}}, /* {FILTER_NAMES[c]} */")
    out.extend([
        "};",
        "/* {I, Q} per class and gain DCO_PRESET_LO..HI; -1 = no board measured the row. */",
        "static const int16_t c5vrx5_dco_preset_class[DCO_PRESET_CLASSES][DCO_PRESET_HI - DCO_PRESET_LO + 1u][2] = {",
    ])
    for c, table in enumerate(tables):
        code, skirt, bw20 = FILTERS[c]
        out.append(f"    {{ /* {FILTER_NAMES[c]} (code {code}, skirt {skirt}, {'BW20' if bw20 else 'BW40'}) */")
        for k, (i, q, n) in enumerate(table):
            out.append(f"        {{{i}, {q}}},   /* G{lo + k}, {n} board(s) */")
        out.append("    },")
    out.extend([
        "};",
        "#define c5vrx5_dco_preset (c5vrx5_dco_preset_class[0])",
    ])
    return "\n".join(out) + "\n"


def main(argv):
    if len(argv) >= 3 and argv[1] == "--check":
        with open(argv[2], encoding="utf-8") as f:
            current = f.read()
        if current != header(argv[3:]):
            print(f"{argv[2]} is not generated from the given logs", file=sys.stderr)
            return 1
        print(f"PASS: {argv[2]} reproduces from {len(argv) - 3} log(s)")
        return 0
    if len(argv) < 2:
        print("usage: dco_preset.py LOG [LOG ...] > main/dco_preset.h", file=sys.stderr)
        return 2
    sys.stdout.write(header(argv[1:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
