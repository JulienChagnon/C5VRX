"""C5VRX by Twotoz/contributors: PR #250 P64 table and assembled datapath.

Uses the ESP-IDF assembler/parser and existing BitScrambler emulator. Host
evidence only: physical throughput, picture quality and range remain untested.
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research"))
import p64_model
import u85q_lp


def reference(raw, table):
    """Independent two-lookups-per-pair reference; reset state/output zero."""
    output = np.zeros(len(raw), dtype=np.uint8)
    state = 0
    positions = (4, 5, 6, 7, 0, 1, 2, 3, 15, 11)
    for pair in range(len(raw) // 2 - 1):
        word = int(raw[2 * pair]) | (int(raw[2 * pair + 1]) << 8)
        address = sum(((word >> bit) & 1) << index for index, bit in enumerate(positions))
        token = int(table[address]) >> 12
        value = int(table[state * 16 + token])
        state = (value >> 6) & 63
        output[2 * pair + 2:2 * pair + 4] = value & 63
    return output


def main():
    table = p64_model.checked_in_table()
    assert len(table) == 1024
    assert np.array_equal(table, p64_model.champion_table())
    params = json.loads((ROOT / "research/data/p64_champion.json").read_text())["best_candidate"]
    n_sync = int(params["n_sync"])
    assert n_sync == 14 and 360 / (64 - n_sync) == 7.2
    reachable = {0}
    while True:
        expanded = reachable | {(int(table[s * 16 + t]) >> 6) & 63 for s in reachable for t in range(16)}
        if expanded == reachable:
            break
        reachable = expanded
    assert len(reachable) == 64
    for word in table:
        value = int(word)
        if ((value >> 6) & 63) < n_sync:
            assert (value & 63) <= params["max_sync_code"]
        else:
            assert (value & 63) >= params["min_video_code"]

    program = ROOT / "main/u85q_64.bsasm"
    instructions, meta, lut = u85q_lp.assemble(program)
    assert len(instructions) == 8 and np.array_equal(lut, table)
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    # Exact production program_words() acceptance, including full LUT size.
    assert binary[:7] == bytes((1, 0, 3, 8, 0, 2, 1))
    assert len(binary) == 300 + 2048
    assert struct.unpack_from("<1024H", binary, 300) == tuple(int(v) for v in table)
    rng = np.random.default_rng(250)
    raw = rng.integers(0, 256, 12000, dtype=np.uint8)
    got = np.frombuffer(u85q_lp.emulate(program, raw.tobytes(), len(raw) - 8), dtype=np.uint8)
    expected = reference(raw, table)[:len(got)]
    assert np.array_equal(got & 63, expected), "assembled P64 differs from pair/state reference"
    assert np.array_equal(got[::2] & 63, got[1::2] & 63), "DAC20 must duplicate at TX40"
    print("PASS: P64 champion/header/program match; 64 states; clamps; full loader image; assembled byte-exact DAC20/TX40")


if __name__ == "__main__":
    main()
