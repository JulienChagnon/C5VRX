#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: reproducible DAC-LSB comparator invariance.

Run inside the pinned espressif/idf:v6.0.2 environment, e.g.:
  python research/native_flag_emulator.py --output /tmp/native_flag_emulator.json

Uses the real C output_entry()/kp_tables and actual SDK assembler. Loads the
existing u85q_lp.py generic emulator through AST without importing numpy/numba;
the only instrumentation records post-op counters, LUT address and mux output.
Temporary variants are checked against the archived LP flag program
and C comparator mapping before emulation.

Normal U85Q already writes L0 into both output bytes. The LP variant replaces
controller A10 at output0 with L0; worker O0 repeats that bit. Counter operands
use L1..L5 and A11..A15, so retaining the original remaining bits preserves the
LP recurrence. This flag compares raw decoder x, NOT filtered counter output y.
Scale the original reference word FIRST, then replace only bit0 using the
UNSCALED reference DAC comparison. Applying output scaling to a flagged word
would generally destroy the flag. Physical DAC LSB must separately be held LOW;
that hardware action is neither implemented nor verified by this experiment.
"""
from __future__ import annotations

import argparse
import ast
import functools
import hashlib
import json
import os
from pathlib import Path
import random
import shlex
import struct
import subprocess
import sys
import tempfile
from typing import Any

ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generic_emulator(idf: Path) -> dict[str, Any]:
    """Reuse the existing reference emulator with one observation-only hook."""
    source = ROOT / "research/u85q_lp.py"
    tree = ast.parse(source.read_text())
    wanted = {"_bsasm", "assemble", "emulate", "bsasm_op"}
    functions = [node for node in tree.body
                 if isinstance(node, ast.FunctionDef) and node.name in wanted]
    if {node.name for node in functions} != wanted:
        raise RuntimeError("existing generic emulator API changed")
    emulator = next(node for node in functions if node.name == "emulate")
    loop = next(node for node in emulator.body if isinstance(node, ast.While))
    insertion = [i for i, node in enumerate(loop.body)
                 if isinstance(node, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == "lut_out"
                         for target in node.targets)]
    if len(insertion) != 1:
        raise RuntimeError("generic emulator LUT update changed; review tracing")
    loop.body.insert(insertion[0], ast.parse(
        "trace.append((tuple(ctr), (out >> 16) & 1023, out))").body[0])
    module = ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[]))
    os.environ["IDF_PATH"] = str(idf)
    namespace: dict[str, Any] = {
        "os": os, "sys": sys, "Path": Path, "json": json,
        "lru_cache": functools.lru_cache, "trace": [],
    }
    exec(compile(module, str(source), "exec"), namespace)
    return namespace


def actual_luts(directory: Path, compiler: str) -> tuple[list[list[int]], list[list[list[int]]]]:
    """Compile the actual production C mapping, rather than duplicating it."""
    source = directory / "read_luts.c"
    executable = directory / ("read_luts.exe" if os.name == "nt" else "read_luts")
    source.write_text(r'''#include <stdio.h>
#include <stdint.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "kp_tables.h"
#include "output_level.h"
#include "sync_flag.h"
int main(void) {
#ifdef _WIN32
    if (_setmode(_fileno(stdout), _O_BINARY) == -1) return 5;
#endif
    const uint32_t dimensions[2] = {KP_LEVELS, OUTPUT_LEVELS};
    if (fwrite(dimensions, sizeof(dimensions), 1, stdout) != 1) return 1;
    if (fwrite(kp_tables, sizeof(kp_tables), 1, stdout) != 1) return 2;
    for (unsigned kp = 0; kp < KP_LEVELS; ++kp)
        for (unsigned level = 0; level < OUTPUT_LEVELS; ++level)
            for (unsigned i = 0; i < 1024; ++i) {
                uint16_t word = output_entry(kp_tables[kp][i], level, NULL);
                uint16_t expected = (uint16_t)((word & ~1u) |
                    ((kp_tables[kp][i] & 63u) < SYNC_FLAG_THRESHOLD));
                if (sync_flag_entry(kp_tables[kp][i], level, NULL) != expected) return 4;
                if (fwrite(&word, sizeof(word), 1, stdout) != 1) return 3;
            }
    return 0;
}
''')
    subprocess.run(shlex.split(compiler) + ["-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-I", str(ROOT / "main"), str(source), "-o", str(executable)], check=True)
    result = subprocess.run([str(executable)], capture_output=True, check=True).stdout
    if len(result) < 8:
        raise RuntimeError("C LUT exporter produced an incomplete header")
    levels, outputs = struct.unpack("=II", result[:8])
    if (levels, outputs) != (8, 10):
        raise RuntimeError(f"review changed profile dimensions: {levels} kp, {outputs} outputs")
    count = levels * 1024 + levels * outputs * 1024
    if len(result) != 8 + count * 2:
        raise RuntimeError("C LUT exporter size disagrees with profile dimensions")
    words = struct.unpack(f"={count}H", result[8:])
    reference = [list(words[k * 1024:(k + 1) * 1024]) for k in range(levels)]
    scaled = []
    for kp in range(levels):
        table = []
        for level in range(outputs):
            start = levels * 1024 + (kp * outputs + level) * 1024
            table.append(list(words[start:start + 1024]))
        scaled.append(table)
    return reference, scaled


def semantic(value: Any) -> Any:
    """Assembler source positions are provenance, not instruction semantics."""
    if isinstance(value, dict):
        return {key: semantic(item) for key, item in value.items() if key != "ele"}
    if isinstance(value, (list, tuple)):
        return [semantic(item) for item in value]
    return value


def replace_lut(text: str, words: list[int]) -> str:
    lines = text.splitlines()
    positions = [i for i, line in enumerate(lines) if line.startswith("lut ")]
    if len(positions) != 1:
        raise RuntimeError("expected one shared1024-word LUT in program")
    lines[positions[0]] = "lut " + " ".join(map(str, words))
    return "\n".join(lines) + "\n"


def prove(idf: Path, compiler: str, threshold: int, output_bytes: int, seed: int) -> dict[str, Any]:
    if not __debug__:
        raise RuntimeError("run without python -O: this proof requires enabled assertions")
    namespace = generic_emulator(idf)
    assemble = namespace["assemble"]
    emulate = namespace["emulate"]
    normal_path = ROOT / "main/u85q.bsasm"
    lp_path = ROOT / "research/legacy_demod/u85q_lp.bsasm"
    production_flag_path = ROOT / "research/legacy_demod/u85q_lp_flag.bsasm"
    normal = assemble(normal_path)
    lp = assemble(lp_path)
    if normal[1]["lut_width_bits"] != 16 or lp[1]["lut_width_bits"] != 16:
        raise RuntimeError("proof requires the current shared LUT16 programs")
    counters = {"cases": 0, "bundles": 0, "output_bytes": 0, "flag_samples": 0,
                "lut_words": 0, "changed_lut_words": 0, "grounded_lsb_code_loss_ones": 0}
    rows = []
    with tempfile.TemporaryDirectory(prefix="c5vrx-native-flag-") as temporary:
        directory = Path(temporary)
        reference, scaled = actual_luts(directory, compiler)
        marked0 = [(word & ~1) | int((original & 63) < threshold)
                   for word, original in zip(scaled[0][0], reference[0])]
        normal_variant = directory / "u85q_comparator.bsasm"
        lp_variant = directory / "u85q_lp_comparator.bsasm"
        normal_variant.write_text(replace_lut(normal_path.read_text(), marked0))
        lp_text = lp_path.read_text()
        if lp_text.count("set 0..5 A10..A15,") != 4:
            raise RuntimeError("LP controller mux changed; review comparator route")
        lp_variant.write_text(replace_lut(lp_text.replace("set 0..5 A10..A15,",
            "set 0 L0,\n    set 1..5 A11..A15,"), marked0))
        flag_normal, flag_lp = assemble(normal_variant), assemble(lp_variant)
        production_flag = assemble(production_flag_path)
        assert semantic(production_flag[0]) == semantic(flag_lp[0]), "firmware LP flag instructions differ from proof"
        assert semantic(production_flag[1]) == semantic(flag_lp[1]), "firmware LP flag configuration differs from proof"
        assert semantic(normal[0]) == semantic(flag_normal[0])
        for before, after in zip(lp[0], flag_lp[0]):
            for name in ("read", "write", "op"):
                assert semantic(before.get(name)) == semantic(after.get(name))
            differences = {bit for bit in before["mux"]
                           if semantic(before["mux"][bit]) != semantic(after["mux"][bit])}
            assert differences <= {0}
        generator = random.Random(seed)
        raw = bytes(generator.randrange(256) for _ in range(2 * output_bytes + 64))
        raw_hash = hashlib.sha256(raw).hexdigest()
        for kp in range(8):
            for level in range(10):
                baseline_lut = scaled[kp][level]
                marked_lut = [(word & ~1) | int((original & 63) < threshold)
                              for word, original in zip(baseline_lut, reference[kp])]
                assert all(((old ^ new) & ~1) == 0 for old, new in zip(baseline_lut, marked_lut))
                counters["lut_words"] += 1024
                counters["changed_lut_words"] += sum(a != b for a, b in zip(baseline_lut, marked_lut))
                for baseline, variant, name in ((normal, flag_normal, "normal"), (lp, flag_lp, "lp")):
                    # Firmware loads per-profile LUT RAM separately from its
                    # instruction program. Emulate those exact C-exported words
                    # with the SDK-parsed instruction/meta records, without
                    # generating or modifying any production program file.
                    programs = {"baseline": (baseline[0], baseline[1], baseline_lut),
                                "comparator": (variant[0], variant[1], marked_lut)}
                    namespace["assemble"] = lambda path: programs[path]
                    namespace["trace"] = []
                    before = emulate("baseline", raw, output_bytes)
                    old_trace = namespace["trace"]
                    namespace["trace"] = []
                    after = emulate("comparator", raw, output_bytes)
                    new_trace = namespace["trace"]
                    assert len(old_trace) == len(new_trace)
                    flags = 0
                    for bundle, (old, new) in enumerate(zip(old_trace, new_trace)):
                        assert old[:2] == new[:2], (kp, level, name, bundle, "counter_or_lookup")
                        assert ((old[2] ^ new[2]) & ~(257 if name == "normal" else 1)) == 0
                        if bundle % 2 == 0 and bundle:
                            previous_worker_address = old_trace[bundle - 1][1]
                            expected = int((reference[kp][previous_worker_address] & 63) < threshold)
                            assert (new[2] & 1) == expected
                            if name == "normal":
                                assert ((new[2] >> 8) & 1) == expected
                            else:
                                assert (new_trace[bundle + 1][2] & 1) == expected
                            flags += 2
                    assert all((a & 254) == (b & 254) for a, b in zip(before, after))
                    physical = [byte & 62 for byte in after]
                    assert all(0 <= (old & 63) - code <= 1 for old, code in zip(before, physical))
                    lost = sum((old & 63) - code for old, code in zip(before, physical))
                    row = {"kp_index": kp, "output_index": level, "program": name,
                           "bundles": len(old_trace), "output_bytes": len(before), "flag_samples": flags}
                    rows.append(row)
                    counters["cases"] += 1
                    counters["bundles"] += row["bundles"]
                    counters["output_bytes"] += len(before)
                    counters["flag_samples"] += flags
                    counters["grounded_lsb_code_loss_ones"] += lost
    paths = {"normal_program": normal_path, "lp_program": lp_path,
             "production_flag_program": production_flag_path,
             "production_flag_mapping": ROOT / "main/sync_flag.h",
             "kp_tables": ROOT / "main/kp_tables.h", "output_mapping": ROOT / "main/output_level.h",
             "generic_emulator": ROOT / "research/u85q_lp.py", "runner": Path(__file__),
             "sdk_assembler": idf / "tools/bsasm.py",
             "sdk_target": idf / "components/esp_driver_bitscrambler/bsasm_targets/esp32c5.json"}
    return {"schema": 2, "attribution": "C5VRX by Twotoz and contributors",
            "result": "PASS", "threshold_unscaled_dac": threshold, "seed": seed,
            "input_sha256": raw_hash, "output_bytes_per_case": output_bytes,
            "counts": counters, "profile_coverage": {"kp_settings": 8, "output_levels": 10,
            "programs": ["normal", "lp"]}, "sha256": {name: digest(path) for name, path in paths.items()},
            "assertions": ["SDK instructions/opcodes/read/write unchanged except LP controller output0",
                "actual firmware LP flag instructions/configuration equal the emulated variant",
                "actual C sync_flag_entry equals scaled output with canonical comparator inserted last",
                "every LUT address and A/B counter equals baseline after every executed bundle",
                "every non-LSB output bit equals baseline",
                "duplicated flag equals unscaled preceding worker DAC comparison",
                "grounded physical DAC equals original DAC with only bit0 cleared"],
            "limitations": {"firmware_enabled": False, "physical_gpio_grounding_verified": False,
                "monitor_lock_proven": False, "camera_timing_proven": False,
                "input": "deterministic byte stream, not exhaustive IQ sequences",
                "flag": "raw decoder x; bypasses LP counter y smoothing",
                "scale_order": "apply actual C output_entry first, replace bit0 from unscaled reference last",
                "polarity": "HIGH for unscaled DAC<threshold; LOW-sync parser needs explicit adaptation",
                "negative_study": "raw flag chatter may erase transitions in an inertial filter; this invariance proof does not resolve it"},
            "case_digest_sha256": hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest(),
            "per_program": {name: {"cases": sum(row["program"] == name for row in rows),
                "bundles_per_case": sorted({row["bundles"] for row in rows if row["program"] == name}),
                "flag_samples_per_case": sorted({row["flag_samples"] for row in rows if row["program"] == name})}
                for name in ("normal", "lp")}}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--idf-path", type=Path, default=os.environ.get("IDF_PATH"))
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    parser.add_argument("--threshold", type=int, default=19)
    parser.add_argument("--bytes", dest="output_bytes", type=int, default=2048)
    parser.add_argument("--seed", type=int, default=213)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.idf_path:
        parser.error("set IDF_PATH to the pinned ESP-IDF checkout or use --idf-path")
    if not 0 <= args.threshold <= 64 or args.output_bytes < 64 or args.output_bytes % 2:
        parser.error("threshold must be0..64; --bytes must be even and at least64")
    result = prove(Path(args.idf_path), args.cc, args.threshold, args.output_bytes, args.seed)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    count = result["counts"]
    print(f"PASS: {count['cases']} cases, {count['bundles']} bundles, "
          f"{count['output_bytes']} output bytes, {count['flag_samples']} comparator flags")
    print("No firmware, hardware comparator, camera qualification or monitor-lock claim.")


if __name__ == "__main__":
    main()
