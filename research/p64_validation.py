"""C5VRX/Twotoz: U80P64-R1/PR250/U85Q validation; host evidence only.

Uses the PR250 fast scorer (research/p5t_eval.py) and existing physical-ADC
model. No attached hardware or VTX is used. Run with --check in CI.
"""
import argparse
import json
from pathlib import Path

import sim
import p5t_eval
import p64_model

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "research/data/p64_repair_validation.json"


def run():
    models = (("U85Q", sim.u85q_lut(), 5),
              ("old-P64", p64_model.champion_table(), 4),
              ("repair-0.8", p64_model.repaired_table(), 4))
    results = []
    for standard in ("PAL", "NTSC"):
        for deviation in (0.7, 1.4):
            for cn in (30.0, 4.0, 0.0):
                case = sim.make_case(2903, cn, standard, deviation, filter_hz=12e6,
                                     fields=1, physical_adc=True, adc_gain=3.0)
                case["hs0"] = p5t_eval.hs0_of(case["truth"])
                scores = {}
                for name, table, token_bits in models:
                    noisy = sim._pair_codes(case["raw"], table, sim.PAIR_BITS, token_bits)
                    clean = sim._pair_codes(case["clean"], table, sim.PAIR_BITS, token_bits)
                    score = p5t_eval.score_codes(case, noisy, clean)
                    scores[name] = {key: score[key] for key in ("lock", "sinad")}
                results.append(dict(standard=standard, dev=deviation, cn=cn, scores=scores))
                print(standard, deviation, cn,
                      [(name, round(score["lock"], 3), round(score["sinad"], 2))
                       for name, score in scores.items()], flush=True)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    results = run()
    if args.check:
        expected = json.loads(REPORT.read_text())["results"]
        assert len(expected) == len(results) == 12
        for actual, saved in zip(results, expected):
            assert (actual["standard"], actual["dev"], actual["cn"]) == (saved["standard"], saved["dev"], saved["cn"])
            for name in actual["scores"]:
                assert abs(actual["scores"][name]["lock"] - saved["scores"][name]["lock"]) < 0.02
                assert abs(actual["scores"][name]["sinad"] - saved["scores"][name]["sinad"]) < 0.05
            base, repair = actual["scores"]["U85Q"], actual["scores"]["repair-0.8"]
            if actual["cn"] == 30:
                assert repair["lock"] == 1.0 and repair["sinad"] >= base["sinad"] - 0.5
            elif actual["cn"] == 4:
                assert repair["lock"] >= base["lock"] - 0.03
            elif actual["dev"] == 0.7:
                assert repair["lock"] >= base["lock"] - 0.05
        print("PASS: 12 PAL/NTSC validation cases reproduce; clean-picture and lock gates pass (host only)")


if __name__ == "__main__":
    main()
