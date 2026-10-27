"""C5VRX by Twotoz and the C5VRX contributors: picture quality of U80P64-R1
against carrier-to-noise and the total IQ radius, for the LEVEL AGC.

The bit-exact demod table (p64_model.repaired_table) runs on the physical-ADC
model of sim.py (ultrafine lane with its folding, 12 MHz filter, one fixed
ADC gain per case): adc_gain is the total (carrier plus noise) rms radius in
cells. Every case also reports its C/N after the filter (carrier against
noise at the ADC), the scale the LEVEL AGC works on (cells^2 at a row).
Host simulation only; level_quality.py turns the output into the firmware
table main/level_quality.h.

    python research/level_quality_grid.py research/data/level_quality_grid.json
"""
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import p5t_eval  # noqa: E402
import p64_model  # noqa: E402

CNS = [-4, -2, 0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 30]     # C/N per 20 MHz (sim.make_case)
RMS = [0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0, 10.0, 13.0]
CASES = [(3101, 'PAL'), (3102, 'NTSC'), (3103, 'PAL'), (3104, 'NTSC')]
_table = None


def one(job):
    global _table
    if _table is None:
        _table = p64_model.repaired_table()
    cn, rms, seed, std = job
    c = sim.make_case(seed, cn, std, 1.0, filter_hz=12e6, fields=1, physical_adc=True, adc_gain=rms)
    c['hs0'] = p5t_eval.hs0_of(c['truth'])
    noisy = sim._pair_codes(c['raw'], _table, sim.PAIR_BITS, 4)
    clean = sim._pair_codes(c['clean'], _table, sim.PAIR_BITS, 4)
    s = p5t_eval.score_codes(c, noisy, clean)
    a, ca = c['adc'], c['clean_adc']
    post = 10 * np.log10(np.mean(abs(ca) ** 2) / np.mean(abs(a - ca) ** 2))
    return dict(cn=cn, rms=rms, seed=seed, std=std, lock=float(s['lock']), sinad=float(s['sinad']),
                cn_post=float(post), rms_meas=float(np.sqrt(np.mean(abs(a / 16.) ** 2))))


def main():
    jobs = [(cn, r, s, st) for cn in CNS for r in RMS for s, st in CASES]
    t0 = time.time()
    out = []
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=4)):
            out.append(r)
            if i % 100 == 0:
                print(i, len(jobs), round(time.time() - t0), flush=True)
    out.sort(key=lambda r: (r['cn'], r['rms'], r['seed']))
    Path(sys.argv[1]).write_text(json.dumps(out, indent=0))
    print('done', len(out), round(time.time() - t0))


if __name__ == '__main__':
    main()
