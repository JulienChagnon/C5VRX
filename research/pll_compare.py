"""C5VRX by Twotoz and contributors: exploratory detector comparison.
Not an information-theoretic upper bound or a firmware implementation.
How much FM information is left in the C5's 4-bit IQ beyond U85Q?
Offline comparison on the same simulated raw IQ (research/sim.py, PAL/NTSC,
deviation 1.0, 12 MHz filter): U85Q (bit-exact) and U85Q-LP against a
plain discriminator and a second-order PLL (threshold extension) on the
same 4-bit samples, scored with the same goggle model and luma SINAD.
Not a firmware design: a float, per-sample loop the BitScrambler cannot
run; it tests candidate improvements without proving an attainable bound."""
import multiprocessing as mp
import sys
import argparse
import json
from pathlib import Path
from numba import njit

import numpy as np
from scipy import signal as sg

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import u85q_lp  # noqa: E402
import kp_morph

LUT = sim.u85q_lut()


def iq(raw):
    i = ((raw.astype(np.int64) >> 4) ^ 8) - 8
    q = ((raw.astype(np.int64) & 15) ^ 8) - 8
    return (i + .5) + 1j * (q + .5)


def score(c, v, vc):
    y, yc = sg.sosfilt(sim.GOGGLE, v), sg.sosfilt(sim.GOGGLE, vc)
    yi = sim.apply_cal(y, sim.calibrate(yc, c['truth'], c['region']))
    t, r = c['truth'][3000:-3000], c['region'][3000:-3000]
    m = r == 1
    e = yi[3000:-3000][m] - t[m]
    sinad = float(10 * np.log10(np.var(t[m]) / max(np.mean(e ** 2), 1e-12)))
    pic = min(sim.goggle(yi, c['truth'], c['standard'], 'ideal'), sim.goggle(yi, c['truth'], c['standard'], 'peak'))
    return pic, sinad


def disc(z):
    d = np.angle(z[1:] * np.conj(z[:-1]))
    return np.concatenate([[0.0], d])


@njit(cache=True)
def pll(z, bn):
    """Second-order PLL, noise bandwidth bn (fraction of the sample rate),
    damping ratio 0.707; output: the loop's frequency estimate."""
    zeta = 0.707
    wn = 2 * np.pi * bn / (zeta + 1 / (4 * zeta))
    kp, ki = 2 * zeta * wn, wn * wn
    th, fr = 0.0, 0.0
    out = np.empty(len(z))
    zz = z / np.maximum(np.abs(z), 1e-9)
    for n in range(len(z)):
        e = np.angle(zz[n] * complex(np.cos(-th), np.sin(-th)))
        fr += ki * e
        th += fr + kp * e
        out[n] = fr + kp * e
    return out


METHODS = ('U85Q', 'U85Q-LP', 'U60Q', 'U60Q-LP', 'U50Q', 'U50Q-LP', 'discriminator', 'PLL 0.04', 'PLL 0.04 LP20', 'PLL 0.07', 'PLL 0.10')


def one(args):
    cnr, seed, std, deviation, physical = args
    # Same raster/noise realization across C/N and exactly the same input
    # bytes for every detector. Physical mode models the folded ADC slice
    # with a single scale shared by clean/noisy IQ, as in sim.make_case.
    c = sim.make_case(seed, cnr, std, deviation, filter_hz=12e6, fields=1, physical_adc=physical, adc_gain=2.0 if physical else None)
    out = {}
    pll_cache = {}
    for name in METHODS:
        if name == 'U85Q':
            a, b = sim.u85q_codes(c['raw']), sim.u85q_codes(c['clean'])
            out[name] = score(c, sim.DAC_VOLTS[a], sim.DAC_VOLTS[b])
        elif name == 'U85Q-LP':
            a, b = u85q_lp.lp_codes(c['raw'], LUT), u85q_lp.lp_codes(c['clean'], LUT)
            out[name] = score(c, sim.DAC_VOLTS[a], sim.DAC_VOLTS[b])
        elif name.startswith(('U60Q', 'U50Q')):
            lut = kp_morph.build_lut(.60 if name.startswith('U60') else .50)
            decode = u85q_lp.lp_codes if name.endswith('-LP') else sim.u85q_codes
            a, b = decode(c['raw'], lut), decode(c['clean'], lut)
            out[name] = score(c, sim.DAC_VOLTS[a], sim.DAC_VOLTS[b])
        elif name == 'discriminator':
            out[name] = score(c, disc(iq(c['raw'])), disc(iq(c['clean'])))
        else:
            bn = float(name.split()[1])
            if bn not in pll_cache:
                pll_cache[bn] = (pll(iq(c['raw']), bn), pll(iq(c['clean']), bn))
            a, b = pll_cache[bn]
            if name.endswith('LP20'):
                # Same nominal post-filter pole/rate as U85Q-LP, but still
                # floating point: no DAC quantization or hardware claim.
                a, b = (np.repeat(sg.lfilter([.5], [1, -.5], v[::2]), 2)[:len(v)]
                        for v in (a, b))
            out[name] = score(c, a, b)
    return cnr, seed, out


CNRS = (-3, -2, -1, 0, 1, 2, 4)
SEEDS = ((11, 'PAL'), (12, 'NTSC'), (13, 'PAL'), (14, 'NTSC'))

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--physical', action='store_true')
    ap.add_argument('--deviation', type=float, default=1.0)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    jobs = [(c, s, st, a.deviation, a.physical) for c in CNRS for s, st in SEEDS]
    with mp.Pool(a.workers) as p:
        res = []
        for row in p.imap(one, jobs):
            res.append(row)
            print(f'{len(res)}/{len(jobs)}', flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(dict(deviation=a.deviation, physical=a.physical,
        adc_gain=2.0 if a.physical else None, filter_hz=12e6, seeds=SEEDS,
        scope='Simulation only; float PLL, no DAC or BitScrambler feasibility proof; clean-reference calibration; sync-line score is not complete picture quality',
        results=res), indent=2)+'\n')
    print('mean sync-line fraction / luma SINAD dB')
    print(f'{"":15s}' + ''.join(f'{c:>12d}' for c in CNRS))
    for n in METHODS:
        cells = []
        for c in CNRS:
            v = [r[2][n] for r in res if r[0] == c]
            pic, sn = np.mean([x[0] for x in v]), np.mean([x[1] for x in v])
            cells.append(f'{pic:4.2f}/{sn:5.1f}')
        print(f'{n:15s}' + ''.join(f'{x:>12s}' for x in cells))
