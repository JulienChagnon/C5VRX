"""C5VRX by Twotoz and contributors: bounded low-C/N P64 parameter search.

Extends p64_model.py / PR250 and the U80P64-R1 repair. Same Q4/I4 input,
two LUT16 lookups per 50 ns, 64 recurrent states, DAC20 duplicated at TX40.
This is host simulation research; it does not select or flash firmware.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from numba import njit

import p64_model
import p5t_eval
import sim

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / 'research/data/p64_lowcn_search'


def digest(table):
    return hashlib.sha256(table.astype('<u2').tobytes()).hexdigest()


@njit(cache=True)
def tone_gate(table):
    """Reject phase stalls, wrong DC transfer and gross cycle slips cheaply."""
    worst = 0.
    for radius in (2., 4., 6.):
        for phase in (.27, 2.7):
            for mhz in (-5.5, -3.3, -1., 0., 2., 4., 6.5):
                raw = np.empty(2400, np.uint8)
                for k in range(len(raw)):
                    angle = 2*np.pi*mhz*k/40 + phase
                    i = int(np.floor(radius*np.cos(angle)))
                    q = int(np.floor(radius*np.sin(angle)))
                    raw[k] = ((i & 15) << 4) | (q & 15)
                out = sim._pair_codes(raw, table, sim.PAIR_BITS, 4)
                err = abs(np.mean(out[600:]) - (mhz+6)/15*63)
                worst = max(worst, err)
                if err > .5:
                    return False, worst
    return True, worst


def make_case(seed, cn, std, dev, gain=3., offset=0.):
    case = sim.make_case(seed, cn, std, dev, filter_hz=12e6, fields=1,
                         physical_adc=True, adc_gain=gain, offset_hz=offset)
    case = {k: case[k] for k in ('raw', 'clean', 'truth', 'region', 'standard')}
    case['hs0'] = p5t_eval.hs0_of(case['truth'])
    return case


def score(case, table):
    noisy = sim._pair_codes(case['raw'], table, sim.PAIR_BITS, 4)
    clean = sim._pair_codes(case['clean'], table, sim.PAIR_BITS, 4)
    got = p5t_eval.score_codes(case, noisy, clean)
    return {k: got[k] for k in ('sinad', 'lock', 'spark', 'streak')}


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    temporary.replace(path)


def candidate(rng, initial, elite, trial):
    p = dict(initial if trial < 40 or rng.random() < .30 else elite)
    # Explore clean analytic tracking parameters, never a forced sync DAC clamp.
    bounds = {
        'kp_vid': (.64, .96), 'kp_sync': (.38, .92),
        'v_sync_prior': (-1.25, -.65), 'v_video_prior': (-.16, .30),
        'exit_sync_th': (-.70, -.18), 'enter_sync_th': (-1.45, -.65),
    }
    if trial < 12:
        p['kp_vid'] = (.66, .70, .74, .78, .82, .86)[trial % 6]
        p['kp_sync'] = .50 if trial < 6 else .75
        return p
    fresh = trial < 40 or rng.random() < .35
    for key, (lo, hi) in bounds.items():
        if fresh:
            p[key] = float(rng.uniform(lo, hi))
        elif rng.random() < .6:
            p[key] = float(np.clip(p[key] + rng.normal(0, (hi-lo)*.12), lo, hi))
    if fresh or rng.random() < .15:
        p['n_sync'] = int(rng.choice([8, 10, 12, 14, 16, 18, 20, 24]))
    if fresh:
        p['token_sectors'] = int(rng.choice([16, 16, 16, 12]))
    if p['token_sectors'] == 12:
        p['r_uncertainty'] = float(rng.choice([.8, 1.2, 1.6, 2.0]))
        p['w_uncertainty'] = float(rng.uniform(.15, .65))
    else:
        p['r_uncertainty'] = 0.
    p['phase_consistent_output'] = True
    return p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=900.)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    initial = json.loads((ROOT/'research/data/p64_repair.json').read_text())['params']
    baseline = p64_model.repaired_table()
    # Separate training and final validation seeds. No final seed is loaded here.
    low_specs = [(731, cn, 'PAL', 1.) for cn in (-10., -6., -3., -1., 0.)]
    low_specs += [(732, -2., 'NTSC', 1.)]
    high_specs = [(733, 20., std, dev) for std in ('PAL', 'NTSC') for dev in (.7, 1.4)]
    specs = low_specs + high_specs
    bank = [make_case(*spec) for spec in specs]
    base_scores = [score(c, baseline) for c in bank]
    tone_gate(baseline)
    meta = dict(source='C5VRX by Twotoz and contributors; extends PR250 / U80P64-R1',
                evidence='host LUT simulation only; not flashed or hardware verified',
                baseline_sha256=digest(baseline), seed=250731, specs=specs,
                cn_reference_bandwidth_hz=20000000, rx_filter_hz=12000000,
                adc_gain=3., optional_demod_lp=False, requested_seconds=args.seconds,
                high_gate='each 20 dB case: SINAD loss <= 0.3 dB; lock loss <= 0.02',
                low_gate='each training case: SINAD loss <= 0.25 dB; lock loss <= 0.05',
                fitness='mean low-C/N SINAD gain + 3 * mean lock-fraction gain')
    best = dict(params=initial, fitness=0., scores=base_scores, sha256=digest(baseline), trial=0)
    start = time.monotonic()
    trial = rejected_tone = rejected_high = 0
    rng = np.random.default_rng(250731)
    seen = {digest(baseline)}
    save(args.out/'checkpoint.json', dict(metadata=meta, baseline=base_scores, best=best))
    print('SEARCH_STARTED', json.dumps(meta), flush=True)
    with (args.out/'trials.jsonl').open('w', encoding='utf-8') as log:
        while time.monotonic()-start < args.seconds:
            trial += 1
            params = candidate(rng, initial, best['params'], trial)
            table = p64_model.build_lut64_from_params(params)
            sha = digest(table)
            if sha in seen:
                continue
            seen.add(sha)
            passed, error = tone_gate(table)
            if not passed:
                rejected_tone += 1
                log.write(json.dumps(dict(trial=trial, rejected='tone', max_error=error))+'\n')
                continue
            high = [score(c, table) for c in bank[len(low_specs):]]
            if any(s['sinad'] < b['sinad']-.3 or s['lock'] < b['lock']-.02
                   for s, b in zip(high, base_scores[len(low_specs):])):
                rejected_high += 1
                log.write(json.dumps(dict(trial=trial, rejected='high', scores=high))+'\n')
                continue
            low = [score(c, table) for c in bank[:len(low_specs)]]
            delta_s = [s['sinad']-b['sinad'] for s,b in zip(low,base_scores)]
            delta_l = [s['lock']-b['lock'] for s,b in zip(low,base_scores)]
            fitness = float(np.mean(delta_s) + 3*np.mean(delta_l))
            eligible = min(delta_s) >= -.25 and min(delta_l) >= -.05
            item = dict(trial=trial, params=params, sha256=sha, fitness=fitness,
                        eligible=eligible, scores=low+high,
                        mean_sinad_gain=float(np.mean(delta_s)), mean_lock_gain=float(np.mean(delta_l)))
            log.write(json.dumps(item)+'\n'); log.flush()
            if eligible and fitness > best['fitness']:
                best = item
                print('NEW_BEST', trial, round(fitness,4), json.dumps(params), flush=True)
                save(args.out/'checkpoint.json', dict(metadata=meta, baseline=base_scores, best=best,
                     trials=trial, elapsed_seconds=time.monotonic()-start))
            if trial % 10 == 0:
                print('PROGRESS', trial, 'best', round(best['fitness'],4),
                      'elapsed', round(time.monotonic()-start), flush=True)
        result = dict(metadata=meta, baseline=base_scores, best=best, trials=trial,
                      rejected_tone=rejected_tone, rejected_high=rejected_high,
                      elapsed_seconds=time.monotonic()-start)
        save(args.out/'checkpoint.json', result)
        print('SEARCH_COMPLETE', json.dumps({k:v for k,v in result.items() if k not in ('metadata','baseline','best')}),
              'best', best['fitness'], flush=True)


if __name__ == '__main__':
    main()
