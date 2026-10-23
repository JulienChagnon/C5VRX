"""C5VRX by Twotoz/contributors: freeze and independently test low-C/N P64.

Uses seeds excluded from the optimizer, exact LUT16 programs and fixed gain.
No board, serial port or RF transmitter is accessed. Research artifacts only.
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

import p64_model
import p64_lowcn_search as search
import p5t_eval
import sim

ROOT = Path(__file__).resolve().parents[1]
MODEL_ID = 'P64-LowCN-E1'


def transfer_checks(table):
    sample = np.arange(8000)
    worst = 0.
    for radius in (2., 4., 6.):
        for phase in (.27, 1.1, 2.7, -2.1):
            for mhz in np.arange(-5.5, 6.51, .25):
                z = radius * np.exp(2j*np.pi*mhz*sample/40 + 1j*phase)
                raw = (((np.floor(z.real).astype(np.int64) & 15) << 4) |
                       (np.floor(z.imag).astype(np.int64) & 15)).astype(np.uint8)
                codes = sim._pair_codes(raw, table, sim.PAIR_BITS, 4)[1000:]
                worst = max(worst, abs(float(codes.mean())-(mhz+6)/15*63))
    transition_error = 0.
    freq = np.r_[np.full(2000, -3.3e6), np.full(4000, -1e6)]
    for radius in (2., 4., 6.):
        for phase in (.27, 1.1, 2.7, -2.1):
            z = radius * np.exp(2j*np.pi*np.cumsum(freq)/40e6 + 1j*phase)
            raw = (((np.floor(z.real).astype(np.int64) & 15) << 4) |
                   (np.floor(z.imag).astype(np.int64) & 15)).astype(np.uint8)
            porch = sim._pair_codes(raw, table, sim.PAIR_BITS, 4)[2040:-8]
            average = np.convolve(porch, np.ones(32)/32, mode='valid')
            transition_error = max(transition_error, float(np.max(abs(average-21))))
    return dict(tones=588, worst_mean_code_error=float(worst), tone_pass=bool(worst < .5),
                transitions=12, worst_porch_window_error=transition_error,
                transition_pass=transition_error < .5)


def export_program(table, out):
    import u85q_lp
    source = (ROOT/'main/u85q_64.bsasm').read_text()
    source = re.sub(r'^lut .+$', 'lut '+' '.join(str(int(v)) for v in table), source, flags=re.M)
    program = out/'p64_lowcn.bsasm'
    program.write_text('# '+MODEL_ID+'; research candidate, not hardware verified.\n'+source, encoding='utf-8')
    instructions, meta, lut = u85q_lp.assemble(program)
    assert len(instructions) == 8 and np.array_equal(lut, table)
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    assert len(binary) == 300+2048
    assert binary[:7] == bytes((1, 0, 3, 8, 0, 2, 1))
    (out/'p64_lowcn.bs').write_bytes(binary)
    sys.path.insert(0, str(ROOT/'test'))
    from test_p64 import reference
    raw = np.random.default_rng(991700).integers(0, 256, 12000, dtype=np.uint8)
    got = np.frombuffer(u85q_lp.emulate(program, raw.tobytes(), len(raw)-8), dtype=np.uint8)
    assert np.array_equal(got & 63, reference(raw, table)[:len(got)])
    assert np.array_equal(got[::2] & 63, got[1::2] & 63)
    return dict(instructions=8, loader_bytes=len(binary), independent_emulator_pass=True,
                output='DAC20 duplicated at TX40', lut_sha256=search.digest(table))


def main():
    global MODEL_ID
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=search.DEFAULT_OUT/'checkpoint.json')
    parser.add_argument('--out', type=Path, default=search.DEFAULT_OUT)
    parser.add_argument('--seed-base', type=int, default=9917)
    parser.add_argument('--model-id', default=MODEL_ID)
    args = parser.parse_args()
    MODEL_ID = args.model_id
    args.out.mkdir(parents=True, exist_ok=True)
    checkpoint = json.loads(args.checkpoint.read_text())
    params = checkpoint['best']['params']
    table = p64_model.build_lut64_from_params(params)
    assert search.digest(table) == checkpoint['best']['sha256']
    # Freeze the model before constructing any held-out case.
    search.save(args.out/'candidate.json', dict(model_id=MODEL_ID, params=params,
                lut_sha256=search.digest(table), source='U80P64-R1 / PR250; C5VRX by Twotoz and contributors',
                evidence='experimental host candidate; not installed in firmware'))
    functional = transfer_checks(table)
    loader = export_program(table, args.out)
    print('FUNCTIONAL', json.dumps(functional), json.dumps(loader), flush=True)
    models = [('U85Q', sim.u85q_lut(), 5),
              ('U80P64-R1', p64_model.repaired_table(), 4), (MODEL_ID, table, 4)]
    rows = []
    cn_points = (-10., -8., -6., -4., -3., -2., -1., 0., 2., 4., 8., 20.)
    for std in ('PAL', 'NTSC'):
        seed = args.seed_base if std == 'PAL' else args.seed_base + 1
        for dev in (.7, 1., 1.4):
            for cn in cn_points:
                case = search.make_case(seed, cn, std, dev)
                scores = {}
                for name, lut, bits in models:
                    noisy = sim._pair_codes(case['raw'], lut, sim.PAIR_BITS, bits)
                    clean = sim._pair_codes(case['clean'], lut, sim.PAIR_BITS, bits)
                    got = p5t_eval.score_codes(case, noisy, clean)
                    scores[name] = {key: got[key] for key in ('sinad', 'lock', 'spark', 'streak')}
                rows.append(dict(standard=std, seed=seed, deviation=dev, cn_db=cn, scores=scores))
            print('VALIDATED', std, dev, flush=True)
    # Additional fixed-context stress at the transition and good-signal levels.
    stress = []
    for gain, offset in ((2., 0.), (4., 0.), (3., -250e3), (3., 250e3)):
        for cn in (-2., 0., 10.):
            case = search.make_case(args.seed_base+2, cn, 'NTSC', 1., gain, offset)
            stress.append(dict(adc_gain=gain, offset_hz=offset, cn_db=cn,
                          scores={name: search.score(case, lut) for name,lut,_ in models[1:]}))
    noise_case = search.make_case(args.seed_base+3, -100., 'NTSC', 1.)
    strong = search.make_case(args.seed_base+3, 30., 'NTSC', 1.)
    # Real clean-carrier calibration, never an unstable near-zero clean input.
    noise_case['clean'] = strong['clean']
    noise_scores = {name: search.score(noise_case, lut) for name,lut,_ in models[1:]}

    low = [r for r in rows if r['cn_db'] < 0]
    high = [r for r in rows if r['cn_db'] >= 0]
    def delta(r, metric):
        return r['scores'][MODEL_ID][metric]-r['scores']['U80P64-R1'][metric]
    summary = dict(mean_low_sinad_gain_db=float(np.mean([delta(r,'sinad') for r in low])),
                   mean_low_lock_gain_pp=float(100*np.mean([delta(r,'lock') for r in low])),
                   worst_high_sinad_change_db=min(delta(r,'sinad') for r in high),
                   worst_high_lock_change_pp=100*min(delta(r,'lock') for r in high),
                   high_gate_pass=all(delta(r,'sinad') >= -.3 and delta(r,'lock') >= -.03 for r in high),
                   useful_low_gain=float(np.mean([delta(r,'sinad') for r in low])) > .1 or
                       float(np.mean([delta(r,'lock') for r in low])) > .03,
                   noise_only_lock={name: s['lock'] for name,s in noise_scores.items()})
    summary['accepted_host_candidate'] = bool(functional['tone_pass'] and functional['transition_pass'] and
                                              summary['high_gate_pass'] and summary['useful_low_gain'])
    report = dict(model_id=MODEL_ID, evidence='synthetic LUT/ADC/goggle simulation, not hardware or range evidence',
                  params=params, search_seconds=checkpoint['elapsed_seconds'], trials=checkpoint['trials'],
                  cn_reference_bandwidth_hz=20000000, rx_filter_hz=12000000, adc_gain=3.,
                  functional=functional, loader=loader, summary=summary, rows=rows, stress=stress,
                  noise_only=noise_scores, held_out_seeds=list(range(args.seed_base,args.seed_base+4)))
    search.save(args.out/'validation.json', report)
    with (args.out/'validation.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['standard','deviation','C/N (dB)','model','SINAD (dB)','Line sync (%)'])
        for row in rows:
            for name, s in row['scores'].items():
                writer.writerow([row['standard'],row['deviation'],row['cn_db'],name,s['sinad'],100*s['lock']])
    print('VALIDATION_COMPLETE', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
