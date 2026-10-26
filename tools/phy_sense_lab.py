#!/usr/bin/env python3
"""Record and score the VTX-on PHY sensitivity lab (console S).

C5VRX by Twotoz and the C5VRX contributors. Extends the PR #254 VTX-off
noise-lab tooling (tools/phy_noise_lab.py at 6274b2d). See
docs/PHY_SENSITIVITY_LAB.md for the test plan and the evidence limits.

  record PORT OUTDIR [--suite a]   send S + suite, save console.log, score it
  analyse LOG [LOG ...] [--json F] score one or more runs; several runs of the
                                   same candidate are combined at the end

Each candidate ran STOCK-TEST-TEST-STOCK with 16 captures per stage. Per
metric the score is mean(TEST) - mean(STOCK) with its standard error; the
shams of the same run (STOCK twice) give the comparison's own spread. A
candidate moves a metric only beyond max(3 SE, 2 x sham RMS, floor).
BETTER/WORSE are per-run screening verdicts at one input level, not
sensitivity or range claims.
"""
import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

METRICS = {
    # name: (direction: +1 higher is better, floor)
    'cn_db': (+1, 0.10),
    'video_snr_db': (+1, 0.20),
    'click_pm': (-1, 0.05),
    # Share of captures in which the AFC decoder found syncs (1 or 0 per
    # capture). video_snr_db alone would drop a TEST that loses sync
    # entirely and score the candidate on the other metrics.
    'video_cov': (+1, 0.10),
}
EXTRA = ('origin_pm', 'rail_pm')
CAPTURES_PER_STAGE = 16
KEY = re.compile(r'(\w+)=(\S+)')


def records(lines):
    """SENSLAB records; another task's text can prefix a line (STATUS beat)."""
    for raw in lines:
        at = raw.find('SENSLAB')
        if at >= 0:
            yield raw[at:].strip()


def num(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def parse(lines):
    run = {'begin': {}, 'end': {}, 'profiles': {}, 'skips': [], 'warnings': [], 'refused': None}
    current = None
    for rec in records(lines):
        tag, _, rest = rec.partition(' ')
        fields = dict(KEY.findall(rest))
        if tag == 'SENSLAB_BEGIN':
            run['begin'] = fields
        elif tag == 'SENSLAB_END':
            run['end'] = fields
        elif tag == 'SENSLAB' and rest.startswith('refused='):
            run['refused'] = rest[len('refused='):]
        elif tag == 'SENSLAB_WARN':
            run['warnings'].append(rest)
        elif tag == 'SENSLAB_SKIP':
            run['skips'].append(fields)
        elif tag == 'SENSLAB_PROFILE':
            pid = int(fields['id'])
            current = run['profiles'].setdefault(pid, {
                'id': pid, 'name': fields.get('name'), 'kind': fields.get('kind'),
                'noop': fields.get('noop') == '1', 'caps': {0: [], 1: [], 2: [], 3: []},
                'applied': None, 'restored': None, 'status': None, 'aborts': []})
        elif tag == 'SENSLAB_CAP':
            pid, step = int(fields['id']), int(fields['step'])
            prof = run['profiles'].get(pid)
            if prof is None or step not in prof['caps']:
                continue
            values = {k: num(fields.get(k)) for k in (*METRICS, *EXTRA) if k != 'video_cov'}
            values['video_cov'] = 0.0 if values['video_snr_db'] is None else 1.0
            prof['caps'][step].append(values)
        elif tag == 'SENSLAB_APPLY' and current is not None:
            current['applied'] = fields.get('verified') == '1'
        elif tag == 'SENSLAB_RESTORE':
            prof = run['profiles'].get(int(fields['id']))
            if prof is not None:
                prof['restored'] = fields.get('verified') == '1'
        elif tag == 'SENSLAB_ABORT':
            prof = run['profiles'].get(int(fields['id']))
            if prof is not None:
                prof['aborts'].append(fields.get('reason'))
        elif tag == 'SENSLAB_RESULT':
            prof = run['profiles'].get(int(fields['id']))
            if prof is not None:
                prof['status'] = fields.get('status')
    return run


def mean_se(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None, 0
    m = sum(vals) / len(vals)
    if len(vals) < 2:
        return m, None, len(vals)
    var = sum((v - m) ** 2 for v in vals) / (len(vals) - 1)
    return m, math.sqrt(var / len(vals)), len(vals)


def deltas(prof):
    out = {}
    for metric in (*METRICS, *EXTRA):
        stock = [c[metric] for s in (0, 3) for c in prof['caps'][s]]
        test = [c[metric] for s in (1, 2) for c in prof['caps'][s]]
        ms, ses, ns = mean_se(stock)
        mt, set_, nt = mean_se(test)
        if ms is None or mt is None or ses is None or set_ is None:
            out[metric] = None
            continue
        s0 = mean_se([c[metric] for c in prof['caps'][0]])[0]
        s3 = mean_se([c[metric] for c in prof['caps'][3]])[0]
        t1 = mean_se([c[metric] for c in prof['caps'][1]])[0]
        t2 = mean_se([c[metric] for c in prof['caps'][2]])[0]
        out[metric] = {
            'stock': ms, 'test': mt, 'delta': mt - ms, 'se': math.sqrt(ses ** 2 + set_ ** 2),
            'n_stock': ns, 'n_test': nt,
            'drift': None if s0 is None or s3 is None else s3 - s0,
            'agree': None if t1 is None or t2 is None else t2 - t1,
        }
    return out


def complete(prof):
    return (prof['status'] == 'PASS' and prof['restored'] is True and
            all(len(prof['caps'][s]) == CAPTURES_PER_STAGE for s in range(4)))


def sham_rms(profiles):
    """Spread of the comparison itself: RMS delta of the complete shams. A no-op
    candidate (TEST equal to STOCK) is reported, not pooled: one that still
    moved would hide a real instability by widening every threshold."""
    out = {}
    for metric in METRICS:
        vals = [p['d'][metric]['delta'] for p in profiles
                if p['kind'] == 'sham' and p['complete'] and p['d'].get(metric)]
        out[metric] = {'rms': math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else None, 'n': len(vals)}
    return out


def verdict(prof, spread):
    if prof['status'] in ('REJECTED_ENTRY', 'BREAKS_IQ'):
        return prof['status']
    if not prof['complete']:
        return 'INCOMPLETE'
    if prof['kind'] == 'sham':
        return 'SHAM'
    if prof['noop']:
        return 'NOOP'
    better, worse, unstable = [], [], False
    for metric, (direction, floor) in METRICS.items():
        d = prof['d'].get(metric)
        if not d:
            continue
        rms = spread[metric]['rms'] or 0.0
        threshold = max(3.0 * d['se'], 2.0 * rms, floor)
        d['threshold'] = threshold
        for side in ('drift', 'agree'):
            if metric == 'cn_db' and d[side] is not None and abs(d[side]) > max(threshold, 0.3):
                unstable = True
        if d['delta'] * direction > threshold:
            better.append(metric)
        elif d['delta'] * direction < -threshold:
            worse.append(metric)
    prof['better'], prof['worse'] = better, worse
    if unstable:
        return 'UNSTABLE'
    if better and worse:
        return 'MIXED'
    if better:
        return 'BETTER'
    if worse:
        return 'WORSE'
    return 'NO_EFFECT'


def score(run):
    profiles = []
    for pid in sorted(run['profiles']):
        prof = run['profiles'][pid]
        prof['complete'] = complete(prof)
        prof['d'] = deltas(prof)
        profiles.append(prof)
    spread = sham_rms(profiles)
    for prof in profiles:
        prof['verdict'] = verdict(prof, spread)
    return profiles, spread


def fmt(d, metric, width=7, digits=2):
    if not d.get(metric):
        return ' ' * (width - 2) + 'na'
    return f"{d[metric]['delta']:+{width}.{digits}f}"


def report(path, run, profiles, spread, out):
    b = run['begin']
    out.write(f"== {path}\n")
    if run['refused']:
        out.write(f"   REFUSED {run['refused']}\n")
        return
    out.write(f"   freq={b.get('freq')} MHz gain=G{b.get('gain')} baseline C/N={b.get('baseline_cn_db')} dB "
              f"video SNR={b.get('baseline_video_snr_db')} dB clicks={b.get('baseline_click_pm')} pm "
              f"end={run['end'].get('status', 'MISSING')}\n")
    for w in run['warnings']:
        out.write(f"   WARN {w}\n")
    for metric in METRICS:
        s = spread[metric]
        rms = 'na' if s['rms'] is None else f"{s['rms']:.3f}"
        out.write(f"   sham spread {metric}: rms={rms} from {s['n']} shams\n")
    out.write("   id   name                      verdict        d_cn_db  d_video  d_click  d_vcov  drift_cn\n")
    for p in profiles:
        drift = p['d'].get('cn_db') or {}
        dr = drift.get('drift')
        out.write(f"   {p['id']:<4} {p['name'][:25]:<25} {p['verdict']:<14} {fmt(p['d'], 'cn_db')}  "
                  f"{fmt(p['d'], 'video_snr_db')}  {fmt(p['d'], 'click_pm', 7, 3)}  {fmt(p['d'], 'video_cov', 6, 2)}  "
                  f"{'na' if dr is None else f'{dr:+.2f}'}\n")
    for s in run['skips']:
        out.write(f"   skip {s.get('name')}: {s.get('reason')}\n")


def combine(runs):
    """Across runs: a candidate counts only if its verdict repeats."""
    names = {}
    for _, run, profiles in runs:
        freq = run['begin'].get('freq')
        for p in profiles:
            if p['kind'] in ('sham',):
                continue
            names.setdefault(p['name'], []).append((freq, p['verdict'], (p['d'].get('cn_db') or {}).get('delta')))
    summary = {}
    for name, rows in sorted(names.items()):
        verdicts = [v for _, v, _ in rows]
        freqs = sorted({f for f, v, _ in rows if v in ('BETTER', 'WORSE')})
        if verdicts.count('BETTER') >= 2 and not any(v in ('WORSE', 'MIXED') for v in verdicts):
            overall = 'REPRODUCED_BETTER'
        elif verdicts.count('WORSE') >= 2 and not any(v in ('BETTER', 'MIXED') for v in verdicts):
            overall = 'REPRODUCED_WORSE'
        elif len(rows) < 2:
            overall = 'SINGLE_RUN'
        else:
            overall = 'NOT_REPRODUCED'
        summary[name] = {'overall': overall, 'runs': [{'freq': f, 'verdict': v, 'd_cn_db': d} for f, v, d in rows],
                         'freqs_with_effect': freqs}
    return summary


def analyse(paths, json_path=None, out=sys.stdout):
    runs = []
    for path in paths:
        text = Path(path).read_text(errors='replace').splitlines()
        run = parse(text)
        profiles, spread = score(run)
        report(path, run, profiles, spread, out)
        runs.append((str(path), run, profiles))
    summary = combine(runs)
    if len(runs) > 1:
        out.write("== combined over runs (BETTER/WORSE must repeat; one run is a screening result)\n")
        for name, s in summary.items():
            if s['overall'] != 'NOT_REPRODUCED' or any(r['verdict'] in ('BETTER', 'WORSE') for r in s['runs']):
                out.write(f"   {name:<25} {s['overall']:<18} " +
                          ' '.join(f"{r['freq']}:{r['verdict']}" for r in s['runs']) + "\n")
    result = {
        'runs': [{'log': p, 'begin': r['begin'], 'end': r['end'], 'refused': r['refused'], 'warnings': r['warnings'],
                  'skips': r['skips'],
                  'profiles': [{k: v for k, v in pr.items() if k != 'caps'} for pr in prof]}
                 for p, r, prof in runs],
        'combined': summary,
        'note': 'Screening at one input level per run; not an input-referred sensitivity or range result.',
    }
    if json_path:
        Path(json_path).write_text(json.dumps(result, indent=1, default=str))
    return result


def record(port, outdir, suite, timeout_s):
    import serial
    outdir = Path(outdir)
    if outdir.exists():
        raise SystemExit('use a fresh output directory for every run')
    outdir.mkdir(parents=True)
    log = outdir / 'console.log'
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.1, 1
    s.dtr = s.rts = False
    s.open()
    try:
        s.reset_input_buffer()
        s.write(b'S')
        time.sleep(0.4)
        s.write(suite.encode())
        end = time.monotonic() + timeout_s
        buffer = b''
        with log.open('wb') as f:
            while time.monotonic() < end:
                chunk = s.read(4096)
                if not chunk:
                    continue
                f.write(chunk)
                f.flush()
                buffer = (buffer + chunk)[-4096:]
                if b'SENSLAB_END' in buffer or b'SENSLAB refused=' in buffer or b'SENSLAB_FATAL' in buffer:
                    time.sleep(0.5)
                    f.write(s.read(65536))
                    break
            else:
                print('timeout: no SENSLAB_END', file=sys.stderr)
    finally:
        s.close()
    analyse([log], outdir / 'summary.json')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('port')
    r.add_argument('outdir')
    r.add_argument('--suite', default='a', choices=list('0123456789a'))
    r.add_argument('--timeout', type=float, default=3900.0)
    a = sub.add_parser('analyse')
    a.add_argument('logs', nargs='+')
    a.add_argument('--json')
    args = ap.parse_args()
    if args.cmd == 'record':
        record(args.port, args.outdir, args.suite, args.timeout)
    else:
        analyse(args.logs, args.json)


if __name__ == '__main__':
    main()
