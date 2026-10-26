#!/usr/bin/env python3
"""tools/phy_sense_lab.py on synthetic console logs in the firmware format."""
import io
import math
import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import phy_sense_lab as lab  # noqa: E402


def cap(pid, step, rep, cn, video, click, rng):
    stage = 'STOCK' if step in (0, 3) else 'TEST'
    v = video + rng.gauss(0, 0.4)
    return (f"SENSLAB_CAP id={pid} step={step} stage={stage} rep={rep} gain=74 cn_db={cn + rng.gauss(0, 0.25):.3f} "
            f"click_pm={max(0.0, click + rng.gauss(0, 0.05)):.3f} origin_pm=3.000 rail_pm=0.100 video_lines=3 "
            f"video_snr_db={v:.2f} m2=4.0000 kurt=1.6000 dc=0.010/-0.020 fnv1a=0000abcd t_ms={1000 + rep}")


def profile(lines, pid, name, rng, d_cn=0.0, d_video=0.0, d_click=0.0, kind='patch', noop=0, status='PASS',
            drop_last=False, stock_drift=0.0):
    lines.append(f"SENSLAB_PROFILE id={pid} name={name} kind={kind} patches={0 if kind == 'sham' else 1} noop={noop} "
                 "sequence=STOCK_TEST_TEST_STOCK")
    for step in range(4):
        if step == 1:
            lines.append(f"SENSLAB_APPLY id={pid} verified={0 if status == 'REJECTED_ENTRY' else 1}")
            if status == 'REJECTED_ENTRY':
                lines.append(f"SENSLAB_RESTORE id={pid} verified=1")
                break
        if step == 3:
            lines.append(f"SENSLAB_RESTORE id={pid} verified=1")
        test = step in (1, 2)
        n = 15 if (drop_last and step == 3) else 16
        drift = stock_drift if step == 3 else 0.0
        for rep in range(n):
            line = cap(pid, step, rep, 5.0 + (d_cn if test else drift), 3.0 + (d_video if test else 0.0),
                       0.8 + (d_click if test else 0.0), rng)
            if pid == 7 and step == 2 and rep == 3:
                line = 'STATUS channel=R1 freq=5658 ' + line   # a joined console line
            lines.append(line)
    lines.append(f"SENSLAB_RESULT id={pid} name={name} status={status} d_cn_db=+0.000")


def make_run(seed, freq, good=1.0):
    rng = random.Random(seed)
    lines = ['boot noise', 'SENSLAB_SUITE key=a',
             f"SENSLAB_BEGIN fw=test suite=a freq={freq} offset_khz=0 gain=74 entry_gain=76 baseline_cn_db=5.10 "
             "baseline_click_pm=0.800 baseline_video_snr_db=3.00"]
    for k in range(6):
        profile(lines, 100 + k, 'SHAM', rng, kind='sham')
    profile(lines, 7, 'GOOD_KNOB', rng, d_cn=1.2 * good, d_video=1.5 * good, d_click=-0.4 * good)
    profile(lines, 8, 'BAD_KNOB', rng, d_cn=-1.2, d_video=-1.5, d_click=0.5)
    profile(lines, 9, 'NOTHING', rng)
    profile(lines, 10, 'LATCHLESS', rng, status='REJECTED_ENTRY')
    profile(lines, 11, 'SHORT', rng, drop_last=True)
    profile(lines, 12, 'ALREADY_STOCK', rng, d_cn=0.9, noop=1)
    profile(lines, 13, 'DRIFTING', rng, d_cn=0.6, stock_drift=1.5)
    lines.append('SENSLAB_SKIP id=604 name=SPUR_SLOT0_OFF reason=slot0_inactive')
    lines.append('SENSLAB_END status=COMPLETE profiles=13 rejected=1 breaks_iq=0 skipped=1 restored=1')
    return lines


def main():
    failures = 0

    def check(ok, what):
        nonlocal failures
        if not ok:
            print('FAIL', what)
            failures += 1

    with tempfile.TemporaryDirectory() as tmp:
        a, b = Path(tmp, 'a.log'), Path(tmp, 'b.log')
        a.write_text('\n'.join(make_run(1, 5805)) + '\n')
        b.write_text('\n'.join(make_run(2, 5865)) + '\n')
        out = io.StringIO()
        result = lab.analyse([a, b], Path(tmp, 's.json'), out)
        text = out.getvalue()
        print(text)
        verdicts = {p['name']: p['verdict'] for p in result['runs'][0]['profiles']}
        check(verdicts['GOOD_KNOB'] == 'BETTER', 'a clear improvement is BETTER')
        check(verdicts['BAD_KNOB'] == 'WORSE', 'a clear loss is WORSE')
        check(verdicts['NOTHING'] == 'NO_EFFECT', 'no change is NO_EFFECT')
        check(verdicts['LATCHLESS'] == 'REJECTED_ENTRY', 'a write that did not latch is REJECTED_ENTRY')
        check(verdicts['SHORT'] == 'INCOMPLETE', 'a missing capture is INCOMPLETE')
        check(verdicts['ALREADY_STOCK'] == 'NOOP', 'a candidate equal to stock is a no-op')
        check(verdicts['DRIFTING'] == 'UNSTABLE', 'drifting STOCK controls are UNSTABLE')
        check(all(v == 'SHAM' for n, v in verdicts.items() if n == 'SHAM'), 'shams are shams')
        good = next(p for p in result['runs'][0]['profiles'] if p['name'] == 'GOOD_KNOB')
        check(len(good['d']['cn_db']) and abs(good['d']['cn_db']['delta'] - 1.2) < 0.2, 'delta estimate')
        check(set(good['better']) == {'cn_db', 'video_snr_db', 'click_pm'}, 'all three metrics agree')
        comb = result['combined']
        check(comb['GOOD_KNOB']['overall'] == 'REPRODUCED_BETTER', 'repeated BETTER is reproduced')
        check(comb['BAD_KNOB']['overall'] == 'REPRODUCED_WORSE', 'repeated WORSE is reproduced')
        check(comb['NOTHING']['overall'] == 'NOT_REPRODUCED', 'no effect is not reproduced')
        check(Path(tmp, 's.json').stat().st_size > 1000, 'JSON written')
        check('skip SPUR_SLOT0_OFF' in text, 'skips are listed')
        # One BETTER and one NO_EFFECT run do not reproduce.
        c = Path(tmp, 'c.log')
        c.write_text('\n'.join(make_run(3, 5805, good=0.0)) + '\n')
        result = lab.analyse([a, c], None, io.StringIO())
        check(result['combined']['GOOD_KNOB']['overall'] == 'NOT_REPRODUCED', 'a single BETTER does not reproduce')
        # A refused run.
        d = Path(tmp, 'd.log')
        d.write_text('SENSLAB refused=no_vtx_video (the VTX must be ON)\n')
        out = io.StringIO()
        result = lab.analyse([d], None, out)
        check(result['runs'][0]['refused'].startswith('no_vtx_video'), 'refusals are reported')
        check(not math.isnan(len(out.getvalue())), 'report text')

    # The firmware's record format carries every field the tool scores.
    source = (Path(__file__).resolve().parent.parent / 'main' / 'sense_lab.c').read_text()
    cap_format = source[source.index('"SENSLAB_CAP '):source.index('t_ms=')]
    for field in ('id', 'step', 'stage', *lab.METRICS, *lab.EXTRA):
        check(f' {field}=' in cap_format or f'"SENSLAB_CAP {field}=' in cap_format, f'firmware prints {field}')
    for record in ('SENSLAB_PROFILE id=', 'SENSLAB_APPLY id=', 'SENSLAB_RESTORE id=', 'SENSLAB_RESULT id=',
                   'SENSLAB_BEGIN fw=', 'SENSLAB_END status=', 'SENSLAB_SKIP id=', 'SENSLAB refused='):
        check(record in source, f'firmware prints {record}')
    check(lab.CAPTURES_PER_STAGE == int(source.split('#define SENSE_CAPTURES')[1].split('u')[0]),
          'captures per stage match the firmware')

    if failures:
        print(f'test_sense_lab_tool: {failures} failure(s)')
        return 1
    print('test_sense_lab_tool: OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
