"""C5VRX by Twotoz/contributors: restart/dropout association in saved logs.

Dropout events are classified from per-second observations, not a physical
sync edge. Association windows cannot establish causality. Merge overlapping
blackbox exports by session/event identity; report missing boot provenance.
"""
import argparse
import collections
import glob
import hashlib
import json
from pathlib import Path


def audit(patterns):
    sessions = {}
    inputs = {}
    for pattern in patterns:
        for name in sorted(glob.glob(pattern)):
            path = Path(name)
            data = path.read_bytes()
            j = json.loads(data)
            if 'session' not in j or 'events' not in j:
                continue
            inputs[path.name + ':' + path.parent.name] = hashlib.sha256(data).hexdigest()
            s = sessions.setdefault(j['session'], {'events': {}, 'boot': None})
            if j.get('boot'):
                if s['boot'] and s['boot'] != j['boot']:
                    raise ValueError(f"conflicting boot metadata for session {j['session']}")
                s['boot'] = j['boot']
            for e in j['events']:
                s['events'][(e['t_ms'], e['event'], e.get('data', ''))] = e
    rows = []
    for sid, s in sorted(sessions.items()):
        events = sorted(s['events'].values(), key=lambda e: e['t_ms'])
        switches = []
        for e in events:
            if e['event'] != 'kp':
                continue
            data = bytes.fromhex(e.get('data', ''))
            if len(data) < 6:
                continue
            switches.append(dict(t_ms=e['t_ms'], level=data[0], lp=bool(data[1]),
                                 success=bool(data[2]), duration_us=int.from_bytes(data[3:6], 'little')))
        drops = []
        for e in events:
            if e['event'] != 'dropout':
                continue
            before = [k for k in switches if k['t_ms'] <= e['t_ms']]
            after = [k for k in switches if k['t_ms'] > e['t_ms']]
            drops.append(dict(t_ms=e['t_ms'], category=e.get('data'),
                              previous_restart_ms=e['t_ms'] - before[-1]['t_ms'] if before else None,
                              next_restart_ms=after[0]['t_ms'] - e['t_ms'] if after else None))
        rows.append(dict(session=sid, boot=s['boot'], events=len(events),
                         dropped_event_records=sum(e['event'] == 'dropped' for e in events),
                         switches=switches, dropouts=drops))
    drops = [d for s in rows for d in s['dropouts']]
    result = dict(
        scope='Temporal association only; per-second dropout classifier; partial circular logs; no physical flywheel intervention trace',
        input_sha256=inputs, sessions=rows,
        totals=dict(sessions=len(rows), switches=sum(len(s['switches']) for s in rows),
                    dropouts=len(drops),
                    dropouts_preceded_by_restart={str(ms): sum(d['previous_restart_ms'] is not None and
                        d['previous_restart_ms'] <= ms for d in drops) for ms in [100, 1000, 2000, 5000]},
                    boot_flywheel_counts=dict(collections.Counter(
                        str(s['boot'].get('flywheel')) if s['boot'] else 'missing' for s in rows))))
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('patterns', nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    result = audit(args.patterns)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['totals'], indent=2))
