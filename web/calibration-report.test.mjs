import test from 'node:test';
import assert from 'node:assert/strict';
import { CalibrationCapture } from './calibration-report.js';
import { ReceiverConsole } from './receiver-console.js';
function frame(records = [{ type: 'device', schema: 1, board_id: '1234abcd' }, { type: 'capture', live_context_unchanged: true }]) {
  let hash = 2166136261;
  const lines = records.map((r, n) => {
    const json = JSON.stringify(r);
    for (const b of new TextEncoder().encode(json + '\n')) hash = Math.imul(hash ^ b, 16777619) >>> 0;
    return `CALDUMP_LINE n=${n} ${json}\n`;
  });
  return `CALDUMP_BEGIN schema=1\n${lines.join('')}CALDUMP_END lines=${records.length} fnv1a=${hash.toString(16).padStart(8, '0')}\n`;
}
test('verified export survives arbitrary USB chunk boundaries and interleaved telemetry', () => {
  const c = new CalibrationCapture(); c.start();
  for (const ch of frame().replace('CALDUMP_LINE n=1', 'STATUS gain=83\nCALDUMP_LINE n=1')) c.push(ch);
  assert.equal(c.done, true);
  const report = c.report({ boardLabel: 'Board 2', vtx: 'off', notes: 'USB, before F' });
  assert.equal(report.tester.boardLabel, 'Board 2');
  assert.equal(report.tester.vtx, 'off');
  assert.equal(report.integrity.complete, true);
  assert.equal(report.records[0].board_id, '1234abcd');
});
test('a truncated, missing, duplicated, corrupt or unsupported frame never becomes a download', () => {
  for (const text of [frame().replace(/CALDUMP_LINE n=0.*\n/, ''), frame().replace('n=1', 'n=0'),
    frame().replace('1234abcd', '1234abce'), frame().replace('schema=1', 'schema=2'), frame().replace('lines=2', 'lines=3')]) {
    const c = new CalibrationCapture();c.start();c.push(text);
    assert.equal(c.done, false); assert.ok(c.error); assert.throws(() => c.report());
  }
  const c = new CalibrationCapture();c.start();c.push(frame().split('CALDUMP_END')[0]);
  assert.equal(c.done, false);assert.throws(() => c.report());
});
test('restart discards old fragments, errors and records', () => {
  const c = new CalibrationCapture();c.start();c.push('CALDUMP_BEGIN schema=1\nCALDUMP_LI');
  c.fail('Disconnected');c.start();c.push(frame());assert.equal(c.done, true);assert.equal(c.records.length, 2);
});
test('bounded capture rejects oversized USB input', () => {
  const c = new CalibrationCapture();c.start();c.push('x'.repeat(9000));assert.ok(c.error);assert.equal(c.done, false);
});
test('old firmware cannot enable dump or full-optimization actions', () => {
  const r = new ReceiverConsole();r.push('C5VRX5 version=5.0.0\n');
  assert.equal(r.supports('C'), false);assert.equal(r.supports('B'), false);
  r.push('FIRMWARE id=5.0.0+abcd cal_dump=1\n');assert.equal(r.supports('C'), true);
  r.reset();assert.equal(r.supports('C'), false);
});
