// Flight log reader (web/flight-log.js) against an image built the way
// main/flightlog.c writes one (layout main/flightlog_format.h).
import test from 'node:test';
import assert from 'node:assert/strict';
import { FlightLogCapture, decodeImage, summarize, secondsCsv, crc32, SECTOR, PARTITION_BYTES, SNAP_FIRST, SNAP_HEAD, SNAP_BYTES } from './flight-log.js';

const le = (n, bytes) => Array.from({ length: bytes }, (_, i) => (n >>> (8 * i)) & 0xff);

function second(t, snr, gain, dcI, temp, dropout = 0, level = null, output = null) {
  const p = new Uint8Array(output ? 69 : level === null ? 34 : 38);
  const d = new DataView(p.buffer);
  d.setUint32(0, t, true); d.setInt16(4, snr * 10, true); d.setUint16(6, 120, true); d.setInt16(8, 85, true);
  p[10] = 90; p[11] = dropout; p[12] = gain - 1; p[13] = gain; p[14] = gain + 1;
  d.setInt8(18, dcI / 0.05); d.setInt8(30, temp); p[28] = 1 | (3 << 2); p[29] = 1;
  if (level !== null) { d.setInt16(34, level * 10, true); d.setInt8(36, -95); p[37] = 1 | 2 | 4 | 8; }
  if (output) {
    p[50] = 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128; p[51] = 2; p[52] = 2; p[53] = 6;
    d.setUint16(54, 500, true); d.setUint16(56, 480, true); p[58] = 97; p[59] = 95;
    d.setUint16(60, 15625, true); d.setUint16(62, 12, true); d.setUint16(64, 3, true);
    d.setUint16(66, 106, true); p[68] = 2;
  }
  return p;
}

function image({ closed = true } = {}) {
  const img = new Uint8Array(PARTITION_BYTES).fill(0xff);
  const block = new Uint8Array(SECTOR).fill(0xff);
  block.set([...le(0x424c3543, 4), 1, 0, 0, 0, ...le(7, 4), ...le(42, 4), ...le(0, 4)]);
  let at = 20;
  const put = (type, payload) => { block[at] = type; block[at + 1] = payload.length; block.set(payload, at + 2); at += 2 + payload.length; };
  const session = new Uint8Array(48);
  session.set(le(7, 4)); session[4] = 1; session[5] = 9;   // brownout
  session.set(new TextEncoder().encode('5.0.9+abc'), 6);
  session.set(le(5865, 2), 46);
  put(1, session);
  put(4, second(10, 12.5, 80, 0.4, 41));
  put(4, second(11, 3.0, 83, 1.2, 42, 3, -88.5, true));
  put(5, new Uint8Array([...le(11500, 4), 10, 2]));
  if (closed) block.set(le(crc32(block.subarray(0, SECTOR - 4)), 4), SECTOR - 4);
  img.set(block, SECTOR * 5);
  const head = new Uint8Array(SNAP_HEAD);
  const iq = new Uint8Array(SNAP_BYTES).fill(0x11);
  head.set([...le(0x534c3543, 4), 1, 1, ...le(SNAP_BYTES, 2), ...le(7, 4), ...le(3, 4), ...le(11400, 4), ...le(5865, 2), 83]);
  head[38] = 2; head[39] = 6; head[40] = 1 | 16;
  head.set(le(crc32(iq, crc32(head.subarray(0, 60))), 4), 60);
  img.set(head, SNAP_FIRST * SECTOR);
  img.set(iq, SNAP_FIRST * SECTOR + SNAP_HEAD);
  return img;
}

test('decodes sessions, seconds, events and snapshots', () => {
  const { sessions, snapshots } = decodeImage(image());
  assert.equal(sessions.length, 1);
  const s = sessions[0];
  assert.equal(s.session, 7);
  assert.equal(s.boot.firmware, '5.0.9+abc');
  assert.equal(s.boot.resetReason, 'brownout');
  assert.equal(s.seconds.length, 2);
  assert.equal(s.seconds[0].snr, 12.5);
  assert.equal(s.seconds[0].profile, 'MATCHED');
  assert.equal(s.seconds[0].kp, 3);
  assert.equal(s.seconds[1].dropoutCategory, 'AT_RF_FLOOR');
  assert.equal(s.events[0].event, 'dropout');
  assert.equal(s.openBlocks, 0);
  assert.equal(snapshots.length, 1);
  assert.equal(snapshots[0].ok, true);
  assert.equal(snapshots[0].trigger, 'dropout');
  const sum = summarize(s);
  assert.equal(sum.durationS, 2);
  assert.equal(sum.snrMedian, 12.5);
  assert.equal(sum.snrWorst, 3);
  assert.equal(sum.dropouts, 1);
  assert.equal(sum.tempMax, 42);
  assert.equal(s.seconds[0].level, null);         // a 34-byte record: no level
  assert.equal(s.seconds[1].level, -88.5);
  assert.equal(s.seconds[1].noise, -95);
  assert.equal(s.seconds[0].outputDemod, null);  // an old record stays explicitly unknown
  assert.equal(s.seconds[1].outputDemod, 'U85Q-LP 1a09f5c092e8');
  assert.equal(s.seconds[1].outputCoveragePct, 96);
  assert.equal(s.seconds[1].nativeHLockPct, 97);
  assert.equal(s.seconds[1].repairedTips, 12);
  assert.equal(s.seconds[1].longestHUnlockMs, 106);
  assert.equal(sum.levelWorstWithPicture, -88.5);
  assert.equal(sum.outputDemod, 'U85Q-LP 1a09f5c092e8');
  assert.equal(sum.nativeHLockMedian, 97);
  assert.equal(sum.repairedTips, 12);
  assert.equal(snapshots[0].outputDemod, 'U85Q-LP 1a09f5c092e8');
  assert.equal(snapshots[0].outputLevel, 6);
  assert.match(secondsCsv(s), /^t,snr,lines/);
  assert.match(secondsCsv(s), /nativeHLockPct/);
});

test('an open block (power pulled) still reads, flagged open', () => {
  const { sessions } = decodeImage(image({ closed: false }));
  assert.equal(sessions[0].seconds.length, 2);
  assert.equal(sessions[0].openBlocks, 1);
});

test('a corrupted snapshot is reported, not trusted', () => {
  const img = image();
  img[SNAP_FIRST * SECTOR + SNAP_HEAD + 100] ^= 0xff;
  assert.equal(decodeImage(img).snapshots[0].ok, false);
});

test('console dump lines rebuild the image; damaged lines get another pass; missing partition is explained', () => {
  const img = image();
  const cap = new FlightLogCapture();
  cap.start();
  const hex = (o, n) => Array.from(img.subarray(o, o + n), (b) => b.toString(16).padStart(2, '0')).join('');
  const line = (o) => `FLOG_D ${o} ${hex(o, o === 0 ? 32 : 256)} ${crc32(img.subarray(o, o + (o === 0 ? 32 : 256))).toString(16).padStart(8, '0')}`;
  const offsets = [0];
  for (let o = SECTOR * 5; o < SECTOR * 6; o += 256) offsets.push(o);
  for (let o = SNAP_FIRST * SECTOR; o < (SNAP_FIRST + 2) * SECTOR; o += 256) offsets.push(o);
  assert.equal(cap.push('FLOG_BEGIN format=1 bytes=1572864 blocks=1 snapshots=1 session=7 fw=x'), true);
  for (const o of offsets) {
    let l = line(o);
    if (o === SECTOR * 5 + 512) l = l.slice(0, 300) + 'HB t_s=12 stage=ready' + l.slice(300);   // another task's line inside
    if (o === SECTOR * 5 + 768) l = l.replace(/ ([0-9a-f]{8})$/, ' 00000000');                 // looks like hex, wrong CRC
    assert.equal(cap.push(`STATUS x=1 ${l}`), true);
  }
  assert.equal(cap.push('FLOG_END blocks=1 snapshots=1 ms=38250 usb=fifo'), true);
  assert.equal(cap.damaged, 2);
  assert.equal(cap.done, false);
  assert.equal(cap.needsPass, true);
  cap.nextPass();
  for (const o of offsets) cap.push(line(o));
  cap.push('FLOG_END blocks=1 snapshots=1 ms=8150 usb=driver');
  assert.equal(cap.done, true);
  assert.equal(cap.dumpMs, 46400);
  assert.equal(cap.slowPasses, 1);
  assert.equal(cap.passes, 2);
  assert.ok(cap.progress > 0.99);
  assert.equal(decodeImage(cap.image).sessions[0].seconds.length, 2);
  const missing = new FlightLogCapture();
  missing.start();
  missing.push('FLOG partition=missing (flash the full image to add it)');
  assert.match(missing.error, /full firmware image/);
  assert.equal(new FlightLogCapture().push('CONTROL channel=A1'), false);
  const old = new FlightLogCapture();   // firmware before ms=/usb=
  old.start();
  old.push('FLOG_END blocks=0 snapshots=0');
  assert.equal(old.dumpMs, null);
  assert.equal(old.slowPasses, 0);
});
