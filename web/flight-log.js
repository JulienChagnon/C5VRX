// C5VRX-5 flight log in the browser (C5VRX by Twotoz and the C5VRX
// contributors). Reads the console dump (key W: FLOG_BEGIN / FLOG_D / FLOG_END)
// into a partition image and decodes it. Layout: main/flightlog_format.h,
// docs/FLIGHT_LOG.md; tools/flight_log.py reads the same image (.c5log).

export const SECTOR = 4096;
export const RECORD_BLOCKS = 255;
export const SNAP_FIRST = 256;
export const SNAP_SLOTS = 64;
export const SNAP_SECTORS = 2;
export const SNAP_HEAD = 64;
export const SNAP_BYTES = SNAP_SECTORS * SECTOR - SNAP_HEAD;
export const PARTITION_BYTES = (SNAP_FIRST + SNAP_SLOTS * SNAP_SECTORS) * SECTOR;
const BLOCK_HEAD = 20;
const CRC_AT = SECTOR - 4;
const MAGIC_BLOCK = 0x424c3543;
const MAGIC_SNAP = 0x534c3543;

export const EVENTS = { 1: 'channel', 2: 'profile', 3: 'kp', 4: 'cal_start', 5: 'cal_end', 6: 'dc_cal_needed',
  7: 'gain_mode', 8: 'menu', 9: 'fault', 10: 'dropout', 11: 'recovered', 12: 'snapshot', 13: 'dropped', 14: 'attack',
  15: 'output_gap', 16: 'output_unlock' };
export const DROPOUT = ['CALIBRATION_OR_PHY', 'FAST_EVIDENCE_LOSS', 'AT_RF_FLOOR', 'DEMOD_CLIFF', 'UNKNOWN'];
export const SPHASE = ['unverified', 'settled', 'drifted', 'unsettled', 'noise_limited', 'probe_failed'];
export const PROFILES = ['DEFAULT', 'MATCHED', 'NARROW', 'WIDE'];
export const RX_STATES = ['', 'ACQUIRE', 'TRACK', 'EDGE', 'RECOVER', 'FAULT', 'OWNED'];
export const RX_REASONS = ['none', 'no_carrier', 'weak_signal', 'dc_clipping', 'overload', 'dc_residual', 'sync_loss_with_rf', 'owned', 'fault', 'low_picture_snr'];
export const RESET_REASONS = ['unknown', 'power-on', 'external', 'software', 'panic', 'interrupt watchdog',
  'task watchdog', 'watchdog', 'deep sleep', 'brownout', 'SDIO', 'USB', 'JTAG', 'eFuse', 'power glitch', 'CPU lockup'];
const TRIGGERS = { 1: 'dropout', 2: 'picture drop', 3: 'first carrier', 4: 'periodic', 5: 'overload burst', 6: 'range edge', 7: 'recover', 8: 'output unlock' };
const OUTPUT_DEMODS = { 1: 'U85Q 1a09f5c092e8', 2: 'U85Q-LP 1a09f5c092e8' };
const OUTPUT_STANDARDS = { 1: 'NTSC', 2: 'PAL' };

let crcTable;
export function crc32(bytes, crc = 0) {
  if (!crcTable) {
    crcTable = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      crcTable[n] = c >>> 0;
    }
  }
  crc = ~crc >>> 0;
  for (let i = 0; i < bytes.length; i++) crc = crcTable[(crc ^ bytes[i]) & 0xff] ^ (crc >>> 8);
  return ~crc >>> 0;
}

// Collects a console dump. push() takes complete lines. Every FLOG_D line
// carries a CRC-32: other firmware tasks print too, so a line can arrive
// damaged; it is dropped and, after FLOG_END, `needsPass` asks for another
// dump (key W) that fills in only what is still missing. `done` once every
// announced block and snapshot arrived intact, or after MAX_PASSES.
// FLOG_END carries the pass's time (ms=) and USB path (usb=driver|fifo,
// main/flightlog.c); `dumpMs` sums the passes, `slowPasses` counts the fifo
// ones. Older firmware sends neither: dumpMs stays null.
const LINE_BYTES = 256;
export const MAX_PASSES = 4;
const LINE = /^FLOG_D (\d+) ([0-9a-f]+) ([0-9a-f]{8})$/;

function unit(offset) {
  if (offset < SECTOR) return { start: 0, lines: [0] };
  const sector = Math.floor(offset / SECTOR);
  const start = sector >= SNAP_FIRST ? (SNAP_FIRST + Math.floor((sector - SNAP_FIRST) / SNAP_SECTORS) * SNAP_SECTORS) * SECTOR : sector * SECTOR;
  const length = sector >= SNAP_FIRST ? SNAP_SECTORS * SECTOR : SECTOR;
  const lines = [];
  for (let o = start; o < start + length; o += LINE_BYTES) lines.push(o);
  return { start, lines };
}

export class FlightLogCapture {
  constructor() { this.reset(); }
  reset() {
    this.image = new Uint8Array(PARTITION_BYTES).fill(0xff);
    this.got = new Set();
    this.active = false;
    this.done = false;
    this.needsPass = false;
    this.passes = 0;
    this.damaged = 0;
    this.dumpMs = null;
    this.slowPasses = 0;
    this.error = '';
    this.info = {};
  }
  start() { this.reset(); this.nextPass(); }
  nextPass() { this.active = true; this.needsPass = false; this.passes += 1; }
  get expectedBytes() { return 32 + Number(this.info.blocks || 0) * SECTOR + Number(this.info.snapshots || 0) * SNAP_SECTORS * SECTOR; }
  get progress() { return this.expectedBytes > 32 ? Math.min(1, (this.got.size * LINE_BYTES) / this.expectedBytes) : 0; }
  // Complete units: header, record blocks, snapshot slots.
  completeness() {
    const units = new Map();
    for (const o of this.got) { const u = unit(o); units.set(u.start, u.lines); }
    let header = false, blocks = 0, snapshots = 0;
    for (const [start, lines] of units) {
      if (!lines.every((o) => this.got.has(o))) continue;
      if (start === 0) header = true; else if (start >= SNAP_FIRST * SECTOR) snapshots++; else blocks++;
    }
    return { header, blocks, snapshots };
  }
  // Returns true when the line belonged to the dump.
  push(line) {
    const at = line.indexOf('FLOG');
    if (at < 0) return false;
    line = line.slice(at).trim();
    if (line.startsWith('FLOG partition=missing')) { this.error = 'This module has no flight-log partition: flash the full firmware image once.'; this.active = false; return true; }
    if (!this.active) return line.startsWith('FLOG_D ');
    if (line.startsWith('FLOG_BEGIN')) {
      for (const [, k, v] of line.matchAll(/(\w+)=(\S+)/g)) this.info[k] = v;
      return true;
    }
    if (line.startsWith('FLOG_D ')) {
      const m = line.match(LINE);
      const o = m ? Number(m[1]) : NaN;
      if (!m || m[2].length % 2 || o + m[2].length / 2 > PARTITION_BYTES) { this.damaged++; return true; }
      const chunk = new Uint8Array(m[2].length / 2);
      for (let i = 0; i < chunk.length; i++) chunk[i] = parseInt(m[2].substr(i * 2, 2), 16);
      if (crc32(chunk) !== parseInt(m[3], 16)) { this.damaged++; return true; }
      this.image.set(chunk, o);
      this.got.add(o);
      return true;
    }
    if (line.startsWith('FLOG_END')) {
      this.active = false;
      const ms = line.match(/ ms=(\d+)/);
      if (ms) this.dumpMs = (this.dumpMs ?? 0) + Number(ms[1]);
      if (/ usb=fifo\b/.test(line)) this.slowPasses++;
      const c = this.completeness();
      const whole = c.header && c.blocks >= Number(this.info.blocks || 0) && c.snapshots >= Number(this.info.snapshots || 0);
      if (whole || this.passes >= MAX_PASSES) this.done = true; else this.needsPass = true;
      return true;
    }
    if (line.startsWith('FLOG_ERR')) { this.error = line; return true; }
    return line.startsWith('FLOG');
  }
}

function view(bytes) { return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength); }

function* records(block) {
  let at = BLOCK_HEAD;
  while (at + 2 <= CRC_AT) {
    const type = block[at], len = block[at + 1];
    if (type === 0 || type === 0xff || at + 2 + len > CRC_AT) break;
    yield [type, block.subarray(at + 2, at + 2 + len)];
    at += 2 + len;
  }
}

export function decodeSecond(p) {
  const d = view(p);
  const s = {
    t: d.getUint32(0, true), snr: d.getInt16(4, true) / 10, lines: d.getUint16(6, true) / 100,
    cn: d.getInt16(8, true), video: p[10], dropout: p[11], gainMin: p[12], gain: p[13], gainMax: p[14],
    clip: p[15] * 4, gainWrites: p[16], overloads: p[17], dcI: d.getInt8(18) * 0.05, dcQ: d.getInt8(19) * 0.05,
    offI: d.getInt8(20), offQ: d.getInt8(21), servo: p[22], afcOffset: d.getInt16(23, true),
    afcResidual: d.getInt16(25, true), afcSteps: p[27], profile: PROFILES[p[28] & 3], kp: (p[28] >> 2) & 7,
    lp: (p[28] >> 5) & 1, sphase: SPHASE[p[29]] ?? 'unknown', temp: d.getInt8(30), flags: d.getUint16(31, true),
    rxState: RX_STATES[p[33] & 7] ?? '', rxReason: p[33] ? RX_REASONS[p[33] >> 3] ?? 'unknown' : '',
    // Received level at the chip pin (main/rx_level.h), 38-byte records only.
    level: null, noise: null, levelExact: false,
    // Output observer fields are null for pre-69-byte records: missing is UNKNOWN.
    outputDemod: null, outputLevel: null, outputStandard: null, outputVerified: false,
    flywheelRequested: false, flywheelActive: false, outputObserverFresh: false,
    nativeHTrusted: false, nativeVTrusted: false, cameraPresent: false, outputFlagFormat: false,
    outputSamples: null, outputFreshSamples: null, outputCoveragePct: null,
    nativeHLockPct: null, nativeVLockPct: null, nativeHLines: null,
    repairedTips: null, suppressedTips: null, longestHUnlockMs: null, outputObserverGaps: null
  };
  // Why the gain moved (42-byte records only).
  s.writesAttack = p.length >= 42 ? p[38] : null;
  s.writesRelease = p.length >= 42 ? p[39] : null;
  s.sentinel = p.length >= 42 ? p[40] : null;
  s.overloadsAfterWrite = p.length >= 42 ? p[41] : null;
  // C/N-AUTO (50-byte records)
  const cnv = (o) => { const v = d.getInt16(o, true); return v <= -990 ? null : v / 10; };
  s.cnPower = p.length >= 50 ? cnv(42) : null;
  s.cnM2m4 = p.length >= 50 ? cnv(44) : null;
  s.cnConfidence = p.length >= 50 ? p[48] & 3 : null;
  if (p.length >= 69) {
    const of = p[50];
    s.outputVerified = !!(of & 1);
    s.flywheelRequested = !!(of & 2);
    s.flywheelActive = !!(of & 4);
    s.outputObserverFresh = !!(of & 8);
    s.nativeHTrusted = !!(of & 16);
    s.nativeVTrusted = !!(of & 32);
    s.cameraPresent = !!(of & 64);
    s.outputFlagFormat = !!(of & 128);
    s.outputDemod = s.outputVerified ? OUTPUT_DEMODS[p[51]] ?? `unknown (${p[51]})` : null;
    s.outputStandard = OUTPUT_STANDARDS[p[52]] ?? null;
    s.outputLevel = p[53] === 0xff ? null : p[53];
    s.outputSamples = d.getUint16(54, true);
    s.outputFreshSamples = d.getUint16(56, true);
    s.outputCoveragePct = s.outputSamples ? Math.round(s.outputFreshSamples * 100 / s.outputSamples) : null;
    s.nativeHLockPct = p[58] === 0xff ? null : p[58];
    s.nativeVLockPct = p[59] === 0xff ? null : p[59];
    s.nativeHLines = d.getUint16(60, true);
    s.repairedTips = d.getUint16(62, true);
    s.suppressedTips = d.getUint16(64, true);
    s.longestHUnlockMs = d.getUint16(66, true);
    s.outputObserverGaps = p[68];
  }
  if (p.length >= 38 && (p[37] & 1) && d.getInt16(34, true) !== -32768) {
    s.level = d.getInt16(34, true) / 10;
    s.noise = d.getInt8(36);
    s.levelExact = !!(p[37] & 4);
  }
  s.cn = s.cn <= -990 ? null : s.cn / 10;
  s.temp = s.temp <= -100 ? null : s.temp;
  s.dc = Math.hypot(s.dcI, s.dcQ);
  s.dropoutCategory = s.dropout ? DROPOUT[s.dropout - 1] ?? 'UNKNOWN' : '';
  return s;
}

export function decodeImage(image) {
  const blocks = [];
  for (let k = 0; k < RECORD_BLOCKS; k++) {
    const b = image.subarray((1 + k) * SECTOR, (2 + k) * SECTOR);
    const d = view(b);
    if (d.getUint32(0, true) !== MAGIC_BLOCK || b[4] !== 1) continue;
    blocks.push({ seq: d.getUint32(12, true), session: d.getUint32(8, true), closed: d.getUint32(CRC_AT, true) === crc32(b.subarray(0, CRC_AT)), b });
  }
  blocks.sort((a, b) => a.seq - b.seq);
  const sessions = new Map();
  for (const blk of blocks) {
    let s = sessions.get(blk.session);
    if (!s) sessions.set(blk.session, s = { session: blk.session, seconds: [], events: [], carriers: [], traces: [], blocks: 0, openBlocks: 0 });
    s.blocks++;
    if (!blk.closed) s.openBlocks++;
    for (const [type, p] of records(blk.b)) {
      const d = view(p);
      if (type === 1 && p.length >= 48) {
        s.boot = { resetReason: RESET_REASONS[p[5]] ?? String(p[5]), firmware: new TextDecoder().decode(p.subarray(6, 30)).replace(/\0.*$/s, ''),
          calGeneration: d.getUint16(30, true), boardId: d.getUint32(33, true).toString(16).padStart(8, '0'),
          channelIndex: p[40], gainMode: p[43], freq: d.getUint16(46, true) };
      } else if (type === 2 && p.length >= 56) {
        const filters = {};
        ['NORMAL', 'NARROW', 'WIDE'].forEach((name, c) => {
          const e = 6 + c * 8;
          filters[name] = p[e] ? { bw20: p[e + 1], code: d.getInt8(e + 2), skirt: p[e + 3], widthKhz: d.getUint16(e + 4, true), nbwKhz: d.getUint16(e + 6, true) } : null;
        });
        s.calib = { manual: !!p[0], filterResult: ['', 'loaded', 'stored', 'other'][Math.min(p[1], 3)], durationMs: d.getUint32(2, true),
          filters, dcRows: p[45], dcMissing: p[46], dcTemplate: p[47], repairFlags: p[48], sphase: SPHASE[p[49]] ?? 'unknown' };
      } else if (type === 3 && p.length >= 17) {
        s.carriers.push({ tMs: d.getUint32(0, true), freq: d.getUint16(4, true), afcOffset: d.getInt16(6, true),
          porchKhz: d.getInt32(8, true), deviation: d.getInt16(12, true) / 100, profile: PROFILES[p[14] & 3],
          standard: ['', 'NTSC', 'PAL'][Math.min(p[15], 2)] });
      } else if (type === 4 && p.length >= 34) {
        s.seconds.push(decodeSecond(p));
      } else if (type === 5 && p.length >= 5) {
        s.events.push({ tMs: d.getUint32(0, true), event: EVENTS[p[4]] ?? String(p[4]), data: p.slice(5) });
      } else if (type === 6 && p.length >= 6) {
        s.traces.push({ onsetMs: d.getUint32(0, true), part: p[4], count: p[5] });
      }
    }
  }
  const snapshots = [];
  for (let k = 0; k < SNAP_SLOTS; k++) {
    const at = (SNAP_FIRST + k * SNAP_SECTORS) * SECTOR;
    const h = image.subarray(at, at + SNAP_HEAD);
    const d = view(h);
    if (d.getUint32(0, true) !== MAGIC_SNAP) continue;
    const iq = image.subarray(at + SNAP_HEAD, at + SNAP_SECTORS * SECTOR);
    snapshots.push({ slot: k, ok: d.getUint32(60, true) === crc32(iq, crc32(h.subarray(0, 60))), trigger: TRIGGERS[h[5]] ?? String(h[5]),
      session: d.getUint32(8, true), seq: d.getUint32(12, true), tMs: d.getUint32(16, true), freq: d.getUint16(20, true),
      gain: h[22], outputDemod: h[40] & 1 ? OUTPUT_DEMODS[h[38]] ?? `unknown (${h[38]})` : null,
      outputLevel: h[40] & 1 && h[39] !== 0xff ? h[39] : null,
      outputFlags: h[40], iq });
  }
  snapshots.sort((a, b) => a.seq - b.seq);
  return { sessions: [...sessions.values()].sort((a, b) => a.session - b.session), snapshots };
}

const median = (values) => {
  const v = values.filter((x) => x !== null && Number.isFinite(x)).sort((a, b) => a - b);
  return v.length ? v[Math.floor(v.length / 2)] : null;
};

// One line per session for the table.
export function summarize(session) {
  const sec = session.seconds;
  const withPicture = sec.filter((s) => s.lines > 0);
  return {
    session: session.session,
    durationS: sec.length ? sec[sec.length - 1].t - sec[0].t + 1 : 0,
    firmware: session.boot?.firmware ?? '',
    resetReason: session.boot?.resetReason ?? '',
    freq: session.boot?.freq ?? null,
    pictureS: withPicture.length,
    snrMedian: median(withPicture.map((s) => s.snr)),
    snrWorst: withPicture.length ? Math.min(...withPicture.map((s) => s.snr)) : null,
    levelMedian: median(sec.map((s) => s.level)),
    levelWorstWithPicture: withPicture.some((s) => s.level !== null) ? Math.min(...withPicture.filter((s) => s.level !== null).map((s) => s.level)) : null,
    dropouts: session.events.filter((e) => e.event === 'dropout').length,
    dcWorst: sec.length ? Math.max(...sec.map((s) => s.dc)) : null,
    tempMax: median(sec.map((s) => s.temp)) === null ? null : Math.max(...sec.filter((s) => s.temp !== null).map((s) => s.temp)),
    deviation: session.carriers.length ? session.carriers[session.carriers.length - 1].deviation : null,
    outputDemod: [...sec].reverse().find((s) => s.outputDemod)?.outputDemod ?? null,
    outputLevel: [...sec].reverse().find((s) => s.outputLevel !== null)?.outputLevel ?? null,
    nativeHLockMedian: median(sec.map((s) => s.nativeHLockPct)),
    outputCoverageMedian: median(sec.map((s) => s.outputCoveragePct)),
    repairedTips: sec.some((s) => s.repairedTips !== null) ? sec.reduce((n, s) => n + (s.repairedTips ?? 0), 0) : null,
    longestHUnlockMs: sec.some((s) => s.longestHUnlockMs !== null) ? Math.max(...sec.map((s) => s.longestHUnlockMs ?? 0)) : null,
    states: Object.fromEntries(['TRACK', 'EDGE', 'RECOVER', 'ACQUIRE'].map((st) => [st, sec.filter((s) => s.rxState === st).length])),
    topReason: (() => { const c = {}; for (const s of sec) if (s.rxReason && s.rxReason !== 'none' && s.rxReason !== 'owned') c[s.rxReason] = (c[s.rxReason] ?? 0) + 1; return Object.entries(c).sort((a, b) => b[1] - a[1])[0]?.[0] ?? ''; })()
  };
}

export function secondsCsv(session) {
  const keys = ['t', 'snr', 'lines', 'cn', 'video', 'dropoutCategory', 'gainMin', 'gain', 'gainMax', 'clip', 'gainWrites',
    'overloads', 'dcI', 'dcQ', 'offI', 'offQ', 'servo', 'afcOffset', 'afcResidual', 'afcSteps', 'profile', 'kp', 'lp',
    'sphase', 'temp', 'flags', 'rxState', 'rxReason', 'level', 'noise', 'levelExact', 'writesAttack', 'writesRelease', 'sentinel', 'overloadsAfterWrite', 'cnPower', 'cnM2m4', 'cnConfidence',
    'outputDemod', 'outputLevel', 'outputStandard', 'outputVerified', 'flywheelRequested', 'flywheelActive',
    'outputObserverFresh', 'nativeHTrusted', 'nativeVTrusted', 'cameraPresent', 'outputFlagFormat',
    'outputSamples', 'outputFreshSamples', 'outputCoveragePct', 'nativeHLockPct', 'nativeVLockPct',
    'nativeHLines', 'repairedTips', 'suppressedTips', 'longestHUnlockMs', 'outputObserverGaps'];
  const rows = session.seconds.map((s) => keys.map((k) => (s[k] === null || s[k] === undefined ? '' : typeof s[k] === 'number' ? +s[k].toFixed(3) : s[k])).join(','));
  return [keys.join(','), ...rows].join('\n') + '\n';
}
