// C5VRX by Twotoz and contributors: bounded, checksummed read-only USB export.
export class CalibrationCapture {
  constructor() { this.active = false; this.done = false; this.error = ''; this.pending = ''; }
  start() {
    this.active = true; this.done = false; this.error = ''; this.pending = '';
    this.begun = false; this.records = []; this.hash = 2166136261; this.bytes = 0;
  }
  fail(message) { this.active = false; this.done = false; this.error = message; }
  push(chunk) {
    if (!this.active) return false;
    const lines = (this.pending + chunk).split('\n');
    this.pending = lines.pop();
    if (this.pending.length > 8192) { this.fail('Oversized report line. Try again.'); return true; }
    let changed = false;
    for (const raw of lines) {
      if (!this.active) break;
      const line = raw.replace(/\r$/, '');
      if (line.startsWith('CALDUMP_BEGIN ')) {
        if (this.begun || line !== 'CALDUMP_BEGIN schema=1') this.fail('Unexpected report format. Try again.');
        else this.begun = true;
        changed = true;
      } else if (line.startsWith('CALDUMP_ERROR')) {
        this.fail('The receiver could not complete the report. Try again.'); changed = true;
      } else if (this.begun && line.startsWith('CALDUMP_LINE ')) {
        changed = true;
        const match = line.match(/^CALDUMP_LINE n=(\d+) (.+)$/);
        if (!match || Number(match[1]) !== this.records.length) { this.fail('Report data was lost over USB. Try again.'); continue; }
        this.bytes += match[2].length;
        if (this.records.length >= 1024 || this.bytes > 524288) { this.fail('Report exceeded its size limit.'); continue; }
        try {
          const record = JSON.parse(match[2]);
          if (!record || typeof record !== 'object' || Array.isArray(record) || typeof record.type !== 'string') throw new Error();
          this.records.push(record);
          for (const byte of new TextEncoder().encode(match[2] + '\n')) this.hash = Math.imul(this.hash ^ byte, 16777619) >>> 0;
        } catch { this.fail('Invalid report data. Try again.'); }
      } else if (this.begun && line.startsWith('CALDUMP_END ')) {
        changed = true;
        const match = line.match(/^CALDUMP_END lines=(\d+) fnv1a=([0-9a-f]{8})$/);
        if (!match || Number(match[1]) !== this.records.length || Number.parseInt(match[2], 16) !== this.hash ||
            !this.records.some(r => r.type === 'device' && r.schema === 1) || !this.records.some(r => r.type === 'capture')) {
          this.fail('Incomplete report or checksum mismatch. Try again.');
        } else { this.done = true; this.active = false; }
      }
    }
    return changed;
  }
  report({ boardLabel = '', vtx = 'unknown', notes = '', capturedAt = new Date().toISOString() } = {}) {
    if (!this.done) throw new Error('No complete verified calibration report.');
    return { schema: 'c5vrx.calibration-report/1', capturedAt,
      tester: { boardLabel: boardLabel.slice(0, 80), vtx, notes: notes.slice(0, 2000) },
      integrity: { complete: true, records: this.records.length, fnv1a: this.hash.toString(16).padStart(8, '0') },
      records: this.records };
  }
}
