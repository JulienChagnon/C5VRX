// Test helper (not used by the flasher): a small ESP application image laid
// out as esptool writes one, with a C5VRX-5 pin table in its data.
import { PIN_MAGIC } from './image-pins.js';

export const XIAO_PINS = { dac: [23, 24, 11, 12, 8, 9], iq: [1, 0, 25, 2, 27, 5, 3, 4], boot: 28 };

export function pinTable(pins) {
  const table = new Uint8Array(32);
  table.set(PIN_MAGIC, 0);
  table.set(pins.dac, 8);
  table.set(pins.iq, 14);
  table[22] = pins.boot;
  return table;
}

// tables: how many pin tables the data segment holds (0, 1 or 2).
export async function fakeAppImage({ pins = XIAO_PINS, tables = 1, hashAppended = true } = {}) {
  const code = Uint8Array.from({ length: 64 }, (_, k) => (k * 37 + 11) & 0xff);
  const data = [new Uint8Array(40).fill(0x5a)];
  for (let k = 0; k < tables; ++k) data.push(pinTable(pins), new Uint8Array(12).fill(0xa5));
  const rodata = Uint8Array.from(data.flatMap(part => [...part]));
  const segments = [[0x42000020, code], [0x40800000, rodata]];
  const header = new Uint8Array(24);
  header.set([0xe9, segments.length, 0x02, 0x2f, 0x00, 0x00, 0x80, 0x40]);
  header[8] = 0xee;
  header[12] = 0x17;   // chip id: ESP32-C5
  header[23] = hashAppended ? 1 : 0;
  const parts = [header];
  let checksum = 0xef;
  for (const [address, bytes] of segments) {
    const head = new Uint8Array(8);
    new DataView(head.buffer).setUint32(0, address, true);
    new DataView(head.buffer).setUint32(4, bytes.length, true);
    parts.push(head, bytes);
    for (const b of bytes) checksum ^= b;
  }
  let length = parts.reduce((n, p) => n + p.length, 0);
  const padding = 15 - (length % 16);
  parts.push(new Uint8Array(padding), Uint8Array.of(checksum));
  length += padding + 1;
  const image = new Uint8Array(length + (hashAppended ? 32 : 0));
  let at = 0;
  for (const p of parts) { image.set(p, at); at += p.length; }
  if (hashAppended) image.set(new Uint8Array(await crypto.subtle.digest('SHA-256', image.subarray(0, length))), length);
  return image;
}

// The application at 0x10000 behind an erased bootloader area, as in c5vrx5_merged.bin.
export function mergedImage(app) {
  const merged = new Uint8Array(0x10000 + app.length).fill(0xff);
  merged.set(app, 0x10000);
  return merged;
}
