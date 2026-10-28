// C5VRX-5 by Twotoz and the C5VRX contributors: write the pin editor's pins
// into a C5VRX-5 application image.
//
// The firmware keeps its pins in one 32-byte table (main/board.h
// board_pins_t, main/board.c g_board_pins): the magic "C5VRXPN1", DAC
// D4..D9, IQ bits 0..7, the menu button, a custom flag and 8 reserved bytes.
// The table sits in a flash data segment of the ESP application image, so
// changing it changes the image checksum and its appended SHA-256, which
// the bootloader checks; both are recomputed here.
//
// ESP image format (esptool, bootloader_support): a 24-byte header (magic
// 0xE9, segment count at byte 1, hash_appended at byte 23), then per segment
// an 8-byte header (load address, length) and its data. After the last
// segment the file is padded so that the checksum byte (0xEF XOR every
// segment data byte) ends a 16-byte block; with hash_appended a SHA-256 of
// everything up to and including the checksum follows.

export const PIN_MAGIC = [0x43, 0x35, 0x56, 0x52, 0x58, 0x50, 0x4e, 0x31];   // "C5VRXPN1"
export const PIN_TABLE_BYTES = 32;
const IMAGE_MAGIC = 0xe9;
const HEADER_BYTES = 24;

export function appImageLayout(bytes, offset = 0) {
  const hex = n => `0x${n.toString(16)}`;
  if (bytes[offset] !== IMAGE_MAGIC) throw new Error(`No ESP application image at ${hex(offset)}`);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = bytes[offset + 1];
  const segments = [];
  let at = offset + HEADER_BYTES;
  for (let k = 0; k < count; ++k) {
    if (at + 8 > bytes.length) throw new Error(`Image at ${hex(offset)} ends inside segment ${k}`);
    const length = view.getUint32(at + 4, true);
    segments.push({ start: at + 8, length });
    at += 8 + length;
    if (at > bytes.length) throw new Error(`Image at ${hex(offset)} ends inside segment ${k}`);
  }
  const rel = at - offset;
  const checksumAt = at + (15 - (rel % 16));
  const hashAppended = bytes[offset + 23] === 1;
  const end = checksumAt + 1 + (hashAppended ? 32 : 0);
  if (end > bytes.length) throw new Error(`Image at ${hex(offset)} is cut short`);
  return { offset, segments, checksumAt, hashAppended, end };
}

async function sha256(data) {
  return new Uint8Array(await globalThis.crypto.subtle.digest('SHA-256', data));
}

// Checksum and SHA-256 as the bootloader computes them.
export async function verifyAppImage(bytes, offset = 0) {
  const layout = appImageLayout(bytes, offset);
  let checksum = 0xef;
  for (const s of layout.segments) for (let k = s.start; k < s.start + s.length; ++k) checksum ^= bytes[k];
  const checksumOk = checksum === bytes[layout.checksumAt];
  let hashOk = null;
  if (layout.hashAppended) {
    const digest = await sha256(bytes.subarray(offset, layout.checksumAt + 1));
    hashOk = digest.every((b, k) => b === bytes[layout.checksumAt + 1 + k]);
  }
  return { checksumOk, hashOk };
}

// Every place the magic starts inside segment data.
export function findPinTables(bytes, layout) {
  const found = [];
  for (const s of layout.segments) {
    for (let k = s.start; k + PIN_TABLE_BYTES <= s.start + s.length; ++k) {
      if (PIN_MAGIC.every((b, j) => bytes[k + j] === b)) found.push(k);
    }
  }
  return found;
}

const int8 = b => (b > 127 ? b - 256 : b);

export function readPinTable(bytes, at) {
  return {
    dac: Array.from(bytes.subarray(at + 8, at + 14), int8),
    iq: Array.from(bytes.subarray(at + 14, at + 22), int8),
    boot: int8(bytes[at + 22]),
    custom: bytes[at + 23] === 1
  };
}

// The pins the image carries now, or null for a build older than the table.
export function imagePins(bytes, offset = 0) {
  const layout = appImageLayout(bytes, offset);
  const tables = findPinTables(bytes, layout);
  return tables.length === 1 ? readPinTable(bytes, tables[0]) : null;
}

// A copy of bytes with the pins written into the application image at
// offset (0 for an application file, 0x10000 in a merged image), its
// checksum and SHA-256 fixed and checked again.
export async function writeImagePins(bytes, offset, pins) {
  const out = new Uint8Array(bytes);
  const layout = appImageLayout(out, offset);
  const tables = findPinTables(out, layout);
  if (tables.length === 0) throw new Error('This build has no pin table: it is older than the pin editor. Choose a newer build, or use the board\'s own pins.');
  if (tables.length > 1) throw new Error(`The image has ${tables.length} pin tables; refusing to guess which one the firmware reads.`);
  const at = tables[0];
  const table = new Uint8Array(PIN_TABLE_BYTES);
  table.set(PIN_MAGIC, 0);
  pins.dac.forEach((p, k) => { table[8 + k] = p & 0xff; });
  pins.iq.forEach((p, k) => { table[14 + k] = p & 0xff; });
  table[22] = pins.boot & 0xff;
  table[23] = 1;
  let delta = 0;
  for (let k = 0; k < PIN_TABLE_BYTES; ++k) delta ^= out[at + k] ^ table[k];
  out.set(table, at);
  out[layout.checksumAt] ^= delta;
  if (layout.hashAppended) out.set(await sha256(out.subarray(offset, layout.checksumAt + 1)), layout.checksumAt + 1);
  const check = await verifyAppImage(out, offset);
  if (!check.checksumOk || check.hashOk === false) throw new Error('The image did not verify after writing the pins; nothing was flashed.');
  return { bytes: out, at, before: readPinTable(bytes, at) };
}
