import test from 'node:test';
import assert from 'node:assert/strict';
import { appImageLayout, verifyAppImage, imagePins, writeImagePins } from './image-pins.js';
import { fakeAppImage, mergedImage, XIAO_PINS } from './fake-image.mjs';

const DEVKITC = { dac: [23, 24, 6, 10, 8, 9], iq: [1, 0, 25, 2, 26, 5, 3, 4], boot: 28 };

test('the fake image verifies like a real one, and a flipped byte does not', async () => {
  const image = await fakeAppImage();
  assert.deepEqual(await verifyAppImage(image), { checksumOk: true, hashOk: true });
  const layout = appImageLayout(image);
  const broken = image.slice();
  broken[layout.segments[1].start] ^= 1;
  const check = await verifyAppImage(broken);
  assert.equal(check.checksumOk, false);
  assert.equal(check.hashOk, false);
});

test('pins are written, the checksum and SHA-256 follow, nothing else changes', async () => {
  const image = await fakeAppImage();
  const { bytes, before } = await writeImagePins(image, 0, DEVKITC);
  assert.deepEqual(before, { ...XIAO_PINS, custom: false });
  assert.deepEqual(imagePins(bytes), { ...DEVKITC, custom: true });
  assert.deepEqual(await verifyAppImage(bytes), { checksumOk: true, hashOk: true });
  const layout = appImageLayout(bytes);
  const changed = [...bytes.keys()].filter(k => bytes[k] !== image[k]);
  const tableAt = layout.segments[1].start + 40;
  assert.ok(changed.every(k => (k >= tableAt && k < tableAt + 32) || k === layout.checksumAt || k > layout.checksumAt));
  assert.equal(image[tableAt + 23], 0, 'the input is not modified');
});

test('a merged image is patched at 0x10000, the bootloader area untouched', async () => {
  const merged = mergedImage(await fakeAppImage());
  const { bytes } = await writeImagePins(merged, 0x10000, DEVKITC);
  assert.ok(bytes.subarray(0, 0x10000).every(b => b === 0xff));
  assert.deepEqual(imagePins(bytes, 0x10000).dac, DEVKITC.dac);
  assert.deepEqual(await verifyAppImage(bytes, 0x10000), { checksumOk: true, hashOk: true });
});

test('an image without an appended hash keeps a valid checksum', async () => {
  const image = await fakeAppImage({ hashAppended: false });
  const { bytes } = await writeImagePins(image, 0, DEVKITC);
  assert.deepEqual(await verifyAppImage(bytes), { checksumOk: true, hashOk: null });
});

test('an older build without a table, or an ambiguous one, is refused', async () => {
  await assert.rejects(writeImagePins(await fakeAppImage({ tables: 0 }), 0, DEVKITC), /older than the pin editor/);
  assert.equal(imagePins(await fakeAppImage({ tables: 0 })), null);
  await assert.rejects(writeImagePins(await fakeAppImage({ tables: 2 }), 0, DEVKITC), /2 pin tables/);
  await assert.rejects(writeImagePins(new Uint8Array(64), 0, DEVKITC), /No ESP application image/);
  const cut = (await fakeAppImage()).slice(0, 80);
  await assert.rejects(writeImagePins(cut, 0, DEVKITC), /ends inside segment|cut short/);
});
