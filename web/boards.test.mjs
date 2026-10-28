import test from 'node:test';
import assert from 'node:assert/strict';
import { BOARDS, findBoard, imageAssets, decodeC5Package, flashMbFromId, detectChip, autoImage, imageProblem } from './boards.js';

const efuse = ({ temp = 0, flash = 0, psram = 0 }) => ((temp << 24) | (psram << 19) | (flash << 13)) >>> 0;
const image = id => BOARDS.flatMap(board => board.images).find(i => i.id === id);

test('eFuse package word names the chip like esptool', () => {
  assert.deepEqual(decodeC5Package(efuse({ temp: 1, psram: 2 })), { name: 'ESP32-C5HR8', psramMb: 8 });
  assert.deepEqual(decodeC5Package(efuse({ temp: 1, flash: 1 })), { name: 'ESP32-C5HF4', psramMb: 0 });
  assert.deepEqual(decodeC5Package(efuse({})), { name: 'ESP32-C5N', psramMb: 0 });
  assert.equal(decodeC5Package(efuse({ psram: 5 })).psramMb, null);
});

test('JEDEC flash ID gives the size; unknown codes stay unknown', () => {
  assert.equal(flashMbFromId(0x174020), 8);
  assert.equal(flashMbFromId(0x16405e), 4);
  assert.equal(flashMbFromId(0x18400b), 16);
  assert.equal(flashMbFromId(0xffffff), null);
});

test('chip detection reads the eFuse and flash ID, and survives a loader that cannot', async () => {
  const loader = {
    chip: { CHIP_NAME: 'ESP32-C5', EFUSE_BLOCK1_ADDR: 0x600b4844 },
    readReg: async addr => { assert.equal(addr, 0x600b4844 + 8); return efuse({ temp: 1, psram: 2 }); },
    readFlashId: async () => 0x174020
  };
  assert.deepEqual(await detectChip(loader), { name: 'ESP32-C5HR8', flashMb: 8, psramMb: 8 });
  const broken = { chip: loader.chip, readReg: async () => { throw new Error('no'); }, readFlashId: async () => { throw new Error('no'); } };
  assert.deepEqual(await detectChip(broken), { name: null, flashMb: null, psramMb: null });
});

test('auto picks PSRAM when the chip has it and the largest layout that fits', () => {
  const xiao = findBoard('xiao');
  assert.equal(autoImage(xiao, { flashMb: 8, psramMb: 8 }).id, 'xiao-psram');
  assert.equal(autoImage(xiao, { flashMb: 8, psramMb: 0 }).id, 'xiao');
  assert.equal(autoImage(xiao, { flashMb: 4, psramMb: 0 }), null);   // no 4 MB XIAO exists
  assert.equal(autoImage(findBoard('custom'), { flashMb: 4, psramMb: 0 }).id, 'xiao-4mb');
  assert.equal(autoImage(findBoard('custom'), { flashMb: 8, psramMb: 8 }).id, 'xiao-psram');
  assert.equal(autoImage(xiao, { flashMb: 16, psramMb: null }).id, 'xiao');
  assert.equal(autoImage(xiao, null).id, 'xiao');
  // A release without the PSRAM image still flashes the standard one.
  assert.equal(autoImage(xiao, { flashMb: 8, psramMb: 8 }, i => i.id !== 'xiao-psram').id, 'xiao');
  assert.equal(autoImage(findBoard('nicemcu'), { flashMb: 4, psramMb: 0 }).id, 'nicemcu');
  assert.equal(autoImage(findBoard('devkitc'), { flashMb: 2, psramMb: 0 }), null);
  assert.equal(findBoard('unknown').id, 'xiao');
});

test('an 8 MB image never goes on a 4 MB chip; PSRAM without PSRAM needs a yes', () => {
  assert.equal(imageProblem(image('xiao'), { flashMb: 4, psramMb: 0 }).level, 'block');
  assert.match(imageProblem(image('xiao'), { flashMb: 4, psramMb: 0 }).text, /4 MB image/);
  assert.equal(imageProblem(image('xiao-psram'), { flashMb: 8, psramMb: 0 }).level, 'ask');
  assert.equal(imageProblem(image('xiao-psram'), { flashMb: 8, psramMb: 8 }), null);
  assert.equal(imageProblem(image('devkitc'), { flashMb: 8, psramMb: 0 }), null);
  assert.equal(imageProblem(image('devkitc'), { flashMb: null, psramMb: 0 }).level, 'ask');
});

test('release files: variants by name, the standard image by its application', () => {
  const assets = ['c5vrx5.bin', 'c5vrx5_merged.bin', 'c5vrx5-devkitc.app.bin', 'c5vrx5-devkitc.full.bin', 'c5vrx5-pico.app.bin']
    .map(name => ({ name }));
  assert.equal(imageAssets(image('xiao'), assets).full.name, 'c5vrx5_merged.bin');
  assert.equal(imageAssets(image('devkitc'), assets).app.name, 'c5vrx5-devkitc.app.bin');
  assert.equal(imageAssets(image('pico'), assets), null);   // no full image
  assert.equal(imageAssets(image('xiao'), [{ name: 'c5vrx5.bin' }]).full, null);
});

test('board table: unique ids, six DAC and eight IQ pins, no pin used twice', () => {
  const ids = new Set();
  for (const board of BOARDS) {
    assert.ok(!ids.has(board.id));
    ids.add(board.id);
    assert.equal(board.dac.length, 6);
    assert.equal(board.iq.length, 8);
    const pins = [...board.dac, ...board.iq, board.boot];
    assert.equal(new Set(pins).size, pins.length, board.id);
    assert.ok(board.images.length > 0);
    assert.match(board.pinout, /^https:\/\//, board.id);
  }
  assert.equal(BOARDS[0].images[0].asset, 'standard');
});

test('pin rules match the firmware case by case (test/data/board_pin_cases.txt)', async () => {
  const { pinProblem } = await import('./boards.js');
  const fs = await import('node:fs');
  const text = fs.readFileSync(new URL('../test/data/board_pin_cases.txt', import.meta.url), 'utf8');
  let cases = 0;
  for (const line of text.split(/\r?\n/)) {
    if (!line.trim() || line.startsWith('#')) continue;
    const [psram, dac, iq, boot, want] = line.trim().split(/\s+/);
    const pins = { dac: dac.split(',').map(Number), iq: iq.split(',').map(Number), boot: Number(boot) };
    assert.equal(pinProblem(pins, psram === '1') ?? 'ok', want, line);
    ++cases;
  }
  assert.ok(cases >= 20, `only ${cases} cases`);
});

test('warnings: onboard parts, strapping pins and pins on no header', async () => {
  const { pinWarnings, presetPins } = await import('./boards.js');
  const devkitc = findBoard('devkitc');
  const layout = devkitc.layouts[0];
  assert.deepEqual(pinWarnings(presetPins(devkitc), devkitc, layout), []);
  const xiao = findBoard('xiao');
  assert.deepEqual(pinWarnings(presetPins(xiao), xiao, xiao.layouts[0]), [], 'the XIAO LED on IQ is its own choice');
  const moved = { ...presetPins(devkitc), dac: [23, 24, 11, 7, 8, 9] };
  const warnings = pinWarnings(moved, devkitc, layout);
  assert.ok(warnings.some(w => /D6 on GPIO11: the board's USB-UART bridge \(TX\)/.test(w)));
  assert.ok(warnings.some(w => /D7 on GPIO7, a strapping pin/.test(w)));
  const iqOnLed = { ...presetPins(devkitc), iq: [1, 0, 25, 2, 27, 5, 3, 4] };
  assert.ok(pinWarnings(iqOnLed, devkitc, layout).some(w => /IQ I4 on GPIO27: the board's RGB LED/.test(w)));
  const nowhere = { ...presetPins(xiao), dac: [23, 24, 6, 12, 8, 9] };
  assert.ok(pinWarnings(nowhere, xiao, xiao.layouts[0]).some(w => /D6 on GPIO6: not on this board's headers/.test(w)));
});

test('pin choices name each GPIO on the board and block the unusable ones', async () => {
  const { pinChoices } = await import('./boards.js');
  const xiao = findBoard('xiao');
  const dac = pinChoices(xiao, xiao.layouts[0], 'dac');
  assert.equal(dac.length, 29);
  assert.equal(dac[23].label, 'GPIO23 · D4');
  assert.equal(dac[27].label, 'GPIO27 · user LED · strapping pin · not on a header');
  assert.equal(dac[7].label, 'GPIO7 · D3 · strapping pin');
  assert.deepEqual([dac[13].disabled, dac[14].disabled, dac[16].disabled, dac[22].disabled, dac[28].disabled], [true, true, true, true, true]);
  assert.equal(dac[13].label, 'GPIO13 · USB');
  assert.equal(pinChoices(xiao, xiao.layouts[0], 'boot')[28].disabled, false);
  const pico = findBoard('pico');
  assert.equal(pinChoices(pico, pico.layouts[0], 'dac')[10].label, 'GPIO10 · pin 14');
  const devkitc = findBoard('devkitc');
  assert.equal(pinChoices(devkitc, devkitc.layouts[0], 'iq')[23].label, 'GPIO23', 'the printed label is the GPIO number');
  assert.equal(pinChoices(findBoard('custom'), null, 'dac')[6].label, 'GPIO6');
});
