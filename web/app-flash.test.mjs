import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { CalibrationCapture } from './calibration-report.js';
import { ReceiverConsole, MENU_COMMAND_KEYS } from './receiver-console.js';
import { FlightLogCapture, decodeImage, summarize, secondsCsv } from './flight-log.js';
import * as boardsModule from './boards.js';
import * as diagramModule from './board-diagram.js';
import * as imagePinsModule from './image-pins.js';
import { fakeAppImage, mergedImage, XIAO_PINS } from './fake-image.mjs';

// Exercise the actual click handler with a mock loader: never touch USB hardware.
const source = fs.readFileSync(process.env.APP_SOURCE_PATH || new URL('./app.js', import.meta.url), 'utf8').replace(/^import .*;\r?\n/gm, '');
function harness(sourceName, packageType = 'merged', confirmed = true) {
  const elements = new Map();
  const makeElement = () => ({
    style: {}, handlers: {}, value: '', textContent: '', checked: false, dataset: {}, innerHTML: '',
    classList: { toggle() {}, add() {}, remove() {} },
    addEventListener(name, fn) { this.handlers[name] = fn; },
    setAttribute() {}, appendChild() {}, append() {}, replaceChildren() {}
  });
  const get = id => {
    if (!elements.has(id)) elements.set(id, makeElement());
    return elements.get(id);
  };
  const html = fs.readFileSync(new URL('./index.html', import.meta.url), 'utf8');
  const shortcuts = [...html.matchAll(/<button[^>]*data-command="([^"]+)"([^>]*)>/g)].map((match, i) => {
    const element = get(`shortcut-${i}`);
    element.dataset = { command: match[1], hint: match[2].match(/data-hint="([^"]+)"/)?.[1] };
    return element;
  });
  const downloads = [], flashes = [], alerts = [], commands = [], lifecycle = [], resets = [];
  const files = {};   // release file name -> bytes the mock fetch serves
  const context = vm.createContext({
    console, URL, Date, TextDecoder, TextEncoder, CalibrationCapture, ReceiverConsole, MENU_COMMAND_KEYS, setTimeout, clearTimeout,
    FlightLogCapture, decodeImage, summarize, secondsCsv,
    ...boardsModule, ...diagramModule, ...imagePinsModule, crypto: globalThis.crypto,
    setInterval: () => 0, clearInterval() {},
    document: { baseURI: 'https://c5vrx.com/', getElementById: get, visibilityState: 'visible',
      querySelectorAll: selector => selector === '[data-command]' ? shortcuts : [], addEventListener() {}, createElement: makeElement },
    navigator: { serial: { addEventListener() {} } },
    window: { confirm: () => confirmed, matchMedia: () => ({ matches: true, addEventListener() {} }) },
    SerialTerminal: class {
      constructor(options) { this.state = 'disconnected'; this.options = options; }
      async connect() { lifecycle.push('terminal connect'); this.state = 'connected'; this.options.onState(this.state); }
      async disconnect() { lifecycle.push('terminal disconnect'); this.state = 'disconnected'; this.options.onState(this.state); }
      async send(text) { commands.push(text); return true; }
    },
    alert: message => alerts.push(message),
    fetch: async url => {
      downloads.push(url);
      const data = files[url.split('/').pop()] || new Uint8Array([1, 2, 3, 4]);
      return { ok: true, arrayBuffer: async () => data.slice().buffer };
    },
    mockLoader: { writeFlash: async options => flashes.push(options),
      writeReg: async (...args) => resets.push(['writeReg', ...args]), after: async mode => resets.push(['after', mode]) }
  });
  vm.runInContext(source, context);
  for (const id of ['selectRelease', 'selectAlphaBuild']) get(id).value = '0';
  for (const id of ['selectPackageType', 'selectAlphaPackageType']) get(id).value = packageType;
  vm.runInContext(`
    activeSource = ${JSON.stringify(sourceName)};
    isConnected = true; esploader = mockLoader;
    const rel = { tag_name: 'c5vrx5-pr-1', prerelease: true, assets: [
      { name: 'bootloader.bin', local_url: 'firmware/alpha/bootloader.bin', size: 4 },
      { name: 'partition-table.bin', local_url: 'firmware/alpha/partition-table.bin', size: 4 },
      { name: 'c5vrx5.bin', local_url: 'firmware/alpha/c5vrx5.bin', size: 4 },
      { name: 'c5vrx5_merged.bin', local_url: 'firmware/alpha/c5vrx5_merged.bin', size: 4 }
    ] };
    githubReleases = [rel]; githubAlphaBuilds = [rel];
  `, context);
  return { get, context, files, downloads, flashes, alerts, commands, lifecycle, resets, shortcuts, click: () => get('btnFlash').handlers.click() };
}

for (const remote of ['github', 'alpha']) {
  test(`${remote} Full firmware downloads the merged binary at 0x0`, async () => {
    const h = harness(remote);
    await h.click();
    assert.deepEqual(h.downloads, ['https://c5vrx.com/firmware/alpha/c5vrx5_merged.bin']);
    assert.equal(h.flashes.length, 1);
    assert.equal(h.flashes[0].fileArray[0].address, 0);
    assert.ok(!h.alerts.some(x => x.startsWith('Flashing error:')));
  });
}
test('alpha App only downloads the application at 0x10000', async () => {
  const h = harness('alpha', 'app_only');
  await h.click();
  assert.deepEqual(h.downloads, ['https://c5vrx.com/firmware/alpha/c5vrx5.bin']);
  assert.equal(h.flashes[0].fileArray[0].address, 0x10000);
});
test('alpha Full firmware falls back to all three flash parts', async () => {
  const h = harness('alpha');
  vm.runInContext("githubAlphaBuilds[0].assets = githubAlphaBuilds[0].assets.filter(a => !a.name.includes('merged'))", h.context);
  await h.click();
  assert.equal(h.downloads.length, 3);
  assert.deepEqual(Array.from(h.flashes[0].fileArray, part => part.address), [0x2000, 0x8000, 0x10000]);
});
test('cancelled alpha confirmation never downloads or flashes', async () => {
  const h = harness('alpha', 'merged', false);
  await h.click();
  assert.equal(h.downloads.length, 0);
  assert.equal(h.flashes.length, 0);
});
test('only Local File requires a local binary', async () => {
  const h = harness('local');
  await h.click();
  assert.equal(h.downloads.length, 0);
  assert.equal(h.flashes.length, 0);
  assert.ok(h.alerts.includes('Flashing error: No local binary selected'));
});

test('terminal connection releases flasher before identifying firmware', async () => {
  const h = harness('alpha');
  vm.runInContext("transport = { disconnect: async () => { log('released'); } }", h.context);
  await h.get('btnTerminalConnect').handlers.click();
  assert.equal(vm.runInContext('isConnected', h.context), false);
  assert.deepEqual(h.commands, ['hT']);
  assert.equal(h.get('btnConnect').disabled, false);
});

test('menu buttons and typed keys send at once: no browser popup, no Enter', async () => {
  const h = harness('alpha', 'merged', false);   // a confirm() answer of "cancel" must not matter
  vm.runInContext("isConnected = false; serialConsole.state = 'connected'; receiverConsole.push('C5VRX-5 keys: T status\\n');", h.context);
  const calibrate = h.shortcuts.find(b => b.dataset.command === 'F');
  await calibrate.handlers.click();
  assert.deepEqual(h.commands, ['F']);
  h.get('commandInput').value = 'F';
  await h.get('commandForm').handlers.submit({ preventDefault() {} });
  assert.deepEqual(h.commands, ['F', 'F']);
  h.get('commandInput').value = 'Fg';
  await h.get('commandForm').handlers.submit({ preventDefault() {} });
  assert.deepEqual(h.commands, ['F', 'F']);
  vm.runInContext('syncDeviceControls()', h.context);
  assert.match(calibrate.title, /Live video stops/, 'the warning stays visible as a tooltip');
});

test('the STATUS beat updates the menu by itself and stays out of the log', async () => {
  const h = harness('alpha');
  vm.runInContext("isConnected = false; serialConsole.state = 'connected'", h.context);
  vm.runInContext("appendSerial('C5VRX5 version=v5.0.9 demod=U85Q\\nSTATUS channel=F4 freq=5800 afc_locked=1 standard=AUTO detected=PAL signal=71 menu=0 mode=WIDE kp_auto=2 output=1.5x live=1 busy=0\\nHB t_s=4\\nSTAT')", h.context);
  vm.runInContext("appendSerial('US channel=F4 freq=5800 signal=72 busy=0\\n'); flushLog()", h.context);
  const shown = h.get('output').textContent;
  assert.doesNotMatch(shown, /STATUS/);
  assert.match(shown, /HB t_s=4/);
  const view = vm.runInContext('receiverConsole.view()', h.context);
  assert.equal(view.channel, 'F4');
  assert.equal(view.signal, 72);
  assert.equal(view.output, '1.5x');
  assert.equal(view.profile, 'WIDE');
  await h.shortcuts.find(b => b.dataset.command === 'c').handlers.click();
  await new Promise(resolve => setTimeout(resolve, 800));
  assert.deepEqual(h.commands, ['c'], 'with the beat no extra status report is requested');
});

test('legacy identity leaves current shortcuts disabled and help usable', () => {
  const h = harness('alpha');
  vm.runInContext("isConnected = false; serialConsole.state = 'connected'; appendSerial('C5VRX-3 ready\\n');", h.context);
  assert.equal(h.shortcuts.find(b => b.dataset.command === 'g').disabled, true);
  assert.equal(h.shortcuts.find(b => b.dataset.command === 'h').disabled, false);
});


test('a menu command is followed by one status refresh; browsing sends nothing', async () => {
  const h = harness('alpha');
  vm.runInContext("isConnected = false; serialConsole.state = 'connected'; receiverConsole.push('C5VRX-5 keys: T status\\n');", h.context);
  await h.shortcuts.find(b => b.dataset.command === 'c').handlers.click();
  assert.deepEqual(h.commands, ['c']);
  await new Promise(resolve => setTimeout(resolve, 800));
  assert.deepEqual(h.commands, ['c', 'T']);
  await h.shortcuts.find(b => b.dataset.command === 'n').handlers.click();
  await new Promise(resolve => setTimeout(resolve, 800));
  assert.deepEqual(h.commands, ['c', 'T', 'n'], 'moving the goggle cursor changes no reported value');
});

test('Pages deployment fingerprints every module app.js imports', () => {
  const app = fs.readFileSync(new URL('./app.js', import.meta.url), 'utf8');
  const script = fs.readFileSync(new URL('../tools/prepare_pages_site.sh', import.meta.url), 'utf8');
  const listed = script.match(/modules = \(([^)]*)\)/)[1];
  const nested = script.match(/for name in \(([^)]*)\):\n    module = /)[1];
  for (const [, name] of app.matchAll(/^import .* from '\.\/([^']+)';/gm)) {
    assert.ok(listed.includes(`"${name}"`), `${name} would be served from a stale cache`);
    // A module that imports another module needs its imports fingerprinted too.
    const module = fs.readFileSync(new URL(`./${name}`, import.meta.url), 'utf8');
    for (const [, dep] of module.matchAll(/^import .* from '\.\/([^']+)';/gm)) {
      assert.ok(listed.includes(`"${dep}"`) && nested.includes(`"${name}"`), `${name} imports ${dep} unfingerprinted`);
    }
  }
});

test('Local File refuses an offset that would overwrite the wrong sector', async () => {
  const h = harness('local');
  vm.runInContext('localFileBinary = new Uint8Array([1, 2, 3, 4]).buffer;', h.context);
  for (const offset of ['0x10', 'zz']) {
    h.get('inputFlashOffset').value = offset;
    await h.click();
  }
  assert.equal(h.flashes.length, 0);
  h.get('inputFlashOffset').value = '0x10000';
  await h.click();
  assert.equal(h.flashes[0].fileArray[0].address, 0x10000);
});

test('menu buttons are disabled while the receiver is busy, diagnostics stay usable', () => {
  const h = harness('alpha');
  vm.runInContext("isConnected = false; serialConsole.state = 'connected'; appendSerial('C5VRX-5 keys: T\\nHB t_s=1 stage=ready gain=40 menu=0 startup=1 busy=1\\n');", h.context);
  const key = k => h.shortcuts.find(b => b.dataset.command === k);
  assert.equal(key('c').disabled, true);
  assert.equal(key('D').disabled, false);
  assert.equal(key('T').disabled, false);
  vm.runInContext("appendSerial('HB t_s=6 stage=ready gain=40 menu=0 startup=0 busy=0\\n');", h.context);
  assert.equal(key('c').disabled, false);
});

test('controls stay locked until the post-flash disconnect completes', async () => {
  const h = harness('alpha');
  let release;
  vm.runInContext('transport = { disconnect: () => new Promise(r => { globalThis.releaseDisconnect = r; }) };', h.context);
  const done = h.click();
  // The watchdog reset waits a second before the port is released.
  for (let t = 0; t < 300 && !vm.runInContext('globalThis.releaseDisconnect', h.context); ++t)
    await new Promise(resolve => setTimeout(resolve, 10));
  assert.equal(vm.runInContext('isFlashing', h.context), true);
  assert.equal(h.get('btnTerminalConnect').disabled, true);
  release = vm.runInContext('releaseDisconnect', h.context);
  release();
  await done;
  assert.equal(vm.runInContext('isFlashing || isConnected', h.context), false);
  assert.equal(h.get('btnTerminalConnect').disabled, false);
});

test('calibration report button requests only read-only C, never F/B or flash', async () => {
  const h = harness('github');
  vm.runInContext("serialConsole.state='connected'; receiverConsole.push('C5VRX5 version=5.0.0-pr.272\\nFIRMWARE id=pr272 cal_dump=1\\n');", h.context);
  await h.get('btnCalibrationReport').handlers.click();
  assert.deepEqual(h.commands, ['C']);
  assert.equal(h.flashes.length, 0);
  assert.equal(h.get('btnCalibrationReport').disabled, true);
  vm.runInContext('clearTimeout(calibrationTimer); calibrationCapture.fail("test end");', h.context);
});

// A serial port that answers the download key the way a running receiver
// does (reply), or not at all (a board in the ROM loader or a busy console).
function mockPort(reply) {
  const port = { written: [], getInfo: () => ({ usbVendorId: 0x303a, usbProductId: 0x1001 }), close: async () => {} };
  port.open = async () => {
    let wake = null, pending = null;
    port.readable = { getReader: () => ({
      read: () => new Promise(resolve => { if (pending) { resolve(pending); pending = null; } else wake = resolve; }),
      cancel: async () => wake?.({ done: true }), releaseLock() {} }) };
    port.writable = { getWriter: () => ({
      write: async bytes => {
        port.written.push(...bytes);
        if (!reply) return;
        const chunk = { value: new TextEncoder().encode(reply) };
        if (wake) { const w = wake; wake = null; w(chunk); } else pending = chunk;
      },
      abort: async () => {} }) };
  };
  return port;
}

// reconnectAs: the port Chrome hands back after the restart drops the board
// off USB ('disconnect' then 'connect' on navigator.serial).
function connectHarness(port, failingModes, reconnectAs = null) {
  const h = harness('alpha');
  const modes = [], loaderPorts = [], listeners = { connect: new Set(), disconnect: new Set() };
  vm.runInContext('isConnected = false; esploader = null;', h.context);
  const serial = h.context.navigator.serial;
  serial.addEventListener = (name, fn) => listeners[name]?.add(fn);
  serial.removeEventListener = (name, fn) => listeners[name]?.delete(fn);
  serial.requestPort = async () => port;
  serial.getPorts = async () => [reconnectAs || port];
  if (reconnectAs) {
    const write = port.open;
    port.open = async () => {
      await write();
      const writer = port.writable.getWriter();
      port.writable.getWriter = () => ({ ...writer, write: async bytes => {
        await writer.write(bytes);
        for (const fn of listeners.disconnect) fn({ target: port });
        setTimeout(() => { for (const fn of listeners.connect) fn({ target: reconnectAs }); }, 300);
      } });
    };
  }
  h.context.Transport = class { constructor(p) { this.port = p; loaderPorts.push(p); } async disconnect() {} };
  h.context.ESPLoader = class {
    constructor(options) { this.transport = options.transport; }
    async main(mode) { modes.push(mode); if (failingModes.includes(mode)) throw new Error('no sync'); }
  };
  return { ...h, modes, loaderPorts, listeners, connect: () => h.get('btnConnect').handlers.click() };
}

test('a running receiver takes the key and its loader answers without any reset', async () => {
  const port = mockPort('C5VRX5 download_boot=requested\r\n');
  const h = connectHarness(port, []);
  await h.connect();
  assert.deepEqual(port.written, [0x60]);
  assert.deepEqual(h.modes, ['no_reset']);
  assert.equal(vm.runInContext('isConnected', h.context), true);
  assert.equal(h.alerts.length, 0);
});

test('after the key, a silent loader is never USB-reset (that hung the board)', async () => {
  const port = mockPort('C5VRX5 download_boot=requested\r\n');
  const h = connectHarness(port, ['no_reset']);
  await h.connect();
  assert.deepEqual(h.modes, ['no_reset']);
  assert.equal(vm.runInContext('isConnected', h.context), false);
  assert.match(h.alerts[0], /Unplug it, plug it back in/);
});

test('no reply to the key (BOOT held, other firmware): the USB reset, as before', async () => {
  const port = mockPort(null);
  const h = connectHarness(port, ['no_reset']);
  await h.connect();
  assert.deepEqual(port.written, [0x60]);
  assert.deepEqual(h.modes, ['no_reset', 'usb_reset']);
  assert.equal(vm.runInContext('isConnected', h.context), true);
});

test('a port that cannot take the key still gets the loader', async () => {
  const port = mockPort(null);
  port.open = async () => { throw new Error('busy'); };
  const h = connectHarness(port, ['no_reset']);
  await h.connect();
  assert.deepEqual(h.modes, ['no_reset', 'usb_reset']);
  assert.equal(vm.runInContext('isConnected', h.context), true);
});

test('after the selected port reconnects, the loader keeps using that same port', async () => {
  const port = mockPort('C5VRX5 download_boot=requested\r\n');
  const h = connectHarness(port, [], port);
  await h.connect();
  assert.deepEqual(h.modes, ['no_reset']);
  assert.equal(h.loaderPorts[0], port);
  assert.equal(h.listeners.connect.size + h.listeners.disconnect.size, 0);
  assert.equal(vm.runInContext('isConnected', h.context), true);
});

test('a replacement port requires reselection before any bootloader handshake', async () => {
  const port = mockPort('C5VRX5 download_boot=requested\r\n');
  const back = mockPort(null);
  const h = connectHarness(port, [], back);
  // Skip only the reconnect polling delay; exercise the actual click handler.
  h.context.setTimeout = (fn, ms) => setTimeout(fn, ms === 100 ? 0 : ms);
  await h.connect();
  assert.deepEqual(h.modes, []);
  assert.equal(h.loaderPorts.length, 0);
  assert.equal(h.listeners.connect.size + h.listeners.disconnect.size, 0);   // listeners removed
  assert.equal(vm.runInContext('isConnected', h.context), false);
  assert.match(h.alerts[0], /select the receiver again/);
});

for (const sameModel of [false, true]) {
  test(`unrelated USB events cannot replace the selected port (same model: ${sameModel})`, async () => {
    const port = mockPort(null);
    const other = mockPort(null);
    if (!sameModel) other.getInfo = () => ({ usbVendorId: 0x1234, usbProductId: 0x5678 });
    const h = connectHarness(port, []);
    h.context.selectedPort = port;
    const watch = vm.runInContext('watchReconnect(selectedPort)', h.context);
    for (const fn of h.listeners.disconnect) fn({ target: other });
    for (const fn of h.listeners.connect) fn({ target: other });
    assert.equal(watch.vanished, false);
    assert.equal(watch.back, null);
    h.context.reconnectWatch = watch;
    assert.equal(await vm.runInContext('portAfterRestart(selectedPort, reconnectWatch)', h.context), port);
    assert.equal(h.listeners.connect.size + h.listeners.disconnect.size, 0);
  });

  test(`a missing selected port never falls back to another permitted port (same model: ${sameModel})`, async () => {
    const port = mockPort(null);
    const other = mockPort(null);
    if (!sameModel) other.getInfo = () => ({ usbVendorId: 0x1234, usbProductId: 0x5678 });
    const h = connectHarness(port, []);
    h.context.navigator.serial.getPorts = async () => [other];
    h.context.selectedPort = port;
    const watch = vm.runInContext('watchReconnect(selectedPort)', h.context);
    for (const fn of h.listeners.connect) fn({ target: other });
    h.context.reconnectWatch = watch;
    await assert.rejects(vm.runInContext('portAfterRestart(selectedPort, reconnectWatch)', h.context), /select the receiver again/);
    assert.equal(h.loaderPorts.length, 0);
    assert.equal(h.listeners.connect.size + h.listeners.disconnect.size, 0);
  });
}

test('after flashing, the LP watchdog restarts the C5 into the new firmware, not the RTS reset', async () => {
  const h = harness('alpha');
  await h.click();
  assert.deepEqual(h.resets, [
    ['writeReg', 0x600b1c18, 0x50d83aa1], ['writeReg', 0x600b1c04, 2000], ['writeReg', 0x600b1c00, 0xd0000102]]);
});

test('a loader that refuses the watchdog writes still gets the RTS reset', async () => {
  const h = harness('alpha');
  h.context.mockLoader.writeReg = async () => { throw new Error('write failed'); };
  await h.click();
  assert.deepEqual(h.resets, [['after', 'hard_reset']]);
});

// Board images: the board and the detected chip choose the release files.
function boardHarness({ board = 'xiao', image = 'auto', chip = null, packageType = 'merged', confirmed = true, variants = true } = {}) {
  const h = harness('alpha', packageType, confirmed);
  h.get('selectBoard').value = board;
  h.get('selectImage').value = image;
  const extra = variants ? ['xiao-psram', 'xiao-4mb', 'devkitc', 'devkitc-psram', 'pico', 'nicemcu']
    .flatMap(id => [`c5vrx5-${id}.app.bin`, `c5vrx5-${id}.full.bin`]) : [];
  vm.runInContext(`chipInfo = ${JSON.stringify(chip)};
    githubAlphaBuilds[0].assets.push(...${JSON.stringify(extra)}.map(name => ({ name, local_url: 'firmware/alpha/' + name, size: 4 })));`, h.context);
  return h;
}
const flashedFile = h => h.downloads.map(url => url.split('/').pop());

test('auto flashes the XIAO PSRAM image on a chip with in-package PSRAM', async () => {
  const h = boardHarness({ chip: { name: 'ESP32-C5HR8', flashMb: 8, psramMb: 8 } });
  await h.click();
  assert.deepEqual(flashedFile(h), ['c5vrx5-xiao-psram.full.bin']);
  assert.equal(h.flashes[0].fileArray[0].address, 0);
  assert.equal(h.flashes[0].flashSize, 'keep');
});
test('auto flashes the standard image on a XIAO-wired chip without PSRAM', async () => {
  const h = boardHarness({ chip: { name: 'ESP32-C5N', flashMb: 8, psramMb: 0 } });
  await h.click();
  assert.deepEqual(flashedFile(h), ['c5vrx5_merged.bin']);
});
test('a 4 MB chip gets the 4 MB layout, full and app only', async () => {
  const full = boardHarness({ board: 'devkitc', chip: { name: 'ESP32-C5N', flashMb: 4, psramMb: 0 } });
  await full.click();
  assert.deepEqual(flashedFile(full), ['c5vrx5-devkitc.full.bin']);
  const app = boardHarness({ board: 'devkitc', packageType: 'app_only', chip: { name: 'ESP32-C5N', flashMb: 4, psramMb: 0 } });
  await app.click();
  assert.deepEqual(flashedFile(app), ['c5vrx5-devkitc.app.bin']);
  assert.equal(app.flashes[0].fileArray[0].address, 0x10000);
});
test('an 8 MB image is refused on a 4 MB chip before anything is downloaded', async () => {
  const h = boardHarness({ image: 'xiao', chip: { name: 'ESP32-C5HF4', flashMb: 4, psramMb: 0 } });
  await h.click();
  assert.equal(h.downloads.length, 0);
  assert.equal(h.flashes.length, 0);
  assert.match(h.alerts[0], /needs 8 MB of flash/);
});
test('a 4 MB chip with an older build that has only the 8 MB image flashes nothing', async () => {
  const h = boardHarness({ chip: { name: 'ESP32-C5HF4', flashMb: 4, psramMb: 0 }, variants: false });
  await h.click();
  assert.equal(h.flashes.length, 0);
  assert.match(h.alerts[0], /no image for this board/);
});
test('a PSRAM image on a chip without in-package PSRAM needs a confirmation', async () => {
  const chip = { name: 'ESP32-C5N', flashMb: 8, psramMb: 0 };
  const declined = boardHarness({ board: 'devkitc', image: 'devkitc-psram', chip, confirmed: false });
  await declined.click();
  assert.equal(declined.flashes.length, 0);
  const accepted = boardHarness({ board: 'devkitc', image: 'devkitc-psram', chip });
  await accepted.click();
  assert.deepEqual(flashedFile(accepted), ['c5vrx5-devkitc-psram.full.bin']);
});
test('legacy builds keep the 8 MB flash size setting', async () => {
  const h = harness('github');
  await h.click();
  assert.equal(h.flashes[0].flashSize, '8MB');
});

// Pin editor: edited pins are written into the image's pin table.
async function pinHarness({ board = 'xiao', pins = null, packageType = 'merged', tables = 1, noMerged = false } = {}) {
  const h = boardHarness({ board, packageType, chip: { name: 'ESP32-C5HR8', flashMb: 8, psramMb: 0 } });
  const app = await fakeAppImage({ tables });
  h.files['c5vrx5.bin'] = app;
  h.files['c5vrx5_merged.bin'] = mergedImage(app);
  vm.runInContext(`for (const a of githubAlphaBuilds[0].assets) a.size = 0;
    ${noMerged ? "githubAlphaBuilds[0].assets = githubAlphaBuilds[0].assets.filter(a => !a.name.includes('merged'));" : ''}
    editedPins = ${JSON.stringify(pins)};`, h.context);
  return h;
}
const DEVKITC_PINS = { dac: [23, 24, 6, 10, 8, 9], iq: [1, 0, 25, 2, 26, 5, 3, 4], boot: 28 };
const flashedPins = (h, part = 0, offset = 0x10000) => imagePinsModule.imagePins(h.flashes[0].fileArray[part].data, offset);

test('edited pins go into the merged image at 0x10000, which still verifies', async () => {
  const h = await pinHarness({ pins: DEVKITC_PINS });
  await h.click();
  assert.equal(h.flashes.length, 1, h.alerts.join());
  assert.deepEqual(flashedPins(h), { ...DEVKITC_PINS, custom: true });
  assert.deepEqual(await imagePinsModule.verifyAppImage(h.flashes[0].fileArray[0].data, 0x10000), { checksumOk: true, hashOk: true });
});
test('edited pins go into an app-only image at offset 0', async () => {
  const h = await pinHarness({ pins: DEVKITC_PINS, packageType: 'app_only' });
  await h.click();
  assert.equal(h.flashes[0].fileArray[0].address, 0x10000);
  assert.deepEqual(flashedPins(h, 0, 0).dac, DEVKITC_PINS.dac);
});
test('edited pins reach the application of a three-part flash', async () => {
  const h = await pinHarness({ pins: DEVKITC_PINS, noMerged: true });
  await h.click();
  const app = h.flashes[0].fileArray.find(f => f.address === 0x10000);
  assert.deepEqual(imagePinsModule.imagePins(app.data, 0).iq, DEVKITC_PINS.iq);
});
test('the board\'s own pins flash the image unchanged', async () => {
  const h = await pinHarness({ pins: null });
  await h.click();
  assert.deepEqual(flashedPins(h), { ...XIAO_PINS, custom: false });
});
test('the custom board always writes its pins, even the XIAO map', async () => {
  const h = await pinHarness({ board: 'custom', pins: null });
  await h.click();
  assert.deepEqual(flashedPins(h), { ...XIAO_PINS, custom: true });
});
test('pins the rules reject never reach a download', async () => {
  const h = await pinHarness({ pins: { ...DEVKITC_PINS, dac: [23, 24, 6, 10, 8, 13] } });
  await h.click();
  assert.equal(h.downloads.length, 0);
  assert.equal(h.flashes.length, 0);
  assert.match(h.alerts[0], /USB/);
});
test('a build older than the pin table refuses edited pins and flashes nothing', async () => {
  const h = await pinHarness({ pins: DEVKITC_PINS, tables: 0 });
  await h.click();
  assert.equal(h.flashes.length, 0);
  assert.match(h.alerts[0], /older than the pin editor/);
});
test('the wiring pictures follow the edited pins', async () => {
  // D6 moves from D6/GPIO11 to the XIAO's D10/GPIO10.
  const h = await pinHarness({ pins: { ...XIAO_PINS, dac: [23, 24, 10, 12, 8, 9] } });
  vm.runInContext('renderBoardInfo()', h.context);
  const picture = h.get('boardDiagram').innerHTML;
  assert.match(picture, /GPIO10<\/text><line class="bd-lead bd-lead-dac"[^>]*\/><rect class="bd-chip bd-chip-dac"[^>]*\/><text[^>]*>D6 · 2 kΩ</);
  assert.match(picture, /GPIO11<\/text><circle/, 'GPIO11 is free now');
  assert.match(picture, /Inside the board, nothing to wire: GPIO2, GPIO27, GPIO5, GPIO3, GPIO4, GPIO28 \(BOOT button\)/);
  assert.match(h.get('dacDiagram').innerHTML, /GPIO10<\/text><text class="bd-silk"[^>]*>D6</);
  assert.match(h.get('pinStatus').textContent, /written into the image/);
});
test('a DAC pin on no header is shown as unwirable and warned about', async () => {
  const h = await pinHarness({ pins: DEVKITC_PINS });   // GPIO6 has no XIAO header pin
  vm.runInContext('renderBoardInfo()', h.context);
  assert.match(h.get('boardDiagram').innerHTML, /Not on a header, cannot be wired: D6 on GPIO6/);
  assert.match(h.get('pinStatus').textContent, /D6 on GPIO6: not on this board's headers/);
});
test('a board without a layout gets the GPIO grid with every pin job', async () => {
  const h = await pinHarness({ board: 'custom' });
  vm.runInContext('renderBoardInfo()', h.context);
  const grid = h.get('boardDiagram').innerHTML;
  assert.match(grid, /GPIO23<\/text><text class="bd-cell-text bd-cell-text-dac"[^>]*>D4</);
  assert.match(grid, /GPIO13<\/text><text class="bd-cell-text bd-cell-text-reserved"[^>]*>USB D-</);
  assert.match(grid, /GPIO27<\/text><text class="bd-cell-text bd-cell-text-iq"[^>]*>IQ I4</);
});
test('the summary names the image, the chip and images a build lacks', async () => {
  const before = boardHarness({ variants: false });
  vm.runInContext('renderBoardInfo()', before.context);
  const old = before.get('boardInfo').innerHTML;
  assert.match(old, /Not in this build: 8 MB flash \+ PSRAM\./);
  assert.match(old, /Connect to read its flash and PSRAM/);
  const after = boardHarness({ chip: { name: 'ESP32-C5HR8', flashMb: 8, psramMb: 8 } });
  vm.runInContext('renderBoardInfo()', after.context);
  const now = after.get('boardInfo').innerHTML;
  assert.match(now, /8 MB flash \+ PSRAM \(Auto, from the chip\)/);
  assert.match(now, /ESP32-C5HR8 · 8 MB flash · 8 MB PSRAM/);
  assert.match(now, /Tested on hardware · <a href="https:\/\/files\.seeedstudio\.com/);
  assert.doesNotMatch(now, /Not in this build/);
});
