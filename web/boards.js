// Boards and firmware images for the C5VRX-5 flasher. The pins repeat
// main/board.h, the images the CI build matrix (.github/workflows/c5vrx5.yml);
// test/test_boards.py checks all three. The array is plain JSON.
//
// dac, iq, boot: the board's own pins (the image's compiled map).
// parts: GPIOs an onboard part uses. layouts: the headers as seen from the
// top, each pin [GPIO or null, printed label], top to bottom, from the
// vendor pinout or schematic (pinout, linked, not copied); internal: GPIOs
// on no header. image.asset: 'standard' is the default XIAO release files
// (c5vrx5.bin, c5vrx5_merged.bin); any other id is c5vrx5-<id>.app.bin and
// c5vrx5-<id>.full.bin. image.pins: the compiled map it carries. psram: the
// image needs PSRAM and will not boot without it. custom: the user enters
// the pins, which the flasher writes into the image (web/image-pins.js).
export const BOARDS = [
  {
    "id": "xiao",
    "name": "Seeed XIAO ESP32-C5",
    "pinout": "https://files.seeedstudio.com/wiki/XIAO_ESP32C5/Getting_started/XIAO_ESP32-C5_front_pinout.png",
    "tested": true,
    "dac": [23, 24, 11, 12, 8, 9],
    "iq": [1, 0, 25, 2, 27, 5, 3, 4],
    "boot": 28,
    "parts": { "6": "battery divider", "26": "battery divider", "27": "user LED", "28": "BOOT button" },
    "note": "",
    "layouts": [
      {
        "id": "top", "name": "Top view", "top": "USB-C", "bottom": "RESET · antenna · BOOT",
        "left": [[1, "D0"], [0, "D1"], [25, "D2"], [7, "D3"], [23, "D4"], [24, "D5"], [11, "D6"]],
        "right": [[null, "5V"], [null, "GND"], [null, "3V3"], [10, "D10"], [9, "D9"], [8, "D8"], [12, "D7"]],
        "internal": [2, 3, 4, 5, 6, 26, 27, 28]
      }
    ],
    "images": [
      { "id": "xiao", "asset": "standard", "flashMb": 8, "psram": false, "pins": "xiao" },
      { "id": "xiao-psram", "asset": "xiao-psram", "flashMb": 8, "psram": true, "pins": "xiao" }
    ]
  },
  {
    "id": "devkitc",
    "name": "Espressif ESP32-C5-DevKitC-1",
    "pinout": "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp32-c5-devkitc-1/user_guide.html#header-block",
    "tested": false,
    "dac": [23, 24, 6, 10, 8, 9],
    "iq": [1, 0, 25, 2, 26, 5, 3, 4],
    "boot": 28,
    "parts": { "11": "USB-UART bridge (TX)", "12": "USB-UART bridge (RX)", "27": "RGB LED", "28": "BOOT button" },
    "note": "Use the USB port marked USB, not UART: the receiver's console is on native USB.",
    "layouts": [
      {
        "id": "v1.2", "name": "v1.2", "top": "antenna", "bottom": "UART · USB",
        "left": [[null, "3V3"], [null, "RST"], [2, "2"], [3, "3"], [0, "0"], [1, "1"], [6, "6"], [7, "7"], [8, "8"], [9, "9"], [10, "10"], [26, "26"], [25, "25"], [null, "5V"], [null, "G"], [null, "NC"]],
        "right": [[null, "G"], [11, "TX"], [12, "RX"], [24, "24"], [23, "23"], [15, "NC/15"], [27, "27"], [4, "4"], [5, "5"], [null, "NC"], [28, "28"], [null, "G"], [14, "14"], [13, "13"], [null, "G"], [null, "NC"]],
        "internal": []
      },
      {
        "id": "v1.1", "name": "v1.1", "top": "antenna", "bottom": "UART · USB",
        "left": [[null, "3V3"], [null, "RST"], [2, "2"], [3, "3"], [4, "4"], [5, "5"], [0, "0"], [1, "1"], [27, "27"], [6, "6"], [7, "7"], [26, "26"], [25, "25"], [null, "5V"], [null, "G"], [null, "NC"]],
        "right": [[null, "G"], [11, "TX"], [12, "RX"], [24, "24"], [23, "23"], [15, "NC/15"], [10, "10"], [9, "9"], [8, "8"], [null, "NC"], [28, "28"], [null, "G"], [14, "14"], [13, "13"], [null, "G"], [null, "NC"]],
        "internal": []
      }
    ],
    "images": [
      { "id": "devkitc", "asset": "devkitc", "flashMb": 4, "psram": false, "pins": "devkitc" },
      { "id": "devkitc-psram", "asset": "devkitc-psram", "flashMb": 8, "psram": true, "pins": "devkitc" }
    ]
  },
  {
    "id": "pico",
    "name": "Waveshare ESP32-C5-Pico",
    "pinout": "https://github.com/waveshareteam/ESP32-C5-Pico/blob/main/schematic/ESP32-C5-Pico.pdf",
    "tested": false,
    "dac": [10, 15, 11, 12, 8, 9],
    "iq": [1, 0, 25, 2, 26, 5, 3, 4],
    "boot": 28,
    "parts": { "6": "battery divider", "23": "I2C SDA (TCA9554, 10k pull-up)", "24": "I2C SCL (TCA9554, 10k pull-up)", "27": "WS2812 LED", "28": "BOOT button" },
    "note": "GPIO0 and GPIO1 are the header pins marked UART TXD and RXD. The labels are the Pico pin numbers.",
    "layouts": [
      {
        "id": "top", "name": "Top view", "top": "USB-C", "bottom": "",
        "left": [[0, "1"], [1, "2"], [null, "GND"], [2, "4"], [3, "5"], [4, "6"], [5, "7"], [null, "GND"], [6, "9"], [7, "10"], [8, "11"], [9, "12"], [null, "GND"], [10, "14"], [11, "15"], [12, "16"], [15, "17"], [null, "GND"], [null, "EXT1"], [null, "EXT2"]],
        "right": [[null, "VBUS"], [null, "VSYS"], [null, "GND"], [null, "3V3EN"], [null, "3V3"], [null, "NC"], [28, "34"], [null, "GND"], [27, "32"], [26, "31"], [null, "RUN"], [25, "29"], [null, "GND"], [24, "27"], [23, "26"], [null, "EXT3"], [null, "EXT4"], [null, "GND"], [null, "EXT5"], [null, "EXT6"]],
        "internal": []
      }
    ],
    "images": [
      { "id": "pico", "asset": "pico", "flashMb": 4, "psram": false, "pins": "pico" }
    ]
  },
  {
    "id": "nicemcu",
    "name": "NiceMCU ESP32-C5 Mini",
    "pinout": "https://www.cnx-software.com/2026/04/28/esp32-c5-mini-usb-c-board-offers-dual-band-wifi-6-and-up-to-14x-gpio-pins-for-iot-projects/",
    "tested": false,
    "dac": [4, 5, 6, 7, 8, 9],
    "iq": [1, 0, 25, 10, 27, 26, 3, 24],
    "boot": 28,
    "parts": { "23": "user LED", "28": "BOOT button" },
    "note": "The header pins marked TX and RX are GPIO11 and GPIO12 (the vendor pinout labels TX as IO14; the schematic shows GPIO11).",
    "layouts": [
      {
        "id": "top", "name": "Top view", "top": "antenna", "bottom": "RST · BOOT · USB-C",
        "left": [[0, "IO0"], [1, "IO1"], [2, "IO2"], [3, "IO3"], [4, "IO4"], [null, "RST"], [null, "3V3"], [null, "GND"], [null, "5V"]],
        "right": [[15, "IO15"], [11, "TX"], [12, "RX"], [10, "IO10"], [9, "IO9"], [8, "IO8"], [7, "IO7"], [6, "IO6"], [5, "IO5"]],
        "internal": [23, 24, 25, 26, 27, 28]
      }
    ],
    "images": [
      { "id": "nicemcu", "asset": "nicemcu", "flashMb": 4, "psram": false, "pins": "nicemcu" }
    ]
  },
  {
    "id": "custom",
    "name": "Other ESP32-C5 board (enter its pins)",
    "pinout": "https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/api-reference/peripherals/gpio.html",
    "tested": false,
    "custom": true,
    "dac": [23, 24, 11, 12, 8, 9],
    "iq": [1, 0, 25, 2, 27, 5, 3, 4],
    "boot": 28,
    "parts": {},
    "note": "Enter its GPIO numbers under Custom pins. Check the schematic: nothing may be wired to the IQ pins, and a pin with an LED or a pull-up is a poor DAC pin.",
    "layouts": [],
    "images": [
      { "id": "xiao", "asset": "standard", "flashMb": 8, "psram": false, "pins": "xiao" },
      { "id": "xiao-psram", "asset": "xiao-psram", "flashMb": 8, "psram": true, "pins": "xiao" },
      { "id": "xiao-4mb", "asset": "xiao-4mb", "flashMb": 4, "psram": false, "pins": "xiao" }
    ]
  }
];

// The DAC network (main/dac_check.h): branch D4..D9, LSB first, into one
// node with a 200 ohm load to GND.
export const DAC_OHMS = [8200, 3900, 2000, 1000, 470, 240];
export const DAC_LOAD_OHMS = 200;
// PARLIO RX / MODEM_DIAG bit order (main/board.h).
export const IQ_NAMES = ['Q4', 'Q5', 'Q6', 'Q9', 'I4', 'I5', 'I6', 'I9'];
// ESP32-C5 strapping pins (datasheet, Boot Configurations).
export const STRAPPING = [2, 3, 7, 25, 26, 27, 28];

const MB = 1024 * 1024;

export function findBoard(id) {
  return BOARDS.find(board => board.id === id) || BOARDS[0];
}

export function ohms(value) {
  return value >= 1000 ? `${String(value / 1000)} kΩ` : `${value} Ω`;
}

export function imageLabel(image) {
  return `${image.flashMb} MB flash${image.psram ? ' + PSRAM' : ''}`;
}

export function presetPins(board) {
  return { dac: [...board.dac], iq: [...board.iq], boot: board.boot };
}

export function samePins(a, b) {
  return a.boot === b.boot && a.dac.every((p, k) => p === b.dac[k]) && a.iq.every((p, k) => p === b.iq[k]);
}

// Why a pin map cannot be used, or null: the rules of main/board.h
// board_pins_problem(), in the same order (test/data/board_pin_cases.txt).
export function pinProblem(pins, psram) {
  const all = [...pins.dac, ...pins.iq, pins.boot];
  if (all.length !== 15 || all.some(p => !Number.isInteger(p) || p < 0 || p > 28)) return 'range';
  if (all.some(p => p === 13 || p === 14)) return 'usb';
  if (all.some(p => p >= 16 && p <= 22)) return 'flash';
  if (new Set(all).size !== all.length) return 'duplicate';
  if (all.slice(0, 14).includes(28)) return 'boot_strap';
  if (psram && all.includes(15)) return 'psram_cs';
  return null;
}

export const PIN_PROBLEM_TEXT = {
  range: 'Every pin must be a GPIO number from 0 to 28.',
  usb: 'GPIO13 and GPIO14 carry USB, which the console and the flasher use.',
  flash: 'GPIO16 to GPIO22 are the flash and PSRAM bus.',
  duplicate: 'A GPIO is used for two jobs.',
  boot_strap: 'GPIO28 is the boot-mode pin and the BOOT button: only the menu button may use it.',
  psram_cs: 'With PSRAM, GPIO15 can be the PSRAM chip select on modules: choose another pin or an image without PSRAM.'
};

// Things worth a second look that do not stop the flash.
export function pinWarnings(pins, board, layout) {
  const warnings = [];
  const onHeader = new Set(layout ? [...layout.left, ...layout.right].map(([gpio]) => gpio).filter(g => g !== null) : []);
  pins.dac.forEach((p, k) => {
    if (board.parts[p]) warnings.push(`D${4 + k} on GPIO${p}: the board's ${board.parts[p]} is on that pin.`);
    if (STRAPPING.includes(p)) warnings.push(`D${4 + k} on GPIO${p}, a strapping pin: check that the board still boots with the DAC connected.`);
    if (layout && !onHeader.has(p)) warnings.push(`D${4 + k} on GPIO${p}: not on this board's headers.`);
  });
  pins.iq.forEach((p, k) => {
    // The board's own map already uses some loaded pins (the XIAO user LED).
    if (board.parts[p] && !board.iq.includes(p))
      warnings.push(`IQ ${IQ_NAMES[k]} on GPIO${p}: the board's ${board.parts[p]} is on that pin, and the receiver drives it at 40 MHz.`);
  });
  if (layout && !onHeader.has(pins.boot) && !board.parts[pins.boot]) warnings.push(`Menu button on GPIO${pins.boot}: not on this board's headers.`);
  return warnings;
}

// Release files of an image, or null when the release lacks it. The
// standard image only needs its application: older releases may have no
// merged file, and the flasher then writes the three parts.
export function imageAssets(image, assets) {
  if (image.asset === 'standard') {
    const app = assets.find(a => /^c5vrx(?:[345])?\.bin$/i.test(a.name || ''));
    return app ? { app, full: assets.find(a => a.name === 'c5vrx5_merged.bin') || null } : null;
  }
  const app = assets.find(a => a.name === `c5vrx5-${image.asset}.app.bin`);
  const full = assets.find(a => a.name === `c5vrx5-${image.asset}.full.bin`);
  return app && full ? { app, full } : null;
}

// ESP32-C5 eFuse BLOCK1 word 2 (esptool targets/esp32c5.py): temperature
// grade, in-package flash and in-package PSRAM. External PSRAM on a module
// (for example WROOM-1 N8R4) is not in the eFuse: psramMb reads 0.
export function decodeC5Package(word) {
  const temp = (word >>> 24) & 3, flashCap = (word >>> 13) & 7, psramCap = (word >>> 19) & 7;
  const psramMb = { 0: 0, 1: 4, 2: 8 }[psramCap] ?? null;
  const name = 'ESP32-C5' + ({ 0: 'N', 1: 'H' }[temp] ?? '?') + ({ 0: '', 1: 'F4' }[flashCap] ?? 'F?') +
    ({ 0: '', 1: 'R4', 2: 'R8' }[psramCap] ?? 'R?');
  return { name, psramMb };
}

// JEDEC flash ID: the third byte is log2 of the size in bytes.
export function flashMbFromId(id) {
  const code = (id >>> 16) & 0xff;
  return code >= 0x12 && code <= 0x1a ? (1 << code) / MB : null;
}

// What the flasher learns from a connected chip; every field may stay null.
export async function detectChip(loader) {
  const chip = { name: null, flashMb: null, psramMb: null };
  try {
    if (loader.chip?.CHIP_NAME === 'ESP32-C5' && loader.chip.EFUSE_BLOCK1_ADDR) {
      Object.assign(chip, decodeC5Package(await loader.readReg(loader.chip.EFUSE_BLOCK1_ADDR + 8)));
    }
  } catch { /* unknown: the flasher asks before risky images */ }
  try {
    chip.flashMb = flashMbFromId(await loader.readFlashId());
  } catch { /* unknown */ }
  return chip;
}

// Auto: the PSRAM image when the chip reports PSRAM, the largest flash
// layout that fits, and only images this release contains.
export function autoImage(board, chip, available = () => true) {
  const fits = board.images.filter(image => available(image) &&
    (chip?.flashMb == null || image.flashMb <= chip.flashMb) &&
    (!image.psram || chip?.psramMb > 0));
  const score = image => (image.psram ? 100 : 0) + image.flashMb;
  return fits.sort((a, b) => score(b) - score(a))[0] || null;
}

// Why an image must not go on this chip ('block'), or should not without a
// second thought ('ask'); null when nothing is known against it.
export function imageProblem(image, chip) {
  if (chip?.flashMb != null && image.flashMb > chip.flashMb) {
    return { level: 'block', text: `This image needs ${image.flashMb} MB of flash, but the board has ${chip.flashMb} MB. Choose the ${chip.flashMb} MB image.` };
  }
  if (image.psram && !(chip?.psramMb > 0)) {
    return { level: 'ask', text: 'The chip reports no PSRAM in its package. The PSRAM image does not boot without PSRAM. Flash it only if your module has PSRAM (for example a DevKitC-1 N8R4).' };
  }
  if (chip?.flashMb == null) {
    return { level: 'ask', text: `The flash size could not be read. This image needs at least ${image.flashMb} MB of flash.` };
  }
  return null;
}

// The choices of one pin field ('dac', 'iq' or 'boot'): every GPIO with its
// header label and onboard part on this board. USB and flash pins are shown
// but cannot be chosen, and GPIO28 only for the menu button.
export function pinChoices(board, layout, group) {
  const header = new Map(layout ? [...layout.left, ...layout.right].filter(([gpio]) => gpio !== null) : []);
  return Array.from({ length: 29 }, (_, gpio) => {
    const notes = [];
    const silk = header.get(gpio);
    if (silk !== undefined && silk !== String(gpio)) notes.push(/^\d+$/.test(silk) ? `pin ${silk}` : silk);
    if (board.parts[gpio]) notes.push(board.parts[gpio]);
    let disabled = false;
    if (gpio === 13 || gpio === 14) { notes.push('USB'); disabled = true; }
    else if (gpio >= 16 && gpio <= 22) { notes.push('flash'); disabled = true; }
    else if (gpio === 28 && group !== 'boot') { if (!board.parts[gpio]) notes.push('boot strap'); disabled = true; }
    else {
      if (group === 'dac' && STRAPPING.includes(gpio)) notes.push('strapping pin');
      // IQ pins need no wire, so only the DAC and the button care about headers.
      if (layout && silk === undefined && group !== 'iq') notes.push('not on a header');
    }
    return { value: gpio, label: [`GPIO${gpio}`, ...notes].join(' · '), disabled };
  });
}
