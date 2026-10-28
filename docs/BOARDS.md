# Boards

C5VRX-5 is one firmware with a pin map per board, chosen when it is built.
The web flasher has a **Board** list for the pins and a **Flash and PSRAM**
list for the memory layout. On **Auto** the flasher reads the connected chip
and picks the image itself. Under the lists it draws the selected board with
its headers in their real order: which pin gets which DAC resistor, which
pins to leave free, the power and GND pins, and the resistor network as a
small schematic with the GPIO of every branch. **Custom pins** moves any
of them (see [Pin editor](#pin-editor)); the pictures follow.

Only the Seeed XIAO ESP32-C5 has been tested on hardware. The other maps
come from vendor documentation and schematics; please report how they work.

## Pin maps

DAC branches D4..D9 are the six resistors of the [DAC](hardware/c5vrx-dac-schematic.png)
(8.2k, 3.9k, 2k, 1k, 470R, 240R into one node with a 200R load), LSB first.
The names stay D4..D9 on every board, also on the DAC TEST page; the table
gives the GPIO each one goes to.

| Board | D4 | D5 | D6 | D7 | D8 | D9 | Menu button | Leave unconnected (IQ loopback) | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Seeed XIAO ESP32-C5 | 23 | 24 | 11 | 12 | 8 | 9 | 28 (BOOT) | 1, 0, 25, 2, 27, 5, 3, 4 | board tested |
| Espressif ESP32-C5-DevKitC-1 (v1.1, v1.2) | 23 | 24 | 6 | 10 | 8 | 9 | 28 (BOOT) | 1, 0, 25, 2, 26, 5, 3, 4 | not tested |
| Waveshare ESP32-C5-Pico | 10 | 15 | 11 | 12 | 8 | 9 | 28 (BOOT) | 1, 0, 25, 2, 26, 5, 3, 4 | not tested |
| NiceMCU ESP32-C5 Mini | 4 | 5 | 6 | 7 | 8 | 9 | 28 (BOOT) | 1, 0, 25, 10, 27, 26, 3, 24 | not tested |

The IQ loopback pins carry the radio's samples from the modem to the
receiver at 40 MHz inside the chip (GPIO matrix). They need no wiring, but
nothing may be connected to them. The reserved seventh and eighth DAC
branches (XIAO D10/GPIO10 and D3/GPIO7, README) are XIAO plans; another
board would need its own free pins for them.

Why the maps differ:

- **DevKitC-1:** GPIO11/12 are UART0, wired to the board's USB-UART bridge,
  and GPIO27 drives the RGB LED. D6/D7 move to GPIO6/10 and the I-LSB
  loopback to GPIO26. GPIO15 stays free: modules with SPI PSRAM (N8R4) use
  it as the PSRAM chip select. Plug the receiver in through the port marked
  **USB** (native USB); the console does not run on the UART port.
- **Waveshare ESP32-C5-Pico:** GPIO23/24 are the I2C bus of its TCA9554
  expander (10k pull-ups), GPIO6 the battery divider, GPIO27 the WS2812 LED
  ([schematic](https://github.com/waveshareteam/ESP32-C5-Pico/tree/main/schematic)).
  GPIO0/1 reach the header pins marked UART TXD/RXD through 0R resistors.
- **NiceMCU ESP32-C5 Mini:** ESP32-C5HF4 with 4 MB flash in the package.
  GPIO23 is the user LED. GPIO24..27 reach no header, so four IQ loopback
  bits use them and header IO15 stays free. The headers carry IO0..IO10,
  IO15 and TX/RX; the schematic shows TX/RX as GPIO11/12 (the vendor
  pinout image labels TX as IO14, which is USB D+). Map from the board
  schematic, first proposed in PR #197.

The DAC uses no boot-mode strapping pin (GPIO26..28) on any board. GPIO7
(NiceMCU D7) and MTMS/MTDI are strapping pins too, but GPIO7 only selects
the JTAG source when `EFUSE_JTAG_SEL_ENABLE` is burnt (default off), and
MTMS/MTDI only set SDIO edges (ESP32-C5 datasheet, Boot Configurations).

The flasher links each board's pinout or schematic: [XIAO](https://files.seeedstudio.com/wiki/XIAO_ESP32C5/Getting_started/XIAO_ESP32-C5_front_pinout.png),
[DevKitC-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp32-c5-devkitc-1/user_guide.html#header-block),
[Waveshare Pico](https://github.com/waveshareteam/ESP32-C5-Pico/blob/main/schematic/ESP32-C5-Pico.pdf),
[NiceMCU Mini](https://www.cnx-software.com/2026/04/28/esp32-c5-mini-usb-c-board-offers-dual-band-wifi-6-and-up-to-14x-gpio-pins-for-iot-projects/).
Each map above was checked against these on 2026-10-28.

Any other ESP32-C5 board (for example the DFRobot FireBeetle 2 ESP32-C5 or
the Waveshare ESP32-C5-MINI-KIT, whose pinouts are published only as
images) is **Other ESP32-C5 board (enter its pins)**: the flasher starts
from the XIAO map, shows every GPIO's job on a grid, and writes the pins
you enter into the image.

## Images

| Image | Board | Flash | PSRAM | Release files |
| --- | --- | --- | --- | --- |
| xiao | XIAO, other boards | 8 MB | no | `c5vrx5.bin`, `c5vrx5_merged.bin` |
| xiao-psram | XIAO, other boards | 8 MB | yes | `c5vrx5-xiao-psram.app.bin`, `.full.bin` |
| xiao-4mb | other boards | 4 MB | no | `c5vrx5-xiao-4mb.app.bin`, `.full.bin` |
| devkitc | DevKitC-1 | 4 MB | no | `c5vrx5-devkitc.app.bin`, `.full.bin` |
| devkitc-psram | DevKitC-1 | 8 MB | yes | `c5vrx5-devkitc-psram.app.bin`, `.full.bin` |
| pico | Waveshare Pico | 4 MB | no | `c5vrx5-pico.app.bin`, `.full.bin` |
| nicemcu | NiceMCU Mini | 4 MB | no | `c5vrx5-nicemcu.app.bin`, `.full.bin` |

The `.full.bin` file is the merged image for address 0x0 (bootloader,
partition table and application); `.app.bin` is the application for
0x10000. A 4 MB image also runs on an 8 MB or larger chip: ESP-IDF uses the
size in the image header. An 8 MB image stops at startup on a 4 MB chip, so
the flasher refuses it there.

The 4 MB layout (`partitions_4mb.csv`) keeps the 1.5 MB flight log and gives
the application 2 MB instead of 3 MB (the application is about 1.2 MB).

### PSRAM

A PSRAM image moves Wi-Fi buffers out of internal RAM (boot `heap_min` about
2.1 KB to 21.9 KB on the XIAO, [FINDINGS](FINDINGS.md#psram-8-mb-on-the-xiao-now-mapped-board-2026-10-26))
and panics at boot on a board without PSRAM. The standard images do not use
PSRAM. One image for both is not possible without a cost: enabling PSRAM
support adds code to internal RAM, and the standard image has only about
2 KB of boot heap to spare.

On **Auto** the flasher picks the PSRAM image when the chip reports PSRAM in
its package (eFuse, for example `ESP32-C5HR8` on the XIAO). PSRAM on a
module next to the chip (for example ESP32-C5-WROOM-1 N8R4 on a DevKitC-1)
is not in the eFuse: choose the PSRAM image by hand, and the flasher asks
before writing it.

## Pin editor

**Custom pins** has a list of GPIOs (named with the board's header labels) for the six DAC branches, the eight
IQ loopback bits and the menu button. When they differ from the board's own
pins (always for **Other ESP32-C5 board**), the flasher writes them into the
image while flashing:

- The firmware keeps its pins in one 32-byte table in the application
  image (`board_pins_t` in `main/board.h`, `g_board_pins` in `main/board.c`):
  the magic `C5VRXPN1`, D4..D9, IQ bits 0..7, the button, a custom flag and
  8 reserved bytes. It reads the table once at boot (`volatile`, so no pin
  is folded into the code) and uses it for the DAC (PARLIO TX), the IQ
  loopback (MODEM_DIAG routing and PARLIO RX) and the button.
- The flasher (`web/image-pins.js`) finds the one table by its magic in the
  image's data segments, writes the pins, sets the custom flag, and
  recomputes the image checksum and the appended SHA-256 that the
  bootloader checks. It verifies the result before flashing and refuses an
  image with no table (builds older than the pin editor) or more than one.
  Full firmware and App only both work; the pins are saved in the browser.
- Both sides apply the same rules (`board_pins_problem` in `main/board.h`,
  `pinProblem` in `web/boards.js`, shared cases in
  `test/data/board_pin_cases.txt`): every pin is GPIO 0..28, not GPIO13/14
  (USB), not GPIO16..22 (flash/PSRAM), no GPIO twice, GPIO28 only as the
  menu button (it is the boot-mode strap and the BOOT button), and no
  GPIO15 with a PSRAM image. A table that breaks them (the flasher never
  writes one) makes the firmware fall back to its compiled map.
- The flasher also warns about a DAC pin on a strapping pin, on a pin with
  an onboard part (LED, pull-up, USB-UART bridge) or on no header, and about
  an IQ pin on a pin with an onboard part.

`T` prints a BOARD line with the pins in use, for example
`BOARD id=xiao pins=custom dac=23,24,10,12,8,9 iq=1,0,25,2,27,5,3,4 boot=28`.

Checked so far (2026-10-28): esptool `image_info` reports the checksum and
the validation hash of a real XIAO image as valid after the flasher's code
wrote other pins into it (exactly 35 bytes change: the two table bytes, the
checksum and the 32-byte hash).

## Building one image

```sh
idf.py -B build-devkitc -DSDKCONFIG=sdkconfig.devkitc \
  "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.flash4mb.defaults;sdkconfig.board-devkitc.defaults" build
python tools/flash.py PORT --build build-devkitc --check
```

The overlays: `sdkconfig.flash4mb.defaults` (4 MB layout),
`sdkconfig.psram.defaults` (PSRAM) and `sdkconfig.board-<board>.defaults`
(pin map). `.github/workflows/c5vrx5.yml` lists the exact chain for each
image. `idf.py menuconfig` also offers the pin map under **C5VRX-5 board**.

## Adding a board

1. Add its pins to `main/board.h`, a Kconfig option to
   `main/Kconfig.projbuild` and `sdkconfig.board-<board>.defaults`.
2. Add it with its images to `web/boards.js` and to the `boards` matrix in
   `.github/workflows/c5vrx5.yml`.
3. Add its headers as a layout (each pin with its GPIO and printed label,
   top to bottom, from the vendor pinout) and its onboard parts to
   `web/boards.js`, so the flasher can draw it.
4. Add a row to the tables above.

`test/test_boards.py` fails until the pins, images, overlays, layouts and
partition tables agree. The FIRMWARE line of `T` names the board (`board=`)
and whether the image uses PSRAM (`psram=`); the BOARD line gives the pins.
