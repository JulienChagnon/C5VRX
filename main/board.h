/* C5VRX-5 by Twotoz and the C5VRX contributors: board pin maps.
 *
 * One firmware, one pin map per board, chosen at build time (Kconfig
 * "C5VRX-5 board"; CI builds every board, docs/BOARDS.md). Per board:
 *   DAC   six resistor branches D4..D9 (8.2k/3.9k/2k/1k/470R/240R), LSB first;
 *   IQ    MODEM_DIAG / PARLIO RX bits 0..7 = Q[4] Q[5] Q[6] Q[9] I[4] I[5] I[6] I[9],
 *         GPIO-matrix loopbacks driven at 40 MHz: nothing may be connected;
 *   BOOT  the active-low menu button.
 * Never GPIO13/14 (USB), GPIO16..22 (flash/PSRAM) or a pin an onboard part
 * drives. web/boards.js repeats these maps for the flasher;
 * test/test_boards.py keeps the two and the CI build matrix in step.
 *
 * The selected map is also one table in the image (board_pins_t, board.c).
 * The web flasher's pin editor finds it by its magic, rewrites it and fixes
 * the image checksum and SHA-256; the firmware reads it once at boot. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

/* Seeed XIAO ESP32-C5 (board tested). GPIO27 is also the user LED (1k), see
 * docs/FINDINGS.md "GPIO27 IQ loopback experiment"; GPIO6/26 are the
 * battery divider. */
#define BOARD_XIAO_DAC      23, 24, 11, 12, 8, 9
#define BOARD_XIAO_IQ       1, 0, 25, 2, 27, 5, 3, 4
#define BOARD_XIAO_BOOT     28

/* Espressif ESP32-C5-DevKitC-1 v1.1/v1.2 (not board tested). GPIO11/12 are
 * UART0 to the USB-UART bridge and GPIO27 drives the RGB LED, so D6/D7
 * move to GPIO6/10 and I[4] to GPIO26. GPIO15 stays free for the SPI PSRAM
 * chip select of N8R4 modules. */
#define BOARD_DEVKITC_DAC   23, 24, 6, 10, 8, 9
#define BOARD_DEVKITC_IQ    1, 0, 25, 2, 26, 5, 3, 4
#define BOARD_DEVKITC_BOOT  28

/* Waveshare ESP32-C5-Pico (schematic, not board tested). GPIO23/24 are the
 * I2C bus of its TCA9554 expander (10k pull-ups), GPIO6 the battery
 * divider, GPIO27 the WS2812 LED; GPIO0/1 reach the header through 0R. */
#define BOARD_PICO_DAC      10, 15, 11, 12, 8, 9
#define BOARD_PICO_IQ       1, 0, 25, 2, 26, 5, 3, 4
#define BOARD_PICO_BOOT     28

/* NiceMCU ESP32-C5 Mini, ESP32-C5HF4 (schematic, PR #197, not board
 * tested). GPIO23 is the user LED; GPIO24..27 reach no header, so four IQ
 * bits use them and header IO15 stays free. GPIO7 straps only the JTAG
 * source, and only with EFUSE_JTAG_SEL_ENABLE burnt. */
#define BOARD_NICEMCU_DAC   4, 5, 6, 7, 8, 9
#define BOARD_NICEMCU_IQ    1, 0, 25, 10, 27, 26, 3, 24
#define BOARD_NICEMCU_BOOT  28

#if defined(CONFIG_C5VRX_BOARD_DEVKITC)
#define BOARD_ID            "devkitc"
#define BOARD_DAC_GPIOS     BOARD_DEVKITC_DAC
#define BOARD_IQ_GPIOS      BOARD_DEVKITC_IQ
#define BOARD_BOOT_GPIO     BOARD_DEVKITC_BOOT
#elif defined(CONFIG_C5VRX_BOARD_PICO)
#define BOARD_ID            "pico"
#define BOARD_DAC_GPIOS     BOARD_PICO_DAC
#define BOARD_IQ_GPIOS      BOARD_PICO_IQ
#define BOARD_BOOT_GPIO     BOARD_PICO_BOOT
#elif defined(CONFIG_C5VRX_BOARD_NICEMCU)
#define BOARD_ID            "nicemcu"
#define BOARD_DAC_GPIOS     BOARD_NICEMCU_DAC
#define BOARD_IQ_GPIOS      BOARD_NICEMCU_IQ
#define BOARD_BOOT_GPIO     BOARD_NICEMCU_BOOT
#else
#define BOARD_ID            "xiao"
#define BOARD_DAC_GPIOS     BOARD_XIAO_DAC
#define BOARD_IQ_GPIOS      BOARD_XIAO_IQ
#define BOARD_BOOT_GPIO     BOARD_XIAO_BOOT
#endif

/* The pin table in the image. Layout shared with web/image-pins.js: 8-byte
 * magic, DAC D4..D9, IQ bits 0..7, menu button, custom flag (1 = written by
 * the web flasher), 8 reserved bytes; 32 bytes in all. */
#define BOARD_PINS_MAGIC    {'C', '5', 'V', 'R', 'X', 'P', 'N', '1'}
typedef struct {
    char magic[8];
    int8_t dac[6];
    int8_t iq[8];
    int8_t boot;
    uint8_t custom;
    uint8_t reserved[8];
} board_pins_t;
_Static_assert(sizeof(board_pins_t) == 32, "web/image-pins.js reads 32 bytes");

/* Why a pin map cannot be used, or NULL. The same rules as the flasher
 * (web/boards.js pinProblem; shared cases in test/data/board_pin_cases.txt):
 *   range      every pin is a GPIO 0..28;
 *   usb        GPIO13/14 carry the USB console;
 *   flash      GPIO16..22 are the flash/PSRAM bus;
 *   duplicate  one GPIO for two jobs;
 *   boot_strap GPIO28 is the boot-mode strap and the BOOT button on every
 *              known board: only the menu button may use it (a DAC branch
 *              would pull it low at reset into download mode, an IQ output
 *              would fight the button);
 *   psram_cs   with PSRAM, GPIO15 can be the module PSRAM chip select. */
static inline const char *board_pins_problem(const board_pins_t *p, bool psram)
{
    int8_t all[15];
    for (unsigned k = 0; k < 6u; ++k) all[k] = p->dac[k];
    for (unsigned k = 0; k < 8u; ++k) all[6u + k] = p->iq[k];
    all[14] = p->boot;
    for (unsigned k = 0; k < 15u; ++k) if (all[k] < 0 || all[k] > 28) return "range";
    for (unsigned k = 0; k < 15u; ++k) if (all[k] == 13 || all[k] == 14) return "usb";
    for (unsigned k = 0; k < 15u; ++k) if (all[k] >= 16 && all[k] <= 22) return "flash";
    for (unsigned a = 0; a < 15u; ++a)
        for (unsigned b = a + 1u; b < 15u; ++b) if (all[a] == all[b]) return "duplicate";
    for (unsigned k = 0; k < 14u; ++k) if (all[k] == 28) return "boot_strap";
    if (psram) for (unsigned k = 0; k < 15u; ++k) if (all[k] == 15) return "psram_cs";
    return NULL;
}

/* Reads the image's table once (app_main, before any task); a table that
 * fails board_pins_problem() falls back to the compiled map. */
void board_init(void);
const board_pins_t *board_pins(void);
/* BOARD line: id, preset or custom pins, the map, and a rejected table. */
void board_print(void);
