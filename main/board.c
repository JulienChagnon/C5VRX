/* C5VRX-5 by Twotoz and the C5VRX contributors: the board pin table.
 *
 * g_board_pins is the one place the image keeps its pins. The web flasher's
 * pin editor finds it by its magic and rewrites it (web/image-pins.js), so
 * it must stay a single table in flash that the code reads at run time:
 * volatile keeps the compiler from folding the compiled map into the code,
 * used keeps it in the image. */
#include "board.h"
#include <stdio.h>

__attribute__((used, aligned(4))) const volatile board_pins_t g_board_pins = {
    .magic = BOARD_PINS_MAGIC,
    .dac = {BOARD_DAC_GPIOS},
    .iq = {BOARD_IQ_GPIOS},
    .boot = BOARD_BOOT_GPIO,
};

/* The fallback map has no magic: the flasher must find exactly one table. */
static const board_pins_t s_preset = {
    .dac = {BOARD_DAC_GPIOS},
    .iq = {BOARD_IQ_GPIOS},
    .boot = BOARD_BOOT_GPIO,
};
static board_pins_t s_pins;
static const char *s_rejected;   /* why the image's table was not used */
static bool s_ready;

static bool psram_image(void)
{
#ifdef CONFIG_SPIRAM
    return true;
#else
    return false;
#endif
}

void board_init(void)
{
    board_pins_t table;
    const volatile uint8_t *src = (const volatile uint8_t *)&g_board_pins;
    uint8_t *dst = (uint8_t *)&table;
    for (size_t k = 0; k < sizeof(table); ++k) dst[k] = src[k];
    /* A corrupted table fails the image SHA-256 at boot; this only catches a
     * map the rules reject (the flasher checks the same rules first). */
    s_rejected = board_pins_problem(&table, psram_image());
    s_pins = s_rejected ? s_preset : table;
    s_ready = true;
}

const board_pins_t *board_pins(void)
{
    if (!s_ready) board_init();   /* app_main calls board_init() before any task */
    return &s_pins;
}

void board_print(void)
{
    const board_pins_t *p = board_pins();
    printf("BOARD id=%s pins=%s dac=%d,%d,%d,%d,%d,%d iq=%d,%d,%d,%d,%d,%d,%d,%d boot=%d",
           BOARD_ID, p->custom ? "custom" : "preset", p->dac[0], p->dac[1], p->dac[2], p->dac[3], p->dac[4], p->dac[5],
           p->iq[0], p->iq[1], p->iq[2], p->iq[3], p->iq[4], p->iq[5], p->iq[6], p->iq[7], p->boot);
    if (s_rejected) printf(" rejected=%s (compiled map used)", s_rejected);
    printf("\n");
}
