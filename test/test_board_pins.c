/* C5VRX-5 by Twotoz and the C5VRX contributors: board_pins_problem() against
 * the shared rule cases (test/data/board_pin_cases.txt, also checked by
 * web/boards.test.mjs), and every compiled board map. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"

static int parse(const char *text, int8_t *out, unsigned n)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s", text);
    unsigned k = 0;
    for (char *tok = strtok(buf, ","); tok && k < n; tok = strtok(NULL, ",")) out[k++] = (int8_t)atoi(tok);
    return k == n;
}

static int check_preset(const char *name, const board_pins_t *p)
{
    const char *why = board_pins_problem(p, false);
    if (why) printf("FAIL preset %s: %s\n", name, why);
    return why != NULL;
}

int main(int argc, char **argv)
{
    if (argc != 2) { fprintf(stderr, "usage: test_board_pins board_pin_cases.txt\n"); return 2; }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 2; }
    char line[256];
    unsigned cases = 0, failed = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        int psram, boot;
        char dac[64], iq[64], want[32];
        if (sscanf(line, "%d %63s %63s %d %31s", &psram, dac, iq, &boot, want) != 5) {
            printf("FAIL unreadable case: %s", line);
            ++failed;
            continue;
        }
        board_pins_t p = {.magic = BOARD_PINS_MAGIC, .boot = (int8_t)boot};
        if (!parse(dac, p.dac, 6) || !parse(iq, p.iq, 8)) { printf("FAIL short case: %s", line); ++failed; continue; }
        const char *why = board_pins_problem(&p, psram != 0);
        const char *got = why ? why : "ok";
        if (strcmp(got, want)) { printf("FAIL %s -> %s: %s", want, got, line); ++failed; }
        ++cases;
    }
    fclose(f);
    const board_pins_t xiao = {.dac = {BOARD_XIAO_DAC}, .iq = {BOARD_XIAO_IQ}, .boot = BOARD_XIAO_BOOT};
    const board_pins_t devkitc = {.dac = {BOARD_DEVKITC_DAC}, .iq = {BOARD_DEVKITC_IQ}, .boot = BOARD_DEVKITC_BOOT};
    const board_pins_t pico = {.dac = {BOARD_PICO_DAC}, .iq = {BOARD_PICO_IQ}, .boot = BOARD_PICO_BOOT};
    const board_pins_t nicemcu = {.dac = {BOARD_NICEMCU_DAC}, .iq = {BOARD_NICEMCU_IQ}, .boot = BOARD_NICEMCU_BOOT};
    failed += check_preset("xiao", &xiao) + check_preset("devkitc", &devkitc) +
              check_preset("pico", &pico) + check_preset("nicemcu", &nicemcu);
    if (cases < 20) { printf("FAIL only %u cases read\n", cases); ++failed; }
    printf("%s: %u pin rule cases, 4 board maps\n", failed ? "FAIL" : "PASS", cases);
    return failed ? 1 : 0;
}
