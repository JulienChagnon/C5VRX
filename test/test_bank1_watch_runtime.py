"""C5VRX/Twotoz: execute the production bank-1 watcher across a queued retune."""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SDK = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#undef NDEBUG
#include <assert.h>
#include "arc_phy.h"
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portMAX_DELAY 100
#define pdTRUE 1
#define PHY_VBANK1_GROUPS 7u
#define PHY_VBANK1_STAGES 9u
typedef int StaticSemaphore_t;
typedef void *SemaphoreHandle_t;
static unsigned locks, hooks;
static bool queued_retune, request_retune, corrupt_row, unsupported, mutate_at_unlock;
static unsigned group = 4;
static int16_t table[7 * 9 * 2];
static arc_gain_table_t s_table;
unsigned char phy_param[2048];
static void retune(void);
static void *xSemaphoreCreateRecursiveMutexStatic(int *p) { return p; }
static int xSemaphoreTakeRecursive(void *p, int timeout) {
    (void)timeout; assert(p); ++locks; return 1;
}
static int xSemaphoreGiveRecursive(void *p) {
    assert(p && locks); --locks;
    if (!locks && queued_retune) { queued_retune = false; retune(); }
    if (!locks && mutate_at_unlock) { mutate_at_unlock = false; ++table[(group * 9) * 2]; }
    return 1;
}
'''
MOCKS = r'''
static const int16_t *phy_vendor_bank1_table(unsigned *g) {
    assert(locks); *g = group; return unsupported ? NULL : table;
}
static void read_agc_range(uint32_t *words, unsigned start, unsigned n) {
    assert(locks && n == 4);
    if (request_retune) { request_retune = false; queued_retune = true; }
    arc_gain_tuple_t t;
    assert(arc_gain_tuple_decode(&s_table, (uint8_t)(start / 4), &t));
    unsigned k = (group * 9 + t.rf_stage) * 2;
    words[0] = (uint32_t)table[k+1] << 22;
    words[1] = (uint32_t)(table[k] + (corrupt_row && start == 0)) << 8;
    words[2] = words[3] = 0;
}
static bool phy_dco_banks(int b1[2], int b2[2]) {
    assert(locks);
    unsigned k = (group * 9 + 8) * 2;
    b1[0] = table[k]; b1[1] = table[k+1]; b2[0] = b2[1] = 0;
    return true;
}
static unsigned phy_dco_held(void) { assert(locks); return 1; }
static char output[4096];
static size_t output_size;
static int record_printf(const char *fmt, ...) {
    assert(!locks);
    va_list args; va_start(args, fmt);
    int n = vsnprintf(output + output_size, sizeof(output) - output_size, fmt, args);
    va_end(args);
    assert(n >= 0 && (size_t)n < sizeof(output) - output_size);
    output_size += (size_t)n;
    return n;
}
#define printf record_printf
'''
HARNESS = r'''
#undef printf
static void invalidate(void) { ++hooks; }
static void retune(void) {
    phy_begin(); group = 5;
    rf_bank1_check(false); /* capture_table() under recursive write ownership */
    phy_end();
}
static void clear_output(void) { output_size = 0; output[0] = 0; }
int main(void) {
    arc_gain_table_from_bytes(&s_table, NULL, 83);
    for (unsigned g = 0; g < 7; ++g)
        for (unsigned s = 0; s < 9; ++s) {
            table[(g * 9 + s) * 2] = 260 + g;
            table[(g * 9 + s) * 2 + 1] = 220 + g;
        }
    phy_set_observation_hook(invalidate);
    rf_bank1_check(true);
    assert(strstr(output, "group=4 table_changed=0 mem_rows=84 mem_bad=0"));
    assert(!locks && !hooks && !phy_generation() && !s_bank1_events);
    uint32_t boot_hash = s_vbank1_hash;
    clear_output(); request_retune = true;
    rf_bank1_check(false);
    assert(group == 5 && !queued_retune && !locks);
    assert(hooks == 1 && phy_generation() == 1);
    rf_bank1_check(false);
    assert(!output_size && !s_bank1_events && s_vbank1_hash == boot_hash);

    corrupt_row = true; rf_bank1_check(false);
    assert(s_bank1_events == 1 && strstr(output, "mem_bad=1 first_bad=G0"));
    clear_output(); rf_bank1_check(false);
    assert(!output_size && s_bank1_events == 1);
    corrupt_row = false; rf_bank1_check(false);
    assert(s_bank1_events == 2 && strstr(output, "mem_bad=0"));

    clear_output(); ++table[(group * 9) * 2]; mutate_at_unlock = true;
    rf_bank1_check(false);
    assert(strstr(output, "table_changed=1 table_now=266/225,265/225"));
    assert(table[(group * 9) * 2] == 267 && s_vbank1_hash == boot_hash);
    assert(s_bank1_events == 3 && !locks && hooks == 1 && phy_generation() == 1);
    clear_output(); unsupported = true; rf_bank1_check(true);
    assert(!strcmp(output, "BANK1_CHECK supported=0\n") && !locks);
    unsupported = false; s_table.band5 = false; clear_output(); rf_bank1_check(true);
    assert(!strcmp(output, "BANK1_CHECK supported=0\n") && !locks);
    puts("PASS: coherent bank-1 retune, real faults, boot baseline, unlocked output and unsupported PHY");
    return 0;
}
'''


def main():
    phy = (ROOT / "main/phy.c").read_text().split("#ifdef C5VRX_PHY_PINNED", 1)[0]
    phy = re.sub(r"^#include[^\n]*\n", "", phy, flags=re.MULTILINE)
    rf = (ROOT / "main/rf.c").read_text()
    watcher = rf[rf.index("static uint32_t s_vbank1_hash"):rf.index("static void capture_table(void)")]
    with tempfile.TemporaryDirectory() as directory:
        base = Path(directory)
        source = base / "watch.c"
        binary = base / ("watch.exe" if os.name == "nt" else "watch")
        source.write_text(SDK + phy + MOCKS + watcher + HARNESS)
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(ROOT / "main"),
            str(source), str(ROOT / "main/arc_phy.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
