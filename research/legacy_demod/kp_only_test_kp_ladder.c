/* C5VRX-5 by Twotoz and the C5VRX contributors: host test of the verified
 * kp ladder policy (main/kp_ladder.h) and of the LUT tables it moves
 * between (main/kp_tables.h).
 *
 *   cc -O2 -std=c11 -I../main test_kp_ladder.c -o t && ./t
 */
#include <stdio.h>
#include <stdlib.h>
#include "kp_only_kp_ladder.h"
#include "kp_tables.h"
#include "kp_only_kp_switch_guard.h"

/* U85Q-only ladder (K mode 1): the joint update with LP disallowed. */
static uint8_t kp_ladder_update(kp_ladder_t *l, int cn_x10, bool ok, int dev, int64_t t)
{
    return kp_ladder_update_evidence(l, cn_x10, ok, dev, t);
}
static uint8_t kp_ladder_target(int cn_x10, int dev) { return kp_ladder_target_g(cn_x10, dev, false, false); }

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

#define TICK_US 50000LL

/* Runs `seconds` of control windows at a C/N (x10) with estimate noise. */
static uint8_t run(kp_ladder_t *l, int cn_x10, int noise_x10, int dev, double seconds, int64_t *t, unsigned *changes)
{
    uint8_t last = l->level;
    for (int64_t end = *t + (int64_t)(seconds * 1e6); *t < end; *t += TICK_US) {
        int cn = cn_x10 + (noise_x10 ? rand() % (2 * noise_x10 + 1) - noise_x10 : 0);
        uint8_t lv = kp_ladder_update(l, cn, true, dev, *t);
        if (changes && lv != last) ++*changes;
        last = lv;
    }
    return l->level;
}

int main(void)
{
    char name[128];
    /* Tables: same token bits at every level, level 0 differs from 1. */
    int tokens_equal = 1, differ = 0;
    for (int lv = 1; lv < KP_LEVELS; ++lv)
        for (int k = 0; k < 1024; ++k) {
            tokens_equal &= (kp_tables[lv][k] >> 11) == (kp_tables[0][k] >> 11);
            differ += kp_tables[lv][k] != kp_tables[0][k];
        }
    check("every level keeps the token bits (one shared controller interpretation)", tokens_equal);
    check("levels differ in the tracker half", differ > 0);
    check("ladder and tables agree on the level count", KP_LEVELS == KP_LADDER_LEVELS);

    kp_ladder_t l = {0};   /* zeroed like the firmware's static s_kp: reset keeps the counters */
    int64_t t = 0;
    /* Curves (best_combo.py): kp at a C/N per deviation class. */
    check("deviation 1.0, 20 dB C/N: kp 0.75", kp_level_x100[kp_ladder_target(200, 100)] == 75);
    check("deviation 1.0, 4..8 dB C/N: kp 0.55", kp_level_x100[kp_ladder_target(40, 100)] == 55 &&
                                              kp_level_x100[kp_ladder_target(80, 100)] == 55);
    check("deviation 1.0 near the floor: back to kp 0.75 (lock)", kp_level_x100[kp_ladder_target(0, 100)] == 75);
    check("deviation 0.69, 8 dB C/N: kp 0.50", kp_level_x100[kp_ladder_target(80, 69)] == 50);
    check("wide VTX, any C/N: never below kp 0.80", kp_level_x100[kp_ladder_target(40, 140)] == 80 &&
                                                 kp_level_x100[kp_ladder_target(0, 140)] == 85);
    check("unknown deviation = wide row", kp_ladder_target(40, -1) == kp_ladder_target(40, 140));
    int deepest = 0;
    for (int dev = 0; dev <= 300; dev += 5)
        for (int cn = -50; cn <= 300; cn += 5) {
            int lv = kp_ladder_target(cn, dev);
            if (kp_ladder_class(dev) == KP_CLASS_WIDE && lv > deepest) deepest = lv;
        }
    check("wide row never goes deeper than kp 0.80", kp_level_x100[deepest] >= 80);
    /* Strong carrier, normal VTX: one step to kp 0.80 then 0.75. */
    kp_ladder_reset(&l);
    uint8_t lv = run(&l, 200, 15, 100, 20, &t, NULL);
    check("strong carrier, deviation 1.0: settles on kp 0.75", kp_level_x100[lv] == 75);
    /* Range edge (6 dB), normal VTX: walks down one level at a time. */
    unsigned changes = 0;
    lv = run(&l, 60, 10, 100, 20, &t, &changes);
    snprintf(name, sizeof(name), "6 dB C/N, deviation 1.0: kp 0.%02u in %u single steps", kp_level_x100[lv], changes);
    check(name, kp_level_x100[lv] == 55 && changes == 4);
    check("carrier strong again: back to kp 0.75", kp_level_x100[run(&l, 200, 15, 100, 20, &t, NULL)] == 75);
    check("deviation 0.69 at 6 dB: kp 0.50", kp_level_x100[run(&l, 60, 10, 69, 20, &t, NULL)] == 50);
    kp_ladder_reset(&l);
    check("deviation 1.4 at 6 dB: kp 0.80", kp_level_x100[run(&l, 60, 10, 140, 20, &t, NULL)] == 80);
    /* Deviation becomes unknown (channel change) while deep: back at once. */
    kp_ladder_reset(&l);
    (void)run(&l, 60, 0, 69, 20, &t, NULL);
    lv = kp_ladder_update(&l, 60, true, -1, t);
    check("deviation lost while deep: kp 0.80 immediately", kp_level_x100[lv] == 80);
    /* No hopping: C/N noise of +-3 dB around 8 dB stays within two levels. */
    kp_ladder_reset(&l);
    (void)run(&l, 100, 0, 100, 10, &t, NULL);
    uint8_t lo = l.level, hi = l.level, mid = l.level;
    for (int k = 0; k < 1200; ++k, t += TICK_US) {     /* 60 s */
        uint8_t v = kp_ladder_update(&l, 100 + rand() % 61 - 30, true, 100, t);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    snprintf(name, sizeof(name), "C/N 10 dB +-3 dB noise for 60 s: level %u..%u around %u", lo, hi, mid);
    check(name, hi - lo <= 2);
    /* Missing input-referred evidence is not a synthetic 0 dB sample. */
    kp_ladder_reset(&l);
    (void)run(&l, 60, 0, 69, 20, &t, NULL);
    uint8_t unknown_held = l.level;
    for (int k = 0; k < 100; ++k, t += TICK_US)
        (void)kp_ladder_update(&l, CN_UNKNOWN_X10, true, 69, t);
    check("unknown C/N holds kp and cancels its pending direction",
          l.level == unknown_held && l.pending_dir == 0);
    /* Unusable windows (retune, gain write, menu) hold the level. */
    kp_ladder_reset(&l);
    (void)run(&l, 60, 0, 100, 20, &t, NULL);
    uint8_t held = l.level;
    for (int k = 0; k < 200; ++k, t += TICK_US) (void)kp_ladder_update(&l, CN_UNKNOWN_X10, false, 100, t);
    check("unusable windows: level held", l.level == held);
    /* Window usability: settle, context and a recent gain write only (control.c). */
    check("usable: settled, same context, no gain write for 5 ms",
          kp_ladder_window_usable(true, true, KP_GAIN_WRITE_GUARD_US) && KP_GAIN_WRITE_GUARD_US == 5000);
    check("unusable: settling, context change, gain write within 5 ms",
          !kp_ladder_window_usable(false, true, 1000000) && !kp_ladder_window_usable(true, false, 1000000) &&
          !kp_ladder_window_usable(true, true, KP_GAIN_WRITE_GUARD_US - 1));

    /* Filter classes retain independent curves and the measured-width gate. */
    check("narrow rows require a measured compatible filter",
          kp_ladder_narrow_rows(true,10000) && !kp_ladder_narrow_rows(true,11900) &&
          !kp_ladder_narrow_rows(false,10000));
    check("16 MHz wide profile keeps kp 0.85",
          kp_ladder_target_g(200,170,false,true)==0);
    printf("%s: %u checks, %u failures\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail ? 1 : 0;
}
