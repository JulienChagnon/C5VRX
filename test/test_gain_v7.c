/* C5VRX-5 by Twotoz and the C5VRX contributors: Direct Gain V7 peak AGC
 * (main/gain.c), actual dg_tick(): behaviour on clean evidence.
 *
 * A steady carrier gets no writes; a multipath null does not raise the gain;
 * a carrier really gone climbs to the table maximum; noise at the maximum
 * (the range edge) stays there; a Wi-Fi burst under 1 ms is ignored while
 * lasting saturation cuts the gain once; clipping on one rail per axis (DC)
 * never moves the gain while clipping on both rails does.
 *
 *   cc -O2 -std=c11 -I../main test_gain_v7.c ../main/gain.c ../main/arc_phy.c -lm -o t && ./t
 */
#include <stdio.h>
#include <string.h>
#include "gain.h"
#include "phase8_gain_lut.h"

unsigned char phy_param[2048];     /* arc_phy.c reference; unused for the 5 GHz table */

static unsigned s_fail, s_run;
#define CHECK(c, ...) do { ++s_run; if (!(c)) { ++s_fail; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

static dg_observation_t carrier(uint64_t t)   /* radius ~5.8 cells (KEEL-1 working point since PR #230), constant envelope */
{
    return (dg_observation_t){.p50 = 31, .p90 = 33, .p95 = 34, .origin_pm = 0, .clip_pm = 0, .coherence = 90,
                              .repeat_pm = 50, .span = 11, .observed_us = t};
}
static dg_observation_t empty(uint64_t t)
{
    return (dg_observation_t){.p50 = 1, .p90 = 2, .p95 = 2, .origin_pm = 900, .coherence = 5,
                              .repeat_pm = 100, .span = 3, .observed_us = t};
}
static dg_observation_t noise_at_max(uint64_t t)   /* receiver noise at G83, 1.95 cells per axis */
{
    return (dg_observation_t){.p50 = 6, .p90 = 14, .p95 = 18, .origin_pm = 360, .coherence = 38,
                              .repeat_pm = 120, .span = 12, .rails = 0, .observed_us = t};
}
static dg_observation_t folded(uint64_t t)    /* a carrier too large: both rails of both axes */
{
    return (dg_observation_t){.p50 = 80, .p90 = 105, .p95 = 110, .origin_pm = 0, .clip_pm = 700, .coherence = 30,
                              .repeat_pm = 50, .span = 15, .rails = 15, .observed_us = t};
}

static uint8_t step(direct_gain_t *g, dg_observation_t o)
{
    uint8_t before = g->current_gain, target = dg_tick(g, &o);
    if (target != before) dg_sync_applied(g, target, o.observed_us);
    return target;
}

int main(void)
{
    arc_gain_table_t table;
    arc_gain_table_from_bytes(&table, NULL, ARC_5G_GAIN_MAX);
    static direct_gain_t g;
    const dg_map_context_t c = {.freq_mhz = 5865, .lane_mode = 2, .iq_scale = 1, .filter_skirt = 32, .filter_bw20 = 1,
                                .filter_code = 60};
    uint64_t t = 1000;

    dg_reset_context(&g, &table, 40, &c);
    for (int k = 0; k < 5000; ++k, t += 200) step(&g, carrier(t));
    CHECK(g.current_gain == 40 && g.writes == 0, "steady carrier: no writes in 1 s (gain %u, writes %lu)",
          g.current_gain, (unsigned long)g.writes);

    for (int k = 0; k < 25; ++k, t += 200) step(&g, empty(t));        /* a 5 ms multipath null */
    for (int k = 0; k < 500; ++k, t += 200) step(&g, carrier(t));
    CHECK(g.current_gain == 40 && g.writes == 0, "5 ms null: the gain does not move (gain %u, writes %lu)",
          g.current_gain, (unsigned long)g.writes);

    const uint32_t w0 = g.writes;
    for (int k = 0; k < 1500; ++k, t += 200) step(&g, empty(t));      /* the VTX is gone */
    CHECK(g.current_gain == table.max_index, "VTX gone: the table maximum within 300 ms (G%u, %lu writes)",
          g.current_gain, (unsigned long)(g.writes - w0));
    const uint32_t w1 = g.writes;
    for (int k = 0; k < 5000; ++k, t += 200) step(&g, noise_at_max(t));
    CHECK(g.current_gain == table.max_index && g.writes == w1, "noise at the maximum (range edge): stays there");

    dg_reset_context(&g, &table, 60, &c);
    for (int k = 0; k < 3; ++k, t += 200) step(&g, folded(t));        /* 0.6 ms: a Wi-Fi burst */
    for (int k = 0; k < 500; ++k, t += 200) step(&g, carrier(t));
    CHECK(g.current_gain == 60 && g.overloads == 0 && g.bursts_ignored > 0, "a burst under 1 ms is ignored");
    for (int k = 0; k < 8; ++k, t += 200) step(&g, folded(t));        /* lasting saturation */
    CHECK(g.current_gain < 60 && g.overloads == 1, "lasting saturation cuts the gain once (G%u, overloads %lu)",
          g.current_gain, (unsigned long)g.overloads);
    /* One-sided clipping (DC on a rail, gain.h dg_one_sided) is not overload:
     * no carrier at the table maximum, the noise cloud pushed onto Q-8 by
     * DC (board blackbox 2026-10-19: bursts of 255+ writes, 10-20 steps
     * down). Clipping on both rails of an axis still cuts the gain. */
    {
        static direct_gain_t h;
        dg_reset_context(&h, &table, table.max_index, &c);
        uint64_t u = 1000;
        for (int k = 0; k < 40; ++k, u += 200) step(&h, noise_at_max(u));
        const uint8_t top = h.current_gain;
        const uint32_t writes = h.writes;
        dg_observation_t dc = {.p50 = 6, .p90 = 30, .p95 = 40, .origin_pm = 300, .clip_pm = 160, .coherence = 30,
                               .repeat_pm = 200, .span = 9, .rails = 4u /* Q-8 only */};
        for (int k = 0; k < 200; ++k, u += 200) { dc.observed_us = u; step(&h, dc); }
        CHECK(h.current_gain == top && h.writes == writes && h.overloads == 0 && h.one_sided_clips == 200,
              "one-sided clipping leaves the gain (gain %u of %u, writes %lu, overloads %lu, one-sided %lu)",
              h.current_gain, top, (unsigned long)(h.writes - writes), (unsigned long)h.overloads,
              (unsigned long)h.one_sided_clips);
        dg_observation_t big = dc;
        big.rails = 4u | 8u;   /* Q-8 and Q+7: a carrier too large for the window */
        for (int k = 0; k < 40; ++k, u += 200) { big.observed_us = u; step(&h, big); }
        CHECK(h.current_gain < top && h.overloads > 0, "two-sided clipping still cuts the gain (gain %u, top %u)",
              h.current_gain, top);
        dg_observation_t legacy = dc;
        legacy.rails = 0;      /* no rail evidence: the old rule */
        CHECK(!dg_one_sided(&legacy) && dg_one_sided(&dc) && !dg_one_sided(&big), "rail classification");
    }
    {   /* Blackbox session 29: an AFC step (offset only) or a refused write
         * reset the whole core; its memory and counters must survive. */
        static direct_gain_t h;
        dg_reset_context(&h, &table, 60, &c);
        uint64_t u = 5000;
        for (int k = 0; k < 8; ++k, u += 200) step(&h, folded(u));        /* an attack: G60 folds */
        const uint8_t folded_row = h.folded_row;
        const uint64_t until = h.folded_until_us;
        const uint32_t writes = h.writes, overloads = h.overloads;
        dg_map_context_t afc = c;
        afc.offset_khz = 250;                                              /* the AFC moved */
        dg_reset_context(&h, &table, h.current_gain, &afc);
        CHECK(h.folded_row == folded_row && h.folded_until_us == until && h.writes == writes && h.overloads == overloads,
              "an AFC step keeps the attack memory and the counters (row %u/%u, writes %lu/%lu)", h.folded_row, folded_row,
              (unsigned long)h.writes, (unsigned long)writes);
        dg_map_context_t other = c;
        other.freq_mhz = 5845;                                             /* another channel */
        dg_reset_context(&h, &table, h.current_gain, &other);
        CHECK(h.folded_until_us == 0 && h.writes == writes, "a channel change forgets the memory, keeps the counters");
        const uint8_t real = h.current_gain;
        h.current_gain = (uint8_t)(real + 3u);                             /* the core wrote, the PHY refused */
        dg_write_refused(&h, real);
        CHECK(h.current_gain == real && h.state == DG_HOLD && h.writes_refused == 1, "a refused write puts the core back");
    }
    {   /* Recorded G21 cloud: sparse outliers used to hold P95 at43,
         * although the core classified every window as no carrier.
         * Repeated capture is a controller regression, not a physical
         * closed-loop RF simulation (gain changes cannot change old IQ). */
        FILE *f = fopen("data/gain_session55_130.bin", "rb");
        if (!f) f = fopen("test/data/gain_session55_130.bin", "rb");
        CHECK(f != NULL, "recorded session55 fixture available");
        if (f) {
            uint8_t raw[8128];
            size_t bytes = fread(raw, 1, sizeof(raw), f);
            fclose(f);
            CHECK(bytes == sizeof(raw), "complete recorded snapshot");
            dg_observation_t obs[31];
            unsigned outliers = 0;
            for (unsigned i = 0; i < 31; ++i) {
                obs[i] = dg_measure_gain(raw + 256u * i, 256, c5vrx_phase8_gain_lut, 1000);
                outliers += obs[i].p95 > 30;
            }
            CHECK(outliers > 20, "fixture really exercises high P95 despite origin majority");
            static direct_gain_t h;
            dg_reset_context(&h, &table, 21, &c);
            uint64_t u = 1000, first_write = 0;
            for (unsigned i = 0; i < 1500; ++i, u += 200) {
                dg_observation_t o = obs[i % 31]; o.observed_us = u;
                uint8_t before = h.current_gain;
                step(&h, o);
                if (!first_write && h.current_gain != before) first_write = u - 1000;
            }
            CHECK(h.current_gain == table.max_index, "recorded empty cloud no longer parks at G21 (G%u)", h.current_gain);
            CHECK(first_write >= 16000, "recovery retains peak hold (first write %llu us)",
                  (unsigned long long)first_write);
            memset(&h, 0, sizeof(h)); dg_reset_context(&h, &table, 40, &c);
            for (unsigned i = 0; i < 200; ++i, u += 200) step(&h, carrier(u));
            for (unsigned i = 0; i < 25; ++i, u += 200) {
                dg_observation_t o = obs[i]; o.observed_us = u; step(&h, o);
            }
            for (unsigned i = 0; i < 200; ++i, u += 200) step(&h, carrier(u));
            CHECK(h.current_gain == 40 && h.writes == 0, "5ms recorded-cloud fade does not pump gain");
            /* The old fixed-point rule oscillated on off-origin DC. */
            dg_observation_t dc = {.p50=60, .p95=60, .repeat_pm=1000, .span=0, .coherence=99};
            for (unsigned i = 0; i < 1000; ++i, u += 200) { dc.observed_us=u; step(&h, dc); }
            CHECK(h.current_gain == 40 && h.writes == 0, "off-origin fixed DC does not trigger max-gain recovery");
            /* A returning strong carrier must still attack on raw rails. */
            for (unsigned i = 0; i < 40; ++i, u += 200) step(&h, folded(u));
            CHECK(h.current_gain < 40 && h.overloads > 0, "raw overload still wins after origin-cloud recovery");
        }
    }
    printf("%s: %u checks, %u failures\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail ? 1 : 0;
}
