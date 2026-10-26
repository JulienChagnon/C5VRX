/* C5VRX by Twotoz and contributors: real IQ -> afc2_measure regression.
 * Synthetic FM plateaus, not mocked result structs or a board range proof. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "afc.h"
#include "phase8_gain_lut.h"

static uint8_t raw[8190];   /* the control loop's AFC window; most cases use 4096 */
#define RAW4K 4096u
static unsigned checks, failures;
static double s_offset_hz;   /* carrier offset: shifts every level alike */
static unsigned s_phase;     /* line phase of the window start, samples */
static void check(const char *name, bool ok)
{
    ++checks; failures += !ok;
    printf("%s %s\n", ok ? "ok" : "FAIL", name);
}
static void fm(unsigned period, int polarity, bool burst, double picture_hz)
{
    double phase = .3;
    for (unsigned k = 0; k < sizeof(raw); ++k) {
        unsigned pos = (k + s_phase + period - 160u) % period;
        double hz = (pos < 188 ? -2914000. : pos < 400 || pos >= period - 100 ? -1000000. : picture_hz) + s_offset_hz;
        if (burst && pos >= 228 && pos < 308)
            hz += 957000. * sin(6.283185307179586 * (period == 2560 ? 4433618.75 : 3579545.45) * k / 40000000.);
        phase += 6.283185307179586 * hz / 40000000. * -polarity;
        int i = (int)floor(4.2 * cos(phase)), q = (int)floor(4.2 * sin(phase));
        raw[k] = (uint8_t)(((unsigned)i & 15u) << 4 | ((unsigned)q & 15u));
    }
}
int main(void)
{
    for (unsigned standard = 1; standard <= 2; ++standard)
        for (int polarity = -1; polarity <= 1; polarity += 2) {
            unsigned period = standard == 2 ? 2560 : 2542;
            fm(period, polarity, false, 2000000.);
            afc2_result_t r = afc2_measure(raw, RAW4K, c5vrx_phase8_gain_lut);
            char label[96];
            snprintf(label, sizeof(label), "B/W %s polarity %+d: timing-confirmed line", standard == 2 ? "PAL" : "NTSC", polarity);
            check(label, r.lines && r.standard == standard && r.polarity == polarity &&
                  r.sync_pairs && r.porch_pairs && r.burst_x10 < AFC2_BURST_MIN_X10);
            snprintf(label, sizeof(label), "B/W %s polarity %+d: unbiased porch/span", standard == 2 ? "PAL" : "NTSC", polarity);
            check(label, abs(r.porch_khz - polarity * 1000) <= 50 && abs(abs(r.porch_khz - r.sync_khz) - 1914) <= 60);
            r = afc2_measure(raw, 2000, c5vrx_phase8_gain_lut);
            check("single B/W sync has no timing evidence", !r.lines);
            fm(period, polarity, true, 2000000.);
            r = afc2_measure(raw, 2000, c5vrx_phase8_gain_lut);
            check("single colour sync still works", r.lines && r.standard == standard && r.polarity == polarity);
        }
    fm(2400, -1, false, 2000000.);
    check("wrong sync period without burst is rejected", !afc2_measure(raw, RAW4K, c5vrx_phase8_gain_lut).lines);
    fm(2560, -1, false, 7500000.);
    check("fast picture FM does not block timing-confirmed sync", afc2_measure(raw, RAW4K, c5vrx_phase8_gain_lut).lines > 0);
    /* The deviation (profile.c) is the sync-to-porch span: a carrier offset
     * moves both levels, never their distance, so it is measured before the
     * AFC has locked. */
    bool span_ok = true, porch_ok = true;
    for (int off = -1000; off <= 1000; off += 250) {
        s_offset_hz = off * 1000.;
        fm(2560, -1, true, 2000000.);
        afc2_result_t r = afc2_measure(raw, RAW4K, c5vrx_phase8_gain_lut);
        span_ok = span_ok && r.lines && abs(abs(r.porch_khz - r.sync_khz) - 1914) <= 60;
        porch_ok = porch_ok && r.lines && abs(r.porch_khz - (-1000 + off)) <= 60;
    }
    s_offset_hz = 0.;
    check("carrier offset +-1 MHz: span unchanged (deviation needs no AFC lock)", span_ok);
    check("carrier offset +-1 MHz: porch follows the offset", porch_ok);
    uint32_t seed = 42;
    bool noise_ok = true;
    for (unsigned n = 0; n < 64; ++n) {
        for (unsigned k = 0; k < sizeof(raw); ++k) {
            seed = seed * 1664525u + 1013904223u;
            raw[k] = (uint8_t)(seed >> 24);
        }
        noise_ok = noise_ok && !afc2_measure(raw, RAW4K, c5vrx_phase8_gain_lut).lines &&
                   !afc2_measure(raw, sizeof(raw), c5vrx_phase8_gain_lut).lines;
    }
    check("64 independent noise windows (4096 and 8190 B) supply no sync evidence", noise_ok);
    /* Without a burst a sync needs a second one a line earlier. One 4092-byte
     * descriptor (1.6 lines) holds such a pair only at some line phases; the
     * 8190-byte AFC window (3.2 lines) at every phase. */
    unsigned short_ok = 0, long_ok = 0, phases = 0;
    for (s_phase = 0; s_phase < 2560u; s_phase += 64u, ++phases) {
        fm(2560, -1, false, 2000000.);
        short_ok += afc2_measure(raw, 4092u, c5vrx_phase8_gain_lut).lines > 0;
        const afc2_result_t r = afc2_measure(raw, sizeof(raw), c5vrx_phase8_gain_lut);
        long_ok += r.lines >= 2 && r.standard == 2 && r.period_40m >= 2554 && r.period_40m <= 2568;
    }
    s_phase = 0;
    printf("B/W PAL line phases with sync: 4092 B %u/%u, 8190 B %u/%u\n", short_ok, phases, long_ok, phases);
    check("B/W PAL: 8190-byte AFC window confirms timing at every line phase", long_ok == phases);
    check("B/W PAL: one 4092-byte descriptor misses some line phases", short_ok < phases);
    printf("%s: %u checks, %u failures\n", failures ? "FAIL" : "PASS", checks, failures);
    return failures != 0;
}
