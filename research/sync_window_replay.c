/* C5VRX by Twotoz and contributors: bench replay of afc2_measure on real
 * 8190-byte IQ snapshots. Does the 4092-byte control window find fewer
 * syncs than the full snapshot of the same signal, and which gate drops
 * the syncs? Research only; no production decision is taken from it. */
#include <stdio.h>
#include <stdlib.h>
#include "afc.h"
#include "phase8_gain_lut.h"

#define SNAP 8190u
#define CONTROL_WINDOW 4092u   /* receiver.h CONTROL_BYTES: one GDMA descriptor */

static void replay(const char *path)
{
    static uint8_t s[SNAP];
    FILE *f = fopen(path, "rb");
    if (!f || fread(s, 1, SNAP, f) != SNAP) { printf("%s: unreadable\n", path); if (f) fclose(f); return; }
    fclose(f);
    afc2_result_t full = afc2_measure(s, SNAP, c5vrx_phase8_gain_lut);
    unsigned win = 0, stat = 0, lines = 0, valid = 0, burst = 0;
    long burst_sum = 0;
    /* Every 4092-byte sub-window at a 64-sample step: the control window
     * lands at an arbitrary line phase. */
    for (size_t off = 0; off + CONTROL_WINDOW <= SNAP; off += 64u) {
        ++win;
        const uint8_t *w = s + off;
        if (!afc2_envelope_stationary(w, CONTROL_WINDOW)) continue;
        ++stat;
        afc2_result_t r = afc2_measure(w, CONTROL_WINDOW, c5vrx_phase8_gain_lut);
        if (r.lines) ++lines;
        if (r.lines && r.standard && r.porch_pairs && r.sync_pairs) ++valid;
        if (r.burst_x10) { ++burst; burst_sum += r.burst_x10; }
    }
    printf("%s full: lines=%u std=%u pol=%+d burst_x10=%u period=%u porch_khz=%ld snr_x10=%d | "
           "4092 windows=%u stationary=%u lines=%u valid=%u burst_seen=%u mean_burst_x10=%ld\n",
           path, full.lines, full.standard, full.polarity, full.burst_x10, full.period_40m,
           (long)full.porch_khz, full.snr_x10, win, stat, lines, valid, burst,
           burst ? burst_sum / (long)burst : 0L);
}

int main(int argc, char **argv)
{
    for (int k = 1; k < argc; ++k) replay(argv[k]);
    return 0;
}
