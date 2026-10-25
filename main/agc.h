/* C5VRX-5 by Twotoz and the C5VRX contributors: selectable AGC models.
 *
 * Two alternatives to KEEL-1 (gain.c dg_tick), chosen in the menu GAIN row
 * or console G, for a like-for-like comparison on the board. Both use the
 * same observer windows (dg_observation_t) and the same KEEL-1 home rows
 * (3 dB apart from the table maximum down), so the comparison is about the
 * decision only. Host harness: test/test_agc_compare.c.
 *
 * LEVEL: the input-level AGC. A gain row scales carrier and receiver
 *   noise alike (the noise is set before the baseband gain, FINDINGS
 *   "Direct Gain V7"), so the carrier level referred to the input does not
 *   depend on the row in use. LEVEL keeps a short decaying histogram of it
 *   (~20 ms) and for every home row scores what that row would have given
 *   over the histogram: sync and picture SNR from C/N, quantisation and
 *   the detector's small-signal loss, or a fold of the ultrafine lane. It
 *   moves when another row is clearly better (hysteresis), and at once on
 *   a fold.
 *
 * RSSI: a classic AGC on received power. Every 5 ms the mean window power
 *   (carrier plus noise) is placed at a target radius with a +-3 dB dead
 *   band; a fold drops 6 dB at once. No model of noise or picture.
 *
 * Integer only per window (the C5 has no FPU); the per-row quality table is
 * computed once at reset. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "gain.h"

#define AGC_BINS 100              /* 1 dB bins of the input level */
#define AGC_LMIN_DB (-20)         /* bin 0: carrier power at the top row, dB cells^2 */
#define AGC_MAX_HOMES 32u

typedef struct {
    uint8_t max_index, homes;
    uint8_t home[AGC_MAX_HOMES];
    int16_t rel_q8[DG_STATES];        /* row gain minus the top row, dB Q8 */
    uint16_t noise_x16[DG_STATES];    /* receiver noise power per row, cells^2 x16 */
    int8_t quality[AGC_MAX_HOMES][AGC_BINS];
    uint32_t hist[AGC_BINS];
    uint64_t decide_us;
    bool fold_pending;
    uint32_t folds, decisions, moves;
} agc_level_t;

typedef struct {
    uint8_t max_index, homes;
    uint8_t home[AGC_MAX_HOMES];
    int16_t rel_q8[DG_STATES];
    uint32_t sum_x16, windows;
    uint64_t decide_us;
    uint32_t folds, moves;
} agc_rssi_t;

/* row_db_q8: the gain model per row (direct_gain_t model_db_q8), noise_top_x16:
 * receiver noise power at the table maximum, cells^2 x16. */
void agc_level_reset(agc_level_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index, uint16_t noise_top_x16);
uint8_t agc_level_tick(agc_level_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us);

void agc_rssi_reset(agc_rssi_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index);
uint8_t agc_rssi_tick(agc_rssi_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us);

/* A window where carrier and noise reached both rails of both axes: folded. */
bool agc_folded(const dg_observation_t *o);
/* 10 log10(x) in dB Q8, x >= 1 (integer). */
int agc_db_q8(uint32_t x);
