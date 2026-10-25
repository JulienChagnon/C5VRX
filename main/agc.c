/* C5VRX-5 by Twotoz and the C5VRX contributors: LEVEL and RSSI AGC (agc.h). */
#include "agc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* LEVEL parameters (host search, test/test_agc_compare.c). */
#ifndef LEVEL_DECIDE_US
#define LEVEL_DECIDE_US 5000u
#endif
#ifndef LEVEL_DECAY_Q8
#define LEVEL_DECAY_Q8 199u     /* per 5 ms: exp(-5 / 20 ms) */
#endif
#ifndef LEVEL_WEIGHT
#define LEVEL_WEIGHT 256u     /* one window */
#endif
#ifndef LEVEL_FOLD_WEIGHT
#define LEVEL_FOLD_WEIGHT 256u     /* a folded window counts as one, a few dB over */
#endif
#ifndef LEVEL_HYST
#define LEVEL_HYST 6        /* quality units (of 100) needed to move */
#endif
#ifndef LEVEL_FOLD_K
#define LEVEL_FOLD_K 1.15f    /* noise radii the peak may reach */
#endif
#ifndef LEVEL_DC_MARGIN
#define LEVEL_DC_MARGIN 0.35f    /* cells kept free for DC */
#endif
#ifndef LEVEL_RAIL
#define LEVEL_RAIL 7.5f
#endif
#ifndef LEVEL_LEARN_MAX_Q8
#define LEVEL_LEARN_MAX_Q8 (8 * 256)   /* learned model correction per row, at most */
#endif
#ifndef LEVEL_LEARN_WINDOWS
#define LEVEL_LEARN_WINDOWS 8u         /* windows after a move that measure the new row */
#endif
#ifndef LEVEL_LEARN_SETTLE_US
#define LEVEL_LEARN_SETTLE_US 600u     /* the write transient */
#endif
#ifndef LEVEL_SYNC_DB
#define LEVEL_SYNC_DB 3.0f     /* a picture holds above ~3 dB (FINDINGS, static edge) */
#endif
#ifndef LEVEL_PIC_DB
#define LEVEL_PIC_DB 10.0f
#endif
#ifndef LEVEL_SYNC_SHARE
#define LEVEL_SYNC_SHARE 0.6f
#endif
#ifndef LEVEL_FOLD_Q
#define LEVEL_FOLD_Q (-100)   /* a folded moment, against 100 for a full picture */
#endif

/* RSSI parameters. */
#ifndef RSSI_DECIDE_US
#define RSSI_DECIDE_US 5000u
#endif
#ifndef RSSI_TARGET_X16
#define RSSI_TARGET_X16 (12u * 16u)   /* mean power 12 cells^2: radius ~3.5 cells */
#endif
#ifndef RSSI_DEADBAND_Q8
#define RSSI_DEADBAND_Q8 (3 * 256)
#endif
#ifndef RSSI_FOLD_DROP_Q8
#define RSSI_FOLD_DROP_Q8 (6 * 256)
#endif

int agc_db_q8(uint32_t x)
{
    if (x < 1u) x = 1u;
    /* log2 by the leading bit plus a fraction, then x 10 log10(2). */
    int whole = 31 - __builtin_clz(x);
    uint64_t m = (uint64_t)x << 16 >> whole;   /* [65536, 131072) */
    int frac = 0;
    for (int bit = 7; bit >= 0; --bit) {
        m = (m * m) >> 16;
        if (m >= 131072u) { m >>= 1; frac |= 1 << bit; }
    }
    return ((whole * 256 + frac) * 771 + 128) / 256;   /* 771/256 = 3.0103 */
}

bool agc_folded(const dg_observation_t *o)
{
    return o->clip_pm >= 20u && (o->rails & 3u) == 3u && (o->rails & 12u) == 12u;
}

static unsigned build_homes(uint8_t *home, const int16_t *row_db_q8, uint8_t max_index)
{
    unsigned n = 0;
    int last = 1 << 20;
    for (int k = max_index; k >= 0 && n < AGC_MAX_HOMES; --k)
        if (last - row_db_q8[k] >= 3 * 256 - 1) { home[n++] = (uint8_t)k; last = row_db_q8[k]; }
    return n;
}

/* The home row nearest to `row` (by gain). */
static unsigned nearest_home(const uint8_t *home, unsigned homes, const int16_t *rel_q8, uint8_t row)
{
    unsigned best = 0;
    int best_d = 1 << 20;
    for (unsigned h = 0; h < homes; ++h) {
        const int d = abs(rel_q8[home[h]] - rel_q8[row]);
        if (d < best_d) { best_d = d; best = h; }
    }
    return best;
}

/* ---- LEVEL ---- */

static float small_signal_loss_db(float rms)
{
    static const float at[] = {1.5f, 2.2f, 3.0f, 4.2f}, loss[] = {2.9f, 1.4f, 0.6f, 0.0f};
    if (rms <= at[0]) return loss[0] + (at[0] - rms) * 2.0f;
    for (int k = 1; k < 4; ++k)
        if (rms <= at[k]) return loss[k - 1] + (loss[k] - loss[k - 1]) * (rms - at[k - 1]) / (at[k] - at[k - 1]);
    return 0.0f;
}

/* Quality 0..100 of a carrier at input level `level_db` on a row `rel_db`
 * under the top one: sync first, then picture; a fold is LEVEL_FOLD_Q. */
static int quality(float level_db, float rel_db, float noise_top)
{
    const float c = powf(10.0f, (level_db + rel_db) / 10.0f), n = noise_top * powf(10.0f, rel_db / 10.0f);
    if (sqrtf(c) + LEVEL_FOLD_K * sqrtf(n) + LEVEL_DC_MARGIN > LEVEL_RAIL) return LEVEL_FOLD_Q;
    const float snr = 10.0f * log10f(c / (n + 1.0f / 6.0f) + 1e-9f) - small_signal_loss_db(sqrtf(c + n));
    const float sync = 1.0f / (1.0f + expf(-(snr - LEVEL_SYNC_DB)));
    const float pic = 1.0f / (1.0f + expf(-(snr - LEVEL_PIC_DB) / 1.5f));
    return (int)lroundf(100.0f * sync * (LEVEL_SYNC_SHARE + (1.0f - LEVEL_SYNC_SHARE) * pic));
}

static void row_noise(agc_level_t *a, unsigned k)
{
    const float n = a->noise_top_x16 * powf(10.0f, a->rel_q8[k] / 2560.0f);
    a->noise_x16[k] = (uint16_t)(n < 1.0f ? 1.0f : n > 65535.0f ? 65535.0f : n);
}

static void home_quality(agc_level_t *a, unsigned h)
{
    for (int b = 0; b < AGC_BINS; ++b)
        a->quality[h][b] = (int8_t)quality(AGC_LMIN_DB + (b + 0.5f) * AGC_BIN_DB, a->rel_q8[a->home[h]] / 256.0f,
                                           a->noise_top_x16 / 16.0f);
}

void agc_level_reset(agc_level_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index, uint16_t noise_top_x16)
{
    memset(a, 0, sizeof(*a));
    a->max_index = max_index;
    a->noise_top_x16 = noise_top_x16;
    for (unsigned k = 0; k <= max_index; ++k) {
        a->rel_q8[k] = (int16_t)(row_db_q8[k] - row_db_q8[max_index]);
        row_noise(a, k);
    }
    a->homes = (uint8_t)build_homes(a->home, row_db_q8, max_index);
    for (unsigned h = 0; h < a->homes; ++h) home_quality(a, h);
}

/* The model error of the row a move went to, a quarter at a time, at most
 * LEVEL_LEARN_MAX_Q8 in all; the top row is the anchor (noise measured there). */
static void level_learn(agc_level_t *a, uint8_t row, int err_q8)
{
    if (row == a->max_index) return;
    if (err_q8 > 6 * 256) err_q8 = 6 * 256;
    if (err_q8 < -6 * 256) err_q8 = -6 * 256;
    int l = a->learned_q8[row] + err_q8 / 4;
    l = l > LEVEL_LEARN_MAX_Q8 ? LEVEL_LEARN_MAX_Q8 : l < -LEVEL_LEARN_MAX_Q8 ? -LEVEL_LEARN_MAX_Q8 : l;
    a->rel_q8[row] = (int16_t)(a->rel_q8[row] + l - a->learned_q8[row]);
    a->learned_q8[row] = (int16_t)l;
    row_noise(a, row);
    for (unsigned h = 0; h < a->homes; ++h)
        if (a->home[h] == row) home_quality(a, h);
    ++a->learned;
}

static void level_add(agc_level_t *a, int level_q8, uint32_t weight)
{
    int b = ((level_q8 >> 8) - AGC_LMIN_DB) / AGC_BIN_DB;   /* floor (level >= LMIN) */
    if (level_q8 < AGC_LMIN_DB * 256) b = 0;
    b = b < 0 ? 0 : b >= AGC_BINS ? AGC_BINS - 1 : b;
    a->hist[b] += weight;
}

static int32_t level_utility(const agc_level_t *a, unsigned h, uint64_t total)
{
    int64_t u = 0;
    for (int b = 0; b < AGC_BINS; ++b) u += (int64_t)a->hist[b] * a->quality[h][b];
    return total ? (int32_t)(u * 256 / (int64_t)total) : 0;   /* quality x256 */
}

uint8_t agc_level_tick(agc_level_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us)
{
    if (!a->homes || current > a->max_index) return current;
    const int rel = a->rel_q8[current];
    const uint32_t noise = a->noise_x16[current];
    if (agc_folded(o)) {
        /* Above the fold the level is unknown: count it a few dB over. */
        const float lim = LEVEL_RAIL - LEVEL_FOLD_K * sqrtf(noise / 16.0f);
        const float c = lim > 0.5f ? lim * lim : 0.25f;
        level_add(a, agc_db_q8((uint32_t)(c * 16.0f)) - agc_db_q8(16) - rel + 3 * 256, LEVEL_FOLD_WEIGHT);
        a->fold_pending = true;
        a->move_pending = false;   /* a folded level says nothing about the row's gain */
        a->level_windows = 0;
        ++a->folds;
    } else {
        uint32_t c = o->pwr_x16 > noise ? o->pwr_x16 - noise : 0;
        if (c < noise / 50u + 1u) c = noise / 50u + 1u;
        const int level = agc_db_q8(c) - agc_db_q8(16) - rel;
        level_add(a, level, LEVEL_WEIGHT);
        if (a->move_pending && current == a->move_to && now_us - a->move_us >= LEVEL_LEARN_SETTLE_US) {
            a->move_sum_q8 += level;
            if (++a->move_windows >= LEVEL_LEARN_WINDOWS) {
                level_learn(a, current, a->move_sum_q8 / (int)LEVEL_LEARN_WINDOWS - a->move_pre_q8);
                a->move_pending = false;
                a->level_ema_q8 = level;
                a->level_windows = 1;
            }
        } else if (!a->move_pending) {
            /* Only a carrier well above the noise measures a gain. */
            const bool usable = c >= noise;
            if (!usable) a->level_windows = 0;
            else if (!a->level_windows++) a->level_ema_q8 = level;
            else a->level_ema_q8 += (level - a->level_ema_q8) / 4;
        }
    }
    if (!a->fold_pending && now_us - a->decide_us < LEVEL_DECIDE_US) return current;
    a->decide_us = now_us;
    a->fold_pending = false;
    ++a->decisions;
    uint64_t total = 0;
    for (int b = 0; b < AGC_BINS; ++b) total += a->hist[b];
    const unsigned cur_h = nearest_home(a->home, a->homes, a->rel_q8, current);
    const int32_t cur_u = level_utility(a, cur_h, total);
    int32_t best_u = INT32_MIN;
    unsigned best = cur_h;
    for (unsigned h = 0; h < a->homes; ++h) {
        const int32_t u = level_utility(a, h, total) + (h == 0);   /* ties: listen high */
        if (u > best_u) { best_u = u; best = h; }
    }
    for (int b = 0; b < AGC_BINS; ++b) a->hist[b] = (uint32_t)(((uint64_t)a->hist[b] * LEVEL_DECAY_Q8) >> 8);
    if (best == cur_h || best_u <= cur_u + LEVEL_HYST * 256) return current;
    ++a->moves;
    a->move_pending = a->level_windows >= 4u;
    if (a->move_pending) {
        a->move_to = a->home[best];
        a->move_pre_q8 = a->level_ema_q8;
        a->move_sum_q8 = 0;
        a->move_windows = 0;
        a->move_us = now_us;
    }
    a->level_windows = 0;
    return a->home[best];
}

/* ---- RSSI ---- */

void agc_rssi_reset(agc_rssi_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index)
{
    memset(a, 0, sizeof(*a));
    a->max_index = max_index;
    for (unsigned k = 0; k <= max_index; ++k) a->rel_q8[k] = (int16_t)(row_db_q8[k] - row_db_q8[max_index]);
    a->homes = (uint8_t)build_homes(a->home, row_db_q8, max_index);
}

/* The home row whose gain is nearest to `want_q8` (relative to the top). */
static uint8_t rssi_row(const agc_rssi_t *a, int want_q8)
{
    unsigned best = 0;
    int best_d = 1 << 20;
    for (unsigned h = 0; h < a->homes; ++h) {
        const int d = abs(a->rel_q8[a->home[h]] - want_q8);
        if (d < best_d) { best_d = d; best = h; }
    }
    return a->home[best];
}

uint8_t agc_rssi_tick(agc_rssi_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us)
{
    if (!a->homes || current > a->max_index) return current;
    if (agc_folded(o)) {
        ++a->folds;
        a->sum_x16 = a->windows = 0;
        a->decide_us = now_us;
        const uint8_t to = rssi_row(a, a->rel_q8[current] - RSSI_FOLD_DROP_Q8);
        if (to != current) ++a->moves;
        return to;
    }
    a->sum_x16 += o->pwr_x16;
    ++a->windows;
    if (now_us - a->decide_us < RSSI_DECIDE_US) return current;
    a->decide_us = now_us;
    const uint32_t mean = a->windows ? a->sum_x16 / a->windows : 0;
    a->sum_x16 = a->windows = 0;
    const int err = agc_db_q8(mean ? mean : 1u) - agc_db_q8(RSSI_TARGET_X16);
    if (err > -RSSI_DEADBAND_Q8 && err < RSSI_DEADBAND_Q8) return current;
    const uint8_t to = rssi_row(a, a->rel_q8[current] - err);
    if (to != current) ++a->moves;
    return to;
}
