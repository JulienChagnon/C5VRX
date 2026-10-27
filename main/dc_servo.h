/* C5VRX-5 by Twotoz and the C5VRX contributors: live DC servo decisions
 * (pure logic, host test test/test_dc_servo.c; actuator in calibration.c).
 *
 * Every channel setup gives the receiver DC another offset: six boots with
 * the VTX off left 1.0-3.8 cells after 60 s, a range limit of 2-8 dB C/N
 * against 2-3 dB without the DC (research/sensitivity_model.py, 2026-10-19),
 * and within a boot the DC wanders by 1-2 cells over seconds. The former trim
 * tried single steps on 0.1-0.9 s samples, refused clipped windows, and hunted
 * in its deadband; it corrected next to nothing.
 *
 * The servo measures each gain row's DC as the mean of every settled observer
 * probe taken in that row (the AGC already sums I and Q per probe, gain.c;
 * receiver.c keeps one sum per row), decides per row on tens of thousands of
 * samples, and steps both DAC codes of that row by the learned slope (mcells
 * per code, signed, per axis). Per row, because the stored codes are off by
 * different amounts per row after a channel setup (board 2026-10-19: G83 at
 * 0.3 cells, G82 at 3.9 after a channel round) and Direct Gain moves between
 * rows faster than one row collects a mean: short visits add up. A carrier
 * and the noise average out over that many probes. Clipping biases the mean
 * towards zero but never across it, so the loop still converges to the true
 * zero, where the clipping bias vanishes. A step that did not help twice
 * running makes the axis relearn its slope from a small probe step. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifndef DC_SERVO_ROWS
#define DC_SERVO_ROWS          96u      /* >= ARC_VENDOR_GAIN_MAX + 1 */
#endif
#define DC_SERVO_MIN_SAMPLES   40000u   /* ~1 ms of IQ, spread over >= 150 probes */
#define DC_SERVO_DEADBAND      250      /* mcells: ~0.13 of the noise per axis at the edge */
#define DC_SERVO_GAIN_PCT      70       /* fraction of the computed step (no overshoot on slope error) */
#define DC_SERVO_STEP_MAX      8        /* codes per axis and step (phy_dco_try_pair) */
#define DC_SERVO_SLOPE_DEFAULT 150      /* mcells per code until learned (board: 170-550) */
#define DC_SERVO_SLOPE_MIN     30
#define DC_SERVO_SLOPE_MAX     800
#define DC_SERVO_PROBE         3        /* codes, to learn a slope */
/* The DC wanders by itself (flicker, ~0.3 cells between means), so a small
 * step's effect drowns in it: a slope is learned only from steps of at least
 * DC_SERVO_LEARN_STEP codes (~0.6-1.2 cells), and a step counts as failed
 * only while the DC stays above DC_SERVO_BAD_MCELLS, DC_SERVO_BAD_RUN times
 * running (board 2026-10-19: 32 relearns in 100 s with 2 / no floor). */
#define DC_SERVO_LEARN_STEP    3
#define DC_SERVO_BAD_MCELLS    800
#define DC_SERVO_BAD_RUN       3u
/* Folding (board, PR #228): the ultrafine lane keeps the sign bit and folds
 * the magnitude bits (legacy/c5vrx3/main/rf.c: +288 reads as +32). With a
 * few cells of DC part of the cloud folds back, the mean stops growing and
 * then shrinks as the DC grows: every channel with a template 4+ cells off
 * read a flat ~3.0-3.5 cells whatever the code, the slope learned there had
 * the wrong sign, and the servo walked to its limit. The mean's sign stays
 * right (the sign bit is never folded), its size does not. So beyond
 * DC_SERVO_FOLD_MCELLS nothing is learned and the step is full size in the
 * direction of the sign and the polarity; still folded after
 * DC_SERVO_FOLD_FLIP codes, the polarity flips (a wrong default). */
#define DC_SERVO_FOLD_MCELLS   2000
#define DC_SERVO_FOLD_FLIP     64
#define DC_SERVO_FOLD_FLIP_MAX 512   /* the window doubles per flip (an expanding search) */
/* A bracketing search on the sign bit where the sign share saturates (the
 * cloud on one side of zero: a DC beyond ~1.3 sigma, or a row whose noise is
 * under one cell). Still on the same side after a step: the next step
 * doubles (up to DC_SERVO_FOLD_STEP_MAX); across zero: it halves, down to one
 * code. The sign bit is the ADC's own and never folds, so the crossing it
 * brackets is the true zero, at any DC and any noise level. A one-code
 * crossing is held (DC within one code of zero) until the mean moves by
 * DC_SERVO_BRACKET_MOVE_MCELLS. Blackbox 2026-10-27, sessions 246-249: the
 * edge rows G73-G83 sat 13-48 codes from their table codes; with 8-code
 * steps on 40000-sample means that took seconds per row, and rows with
 * noise under a cell hunted +-8 codes around zero. */
#define DC_SERVO_FOLD_STEP_MAX 16
#define DC_SERVO_BRACKET_MOVE_MCELLS 800
/* Decisions on a saturated sign share need far fewer samples: at 4096
 * samples a share below DC_SERVO_PNEG_SAT_LO or above _HI is > 9 sigma from
 * one half (the binomial sd is 7.8 per mille). */
#define DC_SERVO_FAST_SAMPLES  4096u
#define DC_SERVO_PNEG_SAT_LO   30
#define DC_SERVO_PNEG_SAT_HI   970

typedef struct {
    int slope[2];       /* mcells per code, signed, shared by the rows */
    uint8_t known[2];   /* slope measured (not the default) */
    int8_t pol_votes[2]; /* polarity evidence from linear-zone steps (+ / -), kept across relearns */
    struct { int last_dc[2], last_step[2]; uint8_t bad[2], last_folded[2]; int fold_walk[2], last_pneg[2], fold_window[2]; uint8_t fold_bad[2];
             int8_t prev_step[2]; uint8_t bracket[2]; int16_t bracket_dc[2]; } row[DC_SERVO_ROWS];
    uint32_t steps, learned, relearned, sign_rejects, forgotten, turns, brackets;
} dc_servo_t;

/* Whether a sign share is saturated (the fast-decision condition). */
static inline bool dc_servo_saturated(int pneg) { return pneg < DC_SERVO_PNEG_SAT_LO || pneg > DC_SERVO_PNEG_SAT_HI; }

static inline int dc_servo_abs(int v) { return v < 0 ? -v : v; }

static inline void dc_servo_init(dc_servo_t *s)
{
    *s = (dc_servo_t){0};
    s->slope[0] = s->slope[1] = DC_SERVO_SLOPE_DEFAULT;
}

/* Mean of the accumulated probes in mcells. Sums are in dsp units (2v+1,
 * twice a cell), so mcells = sum * 500 / n. */
static inline bool dc_servo_mean(int64_t sum_i, int64_t sum_q, uint32_t samples, int dc[2])
{
    if (samples < DC_SERVO_MIN_SAMPLES) return false;
    dc[0] = (int)(sum_i * 500 / (int64_t)samples);
    dc[1] = (int)(sum_q * 500 / (int64_t)samples);
    return true;
}

/* One decision for one row on a fresh mean measured after that row's last
 * step (same codes and context). Learns the slope from the last step's
 * effect where nothing folds, then returns the next step per axis. */
/* Share of samples with the sign bit set, per mille: the sign bit never
 * folds, so this follows the true DC everywhere (P = Phi(-DC/sigma)); 100..900
 * is |DC| < 1.28 sigma, where nothing folds and the mean is linear. */
#define DC_SERVO_PNEG_LO 100
#define DC_SERVO_PNEG_HI 900
static inline int dc_servo_pneg(uint32_t neg, uint32_t samples)
{
    return samples ? (int)((uint64_t)neg * 1000u / samples) : 500;
}

/* pneg: the sign share per axis (NULL: folding judged from the mean alone). */
static inline void dc_servo_decide_sign(dc_servo_t *s, unsigned r, const int dc[2], const int *pneg, int step[2])
{
    step[0] = step[1] = 0;
    if (r >= DC_SERVO_ROWS) return;
    for (int a = 0; a < 2; ++a) {
        if (!s->slope[a]) s->slope[a] = DC_SERVO_SLOPE_DEFAULT;
        const bool folded = pneg ? (pneg[a] < DC_SERVO_PNEG_LO || pneg[a] > DC_SERVO_PNEG_HI)
                                 : dc_servo_abs(dc[a]) >= DC_SERVO_FOLD_MCELLS;
        /* the DC's sign: the sign share where it can fold, else the mean */
        const int sign = pneg && folded ? (pneg[a] < 500 ? 1 : -1) : (dc[a] > 0 ? 1 : -1);
        if (s->row[r].last_step[a] && pneg && folded && s->row[r].last_folded[a]) {
            /* a full step in the fold zone: the sign share must move toward
             * 500; twice the other way is the wrong polarity */
            const int was = dc_servo_abs(s->row[r].last_pneg[a] - 500), now = dc_servo_abs(pneg[a] - 500);
            if (was < 499 && now >= was + 5) {
                const bool proven = s->pol_votes[a] >= 1 ? s->slope[a] > 0 :
                                    s->pol_votes[a] <= -1 ? s->slope[a] < 0 : false;
                if (++s->row[r].fold_bad[a] >= 2u && !proven) {
                    s->slope[a] = -s->slope[a]; s->known[a] = 0; s->row[r].fold_bad[a] = 0;
                    s->row[r].fold_walk[a] = 0; ++s->turns;
                    s->row[r].fold_window[a] = s->row[r].fold_window[a] ? s->row[r].fold_window[a] : DC_SERVO_FOLD_FLIP;
                }
            } else if (now + 5 <= was) {
                s->row[r].fold_bad[a] = 0;
            }
        }
        if (s->row[r].last_step[a]) {
            const bool linear = !folded && !s->row[r].last_folded[a] &&
                                dc_servo_abs(s->row[r].last_dc[a]) < (pneg ? 1 << 30 : DC_SERVO_FOLD_MCELLS);
            const int moved = dc[a] - s->row[r].last_dc[a];
            const int slope = moved / s->row[r].last_step[a];
            if (linear && dc_servo_abs(s->row[r].last_step[a]) >= DC_SERVO_LEARN_STEP &&
                dc_servo_abs(moved) >= 150 && dc_servo_abs(slope) >= DC_SERVO_SLOPE_MIN &&
                dc_servo_abs(slope) <= DC_SERVO_SLOPE_MAX) {
                /* Smooth a known slope; take the first measurement as is. A
                 * known slope never flips sign on one measurement: the DAC
                 * polarity does not change, a DC jump of something else does
                 * (board 2026-10-19: a channel round taught -88 for +400 and
                 * left 3.6 cells). Repeated failure relearns it (bad). */
                const int v = s->pol_votes[a] + (slope > 0 ? 1 : -1);
                s->pol_votes[a] = (int8_t)(v > 4 ? 4 : v < -4 ? -4 : v);
                if (!s->known[a]) { s->slope[a] = slope; s->known[a] = 1; ++s->learned; }
                else if ((s->slope[a] > 0) == (slope > 0)) { s->slope[a] = (s->slope[a] + slope) / 2; ++s->learned; }
                else ++s->sign_rejects;
            }
            if (linear && dc_servo_abs(dc[a]) >= dc_servo_abs(s->row[r].last_dc[a]) &&
                dc_servo_abs(dc[a]) > DC_SERVO_BAD_MCELLS) {
                if (++s->row[r].bad[a] >= DC_SERVO_BAD_RUN) {
                    /* the size is relearned; the polarity flips only on the
                     * fold walk below (a hardware constant otherwise) */
                    s->slope[a] = s->slope[a] < 0 ? -DC_SERVO_SLOPE_DEFAULT : DC_SERVO_SLOPE_DEFAULT;
                    s->known[a] = 0; s->row[r].bad[a] = 0; ++s->relearned;
                }
            } else {
                s->row[r].bad[a] = 0;
            }
            s->row[r].last_step[a] = 0;
        }
        s->row[r].last_folded[a] = folded;
        s->row[r].last_pneg[a] = pneg ? pneg[a] : 500;
        if (!folded) s->row[r].fold_walk[a] = s->row[r].fold_bad[a] = s->row[r].fold_window[a] = 0;
        if (!s->row[r].fold_window[a]) s->row[r].fold_window[a] = DC_SERVO_FOLD_FLIP;
        if (!folded) s->row[r].bracket[a] = 0;
        if (!folded && dc_servo_abs(dc[a]) <= DC_SERVO_DEADBAND) { s->row[r].prev_step[a] = 0; continue; }
        /* A held one-code crossing: the row is within a code of zero. */
        if (folded && pneg && s->row[r].bracket[a]) {
            if (dc_servo_abs(dc[a] - s->row[r].bracket_dc[a]) <= DC_SERVO_BRACKET_MOVE_MCELLS) continue;
            s->row[r].bracket[a] = 0;
            s->row[r].prev_step[a] = 0;
        }
        int st;
        if (folded && pneg) {
            /* Bracketing search on the sign bit (see DC_SERVO_FOLD_STEP_MAX). */
            const bool proven = s->pol_votes[a] >= 1 ? s->slope[a] > 0 : s->pol_votes[a] <= -1 ? s->slope[a] < 0 : false;
            if (proven && s->row[r].fold_walk[a] >= s->row[r].fold_window[a]) {
                s->row[r].fold_walk[a] = 0;
                s->row[r].fold_window[a] = s->row[r].fold_window[a] * 2 > DC_SERVO_FOLD_FLIP_MAX ?
                                           DC_SERVO_FOLD_FLIP_MAX : s->row[r].fold_window[a] * 2;
            }
            if (s->row[r].fold_walk[a] >= s->row[r].fold_window[a]) {
                s->slope[a] = -s->slope[a];          /* walked this far and still folded: wrong polarity */
                s->known[a] = 0;
                s->row[r].fold_walk[a] = 0;
                s->row[r].prev_step[a] = 0;          /* the old direction says nothing now */
                s->row[r].fold_window[a] = s->row[r].fold_window[a] * 2 > DC_SERVO_FOLD_FLIP_MAX ?
                                           DC_SERVO_FOLD_FLIP_MAX : s->row[r].fold_window[a] * 2;
                ++s->turns;
            }
            const int dir = (sign > 0) == (s->slope[a] > 0) ? -1 : 1;
            const int prev = s->row[r].prev_step[a];
            int mag = DC_SERVO_STEP_MAX;
            if (prev && (prev > 0) == (dir > 0)) {
                mag = 2 * dc_servo_abs(prev);        /* same side: expand */
                if (mag > DC_SERVO_FOLD_STEP_MAX) mag = DC_SERVO_FOLD_STEP_MAX;
            } else if (prev) {
                mag = dc_servo_abs(prev) / 2;        /* crossed zero: halve */
                if (!mag) {                          /* a one-code crossing: hold here */
                    s->row[r].bracket[a] = 1;
                    s->row[r].bracket_dc[a] = (int16_t)(dc[a] > 30000 ? 30000 : dc[a] < -30000 ? -30000 : dc[a]);
                    s->row[r].prev_step[a] = 0;
                    s->row[r].fold_walk[a] = 0;
                    ++s->brackets;
                    continue;
                }
            }
            st = dir * mag;
            s->row[r].fold_walk[a] += mag;
            s->row[r].prev_step[a] = (int8_t)st;
            step[a] = st;
            continue;
        }
        if (folded) {
            /* A polarity with net linear-zone evidence is not flipped here:
             * a strong carrier folds the IQ and the sign share says nothing
             * about the DC then (board 2026-10-20, new antenna: the I slope
             * flipped to -537, the DC ran to 4 cells). Only an unproven
             * (default) polarity walks back. */
            const bool proven = s->pol_votes[a] >= 1 ? s->slope[a] > 0 : s->pol_votes[a] <= -1 ? s->slope[a] < 0 : false;
            if (proven && s->row[r].fold_walk[a] >= s->row[r].fold_window[a]) {
                s->row[r].fold_walk[a] = 0;
                s->row[r].fold_window[a] = s->row[r].fold_window[a] * 2 > DC_SERVO_FOLD_FLIP_MAX ?
                                           DC_SERVO_FOLD_FLIP_MAX : s->row[r].fold_window[a] * 2;
            }
            if (s->row[r].fold_walk[a] >= s->row[r].fold_window[a]) {
                s->slope[a] = -s->slope[a];          /* walked this far and still folded: wrong polarity */
                s->known[a] = 0;
                s->row[r].fold_walk[a] = 0;
                s->row[r].fold_window[a] = s->row[r].fold_window[a] * 2 > DC_SERVO_FOLD_FLIP_MAX ?
                                           DC_SERVO_FOLD_FLIP_MAX : s->row[r].fold_window[a] * 2;
                ++s->turns;
            }
            st = (sign > 0) == (s->slope[a] > 0) ? -DC_SERVO_STEP_MAX : DC_SERVO_STEP_MAX;
            s->row[r].fold_walk[a] += DC_SERVO_STEP_MAX;   /* requested: a limit-blocked walk counts too */
        } else {
            const long num = -(long)dc[a] * DC_SERVO_GAIN_PCT;
            const long den = (long)s->slope[a] * 100;
            long q = num / den;                                     /* rounded to nearest */
            const long rem = num % den;
            if (2 * (rem < 0 ? -rem : rem) >= (den < 0 ? -den : den)) q += (num < 0) != (den < 0) ? -1 : 1;
            st = (int)q;
            if (!st) st = (dc[a] > 0) == (s->slope[a] > 0) ? -1 : 1;
        }
        if (st > DC_SERVO_STEP_MAX) st = DC_SERVO_STEP_MAX;
        if (st < -DC_SERVO_STEP_MAX) st = -DC_SERVO_STEP_MAX;
        s->row[r].prev_step[a] = (int8_t)st;
        step[a] = st;
    }
}

static inline void dc_servo_decide(dc_servo_t *s, unsigned r, const int dc[2], int step[2])
{
    dc_servo_decide_sign(s, r, dc, NULL, step);
}

/* The row's step was applied: remember it for the slope and the check. */
static inline void dc_servo_applied(dc_servo_t *s, unsigned r, const int dc[2], const int step[2])
{
    if (r >= DC_SERVO_ROWS) return;
    for (int a = 0; a < 2; ++a) {
        s->row[r].last_dc[a] = dc[a];
        s->row[r].last_step[a] = step[a];
    }
    ++s->steps;
}

/* The row's context changed before its next mean: no slope evidence. */
static inline void dc_servo_forget_step(dc_servo_t *s, unsigned r)
{
    if (r >= DC_SERVO_ROWS) return;
    if (s->row[r].last_step[0] || s->row[r].last_step[1]) ++s->forgotten;
    s->row[r].last_step[0] = s->row[r].last_step[1] = 0;
    s->row[r].prev_step[0] = s->row[r].prev_step[1] = 0;
    s->row[r].bracket[0] = s->row[r].bracket[1] = 0;
}

static inline void dc_servo_forget_all(dc_servo_t *s)
{
    for (unsigned r = 0; r < DC_SERVO_ROWS; ++r) {
        dc_servo_forget_step(s, r);
        for (int k = 0; k < 2; ++k)
            s->row[r].fold_walk[k] = s->row[r].fold_window[k] = s->row[r].fold_bad[k] = s->row[r].last_folded[k] = 0;
    }
}
