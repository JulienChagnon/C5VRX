/* C5VRX by Twotoz and contributors: manual calibration, passive evidence.
 * Missing maps and a single noisy estimate are never a calibration fault. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define CAL_BAD_DC_MCELLS 1500
#define CAL_CONFIRM_BLOCKS 3u
#define CAL_EVIDENCE_GAP_US 2000000LL
enum { CAL_UNKNOWN, CAL_GOOD, CAL_BAD };
typedef struct {
    uint32_t generation;
    int64_t last_us;
    unsigned good, bad, state, reason;
    int last_i, last_q;
} cal_watch_t;

/* Independent blocks in the same physical context, no accumulation across
 * long unknown gaps. Stable bad DC or repeated hardware mismatch warns;
 * three good blocks recover. A transient or an unidentifiable cloud does not. */
static inline bool cal_watch_add(cal_watch_t *s, uint32_t generation, int64_t now,
                                bool qualified, unsigned reason, int i, int q)
{
    if (s->generation != generation || (s->last_us && now - s->last_us > CAL_EVIDENCE_GAP_US))
        *s = (cal_watch_t){.generation = generation};
    if (!qualified) { s->good = s->bad = 0; return false; }
    const bool bad = reason || abs(i) > CAL_BAD_DC_MCELLS || abs(q) > CAL_BAD_DC_MCELLS;
    if (bad) {
        s->good = 0;
        if (s->bad && (s->reason != reason || abs(s->last_i - i) > 750 || abs(s->last_q - q) > 750)) s->bad = 0;
        ++s->bad;
        s->reason = reason;
    } else { s->bad = 0; ++s->good; }
    s->last_i = i; s->last_q = q; s->last_us = now;
    bool warn = s->state != CAL_BAD && s->bad >= CAL_CONFIRM_BLOCKS;
    if (s->bad >= CAL_CONFIRM_BLOCKS) s->state = CAL_BAD;
    if (s->good >= CAL_CONFIRM_BLOCKS) s->state = CAL_GOOD;
    return warn;
}
