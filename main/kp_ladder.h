/* C5VRX by Twotoz and contributors: U85Q kp policy, without LP switching.
 * Fixed P64 models do not use this ladder. Unknown C/N holds the level;
 * channel/filter changes discard stale evidence in control.c. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cn_estimate.h"
#define KP_LADDER_LEVELS 8
#define KP_HOLD_US 1000000LL
#define KP_STEP_GAP_US 500000LL
#define KP_HOLD_UP_US 300000LL
#define KP_STEP_GAP_UP_US 200000LL
#define KP_GAIN_WRITE_GUARD_US 5000LL
#define KP_STALE_US 500000LL
#define KP_NARROW_MAX_WIDTH_KHZ 11000u
typedef struct {
    uint8_t level;
    int32_t cn_q4;
    bool cn_init, narrow, wide;
    int8_t pending_dir;
    int64_t pending_since, last_step_us, last_usable_us;
    uint32_t steps;
} kp_ladder_t;
static inline void kp_ladder_reset(kp_ladder_t *l) { *l=(kp_ladder_t){0}; }
static inline bool kp_ladder_window_usable(bool settled, bool context_unchanged, int64_t since_gain_write_us)
{
    return settled && context_unchanged && since_gain_write_us >= KP_GAIN_WRITE_GUARD_US;
}
enum { KP_CLASS_LOW, KP_CLASS_NORMAL, KP_CLASS_MID, KP_CLASS_WIDE, KP_CLASSES };
static const int16_t kp_cn_points_x10[6] = {0, 20, 40, 80, 120, 200};
static const uint8_t kp_curve_x100[KP_CLASSES][6] = {
    {70, 55, 50, 50, 50, 75},   /* deviation <= 0.80 */
    {75, 60, 55, 55, 65, 75},   /* <= 1.05 */
    {80, 70, 65, 65, 70, 80},   /* <= 1.15 (interpolated, no own run) */
    {85, 85, 80, 80, 80, 80},   /* wider or unknown */
};

static const uint8_t kp_curve_narrow_x100[2][6] = {{50,50,50,50,50,80},{80,55,50,50,60,85}};
static const uint8_t kp_curve_wide16_x100[6] = {85,85,85,85,85,85};
static inline bool kp_ladder_narrow_rows(bool matched_profile, unsigned narrow_width_khz)
{
    return matched_profile && (narrow_width_khz == 0u || narrow_width_khz <= KP_NARROW_MAX_WIDTH_KHZ);
}
static inline int kp_ladder_class(int dev_x100)
{
    if (dev_x100 <= 0) return KP_CLASS_WIDE;
    if (dev_x100 <= 80) return KP_CLASS_LOW;
    if (dev_x100 <= 105) return KP_CLASS_NORMAL;
    if (dev_x100 <= 115) return KP_CLASS_MID;
    return KP_CLASS_WIDE;
}
static inline int kp_ladder_curve_g(int cls, int cn_x10, bool narrow, bool wide)
{
    const uint8_t *k = narrow && cls <= KP_CLASS_NORMAL ? kp_curve_narrow_x100[cls]
        : wide && cls == KP_CLASS_WIDE ? kp_curve_wide16_x100 : kp_curve_x100[cls];
    if (cn_x10 <= kp_cn_points_x10[0]) return k[0];
    for (int i = 1; i < 6; ++i)
        if (cn_x10 <= kp_cn_points_x10[i])
            return k[i - 1] + (k[i] - k[i - 1]) * (cn_x10 - kp_cn_points_x10[i - 1]) /
                              (kp_cn_points_x10[i] - kp_cn_points_x10[i - 1]);
    return k[5];
}
static inline uint8_t kp_ladder_level_for(int kp_x100)
{
    int lv = (85 - kp_x100 + 2) / 5;
    return (uint8_t)(lv < 0 ? 0 : lv >= KP_LADDER_LEVELS ? KP_LADDER_LEVELS - 1 : lv);
}
static inline uint8_t kp_ladder_target_g(int cn, int dev, bool narrow, bool wide)
{ return kp_ladder_level_for(kp_ladder_curve_g(kp_ladder_class(dev),cn,narrow,wide)); }
static inline uint8_t kp_ladder_update_evidence(kp_ladder_t *l, int cn_x10, bool window_ok,
                                                int dev_x100, int64_t now_us)
{
    /* Deeper than the wide row ever goes (deviation lost on a channel
     * change, or measured wide): back to that row's deepest level at once. */
    const uint8_t safe = kp_ladder_class(dev_x100) == KP_CLASS_WIDE ? kp_ladder_level_for(80) : KP_LADDER_LEVELS - 1;
    if (l->level > safe) {
        l->level = safe;
        l->pending_dir = 0;
        l->last_step_us = now_us;
        ++l->steps;
        return l->level;
    }
    const bool cn_known = cn_x10 > CN_UNKNOWN_X10;
    if (!cn_known) {
        l->pending_dir = 0;                 /* never finish a cached C/N step */
    }
    if (now_us - l->last_usable_us > KP_STALE_US) l->pending_dir = 0;
    /* Per-window estimates jitter by a few dB: decide on a smoothed C/N
     * (first-order, 1/8 per 50 ms window, ~0.4 s). */
    if (window_ok && cn_known) {
        l->last_usable_us = now_us;
        if (!l->cn_init) { l->cn_q4 = cn_x10 * 16; l->cn_init = true; }
        else l->cn_q4 += (cn_x10 * 16 - l->cn_q4) / 8;
    }
    const int cn = l->cn_init ? l->cn_q4 / 16 : 0;
    if (!window_ok || !cn_known) return l->level;
    const uint8_t target = kp_ladder_target_g(cn, dev_x100, l->narrow, l->wide);
    const int8_t dir = target > l->level ? 1 : target < l->level ? -1 : 0;
    if (dir != l->pending_dir) {
        l->pending_dir = dir;
        l->pending_since = now_us;
        return l->level;
    }
    const int64_t hold = dir < 0 ? KP_HOLD_UP_US : KP_HOLD_US, gap = dir < 0 ? KP_STEP_GAP_UP_US : KP_STEP_GAP_US;
    if (dir && now_us - l->pending_since >= hold && now_us - l->last_step_us >= gap) {
        l->level = (uint8_t)(l->level + dir);
        l->last_step_us = now_us;
        ++l->steps;
    }
    return l->level;
}
