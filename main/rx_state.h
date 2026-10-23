/* C5VRX-5 by Twotoz and the C5VRX contributors: receiver state supervisor,
 * shadow mode (pure logic, host test test/test_rx_state.c).
 *
 * The control loops (Direct Gain, DC servo, AFC, profile) act on
 * partly overlapping evidence on their own schedules, and one can mislead
 * another: in the first blackbox reads a DC-pushed cloud read as overload
 * and Direct Gain dropped 10-20 steps (PR #225 proposal: one coordinated
 * control plane). Before any arbitration, this supervisor only *names* the
 * receiver's situation once a second, from the same observations, and logs
 * it: no PHY write. The flight log then shows which conflicts really occur
 * in the field, and arbitration gets built for those (Phase B).
 *
 * States: ACQUIRE (no carrier/sync yet), TRACK (picture, healthy evidence),
 * EDGE (picture on weak evidence), RECOVER (picture lost within the last
 * RX_RECOVER_S seconds), FAULT (latched fault or paused), OWNED (menu,
 * scan or calibration own the receiver). A move between ACQUIRE, TRACK,
 * EDGE and RECOVER needs RX_DWELL_S agreeing seconds; FAULT and OWNED apply
 * at once.
 *
 * Reason: the impairment that explains the second best, in priority order;
 * "unknown" evidence never becomes a reason by itself. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { RX_ACQUIRE = 0, RX_TRACK = 1, RX_EDGE = 2, RX_RECOVER = 3, RX_FAULT = 4, RX_OWNED = 5 };
enum {
    RXR_NONE = 0, RXR_NO_CARRIER = 1, RXR_WEAK = 2, RXR_DC_CLIPPING = 3, RXR_OVERLOAD = 4, RXR_DC_RESIDUAL = 5,
    RXR_SYNC_LOSS_WITH_RF = 6, RXR_OWNED = 7, RXR_FAULT = 8, RXR_LOW_PICTURE_SNR = 9,
};

#define RX_DWELL_S        2u
#define RX_RECOVER_S      10u
#define RX_TRACK_CN_X10   80     /* C/N estimate for TRACK; below it a picture is EDGE */
#define RX_RF_CN_X10      60     /* RF clearly there (diag_logic.h DIAG_CLIFF_CN_X10) */
#define RX_VIDEO_OK_PCT   50u
#define RX_VIDEO_LOST_PCT 5u
#define RX_DC_RESIDUAL_MCELLS 1000
#define RX_VQ_LOW_X10     30     /* picture SNR (VIDEO_Q) below it: EDGE */
#define RX_CN_UNKNOWN     (-990)

typedef struct {
    uint8_t video_pct;          /* control windows with sync this second */
    int16_t cn_x10;             /* RX_CN_UNKNOWN when there is no estimate */
    bool vq_known;
    int16_t vq_snr_x10;
    uint16_t overloads;         /* this second */
    uint16_t one_sided_clips;   /* this second */
    int dc_mcells;              /* |DC| of the active row */
    bool fault, owned;
} rx_obs_t;

typedef struct {
    uint8_t state, candidate, agree, reason;
    uint32_t since_video_s;     /* seconds since the last second with a picture */
    uint32_t dwell_s;           /* seconds in the current state */
    uint32_t seconds[6];        /* time in each state */
} rx_state_t;

static inline void rx_state_init(rx_state_t *s)
{
    *s = (rx_state_t){.state = RX_ACQUIRE, .candidate = RX_ACQUIRE, .since_video_s = 1000000u};
}

static inline uint8_t rx_reason(const rx_obs_t *o)
{
    const bool cn_known = o->cn_x10 > RX_CN_UNKNOWN;
    if (o->fault) return RXR_FAULT;
    if (o->owned) return RXR_OWNED;
    if (o->overloads) return RXR_OVERLOAD;
    if (o->one_sided_clips) return RXR_DC_CLIPPING;
    if (o->dc_mcells > RX_DC_RESIDUAL_MCELLS) return RXR_DC_RESIDUAL;
    if (o->video_pct <= RX_VIDEO_LOST_PCT && cn_known && o->cn_x10 >= RX_RF_CN_X10) return RXR_SYNC_LOSS_WITH_RF;
    if (o->video_pct > RX_VIDEO_LOST_PCT && o->vq_known && o->vq_snr_x10 < RX_VQ_LOW_X10) return RXR_LOW_PICTURE_SNR;
    if (cn_known && o->cn_x10 < RX_TRACK_CN_X10 && o->video_pct > RX_VIDEO_LOST_PCT) return RXR_WEAK;
    if (o->video_pct <= RX_VIDEO_LOST_PCT && !cn_known) return RXR_NO_CARRIER;
    return RXR_NONE;
}

/* What the evidence of one second says, before hysteresis. */
static inline uint8_t rx_candidate(const rx_state_t *s, const rx_obs_t *o)
{
    if (o->fault) return RX_FAULT;
    if (o->owned) return RX_OWNED;
    if (o->video_pct >= RX_VIDEO_OK_PCT) {
        const bool weak_cn = o->cn_x10 > RX_CN_UNKNOWN && o->cn_x10 < RX_TRACK_CN_X10;
        const bool weak_vq = o->vq_known && o->vq_snr_x10 < RX_VQ_LOW_X10;
        return weak_cn || weak_vq ? RX_EDGE : RX_TRACK;
    }
    if (o->video_pct > RX_VIDEO_LOST_PCT) return RX_EDGE;
    return s->since_video_s < RX_RECOVER_S ? RX_RECOVER : RX_ACQUIRE;
}

/* One second. Returns the (possibly unchanged) state. */
static inline uint8_t rx_state_step(rx_state_t *s, const rx_obs_t *o)
{
    s->since_video_s = o->video_pct > RX_VIDEO_LOST_PCT ? 0u : s->since_video_s + 1u;
    s->reason = rx_reason(o);
    const uint8_t c = rx_candidate(s, o);
    if (c == RX_FAULT || c == RX_OWNED || s->state == RX_FAULT || s->state == RX_OWNED) {
        if (c != s->state) { s->state = c; s->dwell_s = 0; }
        s->candidate = c; s->agree = 0;
    } else if (c == s->state) {
        s->agree = 0;
    } else {
        s->agree = c == s->candidate ? (uint8_t)(s->agree + 1u) : 1u;
        s->candidate = c;
        /* RECOVER follows a lost picture at once: it is the moment that matters. */
        if (s->agree >= RX_DWELL_S || (c == RX_RECOVER && (s->state == RX_TRACK || s->state == RX_EDGE))) {
            s->state = c; s->dwell_s = 0; s->agree = 0;
        }
    }
    ++s->dwell_s;
    if (s->state < 6u) ++s->seconds[s->state];
    return s->state;
}

/* Flight log byte: state + 1 (3 bits; 0 = not recorded) | reason (5 bits) << 3. */
static inline uint8_t rx_state_byte(const rx_state_t *s) { return (uint8_t)(((s->state + 1u) & 7u) | ((s->reason & 31u) << 3)); }

static inline const char *rx_state_name(unsigned st)
{
    static const char *const n[] = {"ACQUIRE", "TRACK", "EDGE", "RECOVER", "FAULT", "OWNED"};
    return st < 6u ? n[st] : "?";
}

static inline const char *rx_reason_name(unsigned r)
{
    static const char *const n[] = {"none", "no_carrier", "weak_signal", "dc_clipping", "overload", "dc_residual",
                                    "sync_loss_with_rf", "owned", "fault", "low_picture_snr"};
    return r < 10u ? n[r] : "?";
}
