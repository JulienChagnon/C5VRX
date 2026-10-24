/* C5VRX-5 by Twotoz and the C5VRX contributors: Direct Gain V6 core.
 *
 * V6 (2026-10-18) keeps V5's measurement, healthy band, continuous tracking
 * and overload safety, and adds a carrier memory: the last healthy gain of
 * this channel/filter survives tracking resets (AFC steps, PHY leases), a
 * lost carrier climbs in bounded steps from it instead of jumping to the
 * table maximum, and a returning carrier that clips above it goes back to
 * it in one write instead of a staged walk down (board 2026-10-18: G21 ->
 * G63 with 742 per mille clipping after every fade; poor reacquisition).
 *
 * The Direct Gain V3 core (centred Q4 envelope, Phase8 coherence, exact
 * vendor 5 GHz gain tuples, zero-write healthy band) plus V5's direct
 * first-window correction, table-maximum listening without a carrier,
 * anti-hunt damping, the interference-burst gate (zerowidth/C5VRX PR #3)
 * and staged overload recovery (Louis Hitchcock, C5VRX #182/#184).
 * C5VRX-5 holds the ultrafine IQ lane for the whole session, so the
 * adaptive-lane and radius-boost paths of C5VRX-4 are not carried. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "arc_phy.h"

#define DG_STATES (ARC_VENDOR_GAIN_MAX + 1u)

typedef enum { DG_ACQUIRE = 0, DG_HOLD, DG_SETTLE, DG_VERIFY } dg_state_t;
typedef enum { DG_FINE = 0, DG_BB, DG_RF } dg_transition_t;

/* Transfer conditions for a measured gain response; gain index itself varies
 * inside a map. Epochs guard in-flight samples, not persisted identity. */
typedef struct {
    uint32_t freq_mhz, lane_mode, iq_scale, filter_skirt, filter_bw20;
    int32_t offset_khz, filter_code;
} dg_map_context_t;

typedef struct {
    uint8_t p50, p90, p95;
    uint16_t origin_pm, clip_pm;
    uint8_t coherence;
    uint16_t repeat_pm;     /* samples equal to the one before, per mille */
    uint8_t span;           /* code range of the wider IQ axis over the window */
    uint16_t env_ratio_x100; /* DC-removed envelope mean^2/var of |z|^2, x100 (noise 100) */
    uint8_t centered;        /* judged around the window DC (DC-dominated, gain.c) */
    int32_t sum_i, sum_q;    /* sums of 2v+1 per axis (dsp units): the probe DC for the servo */
    uint16_t neg_i, neg_q;   /* samples with the sign bit set: never folded (dc_servo.h) */
    uint8_t rails;           /* rails reached: bit 0 I-8, 1 I+7, 2 Q-8, 3 Q+7 */
    uint16_t pwr_x16;        /* DC-removed mean power, cells^2 x16 (rx_level.h) */
    uint64_t observed_us;
} dg_observation_t;

/* A fixed IQ point is not a carrier. Below one quantizer step of receiver
 * noise (DCO FLOOR rows, G54-69 on the board) the uncorrected DC is a
 * constant code: coherence 99-100 %, P50 ~15, inside the healthy band, and
 * Direct Gain held there with no VTX (board walk 2026-10-13, docs/FINDINGS.md).
 * IQSNAP: VTX off at G59 repeats 476-873 per mille per 64 bytes and spans
 * 1-2 codes per 256-byte probe; a weak carrier in noise at G73-75 repeats
 * 15-301 and spans 7-12. Repeats alone are not enough: a strong clean
 * carrier at 40 MS/s repeats often too (half the windows at G27, 1 m from
 * the VTX, read as a fixed point and sent the ladder into LP; board
 * 2026-10-13). A carrier draws a circle around the DC, so it spans many
 * codes even when neighbouring samples repeat.
 *
 * Measurement only: the C/N model reads it as no carrier (receiver.c
 * receiver_obs_cn_x10). Direct Gain does not act on it. Treating it as
 * no carrier sent the gain to the table maximum, where the first window
 * after the write still carried the row's uncorrected DC, clipped, dropped
 * back into the FLOOR rows and looped: ~1400 writes and hundreds of
 * overloads a second with the VTX off (board walk 3, 2026-10-13). The DC
 * of the FLOOR rows has to be corrected in the DCO path instead. */
/* Clipping on one rail per axis only: the cloud pushed against a rail by DC
 * (or the slow DC wobble), not a carrier too large for the window, which
 * reaches both rails of an axis. Lowering the gain for it is the wrong
 * action: the DC servo removes it, and the gain drop cost 10-20 steps in
 * bursts of 255+ writes a second (board blackbox 2026-10-19). rails == 0 is
 * unknown (an observation without rail evidence) and keeps the old rule. */
static inline bool dg_one_sided(const dg_observation_t *o)
{
    return o->rails && (o->rails & 3u) != 3u && (o->rails & 12u) != 12u;
}

#define DG_STUCK_PM      400u
#define DG_STUCK_SPAN    2u
static inline bool dg_stuck(const dg_observation_t *o)
{
    return o->repeat_pm >= DG_STUCK_PM && o->span <= DG_STUCK_SPAN;
}

typedef struct {
    arc_gain_table_t table;
    dg_map_context_t map_context;
    bool map_context_valid;
    arc_gain_tuple_t tuple[DG_STATES];
    uint16_t relative_power_q10[DG_STATES];
    uint16_t uncertainty_pm[DG_STATES];
    uint16_t artifact_score[DG_STATES];
    uint8_t confidence[DG_STATES];
    uint16_t settle_us[3];
    int32_t virtual_gain_q8;
    uint8_t current_gain, target_gain, prior_gain, corrections;
    uint8_t high_windows, weak_windows, last_direction;
    dg_transition_t transition;
    dg_state_t state;
    dg_observation_t before, before_previous, previous, last_tracking;
    uint8_t stable_windows;
    uint64_t write_us;
    uint32_t writes, holds, verified, learned, overloads;
    uint32_t one_sided_clips;     /* clipped windows not counted as overload (dg_one_sided) */
    bool overload_recovery;       /* staged recovery after an overload drop */
    int8_t last_write_dir;        /* anti-hunt: direction reversals */
    uint8_t reversals;
    uint64_t dir_write_us, damp_until_us, excursion_since_us;
    uint32_t writes_moderate, writes_severe, damp_events, model_mismatches;
    uint64_t saturated_since_us;  /* interference-burst gate */
    bool saturated_run;
    uint32_t bursts_ignored;
    uint64_t last_healthy_us;     /* fade hold: last window in the healthy band */
    /* Map learning only with a confirmed VTX (set by the caller every
     * window). The coherence test alone passes plain 4-bit receiver noise
     * through the 12 / 10 MHz filters (~58 / 61 % coherent), so with the VTX
     * off noise ratios were learned and persisted. */
    bool learn_enabled;
    uint32_t learn_refused;       /* transitions not learned: no confirmed VTX */
    uint32_t fade_holds;          /* windows held through a fade */
    /* V6 carrier memory: kept across tracking resets on the same channel,
     * filter, lane and table (not the AFC offset). */
    bool anchor_valid;
    uint8_t anchor_gain;
    uint64_t anchor_us, carrier_us, climb_us;
    uint32_t anchor_returns, climbs;
    /* V7 (gain.c): model gain per table step, peak memory, write pacing. */
    int16_t model_db_q8[DG_STATES];
    uint8_t peak_p95[32];         /* highest P95 per 2 ms over 64 ms */
    uint8_t peak_i;
    uint64_t peak_since_us, peak_bucket_us, up_after_us, down_after_us;
    bool far_run;                 /* releasing a carrier that stays far below (gain.c) */
    /* KEEL-1 (gain.c): the home rows the gain may use, ascending. */
    uint8_t keel_row[DG_STATES];
    uint8_t keel_rows;
    uint8_t folded_row;           /* the row the last attack left (gain.c, KEEL-1 attack memory) */
    uint64_t folded_until_us;
    uint32_t writes_refused;      /* writes the PHY did not take (receiver.c apply_target) */
    uint32_t overloads_after_write;   /* overloads within 5 ms of the last write (transient suspects) */
    /* KEEL-1 learned headroom: extra dB (Q8) a release into a row needs
     * after releases into it folded; decays. Per row, self-learned. */
    uint16_t heat_q8[DG_STATES];
    uint64_t heat_us;                 /* last decay */
    uint8_t released_row;             /* the row the last release went to */
    uint64_t released_us;
    uint32_t heat_events;
    uint32_t magic;
} direct_gain_t;

/* Measured gain map, persisted so a boot does not explore blindly. */
#define DG_MAP_VERSION 7u   /* 7: 11p; discard earlier RF-context maps */
typedef struct {
    uint8_t version, max_index;
    uint16_t freq_mhz;            /* context: channel it was measured on */
    uint8_t lane_mode, band5;
    uint8_t context_valid;
    dg_map_context_t context;
    arc_gain_table_t table;
    uint8_t confidence[DG_STATES];
    uint8_t reserved[DG_STATES];
    uint16_t power_q10[DG_STATES];
    uint16_t uncertainty_pm[DG_STATES];
} dg_map_blob_t;

/* Tracking reset retains an already-bound context; transfer changes must
 * use dg_reset_context with a fresh hardware/context snapshot. */
void dg_reset(direct_gain_t *g, const arc_gain_table_t *table, uint8_t current_gain);
void dg_reset_context(direct_gain_t *g, const arc_gain_table_t *table, uint8_t current_gain,
                      const dg_map_context_t *context);
bool dg_map_context_equal(const dg_map_context_t *a, const dg_map_context_t *b);
bool dg_map_matches(const direct_gain_t *g, const arc_gain_table_t *table, const dg_map_context_t *context);
unsigned dg_export_map(const direct_gain_t *g, dg_map_blob_t *blob);
bool dg_import_map(direct_gain_t *g, const dg_map_blob_t *blob);
/* Low-confidence prior from measured boards (gain_map_preset.h): only for
 * indices without own data; false when it has no entry at the anchor. */
bool dg_import_preset(direct_gain_t *g, const uint16_t *power_q10, unsigned count, uint8_t max_index, uint8_t band5);
dg_observation_t dg_measure(const uint8_t *samples, size_t bytes,
                            const uint8_t phase8_lut[256], uint64_t observed_us);
/* Same gain evidence, without unused envelope/C/N work for the sentinel. */
dg_observation_t dg_measure_gain(const uint8_t *samples, size_t bytes,
                                 const uint8_t phase8_lut[256], uint64_t observed_us);
uint8_t dg_tick(direct_gain_t *g, const dg_observation_t *o);
/* Measured gain per row (KEEL-1 steps, rx_level): thermal_cells2[k] > 0 for
 * rows whose receiver noise was measured well above the 4-bit step; their
 * gain becomes the measured one (relative to the highest such row, which
 * keeps its model value); rows below the lowest measured one follow the
 * model, shifted to join it. Rebuilds the home rows. Returns rows set. */
unsigned dg_apply_measured_gain(direct_gain_t *g, const float thermal_cells2[DG_STATES]);
void dg_sync_applied(direct_gain_t *g, uint8_t gain, uint64_t write_us);
/* A write the PHY did not take: the core is back on the real gain, no settle. */
void dg_write_refused(direct_gain_t *g, uint8_t actual_gain);
