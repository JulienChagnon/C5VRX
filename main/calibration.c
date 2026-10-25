/* C5VRX-5 by Twotoz and contributors: manual receive-chain calibration.
 * Boot loads a baseline or a matching saved calibration and returns at once.
 * Passive, context-qualified monitoring may recommend CALIBRATE; it never
 * searches, retunes, slips the sampling phase, or trims DC automatically.
 * Menu CALIBRATE / F checks and repairs the current filter's DC rows.
 * Explicit console B also optimizes the filter and prepares other profiles.
 * Gain control/AFC remain the normal receive controls, outside this policy. */
#include "receiver.h"
#include "cn_auto.h"
#include "calibration_noise.h"
#include "calibration_policy.h"
#include <math.h>
#include <stdarg.h>
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/temperature_sensor.h"
#include "dsp.h"
#include "dc_live.h"
#include "dc_trim.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "phy.h"
#include "profile_logic.h"
#include "sphase_logic.h"
#include "dco_slots.h"
#include "dc_health.h"
#include "dc_servo.h"
#include "flightlog.h"
#include "flightlog_format.h"
#include "rf.h"
#include "soc/pcr_struct.h"

extern unsigned char phy_param[];

typedef struct {
    uint32_t windows;
    uint64_t hf4_d4, hf4_count, hf2_d2;
    int dc_i, dc_q;          /* milli-cells: circle centre or plain mean */
    bool dc_circle, dc_qualified;
    unsigned envelope_x100;
    metrics_t m;             /* last window */
    cna_sums_t noise_sums;   /* optional DC-verification reuse */
    double noise_m2, noise_m4;
} window_stats_t;

static int64_t s_boot_policy_ms, s_manual_ms, s_boot_phase_ms, s_probe_deadline;
static bool s_boot_phase_pending;
static int64_t s_boot_phase_until;
static bool s_last_manual_full;
static volatile bool s_dump_requested;
void calibration_dump_request(void) { s_dump_requested = true; }

static cal_watch_t s_basemap_watch;
static uint32_t s_basemap_context;
static unsigned s_health_blocks, s_health_rejected;
static int64_t s_health_us;

static volatile bool s_startup = true;
bool calibration_in_startup(void) { return s_startup; }

static bool quiet_owner(void)
{
    return !s_menu_active && !s_scan_active && !receiver_paused() && !phy_busy();
}

/* Status screens of the startup phase own TX (menu raster), so the menu
 * check must not count them as a busy owner. */
static bool startup_owner(void)
{
    return (!s_menu_active || menu_status_active()) && !s_scan_active && !receiver_paused() && !phy_busy();
}

static bool collect_stats(unsigned windows, window_stats_t *out, bool noise)
{
    uint8_t sample[PROBE_BYTES];
    dsp_circle_t circle = {0};
    unsigned rail_sum = 0;
    memset(out, 0, sizeof(*out));
    for (unsigned tries = 0; tries < windows * 3u && out->windows < windows; ++tries) {
        if (s_probe_deadline && (esp_timer_get_time() >= s_probe_deadline || !menu_splash_active())) break;
        vTaskDelay(2);   /* two ticks: one tick starved IDLE with the observer */
        if (!copy_probe_window(sample, NULL)) continue;
        for (unsigned r = 0; r < PROBE_REGIONS; ++r) {
            dsp_hf4_sums(sample + r * PROBE_REGION_BYTES, PROBE_REGION_BYTES, &out->hf4_d4);
            dsp_hf2_sums(sample + r * PROBE_REGION_BYTES, PROBE_REGION_BYTES, &out->hf2_d2);
        }
        out->hf4_count += PROBE_REGIONS * (PROBE_REGION_BYTES - 4u);
        dsp_circle_sums(sample, sizeof(sample), &circle);
        out->m = measure_window(sample, sizeof(sample));
        rail_sum += (unsigned)out->m.clip_pm;
        if (noise) {
            cna_sums_t p = {0};
            for (unsigned k = 0; k < sizeof(sample); ++k) cna_sums_add(&p, sample[k]);
            float mi, mq;
            double m2, m4;
            cna_mean(&p, &mi, &mq); cna_moments(&p, mi, mq, &m2, &m4);
            out->noise_m2 += m2; out->noise_m4 += m4;
            cna_sums_merge(&out->noise_sums, &p);
        }
        ++out->windows;
    }
    if (!out->windows) return false;
    out->dc_circle = dc_live_measure(&circle, rail_sum / out->windows,
        &out->dc_i, &out->dc_q, &out->envelope_x100);
    int ni, nq;
    out->dc_qualified = out->dc_circle || dc_live_noise_mean(&circle, rail_sum / out->windows, &ni, &nq);
    return out->windows * 2u >= windows;
}

static bool collect(unsigned windows, window_stats_t *out)
{
    return collect_stats(windows, out, false);
}

/* ---- Sampling phase --------------------------------------------------------
 * PARLIO RX (PLL_F240M/6) and MODEM_DIAG (every 3 ticks of 240 MHz) have a
 * phase fixed at reset; about 3 boots in 10 sample mid-transition (zerowidth
 * PR #3). The fourth-difference excess separates them on noise and on a
 * carrier; a bad phase is slipped one RX tick at a time, only in the
 * startup phase of every boot. Live monitoring is passive; an automatic
 * correction is deferred until an exact bounded phase actuator and verified
 * rollback exist. The existing 1-us divider pulse changes RX/TX separation
 * and is not a proved single-position move.
 *
 * Verdict (sphase_logic.h, test/test_sphase.c): the absolute limit is the
 * fast path; above it only a scan whose positions differ is evidence of a
 * mid-transition read. Positions that all read alike are intrinsic noise
 * (NOISE_LIMITED): no repair, not a phase fault. The stored DC row is held
 * while the gain worker is paused for a scan, and the final qualification
 * runs after the status screen has handed TX back to live video, because
 * the TX context changes the noise the probe sees (board 2026-10-12). */
#define SPHASE_POSITIONS  9u
#define SPHASE_TRIES      12u
static int s_sphase_excess = -1, s_sphase_white = -1, s_sphase_dc[2];
static unsigned s_sphase_scans;
static sphase_scan_t s_sphase_last = {.best = 0x7fffffff, .worst = -1};
static const char *s_sphase_state = "unverified";
static void dco_hold_for_scan(void);
static bool video_recent(void);

static int sphase_excess(const window_stats_t *w)
{
    int e = dsp_hf4_excess_milli(w->hf4_d4, w->hf4_count);
    return e > 0 ? e : 0;
}

/* The gate metric: absolute excess (HF4/HF2 whiteness is logged only; a
 * normalized gate failed board repeat scans, phasefix6). */
static int sphase_metric(const window_stats_t *w) { return sphase_excess(w); }

static int sphase_record(const window_stats_t *w)
{
    s_sphase_excess = sphase_excess(w);
    s_sphase_white = sphase_white_permille(w->hf4_d4, w->hf2_d2);
    s_sphase_dc[0] = w->dc_i;
    s_sphase_dc[1] = w->dc_q;
    return sphase_metric(w);
}

/* positions/windows/tries: startup 9 / 48 / 12 on any signal; on the
 * carrier 3 / 16 / 6 (one cycle, ~0.4 s with the control loop paused),
 * settling within the tight carrier margin of the cleanest position. */
static bool phase_slip(void)
{
    if (!s_probe_deadline) { rx_clock_slip(1); return true; }
    /* Close the splash-check/write race with main's TX handover. This
     * encloses only the existing one-microsecond register pulse, no waits. */
    vTaskSuspendAll();
    const bool allowed = menu_splash_active() && esp_timer_get_time() < s_probe_deadline;
    if (allowed) rx_clock_slip(1);
    (void)xTaskResumeAll();
    return allowed;
}

static sphase_verdict_t sphase_scan_n(const char *tag, unsigned positions, unsigned windows, unsigned tries,
                                      bool carrier)
{
    if (!receiver_pause(tag)) return SPHASE_UNSETTLED;
    if (s_probe_deadline && (!menu_splash_active() || esp_timer_get_time() >= s_probe_deadline)) {
        receiver_resume(); return SPHASE_UNSETTLED;
    }
    if (!s_probe_deadline) dco_hold_for_scan();
    window_stats_t w;
    const bool fast = !carrier && positions == 3u;
    sphase_scan_reset(&s_sphase_last);
    for (unsigned pos = 0; pos < positions; ++pos) {
        if (s_probe_deadline && (esp_timer_get_time() >= s_probe_deadline || !menu_splash_active())) break;
        if (pos) { if (!phase_slip()) break; vTaskDelay(pdMS_TO_TICKS(20)); }
        if (collect(windows, &w) && (!fast || w.windows == windows))
            sphase_scan_add(&s_sphase_last, sphase_metric(&w));
    }
    sphase_verdict_t v = SPHASE_UNSETTLED;
    const bool contrast = carrier ? sphase_carrier_contrast(&s_sphase_last) : sphase_contrast(&s_sphase_last);
    if (fast && (s_sphase_last.positions != positions || !contrast)) {
        /* Three alike positions are not enough to establish NOISE_LIMITED;
         * the caller repeats the original nine-position scan. */
    } else if (s_sphase_last.valid && !contrast) {
        /* Every position reads alike: intrinsic noise, slipping cannot help. */
        if (collect(windows, &w))
            v = carrier ? (sphase_record(&w) < SPHASE_LIVE_LIMIT ? SPHASE_SETTLED : SPHASE_NOISE_LIMITED)
                        : sphase_classify(sphase_record(&w), SPHASE_LIMIT, &s_sphase_last);
    } else {
        /* Positions repeat every three ticks: stop on one near the cleanest. */
        for (unsigned n = 0; s_sphase_last.valid && n < tries; ++n) {
            if (s_probe_deadline && (esp_timer_get_time() >= s_probe_deadline || !menu_splash_active())) break;
            if (!collect(windows, &w) || (fast && w.windows != windows)) break;
            int e = sphase_metric(&w);
            const int margin = carrier ? sphase_carrier_margin(s_sphase_last.best) : sphase_margin(s_sphase_last.best);
            if (e <= s_sphase_last.best + margin) {
                if (fast) {
                    const int first = e;
                    if (!collect(windows, &w) || w.windows != windows) break;
                    e = sphase_metric(&w);
                    if (e > s_sphase_last.best + margin || abs(e - first) > margin) break;
                }
                (void)sphase_record(&w);
                v = SPHASE_SETTLED;
                break;
            }
            if (!phase_slip()) break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    receiver_resume();
    printf("SPHASE scan tag=%s positions=%u best=%d worst=%d contrast=%u verdict=%s\n", tag,
           s_sphase_last.positions, s_sphase_last.best, s_sphase_last.worst, contrast, sphase_verdict_name(v));
    return v;
}

static sphase_verdict_t sphase_scan(void)
{
    const sphase_verdict_t quick = sphase_scan_n("SPHASE_QUICK", 3u, 48u, 3u, false);
    if (quick == SPHASE_SETTLED) return quick;
    return sphase_scan_n("SPHASE", SPHASE_POSITIONS, 48u, SPHASE_TRIES, false);
}

static sphase_live_t s_sphase_live = {.gap_us = SPHASE_LIVE_GAP_US};

static bool sphase_on_carrier(void)
{
    return sphase_live_carrier(video_recent(), s_current_gain, rf_gain_table()->max_index);
}

/* One carrier scan: choose the cleanest of the three positions. repair:
 * started by an excursion (not the first, proactive choice of the boot). */
static void sphase_carrier_scan(const char *stage, bool repair)
{
    ++s_sphase_scans;
    sphase_verdict_t v = sphase_scan_n(repair ? "SPHASE_REPAIR" : "SPHASE_CARRIER", 3u, 16u, 6u, true);
    const bool contrast = sphase_carrier_contrast(&s_sphase_last);
    sphase_live_done(&s_sphase_live, contrast, repair, esp_timer_get_time());
    s_sphase_state = v == SPHASE_SETTLED ? "settled" : v == SPHASE_NOISE_LIMITED ? "noise_limited" : "drifted";
    printf("SPHASE %s carrier_scan repair=%u verdict=%s contrast=%u excess_mc2=%d best=%d worst=%d scans=%u "
           "moved=%u false_alarms=%u next_gap_s=%lld\n", stage, repair, sphase_verdict_name(v), contrast,
           s_sphase_excess, s_sphase_last.best, s_sphase_last.worst, s_sphase_live.scans, s_sphase_live.moved,
           s_sphase_live.false_alarms, s_sphase_live.gap_us / 1000000LL);
}

/* Measure, classify and (startup only) repair with at most two scans. A
 * fresh context drops the previous scan: its positions were measured under
 * another TX/noise context and are no evidence here. */
static void sphase_qualify(const char *stage, bool fresh_context)
{
    window_stats_t w;
    if (fresh_context) sphase_scan_reset(&s_sphase_last);
    dco_hold_for_scan();
    if (!collect(48, &w)) {
        s_sphase_state = "probe_failed";
        printf("SPHASE %s state=probe_failed\n", stage);
        return;
    }
    sphase_verdict_t v = sphase_classify(sphase_record(&w), SPHASE_LIMIT, &s_sphase_last);
    for (unsigned n = 0; v == SPHASE_UNSETTLED && n < 2u; ++n) {
        ++s_sphase_scans;
        v = sphase_scan();
    }
    s_sphase_state = sphase_verdict_name(v);
    printf("SPHASE %s state=%s excess_mc2=%d white_pm=%d dc_mcell=%d/%d scans=%u best=%d worst=%d\n",
           stage, s_sphase_state, s_sphase_excess, s_sphase_white, s_sphase_dc[0], s_sphase_dc[1],
           s_sphase_scans, s_sphase_last.valid ? s_sphase_last.best : -1, s_sphase_last.valid ? s_sphase_last.worst : -1);
}

const char *calibration_sphase_state(void) { return s_sphase_state; }
/* NOISE_LIMITED is not a phase fault: every position reads the same noise. */
bool calibration_sphase_settled(void)
{
    return !strcmp(s_sphase_state, "settled") || !strcmp(s_sphase_state, "noise_limited");
}

/* ---- Narrow analog filter --------------------------------------------------
 * The FM threshold follows the pre-detection noise bandwidth, and PARLIO
 * keeps every second sample of the ~80 MS/s bus unfiltered, so the noise
 * the demod sees is set by the analog filter, alias included. The
 * capacitor codes are per-chip calibrated, so the filter is measured, not
 * assumed: with no carrier at maximum gain every setting (digital BW40/
 * BW20 x RX0 code x second-stage skirt) gets a 64-point noise spectrum, and
 * the one with the lowest noise bandwidth whose -3 dB width still covers
 * 14 MHz (enough for every tested VTX deviation, docs/FINDINGS.md) and whose
 * noise stays incoherent (so Direct Gain still recognises "no carrier")
 * is kept. Built on ESPARGOS esp-sdr's C5 BANDWIDTH control (GPL-3.0). */
/* 3: wide class; 4: codes 52/62/63 (one new sweep after the update);
 * 5: settings measured in the analog branch they fly in (rf_set_filter sets
 * the bandwidth mode up before measuring; every earlier sweep mixed the two
 * branches, board 2026-10-19: chosen 12.5 MHz flew at 19.7 MHz) */
#define FILTER_VERSION     6u  /* 6: 11p baseline plus measured analog profiles */
/* Two classes from one sweep (research: 12 MHz helps every tested VTX
 * deviation and keeps strong-signal sharpness; 10 MHz adds up to ~1.7 dB
 * for deviation <= ~1.1 but hurts wide VTXs, so it is used only when the
 * deviation was measured). */
#define FILTER_MIN_KHZ     12000u
#define FILTER_NARROW_KHZ  10000u
/* Wide VTXs (deviation >= 1.6): the 12 MHz class clips their spectrum
 * (research/data/wide.txt). */
#define FILTER_WIDE_KHZ    16000u
/* Quiet test: dsp_quiet_reason() (dsp.h). */
#define QUIET_NO_SYNC_US   2000000LL
/* s_last_video_us starts at 0: "no sync seen yet" is not recent sync. */
static bool video_recent(void)
{
    const int64_t t = s_last_video_us;
    return t != 0 && esp_timer_get_time() - t < QUIET_NO_SYNC_US;
}
static unsigned quiet_limit(int clip_pm) { return dsp_quiet_limit(clip_pm); }
#define CAL_WINDOWS        96u
typedef struct {
    uint8_t valid, bw20, skirt;
    int8_t code;
    uint16_t width_khz, nbw_khz;
} filter_entry_t;
typedef struct {
    uint8_t version;
    filter_entry_t normal, narrow, wide;
    uint16_t calibrated_nbw_khz;
} filter_blob_t;
static filter_blob_t s_filter_cal;
static uint16_t s_original_mhz;
static const char *s_filter_result = "never";

/* Accumulate in-place: a promoted setting keeps its screening FFTs and
 * extends to all 96 windows, with no retune or extra settling delay. */
static bool noise_collect(cal_noise_t *a, unsigned windows)
{
    uint8_t sample[PROBE_BYTES];
    for (unsigned tries = 0; tries < windows * 3u && a->got < windows; ++tries) {
        vTaskDelay(1);   /* retain the existing filter collector cadence */
        if (!copy_probe_window(sample, NULL)) continue;
        cal_noise_add(a, sample, sizeof(sample), measure_window(sample, sizeof(sample)).clip_pm);
    }
    return a->got == windows;
}

/* On-screen progress: the measured time per setting gives the time left. */
static unsigned s_cal_done, s_cal_total;
/* Higher code = narrower. Board sweep (FINDINGS, 2026-10-12): the narrow
 * class landed on code 60, the old grid's last code, at 11.9 MHz noise
 * bandwidth instead of the modelled 10 MHz; the field takes 0..63, so 62
 * and 63 are tried too, and 52 fills the gap around the 12 MHz class. */
#define FILTER_CODES  13u
#define FILTER_SKIRTS 5u
static int64_t s_cal_t0;
static bool s_cal_own_channel;
static void cal_status(void)
{
    char line1[48], line2[48];
    int64_t spent = esp_timer_get_time() - s_cal_t0;
    unsigned left = s_cal_done ? (unsigned)(spent / s_cal_done * (s_cal_total - s_cal_done) / 1000000) + 1u : 30u;
    if (s_cal_own_channel) snprintf(line1, sizeof(line1), "KEEP THE VTX OFF");
    else snprintf(line1, sizeof(line1), "ON QUIET CHANNEL %s (VTX MAY STAY ON)", rf_channel()->name);
    snprintf(line2, sizeof(line2), "ABOUT %u S LEFT, THEN LIVE VIDEO", left);
    status_screen("CALIBRATING", line1, line2, (int)(s_cal_done * 100u / s_cal_total));
}

/* *quiet stays false for a setting the PHY did not read back, so it can
 * never be chosen; false only when the probe itself fails. */
static bool measure_setting(rf_filter_t f, unsigned *width, unsigned *nbw, bool *quiet, bool *carrier,
                            const unsigned best_nbw[3], bool *screen_fault)
{
    if (s_cal_total) { ++s_cal_done; cal_status(); }
    *quiet = *carrier = false;
    if (!rf_set_filter(f)) {
        printf("FILTER_CAL bw=%s code=%d skirt=%u skip=not_applied\n", f.bw20 ? "BW20" : "BW40", f.code, f.skirt);
        *width = *nbw = 0;
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    unsigned env = 0;
    int clip = 0;
    cal_noise_t a = {0};
    if (best_nbw) {
        if (!noise_collect(&a, 24u)) return false;
        const unsigned sw = dsp_psd_width_khz(a.psd), sn = dsp_psd_nbw_khz(a.psd);
        env = a.env_sum / a.got; clip = a.clip_sum / (int)a.got;
        if (cal_setting_carrier(env, clip)) { *carrier = true; return true; }
        const bool contender = !cal_setting_quiet(env, clip) ||
            cal_filter_contender(sw, sn, FILTER_MIN_KHZ, best_nbw[0]) ||
            cal_filter_contender(sw, sn, FILTER_NARROW_KHZ, best_nbw[1]) ||
            cal_filter_contender(sw, sn, FILTER_WIDE_KHZ, best_nbw[2]);
        if (!contender && sn) {
            *width = *nbw = 0; /* screened out, never eligible for storage */
            printf("FILTER_CAL screen=reject width_khz=%u nbw_khz=%u\n", sw, sn);
            return true;
        }
        if (!noise_collect(&a, CAL_WINDOWS) || !cal_noise_finish(&a, width, nbw, &env, &clip)) return false;
        if (!cal_filter_screen_agrees(sw, sn, *width, *nbw)) *screen_fault = true;
    } else if (!noise_collect(&a, CAL_WINDOWS) || !cal_noise_finish(&a, width, nbw, &env, &clip)) return false;
    /* A setting with clipped noise is just not chosen; only a clearly
     * non-noise envelope means a carrier appeared. */
    *quiet = cal_setting_quiet(env, clip);
    *carrier = cal_setting_carrier(env, clip);
    printf("FILTER_CAL bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u env_x100=%u limit=%u clip_pm=%d\n",
           f.bw20 ? "BW20" : "BW40", f.code, f.skirt, *width, *nbw, env, quiet_limit(clip), clip);
    return true;
}

/* A channel is quiet when it shows receiver noise at maximum gain: no
 * burst-confirmed sync, noise-like envelope, not dead or railing IQ. */
static const char *noise_check(unsigned *env_out, int *clip_out);
static bool channel_quiet(void)
{
    unsigned env = 0;
    int clip = 0;
    const char *why = video_recent() ? "video_sync" : noise_check(&env, &clip);
    printf("FILTER_CAL channel=%s freq=%u gain=%u quiet=%u reason=%s env_x100=%u clip_pm=%d\n", rf_channel()->name,
           rf_frequency_mhz(), s_current_gain, why == NULL, why ? why : "-", env, clip);
    return why == NULL;
}

/* The filter is the chip's own baseband filter: its noise bandwidth does
 * not depend on the channel. With a VTX on the current channel the sweep
 * runs on the quietest other channel instead and returns afterwards. */
static bool find_quiet_channel(size_t original)
{
    if (channel_quiet()) return true;
    /* Channels within 40 MHz are skipped: a VTX leaks into its neighbours. */
    for (size_t k = 0; k < FPV_CHANNELS; ++k) {
        size_t ch = (original + FPV_CHANNELS / 2 + k) % FPV_CHANNELS;
        if (ch == original) continue;
        if (abs((int)rf_channel_freq(ch) - (int)s_original_mhz) < 40) continue;
        if (rf_set_channel(ch) != ESP_OK) continue;
        receiver_set_gain(rf_gain_table()->max_index);
        vTaskDelay(pdMS_TO_TICKS(50));
        if (channel_quiet()) return true;
    }
    return false;
}

bool filter_calibrate(bool automatic)
{
    if (phy_filter_calibrated_code() < 0) { s_filter_result = "unsupported_phy"; return false; }
    if (!receiver_pause("FILTER_CAL")) { s_filter_result = "busy"; return false; }
    const uint8_t saved_gain = s_current_gain;
    const size_t original = rf_channel_index();
    const int original_offset = rf_offset_khz();
    s_original_mhz = rf_frequency_mhz();
    const rf_filter_t previous = rf_filter(), calibrated = {PHY_FILTER_CALIBRATED, 0, false};
    s_scan_active = true;              /* control loop and AFC stand aside */
    bool ok = rf_set_filter(calibrated);
    receiver_set_gain(rf_gain_table()->max_index);
    vTaskDelay(pdMS_TO_TICKS(50));
    status_screen("CALIBRATING", "LOOKING FOR A QUIET CHANNEL", "ONE TIME ONLY, ABOUT 30 S", -1);
    const char *fail = ok ? NULL : "calibrated_bytes_not_applied";
    if (ok && !(ok = find_quiet_channel(original))) fail = "no_quiet_channel";
    s_cal_own_channel = rf_channel_index() == original;
    s_cal_done = 0;
    s_cal_total = ok ? 2u + 2u * FILTER_CODES * FILTER_SKIRTS : 0u;   /* checks + BW40/BW20 x codes x skirts */
    s_cal_t0 = esp_timer_get_time();
    printf("FILTER_CAL begin mode=%s freq=%u (own %u) target_width_khz=%u\n", automatic ? "auto" : "manual",
           rf_frequency_mhz(), s_original_mhz, FILTER_MIN_KHZ);
    unsigned width = 0, nbw = 0, cal_nbw = 0;
    bool quiet = false, carrier = false;
    if (ok && !(ok = measure_setting(calibrated, &width, &cal_nbw, &quiet, &carrier, NULL, NULL) && quiet))
        fail = carrier ? "carrier_detected" : "not_quiet";
    rf_filter_t best = calibrated, narrow = calibrated;
    unsigned best_nbw = ok && width >= FILTER_MIN_KHZ ? cal_nbw : 0, best_width = width;
    unsigned narrow_nbw = ok && width >= FILTER_NARROW_KHZ ? cal_nbw : 0, narrow_width = width;
    rf_filter_t wide = calibrated;
    unsigned wide_nbw = ok && width >= FILTER_WIDE_KHZ ? cal_nbw : 0, wide_width = width;
    static const int8_t codes[FILTER_CODES] = {PHY_FILTER_CALIBRATED, 0, 8, 16, 24, 32, 40, 48, 52, 56, 60, 62, 63};
    static const uint8_t skirts[FILTER_SKIRTS] = {0, 8, 16, 24, 32};
    /* First pass screens all 130 settings. Any screen/full discrepancy
     * outside the conservative guard invalidates the pruning evidence:
     * restart exhaustively, including every previously rejected setting. */
    bool screen_fault = false;
    for (unsigned pass = 0; ok && pass < 2u; ++pass) {
        for (unsigned d = 0; ok && d < 2u; ++d)
            for (unsigned c = 0; ok && c < sizeof(codes); ++c)
                for (unsigned k = 0; ok && k < sizeof(skirts); ++k) {
                    rf_filter_t f = {codes[c], skirts[k], d == 1u};
                    const unsigned incumbents[3] = {best_nbw, narrow_nbw, wide_nbw};
                    if (!measure_setting(f, &width, &nbw, &quiet, &carrier,
                                         pass == 0u ? incumbents : NULL, &screen_fault)) { ok = false; fail = "probe_failed"; break; }
                    if (carrier) { ok = false; fail = "carrier_detected"; break; }   /* appeared mid-sweep */
                    if (quiet && nbw && width >= FILTER_MIN_KHZ && (!best_nbw || nbw < best_nbw)) {
                        best = f; best_nbw = nbw; best_width = width;
                    }
                    if (quiet && nbw && width >= FILTER_NARROW_KHZ && (!narrow_nbw || nbw < narrow_nbw)) {
                        narrow = f; narrow_nbw = nbw; narrow_width = width;
                    }
                    if (quiet && nbw && width >= FILTER_WIDE_KHZ && (!wide_nbw || nbw < wide_nbw)) {
                        wide = f; wide_nbw = nbw; wide_width = width;
                    }
                }
        if (!screen_fault || !ok || pass == 1u) break;
        printf("FILTER_CAL screen=fallback_full\n");
        best = narrow = wide = calibrated;
        best_nbw = best_width = narrow_nbw = narrow_width = wide_nbw = wide_width = 0;
        s_cal_total += 2u * FILTER_CODES * FILTER_SKIRTS;
    }
    /* Re-check on the calibrated bytes before trusting the sweep. */
    if (ok && !(ok = measure_setting(calibrated, &width, &nbw, &quiet, &carrier, NULL, NULL) && quiet))
        fail = carrier ? "carrier_detected" : "recheck_not_quiet";
    /* Independent full rechecks of each distinct winner. Screening never
     * relaxes the stored width requirement, quiet gate or final precision. */
    rf_filter_t winners[3] = {best, narrow, wide};
    unsigned *widths[3] = {&best_width, &narrow_width, &wide_width};
    unsigned *nbws[3] = {&best_nbw, &narrow_nbw, &wide_nbw};
    static const unsigned minimum[3] = {FILTER_MIN_KHZ, FILTER_NARROW_KHZ, FILTER_WIDE_KHZ};
    for (unsigned k = 0; ok && k < 3u; ++k) {
        if (!*nbws[k]) continue;
        bool duplicate = false;
        for (unsigned j = 0; j < k; ++j)
            if (*nbws[j] && rf_filter_equal(winners[j], winners[k])) {
                *widths[k] = *widths[j]; *nbws[k] = *nbws[j]; duplicate = true; break;
            }
        if (!duplicate) {
            ++s_cal_total;
            if (!measure_setting(winners[k], &width, &nbw, &quiet, &carrier, NULL, NULL) || !quiet ||
                !cal_filter_screen_agrees(*widths[k], *nbws[k], width, nbw)) {
                ok = false; fail = "winner_recheck_failed"; break;
            }
            *widths[k] = width; *nbws[k] = nbw;
        }
        if (*widths[k] < minimum[k]) { ok = false; fail = "winner_width_failed"; }
    }
    /* The normal class must be a measured setting: a noise bandwidth of 0
     * means nothing reached the minimum width. */
    if (ok && !best_nbw) { ok = false; fail = "no_wide_setting"; }
    /* Back to the exact original tuning; a failed retune is retried and
     * reported, never assumed. */
    bool restored = rf_channel_index() == original;
    for (unsigned t = 0; !restored && t < 3u; ++t) {
        restored = rf_set_channel(original) == ESP_OK;
        if (!restored) vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!restored) printf("FILTER_CAL error: retune to %u MHz failed\n", s_original_mhz);
    if (rf_offset_khz() != original_offset) rf_set_offset_khz(original_offset);
    if (rf_offset_khz() != original_offset) {
        printf("FILTER_CAL error: offset %d kHz not restored (now %d)\n", original_offset, rf_offset_khz());
        restored = false;
    }
    /* Nothing is stored unless the receiver is back on its own tuning: a
     * sweep that ends elsewhere is discarded and tried on the next boot. */
    if (ok && !restored) { ok = false; fail = "restore_failed"; }
    if (ok) {
#define KHZ16(v) ((uint16_t)((v) > 65535u ? 65535u : (v)))
        const filter_blob_t cal = {FILTER_VERSION,
            {1u, best.bw20, best.skirt, best.code, KHZ16(best_width), KHZ16(best_nbw)},
            {narrow_nbw ? 1u : 0u, narrow.bw20, narrow.skirt, narrow.code, KHZ16(narrow_width), KHZ16(narrow_nbw)},
            {wide_nbw ? 1u : 0u, wide.bw20, wide.skirt, wide.code, KHZ16(wide_width), KHZ16(wide_nbw)},
            KHZ16(cal_nbw)};
        if (!rf_set_filter(best)) { ok = false; fail = "best_not_applied"; }
        else if (!blob_store("filter", &cal, sizeof(cal))) { ok = false; fail = "nvs_failed"; }
        else s_filter_cal = cal;
    }
    if (ok) {
        s_filter_result = "stored";
        printf("FILTER_CAL normal bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u | narrow bw=%s code=%d "
               "skirt=%u width_khz=%u nbw_khz=%u | wide bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u | "
               "calibrated_nbw_khz=%u\n", best.bw20 ? "BW20" : "BW40",
               best.code, best.skirt, best_width, best_nbw, narrow.bw20 ? "BW20" : "BW40", narrow.code,
               narrow.skirt, narrow_width, narrow_nbw, wide.bw20 ? "BW20" : "BW40", wide.code, wide.skirt,
               wide_width, wide_nbw, cal_nbw);
    } else {
        if (!rf_set_filter(previous)) printf("FILTER_CAL error: previous filter not applied\n");
        s_filter_result = fail ? fail : "sweep_failed";
        printf("FILTER_CAL aborted=%s\n", s_filter_result);
    }
    printf("CAL_STEP step=filter ms=%lld settings=%u result=%s\n",
           (esp_timer_get_time() - s_cal_t0) / 1000, s_cal_done, ok ? "ok" : "failed");
    s_cal_total = 0;
    status_screen(ok ? "FILTER CALIBRATED" : "FILTER NOT CALIBRATED", ok ? "NARROW FILTER STORED" : s_filter_result,
                  ok ? "" : "USE MENU > CALIBRATE", ok ? 100 : -1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    receiver_set_gain(saved_gain);
    s_scan_active = false;
    receiver_resume();
    return ok;
}

const char *calibration_filter_result(void) { return s_filter_result; }

/* Width class of the active analog filter, for model tables keyed on it
 * (cn_estimate.h): 10 narrow, 12 normal, 16 wide, 20 the chip's own ~20 MHz. */
int calibration_filter_class_mhz(void)
{
    rf_filter_t f, now = rf_filter();
    /* The sweep can select the same setting for normal and narrow (board:
     * both 13.125 MHz). Prefer normal, and do not call a measured >11 MHz
     * narrow entry 10 MHz. */
    if (s_filter_cal.normal.valid && filter_setting(FILTER_NORMAL, &f) && rf_filter_equal(f, now))
        return s_filter_cal.normal.width_khz < 11000u ? 10 : s_filter_cal.normal.width_khz >= 15000u ? 16 : 12;
    if (filter_setting(FILTER_NARROW, &f) && rf_filter_equal(f, now))
        return s_filter_cal.narrow.width_khz > 11000u ? 12 : 10;
    if (filter_setting(FILTER_WIDE, &f) && rf_filter_equal(f, now)) return 16;
    return 20;
}

/* Measured noise bandwidth of the active filter (the startup sweep), 0
 * when unknown: C/N-AUTO converts to the 20 MHz scale with it. */
unsigned calibration_filter_nbw_khz(void)
{
    rf_filter_t f, now = rf_filter();
    if (filter_setting(FILTER_NORMAL, &f) && rf_filter_equal(f, now) && s_filter_cal.normal.valid) return s_filter_cal.normal.nbw_khz;
    if (filter_setting(FILTER_NARROW, &f) && rf_filter_equal(f, now) && s_filter_cal.narrow.valid) return s_filter_cal.narrow.nbw_khz;
    if (filter_setting(FILTER_WIDE, &f) && rf_filter_equal(f, now) && s_filter_cal.wide.valid) return s_filter_cal.wide.nbw_khz;
    return s_filter_cal.calibrated_nbw_khz;
}

unsigned calibration_narrow_width_khz(void)
{
    return s_filter_cal.narrow.valid ? s_filter_cal.narrow.width_khz : 0u;
}

bool filter_setting(int filter_class, rf_filter_t *out)
{
    const filter_entry_t *e = filter_class == FILTER_NARROW ? &s_filter_cal.narrow :
                              filter_class == FILTER_WIDE ? &s_filter_cal.wide : &s_filter_cal.normal;
    if (s_filter_cal.version != FILTER_VERSION || !e->valid) {
        if (filter_class == FILTER_NORMAL) { *out = (rf_filter_t){16, 32, false}; return true; }
        return false;
    }
    if (!e->nbw_khz) return false;      /* never measured: not a usable class */
    *out = (rf_filter_t){e->code, e->skirt, e->bw20 != 0};
    return true;
}

/* Manual request (menu or console): runs the startup phase once more, in
 * this task (the console keeps draining USB). */
static volatile bool s_manual_requested, s_full_requested;
void filter_request(void)
{
    if (s_startup) { printf("COMMAND F refused=busy\n"); return; }
    s_manual_requested = true;
}
void filter_request_full(void)
{
    if (s_startup) { printf("COMMAND B refused=busy\n"); return; }
    s_full_requested = true; s_manual_requested = true;
}

/* ---- Per-gain RX DC correction --------------------------------------------
 * Near the range edge the carrier is a few cells; a receiver DC of ~1 cell
 * biases every origin-adjacent phase. The DC DAC pair is searched per gain
 * row in confirmed quiet (no burst-confirmed sync for 3 s, a noise-like
 * envelope for 5 s, never on a channel where a carrier was seen this boot)
 * and held right after every gain write by the post-gain hook (board,
 * 2026-10-07: DC median ~134 LSB without the hook, ~11 with it). */
#define DCO_VERSION      8u   /* 8: 11p RF/filter context */
typedef struct {
    int16_t code[2];
    int16_t residual_mcells[2];
    uint8_t valid;           /* DCO_ROW_* */
} dco_entry_t;
/* A row is OK (codes measured) or FLOOR (measured: receiver noise below
 * one quantizer step there, nothing to correct); 0 = still missing. */
#define DCO_ROW_OK    1u
#define DCO_ROW_FLOOR 2u
/* A row whose search left more than this is not converged (clipped start,
 * see phy_dco_search_from): never OK, never a template, searched again from
 * a converged neighbour. The live servo trims what a good row leaves. */
#define DCO_BAD_MCELLS 1200
static bool dco_entry_bad(const dco_entry_t *e)
{
    return e->valid == 1u && (abs(e->residual_mcells[0]) > DCO_BAD_MCELLS || abs(e->residual_mcells[1]) > DCO_BAD_MCELLS);
}
static uint32_t s_dco_bad_dropped, s_dco_repairs, s_dco_repair_fails;
typedef struct {
    uint16_t freq;
    uint8_t version, lo, hi, band5, iq_scale_sel, filter_bw20;
    int8_t filter_code, filter_skirt;
    dco_entry_t e[ARC_VENDOR_GAIN_MAX + 1u];
} dco_table_t;
/* CPU-only state in LP RAM: internal SRAM is the menu's DMA heap. */
static RTC_FAST_ATTR dco_table_t s_dco;
static RTC_FAST_ATTR int64_t s_dco_found_us[ARC_VENDOR_GAIN_MAX + 1u];
/* One calibration-task scratch buffer: the noise envelope probe (startup,
 * CALIBRATE, DC searches) and the live DC trim window never run nested and
 * never on another task. Sharing it frees 4092 bytes of RTC_FAST, which
 * doubles as boot heap. */
static RTC_FAST_ATTR uint8_t s_cal_scratch[24u * PROBE_BYTES];
_Static_assert(sizeof(s_cal_scratch) >= 4092u, "DC trim needs one control window");
#define s_env_buf s_cal_scratch
static uint8_t s_dco_banned[ARC_VENDOR_GAIN_MAX + 1u];
/* A ban expires: released for good, a row that railed once (often its own
 * DC, which the servo would have fixed) stayed on the chip's DC with the
 * servo skipping it (board 2026-10-20: G83 banned, 4 cells of DC, every
 * probe clipped, KEEL holding G83 on one-sided clipping: a deadlock). The
 * hold is tried again after DCO_BAN_US, doubling per repeat up to 32 s. */
#define DCO_BAN_US     2000000LL
#define DCO_BAN_MAX_US 32000000LL
static int64_t s_dco_ban_until[ARC_VENDOR_GAIN_MAX + 1u], s_dco_ban_len[ARC_VENDOR_GAIN_MAX + 1u];
static uint32_t s_dco_unbans;
static uint8_t s_dco_floor;
static bool s_dco_dirty;
static uint32_t s_dco_searches, s_dco_hook_holds, s_dco_aborts, s_dco_carrier_searches;

static void dco_context(dco_table_t *t, uint16_t freq)
{
    const arc_gain_table_t *table = rf_gain_table();
    rf_filter_t f = rf_filter();
    t->version = DCO_VERSION;
    t->freq = freq;
    t->lo = arc_gain_highest_rf_stage_start(table);
    t->hi = table->max_index;
    t->band5 = table->band5;
    t->iq_scale_sel = phy_param[650];   /* phy_rxiq_scale_set() selector */
    t->filter_code = f.code;
    t->filter_skirt = (int8_t)f.skirt;
    t->filter_bw20 = f.bw20;
}

/* Only the header of the wanted context is ever compared, so it is no
 * longer a full 910-byte table (it lived in RTC_FAST to stay off the task
 * stack: two full tables there took 1840 bytes before NVS frames). */
typedef struct {
    uint16_t freq;
    uint8_t lo, hi, band5, iq_scale_sel;
    uint16_t key;            /* dco_filter_key() of the live filter */
} dco_want_t;
static uint16_t live_filter_key(void)
{
    const rf_filter_t f = rf_filter();
    return dco_filter_key(f.code, (int)f.skirt, f.bw20);
}
static dco_want_t dco_want_now(void)
{
    /* The compared fields of dco_context(), without a table on the stack. */
    const arc_gain_table_t *table = rf_gain_table();
    return (dco_want_t){rf_frequency_mhz(), arc_gain_highest_rf_stage_start(table), table->max_index,
                        table->band5, phy_param[650], live_filter_key()};
}
/* The header without the filter: a table of another filter can still seed
 * this one (its rows then stay template rows until measured here). Which
 * filter a table serves is s_dco_serving; its own filter fields say what it
 * was measured with (dco_startup re-measures when they differ). */
static bool dco_matches(const dco_table_t *t, const dco_want_t *w)
{
    return t->version == DCO_VERSION && t->freq == w->freq && t->lo == w->lo && t->hi == w->hi &&
           t->band5 == w->band5 && t->iq_scale_sel == w->iq_scale_sel;
}

/* Per-channel tables (dco_slots.h): slot blobs "dco0".."dco5" plus the
 * index "dco_idx". The single legacy "dco" table is adopted once. */
static dco_index_t s_dco_index;
static bool s_dco_index_loaded;
static uint16_t s_dco_seed_mhz;     /* the current table was seeded from this channel, 0 = own */
static uint32_t s_dco_seeds;
static dco_mask_t s_dco_templ;      /* rows of the current table that come from a template */
static uint16_t s_dco_serving;      /* dco_filter_key() of the filter the current table is for */
static bool s_dco_serving_valid;
static int s_drift_offset[ARC_VENDOR_GAIN_MAX + 1u][2];   /* live trim per row (dco_post_gain) */
static bool s_drift_set[ARC_VENDOR_GAIN_MAX + 1u];          /* the row has its own trim */
static uint32_t s_dco_reloads;      /* dco_select() loads of another table (each drops the trims) */
static uint8_t s_dco_reload_why;    /* 1 none yet, 2 filter, 3 freq, 4 rows, 5 band, 6 IQ scale, 7 version */

static unsigned dco_template_rows(void) { return dco_mask_count(s_dco_templ, s_dco.lo, s_dco.hi); }

/* A row measured on the own channel is no longer a template row. */
static void dco_row_measured(unsigned g) { dco_mask_put(s_dco_templ, g, false); }

/* Rows stored before this check: a bad row becomes missing on load. */
static unsigned dco_drop_bad(dco_table_t *t)
{
    unsigned n = 0;
    for (unsigned g = t->lo; g <= t->hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        if (dco_entry_bad(&t->e[g])) { t->e[g].valid = 0; ++n; }
    s_dco_bad_dropped += n;
    return n;
}

static void dco_slot_key(unsigned slot, char key[8]) { snprintf(key, 8, "dco%u", slot); }

static bool dco_load_slot(unsigned slot, dco_table_t *out)
{
    char key[8];
    dco_slot_key(slot, key);
    return blob_load(key, out, sizeof(*out)) && out->version == DCO_VERSION && out->freq == s_dco_index.freq[slot];
}

static bool dco_save(void)
{
    char key[8];
    const uint16_t filt = s_dco_serving_valid ? s_dco_serving :
                          dco_filter_key(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20);
    const unsigned slot = dco_slot_for_store(&s_dco_index, s_dco.freq, filt);
    dco_slot_key(slot, key);
    if (!blob_store(key, &s_dco, sizeof(s_dco))) return false;
    dco_index_touch(&s_dco_index, slot, s_dco.freq, filt);
    memcpy(s_dco_index.templ[slot], s_dco_templ, sizeof(s_dco_templ));
    if (!blob_store("dco_idx", &s_dco_index, sizeof(s_dco_index))) return false;
    s_dco_seed_mhz = 0;
    return true;
}

static void dco_index_load(void)
{
    if (s_dco_index_loaded) return;
    s_dco_index_loaded = true;
    if (blob_load("dco_idx", &s_dco_index, sizeof(s_dco_index)) && s_dco_index.version == DCO_INDEX_VERSION) return;
    dco_index_init(&s_dco_index);
    /* Version 2 (one table per frequency): keep the measured tables, each
     * under the filter its header records. */
    dco_index_v2_t v2;
    if (blob_load("dco_idx", &v2, sizeof(v2)) && v2.version == 2u) {
        for (unsigned k = 0; k < DCO_SLOTS; ++k) {
            s_dco_index.freq[k] = v2.freq[k];
            s_dco_index.seq[k] = v2.seq[k];
            memcpy(s_dco_index.templ[k], v2.templ[k], sizeof(dco_mask_t));
            if (v2.freq[k] && dco_load_slot(k, &s_dco))
                s_dco_index.filt[k] = dco_filter_key(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20);
            else
                s_dco_index.freq[k] = 0;
        }
        s_dco_index.next_seq = v2.next_seq;
        memset(&s_dco, 0, sizeof(s_dco));
        if (blob_store("dco_idx", &s_dco_index, sizeof(s_dco_index))) printf("DCO index upgraded to per-filter slots\n");
        return;
    }
    /* Adopt the single table of earlier builds as the first slot. */
    memset(s_dco_templ, 0, sizeof(s_dco_templ));   /* measured on its own channel */
    if (blob_load("dco", &s_dco, sizeof(s_dco)) && s_dco.version == DCO_VERSION && s_dco.freq) {
        s_dco_serving_valid = false;   /* dco_save() keys it by its own filter fields */
        if (dco_save()) printf("DCO adopted the legacy table freq=%u as slot 0\n", s_dco.freq);
    }
}

static void dco_select(void)
{
    const dco_want_t want = dco_want_now();
    if (s_dco_serving_valid && s_dco_serving == want.key && dco_matches(&s_dco, &want)) return;
    (void)phy_dco_release();
    memset(s_dco_found_us, 0, sizeof(s_dco_found_us));
    memset(s_dco_banned, 0, sizeof(s_dco_banned));
    memset(s_dco_ban_until, 0, sizeof(s_dco_ban_until));
    memset(s_dco_ban_len, 0, sizeof(s_dco_ban_len));
    s_dco_floor = 0;
    s_dco_seed_mhz = 0;
    dco_index_load();
    memset(s_dco_templ, 0, sizeof(s_dco_templ));
    const uint16_t freq = want.freq;
    ++s_dco_reloads;
    s_dco_reload_why = !s_dco_serving_valid ? 1u : s_dco_serving != want.key ? 2u : s_dco.freq != want.freq ? 3u :
                       s_dco.lo != want.lo || s_dco.hi != want.hi ? 4u : s_dco.band5 != want.band5 ? 5u :
                       s_dco.iq_scale_sel != want.iq_scale_sel ? 6u : 7u;
    s_dco_serving = want.key;
    s_dco_serving_valid = true;
    memset(s_drift_offset, 0, sizeof(s_drift_offset));   /* trims belong to the table they were made on */
    receiver_dc_reset_all();
    memset(s_drift_set, 0, sizeof(s_drift_set));
    const int own = dco_slot_of(&s_dco_index, freq, want.key);
    if (own >= 0 && dco_load_slot((unsigned)own, &s_dco) && dco_matches(&s_dco, &want)) {
        memcpy(s_dco_templ, s_dco_index.templ[own], sizeof(s_dco_templ));
        if (dco_drop_bad(&s_dco)) printf("DCO table freq=%u: unconverged rows dropped (searched again)\n", freq);
        return;
    }
    /* No own table: start from the nearest one, the same filter first. */
    const int near = dco_nearest_slot(&s_dco_index, freq, want.key);
    if (near >= 0 && dco_load_slot((unsigned)near, &s_dco)) {
        const uint16_t from = s_dco.freq;
        s_dco.freq = freq;
        (void)dco_drop_bad(&s_dco);
        if (dco_matches(&s_dco, &want)) {
            s_dco_seed_mhz = from;
            ++s_dco_seeds;
            unsigned ok = 0;
            for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g) {
                dco_mask_put(s_dco_templ, g, s_dco.e[g].valid != 0);
                ok += s_dco.e[g].valid != 0;
            }
            printf("DCO template freq=%u from=%u rows=%u (nearest table, same_filter=%u; replaced by own measurements)\n",
                   freq, from, ok, s_dco_index.filt[near] == want.key);
            return;
        }
    }
    memset(&s_dco, 0, sizeof(s_dco));
    dco_context(&s_dco, freq);
}

/* Live trim on top of the stored codes, per row, for this table (a gain
 * write used to re-hold the bare table codes and drop it: with Direct Gain
 * moving between G82 and G83 every trim was undone within a second). */
/* A row without its own trim takes the nearest trimmed row's: the DC shift
 * after a channel setup (a profile switch) is common to the rows. Board
 * 2026-10-19, weak VTX after the live switch to MATCHED: G71 trimmed +8/+13
 * to 0.3 cells, while G77, never trimmed, sat at 3.9/3.5 cells for minutes
 * as Direct Gain moved between them, and the picture was gone. */
static int dco_offset(unsigned g, unsigned axis)
{
    if (g > ARC_VENDOR_GAIN_MAX) return 0;
    if (s_drift_set[g]) return s_drift_offset[g][axis];
    for (unsigned d = 1; d <= ARC_VENDOR_GAIN_MAX; ++d) {
        if (g >= d && s_drift_set[g - d]) return s_drift_offset[g - d][axis];
        if (g + d <= ARC_VENDOR_GAIN_MAX && s_drift_set[g + d]) return s_drift_offset[g + d][axis];
    }
    return 0;
}

static int dco_trimmed(unsigned g, unsigned axis)
{
    const int v = s_dco.e[g].code[axis] + dco_offset(g, axis);
    return v < 0 ? 0 : v > 511 ? 511 : v;
}

/* The row whose codes gain g holds: its own, or for a missing row the
 * nearest converged one within 6 rows (the codes of neighbouring rows differ
 * by a few; the chip's own codes left 4+ cells at G83 on 5805 MHz).
 * -1 = nothing to hold. */
#define DCO_BORROW_ROWS 6
static uint32_t s_dco_borrowed;
static int dco_hold_row(uint8_t g)
{
    if (g < s_dco.lo || g > s_dco.hi || s_dco_banned[g] || s_dco.freq != rf_frequency_mhz()) return -1;
    if (s_dco.e[g].valid == DCO_ROW_OK && !dco_mask_get(s_dco_templ, g)) return g;

    return -1;
}

void dco_post_gain(uint8_t g)
{
    if (receiver_paused()) return;
    const int r = dco_hold_row(g);
    if (r < 0) return;
    if (phy_dco_hold(dco_trimmed((unsigned)r, 0), dco_trimmed((unsigned)r, 1)) == ESP_OK) {
        ++s_dco_hook_holds;
        s_dco_borrowed += r != (int)g;
    }
}

/* Sampling-phase probes run with the gain worker paused, where the post-gain
 * hook stays silent: hold the current gain's stored row explicitly so the
 * scan sees the same DC as live video. */
static void dco_hold_for_scan(void)
{
    dco_select();
    const int r = dco_hold_row(s_current_gain);
    if (r < 0) return;
    (void)phy_dco_hold(dco_trimmed((unsigned)r, 0), dco_trimmed((unsigned)r, 1));   /* the live trim too */
}

/* Reuse the exact noise statistics of accepted, quiet DC verification
 * instead of immediately measuring the same gain/filter again. Volatile,
 * startup-only, exact-context and at most 30 s old; no NVS format change. */
#define NOISE_CACHE_LO 54u
#define NOISE_CACHE_ROWS (ARC_VENDOR_GAIN_MAX - NOISE_CACHE_LO + 1u)
static RTC_FAST_ATTR struct {
    int64_t time;
    uint16_t freq, key;
    float m2, kurt;
} s_noise_cache[NOISE_CACHE_ROWS];
static float s_noise_last_m2, s_noise_last_kurt;

static bool dco_measure(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect_stats(64, &w, true) || w.windows != 64u) return false;
    const double m2 = w.noise_m2 / w.windows;
    s_noise_last_kurt = m2 > 0 ? (float)((w.noise_m4 / w.windows) / (m2 * m2)) : 0;
    float mi, mq;
    double M2, M4;
    cna_mean(&w.noise_sums, &mi, &mq);
    cna_moments(&w.noise_sums, mi, mq, &M2, &M4);
    s_noise_last_m2 = (float)M2;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

/* Search at one quarter of the window count; the final callback above
 * still measures 64 windows. Preserve the board-proven two-tick IDLE yield
 * and 50 ms settling, rather than trading stability for a shorter delay. */
static bool dco_measure_fast(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect(16, &w) || w.windows != 16u) return false;
    dc[0] = w.dc_i; dc[1] = w.dc_q;
    return true;
}

/* Receiver noise: flat envelope statistic, and not dead or railing IQ
 * (which also has one; board 2026-10-07). NULL when it is noise, else the
 * reason. */
static const char *noise_check(unsigned *env_out, int *clip_out)
{
    unsigned got = 0;
    for (unsigned tries = 0; tries < 72u && got < 24u; ++tries) {
        vTaskDelay(1);
        if (copy_probe_window(s_env_buf + got * PROBE_BYTES, NULL)) ++got;
    }
    return cal_channel_quiet(s_env_buf, got, false, env_out, clip_out);
}

static bool noise_like(void) { return noise_check(NULL, NULL) == NULL; }

/* A search needs noise wide enough to dither the quantizer: >= 3 cells
 * per axis each with >= 5 % of the samples. A failed probe is not a floor:
 * it used to mark every lower row FLOOR in NVS and stop the DC bar half way. */
enum { NOISE_MEASURABLE, NOISE_BELOW_STEP, NOISE_PROBE_FAILED };
static int noise_measurable(void)
{
    uint8_t buf[PROBE_BYTES];
    unsigned hi[16] = {0}, hq[16] = {0}, n = 0;
    for (unsigned tries = 0; tries < 24u && n < 8u * sizeof(buf); ++tries) {
        vTaskDelay(1);
        if (!copy_probe_window(buf, NULL)) continue;
        for (unsigned k = 0; k < sizeof(buf); ++k) { ++hi[dsp_i(buf[k]) & 15]; ++hq[dsp_q(buf[k]) & 15]; }
        n += sizeof(buf);
    }
    if (n < 4u * sizeof(buf)) return NOISE_PROBE_FAILED;
    unsigned ci = 0, cq = 0;
    for (unsigned c = 0; c < 16u; ++c) { ci += hi[c] * 20u >= n; cq += hq[c] * 20u >= n; }
    return ci >= 3u && cq >= 3u ? NOISE_MEASURABLE : NOISE_BELOW_STEP;
}

/* With a VTX on: a strong FM carrier lies on a circle around the true DC,
 * and the circle centre does not depend on the picture (dsp_circle_dc).
 * The current gain row is searched on such a carrier, but only on the
 * bench: a clean, strong circle and the same gain held for 10 s, never in
 * flight (a search steps the DC for about two seconds). */
#define DCO_CARRIER_RATIO_X100 300u
static bool dco_measure_carrier(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect(64, &w) || !w.dc_circle || w.envelope_x100 < DCO_CARRIER_RATIO_X100) return false;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

static bool dco_carrier_search(bool manual)
{
    const uint8_t g = s_current_gain;
    if (!manual && g <= ARC_VENDOR_GAIN_MAX && s_dco.e[g].valid == DCO_ROW_OK) return false;
    if (s_gain.state != DG_HOLD || g < s_dco.lo || g > s_dco.hi || s_obs_clip_pm) return false;
    window_stats_t w;
    if (!collect(16, &w) || !w.dc_circle || w.envelope_x100 < DCO_CARRIER_RATIO_X100) return false;
    if (!receiver_pause("DCO_CARRIER")) return false;
    (void)phy_dco_release();
    phy_dco_result_t res;
    (void)phy_dco_search(dco_measure_carrier, &res);
    if (abs(res.residual_mcells[0]) > DCO_BAD_MCELLS || abs(res.residual_mcells[1]) > DCO_BAD_MCELLS)
        res.measured = false;
    if (res.measured) {
        s_dco.e[g] = (dco_entry_t){{(int16_t)res.codes[0], (int16_t)res.codes[1]},
            {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
        dco_row_measured(g);
        if (dco_save()) s_dco_dirty = false;
    }
    s_dco_found_us[g] = esp_timer_get_time();
    ++s_dco_searches;
    ++s_dco_carrier_searches;
    receiver_resume();
    return res.measured;
}

/* Noise below one quantizer step: the IQ is a fixed code, so a short
 * window is an exact DC measurement (no averaging needed). */
static bool dco_measure_flat(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!collect(8, &w)) return false;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

/* Row g with the receiver noise below one step: codes from the flat search.
 * Before, such a row and all below it were FLOOR (left at the chip's DC). */
static bool dco_flat_row(uint8_t g, const int *seed)
{
    phy_dco_result_t res;
    (void)phy_dco_search_flat_from(dco_measure_flat, seed, &res);
    if (!res.measured && seed) (void)phy_dco_search_flat(dco_measure_flat, &res);
    if (!res.measured) return false;
    s_dco.e[g] = (dco_entry_t){{(int16_t)res.codes[0], (int16_t)res.codes[1]},
        {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
    dco_row_measured(g);
    s_dco_found_us[g] = esp_timer_get_time();
    ++s_dco_searches;
    return true;
}

/* One quiet search of gain row g (receiver noise). NOISE_BELOW_STEP when
 * the noise is below one quantizer step there (no codes at or below g),
 * NOISE_PROBE_FAILED when nothing could be measured (rows untouched). */
static bool s_dco_batch;
static int dco_quiet_search(uint8_t g, const int *seed)
{
    if (!s_dco_batch && !receiver_pause("DCO")) return NOISE_PROBE_FAILED;
    const uint8_t restore = s_current_gain;
    (void)phy_dco_release();
    receiver_set_gain(g);
    vTaskDelay(pdMS_TO_TICKS(20));
    int measurable = noise_measurable();
    if (measurable == NOISE_PROBE_FAILED) {
        printf("DCO row=%u probe_failed (rows kept for the next boot)\n", g);
    } else if (measurable == NOISE_BELOW_STEP && dco_flat_row(g, seed)) {
        measurable = NOISE_MEASURABLE;          /* this row has codes now; go on downward */
    } else if (measurable == NOISE_BELOW_STEP) {
        /* Less gain, even less noise, and no flip found: the rows below
         * cannot be searched either. Codes measured on a carrier stay. */
        for (int k = g; k >= (int)s_dco.lo; --k) {
            /* Own codes measured on a carrier stay; template codes (another
             * channel or another filter) are not this context's and go. */
            if (s_dco.e[k].valid != DCO_ROW_OK || dco_mask_get(s_dco_templ, (unsigned)k))
                s_dco.e[k].valid = DCO_ROW_FLOOR;
            dco_row_measured((unsigned)k);   /* measured: nothing to correct here */
        }
        s_dco_floor = (uint8_t)(g + 1);
    } else {
        phy_dco_result_t res;
        (void)phy_dco_search_verified(dco_measure_fast, dco_measure, seed, &res);
        if (!res.measured) {
            printf("DCO row=%u fast_search=fallback_full\n", g);
            (void)phy_dco_search_from(dco_measure, seed, &res);
        }
        /* A neighbouring seed may be outside this row's response range. */
        if ((!res.measured || abs(res.residual_mcells[0]) > DCO_BAD_MCELLS ||
             abs(res.residual_mcells[1]) > DCO_BAD_MCELLS) && seed)
            (void)phy_dco_search_from(dco_measure, NULL, &res);
        if (res.measured) {
            const dco_entry_t e = {{(int16_t)res.codes[0], (int16_t)res.codes[1]},
                {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
            if (!dco_entry_bad(&e)) {
                s_dco.e[g] = e;
                dco_row_measured(g);
                if (g >= NOISE_CACHE_LO) {
                    const unsigned k = g - NOISE_CACHE_LO;
                    s_noise_cache[k].time = esp_timer_get_time();
                    s_noise_cache[k].freq = rf_frequency_mhz();
                    s_noise_cache[k].key = live_filter_key();
                    s_noise_cache[k].m2 = s_noise_last_m2;
                    s_noise_cache[k].kurt = s_noise_last_kurt;
                }
            } else {
                s_dco.e[g].valid = 0;                /* not converged: missing, repaired from a neighbour */
                printf("DCO row=%u not_converged residual=%d/%d\n", g, e.residual_mcells[0], e.residual_mcells[1]);
            }
        }
        s_dco_found_us[g] = esp_timer_get_time();
        ++s_dco_searches;
    }
    if (measurable != NOISE_PROBE_FAILED) s_dco_dirty = true;
    if (!s_dco_batch) { receiver_set_gain(restore); receiver_resume(); }
    return measurable;
}

static unsigned dco_rows(uint8_t state)
{
    unsigned n = 0;
    for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g) n += s_dco.e[g].valid == state;
    return n;
}
static unsigned dco_valid_rows(void) { return dco_rows(DCO_ROW_OK); }

bool calibration_dc_ok(uint8_t g)
{
    if (phy_filter_calibrated_code() < 0) return true;     /* unpinned PHY: nothing to hold */
    if (s_dco.freq != rf_frequency_mhz() || s_dco_serving != live_filter_key() ||
        g > ARC_VENDOR_GAIN_MAX || dco_mask_get(s_dco_templ, g)) return false;
    if (g < s_dco.lo || g > s_dco.hi) return true;
    if (s_dco.e[g].valid == DCO_ROW_OK) return !s_dco_banned[g] && phy_dco_held();
    return s_dco.e[g].valid == DCO_ROW_FLOOR || g < s_dco_floor;
}

/* Continuation: every missing row next to a converged one is searched from
 * that row's codes, upward from below and downward from above, until no row
 * changes. The gain steps between rows are ~1 dB, so the start is within the
 * search's linear range wherever the chip's own codes were not. */
static void dco_repair_rows(int64_t deadline)
{
    for (bool changed = true; changed && esp_timer_get_time() < deadline;) {
        changed = false;
        for (int pass = 0; pass < 2; ++pass) {
            for (int k = 0; k <= (int)(s_dco.hi - s_dco.lo); ++k) {
                const int g = pass == 0 ? (int)s_dco.lo + k : (int)s_dco.hi - k;
                const int n = pass == 0 ? g - 1 : g + 1;
                /* A template neighbour seeds too: the seed only has to start
                 * the search inside its linear range. */
                if (n < (int)s_dco.lo || n > (int)s_dco.hi || s_dco.e[g].valid || s_dco.e[n].valid != DCO_ROW_OK ||
                    g < (int)s_dco_floor) continue;
                if (esp_timer_get_time() > deadline || !noise_like()) return;
                const int seed[2] = {s_dco.e[n].code[0], s_dco.e[n].code[1]};
                ++s_dco_repairs;
                if (dco_quiet_search((uint8_t)g, seed) == NOISE_MEASURABLE && s_dco.e[g].valid == DCO_ROW_OK) {
                    printf("DCO row=%u repaired from row=%d codes=%d/%d\n", g, n, s_dco.e[g].code[0], s_dco.e[g].code[1]);
                    changed = true;
                } else {
                    ++s_dco_repair_fails;
                }
            }
        }
    }
}

/* The quiet-noise row search, from the top row down; shared by the own
 * channel (VTX off) and a quiet channel (VTX on). */
/* refine: also measure rows that still come from a template (own channel
 * quiet at boot), replacing them by this channel's own codes. */
static const char *dco_quiet_rows(bool manual, bool refine, int64_t deadline, const char *line1, unsigned *done_out)
{
    if (!receiver_pause("DCO_BATCH")) return "busy";
    const uint8_t restore_gain = s_current_gain;
    s_dco_batch = true;
    const int64_t t0 = esp_timer_get_time();
    const unsigned rows = manual ? (unsigned)(s_dco.hi - s_dco.lo + 1u) :
                          dco_rows(0) + (refine ? dco_template_rows() : 0u);
    unsigned done = 0;
    const char *why = "done";
    for (int g = s_dco.hi; g >= (int)s_dco.lo; --g) {
        if (!manual && s_dco.e[g].valid && !(refine && dco_mask_get(s_dco_templ, (unsigned)g)))
            continue;                                             /* own codes or floor */
        char line2[48];
        unsigned left = done ? (unsigned)((esp_timer_get_time() - t0) / done * (rows - done) / 1000000) + 1u : rows * 2u;
        snprintf(line2, sizeof(line2), "ABOUT %u S LEFT, THEN LIVE VIDEO", left);
        status_screen("CALIBRATING DC", line1, line2, rows ? (int)(done * 100u / rows) : 0);
        if (esp_timer_get_time() > deadline) { why = "timeout"; break; }
        if (!noise_like()) { why = "carrier_detected"; break; }   /* VTX switched on / leaking */
        /* Only own, converged, unbanned rows of this exact filter/channel
         * seed the next row; a template is never calibration evidence. */
        const bool own = s_dco.e[g].valid == DCO_ROW_OK && !dco_mask_get(s_dco_templ, (unsigned)g) &&
            !s_dco_banned[g];
        const bool neighbour = g < (int)s_dco.hi && s_dco.e[g + 1].valid == DCO_ROW_OK &&
            !dco_mask_get(s_dco_templ, (unsigned)g + 1u) && !s_dco_banned[g + 1];
        const int seed[2] = {own ? s_dco.e[g].code[0] : neighbour ? s_dco.e[g + 1].code[0] : 0,
                             own ? s_dco.e[g].code[1] : neighbour ? s_dco.e[g + 1].code[1] : 0};
        const int r = dco_quiet_search((uint8_t)g, own || neighbour ? seed : NULL);
        if (r == NOISE_PROBE_FAILED) { why = "probe_failed"; break; }
        if (r == NOISE_BELOW_STEP) { why = "floor"; break; }     /* the rest needs no correction */
        ++done;
    }
    if (!strcmp(why, "done") || !strcmp(why, "floor")) dco_repair_rows(deadline);
    *done_out = done;
    s_dco_batch = false;
    receiver_set_gain(restore_gain);
    receiver_resume();
    printf("CAL_STEP step=dc rows=%u ms=%lld result=%s\n", done, (esp_timer_get_time() - t0) / 1000, why);
    return why;
}

/* With the VTX on, the own channel never shows plain receiver noise, so the
 * high gain rows (the ones used far out) stayed without DC codes: board
 * DC without correction is ~1.1 cell (C5VRX-4 video_dco.c), and in the
 * stress matrix a 1-cell DC at 4 dB C/N broke the goggle line lock
 * (picture 0.25-0.97: lines shifting in bands). Like the filter sweep, the
 * rows are measured on a quiet channel >= 40 MHz away and the exact tuning
 * is restored. The DC DAC offsets are baseband offsets per gain row, so the
 * codes are stored for the own channel; the live drift nudges trim the
 * remaining difference. Startup only, behind the status screen. */
/* ---- DC health in flight, repair at the next boot (dc_health.h) ---------
 * Passive: reads the table state, the filter registers and the trim's DC
 * mean; never writes the receiver. A sustained finding stores one flag per
 * (frequency, filter) and the menu shows DC CAL NEEDED; dco_startup then
 * re-measures that table and clears the flag only when it is complete. */
static dc_flags_t s_dc_flags;
static bool s_dc_flags_loaded;

static uint32_t s_dc_flag_writes;

static void dc_flags_load(void)
{
    if (s_dc_flags_loaded) return;
    s_dc_flags_loaded = true;
    if (!blob_load("dc_recal", &s_dc_flags, sizeof(s_dc_flags)) || !dc_flags_valid(&s_dc_flags))
        dc_flags_init(&s_dc_flags);
}

static void dc_flags_save(void)
{
    if (blob_store("dc_recal", &s_dc_flags, sizeof(s_dc_flags))) ++s_dc_flag_writes;
    else printf("DC_HEALTH error: repair flags not stored\n");
}

/* Profile name of a filter key on this board (NORMAL/NARROW/WIDE). */
static const char *dc_key_profile(uint16_t key)
{
    static const int classes[3] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE};
    static const char *const names[3] = {"NORMAL", "NARROW", "WIDE"};
    for (unsigned c = 0; c < 3u; ++c) {
        rf_filter_t f;
        if (filter_setting(classes[c], &f) && dco_filter_key(f.code, (int)f.skirt, f.bw20) == key) return names[c];
    }
    return "OTHER";
}

/* No search or automatic repair is attached to this warning. Old NVS
 * repair flags may guide an explicit manual run, never normal boot. */
unsigned calibration_dc_needed(char *buf, size_t size)
{
    const bool bad = s_basemap_watch.state == CAL_BAD &&
        esp_timer_get_time() - s_basemap_watch.last_us <= CAL_EVIDENCE_GAP_US;
    if (size) snprintf(buf, size, "%s", bad ? "BASEMAP" : "");
    return bad;
}

static void dc_health_print(void)
{
    printf("CAL_HEALTH state=%s qualified=%u rejected=%u dc_mcells=%d/%d reason=%u "
           "auto_cal=0 auto_dc_trim=0 boot_phase_budget_ms=600 last_us=%lld\n",
           s_basemap_watch.state == CAL_BAD ? "bad" : s_basemap_watch.state == CAL_GOOD ? "good" : "unknown",
           s_health_blocks, s_health_rejected, s_basemap_watch.last_i, s_basemap_watch.last_q,
           s_basemap_watch.reason, s_basemap_watch.last_us);
}

/* Startup: only the rows of the channel that are still missing (all rows
 * on a manual request). Rows that have codes or a measured noise floor are
 * kept, so a boot after a complete calibration does nothing here. */
static void dco_startup(bool manual, int64_t deadline)
{
    dco_select();
    const unsigned missing = dco_rows(0);
    const bool carrier = video_recent();
    /* Codes measured with another analog filter do not fit the live one
     * (dco_slots.h dco_filter_same: 3.7 cells of DC left on the board, and
     * the gain parked on it). Every row with a result (codes or FLOOR) is
     * then marked as a template row, i.e. not measured in this context; a
     * row leaves that mask only on a new result (dco_row_measured), the mask
     * is stored with the table, so an interrupted pass goes on at the next
     * boot, and the table takes the live filter only when no template row
     * is left. Template rows are measured on the own channel (VTX off) or a
     * quiet channel (VTX on). */
    const rf_filter_t live = rf_filter();
    const unsigned results = dco_valid_rows() + dco_rows(DCO_ROW_FLOOR);
    const bool stale = dco_filter_stale(results,
        dco_filter_same(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20, live.code, (int)live.skirt,
                        live.bw20));
    /* A repair flag from the last flight (dc_health_latch): the whole table
     * is re-measured like a stale one, also with the VTX on. */
    dc_flags_load();
    const bool flagged = dc_flags_find(&s_dc_flags, rf_frequency_mhz(), live_filter_key()) >= 0;
    if (flagged)
        printf("DCO startup repair_flag=1 freq=%u filter_key=%04x profile=%s\n", rf_frequency_mhz(),
               live_filter_key(), dc_key_profile(live_filter_key()));
    if ((stale || flagged) && !dco_template_rows()) {
        for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
            if (s_dco.e[g].valid) dco_mask_put(s_dco_templ, g, true);
        s_dco_dirty = true;
    }
    const unsigned templ = dco_template_rows();
    if (stale)
        printf("DCO startup filter_stale=1 table_bw20=%u table_code=%d table_skirt=%d live_bw20=%u live_code=%d "
               "live_skirt=%u rows_to_measure=%u\n", (unsigned)s_dco.filter_bw20, (int)s_dco.filter_code,
               (int)s_dco.filter_skirt, (unsigned)live.bw20, live.code, (unsigned)live.skirt, templ);
    /* Template rows from another channel are replaced only when the own
     * channel can be measured without a detour (VTX off at boot); rows of
     * another filter also go to a quiet channel. */
    const bool refine = !manual && !missing && templ && (stale || flagged || (!carrier && noise_like()));
    if (!manual && !missing && !refine) {
        printf("DCO startup skip=complete rows=%u floor_rows=%u template_rows=%u\n", dco_valid_rows(),
               dco_rows(DCO_ROW_FLOOR), templ);
        return;
    }
    unsigned done = 0;
    const char *why;
    const char *where = refine ? "own_refine" : "own";
    if (!carrier && noise_like()) {
        why = dco_quiet_rows(manual, refine, deadline, refine ? "OWN CHANNEL, VTX OFF" : "KEEP THE VTX OFF", &done);
    } else {
        /* A carrier on the own channel: measure on a quiet channel; only if
         * there is none, the current row on the carrier's circle. */
        const bool ok = carrier && dco_carrier_search(manual);
        where = "carrier";
        why = ok ? "carrier_row_only" : "need_vtx_off";
        printf("DCO startup carrier=%u result=%s gain=%u missing_rows=%u (VTX off measures all rows)\n",
               carrier, ok ? "stored_current_row" : "skip", s_current_gain, missing);
    }
    const bool complete = !strcmp(why, "floor") || !strcmp(why, "done");
    /* The noise floor ends the search before the last row: that is the
     * normal end, not a stall, so the bar completes and says so. */
    if (complete)
        status_screen("DC CALIBRATED", "LOWER GAINS NEED NO DC CORRECTION", "", 100);
    else
        status_screen("DC NOT COMPLETE", why, "USE MENU > CALIBRATE", -1);
    vTaskDelay(pdMS_TO_TICKS(800));
    if (stale && complete && !dco_template_rows()) {
        s_dco.filter_code = live.code;          /* every row now has a result in the live filter */
        s_dco.filter_skirt = (int8_t)live.skirt;
        s_dco.filter_bw20 = live.bw20;
        s_dco_dirty = true;
    }
    if (s_dco_dirty && dco_save()) s_dco_dirty = false;
    /* The flag goes only with a complete table of this filter, saved. */
    if (complete && !s_dco_dirty && !dco_rows(0) && !dco_template_rows() &&
        dc_flags_clear(&s_dc_flags, rf_frequency_mhz(), live_filter_key())) {
        dc_flags_save();
        printf("DCO startup repair_flag=cleared freq=%u profile=%s\n", rf_frequency_mhz(),
               dc_key_profile(live_filter_key()));
    }
    printf("DCO startup where=%s result=%s searched=%u rows=%u floor_rows=%u missing=%u template_rows=%u "
           "floor_from_gain=%u\n", where, why, done, dco_valid_rows(), dco_rows(DCO_ROW_FLOOR), dco_rows(0),
           dco_template_rows(), s_dco_floor);
}

/* DC readiness per physical filter on the current channel, summarised after
 * every startup DC pass: a profile switch may only narrow onto a filter whose
 * table is complete there (no missing or template rows, measured with that
 * filter); otherwise it would fly on codes of another filter. */
#define DCO_READY_MAX 3u
static struct { uint16_t freq, key; bool ready; } s_dco_ready[DCO_READY_MAX];

static bool dco_current_complete(void)
{
    const rf_filter_t live = rf_filter();
    return !dco_rows(0) && !dco_template_rows() &&
           dco_filter_same(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20, live.code, (int)live.skirt,
                           live.bw20);
}

static void dco_ready_note(void)
{
    const uint16_t key = live_filter_key(), freq = rf_frequency_mhz();
    unsigned slot = 0;
    for (unsigned k = 0; k < DCO_READY_MAX; ++k) {
        if (s_dco_ready[k].freq == freq && s_dco_ready[k].key == key) { slot = k; break; }
        if (s_dco_ready[k].freq != freq) slot = k;
    }
    s_dco_ready[slot].freq = freq;
    s_dco_ready[slot].key = key;
    s_dco_ready[slot].ready = dco_current_complete();
}

bool calibration_dc_ready(rf_filter_t f)
{
    const uint16_t key = dco_filter_key(f.code, (int)f.skirt, f.bw20), freq = rf_frequency_mhz();
    for (unsigned k = 0; k < DCO_READY_MAX; ++k)
        if (s_dco_ready[k].freq == freq && s_dco_ready[k].key == key) return s_dco_ready[k].ready;
    return false;
}

/* Startup: the DC tables of the other profile filters too (NORMAL, NARROW,
 * WIDE, each distinct physical setting once), so a profile switch in flight
 * finds the codes of its own filter. Each filter is set up in its own
 * analog branch (rf_set_filter) and goes through dco_startup(); a pass cut
 * by the budget continues at the next boot (template/missing rows). */
static void dco_startup_profiles(bool manual, int64_t deadline)
{
    const rf_filter_t live = rf_filter();
    dco_ready_note();
    rf_filter_t todo[3];
    unsigned n = 0;
    static const int classes[3] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE};
    for (unsigned c = 0; c < 3u; ++c) {
        rf_filter_t f;
        if (!filter_setting(classes[c], &f) || rf_filter_equal(f, live)) continue;
        bool dup = false;
        for (unsigned k = 0; k < n; ++k) dup |= rf_filter_equal(todo[k], f);
        if (!dup) todo[n++] = f;
    }
    for (unsigned k = 0; k < n && esp_timer_get_time() < deadline; ++k) {
        printf("DCO startup profile_filter bw20=%u code=%d skirt=%u\n", (unsigned)todo[k].bw20, todo[k].code,
               (unsigned)todo[k].skirt);
        if (!rf_set_filter(todo[k])) continue;
        receiver_set_gain(rf_gain_table()->max_index);
        vTaskDelay(pdMS_TO_TICKS(50));
        dco_startup(manual, deadline);
        dco_ready_note();
    }
    if (n) {
        if (!rf_set_filter(live)) printf("DCO startup error: live filter not restored\n");
        dco_select();
    }
    for (unsigned k = 0; k < DCO_READY_MAX; ++k)
        if (s_dco_ready[k].freq)
            printf("DCO_READY freq=%u filter_key=%04x ready=%u\n", s_dco_ready[k].freq, s_dco_ready[k].key,
                   s_dco_ready[k].ready);
}

/* Live: hold the stored codes after gain writes and guard them; never a
 * search (a search steps the DC for seconds). */
static void dco_service(void)
{
    if (!quiet_owner()) return;
    dco_select();
    if (s_dco_dirty && s_gain.state == DG_HOLD && dco_save()) s_dco_dirty = false;
    const int64_t now = esp_timer_get_time();
    for (unsigned r = 0; r <= ARC_VENDOR_GAIN_MAX; ++r)
        if (s_dco_banned[r] && now >= s_dco_ban_until[r]) { s_dco_banned[r] = 0; ++s_dco_unbans; }
    /* Hold guard: held codes must never coincide with dead or railing IQ. */
    static unsigned bad_ticks;
    const uint8_t g = s_current_gain;
    if (phy_dco_held()) {
        bad_ticks = (s_obs_origin_pm >= 900 || s_obs_clip_pm >= 500) ? bad_ticks + 1u : 0u;
        if (bad_ticks >= 3u) {
            (void)phy_dco_release();
            s_dco_banned[g] = 1u;
            s_dco_ban_len[g] = s_dco_ban_len[g] && now - s_dco_ban_until[g] < 60000000LL ?
                               (s_dco_ban_len[g] * 2 > DCO_BAN_MAX_US ? DCO_BAN_MAX_US : s_dco_ban_len[g] * 2) : DCO_BAN_US;
            s_dco_ban_until[g] = now + s_dco_ban_len[g];
            bad_ticks = 0;
            ++s_dco_aborts;
            printf("DCO hold abort gain=%u\n", g);
        }
    } else if (s_gain.state != DG_SETTLE) {
        dco_post_gain(g);
    }
}

/* One passive collector for health and phase evidence. Independent blocks
 * use identical gain/filter/DC/TX context. No PHY ownership, writes, pause,
 * NVS learning, or raster takeover. Native factory DC remains the fallback
 * when there is no exact saved row; foreign tables are search seeds only. */
static int s_drift_avg[2];
static float s_temp_c = -100.0f;
static uint32_t s_servo_writes; /* wire-format compatibility, always zero */
static TaskHandle_t s_cal_task;
static volatile bool s_dc_urgent;
static uint32_t s_dc_urgent_wakes;

static uint32_t health_context(void)
{
    /* Epochs cover gain/DC/retunes. TX ownership matters because the menu
     * raster and live output can produce different self-noise. */
    uint32_t h = 2166136261u;
    const uint32_t part[] = {s_gain_epoch, s_control_epoch, phy_generation(), rf_table_generation(),
        rf_frequency_mhz(), (uint32_t)rf_offset_khz(), live_filter_key(), s_current_gain,
        menu_splash_active(), s_menu_active};
    for (unsigned k = 0; k < sizeof(part) / sizeof(part[0]); ++k) h = (h ^ part[k]) * 16777619u;
    return h;
}

static void drift_service(void)
{
    s_dc_urgent = false;
    if (s_startup || s_scan_active || receiver_paused() || phy_busy() ||
        (s_menu_active && !menu_splash_active()) || rf_lane() != RF_LANE_ULTRAFINE) return;
    const int64_t now = esp_timer_get_time();
    static int64_t next_us;
    if (now < next_us || now - s_last_gain_write_us < 20000) return;
    next_us = now + 250000;
    const uint32_t context = health_context();
    window_stats_t w;
    const bool copied = collect_stats(24, &w, true);
    if (context != health_context()) { ++s_health_rejected; return; }
    const int mismatch = phy_filter_hw_mismatch(NULL, NULL);
    if (context != health_context()) { ++s_health_rejected; return; }
    const double m2 = w.windows ? w.noise_m2 / w.windows : 0;
    const double kurt = m2 > 0 && w.windows ? (w.noise_m4 / w.windows) / (m2 * m2) : 0;
    const bool noise = w.envelope_x100 <= dsp_quiet_limit(w.m.clip_pm) && kurt >= 1.85 && kurt <= 2.25;
    const bool carrier = w.dc_circle && video_recent();
    const bool qualified = mismatch > 0 || (copied && w.windows == 24u && w.dc_qualified && (noise || carrier));
    if (qualified) { ++s_health_blocks; s_drift_avg[0] = w.dc_i; s_drift_avg[1] = w.dc_q; }
    else ++s_health_rejected;
    s_basemap_context = context;
    s_health_us += esp_timer_get_time() - now;
    const bool warn = cal_watch_add(&s_basemap_watch, context, now, qualified,
                                   mismatch > 0 ? DC_HEALTH_FILTER : 0u, w.dc_i, w.dc_q);
    if (warn) printf("CAL_NEEDED reason=%s dc_mcells=%d/%d action=MENU_CALIBRATE auto_cal=0\n",
                     mismatch > 0 ? "filter_readback" : "dc_offset", w.dc_i, w.dc_q);
    /* A high absolute HF statistic alone is intrinsic noise or phase mixing,
     * not enough evidence to label the base map bad. No automatic slips. */
    if (copied && w.windows == 24u && !menu_splash_active()) {
        (void)sphase_record(&w);
        s_sphase_state = s_sphase_excess < (carrier ? SPHASE_LIVE_LIMIT : SPHASE_LIMIT) ? "settled" : "unverified";
    }
}

/* Persist the measured gain map: at most every 30 s, while holding, only
 * when it changed. Short flights end with the battery pulled, so 2 minutes
 * lost most of a flight's learning. Flash wear: a ~540-byte blob in the
 * 24 KiB NVS partition, even written every 30 s, is > 30,000 flight hours
 * of page erase cycles; unchanged maps are never written. */
static void gain_map_service(void)
{
    static int64_t last;
    int64_t now = esp_timer_get_time();
    if (now - last < 30000000LL) return;
    dg_map_blob_t blob;
    if (!receiver_export_gain_map(&blob)) return;
    last = now;
    if (s_gain_map_saved_valid && !memcmp(&blob, &s_gain_map_saved, sizeof(blob))) return;
    if (blob_store("gain_map", &blob, sizeof(blob))) {
        /* Observer imports this cache; publish the complete verified blob. */
        vTaskSuspendAll();
        s_gain_map_saved = blob; s_gain_map_saved_valid = true;
        (void)xTaskResumeAll();
    }
}

/* VTX deviation for the receive profile (profile.c): measured fresh on
 * this boot's carrier while the DEFAULT picture keeps running (a stored
 * deviation is not a VTX identity); only the switch to a matched profile
 * happens behind a status screen. With no VTX yet (calibrated with the VTX
 * off) it gives one 2 s to appear; later the measurement goes
 * on passively and a result is kept as pending for the next boot. */
#define DEVIATION_WAIT_US    2000000LL
#define DEVIATION_MEASURE_US 15000000LL
static void deviation_startup(int64_t deadline)
{
    rf_filter_t matched;
    if (!filter_setting(FILTER_NARROW, &matched) && !filter_setting(FILTER_WIDE, &matched)) {
        profile_set_reason("no_matched_filter");
        printf("PROFILE startup skip=no_matched_filter\n");
        return;
    }
    const int64_t wait_until = esp_timer_get_time() + DEVIATION_WAIT_US;
    while (!video_recent() && esp_timer_get_time() < wait_until && esp_timer_get_time() < deadline)
        vTaskDelay(pdMS_TO_TICKS(200));
    if (!video_recent()) {
        profile_set_reason("no_video_at_startup");
        printf("PROFILE startup skip=no_video (measured passively later, pending for the next boot)\n");
        return;
    }
    profile_measure_begin();
    const int64_t until = esp_timer_get_time() + DEVIATION_MEASURE_US;
    while (profile_measure_result() < 0 && esp_timer_get_time() < until && esp_timer_get_time() < deadline)
        vTaskDelay(pdMS_TO_TICKS(100));
    profile_measure_end();
    int dev = profile_measure_result();
    if (dev < 0) {
        profile_set_reason("inconclusive_at_startup");
        printf("PROFILE startup result=inconclusive profile=%s\n", profile_name(profile_active()));
        return;
    }
    profile_commit(dev);
}

/* C/N-AUTO noise map (cn_auto.h): with no carrier, the noise power of the
 * rows 3 apart from the table maximum down to G54, ~25 ms each, every boot
 * (also on a channel quiet now but measured before: temperature and drift).
 * Startup only, never in flight; skipped with a carrier on the channel. */
static void noise_map_startup(int64_t deadline)
{
    if (!noise_like()) { printf("CN_NOISE startup skip=carrier_or_not_quiet (map from NVS / learned later)\n"); return; }
    if (!receiver_pause("CNMAP")) return;
    const uint8_t restore = s_current_gain;
    unsigned rows = 0, rejected = 0;
    float measured[DG_STATES] = {0};   /* the gain table (receiver_gain_table_measured) */
    /* Every row while the noise is real (it is the gain measurement of the
     * edge rows, gain.c dg_apply_measured_gain), every 3rd below. */
    bool fine = true;
    for (int g = (int)rf_gain_table()->max_index; g >= 54 && esp_timer_get_time() < deadline; g -= fine ? 1 : 3) {
        const unsigned ck = (unsigned)g - NOISE_CACHE_LO;
        if (ck < NOISE_CACHE_ROWS && s_noise_cache[ck].time &&
            esp_timer_get_time() - s_noise_cache[ck].time <= 30000000LL &&
            s_noise_cache[ck].freq == rf_frequency_mhz() && s_noise_cache[ck].key == live_filter_key() &&
            (s_noise_cache[ck].m2 < CNA_QLIM_CELLS2 ||
             (s_noise_cache[ck].kurt >= 1.85f && s_noise_cache[ck].kurt <= 2.25f))) {
            const float m2 = s_noise_cache[ck].m2;
            receiver_cn_noise_store((uint8_t)g, m2);
            if (g < DG_STATES) measured[g] = m2;
            ++rows;
            if (m2 < CNA_QLIM_CELLS2) fine = false;
            printf("CN_NOISE row=%d noise_cells2=%.3f source=dc_verify\n", g, (double)m2);
            continue;
        }
        receiver_set_gain((uint8_t)g);
        vTaskDelay(pdMS_TO_TICKS(20));
        cna_sums_t s = {0};
        uint8_t buf[PROBE_BYTES];
        double m2s = 0.0, m4s = 0.0;
        unsigned got = 0;
        for (unsigned tries = 0; tries < 48u && got < 24u; ++tries) {
            vTaskDelay(1);
            if (!copy_probe_window(buf, NULL)) continue;
            cna_sums_t p = {0};
            for (unsigned k = 0; k < sizeof(buf); ++k) cna_sums_add(&p, buf[k]);
            float mi, mq;
            cna_mean(&p, &mi, &mq);
            double m2, m4;
            cna_moments(&p, mi, mq, &m2, &m4);
            m2s += m2; m4s += m4;
            cna_sums_merge(&s, &p);
            ++got;
        }
        if (got < 8u) continue;
        /* Noise only (Gaussian: M4 ~ 2 M2^2); a row below one 4-bit step
         * reads ~0.25-0.5 cells^2 and is kept (cn_auto marks it
         * quantisation-limited). */
        const double m2 = m2s / got, kurt = (m4s / got) / (m2 * m2);
        /* Receiver noise reads 1.88-2.13 on a clean channel (G83 folds a
         * little: ~1.9); a carrier lowers it, bursts raise it: B2 (5752 MHz,
         * next to Wi-Fi 149/151) read 2.3-14 on 2026-10-20, and row 72 came
         * out above row 73. Neither is the receiver's noise. */
        if (m2 >= CNA_QLIM_CELLS2 && (kurt < 1.85 || kurt > 2.25)) {
            ++rejected;
            printf("CN_NOISE row=%d rejected kurtosis=%.2f (carrier or interference)\n", g, kurt);
            continue;
        }
        float mi, mq;
        cna_mean(&s, &mi, &mq);
        double M2, M4;
        cna_moments(&s, mi, mq, &M2, &M4);
        receiver_cn_noise_store((uint8_t)g, (float)M2);
        if (g < DG_STATES) measured[g] = (float)M2;
        ++rows;
        if (M2 < CNA_QLIM_CELLS2) fine = false;
        printf("CN_NOISE row=%d noise_cells2=%.3f kurtosis=%.2f\n", g, M2, kurt);
    }
    receiver_set_gain(restore);
    receiver_resume();
    /* The gain table only from a clean pass (the stored one stays otherwise):
     * any rejected row above the 4-bit step means interference or a carrier. */
    if (rejected == 0u) receiver_gain_table_measured(measured);
    else printf("CN_NOISE gain_table=kept (rejected rows on this channel)\n");
    receiver_cn_noise_save();
    printf("CN_NOISE startup rows=%u rejected=%u (per-row noise for C/N-AUTO)\n", rows, rejected);
}

/* Manual F: characterize the one active preset, without a grid search or
 * retune. Measure after DC is repaired; do not borrow a foreign table.
 * These actual widths, not the nominal preset name, go into board exports. */
static void filter_measure_current(void)
{
    if (video_recent() || !receiver_pause("FILTER_CHECK")) return;
    const uint8_t restore = s_current_gain;
    receiver_set_gain(rf_gain_table()->max_index);
    dco_hold_for_scan();
    vTaskDelay(pdMS_TO_TICKS(50));
    const int64_t t0 = esp_timer_get_time();
    cal_noise_t a = {0};
    unsigned width = 0, nbw = 0, env = 0;
    int clip = 0;
    const bool quiet = noise_like();
    const bool ok = quiet && rf_filter_applied() && noise_collect(&a, CAL_WINDOWS) &&
        cal_noise_finish(&a, &width, &nbw, &env, &clip) && cal_setting_quiet(env, clip) && width && nbw;
    if (ok) {
        const rf_filter_t f = rf_filter();
        filter_blob_t cal = s_filter_cal;
        cal.version = FILTER_VERSION;
        cal.normal = (filter_entry_t){1u, f.bw20, f.skirt, f.code, (uint16_t)width, (uint16_t)nbw};
        if (blob_store("filter", &cal, sizeof(cal))) { s_filter_cal = cal; s_filter_result = "measured_current"; }
    }
    printf("CAL_STEP step=filter_current ms=%lld result=%s width_khz=%u nbw_khz=%u\n",
           (esp_timer_get_time() - t0) / 1000, ok ? "measured" : "unknown", width, nbw);
    receiver_set_gain(restore);
    receiver_resume();
}

static void calib_record(bool manual, int64_t t0);   /* flight log, below */
#define STARTUP_BUDGET_US 120000000LL
/* A full calibration (first boot, a new calibration generation, CALIBRATE)
 * measures the filter and three DC tables: 2 minutes covered the filter and
 * 19 of 30 rows of one table (board 2026-10-19), and the rest took two more
 * boots. It gets the time to finish once; a normal boot is unchanged. */
#define STARTUP_FULL_BUDGET_US 480000000LL
static void startup_phase(bool full)
{
    const bool manual = true;
    s_startup = true;
    memset(s_noise_cache, 0, sizeof(s_noise_cache));
    {
        const uint8_t m = manual;
        flightlog_event(FLOG_E_CAL_START, &m, 1);
    }
    const int64_t t0 = esp_timer_get_time();
    const int64_t deadline = t0 + (full ? STARTUP_FULL_BUDGET_US : STARTUP_BUDGET_US);
    /* Direct Gain first finds its state (table maximum without a carrier). */
    for (unsigned k = 0; k < 40u && s_gain.state == DG_SETTLE; ++k) vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(manual ? 200 : 1500));
    for (unsigned k = 0; k < 100u && !startup_owner(); ++k) vTaskDelay(pdMS_TO_TICKS(100));
    if (!startup_owner()) {
        printf("STARTUP skip=owner_busy menu=%u scan=%u paused=%u\n", s_menu_active, s_scan_active, receiver_paused());
    } else {
        printf("STARTUP begin manual=%u filter=%s dco_rows=%u deviation_x100=%d\n", manual,
               s_filter_cal.normal.valid ? "stored" : "missing", dco_valid_rows(),
               profile_deviation_x100(rf_channel_index()));
        /* A manual run re-measures the filter and the VTX: leave a matched
         * profile first, so filter and tracker never disagree. */
        if (profile_active() != PROFILE_DEFAULT) {
            status_screen("CALIBRATING", "BACK TO THE DEFAULT PROFILE", "", -1);
            if (!profile_set(PROFILE_DEFAULT)) printf("STARTUP error: DEFAULT profile not applied\n");
        }
        printf("STARTUP step=sphase\n");
        sphase_qualify("startup", true);
        printf("STARTUP step=filter\n");
        if (phy_filter_calibrated_code() < 0) { s_filter_result = "unsupported_phy"; }
        else if (full) {
            (void)filter_calibrate(!manual);
        }
        printf("STARTUP step=dco\n");
        if (esp_timer_get_time() < deadline) dco_startup(manual, deadline);
        /* Reuse fresh NORMAL DC windows before the other filter passes. */
        printf("STARTUP step=noise_map\n");
        if (esp_timer_get_time() < deadline) noise_map_startup(deadline);
        if (!full && esp_timer_get_time() < deadline) filter_measure_current();
        if (full && esp_timer_get_time() < deadline) dco_startup_profiles(manual, deadline);
        else dco_ready_note();
        printf("STARTUP step=deviation\n");
        /* Video needs a moment back on the own channel before measuring. */
        status_screen_end();
        vTaskDelay(pdMS_TO_TICKS(1500));
        /* Final qualification in the live TX/RX context, before the VTX
         * deviation measurement that needs the picture. */
        printf("STARTUP step=sphase_final\n");
        if (sphase_on_carrier()) sphase_carrier_scan("final", false);   /* choose on the carrier */
        else sphase_qualify("final", true);
        if (full) {
            if (esp_timer_get_time() < deadline) deviation_startup(deadline);
            else profile_set_reason("manual_budget");
        }
    }
    /* A fixed gain is saved across boots and costs range without any sign
     * in the picture (a bench setting left behind): say so at every boot. */
    if (!manual && receiver_gain_mode() != GAIN_MODE_AUTO) {
        char mode[16], line[48];
        const char *name = receiver_gain_mode_name(receiver_gain_mode(), mode, sizeof(mode));
        snprintf(line, sizeof(line), "GAIN %s COSTS RANGE: USE AUTO", name);
        printf("GAIN_WARNING mode=%s: a fixed gain is saved and costs range; menu GAIN or console G to AUTO\n",
               name);
        status_screen("GAIN NOT AUTO", line, "MENU > GAIN, HOLD TO CHANGE", -1);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    status_screen_end();
    s_startup = false;
    calib_record(manual, t0);
    flightlog_event(FLOG_E_CAL_END, NULL, 0);
    profile_live_begin();   /* passive from here on: never switches */
    s_manual_ms = (esp_timer_get_time() - t0) / 1000;
    s_last_manual_full = full;
    printf("CAL_MANUAL kind=%s ms=%lld searches=%lu filter=%s\n", full ? "full" : "current_filter",
           (esp_timer_get_time() - t0) / 1000, (unsigned long)s_dco_searches, s_filter_result);
    printf("STARTUP done ms=%lld filter=%s dco_rows=%u dco_missing=%u sphase=%s profile=%s reason=%s\n",
           (esp_timer_get_time() - t0) / 1000, s_filter_result, dco_valid_rows(), dco_rows(0), s_sphase_state,
           profile_name(profile_active()), profile_reason());
}

/* ---- Bench-only IQ lane comparison (console L) ---------------------------
 * Production keeps the ultrafine lane {9,6,5,4}. This compares it with fine
 * {9,7,6,5} and coarse {9,8,7,6} on the same RF input, gain, filter and
 * tuning, in ABBA order (U F C C F U) so a slow fade does not favour one
 * lane. Raw-IQ statistics only: P64's LUT is made for ultrafine, so
 * this is no demodulation, sensitivity or range result. Only on an
 * explicit request; the status screen owns TX for the ~3 s it takes and
 * the ultrafine lane is always routed back. */
#define LANE_WINDOWS 24u
static volatile bool s_lane_requested;
void lane_compare_request(void) { s_lane_requested = true; }

/* Bench (console N, range audit PR #230): does the board's own video output
 * (6-bit DAC and PARLIO TX at 40 MHz, next to the antenna input) raise the
 * receiver's noise floor? Receiver noise at the table maximum with the
 * output running and stopped, alternated four times (drift averages out);
 * the VTX must be off. Video stops ~0.3 s per pair. */
static volatile bool s_selfnoise_requested;
void self_noise_request(void) { s_selfnoise_requested = true; }
static bool selfnoise_measure(double *m2_out, double *kurt_out)
{
    uint8_t buf[PROBE_BYTES];
    double m2s = 0.0, m4s = 0.0;
    unsigned got = 0;
    for (unsigned tries = 0; tries < 96u && got < 48u; ++tries) {
        vTaskDelay(1);
        if (!copy_probe_window(buf, NULL)) continue;
        cna_sums_t p = {0};
        for (unsigned k = 0; k < sizeof(buf); ++k) cna_sums_add(&p, buf[k]);
        float mi, mq;
        cna_mean(&p, &mi, &mq);
        double m2, m4;
        cna_moments(&p, mi, mq, &m2, &m4);
        m2s += m2; m4s += m4;
        ++got;
    }
    if (got < 16u) return false;
    *m2_out = m2s / got;
    *kurt_out = (m4s / got) / (*m2_out * *m2_out);
    return true;
}
static void self_noise_test(void)
{
    if (!quiet_owner() || !receiver_pause("SELFNOISE")) { printf("SELFNOISE refused=busy\n"); return; }
    const uint8_t restore = s_current_gain;
    receiver_set_gain(rf_gain_table()->max_index);
    vTaskDelay(pdMS_TO_TICKS(40));
    double on_db = 0.0, off_db = 0.0;
    unsigned pairs = 0;
    for (unsigned rep = 0; rep < 4u; ++rep) {
        double m_on, k_on, m_off, k_off;
        const bool a = selfnoise_measure(&m_on, &k_on);
        transport_stop_tx();
        vTaskDelay(pdMS_TO_TICKS(30));
        const bool b = selfnoise_measure(&m_off, &k_off);
        transport_resume_live();
        vTaskDelay(pdMS_TO_TICKS(60));
        if (!a || !b) { printf("SELFNOISE pair=%u probe_failed\n", rep); continue; }
        printf("SELFNOISE pair=%u output_on=%.3f kurt=%.2f output_off=%.3f kurt=%.2f diff_db=%+.2f\n", rep, m_on, k_on,
               m_off, k_off, 10.0 * log10(m_on / m_off));
        on_db += 10.0 * log10(m_on); off_db += 10.0 * log10(m_off);
        ++pairs;
    }
    receiver_set_gain(restore);
    receiver_resume();
    if (pairs)
        printf("SELFNOISE gain=%u pairs=%u mean_diff_db=%+.2f (receiver noise with the video output on minus off; "
               "> 0.3 dB = the board raises its own floor)\n", rf_gain_table()->max_index, pairs,
               (on_db - off_db) / pairs);
}

typedef struct { unsigned windows; long p_median, q_phase, origin_pm, clip_pm, env_x100; } lane_stats_t;

static void lane_measure(lane_stats_t *st)
{
    uint8_t sample[PROBE_BYTES];
    vTaskDelay(pdMS_TO_TICKS(20));      /* the ring refills within ~1 ms */
    for (unsigned tries = 0, got = 0; tries < LANE_WINDOWS * 3u && got < LANE_WINDOWS; ++tries) {
        vTaskDelay(1);
        if (!copy_probe_window(sample, NULL)) continue;
        metrics_t m = measure_window(sample, sizeof(sample));
        st->p_median += m.p_median;
        st->q_phase += m.q_phase;
        st->origin_pm += m.origin_pm;
        st->clip_pm += m.clip_pm;
        st->env_x100 += dsp_envelope_ratio_x100(sample, sizeof(sample));
        ++st->windows;
        ++got;
    }
}

static void lane_compare(void)
{
    if (!quiet_owner() || !receiver_pause("LANE_CMP")) { printf("LANE_CMP refused=busy\n"); return; }
    s_scan_active = true;
    status_screen("BENCH LANE TEST", "VIDEO STOPS FOR ABOUT 3 S", "SAME GAIN, FILTER AND CHANNEL", -1);
    static const uint8_t order[] = {RF_LANE_ULTRAFINE, RF_LANE_FINE, RF_LANE_COARSE,
                                    RF_LANE_COARSE, RF_LANE_FINE, RF_LANE_ULTRAFINE};
    lane_stats_t st[RF_LANES];
    memset(st, 0, sizeof(st));
    bool routed = true;
    for (unsigned k = 0; k < sizeof(order) && routed; ++k) {
        routed = rf_set_lane(order[k]) == ESP_OK;
        if (routed) lane_measure(&st[order[k]]);
    }
    esp_err_t back = rf_set_lane(RF_LANE_ULTRAFINE);
    for (unsigned t = 0; back != ESP_OK && t < 3u; ++t) { vTaskDelay(pdMS_TO_TICKS(10)); back = rf_set_lane(RF_LANE_ULTRAFINE); }
    printf("LANE_CMP freq=%u gain=%u filter_code=%d skirt=%u routed=%u restored=%s (raw IQ only; P64 is made for "
           "ultrafine, not a range result)\n", rf_frequency_mhz(), s_current_gain, rf_filter().code, rf_filter().skirt,
           routed, back == ESP_OK && rf_lane() == RF_LANE_ULTRAFINE ? "ultrafine" : "FAILED");
    for (unsigned l = 0; l < RF_LANES; ++l) {
        const long n = st[l].windows ? (long)st[l].windows : 1;
        printf("LANE_CMP lane=%s windows=%u p_median=%ld q_phase=%ld origin_pm=%ld rail_pm=%ld env_x100=%ld\n",
               rf_lane_name(l), st[l].windows, st[l].p_median / n, st[l].q_phase / n, st[l].origin_pm / n,
               st[l].clip_pm / n, st[l].env_x100 / n);
    }
    printf("LANE_CMP note: origin_pm high = signal below the lane's step; rail_pm high = folding risk; compare "
           "q_phase at equal RF input over repeated runs before changing the production lane\n");
    status_screen_end();
    s_scan_active = false;
    receiver_resume();
}

/* One-sided clipping seen by the gain observer: run the DC servo now. */
/* Woken early, the servo still decides on the full DC_SERVO_MIN_SAMPLES of
 * the row (they accumulate continuously): faster, never noisier. Deciding
 * on 10000 samples (~8 ms) chased the flicker noise (board 2026-10-19: 804
 * wakes, 335 steps, 25 sign rejects in a minute without a VTX). */
void calibration_dc_urgent(void)
{
    static int64_t last;
    const int64_t now = esp_timer_get_time();
    if (s_dc_urgent || now - last < 100000) return;
    last = now;
    s_dc_urgent = true;
    if (s_cal_task) xTaskNotifyGive(s_cal_task);
}

/* Framed read-only JSON-lines report. Sequence + FNV cover every payload
 * byte, including newlines, so the browser refuses a truncated USB dump.
 * Only this task owns the DC/NVS scratch state; no calibration is requested.
 * Live readings are labelled observational and bracketed by context epochs. */
typedef struct { unsigned lines; uint32_t hash; bool failed; } cal_dump_t;
static void cal_dump_line(cal_dump_t *d, const char *fmt, ...)
{
    if (d->failed) return;
    char line[512];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0 || (unsigned)n >= sizeof(line)) { d->failed = true; return; }
    for (int k = 0; k < n; ++k) d->hash = (d->hash ^ (uint8_t)line[k]) * 16777619u;
    d->hash = (d->hash ^ '\n') * 16777619u;
    printf("CALDUMP_LINE n=%u %s\n", d->lines++, line);
    vTaskDelay(2); /* bounded chunks, never suspend the live receiver */
}
static void cal_dump_table(cal_dump_t *d, const dco_table_t *t, const dco_mask_t templ, int slot)
{
    const unsigned key = dco_filter_key(t->filter_code, t->filter_skirt, t->filter_bw20);
    for (unsigned g = t->lo; g <= t->hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        cal_dump_line(d, "{\"type\":\"dc_row\",\"slot\":%d,\"freq_mhz\":%u,\"filter_key\":%u,"
            "\"code\":%d,\"skirt\":%d,\"bw20\":%u,\"iq_scale\":%u,\"table_version\":%u,"
            "\"gain\":%u,\"i\":%d,\"q\":%d,\"residual_i_mcells\":%d,\"residual_q_mcells\":%d,"
            "\"valid\":%u,\"template\":%s}", slot, t->freq, key, t->filter_code, t->filter_skirt,
            t->filter_bw20, t->iq_scale_sel, t->version, g, t->e[g].code[0], t->e[g].code[1],
            t->e[g].residual_mcells[0], t->e[g].residual_mcells[1], t->e[g].valid,
            dco_mask_get(templ, g) ? "true" : "false");
}
static void calibration_dump(void)
{
    cal_dump_t d = {.hash = 2166136261u};
    const uint32_t context = health_context();
    const int64_t t0 = esp_timer_get_time();
    uint8_t mac[6] = {0};
    const bool have_id = esp_efuse_mac_get_default(mac) == ESP_OK;
    const uint32_t board = have_id ? flog_crc32(0x5a5a5a5au, mac, sizeof(mac)) : 0;
    esp_chip_info_t chip; esp_chip_info(&chip);
    printf("CALDUMP_BEGIN schema=1\n");
    cal_dump_line(&d, "{\"type\":\"device\",\"schema\":1,\"board_id\":\"%08lx\",\"board_id_valid\":%s,"
        "\"firmware\":\"%s\",\"idf\":\"%s\",\"chip_revision\":%u,\"cal_generation\":%u,\"uptime_us\":%lld}",
        (unsigned long)board, have_id ? "true" : "false", firmware_id(), esp_app_get_description()->idf_ver,
        chip.revision, calibration_generation_id(), t0);
    const rf_filter_t f = rf_filter();
    cal_dump_line(&d, "{\"type\":\"active\",\"freq_mhz\":%u,\"offset_khz\":%d,\"gain\":%u,\"lane\":%u,"
        "\"filter_code\":%d,\"filter_skirt\":%u,\"filter_bw20\":%s,\"filter_measured\":%s,"
        "\"filter_nbw_khz\":%u,\"dc_held\":%s,\"health\":%u,\"dc_i_mcells\":%d,\"dc_q_mcells\":%d,"
        "\"qualified_blocks\":%u,\"rejected_blocks\":%u}", rf_frequency_mhz(), rf_offset_khz(), s_current_gain,
        rf_lane(), f.code, f.skirt, f.bw20 ? "true" : "false", s_filter_cal.normal.valid ? "true" : "false",
        calibration_filter_nbw_khz(), phy_dco_held() ? "true" : "false", s_basemap_watch.state,
        s_basemap_watch.last_i, s_basemap_watch.last_q, s_health_blocks, s_health_rejected);
    int64_t radio_ms, ready_ms;
    firmware_boot_timing(&radio_ms, &ready_ms);
    cal_dump_line(&d, "{\"type\":\"boot_timing\",\"radio_ms\":%lld,\"ready_ms\":%lld,\"target_ms\":10000}",
                  radio_ms, ready_ms);
    int bank1[2] = {0}, bank2[2] = {0};
    const bool banks_valid = phy_dco_banks(bank1, bank2);
    cal_dump_line(&d, "{\"type\":\"dc_hardware\",\"valid\":%s,\"gain\":%u,\"bank1_i\":%d,\"bank1_q\":%d,"
        "\"bank2_i\":%d,\"bank2_q\":%d}", banks_valid ? "true" : "false", s_current_gain, bank1[0], bank1[1], bank2[0], bank2[1]);
    cal_dump_line(&d, "{\"type\":\"policy\",\"auto_calibration\":false,\"auto_dc_trim\":false,"
        "\"baseline_code\":16,\"baseline_skirt\":32,\"baseline_bw20\":false,\"boot_blocking_ms\":%lld,\"boot_phase_ms\":%lld,\"last_manual_ms\":%lld,\"last_manual_full\":%s,\"health_observation_us\":%lld}",
        s_boot_policy_ms, s_boot_phase_ms, s_manual_ms, s_last_manual_full ? "true" : "false", s_health_us);
    const filter_entry_t *filters[] = {&s_filter_cal.normal, &s_filter_cal.narrow, &s_filter_cal.wide};
    for (unsigned k = 0; k < 3u; ++k) {
        const filter_entry_t *x = filters[k];
        cal_dump_line(&d, "{\"type\":\"filter\",\"class\":%u,\"valid\":%s,\"code\":%d,\"skirt\":%u,"
            "\"bw20\":%s,\"width_khz\":%u,\"nbw_khz\":%u}", k, x->valid ? "true" : "false", x->code,
            x->skirt, x->bw20 ? "true" : "false", x->width_khz, x->nbw_khz);
    }
    /* -1 is the active table, slots 0..5 are saved observations. They may
     * duplicate it: consumers group by board/context/gain, not row count. */
    cal_dump_table(&d, &s_dco, s_dco_templ, -1);
    dco_table_t table;
    for (unsigned k = 0; k < DCO_SLOTS; ++k)
        if (s_dco_index_loaded && s_dco_index.freq[k] && dco_load_slot(k, &table))
            cal_dump_table(&d, &table, s_dco_index.templ[k], (int)k);
    dg_map_blob_t gain = {0};
    const bool gain_valid = receiver_export_gain_map(&gain) != 0;
    cal_dump_line(&d, "{\"type\":\"gain_context\",\"valid\":%s,\"version\":%u,\"freq_mhz\":%u,"
        "\"offset_khz\":%ld,\"filter_code\":%ld,\"filter_skirt\":%lu,\"filter_bw20\":%lu,\"iq_scale\":%lu}",
        gain_valid ? "true" : "false", gain.version, gain.freq_mhz, (long)gain.context.offset_khz,
        (long)gain.context.filter_code, (unsigned long)gain.context.filter_skirt,
        (unsigned long)gain.context.filter_bw20, (unsigned long)gain.context.iq_scale);
    if (gain_valid) for (unsigned g = 0; g <= gain.max_index && g < DG_STATES; ++g)
        cal_dump_line(&d, "{\"type\":\"gain_row\",\"gain\":%u,\"power_q10\":%u,\"uncertainty_pm\":%u,"
            "\"confidence\":%u}", g, gain.power_q10[g], gain.uncertainty_pm[g], gain.confidence[g]);
    for (unsigned g = 0; g <= ARC_VENDOR_GAIN_MAX; ++g) {
        float noise = 0; uint16_t samples = 0; int64_t measured_us = 0;
        if (receiver_export_noise((uint8_t)g, &noise, &samples, &measured_us))
            cal_dump_line(&d, "{\"type\":\"noise_row\",\"gain\":%u,\"cells2\":%.6f,\"samples\":%u,"
                "\"measured_us\":%lld}", g, (double)noise, samples, measured_us);
    }
    cal_dump_line(&d, "{\"type\":\"capture\",\"live_context_unchanged\":%s,\"duration_ms\":%lld}",
        context == health_context() ? "true" : "false", (esp_timer_get_time() - t0) / 1000);
    if (d.failed) printf("CALDUMP_ERROR reason=record_too_large\n");
    else printf("CALDUMP_END lines=%u fnv1a=%08lx\n", d.lines, (unsigned long)d.hash);
}

/* Reset-dependent sampling alignment is the sole bounded boot actuator.
 * It runs under the existing splash, never later on live video. A short
 * absolute check usually needs no slip; ambiguous evidence stays unknown.
 * No nine-position retry, filter sweep, gain table search or DC trimming. */
static void boot_phase_check(void)
{
    s_boot_phase_pending = false;
    const int64_t t0 = esp_timer_get_time();
    s_probe_deadline = t0 + 600000;
    s_startup = true;
    window_stats_t w;
    const uint32_t context = health_context();
    if (collect(24, &w) && w.windows == 24u && sphase_record(&w) < SPHASE_LIMIT &&
        collect(24, &w) && w.windows == 24u && sphase_record(&w) < SPHASE_LIMIT &&
        context == health_context())
        s_sphase_state = "settled";
    else if (menu_splash_active() && esp_timer_get_time() < s_probe_deadline) {
        const sphase_verdict_t v = sphase_scan_n("BOOT_PHASE", 3u, 24u, 3u, false);
        s_sphase_state = v == SPHASE_SETTLED ? "settled" : "unverified";
    }
    s_probe_deadline = 0;
    s_startup = false;
    s_boot_phase_ms = (esp_timer_get_time() - t0) / 1000;
    printf("BOOT_PHASE ms=%lld budget_ms=600 state=%s\n", s_boot_phase_ms, s_sphase_state);
}

static void calibration_boot(void)
{
    const int64_t boot_t0 = esp_timer_get_time();
    dco_select();
    dco_ready_note();
    s_startup = false;
    s_boot_phase_pending = true;
    s_boot_phase_until = boot_t0 + 5000000;
    profile_live_begin();
    s_boot_policy_ms = (esp_timer_get_time() - boot_t0) / 1000;
    printf("CAL_BOOT mode=baseline ms=%lld searches=0 sweeps=0 dc_trim=0 boot_phase_budget_ms=600\n",
           (esp_timer_get_time() - boot_t0) / 1000);
}

static void calibration_task(void *arg)
{
    (void)arg;
    s_cal_task = xTaskGetCurrentTaskHandle();
    rf_set_post_gain_hook(dco_post_gain);
    printf("STARTUP task up\n");
    calibration_boot();
    int64_t periodic_us = 0;
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250))) {
            /* Woken for a passive DC observation, and the rest still at its 250 ms
             * pace. The urgent wakes used to skip it altogether: after a
             * channel change the old DC table pushed the noise onto a rail,
             * one-sided clipping woke the servo every 100 ms, dco_service()
             * never ran to load the new channel's table, and the servo skipped
             * the foreign table: no DC correction until the next boot (board,
             * PR #228: A1 -> A2/A3, every probe clipped for minutes). */
            ++s_dc_urgent_wakes;
            drift_service();
            if (esp_timer_get_time() - periodic_us < 250000) continue;
        }
        periodic_us = esp_timer_get_time();
        if (s_boot_phase_pending && periodic_us >= s_boot_phase_until) s_boot_phase_pending = false;
        if (s_boot_phase_pending && menu_splash_active()) boot_phase_check();
        if (s_manual_requested && !s_menu_active) {
            const bool full = s_full_requested;
            s_manual_requested = s_full_requested = false;
            startup_phase(full);
            memset(&s_basemap_watch, 0, sizeof(s_basemap_watch));
            continue;
        }
        if (s_dump_requested) { s_dump_requested = false; calibration_dump(); continue; }
        if (s_lane_requested && !s_menu_active) { s_lane_requested = false; lane_compare(); continue; }
        if (s_selfnoise_requested && !s_menu_active) { s_selfnoise_requested = false; self_noise_test(); continue; }
        dco_service();
        drift_service();
        gain_map_service();
    }
}

/* Calibration generation ("cal_gen" in NVS). A calibration made by an
 * earlier firmware is not trusted: what a table means changed more than once
 * (DC codes measured in the wrong analog filter branch, one table for every
 * filter; docs/FINDINGS.md 2026-10-19), and an upgrade path that keeps old
 * codes kept exactly those errors. A boot with another generation discards
 * the filter calibration, every DC table and the repair flags, and runs the
 * full first-boot calibration. Raise CAL_GENERATION whenever a firmware
 * changes what a stored calibration means. */
/* 11p changes the transfer function/noise seen by the stored tables. */
#define CAL_GENERATION 3u
static void calibration_generation(void)
{
    uint16_t stored = 0;
    if (blob_load("cal_gen", &stored, sizeof(stored)) && stored == CAL_GENERATION) return;
    printf("CALIBRATION generation stored=%u firmware=%u: calibration of an earlier firmware discarded, using baseline; "
           "manual calibration only\n", stored, CAL_GENERATION);
    const filter_blob_t none = {0};
    dco_index_init(&s_dco_index);
    s_dco_index_loaded = true;                 /* never adopt or upgrade the old tables */
    dc_flags_init(&s_dc_flags);
    s_dc_flags_loaded = true;
    const bool ok = blob_store("filter", &none, sizeof(none)) &&
                    blob_store("dco_idx", &s_dco_index, sizeof(s_dco_index)) &&
                    blob_store("dc_recal", &s_dc_flags, sizeof(s_dc_flags));
    const uint16_t now = CAL_GENERATION;
    if (!ok || !blob_store("cal_gen", &now, sizeof(now)))
        printf("CALIBRATION error: generation not stored, discarded again next boot\n");
}

uint16_t calibration_generation_id(void) { return CAL_GENERATION; }

static uint8_t sphase_code(void)
{
    static const char *const names[] = {"unverified", "settled", "drifted", "unsettled", "noise_limited", "probe_failed"};
    for (unsigned k = 0; k < sizeof(names) / sizeof(names[0]); ++k)
        if (!strcmp(s_sphase_state, names[k])) return (uint8_t)k;
    return 0xffu;
}

void calibration_flog_live(int dc[2], int off[2], uint32_t *servo_writes, uint8_t *sphase, int8_t *temp_c)
{
    dc[0] = s_drift_avg[0]; dc[1] = s_drift_avg[1];
    off[0] = dco_offset(s_current_gain, 0); off[1] = dco_offset(s_current_gain, 1);
    *servo_writes = s_servo_writes;
    *sphase = sphase_code();
    *temp_c = (int8_t)(s_temp_c <= -99.f ? -128 : s_temp_c > 127.f ? 127 : (int)s_temp_c);   /* -128: not read yet */
}

/* Flight log: what this startup phase calibrated and the state it left. */
static void calib_record(bool manual, int64_t t0)
{
    uint8_t p[80] = {0};
    p[0] = manual;
    p[1] = (uint8_t)(!strcmp(s_filter_result, "loaded") ? 1 : !strcmp(s_filter_result, "stored") ? 2 : 3);
    flog_put32(p + 2, (uint32_t)((esp_timer_get_time() - t0) / 1000));
    static const int classes[3] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE};
    for (unsigned c = 0; c < 3u; ++c) {
        rf_filter_t f;
        uint8_t *e = p + 6 + c * 8;
        if (!filter_setting(classes[c], &f)) continue;
        const filter_entry_t *x = c == 0 ? &s_filter_cal.normal : c == 1 ? &s_filter_cal.narrow : &s_filter_cal.wide;
        if (!x->valid) continue;
        e[0] = 1; e[1] = f.bw20; e[2] = (uint8_t)(int8_t)f.code; e[3] = (uint8_t)f.skirt;
        flog_put16(e + 4, x->width_khz); flog_put16(e + 6, x->nbw_khz);
    }
    for (unsigned k = 0; k < DCO_READY_MAX; ++k) {
        uint8_t *e = p + 30 + k * 5;
        flog_put16(e, s_dco_ready[k].freq); flog_put16(e + 2, s_dco_ready[k].key); e[4] = s_dco_ready[k].ready;
    }
    p[45] = (uint8_t)dco_valid_rows(); p[46] = (uint8_t)dco_rows(0); p[47] = (uint8_t)dco_template_rows();
    p[48] = (uint8_t)s_dc_flags.count;
    p[49] = sphase_code();
    flog_put16(p + 50, (uint16_t)(int16_t)s_drift_avg[0]); flog_put16(p + 52, (uint16_t)(int16_t)s_drift_avg[1]);
    p[54] = (uint8_t)(int8_t)(s_temp_c < -100.f ? -128 : (int)s_temp_c);
    flightlog_record(FLOG_T_CALIB, p, 56);
}

void calibration_start(void)
{
    calibration_generation();
    rf_filter_t loaded;
    if (blob_load("filter", &s_filter_cal, sizeof(s_filter_cal)) && filter_setting(FILTER_NORMAL, &loaded)) {
        rf_set_filter(loaded);
        s_filter_result = "loaded";
    } else {
        memset(&s_filter_cal, 0, sizeof(s_filter_cal));
        const rf_filter_t baseline = {16, 32, false};
        s_filter_result = rf_set_filter(baseline) ? "baseline_unmeasured" : "baseline_not_applied";
    }
    ESP_ERROR_CHECK(xTaskCreate(calibration_task, "calibration", 8192, NULL, 1, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
}

void calibration_print(void)
{
    rf_filter_t f = rf_filter();
    const unsigned valid = dco_valid_rows();
    printf("FILTER %s active bw=%s code=%d skirt=%u | normal width_khz=%u nbw_khz=%u | narrow valid=%u "
           "width_khz=%u nbw_khz=%u | wide valid=%u width_khz=%u nbw_khz=%u | calibrated_nbw_khz=%u\n",
           s_filter_result, f.bw20 ? "BW20" : "BW40", f.code, f.skirt, s_filter_cal.normal.width_khz,
           s_filter_cal.normal.nbw_khz, s_filter_cal.narrow.valid, s_filter_cal.narrow.width_khz,
           s_filter_cal.narrow.nbw_khz, s_filter_cal.wide.valid, s_filter_cal.wide.width_khz,
           s_filter_cal.wide.nbw_khz, s_filter_cal.calibrated_nbw_khz);
    uint8_t hw[8] = {0}, want[8] = {0};
    const int bad = phy_filter_hw_mismatch(hw, want);
    printf("PHY11P want=1 hw_enabled=%d (-1=unsupported; analog profile checked below)\n", phy_11p_hw_enabled());
    printf("DFILT want=%d hw_mode=%d (-1=unsupported)\n",
           PHY_DFILT_MODE_DEFAULT, phy_dfilt_hw_mode());
    printf("FILTER_HW mismatch=%d hw=%02x%02x%02x%02x%02x%02x%02x%02x want=%02x%02x%02x%02x%02x%02x%02x%02x\n", bad,
           hw[0], hw[1], hw[2], hw[3], hw[4], hw[5], hw[6], hw[7],
           want[0], want[1], want[2], want[3], want[4], want[5], want[6], want[7]);
    printf("DCO held=%u gains_valid=%u floor_rows=%u missing_rows=%u floor=%u searches=%lu carrier_searches=%lu hook_holds=%lu aborts=%lu "
           "drift_avg_mcells=%d/%d nudges=%lu temp_c=%.1f\n", phy_dco_held(), valid, dco_rows(DCO_ROW_FLOOR), dco_rows(0),
           s_dco_floor,
           (unsigned long)s_dco_searches, (unsigned long)s_dco_carrier_searches, (unsigned long)s_dco_hook_holds,
           (unsigned long)s_dco_aborts, s_drift_avg[0], s_drift_avg[1], (unsigned long)s_servo_writes,
           (double)s_temp_c);
    {
        int b1[2], b2[2];
        const uint8_t g = s_current_gain;
        if (phy_dco_banks(b1, b2))
            printf("DCO_BANKS bank1=%d/%d bank2=%d/%d gain=%u row_codes=%d/%d row_valid=%u banned=%u\n", b1[0], b1[1],
                   b2[0], b2[1], g, g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].code[0] : -1,
                   g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].code[1] : -1,
                   g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].valid : 0u, g <= ARC_VENDOR_GAIN_MAX ? s_dco_banned[g] : 0u);
    }
    printf("CAL_POLICY auto_cal=0 auto_dc_trim=0 manual=F full=B health_cpu_us=%lld context=%lu\n",
           s_health_us, (unsigned long)s_basemap_context);
    dc_health_print();
    unsigned stored = 0;
    for (unsigned k = 0; k < DCO_SLOTS; ++k) stored += s_dco_index.freq[k] != 0;
    printf("DCO tables freq=%u template_rows=%u seeded_from=%u seeds=%lu stored_channels=%u/%u:", s_dco.freq,
           dco_template_rows(), s_dco_seed_mhz, (unsigned long)s_dco_seeds, stored, DCO_SLOTS);
    for (unsigned k = 0; k < DCO_SLOTS; ++k) if (s_dco_index.freq[k]) printf(" %u", s_dco_index.freq[k]);
    printf("\n");
    printf("DCO_REPAIR bad_dropped=%lu repairs=%lu repair_fails=%lu borrowed_holds=%lu (unconverged rows dropped, "
           "searched from a converged neighbour; a missing row holds the nearest converged one's codes)\n",
           (unsigned long)s_dco_bad_dropped, (unsigned long)s_dco_repairs, (unsigned long)s_dco_repair_fails,
           (unsigned long)s_dco_borrowed);
    printf("DCO_BAN unbans=%lu (a railing hold is released for 2-32 s, then held again)\n", (unsigned long)s_dco_unbans);
    printf("DCO_ADOPT disabled=manual_only\n");
    for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        printf("CAL_ROW freq=%u filter_key=%04x gain=%u codes=%d/%d residual_mcells=%d/%d valid=%u template=%u\n",
               s_dco.freq, live_filter_key(), g, s_dco.e[g].code[0], s_dco.e[g].code[1],
               s_dco.e[g].residual_mcells[0], s_dco.e[g].residual_mcells[1], s_dco.e[g].valid,
               dco_mask_get(s_dco_templ, g));
    printf("DCO_CONTEXT table_freq=%u table_bw20=%u table_code=%d table_skirt=%d "
           "active_freq=%u active_bw20=%u active_code=%d active_skirt=%u filter_same=%u\n",
           s_dco.freq, (unsigned)s_dco.filter_bw20, (int)s_dco.filter_code, (int)s_dco.filter_skirt,
           rf_frequency_mhz(), (unsigned)f.bw20, f.code, (unsigned)f.skirt,
           (unsigned)(s_dco.filter_bw20 == f.bw20 && s_dco.filter_code == f.code &&
                      s_dco.filter_skirt == (int)f.skirt));
    printf("SPHASE state=%s excess_mc2=%d limit=%d white_pm=%d dc_mcell=%d/%d scans=%u "
           "best=%d worst=%d contrast=%u rx_div=%lu carrier_qualified=%u live_limit=%d carrier_checks=%u "
           "carrier_scans=%u moved=%u false_alarms=%u strikes=%u\n", s_sphase_state,
           s_sphase_excess, SPHASE_LIMIT, s_sphase_white, s_sphase_dc[0], s_sphase_dc[1], s_sphase_scans,
           s_sphase_last.valid ? s_sphase_last.best : -1, s_sphase_last.valid ? s_sphase_last.worst : -1,
           sphase_contrast(&s_sphase_last), (unsigned long)PCR.parl_clk_rx_conf.parl_clk_rx_div_num + 1ul,
           s_sphase_live.qualified, SPHASE_LIVE_LIMIT, s_sphase_live.carrier_checks, s_sphase_live.scans,
           s_sphase_live.moved, s_sphase_live.false_alarms, s_sphase_live.strikes);
}
