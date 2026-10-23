/* C5VRX-5 by Twotoz and the C5VRX contributors: 50 ms control loop.
 * BOOT button and menu, AFC acquisition, PAL/NTSC detection, signal
 * strength and the channel scanner. Observes completed RX descriptors only. */
#include "receiver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "afc_ctrl.h"
#include "kp_ladder.h"
#include "output_level.h"
#include "kp_switch_guard.h"
#include "profile_logic.h"
#include "menu_entry.h"
#include "cn_estimate.h"
#include "analog_video_detect.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "phase8_gain_lut.h"
#include "phy.h"
#include "rf.h"
#include "flightlog.h"
#include "flightlog_format.h"

#define BOOT_BUTTON      GPIO_NUM_28    /* XIAO ESP32-C5 BOOT */
#define TICK_MS          50
#define LONG_TICKS       12             /* 0.6 s */
#define MENU_IDLE_TICKS  600            /* 30 s without a press closes the menu */
#define DAC_IDLE_TICKS   6000           /* 5 min on the bench-only DAC TEST page */
#define RETUNE_SETTLE_TICKS 10          /* 500 ms after a PHY transaction */
#define SCAN_PAIRS       4092u
#define CONTROL_STACK_BYTES 6144u

typedef struct {
    StaticTask_t task;
    StackType_t stack[CONTROL_STACK_BYTES/sizeof(StackType_t)] __attribute__((aligned(portBYTE_ALIGNMENT)));
} control_memory_t;
static control_memory_t *s_control_memory;

bool control_prepare(void)
{
    if(!s_control_memory) {
        s_control_memory=heap_caps_aligned_alloc(portBYTE_ALIGNMENT,sizeof(*s_control_memory),
            MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(s_control_memory)memset(s_control_memory,0,sizeof(*s_control_memory));
    }
    return s_control_memory!=NULL;
}

volatile bool s_menu_active;
/* A long press/'o' while the PHY is busy: retry from this owner, bounded. */
static menu_retry_t s_menu_retry;
static bool menu_open_checked(void)
{
    if (menu_open()) { menu_retry_done(&s_menu_retry); s_menu_cursor = 0; return true; }
    if (menu_open_busy()) menu_retry_busy(&s_menu_retry, esp_timer_get_time());
    else menu_retry_done(&s_menu_retry);
    return false;
}
volatile int s_signal_strength;
volatile bool s_scan_active;
volatile unsigned s_scan_progress;
volatile int s_cfo_khz;
volatile bool s_afc_locked;
volatile std_mode_t s_std_mode = STD_MODE_AUTO;
volatile video_standard_t s_video_std = VIDEO_STD_NTSC;
volatile bool s_detected_std_valid;
volatile video_standard_t s_detected_std = VIDEO_STD_NTSC;
volatile int64_t s_last_video_us;
QueueHandle_t s_commands;

/* One control-task scratch buffer: the control window (gain/kp/AFC) and,
 * during a scan (receiver paused), a separate endpoint area. Saves the
 * scan's malloc. */
#define SCAN_BUF_OFFSET 4096u
static uint8_t s_control_scratch[SCAN_BUF_OFFSET + SCAN_PAIRS];
static uint8_t *const s_window = s_control_scratch;
static uint8_t s_pal_votes, s_ntsc_votes;
static uint8_t s_last_std = 0xFF;      /* last stable standard (NVS) */
static unsigned s_afc_corrections, s_afc_weak_skips;
typedef struct {
    afc2_evidence_t history;
    uint64_t sampled_us, qualified_us;
    uint32_t context, qualified_gain_epoch;
    int32_t raw_porch_khz, residual_khz;
    int cn_x10;
    bool latest_valid, sticky_lock;
    const char *window, *held;
} afc_evidence_snapshot_t;
static afc_evidence_snapshot_t s_afc_evidence = {.window="boot", .held="boot", .cn_x10=CN_UNKNOWN_X10};
/* Verified kp ladder (kp_ladder.h), NVS "kp_auto": 0 off, 1 kp only,
 * 2 kp plus the live U85Q <-> U85Q-LP switch. K cycles 0 -> 1 -> 2 -> 0;
 * k steps one kp level and P toggles the program by hand for the bench test
 * (both set auto off). */
static kp_ladder_t s_kp;
static kp_switch_guard_t s_auto_candidate;
/* Ladder evidence (T CONTROL line): windows offered, and why the unusable
 * ones were (board walk 2026-10-13: the log could not show it). */
static uint32_t s_ladder_windows, s_ladder_copy_fail, s_ladder_settle, s_ladder_gain, s_ladder_ctx, s_ladder_stuck,
    s_guard_confirms, s_kp_restarts_skipped;   /* skipped: a single kp level not worth a restart */
static uint8_t s_kp_auto;
/* LP policy (PR #230): 1 = rescue by the C/N benefit table (default),
 * 0 = the earlier gain rule (G70 on, G65 off). Console O, NVS lp_cn. */
static uint8_t s_lp_by_cn = 1;
static int s_saved_kp_auto = -1;
static bool s_mode_reset_pending;
static void save_auto_mode(void)
{
    /* Holding P/k must not wear NVS by rewriting OFF on every press. */
    if (s_saved_kp_auto != s_kp_auto && blob_store("kp_auto", &s_kp_auto, 1))
        s_saved_kp_auto = s_kp_auto;
}
static void manual_auto_off(void)
{
    s_kp_auto = 0;
    save_auto_mode();
    s_mode_reset_pending = false;
    s_auto_candidate.candidate = false;
}
/* Learned offset sign (afc_ctrl.h sign check), persisted once observed. */
static volatile int8_t s_afc_sign = 1;
static volatile uint8_t s_afc_sign_state;
static bool s_afc_sign_saved;

static void standard_reset(void)
{
    s_pal_votes = s_ntsc_votes = 0;
    s_detected_std_valid = false;
}

/* Burst-confirmed standard from the AFC estimator; a few agreeing
 * windows make it stable and remembered for the next boot's menu. */
static void standard_vote(unsigned afc_standard)
{
    bool pal = afc_standard == 2u;
    if (pal) { if (s_pal_votes < 8u) ++s_pal_votes; if (s_ntsc_votes) --s_ntsc_votes; }
    else { if (s_ntsc_votes < 8u) ++s_ntsc_votes; if (s_pal_votes) --s_pal_votes; }
    if (s_pal_votes >= 3u && s_pal_votes >= s_ntsc_votes + 2u) { s_detected_std = VIDEO_STD_PAL; s_detected_std_valid = true; }
    else if (s_ntsc_votes >= 3u && s_ntsc_votes >= s_pal_votes + 2u) { s_detected_std = VIDEO_STD_NTSC; s_detected_std_valid = true; }
    if (s_detected_std_valid && s_last_std != (uint8_t)s_detected_std) {
        s_last_std = (uint8_t)s_detected_std;
        (void)blob_store("last_std", &s_last_std, 1);
    }
}

video_standard_t menu_standard(void)
{
    if (s_std_mode == STD_MODE_PAL) return VIDEO_STD_PAL;
    if (s_std_mode == STD_MODE_NTSC) return VIDEO_STD_NTSC;
    if (s_detected_std_valid) return s_detected_std;
    return s_last_std == VIDEO_STD_PAL ? VIDEO_STD_PAL : VIDEO_STD_NTSC;
}

/* 0..100 from phase coherence, envelope and gain headroom. */
static int strength(const metrics_t *m, uint8_t gain)
{
    if (m->q_phase < 18 || m->origin_pm > 850) return 0;
    int coherence = (m->q_phase - 18) * 100 / 62;
    int power = ((m->p_median >> 4) - 6) * 100 / 28;   /* ultrafine lane: 4^2 */
    int max_gain = rf_gain_table()->max_index;
    int headroom = (max_gain - (int)gain) * 100 / (max_gain > 2 ? max_gain - 2 : 1);
    coherence = coherence < 0 ? 0 : coherence > 100 ? 100 : coherence;
    power = power < 0 ? 0 : power > 100 ? 100 : power;
    headroom = headroom < 0 ? 0 : headroom > 100 ? 100 : headroom;
    return (coherence * 2 + power + headroom) / 4;
}

static void after_retune(void)
{
    ++s_control_epoch;
    {
        uint8_t e[3] = {(uint8_t)rf_channel_index()};
        flog_put16(e + 1, rf_frequency_mhz());
        flightlog_event(FLOG_E_CHANNEL, e, sizeof(e));
    }
    s_cfo_khz = 0;
    s_afc_locked = false;
    standard_reset();
    settings_save();
    profile_channel_changed();   /* another channel may carry another VTX */
    if (s_menu_active) menu_render();
    printf("[CHANNEL] %s %u MHz%s\n", rf_channel()->name, rf_channel()->freq_mhz,
           rf_freq_in_spec(rf_channel()->freq_mhz) ? "" : " (above the documented C5 range, not characterised)");
}

/* ---- Channel scanner -------------------------------------------------------
 * RF strength alone let 5 GHz Wi-Fi win (issue #128): a channel only
 * qualifies with analog-video confidence (line-period periodicity), and its
 * carrier must sit within 3 MHz of the channel centre (a VTX also shows on
 * neighbouring channels through the filter, and the 50 ns offset estimate
 * wraps every 20 MHz). Among centred candidates the strongest wins. */
_Static_assert(SCAN_BUF_OFFSET >= CONTROL_BYTES, "scan endpoints must not overlap the control window");
static uint8_t *const s_scan_buf = s_control_scratch + SCAN_BUF_OFFSET;

static analog_video_t scan_confidence(void)
{
    analog_video_t v[3] = {{0}};
    for (unsigned w = 0; w < 3u; ++w) {
        if (w) vTaskDelay(pdMS_TO_TICKS(10));
        if (copy_recent_endpoints(s_scan_buf, SCAN_PAIRS))
            v[w] = analog_video_detect(s_scan_buf, SCAN_PAIRS, c5vrx_phase8_gain_lut);
    }
    analog_video_t a = v[0], b = v[1], c = v[2], t;   /* median by confidence */
    if (a.confidence > b.confidence) { t = a; a = b; b = t; }
    if (b.confidence > c.confidence) { t = b; b = c; c = t; }
    if (a.confidence > b.confidence) { t = a; a = b; b = t; }
    return b;
}

static analog_video_t channel_confidence(uint8_t scan_gain, uint8_t max_gain, int *quality)
{
    vTaskDelay(pdMS_TO_TICKS(90));
    metrics_t m = {0};
    if (copy_control_window(s_window, CONTROL_BYTES)) m = measure_window(s_window, CONTROL_BYTES);
    if (quality) *quality = strength(&m, scan_gain);
    analog_video_t video = scan_confidence();
    /* A far VTX is starved at the scan gain: look again at maximum. */
    if (video.confidence < ANALOG_VIDEO_MIN_CONFIDENCE && m.origin_pm > 500) {
        receiver_set_gain(max_gain);
        vTaskDelay(pdMS_TO_TICKS(20));
        analog_video_t far = scan_confidence();
        receiver_set_gain(scan_gain);
        if (far.confidence > video.confidence) video = far;
    }
    return video;
}

static void scan(void)
{
    if (!receiver_pause("SCAN")) return;
    const size_t original = rf_channel_index();
    const uint8_t max_gain = rf_gain_table()->max_index, scan_gain = 52u;
    size_t best = original;
    int best_quality = -1, best_video = -1, best_offset = 0x7fffffff;
    s_scan_active = true;
    s_scan_progress = 0;
    receiver_set_gain(scan_gain);   /* every channel compared at one gain */
    for (size_t ch = 0; ch < FPV_CHANNELS; ++ch) {
        s_scan_progress = (unsigned)(ch * 100u / FPV_CHANNELS);
        if (s_menu_active) menu_render();
        if (rf_set_channel(ch) != ESP_OK) continue;
        int quality = 0;
        analog_video_t v = channel_confidence(scan_gain, max_gain, &quality);
        if (v.confidence < ANALOG_VIDEO_MIN_CONFIDENCE) continue;
        int off = abs(v.offset_khz);
        printf("SCAN candidate %s %u MHz video=%d offset_khz=%d quality=%d %s\n", rf_channel()->name,
               rf_channel()->freq_mhz, v.confidence, v.offset_khz, quality, v.standard == 1 ? "PAL" : "NTSC");
        bool centred = off <= 3000, best_centred = best_offset <= 3000;
        if ((centred && !best_centred) ||
            (centred == best_centred &&
             (quality > best_quality ||
              (quality == best_quality &&
               (off / 500 < best_offset / 500 ||
                (off / 500 == best_offset / 500 && v.confidence > best_video)))))) {
            best = ch;
            best_quality = quality;
            best_video = v.confidence;
            best_offset = off;
        }
    }
    /* Confirm the winner: still video, still centred. */
    if (best_video >= 0) {
        (void)rf_set_channel(best);
        analog_video_t v = channel_confidence(scan_gain, max_gain, NULL);
        if (v.confidence < ANALOG_VIDEO_MIN_CONFIDENCE || abs(v.offset_khz) > 3000) best_video = -1;
    }
    if (best_video < 0) best = original;
    (void)rf_set_channel(best);
    s_scan_active = false;
    receiver_set_gain(max_gain);
    receiver_resume();
    printf("SCAN result=%s channel=%s\n", best_video < 0 ? "none_kept" : "found", rf_channel()->name);
    after_retune();
}

/* ---- Menu actions ---------------------------------------------------------- */
static void cycle_video_standard(void)
{
    menu_set_standard_mode(s_std_mode == STD_MODE_AUTO ? STD_MODE_NTSC :
                           s_std_mode == STD_MODE_NTSC ? STD_MODE_PAL : STD_MODE_AUTO);
    settings_save();
}

static void menu_action(void)
{
    switch (s_menu_cursor) {
    case MENU_BAND: rf_next_band(); after_retune(); break;
    case MENU_CHANNEL: rf_next_channel(); after_retune(); break;
    case MENU_SCAN: scan(); break;
    case MENU_CALIBRATE: menu_close(); filter_request(); return;   /* runs once the menu is closed */
    case MENU_OUTPUT: (void)transport_set_output((transport_output() + 1u) % OUTPUT_LEVELS); break;
    case MENU_GAIN: receiver_set_gain_mode(receiver_gain_mode_next(receiver_gain_mode()), true); break;
    case MENU_FLYWHEEL: transport_sync_flywheel_set(!transport_sync_flywheel_enabled(), true); break;
    case MENU_DAC: menu_dac_step(); break;
    case MENU_VIDEO:
        cycle_video_standard();
        break;
    default: menu_close(); return;
    }
    if (s_menu_active) menu_render();
}

static void command(int c, unsigned *menu_idle)
{
    *menu_idle = 0;
    if (menu_status_active()) {   /* a calibration owns the picture */
        printf("COMMAND %c refused=busy (a calibration or lane test owns the picture; retry when it ends)\n", c);
        return;
    }
    switch (c) {
    case 'G': receiver_set_gain_mode(receiver_gain_mode_next(receiver_gain_mode()), true); break;
    /* Bench: the real gain step of every table row pair (receiver.c). */
    case 'Q': receiver_pair_sweep_toggle(); break;
    case 'U': transport_sync_flywheel_toggle(); break;
    case 'H': transport_live_lut_test(); break;   /* bench: live LUT writes */
    /* DAC self-test: open the menu on DAC TEST, then step like a hold. */
    case 'Z':
        if (!s_menu_active && !menu_open_checked()) break;
        if (s_menu_cursor != MENU_DAC) { s_menu_cursor = MENU_DAC; menu_render(); }
        else menu_dac_step();
        break;
    case 'o': if (s_menu_active) menu_close(); else (void)menu_open_checked(); break;
    case 'e': menu_close(); break;  /* idempotent web-menu EXIT */
    case 'v': cycle_video_standard(); break;
    case 'n': if (s_menu_active) { s_menu_cursor = (s_menu_cursor + 1) % MENU_ITEMS; menu_render(); } break;
    case 'x': if (s_menu_active) menu_action(); break;
    case 'c': rf_next_channel(); after_retune(); break;
    case 'b': rf_next_band(); after_retune(); break;
    case 's': scan(); break;
    case 'O':
        s_lp_by_cn = s_lp_by_cn ? 0u : 1u;
        (void)blob_store("lp_cn", &s_lp_by_cn, 1);
        printf("LP policy %s (saved)\n", s_lp_by_cn ? "RESCUE by C/N benefit (default)" : "GAIN (G70 on, G65 off; earlier rule)");
        break;
    case 'K':
        if (transport_p64()) { printf("KP refused: P64 has a fixed table (m selects U85Q)\n"); break; }
        s_kp_auto = (uint8_t)((s_kp_auto + 1u) % 3u);
        save_auto_mode();
        kp_ladder_reset(&s_kp);
        s_auto_candidate.candidate = false;
        (void)transport_kp_set(profile_base_level(profile_active()));
        s_mode_reset_pending = !transport_apply_auto(profile_base_level(profile_active()), false);
        printf("KP ladder %s\n", s_kp_auto == 2 ? "AUTO kp + LP (program changes briefly restart video; bench test)" :
               s_kp_auto == 1 ? "AUTO kp (brief restarts; bench test)" : "OFF");
        break;
    case 'P': {
        if (transport_p64()) { printf("LP refused: P64 has a fixed table (m selects U85Q)\n"); break; }
        manual_auto_off();
        const bool lp = !transport_lp();
        const bool menu = s_menu_active;
        uint32_t wait_before = transport_switch_wait_ms();
        bool ok = transport_set_lp(lp);
        if (!ok && wait_before)
            printf("PROGRAM manual refused=cooldown wait_ms=%lu (press P again after waiting)\n",
                   (unsigned long)transport_switch_wait_ms());
        else printf("PROGRAM manual %s: %s\n", transport_program_name(), !ok ?
                    "refused (see prog_verify_fail/prog_restart_fail in T)" :
                    menu ? "loaded at menu exit through the normal start sequence" : "switched via pipeline restart");
        break;
    }
    case 'A': {
        profile_set_live_mode(!profile_live_mode());
        printf("PROFILE live switch %s (saved): %s\n", profile_live_mode() ? "ON" : "OFF",
               profile_live_mode() ? "widen after one reliable result, narrow after two agreeing results"
                                   : "report only, PENDING for the next boot or CALIBRATE");
        break;
    }
    case 'k': case 'j': {
        if (transport_p64()) { printf("KP refused: P64 has a fixed table (m selects U85Q)\n"); break; }
        manual_auto_off();
        int next = (transport_kp_target() + (c == 'k' ? 1 : KP_LADDER_LEVELS - 1)) % KP_LADDER_LEVELS;
        printf("KP manual level=%d %s\n", next, transport_kp_set(next) ? "queued (brief pipeline restart)" :
               "refused (needs a verified level and live video)");
        break;
    }
    case 'g': {
        unsigned next = (transport_output() + 1u) % OUTPUT_LEVELS;
        bool ok = transport_set_output(next);
        printf("OUTPUT manual %s result=%s wait_ms=%lu%s\n", transport_output_name(),
               ok ? "applied" : "refused", (unsigned long)transport_switch_wait_ms(),
               transport_output() == 9 ? " (low swing: halves noise & clipping; requires sensitive sync)" :
               (transport_output() ? " WARNING: may clip wide VTX; verify sync/white into 75 ohm" : ""));
        if (s_menu_active) menu_render();
        break;
    }
    case 'm': {
        const bool ok = transport_set_p64(!transport_p64());
        printf("DEMOD %s result=%s (session only; boot default P64)\n",
               transport_program_name(), ok ? "applied" : "refused: cooldown or recovery failure");
        break;
    }
    default: break;
    }
}

static uint32_t afc_context(void)
{
    const uint32_t parts[5] = {(uint32_t)rf_channel_index(), s_control_epoch, (uint32_t)rf_offset_khz(),
                               rf_table_generation(), phy_generation()};
    uint32_t h = 2166136261u;
    for (unsigned k = 0; k < 5u; ++k) h = (h ^ parts[k]) * 16777619u;
    return h;
}

/* End-to-end video quality, one-second periods (video_quality.h). */
static vq_acc_t s_vq_acc;
static vq_t s_vq;
static uint32_t s_vq_periods;

static void control_task(void *arg)
{
    (void)arg;
    afc2_ctrl_t afc = {.target_khz = AFC_PORCH_TARGET_KHZ, .sign = s_afc_sign, .sign_state = s_afc_sign_state};
    uint8_t seen_sign_state = afc.sign_state;
    bool own_write = false;
    uint8_t afc_lost = 0;
    unsigned settle = RETUNE_SETTLE_TICKS, btn = 0, boot_grace = 20, menu_idle = 0, menu_refresh = 0;
    bool long_fired = false, was_locked = false, ladder_was_active = true;
    int64_t next_control_us = esp_timer_get_time();
    int seen_profile = profile_active();
    uint32_t seen_phy = phy_generation(), seen_control = s_control_epoch;
    uint32_t seen_ladder_context = afc_context();
    uint32_t seen_restart_errors = transport_restart_errors();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2));
        transport_sync_live_tick();
        /* The flywheel owner ticks every 2 ms; gain/AFC/ladder/button/menu
         * timing stays at its original 50 ms. */
        int64_t control_now = esp_timer_get_time();
        if (control_now < next_control_us) continue;
        next_control_us = control_now + TICK_MS * 1000;
        int c;
        for (unsigned k = 0; k < 8 && xQueueReceive(s_commands, &c, 0) == pdTRUE; ++k) command(c, &menu_idle);

        /* BOOT button: short = next channel (menu: next item); 0.6 s = menu
         * (menu: do the selected item). */
        /* A calibration status screen owns the picture: no menu, no idle exit. */
        if (menu_status_active()) {
            s_auto_candidate.candidate = false;
            btn = 0; long_fired = false; menu_idle = 0; continue;
        }
        if (boot_grace) { --boot_grace; btn = 0; long_fired = false; }
        else if (gpio_get_level(BOOT_BUTTON) == 0) {
            ++btn;
            menu_idle = 0;
            if (btn >= LONG_TICKS && !long_fired) {
                long_fired = true;
                if (!s_menu_active) (void)menu_open_checked();
                else menu_action();
            }
        } else if (btn) {
            if (!long_fired && btn >= 2) {
                if (s_menu_active) { s_menu_cursor = (s_menu_cursor + 1) % MENU_ITEMS; menu_render(); }
                else { rf_next_channel(); after_retune(); }
            }
            btn = 0;
            long_fired = false;
        }
        if (!s_menu_active && s_menu_retry.pending) {
            uint32_t refused = s_menu_retry.refused;
            if (menu_retry_due(&s_menu_retry, esp_timer_get_time())) (void)menu_open_checked();
            else if (s_menu_retry.refused != refused)
                printf("[MENU] refused: sync release still waiting for the PHY after 1 s; live video kept\n");
        }
        if (s_menu_active) {
            /* The dBm on the CHANNEL page: the gain observer stands still in
             * the menu, so its control windows feed the level, rolled once a
             * second (20 ticks). */
            static uint8_t level_ticks;
            if (copy_control_window(s_window, CONTROL_BYTES)) receiver_level_feed(s_window, CONTROL_BYTES);
            if (++level_ticks >= 20u) { level_ticks = 0; receiver_level_roll(false, false); }
            /* A multimeter needs longer than 30 s; the page is only reached
             * by pressing through the menu. */
            const unsigned idle_limit = s_menu_cursor == MENU_DAC ? DAC_IDLE_TICKS : MENU_IDLE_TICKS;
            if (++menu_idle >= idle_limit) {
                menu_close();
                printf("[MENU] closed after %u s idle\n", idle_limit * TICK_MS / 1000u);
            }
            else if (++menu_refresh >= 5) { menu_refresh = 0; menu_render(); }
        } else menu_idle = menu_refresh = 0;

        transport_poll_faults();
        if (transport_faulted() || seen_restart_errors != transport_restart_errors()) {
            seen_restart_errors = transport_restart_errors();
            manual_auto_off();
            kp_ladder_reset(&s_kp);
            int actual = transport_kp_level();
            if (actual >= 0) s_kp.level = (uint8_t)actual;
            s_kp.lp = transport_lp();
        }
        if (phy_generation() != seen_phy || s_control_epoch != seen_control) {
            seen_phy = phy_generation();
            seen_control = s_control_epoch;
            s_cfo_khz = 0;
            afc2_ctrl_invalidate(&afc);
            settle = own_write ? 2u : RETUNE_SETTLE_TICKS;
        }
        uint32_t gain_epoch = s_gain_epoch, ctx = afc_context();
        if (s_scan_active) {
            s_auto_candidate.candidate = false;
            afc2_ctrl_invalidate(&afc);
            memset(&s_afc_evidence, 0, sizeof(s_afc_evidence));
            s_afc_evidence.window="scan"; s_afc_evidence.held="scan";
            s_afc_evidence.cn_x10=CN_UNKNOWN_X10;
            diag_window(false, false, false);
            continue;
        }
        const bool copied = copy_control_window(s_window, CONTROL_BYTES);
        if (!copied) ++s_ladder_copy_fail;
        metrics_t m = {0};
        if (copied) {
            m = measure_window(s_window, CONTROL_BYTES);
            s_signal_strength = (s_signal_strength * 3 + strength(&m, s_current_gain) + 2) / 4;
        }

        /* AFC: burst/line-timing-confirmed sync/porch levels, never the
         * scene-biased FM slope; acquisition only, sticky once locked. */
        if (afc2_ctrl_sync(&afc, ctx, own_write)) { s_afc_locked = false; afc_lost = 0; }
        own_write = false;
        int64_t now = esp_timer_get_time();
        /* The kp estimate only needs a settled receiver (no retune, gain
         * write or context change); the AFC additionally needs a stationary
         * envelope. Near the FM threshold receiver noise alone fails that
         * test (12 MHz class: 46 % of windows pass at 0 dB C/N, 74 % at
         * 2 dB), which froze the ladder exactly at the range edge
         * (kp_ladder.h, research/ladder_gate.py). Gain-driven LP needs no
         * successful copy or C/N window; retune/PHY ownership still gates it. */
        const bool program_ok = settle == 0 && ctx == afc_context() && !phy_busy();
        const bool ladder_ok = copied && kp_ladder_window_usable(program_ok, ctx == afc_context(), now - s_last_gain_write_us);
        ++s_ladder_windows;
        s_ladder_settle += settle != 0;
        s_ladder_ctx += ctx != afc_context();
        s_ladder_gain += now - s_last_gain_write_us < KP_GAIN_WRITE_GUARD_US;
        s_ladder_stuck += s_obs_stuck;
        bool window_ok = ladder_ok && afc2_envelope_stationary(s_window, CONTROL_BYTES);
        const char *afc_window_reason = !copied ? "copy_missing" : settle ? "settle" :
            ctx != afc_context() ? "context" : phy_busy() ? "phy_busy" : !ladder_ok ? "gain_settle" :
            !window_ok ? "nonstationary" : "measured";
        if (settle) --settle;
        afc2_result_t r = {0};
        if (window_ok) {
            r = afc2_measure(s_window, CONTROL_BYTES, c5vrx_phase8_gain_lut);
            afc2_ctrl_observe_at(&afc, &r, (uint64_t)now);
            vq_add(&s_vq_acc, &r);
        } else afc2_ctrl_miss(&afc);   /* settle and context changes reset via afc2_ctrl_sync */
        if (vq_close(&s_vq_acc, now, &s_vq)) ++s_vq_periods;
        if (copied) s_cfo_khz = r.porch_pairs ? r.porch_khz : 0;
        /* Burst or line timing confirmed the syncs (afc2_measure). */
        bool valid = window_ok && r.lines && r.standard && r.porch_pairs && r.sync_pairs;
        const bool qualified=valid && r.polarity && gain_epoch==s_gain_epoch && ctx==afc_context();
        if (s_afc_evidence.context != ctx) s_afc_evidence.qualified_us=0;
        if (qualified) {
            s_afc_evidence.qualified_us=(uint64_t)now;
            s_afc_evidence.qualified_gain_epoch=gain_epoch;
            s_afc_evidence.raw_porch_khz=r.porch_khz;
            s_afc_evidence.residual_khz=r.porch_khz-afc.target_khz;
        }
        if (valid) { s_last_video_us = now; standard_vote(r.standard); profile_observe(&r); }
        bool was = s_afc_locked;
        if (copied) s_afc_locked = afc2_native_lock(s_afc_locked, valid, afc2_ctrl_can_lock(&afc, true), afc.n, &afc_lost);
        if (was && !s_afc_locked) afc2_ctrl_reacquire(&afc);
        /* Re-acquisition retunes only on a strong carrier: at the range edge
         * the porch estimate is pulled toward the receiver centre and would
         * walk the offset the wrong way (cn_estimate.h, afc_bias.py). A weak
         * window still feeds the estimate history, it just cannot step. */
        const int cn_x10 = receiver_obs_cn_x10();
        const char *afc_held = s_afc_locked ? "sticky_lock" : !valid ? "window" : s_menu_active ? "menu" : "actuator";
        if (!s_afc_locked && valid && !s_menu_active &&
            phy_try_actuator(phy_generation())) {
            int32_t step = 0;
            const bool strong = receiver_obs_cn_x10() >= AFC_MIN_CN_X10;   /* noise-referenced power C/N */
            if (afc2_ctrl_decide(&afc, gain_epoch == s_gain_epoch && strong, &step)) {
                rf_set_offset_khz(rf_offset_khz() + (int)step);
                s_last_gain_write_us = esp_timer_get_time();   /* restore re-forces gain */
                ++s_gain_epoch;
                ++s_afc_corrections;
                own_write = true;
                afc_held="stepped";
                printf("[AFC] step_khz=%ld offset_khz=%d cn_est_x10=%d coherence=%d gain=%u\n", (long)step,
                       rf_offset_khz(), cn_x10, s_obs_coherence, s_current_gain);
            } else if (!strong && afc2_ctrl_stable(&afc, &step) &&
                       (step > AFC2_CTRL_DEADBAND_KHZ || step < -AFC2_CTRL_DEADBAND_KHZ)) {
                ++s_afc_weak_skips;    /* would have stepped on a weak carrier */
            }
            if (!own_write) {
                int32_t median=0;
                afc_held=gain_epoch != s_gain_epoch ? "gain_changed" : !strong ? "weak_or_unknown_cn" :
                    afc.n<AFC2_CTRL_SAMPLES ? "collecting" : !afc2_ctrl_stable(&afc,&median) ? "unstable" :
                    (median>=-AFC2_CTRL_DEADBAND_KHZ && median<=AFC2_CTRL_DEADBAND_KHZ) ? "deadband" : "correction_budget";
            }
            phy_end_actuator();
        }
        s_afc_evidence.history=afc2_ctrl_evidence(&afc,(uint64_t)now);
        s_afc_evidence.sampled_us=(uint64_t)now;
        s_afc_evidence.context=ctx;
        s_afc_evidence.cn_x10=cn_x10;
        s_afc_evidence.latest_valid=qualified;
        s_afc_evidence.sticky_lock=s_afc_locked;
        s_afc_evidence.window=qualified ? "valid" : valid ? "stale_gain_or_context" : window_ok ? "no_confirmed_sync" : afc_window_reason;
        s_afc_evidence.held=afc_held;
        if (afc.sign_state != seen_sign_state) {
            seen_sign_state = afc.sign_state;
            s_afc_sign_state = afc.sign_state;
            printf("[AFC] offset sign %s: %+d\n", afc.sign_state == AFC2_SIGN_FLIPPED ? "FLIPPED" : "confirmed",
                   afc.sign < 0 ? -1 : 1);
            int8_t sign = afc.sign < 0 ? -1 : 1;
            if (!s_afc_sign_saved || sign != s_afc_sign) {
                s_afc_sign = sign;
                s_afc_sign_saved = blob_store("afc_sign", &sign, 1);
            }
        }
        /* The ladder owns kp/LP on every profile (one U85Q topology; a
         * profile only sets the filter and the start level). It is inactive
         * only while the verified level is unknown (fault, pending restart).
         * A profile or receiver context change restarts it from the level
         * that now runs, so C/N never survives a channel/filter/offset/PHY
         * change into a context without its own noise reference. */
        const bool ladder_active = transport_kp_level() >= 0;
        const int profile = profile_active();
        if (ladder_active && (!ladder_was_active || profile != seen_profile ||
                              ctx != seen_ladder_context)) {   /* start from what runs */
            kp_ladder_reset(&s_kp);
            s_kp.level = (uint8_t)transport_kp_level();
            s_kp.lp = transport_lp();
            s_auto_candidate.candidate = false;
        }
        ladder_was_active = ladder_active;
        if (ladder_active) {
            seen_profile = profile;
            seen_ladder_context = ctx;
        }
        if (s_kp_auto && !s_menu_active && !ladder_active) {
            s_auto_candidate.candidate = false;
        } else if (s_kp_auto && !s_menu_active) {
            /* MATCHED profiles (start level below kp 0.85) run the narrow class:
             * decided by the profile, not by filter equality, since the narrow
             * and normal sweep entries may be the same setting; its curves
             * only when the narrow filter measured close to their 10 MHz model. */
            s_kp.narrow = kp_ladder_narrow_rows(profile_base_level(profile) > 0, calibration_narrow_width_khz());
            s_kp.wide = profile == PROFILE_WIDE;   /* the 16 MHz rows of a measured wide VTX */
            /* LP follows the AUTO gain (kp_ladder.h KP_LP_GAIN_*); manual gain uses C/N. */
            s_kp.gain = kp_ladder_gain_evidence(receiver_gain_mode() == GAIN_MODE_AUTO, s_current_gain);
            s_kp.lp_by_cn = s_lp_by_cn != 0;
            const bool program_usable = program_ok && ctx == afc_context();
            uint8_t level = kp_ladder_update_evidence(&s_kp, cn_x10, ladder_ok, program_usable,
                                                      profile_live_deviation_x100(), now, s_kp_auto == 2);
            bool usable = (s_kp.lp != transport_lp() ? program_usable : ladder_ok) && ctx == afc_context();
            /* Only a restart worth its picture gap (kp_switch_worth): a
             * single kp level waits; the ladder keeps its own level. */
            static int64_t last_kp_restart_us;
            const int applied = transport_kp_level();
            static int skipped_level = -1;   /* count a skipped restart once per wanted level, not per tick */
            if (!kp_switch_worth(level, s_kp.lp, applied, transport_lp(), last_kp_restart_us, now)) {
                if ((int)level != skipped_level) { skipped_level = level; ++s_kp_restarts_skipped; }
                level = (uint8_t)applied;
            } else {
                skipped_level = -1;
            }
            /* Confirm the program and the direction, not the exact level the
             * ladder is stepping through (kp_switch_guard.h). */
            if (kp_switch_confirm_toward(&s_auto_candidate, level, s_kp.lp, applied, transport_lp(),
                                         ctx, usable, now)) {
                ++s_guard_confirms;
                s_mode_reset_pending = !transport_apply_auto(level, s_kp.lp);  /* kp + LP in one guarded restart */
                if (!s_mode_reset_pending && (int)level != applied) last_kp_restart_us = now;
            }
        } else {
            s_auto_candidate.candidate = false;
            if (!s_kp_auto) {
                if (s_mode_reset_pending)
                    s_mode_reset_pending = !transport_apply_auto(profile_base_level(profile_active()), false);
                else (void)transport_kp_tick();
            }  /* manual target, same backend cooldown */
        }
        diag_window(window_ok, valid, own_write);   /* own_write: an AFC step this window */
        if (s_afc_locked && !was_locked)
            printf("[VIDEO] locked %s %u MHz %s offset_khz=%d gain=%u\n", rf_channel()->name,
                   rf_channel()->freq_mhz, r.standard == 2 ? "PAL" : "NTSC", rf_offset_khz(), s_current_gain);
        was_locked = s_afc_locked;
    }
}

void control_start(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    if (!blob_load("last_std", &s_last_std, 1)) s_last_std = 0xFF;
    int8_t sign;
    if (blob_load("afc_sign", &sign, 1) && (sign == 1 || sign == -1)) {
        s_afc_sign = sign;
        s_afc_sign_state = AFC2_SIGN_CONFIRMED;
        s_afc_sign_saved = true;
    }
    if (!blob_load("lp_cn", &s_lp_by_cn, 1) || s_lp_by_cn > 1) s_lp_by_cn = 1;
    if (!blob_load("kp_auto", &s_kp_auto, 1) || s_kp_auto > 2) s_kp_auto = 2;
    else s_saved_kp_auto = s_kp_auto; /* an explicitly saved OFF wins */
    s_commands = xQueueCreate(16, sizeof(int));
    ESP_ERROR_CHECK(s_commands ? ESP_OK : ESP_ERR_NO_MEM);
    /* The static zero-initialized ladder skipped its first reset because
     * the transport was already active. That left the adaptive LP hold at
     * zero on a normal boot (board log after PR 211). */
    kp_ladder_reset(&s_kp);
    const int actual = transport_kp_level();
    if (actual >= 0) s_kp.level = (uint8_t)actual;
    s_kp.lp = transport_lp();
    /* IDF task stack depths are bytes. Reserve before driver allocations,
     * then supply that same 6144-byte stack and TCB without another malloc. */
    ESP_ERROR_CHECK(control_prepare() &&
        xTaskCreateStatic(control_task,"control",CONTROL_STACK_BYTES,NULL,3,
            s_control_memory->stack,&s_control_memory->task)!=NULL ? ESP_OK:ESP_ERR_NO_MEM);
}

/* Status beat for the web menu (web/receiver-console.js): one short line
 * every 2 s from the console task, so the menu follows the receiver
 * without a status request. Read-only. */
void control_print_beat(void)
{
    printf("STATUS channel=%s freq=%u afc_locked=%u standard=%s detected=%s signal=%d menu=%u mode=%s kp_auto=%u "
           "output=%s live=%u busy=%u fw=%s\n",
           rf_channel()->name, rf_frequency_mhz(), s_afc_locked,
           s_std_mode == STD_MODE_AUTO ? "AUTO" : s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC",
           s_detected_std_valid ? (s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC") : "none",
           s_signal_strength, s_menu_active, profile_name(profile_active()), s_kp_auto, transport_output_name(),
           profile_live_mode(), menu_status_active(), firmware_id());
}

void control_flog_afc(int16_t *offset_khz, int16_t *residual_khz, uint32_t *corrections)
{
    *offset_khz = (int16_t)rf_offset_khz();
    *residual_khz = (int16_t)s_afc_evidence.residual_khz;
    *corrections = s_afc_corrections;
}

/* End-to-end video quality of the last second (video_quality.h). */
void control_video_quality(vq_t *out) { vTaskSuspendAll(); *out = s_vq; (void)xTaskResumeAll(); }

void control_print(void)
{
    vTaskSuspendAll();
    const afc_evidence_snapshot_t afc_snapshot=s_afc_evidence;
    const uint64_t afc_now=(uint64_t)esp_timer_get_time();
    const bool afc_context_current=afc_snapshot.context==afc_context();
    const bool afc_gain_current=afc_snapshot.qualified_us && afc_snapshot.qualified_gain_epoch==s_gain_epoch;
    (void)xTaskResumeAll();
    const afc2_evidence_t evidence=afc_snapshot.history;
    printf("AFC_EVIDENCE latest_valid=%u context_current=%u qualified_gain_current=%u sticky_lock=%u target_khz=%d "
           "last_qualified_raw_porch_khz=%ld last_qualified_residual_khz=%ld qualified_age_ms=%lld sample_age_ms=%lld "
           "n=%u timestamped=%u stable=%u median_residual_khz=%ld mad_khz=%ld oldest_age_ms=%lld newest_age_ms=%lld "
           "cn_x10=%d window=%s held=%s reference=porch_not_rf_truth\n",
           afc_snapshot.latest_valid, afc_context_current, afc_gain_current, afc_snapshot.sticky_lock, AFC_PORCH_TARGET_KHZ,
           (long)afc_snapshot.raw_porch_khz, (long)afc_snapshot.residual_khz,
           (long long)afc2_evidence_age_ms(afc_context_current && afc_gain_current ? afc_snapshot.qualified_us : 0,afc_now),
           (long long)afc2_evidence_age_ms(afc_snapshot.sampled_us,afc_now),
           evidence.n, evidence.timestamped, evidence.stable, (long)evidence.median_khz, (long)evidence.mad_khz,
           (long long)afc2_evidence_age_ms(evidence.oldest_us,afc_now),
           (long long)afc2_evidence_age_ms(evidence.newest_us,afc_now),
           afc_snapshot.cn_x10, afc_snapshot.window, afc_snapshot.held);
    printf("CONTROL channel=%s freq=%u offset_khz=%d cfo_khz=%d afc_locked=%u afc_corrections=%u "
           "afc_weak_skips=%u afc_sign=%+d(%s) standard=%s detected=%s signal=%d menu=%u deviation_x100=%d mode=%s "
           "kp_auto=%u%s kp_steps=%lu lp_switches=%lu busy=%u\n"
           "LADDER windows=%lu unusable_settle=%lu unusable_gain_write=%lu unusable_context=%lu copy_fail=%lu "
           "stuck_iq=%lu guard_confirms=%lu guard_resets=%lu level=%u lp=%u lp_release_hold_ms=%ld cn_smoothed_x10=%ld kp_restarts_skipped=%lu lp_policy=%s\n",
           rf_channel()->name, rf_frequency_mhz(), rf_offset_khz(), s_cfo_khz, s_afc_locked,
           s_afc_corrections, s_afc_weak_skips, s_afc_sign,
           s_afc_sign_state == AFC2_SIGN_FLIPPED ? "flipped" : s_afc_sign_state == AFC2_SIGN_CONFIRMED ? "confirmed" :
           "unverified", s_std_mode == STD_MODE_AUTO ? "AUTO" : s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC",
           s_detected_std_valid ? (s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC") : "none",
           s_signal_strength, s_menu_active, profile_deviation_x100(rf_channel_index()),
           profile_name(profile_active()), s_kp_auto,
           transport_p64() ? "(inactive: P64 fixed table)" :
           s_kp_auto && transport_kp_level() < 0 ? "(inactive: kp level unverified)" : "", (unsigned long)s_kp.steps,
           (unsigned long)s_kp.lp_switches, menu_status_active(),
           (unsigned long)s_ladder_windows, (unsigned long)s_ladder_settle, (unsigned long)s_ladder_gain,
           (unsigned long)s_ladder_ctx, (unsigned long)s_ladder_copy_fail, (unsigned long)s_ladder_stuck,
           (unsigned long)s_guard_confirms, (unsigned long)s_auto_candidate.resets, s_kp.level, s_kp.lp,
           (long)(s_kp.release_hold_us / 1000), (long)(s_kp.cn_q4 / 16),
           (unsigned long)s_kp_restarts_skipped, s_lp_by_cn ? "cn_rescue" : "gain");
    vq_t vq;
    control_video_quality(&vq);
    printf("VIDEO_Q known=%u snr_x10=%d lines_x100=%u lines=%lu windows=%lu periods=%lu (end-to-end: sync/porch "
           "spread of the demodulated video, last second)\n", vq.known, vq.snr_x10, vq.lines_x100,
           (unsigned long)vq.lines, (unsigned long)vq.windows, (unsigned long)s_vq_periods);
}

