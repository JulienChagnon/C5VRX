/* C5VRX-5 by Twotoz and the C5VRX contributors: onboard flight log
 * (docs/FLIGHT_LOG.md, layout in flightlog_format.h).
 *
 * Producers (control, diag, calibration, profile, transport) only copy a few
 * bytes into a RAM queue. A priority-1 task appends the queue to the current
 * flash block every FLOG_FLUSH_US (NOR flash programs into the erased area of
 * a block; a block's sector is erased once, when it is started), closes a full
 * block with its CRC, and writes IQ snapshots. Video is hardware and never
 * waits for it; a power cut loses at most the unflushed seconds. */
#include "flightlog.h"
#include "flightlog_format.h"
#include "receiver.h"
#include "cn_estimate.h"
#include "profile_logic.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "rf.h"
#include "snapshot.h"

#define FLOG_SUBTYPE     0x99
#define FLOG_QUEUE       1024u
#define FLOG_FLUSH_US    5000000LL
#define FLOG_SNAP_GAP_US 10000000LL
#define FLOG_TRACE_GAP_US 30000000LL

static const esp_partition_t *s_part;
static bool s_ready;
static uint32_t s_session, s_seq, s_snap_seq;
/* What each snapshot slot holds (flog_snap_victim). */
static uint8_t s_slot_trigger[FLOG_SNAP_SLOTS];
static uint32_t s_slot_seq[FLOG_SNAP_SLOTS];
static bool s_slot_used[FLOG_SNAP_SLOTS];
static uint32_t s_snap_skipped;
/* Requests dropped without a snapshot: too little heap, or a busy or unsettled
 * receiver (each request is consumed, the moment is lost). */
static uint32_t s_snap_no_memory, s_snap_refused;
static unsigned s_block;            /* record block (0..FLOG_RECORD_BLOCKS-1) being written */
static unsigned s_used;             /* record bytes in it */
static bool s_block_open;
/* The queue is only copied, so it can live in PSRAM. s_flush_buf is an
 * esp_partition_write source and stays internal: from PSRAM, esp_flash
 * writes 32 bytes per page program with the cache off. */
static EXT_RAM_BSS_ATTR uint8_t s_queue[FLOG_QUEUE];
static uint8_t s_flush_buf[FLOG_QUEUE];
static unsigned s_queue_len;
static uint32_t s_dropped, s_dropped_logged, s_blocks_written, s_snapshots, s_flash_errors;
static portMUX_TYPE s_qlock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_flash_lock;
/* Pending snapshot trigger, written by several tasks: the merge and the
 * logger's take-and-clear both run under s_snap_lock, so a more important
 * request is never overwritten or cleared unseen. */
static portMUX_TYPE s_snap_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_snap_request;
static volatile bool s_dumping;
bool flightlog_dumping(void) { return s_dumping; }
static int64_t s_snap_last_us = -FLOG_SNAP_GAP_US;

bool flightlog_ready(void) { return s_ready; }

static size_t block_offset(unsigned block) { return (size_t)(1u + block) * FLOG_SECTOR; }
static size_t snap_offset(unsigned slot) { return (size_t)(FLOG_SNAP_FIRST + slot * FLOG_SNAP_SECTORS) * FLOG_SECTOR; }

void flightlog_record(uint8_t type, const void *payload, uint8_t len)
{
    if (!s_ready) return;
    taskENTER_CRITICAL(&s_qlock);
    if (s_queue_len + 2u + len > FLOG_QUEUE) {
        ++s_dropped;
    } else {
        s_queue[s_queue_len] = type;
        s_queue[s_queue_len + 1u] = len;
        memcpy(s_queue + s_queue_len + 2u, payload, len);
        s_queue_len += 2u + len;
    }
    taskEXIT_CRITICAL(&s_qlock);
}

void flightlog_event(uint8_t kind, const void *data, uint8_t len)
{
    uint8_t p[40];
    if (len > sizeof(p) - 5u) len = sizeof(p) - 5u;
    flog_put32(p, (uint32_t)(esp_timer_get_time() / 1000));
    p[4] = kind;
    if (len) memcpy(p + 5, data, len);
    flightlog_record(FLOG_T_EVENT, p, (uint8_t)(5u + len));
}

/* ---- flash side (logger task / console, under s_flash_lock) ------------ */

static bool flash_write(size_t offset, const void *data, size_t len)
{
    if (esp_partition_write(s_part, offset, data, len) == ESP_OK) return true;
    ++s_flash_errors;
    return false;
}

static bool block_open(void)
{
    const size_t at = block_offset(s_block);
    if (esp_partition_erase_range(s_part, at, FLOG_SECTOR) != ESP_OK) { ++s_flash_errors; return false; }
    uint8_t head[FLOG_BLOCK_HEAD];
    flog_block_head(head, s_session, s_seq, (uint32_t)(esp_timer_get_time() / 1000));
    if (!flash_write(at, head, sizeof(head))) return false;
    s_used = 0;
    s_block_open = true;
    return true;
}

static void block_close(void)
{
    if (!s_block_open) return;
    const size_t at = block_offset(s_block);
    uint8_t chunk[256];
    uint32_t crc = 0;
    for (unsigned k = 0; k < FLOG_BLOCK_CRC_AT; k += sizeof(chunk)) {
        const unsigned n = FLOG_BLOCK_CRC_AT - k < sizeof(chunk) ? FLOG_BLOCK_CRC_AT - k : sizeof(chunk);
        if (esp_partition_read(s_part, at + k, chunk, n) != ESP_OK) { ++s_flash_errors; return; }
        crc = flog_crc32(crc, chunk, n);
    }
    uint8_t c[4];
    flog_put32(c, crc);
    (void)flash_write(at + FLOG_BLOCK_CRC_AT, c, 4);
    ++s_blocks_written;
    s_block_open = false;
    s_block = (s_block + 1u) % FLOG_RECORD_BLOCKS;
    ++s_seq;
}

/* Append the queued records; a record never splits across blocks. */
static void flush(void)
{
    taskENTER_CRITICAL(&s_qlock);
    const unsigned n = s_queue_len;
    memcpy(s_flush_buf, s_queue, n);
    s_queue_len = 0;
    const uint32_t dropped = s_dropped;
    taskEXIT_CRITICAL(&s_qlock);
    unsigned at = 0;
    while (at < n) {
        if (!s_block_open && !block_open()) return;
        unsigned run = 0;   /* whole records that fit the block */
        while (at + run < n) {
            const unsigned rec = 2u + s_flush_buf[at + run + 1u];
            if (s_used + run + rec > FLOG_BLOCK_ROOM) break;
            run += rec;
        }
        if (run) {
            if (!flash_write(block_offset(s_block) + FLOG_BLOCK_HEAD + s_used, s_flush_buf + at, run)) return;
            s_used += run;
            at += run;
        }
        if (at < n) block_close();
    }
    if (dropped != s_dropped_logged) {
        uint8_t d[4];
        flog_put32(d, dropped - s_dropped_logged);
        s_dropped_logged = dropped;
        flightlog_event(FLOG_E_DROPPED, d, 4);
    }
}

static void snapshot(uint8_t trigger)
{
    const int victim = flog_snap_victim(s_slot_trigger, s_slot_seq, s_slot_used, FLOG_SNAP_SLOTS, trigger);
    if (victim < 0) { ++s_snap_skipped; return; }   /* every slot holds a more important moment */
    if (s_menu_active || receiver_paused() || calibration_in_startup()) { ++s_snap_refused; return; }
    if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < C5V4_LEVEL_SAMPLE_BYTES + 8192u) {
        ++s_snap_no_memory;
        return;
    }
    uint8_t *raw = malloc(C5V4_LEVEL_SAMPLE_BYTES);
    if (!raw) { ++s_snap_no_memory; return; }
    const uint32_t epoch = s_gain_epoch;
    const bool settled = esp_timer_get_time() - s_last_gain_write_us >= 500 && s_gain.state != DG_SETTLE;
    if (!settled || !copy_snapshot(raw) || epoch != s_gain_epoch) { free(raw); ++s_snap_refused; return; }
    int dc[2] = {0, 0}, off[2] = {0, 0};
    uint32_t writes;
    uint8_t sph;
    int8_t temp;
    calibration_flog_live(dc, off, &writes, &sph, &temp);
    const rf_filter_t f = rf_filter();
    uint8_t demod_id, output_level;
    transport_flog_identity(&demod_id, &output_level);
    const flog_snap_t s = {
        .trigger = trigger, .gain = s_current_gain, .lane = (uint8_t)rf_lane(), .profile = (uint8_t)profile_active(),
        .kp = 0xffu, .lp = 0u, .freq_mhz = rf_frequency_mhz(),
        .filter_key = (uint16_t)((f.bw20 ? 0x8000u : 0u) | (((unsigned)(f.code + 1) & 0x7fu) << 8) | (f.skirt & 0xffu)),
        .session = s_session, .seq = s_snap_seq, .t_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .off_i = (int16_t)off[0], .off_q = (int16_t)off[1], .dc_i_mcells = (int16_t)dc[0], .dc_q_mcells = (int16_t)dc[1],
        .demod_id = demod_id, .output_level = output_level,
        .output_flags = (uint8_t)(demod_id ? FLOG_OUT_DEMOD_VERIFIED : 0u),
    };
    uint8_t head[FLOG_SNAP_HEAD];
    flog_snap_head(head, &s, raw);
    const unsigned slot = (unsigned)victim;
    const size_t at = snap_offset(slot);
    bool ok = esp_partition_erase_range(s_part, at, FLOG_SNAP_SECTORS * FLOG_SECTOR) == ESP_OK &&
              flash_write(at, head, sizeof(head)) && flash_write(at + FLOG_SNAP_HEAD, raw, FLOG_SNAP_BYTES);
    free(raw);
    if (!ok) { ++s_flash_errors; return; }
    s_slot_used[slot] = true;
    s_slot_trigger[slot] = trigger;
    s_slot_seq[slot] = s_snap_seq;
    ++s_snap_seq;
    ++s_snapshots;
    s_snap_last_us = esp_timer_get_time();
    uint8_t e[6] = {(uint8_t)slot, trigger};
    flog_put32(e + 2, s.seq);
    flightlog_event(FLOG_E_SNAPSHOT, e, sizeof(e));
}

void flightlog_snapshot_request(uint8_t trigger)
{
    if (!s_ready) return;
    taskENTER_CRITICAL(&s_snap_lock);
    s_snap_request = flog_snap_request_merge(s_snap_request, trigger);
    taskEXIT_CRITICAL(&s_snap_lock);
}

static void logger_task(void *arg)
{
    (void)arg;
    int64_t last_flush = esp_timer_get_time(), last_periodic = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        const int64_t now = esp_timer_get_time();
        /* A reference snapshot every 10 minutes. */
        if (!last_periodic) last_periodic = now;
        else if (now - last_periodic >= 600000000LL) { flightlog_snapshot_request(FLOG_SNAP_PERIODIC); last_periodic = now; }
        if (xSemaphoreTake(s_flash_lock, 0) != pdTRUE) continue;      /* a dump owns the flash */
        uint8_t t = 0;
        taskENTER_CRITICAL(&s_snap_lock);
        if (s_snap_request && now - s_snap_last_us >= FLOG_SNAP_GAP_US) { t = s_snap_request; s_snap_request = 0; }
        taskEXIT_CRITICAL(&s_snap_lock);
        if (t) snapshot(t);   /* outside the lock: it erases and writes flash */
        if (now - last_flush >= FLOG_FLUSH_US || s_queue_len > FLOG_QUEUE / 2u) {
            flush();
            last_flush = now;
        }
        xSemaphoreGive(s_flash_lock);
    }
}

/* ---- session ------------------------------------------------------------ */

static void session_record(void)
{
    uint8_t p[64] = {0};
    flog_put32(p, s_session);
    p[4] = FLOG_FORMAT;
    p[5] = (uint8_t)esp_reset_reason();
    snprintf((char *)p + 6, 24, "%s", firmware_id());
    flog_put16(p + 30, calibration_generation_id());
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    p[32] = (uint8_t)(chip.revision & 0xffu);
    uint8_t mac[6] = {0};
    (void)esp_efuse_mac_get_default(mac);
    flog_put32(p + 33, flog_crc32(0x5a5a5a5au, mac, sizeof(mac)));   /* board id: not the MAC */
    const arc_gain_table_t *t = rf_gain_table();
    p[37] = t->max_index; p[38] = t->band5; p[39] = 0;   /* IQ scale: in the CALIB record context */
    p[40] = (uint8_t)rf_channel_index(); p[41] = (uint8_t)s_std_mode; p[42] = (uint8_t)transport_output();
    p[43] = receiver_gain_mode(); p[44] = 0; p[45] = profile_live_mode();   /* 44: removed flywheel, always 0 */
    flog_put16(p + 46, rf_frequency_mhz());
    flightlog_record(FLOG_T_SESSION, p, 48);
}

void flightlog_start(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)FLOG_SUBTYPE, "flightlog");
    if (!s_part || s_part->size < FLOG_PARTITION_BYTES) {
        printf("FLOG partition=missing (flash the full image to add it); flight log off\n");
        return;
    }
    s_flash_lock = xSemaphoreCreateMutex();
    uint8_t h[32];
    if (esp_partition_read(s_part, 0, h, sizeof(h)) != ESP_OK || !flog_header_ok(h)) {
        flog_header(h);
        if (esp_partition_erase_range(s_part, 0, FLOG_SECTOR) != ESP_OK ||
            esp_partition_write(s_part, 0, h, sizeof(h)) != ESP_OK) {
            printf("FLOG header write failed; flight log off\n");
            return;
        }
        printf("FLOG formatted (format %u)\n", FLOG_FORMAT);
    }
    flog_resume_t r;
    flog_resume_init(&r);
    for (unsigned k = 0; k < FLOG_RECORD_BLOCKS; ++k) {
        uint8_t b[FLOG_BLOCK_HEAD];
        if (esp_partition_read(s_part, block_offset(k), b, sizeof(b)) == ESP_OK)
            flog_resume_add(&r, k, FLOG_RECORD_BLOCKS, flog_parse_head(b));
    }
    s_session = r.next_session;
    s_seq = r.next_seq;
    s_block = r.next_block;
    for (unsigned k = 0; k < FLOG_SNAP_SLOTS; ++k) {
        uint8_t sh[16];
        if (esp_partition_read(s_part, snap_offset(k), sh, sizeof(sh)) != ESP_OK) continue;
        if (flog_get32(sh) != FLOG_MAGIC_SNAP) continue;
        s_slot_used[k] = true;
        s_slot_trigger[k] = sh[5];
        s_slot_seq[k] = flog_get32(sh + 12);
        if (s_slot_seq[k] + 1u > s_snap_seq) s_snap_seq = s_slot_seq[k] + 1u;
    }
    s_ready = true;
    session_record();
    printf("FLOG ready session=%lu block=%u seq=%lu snapshots_next=%lu\n", (unsigned long)s_session, s_block,
           (unsigned long)s_seq, (unsigned long)s_snap_seq);
    if (xTaskCreate(logger_task, "flightlog", 3072, NULL, 1, NULL) != pdPASS) {
        s_ready = false;
        printf("FLOG task failed; flight log off\n");
    }
}

/* ---- per-second summary and traces (diag.c) ----------------------------- */

void flightlog_second(const diag_record_t *r, uint8_t gain_min, uint8_t gain_max, uint8_t dropout, uint8_t rx_state)
{
    if (!s_ready) return;
    static uint32_t gw, ov, sw, ac;
    static bool have;
    vq_t vq;
    control_video_quality(&vq);
    int dc[2], off[2];
    uint32_t servo_writes, afc_corr;
    uint8_t sph;
    int8_t temp;
    calibration_flog_live(dc, off, &servo_writes, &sph, &temp);
    int16_t afc_off, afc_res;
    control_flog_afc(&afc_off, &afc_res, &afc_corr);
    static uint32_t wa, wr, sf, oa;
    const rx_level_t lv = receiver_level();
    const cna_reading_t cn = receiver_cn_reading();
    transport_flog_output_t output;
    transport_flog_output(&output);
    flog_second_t s = {
        .t_s = r->t_s, .vq_snr_x10 = vq.known ? vq.snr_x10 : 0, .vq_lines_x100 = vq.known ? vq.lines_x100 : 0,
        .cn_x10 = r->cn_x10, .video_pct = r->video_pct, .dropout = dropout,
        .gain_min = gain_min, .gain_mean = r->gain, .gain_max = gain_max,
        .clip_q = flog_sat8((uint32_t)(r->clip_pm < 0 ? 0 : r->clip_pm) / 4u),
        .gain_writes = have ? flog_sat8(r->gain_writes - gw) : 0, .overloads = have ? flog_sat8(r->overloads - ov) : 0,
        .dc_i = flog_clamp8(dc[0] / 50), .dc_q = flog_clamp8(dc[1] / 50),
        .off_i = flog_clamp8(off[0]), .off_q = flog_clamp8(off[1]),
        .servo_writes = have ? flog_sat8(servo_writes - sw) : 0,
        .afc_offset_khz = afc_off, .afc_residual_khz = afc_res,
        .afc_corrections = have ? flog_sat8(afc_corr - ac) : 0,
        .demod = (uint8_t)(profile_flog_code(r->profile) | ((r->kp_level < 0 ? 7u : (unsigned)r->kp_level & 7u) << 2) |
                           ((r->lp ? 1u : 0u) << 5)),
        .sphase = sph, .temp_c = temp,
        .flags = (uint16_t)(r->flags | (s_menu_active ? 0x100u : 0u) | (s_scan_active ? 0x200u : 0u) |
                            (calibration_in_startup() ? 0x400u : 0u) | (receiver_paused() ? 0x800u : 0u)),
        .rx_state = rx_state,
        .level_dbm_x10 = (int16_t)(lv.valid ? lv.carrier_dbm_x10 : FLOG_LEVEL_UNKNOWN),
        .noise_dbm = flog_clamp8((lv.noise_dbm_x10 - 5) / 10),
        .level_flags = (uint8_t)((lv.valid ? 1u : 0u) | (lv.above_noise ? 2u : 0u) | (lv.exact ? 4u : 0u) |
                                 (receiver_noise_learned() ? 8u : 0u)),
        .writes_attack = have ? flog_sat8(s_gain.writes_severe - wa) : 0,
        .writes_release = have ? flog_sat8(s_gain.writes_moderate - wr) : 0,
        .sentinel = have ? flog_sat8(receiver_sentinel_forwarded() - sf) : 0,
        .overloads_after_write = have ? flog_sat8(s_gain.overloads_after_write - oa) : 0,
        .cn_power_x10 = (int16_t)cn.power_x10, .cn_m2m4_x10 = (int16_t)cn.m2m4_x10,
        .cn_env_x10 = (int16_t)(s_obs_env_ratio_x100 > 0 ? cn_estimate_env_x10(s_obs_env_ratio_x100, calibration_filter_class_mhz())
                                                           : -990),
        .cn_conf_src = (uint8_t)((cn.confidence & 3u) | (cn.source << 2)), .cn_flags = cn.flags,
        .output_flags = output.flags, .demod_id = output.demod, .output_standard = output.standard,
        .output_level = output.output_level, .output_samples = output.samples,
        .output_fresh_samples = output.fresh_samples, .native_h_lock_pct = output.h_lock_pct,
        .native_v_lock_pct = output.v_lock_pct, .native_h_lines = output.native_h_lines,
        .repaired_tips = output.repaired_tips, .suppressed_tips = output.suppressed_tips,
        .longest_h_unlock_ms = output.longest_h_unlock_ms, .output_observer_gaps = output.observer_gaps,
    };
    wa = s_gain.writes_severe; wr = s_gain.writes_moderate; sf = receiver_sentinel_forwarded(); oa = s_gain.overloads_after_write;
    gw = r->gain_writes; ov = r->overloads; sw = servo_writes; ac = afc_corr; have = true;
    uint8_t p[FLOG_SECOND_LEN];
    flog_pack_second(&s, p);
    flightlog_record(FLOG_T_SECOND, p, sizeof(p));
    /* A sharp drop of the picture quality against its recent level. */
    static int16_t recent[30];
    static unsigned n;
    if (vq.known && vq.lines) {
        int16_t sorted[30];
        const unsigned m = n < 30u ? n : 30u;
        memcpy(sorted, recent, sizeof(sorted));
        for (unsigned a = 1; a < m; ++a)
            for (unsigned b = a; b > 0 && sorted[b - 1] > sorted[b]; --b) { int16_t t = sorted[b]; sorted[b] = sorted[b - 1]; sorted[b - 1] = t; }
        if (m >= 10u && vq.snr_x10 + 60 < sorted[m / 2]) flightlog_snapshot_request(FLOG_SNAP_VQ_DROP);
        if (!n) flightlog_snapshot_request(FLOG_SNAP_FIRST_CARRIER);
        recent[n % 30u] = vq.snr_x10;
        ++n;
    }
}

/* Trace entry, 8 bytes: ms after the first, sync %, gain, C/N (0.5 dB),
 * clip / 4, kp|lp, diag flags; 16 per record (part 0, 1, 2). */
void flightlog_trace_add(const diag_record_t *r, unsigned k, unsigned count)
{
    static uint8_t p[6 + 16 * 8];
    static uint32_t first_ms;
    static int64_t last_trace_us;
    static bool taking;
    if (!s_ready) return;
    /* At most one trace per FLOG_TRACE_GAP_US: with an unstable picture the
     * diagnostic trace fires on every sync onset and took ~75 % of the log
     * (board blackbox 2026-10-19: 177 trace records in 129 s). The per-second
     * records and the dropout events stay complete. */
    if (!k) {
        const int64_t now = esp_timer_get_time();
        taking = !last_trace_us || now - last_trace_us >= FLOG_TRACE_GAP_US;
        if (taking) last_trace_us = now;
        first_ms = r->t_ms;
    }
    if (!taking) return;
    const unsigned slot = k % 16u;
    if (!slot) { flog_put32(p, first_ms); p[4] = (uint8_t)(k / 16u); p[5] = 0; }
    uint8_t *e = p + 6 + slot * 8;
    flog_put16(e, (uint16_t)(r->t_ms - first_ms));
    e[2] = r->video_pct;
    e[3] = r->gain;
    e[4] = (uint8_t)flog_clamp8(r->cn_x10 <= -990 ? -128 : r->cn_x10 / 5);
    e[5] = flog_sat8((uint32_t)(r->clip_pm < 0 ? 0 : r->clip_pm) / 4u);
    e[6] = (uint8_t)((r->kp_level < 0 ? 7u : (unsigned)r->kp_level & 7u) | (r->lp ? 8u : 0u));
    e[7] = r->flags;
    p[5] = (uint8_t)(slot + 1u);
    if (slot == 15u || k + 1u == count) flightlog_record(FLOG_T_TRACE, p, (uint8_t)(6u + p[5] * 8u));
}

/* ---- console ------------------------------------------------------------ */

static unsigned used_blocks(void)
{
    unsigned used = 0;
    for (unsigned k = 0; k < FLOG_RECORD_BLOCKS; ++k) {
        uint8_t b[FLOG_BLOCK_HEAD];
        if (esp_partition_read(s_part, block_offset(k), b, sizeof(b)) == ESP_OK && flog_parse_head(b).valid) ++used;
    }
    return used;
}

/* The dump goes through the USB driver's ring buffer. The everyday console
 * path (main.c) feeds the 64-byte FIFO a byte at a time and drops bytes when
 * the host is slow for 50 ms. Board 2026-10-23, receiver live, same log: that
 * path took 39 s at 66 KiB/s, lost 10 of 4945 lines and 24 records; the
 * driver took 6.5-8.9 s at 290-400 KiB/s with none lost. The driver exists
 * only while W runs, so a board without a host never waits on USB. Its ring
 * is small because the heap is: the largest free block was 9.7 KiB live and
 * from 1 s after boot (an 8 KiB ring was no faster). The on-screen menu's
 * display leaves 1.6 KiB: the dump then uses the byte path, 10-13 s there
 * because the menu pauses the live work. FLOG_END reports heap= for this. */
#define FLOG_DUMP_TX_BYTES 4096u
#define FLOG_DUMP_WAIT_MS  1000u
static bool s_dump_usb, s_dump_failed;

static void dump_usb_open(void)
{
    usb_serial_jtag_driver_config_t cfg = { .tx_buffer_size = FLOG_DUMP_TX_BYTES, .rx_buffer_size = 256 };
    s_dump_failed = false;
    s_dump_usb = !usb_serial_jtag_is_driver_installed() &&
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >= FLOG_DUMP_TX_BYTES + 1024u &&
                 usb_serial_jtag_driver_install(&cfg) == ESP_OK;
    if (s_dump_usb) usb_serial_jtag_vfs_use_driver();   /* other tasks' prints go through the ring too */
}

static void dump_usb_close(void)
{
    if (!s_dump_usb) return;
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(FLOG_DUMP_WAIT_MS));
    usb_serial_jtag_vfs_use_nonblocking();
    (void)usb_serial_jtag_driver_uninstall();
    s_dump_usb = false;
}

/* One whole line; a host that stops reading ends the dump. */
static void dump_line(const char *fmt, ...)
{
    if (s_dump_failed) return;
    char line[600];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 1u, fmt, ap);
    va_end(ap);
    if (n <= 0 || (size_t)n >= sizeof(line) - 1u) return;
    if (!s_dump_usb) { fputs(line, stdout); vTaskDelay(1); return; }
    /* The console's own path writes CR LF; the driver sends bytes as given. */
    if (line[n - 1] == '\n') { line[n - 1] = '\r'; line[n++] = '\n'; }
    if (usb_serial_jtag_write_bytes(line, (size_t)n, pdMS_TO_TICKS(FLOG_DUMP_WAIT_MS)) != n) s_dump_failed = true;
    /* The driver owns the RX FIFO now: keep the download key working. */
    uint8_t c;
    while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1)
        if (c == '`') { dump_usb_close(); rf_reboot_to_download(); }
}

static void dump_range(size_t offset, size_t len)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t chunk[256];
    char line[513];
    for (size_t k = 0; k < len && !s_dump_failed; k += sizeof(chunk)) {
        const size_t n = len - k < sizeof(chunk) ? len - k : sizeof(chunk);
        if (esp_partition_read(s_part, offset + k, chunk, n) != ESP_OK) { dump_line("FLOG_ERR read=%u\n", (unsigned)(offset + k)); return; }
        for (size_t j = 0; j < n; ++j) { line[2 * j] = hex[chunk[j] >> 4]; line[2 * j + 1] = hex[chunk[j] & 15]; }
        line[2 * n] = 0;
        /* Every line carries its CRC: other tasks print too, and a reader
         * re-requests a dump for any line that did not arrive intact. */
        dump_line("FLOG_D %u %s %08lx\n", (unsigned)(offset + k), line, (unsigned long)flog_crc32(0, chunk, n));
    }
    vTaskDelay(1);
}

static void dump(void)
{
    xSemaphoreTake(s_flash_lock, portMAX_DELAY);
    s_dumping = true;
    flush();
    const int64_t t0 = esp_timer_get_time();
    const unsigned heap = (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    dump_usb_open();
    unsigned blocks = 0, snaps = 0;
    for (unsigned k = 0; k < FLOG_RECORD_BLOCKS; ++k) {
        uint8_t b[FLOG_BLOCK_HEAD];
        if (esp_partition_read(s_part, block_offset(k), b, sizeof(b)) == ESP_OK && flog_parse_head(b).valid) ++blocks;
    }
    for (unsigned k = 0; k < FLOG_SNAP_SLOTS; ++k) {
        uint8_t h[4];
        if (esp_partition_read(s_part, snap_offset(k), h, sizeof(h)) == ESP_OK && flog_get32(h) == FLOG_MAGIC_SNAP) ++snaps;
    }
    dump_line("FLOG_BEGIN format=%u bytes=%u blocks=%u snapshots=%u session=%lu fw=%s\n", FLOG_FORMAT,
              (unsigned)FLOG_PARTITION_BYTES, blocks, snaps, (unsigned long)s_session, firmware_id());
    dump_range(0, 32);
    for (unsigned k = 0; k < FLOG_RECORD_BLOCKS; ++k) {
        uint8_t b[FLOG_BLOCK_HEAD];
        if (esp_partition_read(s_part, block_offset(k), b, sizeof(b)) == ESP_OK && flog_parse_head(b).valid)
            dump_range(block_offset(k), FLOG_SECTOR);
    }
    for (unsigned k = 0; k < FLOG_SNAP_SLOTS; ++k) {
        uint8_t h[4];
        if (esp_partition_read(s_part, snap_offset(k), h, sizeof(h)) == ESP_OK && flog_get32(h) == FLOG_MAGIC_SNAP)
            dump_range(snap_offset(k), FLOG_SNAP_SECTORS * FLOG_SECTOR);
    }
    dump_line("FLOG_END blocks=%u snapshots=%u ms=%lu usb=%s heap=%u\n", blocks, snaps,
              (unsigned long)((esp_timer_get_time() - t0) / 1000), s_dump_usb ? "driver" : "fifo", heap);
    dump_usb_close();
    s_dumping = false;
    xSemaphoreGive(s_flash_lock);
}

static void erase_all(void)
{
    xSemaphoreTake(s_flash_lock, portMAX_DELAY);
    printf("FLOG erasing (about 20 s)...\n");
    bool ok = esp_partition_erase_range(s_part, 0, FLOG_PARTITION_BYTES) == ESP_OK;
    uint8_t h[32];
    flog_header(h);
    ok = ok && esp_partition_write(s_part, 0, h, sizeof(h)) == ESP_OK;
    taskENTER_CRITICAL(&s_qlock);
    s_queue_len = 0;
    taskEXIT_CRITICAL(&s_qlock);
    s_block = 0; s_used = 0; s_block_open = false; s_seq = 1; s_snap_seq = 0;
    memset(s_slot_used, 0, sizeof(s_slot_used));
    ++s_session;
    xSemaphoreGive(s_flash_lock);
    printf("FLOG erased=%u session=%lu\n", ok, (unsigned long)s_session);
    session_record();
}

void flightlog_console(char key)
{
    if (!s_ready) { printf("FLOG partition=missing (flash the full image to add it)\n"); return; }
    static int64_t erase_armed;
    const int64_t now = esp_timer_get_time();
    if (key == 'Y') {
        printf("FLOG status ok=1 format=%u session=%lu block=%u used=%u seq=%lu blocks_used=%u blocks_written=%lu "
               "snapshots=%lu snapshot_seq=%lu snapshots_skipped=%lu snapshots_no_memory=%lu snapshots_refused=%lu "
               "queue=%u dropped=%lu flash_errors=%lu\n", FLOG_FORMAT,
               (unsigned long)s_session, s_block, s_used, (unsigned long)s_seq, used_blocks(),
               (unsigned long)s_blocks_written, (unsigned long)s_snapshots, (unsigned long)s_snap_seq, (unsigned long)s_snap_skipped,
               (unsigned long)s_snap_no_memory, (unsigned long)s_snap_refused, s_queue_len,
               (unsigned long)s_dropped, (unsigned long)s_flash_errors);
    } else if (key == 'W') {
        dump();
    } else if (key == 'X') {
        if (erase_armed && now - erase_armed < 3000000) { erase_armed = 0; erase_all(); }
        else { erase_armed = now; printf("FLOG erase armed: press X again within 3 s to erase the flight log\n"); }
    }
}
