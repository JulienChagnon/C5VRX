/* C5VRX-5 by Twotoz and the C5VRX contributors: bench-only VTX-on PHY
 * sensitivity lab (console S). Extends the PR #254 VTX-off noise lab
 * (main/phy_noise_lab.c at 6274b2d, docs/PR254_PHY_FINDINGS.md) and the
 * pinned PHY lab in legacy/c5vrx4/main/phy_rx_lab.c (#150-155). The register
 * footprints below are PR #254's recovered write masks of the pinned
 * libphy (static evidence in that PR); 11p observation by SushiDude
 * (@Ready4Sushi). docs/PHY_SENSITIVITY_LAB.md has the inventory, the test
 * plan and the evidence limits.
 *
 * Every candidate runs STOCK-TEST-TEST-STOCK at one fixed gain, DC pair,
 * frequency, offset and filter, with the VTX carrier present, 16 contiguous
 * 8190-byte captures per stage. Each capture prints carrier-and-noise
 * metrics (sense_lab_logic.h); tools/phy_sense_lab.py scores them against
 * the shams. Nothing is saved: every masked write is restored and verified,
 * a failed restore reboots before the receiver resumes. The video output
 * keeps running, so the goggles show each setting live. */
#include "sense_lab.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "arc_phy.h"
#include "phase8_gain_lut.h"
#include "phy.h"
#include "receiver.h"
#include "rf.h"
#include "sense_lab_logic.h"
#include "snapshot.h"

#define SENSE_CAPTURES   16u
#define SENSE_DWELL_MS   300u
#define SENSE_SHAM_EVERY 6u
#define SENSE_SUITE_US   (INT64_C(900) * 1000000)
#define SENSE_ALL_US     (INT64_C(3600) * 1000000)

static volatile char s_request;

bool sense_lab_suite_valid(char suite) { return (suite >= '0' && suite <= '9') || suite == 'a'; }
void sense_lab_request(char suite) { if (sense_lab_suite_valid(suite)) s_request = suite; }
bool sense_lab_pending(void) { return s_request != 0; }

void sense_lab_help(void)
{
    printf("SENSLAB bench, VTX ON (weak, static), not in flight. Press a suite key within 5 s:\n"
           "  0 shams only, 1 NF-auto patterns, 2 IQ correction, 3 digital filter, 4 packet/RX policy,\n"
           "  5 clocks, 6 ADC selector/LO cap/ULP, 7 analog filter code, 8 gain rows, 9 DAC output off,\n"
           "  a all of 1..9 (about 5-10 min). Record with tools/phy_sense_lab.py; docs/PHY_SENSITIVITY_LAB.md\n");
}

#ifdef C5VRX_PHY_PINNED
extern uint8_t phy_i2c_readReg(uint8_t block, uint8_t host, uint8_t reg);
extern void phy_i2c_writeReg(uint8_t block, uint8_t host, uint8_t reg, uint8_t value);
extern uint16_t phy_pbus_rd(uint32_t block, uint32_t bank);
#define MMIO(a) (*(volatile uint32_t *)(uintptr_t)(a))

/* ---- candidate tables: PR #254 recovered footprints ---------------------- */
#define P_M SENSE_MMIO
#define P_I SENSE_I2C
static const sense_profile_t k_sham = {"SHAM", SENSE_K_PATCH, 0, 0, {{0}}};

static const sense_profile_t k_iq[] = {
    {"IQ_RX_0438_BIT29_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0x20000000, 0)}},
    {"IQ_RX_0438_BIT30_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0x40000000, 0)}},
    {"IQ_RX_0438_BIT31_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0x80000000, 0)}},
    {"IQ_RX_0438_ALL_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0xe0000000, 0)}},
    {"IQ_TX_0C0C_BIT13_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0C0C, 0x2000, 0)}},
    {"IQ_TX_0C0C_BIT14_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0C0C, 0x4000, 0)}},
    {"IQ_TX_0C0C_ALL_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0C0C, 0x6000, 0)}},
    /* Raw zero coefficient fields, never a claimed identity transform. */
    {"IQ_RX_COEFF_A_ZERO", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0x1fc00000, 0)}},
    {"IQ_RX_COEFF_B_ZERO", SENSE_K_PATCH, 1, 0, {P_M(0x600A0438, 0x003f0000, 0)}},
    /* All five coupled enables of phy_iq_corr_enable (RX and TX banks). */
    {"IQ_CORR_ALL_OFF", SENSE_K_PATCH, 2, 0, {P_M(0x600A0438, 0xe0000000, 0), P_M(0x600A0C0C, 0x6000, 0)}},
};
/* Production mode 0 against the earlier vendor modes; ADC selector kept. */
static const sense_profile_t k_dfilt[] = {
    {"DFILT_MODE4", SENSE_K_PATCH, 1, 0, {P_M(0x600A0430, 0x003c0000, 0x00100000)}},
    {"DFILT_MODE8", SENSE_K_PATCH, 1, 0, {P_M(0x600A0430, 0x003c0000, 0x00200000)}},
    {"DFILT_MODE12", SENSE_K_PATCH, 1, 0, {P_M(0x600A0430, 0x003c0000, 0x00300000)}},
};
/* Packet/RX policy footprints (phy_lltf_mask_en, phy_wifi_rifs_mode_en,
 * phy_vht_support, phy_rx11blr_cfg, phy_rx_11b_opt, phy_rx_sense_set,
 * CCA, FFT force, channel helpers, HEMU, PWDET). Names do not establish an
 * effect on continuous MODEM_DIAG: that is what this suite measures. */
static const sense_profile_t k_policy[] = {
    {"CHAN_A_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A7904, 0x400007, 0x400000)}},
    {"CHAN_A_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A7904, 0x400000, 0)}},
    {"CHAN_B_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A7074, 0x2000, 0x2000)}},
    {"CHAN_B_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A7074, 0x2000, 0)}},
    {"PKDADC_BIT31_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A0C38, 0x80000000, 0)}},
    {"BB_CCA_BIT30_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A7018, 0x40000000, 0)}},
    {"MAC_CCA_DISABLE", SENSE_K_PATCH, 1, 0, {P_M(0x600A4C5C, 0xf0000000, 0xa0000000)}},
    {"MAC_CCA_ENABLE", SENSE_K_PATCH, 1, 0, {P_M(0x600A4C5C, 0xf0000000, 0)}},
    {"FFT_FORCE_OFF", SENSE_K_PATCH, 1, 0, {P_M(0x600A7800, 0x00080000, 0)}},
    {"FFT_FORCE_SCALE0", SENSE_K_PATCH, 1, 0, {P_M(0x600A7800, 0x0ff80000, 0x00080000)}},
    {"RX11B_OPT_ARG0", SENSE_K_PATCH, 6, 0, {
        P_M(0x600A7044, 0x003f3f00, 0x003e1800), P_M(0x600A7124, 0x0000fc0f, 0x00006004),
        P_M(0x600A8004, 0x0000f000, 0x00006000), P_M(0x600A7104, 0x000001ff, 0x000001c8),
        P_M(0x600A71D8, 0x0003fe00, 0x0003cc00), P_M(0x600A7014, 0xff800000, 0xff000000)}},
    {"RX11B_OPT_ARG1", SENSE_K_PATCH, 6, 0, {
        P_M(0x600A7044, 0x003f3f00, 0x003f2100), P_M(0x600A7124, 0x0000fc0f, 0x00008403),
        P_M(0x600A8004, 0x0000f000, 0x00009000), P_M(0x600A7104, 0x000001ff, 0x000001bc),
        P_M(0x600A71D8, 0x0003fe00, 0x0003cc00), P_M(0x600A7014, 0xff800000, 0xff000000)}},
    {"RXSENSE_ARG0", SENSE_K_PATCH, 4, 0, {
        P_M(0x600A7010, 0xff800000, 0), P_M(0x600A7014, 0xff800000, 0),
        P_M(0x600A7044, 0x000000ff, 0), P_M(0x600A7108, 0x00000200, 0x00000200)}},
    {"RXSENSE_ARG1", SENSE_K_PATCH, 4, 0, {
        P_M(0x600A7010, 0xff800000, 0x00800000), P_M(0x600A7014, 0xff800000, 0x00800000),
        P_M(0x600A7044, 0x000000ff, 1), P_M(0x600A7108, 0x00000200, 0)}},
    {"LLTF_MASK_00", SENSE_K_PATCH, 1, 0, {P_M(0x600A790C, 0x3000, 0)}},
    {"LLTF_MASK_10", SENSE_K_PATCH, 1, 0, {P_M(0x600A790C, 0x3000, 0x2000)}},
    {"LLTF_MASK_01", SENSE_K_PATCH, 1, 0, {P_M(0x600A790C, 0x3000, 0x1000)}},
    {"LLTF_MASK_11", SENSE_K_PATCH, 1, 0, {P_M(0x600A790C, 0x3000, 0x3000)}},
    {"RIFS_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A70F4, 1, 0)}},
    {"RIFS_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A70F4, 1, 1)}},
    {"VHT_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A7CE4, 0x20, 0)}},
    {"VHT_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A7CE4, 0x20, 0x20)}},
    {"RX11BLR_ARG0", SENSE_K_PATCH, 2, 0, {P_M(0x600A8060, 0xc00, 0), P_M(0x600A807C, 0x800, 0)}},
    {"RX11BLR_ARG1", SENSE_K_PATCH, 2, 0, {P_M(0x600A8060, 0xc00, 0xc00), P_M(0x600A807C, 0x800, 0x800)}},
    {"HEMU_RU26_VENDOR", SENSE_K_PATCH, 1, 0, {P_M(0x600A7890, 0x03000000, 0x01000000)}},
    {"PWDET_VENDOR_CLEAR", SENSE_K_PATCH, 1, 0, {P_M(0x600A0808, 0x0e, 0)}},
};
/* phy_set_txclk_en and phy_wifi_enable_set footprints. Never RX clock
 * gates, PLL, CPU/USB clocks or I2C master gates. */
static const sense_profile_t k_clock[] = {
    {"TXCLK_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A0890, 0x00030000, 0)}},
    {"TXCLK_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A0890, 0x00030000, 0x00030000)}},
    {"WIFI_ENABLE_ARG0", SENSE_K_PATCH, 1, 0, {P_M(0x600A9C18, 2, 0)}},
    {"WIFI_ENABLE_ARG1", SENSE_K_PATCH, 1, 0, {P_M(0x600A9C18, 2, 2)}},
};

/* ---- lab state ------------------------------------------------------------ */
typedef struct {
    uint16_t freq;
    int offset;
    uint8_t gain;              /* STOCK gain, fixed for the run */
    bool dco_check;
    int dco[2];
    uint8_t filt[8];           /* BBTOP 0x67/1 regs 6..13 at entry */
    int64_t deadline;
    uint8_t *raw;
    unsigned profiles, rejected, skipped, broken;
    bool stop;                 /* a context failure ends the run */
} lab_t;
static lab_t L;
static const sense_profile_t *s_active;
static uint32_t s_expect[SENSE_MAX_PATCHES];

static uint32_t patch_read(const sense_patch_t *p)
{
    if (p->kind == SENSE_P_MMIO) return MMIO(p->addr) & p->mask;
    return phy_i2c_readReg(p->block, p->host, p->reg) & (p->mask & 0xffu);
}
static void patch_write(const sense_patch_t *p, uint32_t value)
{
    if (p->kind == SENSE_P_MMIO) {
        MMIO(p->addr) = (MMIO(p->addr) & ~p->mask) | (value & p->mask);
        return;
    }
    const uint8_t m = (uint8_t)(p->mask & 0xffu);
    const uint8_t cur = phy_i2c_readReg(p->block, p->host, p->reg);
    phy_i2c_writeReg(p->block, p->host, p->reg, (uint8_t)((cur & ~m) | (value & m)));
}
static bool patches_ok(void)
{
    if (!s_active) return true;
    for (unsigned i = 0; i < s_active->n; ++i)
        if (patch_read(&s_active->p[i]) != s_expect[i]) return false;
    return true;
}
/* Filter bytes other than those the active profile owns stay at entry. */
static bool filter_ok(void)
{
    for (uint8_t r = 0; r < 8u; ++r) {
        bool owned = false;
        for (unsigned i = 0; s_active && i < s_active->n; ++i) {
            const sense_patch_t *p = &s_active->p[i];
            owned |= p->kind == SENSE_P_I2C && p->block == 0x67 && p->host == 1 && p->reg == 6 + r;
        }
        if (!owned && phy_i2c_readReg(0x67, 1, 6 + r) != L.filt[r]) return false;
    }
    return true;
}
static bool tuple_of(uint8_t gain, uint32_t *packed)
{
    arc_gain_tuple_t t;
    if (!arc_gain_tuple_decode(rf_gain_table(), gain, &t)) return false;
    *packed = t.packed_state;
    return true;
}
/* The actual RF/BB/fine state, the predicate of phy_dco_try_pair. */
static bool hw_gain_ok(uint32_t packed)
{
    const uint16_t rf = phy_pbus_rd(8, 1), bb = phy_pbus_rd(0, 2), fine = phy_pbus_rd(1, 2);
    return rf == (packed >> 12) && (bb & 127u) == ((packed >> 4) & 127u) &&
           ((bb & ~127u) == 0u || (bb & ~127u) == 0x100u) && fine == (0xd0u | (packed & 7u));
}
static const char *context_fail(uint8_t gain, bool dco)
{
    if (esp_timer_get_time() > L.deadline) return "deadline";
    if (s_menu_active) return "menu";
    if (rf_frequency_mhz() != L.freq || rf_offset_khz() != L.offset) return "retune";
    if (s_current_gain != gain) return "gain";
    uint32_t packed;
    if (!tuple_of(gain, &packed) || !hw_gain_ok(packed)) return "hardware_gain";
    if (dco && L.dco_check) {
        int pair[2];
        if (!phy_dco_held() || !phy_dco_read_pair(pair) || pair[0] != L.dco[0] || pair[1] != L.dco[1]) return "dc_pair";
    }
    if (!patches_ok()) return "profile_state";
    if (!filter_ok()) return "filter_state";
    return NULL;
}
/* Failures a TEST setting can cause itself; they end the profile, not the run. */
static bool candidate_failure(const char *why)
{
    return !strcmp(why, "stalled_iq") || !strcmp(why, "no_iq") || !strcmp(why, "capture");
}
static __attribute__((noreturn)) void fatal(const char *why)
{
    printf("SENSLAB_FATAL %s reboot_required\n", why);
    fflush(stdout);
    esp_restart();
}
/* A paused gain write releases the DC hold and the post-gain hook stays
 * silent: re-hold the row's calibrated pair explicitly, then settle. */
static void set_gain_held(uint8_t gain)
{
    receiver_set_gain(gain);
    (void)calibration_dc_hold_current();
    vTaskDelay(pdMS_TO_TICKS(100));
}

static bool capture(uint8_t *raw)
{
    for (unsigned i = 0; i < 40u; ++i) {
        vTaskDelay(1);
        if (copy_snapshot(raw)) return true;
    }
    return false;
}

typedef struct { sense_acc_t cn, click, origin, rail, video; unsigned lines; } stage_t;

static const char *measure_stage(unsigned id, unsigned step, uint8_t gain, bool dco, stage_t *st)
{
    const char *stage = step == 9 ? "BASE" : step == 0 || step == 3 ? "STOCK" : "TEST";
    vTaskDelay(pdMS_TO_TICKS(SENSE_DWELL_MS));
    uint32_t hashes[SENSE_CAPTURES];
    for (unsigned rep = 0; rep < SENSE_CAPTURES; ++rep) {
        const char *why = context_fail(gain, dco);
        if (!why && !capture(L.raw)) why = "capture";
        if (!why) why = context_fail(gain, dco);
        uint32_t hash = 2166136261u;
        sense_metrics_t m = {0};
        if (!why) {
            for (unsigned k = 0; k < C5V4_LEVEL_SAMPLE_BYTES; ++k) hash = (hash ^ L.raw[k]) * 16777619u;
            for (unsigned j = 0; j < rep && !why; ++j)
                if (hashes[j] == hash) why = "stalled_iq";
            hashes[rep] = hash;
        }
        if (!why) {
            sense_measure(L.raw, C5V4_LEVEL_SAMPLE_BYTES, c5vrx_phase8_gain_lut, &m);
            if (!isfinite(m.m2) || m.m2 < 0.05) why = "no_iq";
        }
        if (why) {
            printf("SENSLAB_ABORT id=%u step=%u stage=%s rep=%u reason=%s\n", id, step, stage, rep, why);
            return why;
        }
        printf("SENSLAB_CAP id=%u step=%u stage=%s rep=%u gain=%u cn_db=%.3f click_pm=%.3f origin_pm=%.3f rail_pm=%.3f "
               "video_lines=%u video_snr_db=%.2f m2=%.4f kurt=%.4f dc=%.3f/%.3f fnv1a=%08" PRIx32 " t_ms=%lld\n",
               id, step, stage, rep, gain, m.cn_db, m.click_pm, m.origin_pm, m.rail_pm, m.video_lines, m.video_snr_db,
               m.m2, m.kurt, (double)m.dc_i, (double)m.dc_q, hash, (long long)(esp_timer_get_time() / 1000));
        sense_acc_add(&st->cn, m.cn_db);
        sense_acc_add(&st->click, m.click_pm);
        sense_acc_add(&st->origin, m.origin_pm);
        sense_acc_add(&st->rail, m.rail_pm);
        if (!isnan(m.video_snr_db)) sense_acc_add(&st->video, m.video_snr_db);
        st->lines += m.video_lines;
    }
    return NULL;
}

/* Quick per-profile view; tools/phy_sense_lab.py makes the real verdict. */
static void result(unsigned id, const char *name, const char *status, const stage_t st[4])
{
    sense_acc_t stock_cn = st[0].cn, test_cn = st[1].cn, stock_click = st[0].click, test_click = st[1].click;
    sense_acc_t stock_video = st[0].video, test_video = st[1].video;
    stock_cn.n += st[3].cn.n; stock_cn.sum += st[3].cn.sum; stock_cn.sumsq += st[3].cn.sumsq;
    test_cn.n += st[2].cn.n; test_cn.sum += st[2].cn.sum; test_cn.sumsq += st[2].cn.sumsq;
    stock_click.n += st[3].click.n; stock_click.sum += st[3].click.sum; stock_click.sumsq += st[3].click.sumsq;
    test_click.n += st[2].click.n; test_click.sum += st[2].click.sum; test_click.sumsq += st[2].click.sumsq;
    stock_video.n += st[3].video.n; stock_video.sum += st[3].video.sum; stock_video.sumsq += st[3].video.sumsq;
    test_video.n += st[2].video.n; test_video.sum += st[2].video.sum; test_video.sumsq += st[2].video.sumsq;
    double se_cn = 0.0, se_click = 0.0, se_video = 0.0;
    const double d_cn = sense_delta(&test_cn, &stock_cn, &se_cn);
    const double d_click = sense_delta(&test_click, &stock_click, &se_click);
    const double d_video = sense_delta(&test_video, &stock_video, &se_video);
    printf("SENSLAB_RESULT id=%u name=%s status=%s d_cn_db=%+.3f se_cn_db=%.3f d_click_pm=%+.3f se_click_pm=%.3f "
           "d_video_snr_db=%+.2f se_video_db=%.2f video_caps=%u/%u drift_cn_db=%+.3f agree_cn_db=%+.3f stock_cn_db=%.2f\n",
           id, name, status, d_cn, se_cn, d_click, se_click, d_video, se_video, test_video.n, test_cn.n,
           sense_acc_mean(&st[3].cn) - sense_acc_mean(&st[0].cn), sense_acc_mean(&st[2].cn) - sense_acc_mean(&st[1].cn),
           sense_acc_mean(&stock_cn));
}

static void print_patches(unsigned id, const sense_profile_t *p, const uint32_t *saved)
{
    for (unsigned i = 0; i < p->n; ++i) {
        const sense_patch_t *q = &p->p[i];
        if (q->kind == SENSE_P_MMIO)
            printf("SENSLAB_PATCH id=%u i=%u mmio=%08" PRIx32 " mask=%08" PRIx32 " stock=%08" PRIx32 " test=%08" PRIx32 "\n",
                   id, i, q->addr, q->mask, saved[i], q->value & q->mask);
        else
            printf("SENSLAB_PATCH id=%u i=%u i2c=%02x/%u/%u mask=%02" PRIx32 " stock=%02" PRIx32 " test=%02" PRIx32 "\n", id, i,
                   q->block, q->host, q->reg, q->mask & 0xffu, saved[i], q->value & q->mask & 0xffu);
    }
}

/* One masked-write profile (n == 0: sham). */
static void run_patch(unsigned id, const sense_profile_t *p)
{
    uint32_t saved[SENSE_MAX_PATCHES];
    bool noop = p->n > 0;
    for (unsigned i = 0; i < p->n; ++i) {
        saved[i] = patch_read(&p->p[i]);
        noop &= saved[i] == (p->p[i].value & p->p[i].mask & (p->p[i].kind == SENSE_P_I2C ? 0xffu : 0xffffffffu));
    }
    s_active = p;
    memcpy(s_expect, saved, sizeof(saved[0]) * p->n);
    printf("SENSLAB_PROFILE id=%u name=%s kind=%s patches=%u noop=%u sequence=STOCK_TEST_TEST_STOCK\n", id, p->name,
           p->n ? "patch" : "sham", p->n, noop);
    print_patches(id, p, saved);
    stage_t st[4];
    memset(st, 0, sizeof(st));
    const char *why = measure_stage(id, 0, L.gain, true, &st[0]);
    const char *status = "PASS";
    if (!why) {
        phy_observation_changed();
        for (unsigned i = 0; i < p->n; ++i) patch_write(&p->p[i], p->p[i].value);
        for (unsigned i = 0; i < p->n; ++i)
            s_expect[i] = p->p[i].value & p->p[i].mask & (p->p[i].kind == SENSE_P_I2C ? 0xffu : 0xffffffffu);
        const bool entered = patches_ok();
        printf("SENSLAB_APPLY id=%u verified=%u\n", id, entered);
        if (!entered) { status = "REJECTED_ENTRY"; ++L.rejected; }
        else {
            why = measure_stage(id, 1, L.gain, true, &st[1]);
            if (!why) why = measure_stage(id, 2, L.gain, true, &st[2]);
            if (why && candidate_failure(why)) { status = "BREAKS_IQ"; ++L.broken; why = NULL; }
        }
    }
    phy_observation_changed();
    for (unsigned i = p->n; i-- > 0;) patch_write(&p->p[i], saved[i]);
    memcpy(s_expect, saved, sizeof(saved[0]) * p->n);
    const bool restored = patches_ok() && filter_ok();
    printf("SENSLAB_RESTORE id=%u verified=%u\n", id, restored);
    if (!restored) fatal("restore_failed");
    /* STOCK again: the second control, and after a rejection or a broken
     * capture the witness that the restored state receives normally. */
    if (!why) {
        const char *w = measure_stage(id, 3, L.gain, true, &st[3]);
        if (w) { L.stop = true; if (!strcmp(status, "PASS")) status = "INCOMPLETE"; }
    } else { status = "INCOMPLETE"; L.stop = true; }   /* STOCK itself failed, or the context changed */
    s_active = NULL;
    result(id, p->name, status, st);
    ++L.profiles;
}

/* TEST at another gain row; STOCK back at the run's gain and DC pair. */
static void run_gain(unsigned id, uint8_t row)
{
    char name[24];
    snprintf(name, sizeof(name), "GAIN_ROW_%u", row);
    uint32_t packed = 0;
    (void)tuple_of(row, &packed);
    printf("SENSLAB_PROFILE id=%u name=%s kind=gain stock_gain=%u test_gain=%u test_packed=%08" PRIx32
           " sequence=STOCK_TEST_TEST_STOCK\n", id, name, L.gain, row, packed);
    stage_t st[4];
    memset(st, 0, sizeof(st));
    const char *why = measure_stage(id, 0, L.gain, true, &st[0]);
    if (!why) {
        set_gain_held(row);
        printf("SENSLAB_APPLY id=%u verified=%u dc_held=%u\n", id, s_current_gain == row, phy_dco_held());
        why = measure_stage(id, 1, row, false, &st[1]);
        if (!why) why = measure_stage(id, 2, row, false, &st[2]);
    }
    set_gain_held(L.gain);
    const char *back = context_fail(L.gain, true);
    printf("SENSLAB_RESTORE id=%u verified=%u%s%s\n", id, !back, back ? " reason=" : "", back ? back : "");
    if (back) { why = back; L.stop = true; }
    if (!why) why = measure_stage(id, 3, L.gain, true, &st[3]);
    if (why) L.stop = true;
    result(id, name, why ? "INCOMPLETE" : "PASS", st);
    ++L.profiles;
}

static void run_output_off(unsigned id)
{
    printf("SENSLAB_PROFILE id=%u name=DAC_OUTPUT_OFF kind=output sequence=ON_OFF_OFF_ON\n", id);
    stage_t st[4];
    memset(st, 0, sizeof(st));
    const char *why = !transport_live() || transport_faulted() ? "output_not_live" : measure_stage(id, 0, L.gain, true, &st[0]);
    if (!why) {
        transport_stop_tx();
        printf("SENSLAB_APPLY id=%u verified=%u\n", id, !transport_live());
        why = measure_stage(id, 1, L.gain, true, &st[1]);
        if (!why) why = measure_stage(id, 2, L.gain, true, &st[2]);
    }
    if (!transport_live()) transport_resume_live();
    const bool restored = transport_live() && !transport_faulted();
    printf("SENSLAB_RESTORE id=%u verified=%u\n", id, restored);
    if (!restored) fatal("output_restore_failed");
    if (!why) why = measure_stage(id, 3, L.gain, true, &st[3]);
    if (why) L.stop = true;
    result(id, "DAC_OUTPUT_OFF", why ? "INCOMPLETE" : "PASS", st);
    ++L.profiles;
}

static void skip(unsigned id, const char *name, const char *reason)
{
    printf("SENSLAB_SKIP id=%u name=%s reason=%s\n", id, name, reason);
    ++L.skipped;
}

/* A list with shams at the start, every SENSE_SHAM_EVERY candidates and at the end. */
static void run_list(unsigned base, const sense_profile_t *list, unsigned n)
{
    unsigned id = base;
    run_patch(id++, &k_sham);
    for (unsigned i = 0; i < n && !L.stop; ++i) {
        run_patch(id++, &list[i]);
        if ((i + 1) % SENSE_SHAM_EVERY == 0 && i + 1 < n && !L.stop) run_patch(id++, &k_sham);
    }
    if (!L.stop) run_patch(id, &k_sham);
}

static sense_profile_t s_dyn[16];

static void suite_nf_auto(void)
{
    /* phy_noise_floor_auto_set fields: 16 raw patterns, not documented modes.
     * Patterns asking 0x600A7C50[0]=1 did not latch in PR #254 (rejected). */
    static const char *names[16] = {"NF_AUTO_0", "NF_AUTO_1", "NF_AUTO_2", "NF_AUTO_3", "NF_AUTO_4", "NF_AUTO_5",
                                    "NF_AUTO_6", "NF_AUTO_7", "NF_AUTO_8", "NF_AUTO_9", "NF_AUTO_A", "NF_AUTO_B",
                                    "NF_AUTO_C", "NF_AUTO_D", "NF_AUTO_E", "NF_AUTO_F"};
    for (unsigned k = 0; k < 16u; ++k) {
        sense_profile_t *p = &s_dyn[k];
        memset(p, 0, sizeof(*p));
        p->name = names[k];
        p->kind = SENSE_K_PATCH;
        p->n = 3;
        p->p[0] = (sense_patch_t)P_M(0x600A7018, 0x10800000, ((k & 1u) ? 0x800000u : 0u) | ((k & 2u) ? 0x10000000u : 0u));
        p->p[1] = (sense_patch_t)P_M(0x600A7C44, 1, (k >> 2) & 1u);
        p->p[2] = (sense_patch_t)P_M(0x600A7C50, 1, (k >> 3) & 1u);
    }
    run_list(100, s_dyn, 16);
}

/* ADC selector (exact phy_adc_rate_set footprint), adjacent LO-cap codes,
 * and the chip's own ULP reference code; spur slot 0 when it is active. */
static void suite_analog(void)
{
    unsigned n = 0;
    const uint32_t sel = MMIO(0x600A0448) & 3u;
    if (sel == 0u || sel == 3u) {
        const bool to1 = sel == 0u;
        sense_profile_t *p = &s_dyn[n++];
        memset(p, 0, sizeof(*p));
        p->name = to1 ? "ADC_SELECTOR1" : "ADC_SELECTOR0";
        p->kind = SENSE_K_PATCH;
        p->n = 2;
        p->p[0] = (sense_patch_t)P_M(0x600A0448, 3, to1 ? 3u : 0u);
        p->p[1] = (sense_patch_t)P_I(0x66, 0, 4, 0x04, to1 ? 0u : 4u);
    } else skip(600, "ADC_SELECTOR", "stock_selector_not_0_or_3");
    const unsigned cap = phy_i2c_readReg(0x63, 1, 21) & 31u;
    static const char *capn[2] = {"LO_CAP_MINUS1", "LO_CAP_PLUS1"};
    for (unsigned s = 0; s < 2u; ++s) {
        const int target = (int)cap + (s ? 1 : -1);
        if (cap < 10u || target < 10 || target > 31) { skip(601 + s, capn[s], "outside_vendor_10_31"); continue; }
        sense_profile_t *p = &s_dyn[n++];
        memset(p, 0, sizeof(*p));
        p->name = capn[s];
        p->kind = SENSE_K_PATCH;
        p->n = 1;
        p->p[0] = (sense_patch_t)P_I(0x63, 1, 21, 0x1f, (uint32_t)target);
    }
    const uint8_t ref = phy_i2c_readReg(0x61, 0, 4), en = phy_i2c_readReg(0x61, 0, 5), act = phy_i2c_readReg(0x61, 0, 6);
    if ((en & 64u) && ref != act && abs((int)ref - (int)act) <= 1) {
        sense_profile_t *p = &s_dyn[n++];
        memset(p, 0, sizeof(*p));
        p->name = "ULP_REFERENCE_CODE";
        p->kind = SENSE_K_PATCH;
        p->n = 1;
        p->p[0] = (sense_patch_t)P_I(0x61, 0, 6, 0xff, ref);
    } else skip(603, "ULP_REFERENCE_CODE", "disabled_equal_or_more_than_one_code");
    if (MMIO(0x600A7C14) & 0x2000u) {
        sense_profile_t *p = &s_dyn[n++];
        memset(p, 0, sizeof(*p));
        p->name = "SPUR_SLOT0_OFF";
        p->kind = SENSE_K_PATCH;
        p->n = 1;
        p->p[0] = (sense_patch_t)P_M(0x600A7C14, 0x2000, 0);
    } else skip(604, "SPUR_SLOT0_OFF", "slot0_inactive");
    run_list(610, s_dyn, n);
}

/* The analog RX0 capacitor code (BBTOP 0x67/1 regs 6/7, low six bits) one
 * and two steps either side of the live code; regs 8..13 untouched. With
 * a carrier present this weighs noise bandwidth against deviation clipping. */
static void suite_filter(void)
{
    static const int deltas[4] = {-2, -1, 1, 2};
    static const char *names[4] = {"FILTER_CODE_M2", "FILTER_CODE_M1", "FILTER_CODE_P1", "FILTER_CODE_P2"};
    const int c6 = phy_i2c_readReg(0x67, 1, 6) & 63, c7 = phy_i2c_readReg(0x67, 1, 7) & 63;
    printf("SENSLAB_FILTER stock_reg6_code=%d stock_reg7_code=%d shadow_code=%d skirt=%d\n", c6, c7, phy_filter_code(),
           phy_filter_skirt());
    unsigned n = 0;
    for (unsigned k = 0; k < 4u; ++k) {
        const int a = c6 + deltas[k], b = c7 + deltas[k];
        if (a < 0 || a > 63 || b < 0 || b > 63) { skip(700 + k, names[k], "code_out_of_range"); continue; }
        sense_profile_t *p = &s_dyn[n++];
        memset(p, 0, sizeof(*p));
        p->name = names[k];
        p->kind = SENSE_K_PATCH;
        p->n = 2;
        p->p[0] = (sense_patch_t)P_I(0x67, 1, 6, 0x3f, (uint32_t)a);
        p->p[1] = (sense_patch_t)P_I(0x67, 1, 7, 0x3f, (uint32_t)b);
    }
    run_list(710, s_dyn, n);
}

static void suite_gain(void)
{
    static const int deltas[4] = {-6, -3, 3, 6};
    unsigned id = 800;
    run_patch(id++, &k_sham);
    for (unsigned k = 0; k < 4u && !L.stop; ++k) {
        const int row = (int)L.gain + deltas[k];
        if (row < 20 || row > (int)rf_gain_table()->max_index || !calibration_dc_ok((uint8_t)row)) {
            printf("SENSLAB_SKIP id=%u name=GAIN_ROW_%d reason=outside_table_or_no_dc_correction\n", id++, row);
            ++L.skipped;
            continue;
        }
        run_gain(id++, (uint8_t)row);
    }
    if (!L.stop) run_patch(id, &k_sham);
}

static void run_suite(char suite)
{
    printf("SENSLAB_SUITE key=%c\n", suite);
    switch (suite) {
    case '0':
        for (unsigned k = 0; k < 8u && !L.stop; ++k) run_patch(k, &k_sham);
        break;
    case '1': suite_nf_auto(); break;
    case '2': run_list(200, k_iq, sizeof(k_iq) / sizeof(k_iq[0])); break;
    case '3': run_list(300, k_dfilt, sizeof(k_dfilt) / sizeof(k_dfilt[0])); break;
    case '4': run_list(400, k_policy, sizeof(k_policy) / sizeof(k_policy[0])); break;
    case '5': run_list(500, k_clock, sizeof(k_clock) / sizeof(k_clock[0])); break;
    case '6': suite_analog(); break;
    case '7': suite_filter(); break;
    case '8': suite_gain(); break;
    case '9':
        run_patch(900, &k_sham);
        if (!L.stop) run_output_off(901);
        if (!L.stop) run_output_off(902);
        if (!L.stop) run_patch(903, &k_sham);
        break;
    default: break;
    }
}

/* Highest clean gain at or below the entry gain: no signed-rail pile-up
 * (the sign-preserving lane folds there) with a calibrated DC row. */
static bool screen_gain(uint8_t entry)
{
    for (unsigned step = 0; step < 6u; ++step) {
        const int g = (int)entry - 2 * (int)step;
        if (g < 20) break;
        if (s_current_gain != (uint8_t)g) set_gain_held((uint8_t)g);
        if (!calibration_dc_ok((uint8_t)g)) { printf("SENSLAB_GAIN gain=%d skip=no_dc_correction\n", g); continue; }
        double rail = 0.0, m2 = 0.0;
        unsigned got = 0;
        for (unsigned rep = 0; rep < 4u; ++rep) {
            if (!capture(L.raw)) continue;
            sense_metrics_t m;
            sense_measure(L.raw, C5V4_LEVEL_SAMPLE_BYTES, c5vrx_phase8_gain_lut, &m);
            rail += m.rail_pm; m2 += m.m2; ++got;
        }
        if (got < 4u) { printf("SENSLAB_GAIN gain=%d skip=capture\n", g); continue; }
        rail /= got; m2 /= got;
        const bool clean = rail <= 2.0 && m2 > 0.67;
        printf("SENSLAB_GAIN gain=%d rail_pm=%.3f m2=%.4f clean=%u\n", g, rail, m2, clean);
        if (clean) return true;
    }
    return false;
}

void sense_lab_run(bool quiet)
{
    const char suite = s_request;
    s_request = 0;
    if (!suite) return;
    const int64_t now = esp_timer_get_time();
    if (!quiet) { printf("SENSLAB refused=busy\n"); return; }
    if (calibration_in_startup()) { printf("SENSLAB refused=startup_calibration\n"); return; }
    if (!s_last_video_us || now - s_last_video_us > INT64_C(3000000)) {
        printf("SENSLAB refused=no_vtx_video (the VTX must be ON and the picture locked; weak but steady)\n");
        return;
    }
    memset(&L, 0, sizeof(L));
    L.raw = malloc(C5V4_LEVEL_SAMPLE_BYTES);
    if (!L.raw) { printf("SENSLAB refused=memory\n"); return; }
    if (!receiver_pause("SENSLAB")) { free(L.raw); return; }
    const uint8_t entry_gain = s_current_gain;
    const int64_t video_age_ms = (now - s_last_video_us) / 1000;
    /* The whole run is one PHY transaction: no AFC, gain, DC servo, filter
     * or tuning write can land between STOCK and TEST. */
    phy_begin();
    bool ok = screen_gain(entry_gain);
    uint32_t packed = 0;
    ok = ok && tuple_of(s_current_gain, &packed) && hw_gain_ok(packed);
    if (!ok) printf("SENSLAB refused=gain_screen (no clean gain at or below G%u)\n", entry_gain);
    if (ok) {
        L.gain = s_current_gain;
        L.freq = rf_frequency_mhz();
        L.offset = rf_offset_khz();
        L.dco_check = phy_dco_held() && phy_dco_read_pair(L.dco);
        for (uint8_t r = 0; r < 8u; ++r) L.filt[r] = phy_i2c_readReg(0x67, 1, 6 + r);
        L.deadline = esp_timer_get_time() + (suite == 'a' ? SENSE_ALL_US : SENSE_SUITE_US);
        /* Baseline: how strong is the carrier? The M2M4 score saturates near
         * 10-11 dB with 4-bit IQ, so the comparison needs a weak carrier. */
        stage_t base;
        memset(&base, 0, sizeof(base));
        const char *why = measure_stage(0, 9, L.gain, true, &base);
        if (why) { printf("SENSLAB refused=baseline_%s\n", why); ok = false; }
        if (ok) {
            const double cn = sense_acc_mean(&base.cn), video = sense_acc_mean(&base.video);
            printf("SENSLAB_BEGIN fw=%s suite=%c freq=%u offset_khz=%d gain=%u entry_gain=%u packed=%08" PRIx32
                   " dco=%d/%d held=%u filter_code=%d skirt=%d dfilt=%d p11=%d captures_per_stage=%u dwell_ms=%u "
                   "baseline_cn_db=%.2f baseline_click_pm=%.3f baseline_video_snr_db=%.2f video_caps=%u/%u "
                   "video_age_ms=%lld lane=ultrafine persistent=0\n",
                   firmware_id(), suite, L.freq, L.offset, L.gain, entry_gain, packed, L.dco[0], L.dco[1], L.dco_check,
                   phy_filter_code(), phy_filter_skirt(), phy_dfilt_hw_mode(), phy_11p_hw_enabled(), SENSE_CAPTURES,
                   SENSE_DWELL_MS, cn, sense_acc_mean(&base.click), video, base.video.n, base.cn.n, (long long)video_age_ms);
            if (cn > 9.0)
                printf("SENSLAB_WARN signal_strong baseline_cn_db=%.2f: M2M4 saturates near 10-11 dB with 4-bit IQ; "
                       "weaken the signal until the picture shows light snow (about 2-8 dB)\n", cn);
            if (cn < 0.5) printf("SENSLAB_WARN signal_very_weak baseline_cn_db=%.2f: move closer for usable statistics\n", cn);
            if (!base.video.n) printf("SENSLAB_WARN no_sync_in_baseline: the video SNR metric will be empty\n");
            if (suite == 'a') {
                static const char order[] = "123456789";
                for (unsigned k = 0; order[k] && !L.stop; ++k) run_suite(order[k]);
            } else run_suite(suite);
        }
    }
    s_active = NULL;
    if (s_current_gain != entry_gain) set_gain_held(entry_gain);
    phy_end();
    receiver_resume();
    free(L.raw);
    L.raw = NULL;
    printf("SENSLAB_END status=%s profiles=%u rejected=%u breaks_iq=%u skipped=%u restored=1 "
           "input_referred_claim=none (score with tools/phy_sense_lab.py)\n",
           !ok ? "REFUSED" : L.stop ? "INCOMPLETE" : "COMPLETE", L.profiles, L.rejected, L.broken, L.skipped);
}
#else
void sense_lab_run(bool quiet)
{
    (void)quiet;
    if (!s_request) return;
    s_request = 0;
    printf("SENSLAB refused=unverified_phy\n");
}
#endif
