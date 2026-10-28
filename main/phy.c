/* C5VRX-5 by Twotoz and the C5VRX contributors: owned PHY access, from
 * legacy/c5vrx4/main/phy_rx_lab.c (pinned C5 PHY investigation #150-153, #165). */
#include "phy.h"
#include "dsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include <stdio.h>
#include <string.h>

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
/* Recursive task ownership for nested channel/filter restores; the short
 * spinlock only protects the counters, never vendor I2C. */
static StaticSemaphore_t s_lock_storage;
static SemaphoreHandle_t s_lock;
static uint32_t s_generation, s_depth;
static void (*s_observation_hook)(void);

void phy_set_observation_hook(void (*hook)(void)) { s_observation_hook=hook; }
void phy_observation_changed(void)
{ if(s_observation_hook)s_observation_hook(); }

static void lock_take(void)
{
    portENTER_CRITICAL(&s_mux);
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_storage);
    portEXIT_CRITICAL(&s_mux);
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}
static void lock_give(void) { xSemaphoreGiveRecursive(s_lock); }

void phy_read_begin(void) { lock_take(); }
void phy_read_end(void) { lock_give(); }

void phy_begin(void)
{
    lock_take();
    portENTER_CRITICAL(&s_mux);
    ++s_depth;
    bool outer=s_depth==1u;
    portEXIT_CRITICAL(&s_mux);
    if(outer)phy_observation_changed();
}

void phy_end(void)
{
    portENTER_CRITICAL(&s_mux);
    if (s_depth && --s_depth == 0) ++s_generation;
    portEXIT_CRITICAL(&s_mux);
    lock_give();
}

uint32_t phy_generation(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t g = s_generation;
    portEXIT_CRITICAL(&s_mux);
    return g;
}

bool phy_busy(void)
{
    portENTER_CRITICAL(&s_mux);
    bool busy = s_depth != 0;
    portEXIT_CRITICAL(&s_mux);
    return busy;
}

bool phy_try_actuator(uint32_t expected_generation)
{
    portENTER_CRITICAL(&s_mux);
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_storage);
    portEXIT_CRITICAL(&s_mux);
    if (xSemaphoreTakeRecursive(s_lock, 0) != pdTRUE) return false;
    portENTER_CRITICAL(&s_mux);
    bool valid = !s_depth && s_generation == expected_generation;
    portEXIT_CRITICAL(&s_mux);
    if (!valid) lock_give();
    return valid;
}

/* The same, waiting up to `wait_ticks` for the lock, so a gain write does
 * not bounce off another short owner (board, with the since removed sync
 * flywheel: 16 % of gain writes refused during a sweep without the wait).
 * The generation check after taking it is unchanged. */
bool phy_try_actuator_wait(uint32_t expected_generation, uint32_t wait_ticks)
{
    portENTER_CRITICAL(&s_mux);
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_storage);
    portEXIT_CRITICAL(&s_mux);
    if (xSemaphoreTakeRecursive(s_lock, wait_ticks) != pdTRUE) return false;
    portENTER_CRITICAL(&s_mux);
    bool valid = !s_depth && s_generation == expected_generation;
    portEXIT_CRITICAL(&s_mux);
    if (!valid) lock_give();
    return valid;
}

void phy_end_actuator(void) { lock_give(); }

#ifdef C5VRX_PHY_PINNED
extern uint8_t phy_i2c_readReg(uint8_t block, uint8_t host, uint8_t reg);
extern void phy_i2c_writeReg(uint8_t block, uint8_t host, uint8_t reg, uint8_t value);
static uint8_t s_filter_base[8];
static bool s_filter_base_valid;
#endif
static int s_filter_code = PHY_FILTER_CALIBRATED, s_filter_skirt;

bool phy_filter_capture_base(void)
{
#ifdef C5VRX_PHY_PINNED
    lock_take();
    if (!s_filter_base_valid) {
        for (uint8_t i = 0; i < 8; ++i) s_filter_base[i] = phy_i2c_readReg(0x67, 1, 6 + i);
        s_filter_base_valid = true;
    }
    lock_give();
    return true;
#else
    return false;
#endif
}

/* Default bytes are the 11p baseline (60). Explicit code/skirt profiles
 * retain the factory-byte reference so the sweep can still try wider
 * filters. Verify every write; on mismatch return to the 11p baseline. */
bool phy_filter_set_code(int code)
{
#ifdef C5VRX_PHY_PINNED
    if (code != PHY_FILTER_CALIBRATED && (code < 0 || code > 63)) return false;
    lock_take();
    bool ok = s_filter_base_valid;
    if(ok)phy_observation_changed();
    for (uint8_t i = 0; ok && i < 2; ++i) {
        uint8_t v = code == PHY_FILTER_CALIBRATED ? 60u :
                    (uint8_t)((s_filter_base[i] & ~63u) | (unsigned)code);
        phy_i2c_writeReg(0x67, 1, 6 + i, v);
        ok = phy_i2c_readReg(0x67, 1, 6 + i) == v;
    }
    if (ok) s_filter_code = code;
    else if (s_filter_base_valid) {
        for (uint8_t i = 0; i < 2; ++i) phy_i2c_writeReg(0x67, 1, 6 + i, 60u);
        s_filter_code = PHY_FILTER_CALIBRATED;
    }
    lock_give();
    return ok;
#else
    (void)code;
    return false;
#endif
}

bool phy_filter_set_skirt(int offset)
{
#ifdef C5VRX_PHY_PINNED
    if (offset < 0 || offset > 60) return false;
    lock_take();
    bool ok = s_filter_base_valid;
    if(ok)phy_observation_changed();
    for (uint8_t i = 2; ok && i < 8; ++i) {
        uint8_t v = offset ? dsp_filter_code(s_filter_base[i], offset) : 60u;
        phy_i2c_writeReg(0x67, 1, 6 + i, v);
        ok = phy_i2c_readReg(0x67, 1, 6 + i) == v;
    }
    if (ok) s_filter_skirt = offset;
    else if (s_filter_base_valid) {
        for (uint8_t i = 2; i < 8; ++i) phy_i2c_writeReg(0x67, 1, 6 + i, 60u);
        s_filter_skirt = 0;
    }
    lock_give();
    return ok;
#else
    (void)offset;
    return false;
#endif
}

int phy_filter_code(void) { return s_filter_code; }
int phy_filter_skirt(void) { return s_filter_skirt; }

int phy_filter_hw_mismatch(uint8_t hw[8], uint8_t want[8])
{
#ifdef C5VRX_PHY_PINNED
    if (!s_filter_base_valid) return -1;
    int bad = 0;
    lock_take();
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t w = i < 2 ? (s_filter_code == PHY_FILTER_CALIBRATED ? 60u :
                             (uint8_t)((s_filter_base[i] & ~63u) | (unsigned)s_filter_code)) :
                    (s_filter_skirt ? dsp_filter_code(s_filter_base[i], s_filter_skirt) : 60u);
        uint8_t h = phy_i2c_readReg(0x67, 1, 6 + i);
        if (hw) hw[i] = h;
        if (want) want[i] = w;
        bad += h != w;
    }
    lock_give();
    return bad;
#else
    (void)hw; (void)want;
    return -1;
#endif
}

int phy_filter_calibrated_code(void)
{
#ifdef C5VRX_PHY_PINNED
    return s_filter_base_valid ? (int)(s_filter_base[0] & 63u) : -1;
#else
    return -1;
#endif
}

/* ---- Digital RX filter --------------------------------------------------
 * C5VRX/Twotoz bench #244: mode 0 reduced captured noise power at 5865 MHz
 * versus vendor mode 8; with the VTX on, mode 12 gave the better C/N
 * (phy.h). Reapply after vendor setup (which changes the mode above
 * 5830 MHz). Never change the coupled ADC-rate selector here.
 * Read all four bits: modes with low bits set stalled IQ on the bench. */
#ifdef C5VRX_PHY_PINNED
extern void phy_rx_filter_mode(uint32_t mode);
#define DFILT_REG   0x600A0430u
#define DFILT_SHIFT 18u
#define DFILT_MASK  (UINT32_C(0xF) << DFILT_SHIFT)
#endif

int phy_dfilt_hw_mode(void)
{
#ifdef C5VRX_PHY_PINNED
    lock_take();
    const int mode = (int)((REG_READ(DFILT_REG) & DFILT_MASK) >> DFILT_SHIFT);
    lock_give();
    return mode;
#else
    return -1;
#endif
}

static int s_dfilt_mode = PHY_DFILT_MODE_DEFAULT;
int phy_dfilt_mode(void) { return s_dfilt_mode; }
int phy_dfilt_next(int mode) { return mode == 12 ? 0 : mode == 0 ? 4 : mode == 4 ? 8 : 12; }

bool phy_dfilt_apply_default(void)
{
#ifdef C5VRX_PHY_PINNED
    /* Nested in restore_lock's transaction; standalone calls also invalidate
     * observations before the write and publish a fresh PHY generation. */
    phy_begin();
    phy_rx_filter_mode((uint32_t)s_dfilt_mode);
    const bool ok = phy_dfilt_hw_mode() == s_dfilt_mode;
    phy_end();
    return ok;
#else
    return false;
#endif
}

bool phy_dfilt_select(int mode)
{
    if (mode != 0 && mode != 4 && mode != 8 && mode != 12) return false;
    const int was = s_dfilt_mode;
    s_dfilt_mode = mode;
    if (phy_dfilt_apply_default()) return true;
    s_dfilt_mode = was;
    (void)phy_dfilt_apply_default();
    return false;
}

/* ---- 11p receive baseline -----------------------------------------------
 * PR #254, extending the pinned legacy PHY lab (#150-155).
 * Original 11p observation: SushiDude (@Ready4Sushi). The helper changes
 * flags, eight analog bytes and four MMIO fields; (0,0) is NOT rollback.
 * Calibrated analog profiles may override the bytes after this baseline. */
#ifdef C5VRX_PHY_PINNED
extern unsigned char phy_param[];
extern void phy_11p_set(uint8_t enable, uint8_t mode);
static const struct { uint32_t addr, mask, value; } s_11p_fields[] = {
    {0x600A7CE4u, 0x1cu, 0x10u},
    {0x600A7030u, 0x20u, 0},
    {0x600A7048u, 0x7f00u, 0x4000u},
    {0x600A71C4u, 0xfe0000u, 0x440000u},
};
#endif

/* Only the persistent flags/MMIO: analog profile overrides are verified
 * separately by phy_filter_hw_mismatch(). -1 means unsupported. */
int phy_11p_hw_enabled(void)
{
#ifdef C5VRX_PHY_PINNED
    lock_take();
    bool ok = phy_param[0x26] == 1 && phy_param[0x27] == 0;
    for (unsigned i = 0; i < 4; ++i)
        ok &= (REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask) == s_11p_fields[i].value;
    lock_give();
    return ok;
#else
    return -1;
#endif
}

esp_err_t phy_11p_apply_default(void)
{
#ifndef C5VRX_PHY_PINNED
    return ESP_ERR_NOT_SUPPORTED;
#else
    phy_begin();
    uint32_t saved[4];
    uint8_t analog[8], flags[2] = {phy_param[0x26], phy_param[0x27]};
    for (unsigned i = 0; i < 4; ++i)
        saved[i] = REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask;
    for (uint8_t i = 0; i < 8; ++i) analog[i] = phy_i2c_readReg(0x67, 1, 6 + i);
    phy_11p_set(1, 0);
    bool ok = phy_11p_hw_enabled() == 1;
    for (uint8_t i = 0; i < 8; ++i) ok &= phy_i2c_readReg(0x67, 1, 6 + i) == 60;
    esp_err_t result = ESP_OK;
    if (!ok) {
        phy_param[0x26] = flags[0]; phy_param[0x27] = flags[1];
        for (uint8_t i = 0; i < 8; ++i) phy_i2c_writeReg(0x67, 1, 6 + i, analog[i]);
        for (unsigned i = 0; i < 4; ++i) {
            const uint32_t addr = s_11p_fields[i].addr, mask = s_11p_fields[i].mask;
            REG_WRITE(addr, (REG_READ(addr) & ~mask) | saved[i]);
        }
        bool restored = phy_param[0x26] == flags[0] && phy_param[0x27] == flags[1];
        for (uint8_t i = 0; i < 8; ++i)
            restored &= phy_i2c_readReg(0x67, 1, 6 + i) == analog[i];
        for (unsigned i = 0; i < 4; ++i)
            restored &= (REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask) == saved[i];
        result = restored ? ESP_ERR_INVALID_RESPONSE : ESP_FAIL;
    }
    phy_end();
    return result;
#endif
}

/* ---- RX DC DACs ----------------------------------------------------------
 * Recovered from the pinned libphy: phy_pbus_set_dco() writes PBUS
 * (block,bank) (2,1),(3,1),(2,2),(3,2); phy_pbus_force_test() owns them in
 * debug mode. Bank 2 is the pair ESPARGOS esp-sdr corrects; the response
 * matrix is measured, not assumed. */
#ifdef C5VRX_PHY_PINNED
extern uint16_t phy_pbus_rd(uint32_t block, uint32_t bank);
extern void phy_pbus_force_test(uint32_t block, uint32_t bank, uint32_t value);
extern void phy_pbus_debugmode(void);
extern void phy_pbus_workmode(void);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
#define DCO_PROBE 16
#define DCO_RANGE 96
#define DCO_TRIAL 32
#define DCO_DONE_MCELLS 100
#define PBUS_BLOCKS 11u
/* Only the gain/DC words the hold owns: BB (0), fine (1), DC (2, 3) and the
 * 5 GHz RF code (8). Copying all eleven blocks' read-back coincided with
 * dead/railing IQ on the board (2026-10-07). */
static inline bool pbus_owned(unsigned b) { return b <= 3u || b == 8u; }
static int s_dco_codes[2];
static bool s_dco_held;

static int dco_cost(const int dc[2]) { return dc[0] * dc[0] + dc[1] * dc[1]; }
static int dco_clamp(int v, int base)
{
    if (v < base - DCO_RANGE) v = base - DCO_RANGE;
    if (v > base + DCO_RANGE) v = base + DCO_RANGE;
    return v < 0 ? 0 : v > 511 ? 511 : v;
}
static void dco_apply(int i, int q)
{
    phy_pbus_force_test(2, 2, (uint32_t)i);
    phy_pbus_force_test(3, 2, (uint32_t)q);
}

/* phy_pbus_workmode() forces gain 50 for ~2 us and then clears the force
 * bit (disassembly, 2026-10-07): force the gain that was active again. */
static void pbus_workmode_keep_gain(void)
{
    uint32_t gain = *(volatile uint32_t *)0x600A702Cu;
    phy_pbus_workmode();
    if (gain & (1u << 23)) phy_force_rx_gain(true, (uint8_t)(gain >> 24));
}

static void read_live(uint16_t live[PBUS_BLOCKS][2])
{
    for (unsigned b = 0; b < PBUS_BLOCKS; ++b) if (pbus_owned(b))
        for (unsigned k = 0; k < 2; ++k) live[b][k] = phy_pbus_rd(b, k + 1);
}

/* Bank 1 of the DC words (blocks 2/3, bank 1) is the vendor's coarse DC
 * for the RF stage; the calibrated bank-2 codes of the DC table are only
 * valid on the bank 1 they were measured with. Debug mode used to copy the
 * live bank 1 at every hold (every gain write releases and re-holds), so a
 * vendor rewrite of it moved every row at once. Board 2026-10-27, after a
 * walk: bank 1 245/224 against 279/227 at boot (G54-G83), I off the ADC at
 * G83, half on the rail at G72, the servo trim at its +96 limit and the
 * guard releasing G83 over and over, until a reboot. Bank 1 is now pinned
 * per DC table: the first hold in its gain span takes it, every later one
 * forces the same value (phy_dco_bank1_reset on a new table). */
static int s_bank1_pin[2] = {-1, -1};
static uint8_t s_bank1_lo = 1u, s_bank1_hi = 0u;
static int s_bank1_moved[2] = {-1, -1};
static uint32_t s_bank1_overrides;

static void pin_bank1(uint16_t live[PBUS_BLOCKS][2])
{
    const uint32_t forced = *(volatile uint32_t *)0x600A702Cu;
    if (!(forced & (1u << 23))) return;
    const unsigned gain = (forced >> 24) & 0xffu;
    if (gain < s_bank1_lo || gain > s_bank1_hi) return;
    if (s_bank1_pin[0] < 0) { s_bank1_pin[0] = live[2][0]; s_bank1_pin[1] = live[3][0]; return; }
    if (live[2][0] != (uint16_t)s_bank1_pin[0] || live[3][0] != (uint16_t)s_bank1_pin[1]) {
        s_bank1_moved[0] = live[2][0];
        s_bank1_moved[1] = live[3][0];
        ++s_bank1_overrides;
        live[2][0] = (uint16_t)s_bank1_pin[0];
        live[3][0] = (uint16_t)s_bank1_pin[1];
    }
}

static void enter_debug(uint16_t live[PBUS_BLOCKS][2])
{
    pin_bank1(live);
    phy_pbus_debugmode();
    for (unsigned b = 0; b < PBUS_BLOCKS; ++b) if (pbus_owned(b))
        for (unsigned k = 0; k < 2; ++k) phy_pbus_force_test(b, k + 1, live[b][k]);
}
#endif

/* Baseline, 2x2 response, up to four closed-loop steps, exact rollback.
 * The caller pauses gain control; measure() returns the I/Q centre in
 * milli-cells of the current lane. */
esp_err_t phy_dco_search(bool (*measure)(int dc[2]), phy_dco_result_t *res)
{
    return phy_dco_search_from(measure, NULL, res);
}

/* start: the 4-bit IQ clips, so with several cells of DC the measured mean
 * saturates and the 2x2 probe sees no slope; the steps then ran to the clamp
 * (board, PR #228: G83 on 5805 MHz from the chip's codes, before=4.1 cells,
 * codes 316 = base+96, residual 3.5 cells, stored as OK and used as the
 * template of every other channel). From a neighbour row's converged codes
 * (rows ~1 dB apart) the start is within the linear range. */
esp_err_t phy_dco_search_from(bool (*measure)(int dc[2]), const int start[2], phy_dco_result_t *res)
{
    return phy_dco_search_verified(measure, measure, start, res);
}

esp_err_t phy_dco_search_verified(bool (*measure)(int dc[2]), bool (*verify)(int dc[2]),
                                 const int start[2], phy_dco_result_t *res)
{
    memset(res, 0, sizeof(*res));
#ifndef C5VRX_PHY_PINNED
    (void)verify;
    (void)measure;
    (void)start;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!measure || !verify) return ESP_ERR_INVALID_ARG;
    phy_begin();
    uint16_t live[PBUS_BLOCKS][2];
    read_live(live);
    esp_err_t result = ESP_ERR_INVALID_RESPONSE;
    int before[2] = {0, 0};
    bool debug = false;
    if (start) {
        debug = true;
        enter_debug(live);
        dco_apply(dco_clamp(start[0], start[0]), dco_clamp(start[1], start[1]));
    }
    if (measure(before)) {
        if (!debug) { debug = true; enter_debug(live); }
        int base[2] = {live[2][1], live[3][1]};
        if (start) { base[0] = dco_clamp(start[0], start[0]); base[1] = dco_clamp(start[1], start[1]); }
        int cur[2] = {base[0], base[1]};
        int best[2] = {base[0], base[1]}, best_dc[2] = {before[0], before[1]};
        int di = base[0] > 511 - DCO_PROBE ? -DCO_PROBE : DCO_PROBE;
        int dq = base[1] > 511 - DCO_PROBE ? -DCO_PROBE : DCO_PROBE;
        int mi[2] = {0, 0}, mq[2] = {0, 0};
        /* A centred seed needs confirmation, not two response probes. */
        const int centred_limit = verify != measure ? 300 : DCO_DONE_MCELLS;
        const bool centred = dco_cost(before) <= centred_limit * centred_limit;
        bool ok = true;
        if (!centred) {
            dco_apply(base[0] + di, base[1]);
            ok = measure(mi);
            if (ok) { dco_apply(base[0], base[1] + dq); ok = measure(mq); }
        }
        dco_apply(base[0], base[1]);
        float j[4] = {0, 0, 0, 0};
        if (ok) {
            j[0] = (float)(mi[0] - before[0]) / di; j[1] = (float)(mq[0] - before[0]) / dq;
            j[2] = (float)(mi[1] - before[1]) / di; j[3] = (float)(mq[1] - before[1]) / dq;
        }
        int err[2] = {before[0], before[1]};
        for (unsigned it = 0; ok && it < 4 && dco_cost(best_dc) > DCO_DONE_MCELLS * DCO_DONE_MCELLS; ++it) {
            int sa, sb;
            if (!dsp_dco_step(j, (float)err[0], (float)err[1], DCO_TRIAL, &sa, &sb)) break;
            cur[0] = dco_clamp(cur[0] + sa, base[0]);
            cur[1] = dco_clamp(cur[1] + sb, base[1]);
            dco_apply(cur[0], cur[1]);
            if (!measure(err)) { ok = false; break; }
            if (dco_cost(err) < dco_cost(best_dc)) {
                best[0] = cur[0]; best[1] = cur[1];
                best_dc[0] = err[0]; best_dc[1] = err[1];
            }
        }
        /* Never store a residual measured at an intermediate trial. The
         * short callback guides the search only; verification sees the
         * actual winning codes with the original long observation. */
        if (ok) {
            dco_apply(best[0], best[1]);
            ok = verify(best_dc);
            if (verify != measure && dco_cost(best_dc) > 300 * 300) ok = false;
        }
        if (ok) {
            res->measured = true;
            res->codes[0] = best[0]; res->codes[1] = best[1];
            res->residual_mcells[0] = best_dc[0]; res->residual_mcells[1] = best_dc[1];
            printf("DCO search before=%d/%d codes=%d/%d residual=%d/%d\n", before[0], before[1],
                   best[0], best[1], best_dc[0], best_dc[1]);
            result = ESP_OK;
        }
    }
    if (debug) {
        for (unsigned b = 0; b < PBUS_BLOCKS; ++b) if (pbus_owned(b))
            for (unsigned k = 0; k < 2; ++k) phy_pbus_force_test(b, k + 1, live[b][k]);
        pbus_workmode_keep_gain();
    }
    phy_end();
    return result;
#endif
}

/* FLOOR rows (receiver noise below one quantizer step) used to keep the
 * chip's own DC: (-1.5, +3) cells at G59 on the board, a fixed IQ point that
 * Direct Gain read as a healthy carrier and parked on (board walk
 * 2026-10-13, docs/FINDINGS.md). Without noise dither the mean only moves in
 * whole cells, so the Jacobian search above sees no slope. Bisection on the
 * sign of each axis' mean finds the code where the fixed code flips from
 * cell -0.5 to +0.5, i.e. DC 0 within half a cell. Two rounds absorb the
 * small I/Q cross-coupling. */
static int dco_flat_axis(bool (*measure)(int dc[2]), int cur[2], int axis, int base, bool *ok)
{
    int lo = 0, hi = 0, m[2];
    bool lo_pos = false, bracket = false;
    /* Neighbouring gains usually differ by only a few codes. Grow the
     * bracket instead of measuring the full +/-96 range for every axis.
     * The maximum range and adjacent-code bisection remain unchanged. */
    const int centre = cur[axis];
    for (int radius = 8; ; radius = radius * 2 > DCO_RANGE ? DCO_RANGE : radius * 2) {
        lo = dco_clamp((radius == DCO_RANGE ? base : centre) - radius, base);
        hi = dco_clamp((radius == DCO_RANGE ? base : centre) + radius, base);
        cur[axis] = lo; dco_apply(cur[0], cur[1]);
        if (!measure(m)) { *ok = false; return base; }
        lo_pos = m[axis] > 0;
        const int lo_abs = m[axis] < 0 ? -m[axis] : m[axis];
        cur[axis] = hi; dco_apply(cur[0], cur[1]);
        if (!measure(m)) { *ok = false; return base; }
        if ((m[axis] > 0) != lo_pos) { bracket = true; break; }
        if (radius == DCO_RANGE) {
            const int hi_abs = m[axis] < 0 ? -m[axis] : m[axis];
            *ok = false;
            return hi_abs < lo_abs ? hi : lo;
        }
    }
    if (!bracket) { *ok = false; return base; }
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        cur[axis] = mid; dco_apply(cur[0], cur[1]);
        if (!measure(m)) { *ok = false; return mid; }
        if ((m[axis] > 0) == lo_pos) lo = mid; else hi = mid;
    }
    return lo;
}

esp_err_t phy_dco_search_flat(bool (*measure)(int dc[2]), phy_dco_result_t *res)
{
    return phy_dco_search_flat_from(measure, NULL, res);
}

esp_err_t phy_dco_search_flat_from(bool (*measure)(int dc[2]), const int start[2], phy_dco_result_t *res)
{
    memset(res, 0, sizeof(*res));
#ifndef C5VRX_PHY_PINNED
    (void)measure;
    (void)start;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!measure) return ESP_ERR_INVALID_ARG;
    phy_begin();
    uint16_t live[PBUS_BLOCKS][2];
    read_live(live);
    esp_err_t result = ESP_ERR_INVALID_RESPONSE;
    int before[2] = {0, 0};
    bool debug = false;
    if (measure(before)) {
        debug = true;
        enter_debug(live);
        const int base[2] = {start ? dco_clamp(start[0], start[0]) : live[2][1],
                             start ? dco_clamp(start[1], start[1]) : live[3][1]};
        int cur[2] = {base[0], base[1]}, err[2] = {before[0], before[1]};
        bool ok = true;
        for (unsigned round = 0; round < 2u && ok; ++round)
            for (int axis = 0; axis < 2 && ok; ++axis) {
                bool axis_ok = true;
                cur[axis] = dco_flat_axis(measure, cur, axis, base[axis], &axis_ok);
                if (!axis_ok) ok = false;
            }
        dco_apply(cur[0], cur[1]);
        if (ok && measure(err) && err[0] >= -1000 && err[0] <= 1000 && err[1] >= -1000 && err[1] <= 1000) {
            res->measured = true;
            res->codes[0] = cur[0]; res->codes[1] = cur[1];
            res->residual_mcells[0] = err[0]; res->residual_mcells[1] = err[1];
            result = ESP_OK;
        }
        printf("DCO flat before=%d/%d codes=%d/%d residual=%d/%d %s\n", before[0], before[1], cur[0], cur[1],
               err[0], err[1], result == ESP_OK ? "ok" : "no_flip");
        dco_apply(base[0], base[1]);
    }
    if (debug) {
        for (unsigned b = 0; b < PBUS_BLOCKS; ++b) if (pbus_owned(b))
            for (unsigned k = 0; k < 2; ++k) phy_pbus_force_test(b, k + 1, live[b][k]);
        pbus_workmode_keep_gain();
    }
    phy_end();
    return result;
#endif
}

esp_err_t phy_dco_hold(int code_i, int code_q)
{
#ifndef C5VRX_PHY_PINNED
    (void)code_i; (void)code_q;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (code_i < 0 || code_i > 511 || code_q < 0 || code_q > 511) return ESP_ERR_INVALID_ARG;
    lock_take();
    if (s_dco_held) { lock_give(); return ESP_ERR_INVALID_STATE; }
    phy_observation_changed();
    uint16_t live[PBUS_BLOCKS][2];
    read_live(live);
    enter_debug(live);
    s_dco_codes[0] = code_i;
    s_dco_codes[1] = code_q;
    dco_apply(code_i, code_q);
    s_dco_held = true;
    lock_give();
    return ESP_OK;
#endif
}

bool phy_dco_release(void)
{
#ifndef C5VRX_PHY_PINNED
    return false;
#else
    if (!s_dco_held) return false;
    lock_take();
    phy_observation_changed();
    pbus_workmode_keep_gain();
    s_dco_held = false;
    lock_give();
    return true;
#endif
}

void phy_dco_bank1_reset(uint8_t lo, uint8_t hi)
{
#ifdef C5VRX_PHY_PINNED
    lock_take();
    s_bank1_lo = lo;
    s_bank1_hi = hi;
    s_bank1_pin[0] = s_bank1_pin[1] = -1;
    lock_give();
#else
    (void)lo; (void)hi;
#endif
}

uint32_t phy_dco_bank1_status(int pin[2], int moved[2])
{
#ifdef C5VRX_PHY_PINNED
    pin[0] = s_bank1_pin[0]; pin[1] = s_bank1_pin[1];
    moved[0] = s_bank1_moved[0]; moved[1] = s_bank1_moved[1];
    return s_bank1_overrides;
#else
    pin[0] = pin[1] = moved[0] = moved[1] = -1;
    return 0;
#endif
}

bool phy_dco_banks(int bank1[2], int bank2[2])
{
#ifdef C5VRX_PHY_PINNED
    lock_take();
    bank1[0] = phy_pbus_rd(2, 1); bank1[1] = phy_pbus_rd(3, 1);
    bank2[0] = phy_pbus_rd(2, 2); bank2[1] = phy_pbus_rd(3, 2);
    lock_give();
    return true;
#else
    bank1[0] = bank1[1] = bank2[0] = bank2[1] = 0;
    return false;
#endif
}

/* Vendor RX gain memory source (pinned libphy, phy_wr_rx_gain_mem): bank 1
 * of a 5 GHz row is phy_param+0x180 [frequency group][RF stage of the row]
 * [I, Q], filled by the boot RX DC calibration (phy_rfrx_rxdc_cal_new) and
 * reloaded into the AGC memory by every channel setup (phy_chip_set_chan).
 * The groups are the vendor's phy_freq_to_index, of its own current
 * frequency (phy_param+0x120). */
#ifdef C5VRX_PHY_PINNED
extern unsigned char phy_param[];
static unsigned vendor_freq_group(unsigned mhz)
{
    return mhz < 5241 ? 0 : mhz <= 5320 ? 1 : mhz <= 5560 ? 2 : mhz <= 5640 ? 3 : mhz <= 5755 ? 4 : mhz <= 5835 ? 5 : 6;
}
const int16_t *phy_vendor_bank1_table(unsigned *group)
{
    *group = vendor_freq_group(*(const uint16_t *)(phy_param + 0x120));
    return (const int16_t *)(phy_param + 0x180);
}
#else
const int16_t *phy_vendor_bank1_table(unsigned *group) { *group = 0; return NULL; }
#endif

bool phy_dco_held(void)
{
#ifdef C5VRX_PHY_PINNED
    return s_dco_held;
#else
    return false;
#endif
}

bool phy_dco_nudge(int di, int dq, int out[2])
{
#ifndef C5VRX_PHY_PINNED
    (void)di; (void)dq; (void)out;
    return false;
#else
    lock_take();
    bool ok = s_dco_held;
    if (ok) {
        phy_observation_changed();
        int i = s_dco_codes[0] + di, q = s_dco_codes[1] + dq;
        s_dco_codes[0] = i < 0 ? 0 : i > 511 ? 511 : i;
        s_dco_codes[1] = q < 0 ? 0 : q > 511 ? 511 : q;
        dco_apply(s_dco_codes[0], s_dco_codes[1]);
        if (out) { out[0] = s_dco_codes[0]; out[1] = s_dco_codes[1]; }
    }
    lock_give();
    return ok;
#endif
}

bool phy_dco_read_pair(int out[2])
{
#ifndef C5VRX_PHY_PINNED
    (void)out;
    return false;
#else
    if (!out) return false;
    lock_take();
    int i = phy_pbus_rd(2, 2), q = phy_pbus_rd(3, 2);
    bool ok = i == phy_pbus_rd(2, 2) && q == phy_pbus_rd(3, 2);
    if (ok) { out[0] = i; out[1] = q; }
    lock_give();
    return ok;
#endif
}

esp_err_t phy_dco_try_pair(uint32_t packed_gain, const int expected[2], const int target[2])
{
#ifndef C5VRX_PHY_PINNED
    (void)packed_gain; (void)expected; (void)target;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!expected || !target || expected[0] < 0 || expected[0] > 511 || expected[1] < 0 || expected[1] > 511 ||
        target[0] < 0 || target[0] > 511 || target[1] < 0 || target[1] > 511)
        return ESP_ERR_INVALID_ARG;
    /* One axis per trial, up to PHY_DCO_TRY_MAX codes (the search itself
     * probes 16): a DC that moved by cells after a channel setup needs more
     * than single-code steps. */
    int di = target[0] - expected[0], dq = target[1] - expected[1];
    const int ai = di < 0 ? -di : di, aq = dq < 0 ? -dq : dq;
    if ((ai && aq) || !(ai + aq) || ai > PHY_DCO_TRY_MAX || aq > PHY_DCO_TRY_MAX) return ESP_ERR_INVALID_ARG;
    lock_take();
    int current[2];
    uint16_t rf = phy_pbus_rd(8, 1), bb = phy_pbus_rd(0, 2), fine = phy_pbus_rd(1, 2);
    bool gain_ok = rf == (packed_gain >> 12) && (bb & 127u) == ((packed_gain >> 4) & 127u) &&
                   ((bb & ~127u) == 0u || (bb & ~127u) == 0x100u) && fine == (0xd0u | (packed_gain & 7u));
    if (!gain_ok || !phy_dco_read_pair(current) || current[0] != expected[0] || current[1] != expected[1]) {
        lock_give();
        return ESP_ERR_INVALID_STATE;
    }
    phy_observation_changed();
    if (!s_dco_held) {
        uint16_t live[PBUS_BLOCKS][2];
        read_live(live);
        enter_debug(live);
        s_dco_held = true;
    }
    dco_apply(target[0], target[1]);
    bool ok = phy_dco_read_pair(current) && current[0] == target[0] && current[1] == target[1];
    if (ok) {
        s_dco_codes[0] = target[0]; s_dco_codes[1] = target[1];
        lock_give();
        return ESP_OK;
    }
    dco_apply(expected[0], expected[1]);
    bool restored = phy_dco_read_pair(current) && current[0] == expected[0] && current[1] == expected[1];
    s_dco_codes[0] = expected[0]; s_dco_codes[1] = expected[1];
    lock_give();
    return restored ? ESP_FAIL : ESP_ERR_INVALID_RESPONSE;
#endif
}
