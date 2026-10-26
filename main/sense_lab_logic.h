/* C5VRX-5 by Twotoz and the C5VRX contributors: VTX-on PHY sensitivity lab
 * (console S), pure part. Extends the PR #254 VTX-off noise lab
 * (docs/PR254_PHY_FINDINGS.md, frozen at 6274b2d) and the pinned PHY work in
 * legacy/c5vrx4/main/phy_rx_lab.c (#150-155); 11p observation by SushiDude
 * (@Ready4Sushi). See docs/PHY_SENSITIVITY_LAB.md.
 *
 * PR #254 compared captured noise variance with the VTX off. A lower noise
 * variance can also lower the wanted carrier, so it is not a sensitivity
 * result. This lab compares the same settings with a VTX carrier present and
 * scores carrier and noise together from each contiguous 8190-byte capture:
 *
 * - cn_db: M2M4 carrier-to-noise estimate. For a constant-envelope FM carrier
 *   in complex Gaussian noise, M2 = S + N and M4 = S^2 + 4SN + 2N^2, so
 *   S = sqrt(2 M2^2 - M4) and N = M2 - S. A change that lowers both carrier
 *   and noise leaves it unchanged; a real sensitivity change moves it.
 *   Filter AM (deviation reaching the analog filter skirt) and the 4-bit
 *   quantiser also read as noise, so absolute values are biased; within one
 *   profile at the same input the bias is common to STOCK and TEST, except
 *   for profiles that change the analog filter (see the lab document).
 * - click_pm: adjacent sample pairs whose phase step exceeds 3/4 pi, per
 *   mille. The largest FM step is 2 pi 9 MHz / 40 MS/s = 1.41 rad, so these
 *   are noise passing near the origin: FM threshold clicks, the impulses
 *   that turn into sparkles and lost sync at the range edge.
 * - origin_pm: samples in the four cells around the origin, per mille.
 * - rail_pm: samples on a signed-nibble rail (-8 or 7), per mille.
 * - video_snr_db: demodulated video SNR of the existing AFC decoder
 *   (afc.h afc2_snr_x10: sync-to-blanking distance over the spread of the
 *   instantaneous frequency on the sync tips and back porch), pooled over
 *   the capture's two halves; video_lines counts the syncs it measured.
 *   NAN without lines. It scores what the goggles get after FM detection.
 *
 * Units: cells of the signed Q4/I4 nibbles (x = 2v + 1 in dsp units).
 * Host test: test/test_sense_metrics.c. */
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "afc.h"
#include "cn_auto.h"

#define SENSE_CN_FLOOR_DB   (-20.0)
#define SENSE_CN_CEIL_DB    (30.0)
#define SENSE_PI            3.14159265358979323846
#define SENSE_CLICK_RAD     (0.75 * SENSE_PI)

typedef struct {
    double m2, m4, kurt;     /* DC-removed moments, cells^2 / cells^4 */
    double cn_db;            /* M2M4, clamped to [SENSE_CN_FLOOR_DB, SENSE_CN_CEIL_DB] */
    double click_pm, origin_pm, rail_pm;
    float dc_i, dc_q;        /* window mean, cells */
    unsigned video_lines;    /* syncs measured by the AFC decoder */
    double video_snr_db;     /* NAN without lines */
} sense_metrics_t;

static inline double sense_m2m4_db(double m2, double m4)
{
    if (!(m2 > 0.0)) return SENSE_CN_FLOOR_DB;
    const double d = 2.0 * m2 * m2 - m4;
    const double s = d > 0.0 ? sqrt(d) : 0.0;
    const double n = m2 - s;
    if (s <= 0.0) return SENSE_CN_FLOOR_DB;
    if (n <= s * 1e-3) return SENSE_CN_CEIL_DB;
    const double db = 10.0 * log10(s / n);
    return db < SENSE_CN_FLOOR_DB ? SENSE_CN_FLOOR_DB : db > SENSE_CN_CEIL_DB ? SENSE_CN_CEIL_DB : db;
}

/* Signed nibble cells of one raw byte (I high, Q low), as cna_sums_add. */
static inline void sense_cells(uint8_t raw, int *v_i, int *v_q)
{
    *v_i = (int8_t)(raw & 240u) >> 4;
    *v_q = (int8_t)((raw & 15u) << 4) >> 4;
}

/* Video SNR of the AFC decoder pooled over chunks of at most 4092 bytes. */
static inline void sense_video(const uint8_t *s, size_t n, const uint8_t lut[256], unsigned *lines, double *snr_db)
{
    int64_t ss = 0, sq = 0, ps = 0, pq = 0;
    uint32_t sn = 0, pn = 0;
    *lines = 0;
    for (size_t off = 0; off + 1024u <= n; off += 4092u) {
        const size_t len = n - off < 4092u ? n - off : 4092u;
        const afc2_result_t r = afc2_measure(s + off, len, lut);
        if (!r.lines) continue;
        *lines += r.lines;
        ss += r.sync_sum; sq += r.sync_sq; sn += r.sync_pairs;
        ps += r.porch_sum; pq += r.porch_sq; pn += r.porch_pairs;
    }
    *snr_db = NAN;
    if (*lines && sn >= 32u && pn >= 32u) *snr_db = afc2_snr_x10(ss, sq, sn, ps, pq, pn) / 10.0;
}

static inline void sense_measure(const uint8_t *s, size_t n, const uint8_t lut[256], sense_metrics_t *out)
{
    cna_sums_t sums = {0};
    unsigned origin = 0, rails = 0, clicks = 0;
    for (size_t k = 0; k < n; ++k) {
        int vi, vq;
        sense_cells(s[k], &vi, &vq);
        cna_sums_add(&sums, s[k]);
        origin += (vi == 0 || vi == -1) && (vq == 0 || vq == -1);
        rails += vi == -8 || vi == 7 || vq == -8 || vq == 7;
    }
    float dci = 0.0f, dcq = 0.0f;
    cna_mean(&sums, &dci, &dcq);
    double m2 = 0.0, m4 = 0.0;
    cna_moments(&sums, dci, dcq, &m2, &m4);
    /* Phase steps around the window mean (cell centres at v + 0.5). Single
     * precision: the C5 FPU has no double support (afc.h). */
    const float pi = (float)SENSE_PI, click = (float)SENSE_CLICK_RAD;
    float prev = 0.0f;
    for (size_t k = 0; k < n; ++k) {
        int vi, vq;
        sense_cells(s[k], &vi, &vq);
        const float ph = atan2f((float)vq + 0.5f - dcq, (float)vi + 0.5f - dci);
        if (k) {
            float d = ph - prev;
            if (d > pi) d -= 2.0f * pi;
            else if (d < -pi) d += 2.0f * pi;
            clicks += fabsf(d) > click;
        }
        prev = ph;
    }
    out->m2 = m2;
    out->m4 = m4;
    out->kurt = m2 > 0.0 ? m4 / (m2 * m2) : 0.0;
    out->cn_db = sense_m2m4_db(m2, m4);
    out->click_pm = n > 1 ? 1000.0 * clicks / (double)(n - 1) : 0.0;
    out->origin_pm = n ? 1000.0 * origin / (double)n : 0.0;
    out->rail_pm = n ? 1000.0 * rails / (double)n : 0.0;
    out->dc_i = dci;
    out->dc_q = dcq;
    sense_video(s, n, lut, &out->video_lines, &out->video_snr_db);
}

/* ---- profiles: masked writes, saved and restored exactly ---------------
 * MMIO: addr/mask/value. I2C: analog block/host/reg with an 8-bit mask.
 * A profile with no patches is a sham (STOCK twice); it measures the
 * comparison's own spread. */
enum { SENSE_P_MMIO = 0, SENSE_P_I2C = 1 };
typedef struct {
    uint8_t kind;
    uint8_t block, host, reg;    /* I2C */
    uint32_t addr, mask, value;  /* MMIO; I2C uses the low 8 bits of mask/value */
} sense_patch_t;
#define SENSE_MAX_PATCHES 12u
enum { SENSE_K_PATCH = 0, SENSE_K_GAIN = 1, SENSE_K_OUTPUT_OFF = 2 };
typedef struct {
    const char *name;
    uint8_t kind;                /* SENSE_K_* */
    uint8_t n;
    uint8_t gain;                /* SENSE_K_GAIN: the TEST row */
    sense_patch_t p[SENSE_MAX_PATCHES];
} sense_profile_t;

#define SENSE_MMIO(a, m, v) {SENSE_P_MMIO, 0, 0, 0, (a), (m), (v)}
#define SENSE_I2C(b, h, r, m, v) {SENSE_P_I2C, (b), (h), (r), 0, (m), (v)}

/* Per-stage running statistics of one metric. */
typedef struct { unsigned n; double sum, sumsq; } sense_acc_t;
static inline void sense_acc_add(sense_acc_t *a, double v) { ++a->n; a->sum += v; a->sumsq += v * v; }
static inline double sense_acc_mean(const sense_acc_t *a) { return a->n ? a->sum / a->n : 0.0; }
static inline double sense_acc_var(const sense_acc_t *a)
{
    if (a->n < 2) return 0.0;
    const double m = a->sum / a->n, v = (a->sumsq - a->n * m * m) / (a->n - 1);
    return v > 0.0 ? v : 0.0;
}
/* Difference of two pooled means and its standard error. */
static inline double sense_delta(const sense_acc_t *test, const sense_acc_t *stock, double *se)
{
    const double vt = test->n ? sense_acc_var(test) / test->n : 0.0;
    const double vs = stock->n ? sense_acc_var(stock) / stock->n : 0.0;
    if (se) *se = sqrt(vt + vs);
    return sense_acc_mean(test) - sense_acc_mean(stock);
}
