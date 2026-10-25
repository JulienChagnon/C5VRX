/* C5VRX-5 by Twotoz and the C5VRX contributors: AGC comparison harness.
 *
 * The receiver, DC and fading model of test_keel_track.c (same scenarios,
 * same 4-bit sample observations through dg_measure_gain, same picture
 * score), run with several gain controllers on identical draws:
 *
 *   KEEL-1      the firmware core, dg_tick() (main/gain.c)
 *   LEVEL       the input-level AGC (main/agc.c)
 *   RSSI        a classic mean-power AGC (main/agc.c)
 *   STATIC      the best single home row for the whole run (no writes; an
 *               a-posteriori choice, so a bound for "never write")
 *   IDEAL       any row usable at that tick, no write or DC cost: an upper
 *               bound no causal controller reaches
 *
 * Robustness: -DGAIN_ERR_DB=x adds a fixed random error (+-x dB, per row)
 * between the true row gains and the model every controller uses; -DFOLD_RN=x
 * changes the fold rule's noise multiple (default 1.0) and -DDC0_CELLS the
 * DC. A controller tuned on one model has to hold up on the others.
 *
 *   cc -O2 -std=c11 -I../main test_agc_compare.c ../main/agc.c ../main/gain.c ../main/arc_phy.c -lm -o t && ./t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "agc.h"
#include "gain.h"
#include "phase8_gain_lut.h"

unsigned char phy_param[2048];

#define TICK_US 200u
#ifndef WRITE_COST_US
#define WRITE_COST_US 2000u
#endif
#ifndef FOLD_COST_US
#define FOLD_COST_US 4000u
#endif
#ifndef DC_TAU_US
#define DC_TAU_US 100000.0
#endif
#ifndef DC0_CELLS
#define DC0_CELLS 1.0
#endif
#ifndef DC_FLOOR_CELLS
#define DC_FLOOR_CELLS 0.1
#endif
#ifndef GAIN_ERR_DB
#define GAIN_ERR_DB 0.0
#endif
#ifndef FOLD_RN
#define FOLD_RN 1.0
#endif
#ifndef SEEDS
#define SEEDS 3u
#endif

static arc_gain_table_t table;
static double s_model_db[DG_STATES];   /* what the controllers believe */
static double s_db[DG_STATES];         /* the truth the samples follow */
static double s_dc0[DG_STATES], s_dca[DG_STATES], s_dwell[DG_STATES];
static uint32_t s_seed = 7;
static double urand(void) { s_seed = s_seed * 1664525u + 1013904223u; return (s_seed >> 8) / 16777216.0; }

static void build_model(void)
{
    arc_gain_table_from_bytes(&table, NULL, ARC_5G_GAIN_MAX);
    uint32_t keep = s_seed;
    s_seed = 99u;
    for (unsigned k = 0; k <= table.max_index; ++k) {
        arc_gain_tuple_t t;
        arc_gain_tuple_decode(&table, (uint8_t)k, &t);
        s_model_db[k] = 6.0 * t.rf_stage + 6.0 * log2(t.bb_code + 1.0) + 0.83 * (5 - (int)t.fine_code);
        s_db[k] = s_model_db[k] + GAIN_ERR_DB * (2.0 * urand() - 1.0) * (k != table.max_index);
    }
    s_seed = keep;
}
static void dc_reset(uint32_t seed)
{
    s_seed = seed;
    for (unsigned k = 0; k < DG_STATES; ++k) { s_dc0[k] = DC0_CELLS * (0.5 + urand()); s_dca[k] = 6.2831853 * urand(); s_dwell[k] = 0; }
}
static double dc_cells(unsigned k)
{
    const double d = s_dc0[k] * exp(-s_dwell[k] / DC_TAU_US);
    return d > DC_FLOOR_CELLS ? d : DC_FLOOR_CELLS;
}
static double radius(double level_db, unsigned k) { return level_db <= -200 ? 0 : pow(10.0, (level_db + s_db[k]) / 20.0); }
static double noise_radius(unsigned k) { return 1.95 * sqrt(2.0) * pow(10.0, (s_db[k] - s_db[table.max_index]) / 20.0); }

static uint32_t s_nseed = 12345u;
static double s_phi;
static double gauss(void)
{
    double u1, u2;
    do { s_nseed = s_nseed * 1664525u + 1013904223u; u1 = (s_nseed >> 8) / 16777216.0; } while (u1 <= 0.0);
    s_nseed = s_nseed * 1664525u + 1013904223u; u2 = (s_nseed >> 8) / 16777216.0;
    return sqrt(-2.0 * log(u1)) * cos(6.2831853 * u2);
}
static int wrap4(double x) { const int c = (int)floor(x); return ((c + 8) & 15) - 8; }

static dg_observation_t observe(double level_db, unsigned k, uint64_t t)
{
    uint8_t b[256];
    const double r = radius(level_db, k), sigma = noise_radius(k) / sqrt(2.0), d = dc_cells(k);
    const double di = d * cos(s_dca[k]), dq = d * sin(s_dca[k]);
    for (unsigned n = 0; n < 256u; ++n) {
        const double ts = (double)t * 1e-6 + n / 40e6;
        const double f = -1.0e6 + 1.5e6 * sin(6.2831853 * 15734.0 * ts);
        s_phi += 6.2831853 * f / 40e6;
        const int i = wrap4(r * cos(s_phi) + sigma * gauss() + di);
        const int q = wrap4(r * sin(s_phi) + sigma * gauss() + dq);
        b[n] = (uint8_t)(((unsigned)i & 15u) << 4 | ((unsigned)q & 15u));
    }
    return dg_measure_gain(b, sizeof(b), c5vrx_phase8_gain_lut, t);
}

/* Picture model of test_keel_track.c: detector small-signal loss from the
 * bit-exact U85Q chain, a fold is a lost moment. */
static double small_signal_loss_db(double rms)
{
    static const double at[] = {1.5, 2.2, 3.0, 4.2}, loss[] = {2.9, 1.4, 0.6, 0.0};
    if (rms <= at[0]) return loss[0] + (at[0] - rms) * 2.0;
    for (int k = 1; k < 4; ++k)
        if (rms <= at[k]) return loss[k - 1] + (loss[k] - loss[k - 1]) * (rms - at[k - 1]) / (at[k] - at[k - 1]);
    return 0.0;
}
static double snr_dc(double level_db, unsigned k, double d)
{
    const double r = radius(level_db, k), rn = noise_radius(k);
    if (r + FOLD_RN * rn + d > 7.5) return -99.0;
    return 10.0 * log10(r * r / (rn * rn + 1.0 / 6.0 + d * d) + 1e-9) - small_signal_loss_db(sqrt(r * r + rn * rn));
}
static double snr_db(double level_db, unsigned k) { return snr_dc(level_db, k, dc_cells(k)); }

#define PATHS 12
static double s_phase[PATHS], s_cos[PATHS];
static void fading_init(uint32_t seed)
{
    uint32_t keep = s_seed;
    s_seed = seed;
    for (int p = 0; p < PATHS; ++p) { s_phase[p] = 6.2831853 * urand(); s_cos[p] = cos(6.2831853 * urand()); }
    s_seed = keep;
}
static double fading_db(uint64_t t_us, double doppler_hz, double k_factor)
{
    double re = 0, im = 0;
    for (int p = 0; p < PATHS; ++p) {
        const double a = 6.2831853 * doppler_hz * s_cos[p] * (double)t_us * 1e-6 + s_phase[p];
        re += cos(a); im += sin(a);
    }
    re /= sqrt((double)PATHS); im /= sqrt((double)PATHS);
    const double los = sqrt(k_factor);
    const double h2 = ((los + re) * (los + re) + im * im) / (k_factor + 1.0);
    return 10.0 * log10(h2 > 1e-6 ? h2 : 1e-6);
}

typedef struct {
    const char *name;
    double level_db, doppler, k, step_db, threshold_db, weight;
} scenario_t;
typedef struct { double usable_pct, writes_s, mean_gain; } result_t;

/* ---- controllers ---- */
typedef enum { C_KEEL, C_LEVEL, C_RSSI, C_STATIC, C_IDEAL, C_COUNT } ctl_t;
static const char *const s_ctl_name[C_COUNT] = {"KEEL-1", "LEVEL", "RSSI", "STATIC", "IDEAL"};
static direct_gain_t g;
static agc_level_t s_lv;
static agc_rssi_t s_rs;
static uint8_t s_home[DG_STATES];
static unsigned s_homes;

static void build_homes(void)
{
    /* Home rows 3 dB apart from the table maximum down (KEEL-1 lattice). */
    s_homes = 0;
    double last = 1e9;
    for (int k = (int)table.max_index; k >= 0; --k)
        if (last - s_model_db[k] >= 3.0 - 1e-9) { s_home[s_homes++] = (uint8_t)k; last = s_model_db[k]; }
}

static result_t run(const scenario_t *sc, uint32_t seed, ctl_t ctl, int fixed_row)
{
    dc_reset(seed);
    fading_init(seed * 31u + 7u);
    s_nseed = 12345u + seed;
    s_phi = 0;
    uint8_t applied = ctl == C_STATIC ? (uint8_t)fixed_row : table.max_index;
    dg_reset(&g, &table, applied);
    g.learn_enabled = true;
    if (ctl == C_LEVEL || ctl == C_RSSI) {
        int16_t db[DG_STATES];
        for (unsigned k = 0; k <= table.max_index; ++k) db[k] = (int16_t)lround(s_model_db[k] * 256.0);
        if (ctl == C_LEVEL) agc_level_reset(&s_lv, db, table.max_index, (uint16_t)lround(1.95 * 1.95 * 2.0 * 16.0));
        else agc_rssi_reset(&s_rs, db, table.max_index);
    }
    uint64_t t = 1000, last_write = 0, last_fold = 0;
    const unsigned settle_ticks = 5000u, ticks = 25000u;
    unsigned ok = 0, writes = 0;
    double gain_sum = 0;
    for (unsigned n = 0; n < settle_ticks + ticks; ++n, t += TICK_US) {
        double lv = sc->level_db + fading_db(t, sc->doppler, sc->k);
        if (sc->step_db != 0 && n >= settle_ticks + ticks / 2u) lv = sc->step_db <= -300 ? -300 : lv + sc->step_db;
        if (ctl == C_IDEAL) {
            bool any = false;
            for (unsigned h = 0; h < s_homes && !any; ++h) any = snr_dc(lv, s_home[h], DC_FLOOR_CELLS) >= sc->threshold_db;
            if (n >= settle_ticks) ok += any;
            continue;
        }
        uint8_t target = applied;
        if (ctl != C_STATIC) {
            const dg_observation_t o = observe(lv, applied, t);
            target = ctl == C_KEEL ? dg_tick(&g, &o) : ctl == C_LEVEL ? agc_level_tick(&s_lv, &o, applied, t)
                                                                      : agc_rssi_tick(&s_rs, &o, applied, t);
        }
        if (target != applied) {
            applied = target;
            if (ctl == C_KEEL) dg_sync_applied(&g, target, t);
            last_write = t;
            if (n >= settle_ticks) ++writes;
        } else if (!last_write || t - last_write >= 400u) s_dwell[applied] += TICK_US;
        if (snr_db(lv, applied) < -90.0) last_fold = t;
        if (n >= settle_ticks) {
            const bool fresh = (!last_write || t - last_write >= WRITE_COST_US) && (!last_fold || t - last_fold >= FOLD_COST_US);
            ok += fresh && snr_db(lv, applied) >= sc->threshold_db;
            gain_sum += applied;
        }
    }
    return (result_t){100.0 * ok / ticks, writes / 5.0, gain_sum / ticks};
}

static result_t run_static(const scenario_t *sc, uint32_t seed)
{
    result_t best = {-1, 0, 0};
    for (unsigned h = 0; h < s_homes; ++h) {
        const result_t r = run(sc, seed, C_STATIC, s_home[h]);
        if (r.usable_pct > best.usable_pct) best = r;
    }
    return best;
}

int main(int argc, char **argv)
{
    build_model();
    build_homes();
    const bool verbose = argc > 1 && !strcmp(argv[1], "-v");
    const double top = s_db[table.max_index];
    const double close = 20.0 * log10(4.5) - s_model_db[28], mid = 20.0 * log10(4.5) - s_model_db[60];
    const double edge0 = 20.0 * log10(2.8) - top, edge3 = 20.0 * log10(3.9) - top, edge6 = 20.0 * log10(5.5) - top;
    const double edge10 = 20.0 * log10(8.7) - top, edge18 = 20.0 * log10(21.9) - top;
    const scenario_t sc[] = {
        {"close, walking (K 10, 5 Hz)", close, 5, 10, 0, 10, 1},
        {"close, moving (K 3, 30 Hz)", close, 30, 3, 0, 10, 1},
        {"close, Rayleigh 30 Hz", close, 30, 0, 0, 10, 1},
        {"close, fast (K 3, 80 Hz)", close, 80, 3, 0, 10, 1},
        {"mid, moving (K 3, 30 Hz)", mid, 30, 3, 0, 10, 1},
        {"mid, Rayleigh 30 Hz", mid, 30, 0, 0, 10, 1},
        {"mid, -12 dB behind an obstacle", mid, 30, 3, -12, 10, 1},
        {"edge 6 dB C/N (K 3, 30 Hz)", edge6, 30, 3, 0, 3, 2},
        {"edge 3 dB C/N (K 3, 30 Hz)", edge3, 30, 3, 0, 3, 2},
        {"edge 3 dB C/N, Rayleigh", edge3, 30, 0, 0, 3, 2},
        {"edge 0 dB C/N (K 3, 30 Hz)", edge0, 30, 3, 0, 3, 2},
        {"edge 3 dB C/N, walking (K 10, 5 Hz)", edge3, 5, 10, 0, 3, 2},
        {"walk-out 10 dB C/N (K 3, 10 Hz)", edge10, 10, 3, 0, 3, 2},
        {"walk-out 10 dB C/N, Rayleigh 10 Hz", edge10, 10, 0, 0, 3, 2},
        {"walk-out 18 dB C/N (K 3, 10 Hz)", edge18, 10, 3, 0, 10, 2},
        {"walk-out 18 dB C/N, Rayleigh 10 Hz", edge18, 10, 0, 0, 10, 2},
        {"walking 18 dB C/N (K 3, 2 Hz)", edge18, 2, 3, 0, 10, 2},
        {"walking 18 dB C/N, Rayleigh 5 Hz", edge18, 5, 0, 0, 10, 2},
        {"walking 6 dB C/N (K 3, 2 Hz)", edge6, 2, 3, 0, 3, 2},
        {"close, walking Rayleigh 2 Hz", close, 2, 0, 0, 10, 1},
        {"fast drone 100 km/h, mid (K 3, 540 Hz)", mid, 540, 3, 0, 10, 1},
        {"fast drone 100 km/h, edge 6 dB (K 3)", edge6, 540, 3, 0, 3, 2},
        {"fast drone 200 km/h, edge 3 dB (K 3)", edge3, 1070, 3, 0, 3, 2},
        {"fast drone 100 km/h, walk-out 18 dB Rayleigh", edge18, 540, 0, 0, 10, 2},
    };
    const unsigned n = sizeof(sc) / sizeof(sc[0]);
    double score[C_COUNT] = {0}, writes[C_COUNT] = {0}, edge3u[C_COUNT] = {0}, weights = 0;
    printf("%-44s %16s %16s %16s %16s %8s\n", "scenario (usable %, writes/s)", "KEEL-1", "LEVEL", "RSSI", "STATIC", "IDEAL");
    for (unsigned k = 0; k < n; ++k) {
        result_t a[C_COUNT] = {{0}};
        for (uint32_t seed = 1; seed <= SEEDS; ++seed)
            for (int c = 0; c < C_COUNT; ++c) {
#ifdef SKIP_BOUNDS
                if (c == C_STATIC || c == C_IDEAL) continue;
#endif
                const result_t b = c == C_STATIC ? run_static(&sc[k], seed) : run(&sc[k], seed, (ctl_t)c, 0);
                a[c].usable_pct += b.usable_pct / SEEDS; a[c].writes_s += b.writes_s / SEEDS; a[c].mean_gain += b.mean_gain / SEEDS;
            }
        printf("%-44s", sc[k].name);
        for (int c = 0; c < C_STATIC; ++c) printf(" %6.1f %4.1f G%-3.0f", a[c].usable_pct, a[c].writes_s, a[c].mean_gain);
        printf(" %6.1f      G%-3.0f %6.1f\n", a[C_STATIC].usable_pct, a[C_STATIC].mean_gain, a[C_IDEAL].usable_pct);
        for (int c = 0; c < C_COUNT; ++c) {
            score[c] += sc[k].weight * a[c].usable_pct;
            writes[c] += a[c].writes_s / n;
            if (k == 8u) edge3u[c] = a[c].usable_pct;
        }
        weights += sc[k].weight;
    }
    printf("\n%-10s %8s %10s %12s\n", "", "SCORE", "writes/s", "edge 3 dB");
    for (int c = 0; c < C_COUNT; ++c)
        printf("%-10s %8.2f %10.1f %12.1f\n", s_ctl_name[c], score[c] / weights, writes[c], edge3u[c]);
    (void)verbose;
    return 0;
}
