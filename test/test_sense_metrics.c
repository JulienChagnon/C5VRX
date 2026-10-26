/* Host test for main/sense_lab_logic.h (VTX-on PHY sensitivity lab).
 * A synthetic FM carrier (video-like modulation up to +-6 MHz at 40 MS/s)
 * plus complex Gaussian noise, quantised to the signed Q4/I4 nibbles of the
 * receiver. Checks that the M2M4 estimate follows the true carrier-to-noise
 * ratio (thermal plus 4-bit quantisation noise), that clicks and origin
 * occupancy fall as C/N rises, and that a common gain change (carrier and
 * noise scaled together) leaves the score unchanged. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "sense_lab_logic.h"
#include "phase8_gain_lut.h"

static uint64_t rng = 0x9e3779b97f4a7c15ull;
static double uniform(void)
{
    rng = rng * 6364136223846793005ull + 1442695040888963407ull;
    return ((rng >> 11) + 0.5) / 9007199254740992.0;
}
static double gauss(void)
{
    return sqrt(-2.0 * log(uniform())) * cos(2.0 * SENSE_PI * uniform());
}
static uint8_t quantise(double i, double q)
{
    int vi = (int)floor(i), vq = (int)floor(q);
    vi = vi < -8 ? -8 : vi > 7 ? 7 : vi;
    vq = vq < -8 ? -8 : vq > 7 ? 7 : vq;
    return (uint8_t)(((vi & 15) << 4) | (vq & 15));
}
/* amplitude in cells, noise sigma per axis in cells. */
static void make(uint8_t *buf, size_t n, double amp, double sigma)
{
    double ph = uniform() * 2.0 * SENSE_PI;
    const unsigned offset = (unsigned)(uniform() * 2540.0);
    for (size_t k = 0; k < n; ++k) {
        /* NTSC-like FM line (2540 samples, 63.5 us): 4.7 us sync at -4.5 MHz,
         * back porch at -1 MHz (the AFC target) with a 3.58 MHz colour burst
         * of +-0.6 MHz deviation 1..3 us after the sync, active video up to
         * +5 MHz with a 1 MHz detail tone, 1.5 us front porch. */
        const unsigned pos = (unsigned)((k + offset) % 2540u);
        const double t = k / 40e6;
        double f;
        if (pos < 188u) f = -4.5e6;
        else if (pos < 416u || pos >= 2480u) {
            f = -1.0e6;
            if (pos >= 228u && pos < 308u) f += 0.6e6 * sin(2.0 * SENSE_PI * 3579545.0 * t);
        } else f = -1.0e6 + 6.0e6 * (0.5 + 0.5 * sin(2.0 * SENSE_PI * 1e6 * t));
        ph += 2.0 * SENSE_PI * f / 40e6;
        buf[k] = quantise(amp * cos(ph) + sigma * gauss(), amp * sin(ph) + sigma * gauss());
    }
}
static int failures;
static void check(int ok, const char *what)
{
    if (!ok) { printf("FAIL %s\n", what); ++failures; }
}

int main(void)
{
    enum { N = 8190, REPS = 16 };
    static uint8_t buf[N];
    const double amp = 3.2;
    double prev_cn = -99.0, prev_click = 1e9, prev_origin = 1e9, prev_video = -99.0;
    for (double cn_true = 0.0; cn_true <= 15.0; cn_true += 3.0) {
        /* Thermal noise for this C/N; the quantiser adds 1/6 cells^2. */
        const double noise = amp * amp / pow(10.0, cn_true / 10.0);
        const double sigma = sqrt(noise / 2.0);
        const double eff = 10.0 * log10(amp * amp / (noise + 1.0 / 6.0));
        double cn = 0.0, click = 0.0, origin = 0.0, rail = 0.0, video = 0.0;
        unsigned video_n = 0, lines = 0;
        for (int r = 0; r < REPS; ++r) {
            sense_metrics_t m;
            make(buf, N, amp, sigma);
            sense_measure(buf, N, c5vrx_phase8_gain_lut, &m);
            cn += m.cn_db / REPS; click += m.click_pm / REPS; origin += m.origin_pm / REPS; rail += m.rail_pm / REPS;
            lines += m.video_lines;
            if (!isnan(m.video_snr_db)) { video += m.video_snr_db; ++video_n; }
        }
        if (video_n) video /= video_n;
        printf("cn_true=%5.1f effective=%5.2f m2m4=%6.2f click_pm=%7.3f origin_pm=%7.3f rail_pm=%6.3f lines=%u "
               "video_snr_db=%6.2f (%u/%d captures)\n", cn_true, eff, cn, click, origin, rail, lines, video_n ? video : NAN,
               video_n, REPS);
        if (cn_true >= 6.0) {
            check(video_n == REPS, "the AFC decoder finds syncs from 6 dB C/N up");
            check(video > prev_video, "video SNR rises with C/N");
            prev_video = video;
        }
        /* Absolute bias: the quantisation error of a deterministic constant-
         * frequency carrier is not white noise; only deltas are scored. */
        if (eff <= 10.0) check(fabs(cn - eff) < 2.5, "M2M4 follows the effective C/N within 2.5 dB");
        check(cn > prev_cn, "M2M4 rises with C/N");
        check(click < prev_click || prev_click < 0.05, "clicks fall as C/N rises (until none are left)");
        check(origin < prev_origin || prev_origin < 0.05, "origin occupancy falls as C/N rises (until none is left)");
        prev_cn = cn; prev_click = click; prev_origin = origin;
    }
    /* Strong carrier: essentially no clicks. */
    {
        sense_metrics_t m;
        make(buf, N, amp, 0.05);
        sense_measure(buf, N, c5vrx_phase8_gain_lut, &m);
        check(m.click_pm < 0.5, "a clean carrier gives almost no clicks");
        check(m.cn_db > 10.0, "a clean carrier scores high");
        printf("clean cn_db=%.2f click_pm=%.3f lines=%u video_snr_db=%.2f\n", m.cn_db, m.click_pm, m.video_lines, m.video_snr_db);
        check(m.video_lines >= 2 && m.video_snr_db > 10.0, "a clean carrier gives clearly better video than the 15 dB case");
    }
    /* Carrier and noise scaled together (a pure gain change) leave the
     * score within quantisation effects. */
    {
        double a = 0.0, b = 0.0;
        for (int r = 0; r < REPS; ++r) {
            sense_metrics_t m1, m2;
            make(buf, N, 3.2, 1.6);
            sense_measure(buf, N, c5vrx_phase8_gain_lut, &m1);
            make(buf, N, 2.4, 1.2);
            sense_measure(buf, N, c5vrx_phase8_gain_lut, &m2);
            a += m1.cn_db / REPS; b += m2.cn_db / REPS;
        }
        printf("gain_change cn_a=%.2f cn_b=%.2f\n", a, b);
        check(fabs(a - b) < 0.8, "a common gain change keeps the score");
    }
    /* Accumulators. */
    {
        sense_acc_t s = {0}, t = {0};
        for (int k = 0; k < 10; ++k) { sense_acc_add(&s, 1.0 + (k & 1)); sense_acc_add(&t, 2.0 + (k & 1)); }
        double se;
        const double d = sense_delta(&t, &s, &se);
        check(fabs(d - 1.0) < 1e-12, "delta of means");
        check(se > 0.2 && se < 0.3, "standard error of the delta");
    }
    if (failures) { printf("test_sense_metrics: %d failure(s)\n", failures); return 1; }
    printf("test_sense_metrics: OK\n");
    return 0;
}
