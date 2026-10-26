/* C5VRX by Twotoz and contributors: exact shared filter-measurement math.
 * No prior DC calibration is required; failed copies contribute no samples. */
#pragma once
#include <string.h>
#include "dsp.h"
typedef struct {
    float psd[DSP_FFT_N];
    unsigned got, env_sum;
    int clip_sum;
} cal_noise_t;
static inline void cal_noise_add(cal_noise_t *a, const uint8_t *s, size_t n, int clip_pm)
{
    if (!s || n != 256u) return;
    for (unsigned i = 0; i < 4u; ++i) dsp_psd_accumulate(s + i * DSP_FFT_N, a->psd);
    a->env_sum += dsp_envelope_ratio_x100(s, n);
    a->clip_sum += clip_pm;
    ++a->got;
}
static inline bool cal_noise_finish(const cal_noise_t *a, unsigned *width, unsigned *nbw,
                                    unsigned *env, int *clip)
{
    if (a->got < 48u) return false;
    *width = dsp_psd_width_khz(a->psd);
    *nbw = dsp_psd_nbw_khz(a->psd);
    *env = a->env_sum / a->got;
    *clip = a->clip_sum / (int)a->got;
    return true;
}
/* Screening never supplies a stored calibration. Keep a wide guard (four
 * FFT bins in width, 25% NBW) around a fully measured incumbent. No assumed
 * monotonicity of the undocumented filter controls: visit every setting. */
static inline bool cal_filter_contender(unsigned width, unsigned nbw, unsigned minimum, unsigned best_nbw)
{
    return width + 2500u >= minimum && (!best_nbw || (uint64_t)nbw * 4u <= (uint64_t)best_nbw * 5u);
}
static inline bool cal_filter_screen_agrees(unsigned w0, unsigned n0, unsigned w1, unsigned n1)
{
    unsigned dw = w0 > w1 ? w0 - w1 : w1 - w0;
    unsigned dn = n0 > n1 ? n0 - n1 : n1 - n0;
    return n0 && n1 && dw <= 2500u && (uint64_t)dn * 4u <= n1;
}
static inline const char *cal_channel_quiet(const uint8_t *s, unsigned windows, bool recent_sync,
                                          unsigned *env, int *clip)
{
    if (recent_sync) return "video_sync";
    if (windows < 16u) return "probe_failed";
    return dsp_quiet_reason(s, windows * 256u, env, clip);
}
static inline bool cal_setting_quiet(unsigned env, int clip) { return env <= dsp_quiet_limit(clip); }
static inline bool cal_setting_carrier(unsigned env, int clip)
{
    return env > dsp_quiet_limit(clip) + DSP_QUIET_CARRIER_X100;
}
/* DC batch gate: whether a quiet-test result means a carrier. Low gain rows
 * hold receiver noise of only a few quantizer steps; quantization lifts its
 * envelope statistic above the quiet limit (board 2026-10-26, VTX off and
 * video output proven not to raise the floor: G69 env 1.22, F stopped at
 * G68..G73 as carrier_detected) or puts it in the centre cells ("origin").
 * The row search measures such rows itself (flat when below one step), so
 * as in the filter sweep only a definite carrier stops the batch. */
static inline bool cal_dc_gate_carrier(const char *why, unsigned env, int clip)
{
    if (!why || !strcmp(why, "origin")) return false;
    if (!strcmp(why, "envelope")) return cal_setting_carrier(env, clip);
    return true;
}
