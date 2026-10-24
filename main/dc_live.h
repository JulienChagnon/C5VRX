/* C5VRX by Twotoz/contributors: qualified circle evidence and DC-aware tokens.
 * Extends dsp_circle_dc and the archived U85Q/kp_morph observation law.
 * Carrier-fit qualification is used by calibration collect(). The unused
 * U85Q token-centering experiment lives in research/legacy_demod/dc_center_lut.h;
 * production DC measurement does not rewrite the P64 LUT. */
#pragma once
#include "dsp.h"

/* A few quantizer cells can fit a perfect tiny circle with env=9999 while
 * the VTX is off. Envelope evidence alone is not an identifiable DC centre.
 * Require variance in both independent directions and a reasonably balanced
 * cloud before accepting the existing production circle fit. */
static inline bool dc_live_fit(const dsp_circle_t *c, unsigned rail_pm,
                               int *i_mcells, int *q_mcells, unsigned *env_x100)
{
    if (!c || c->n < 1024 || rail_pm >= 20) return false;
    const double n = (double)c->n, mx = c->x / n, my = c->y / n;
    const double xx = c->xx / n - mx * mx, yy = c->yy / n - my * my;
    const double xy = c->xy / n - mx * my;
    const double spread = sqrt((xx - yy) * (xx - yy) + 4 * xy * xy);
    const double low = .5 * (xx + yy - spread), high = .5 * (xx + yy + spread);
    /* dsp coordinates are twice a cell; low >=4 means >=1 cell^2. */
    if (!(low >= 4. && low >= .2 * high)) return false;
    int i, q; unsigned env;
    if (!dsp_circle_dc(c, &i, &q, &env) || env < 300) return false;
    if (i_mcells) *i_mcells = i;
    if (q_mcells) *q_mcells = q;
    if (env_x100) *env_x100 = env;
    return true;
}

/* Receiver noise, or a weak FM carrier rotating in it, has the DC as its
 * mean: noise is zero-mean and the porch (-1 MHz) and sync tip rotate. This
 * is the estimate where no carrier circle can be fitted (no VTX, and the
 * weak carrier at the range edge, exactly where DC costs most). Same
 * conditions as the fit (dsp coordinates are twice a cell): >= 1 cell^2 of
 * dither in both directions, a balanced cloud, no rails; a fixed point or
 * a tiny cloud never qualifies. Board 2026-10-19: the circle-only trim
 * never ran without a strong carrier (DC_LIVE trials=0) while the DC
 * drifted to 3-4 cells after channel setups. */
static inline bool dc_live_noise_mean(const dsp_circle_t *c, unsigned rail_pm, int *i_mcells, int *q_mcells)
{
    if (!c || c->n < 1024 || rail_pm >= 20) return false;
    const double n = (double)c->n, mx = c->x / n, my = c->y / n;
    const double xx = c->xx / n - mx * mx, yy = c->yy / n - my * my;
    const double xy = c->xy / n - mx * my;
    const double spread = sqrt((xx - yy) * (xx - yy) + 4 * xy * xy);
    const double low = .5 * (xx + yy - spread), high = .5 * (xx + yy + spread);
    if (!(low >= 4. && low >= .2 * high)) return false;
    *i_mcells = (int)(mx * 500.);
    *q_mcells = (int)(my * 500.);
    return true;
}

/* Preserve the established noise mean and envelope diagnostic. A rejected
 * tiny/partial circle must not publish its fitted centre as receiver DC. */
static inline bool dc_live_measure(const dsp_circle_t *c, unsigned rail_pm,
    int *i_mcells, int *q_mcells, unsigned *env_x100)
{
    bool circle = dsp_circle_dc(c, i_mcells, q_mcells, env_x100);
    if (!circle) return false;
    if (dc_live_fit(c, rail_pm, i_mcells, q_mcells, env_x100)) return true;
    *i_mcells = (int)((double)c->x * 500. / (double)c->n);
    *q_mcells = (int)((double)c->y * 500. / (double)c->n);
    return false;
}
