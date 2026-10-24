/* C5VRX by Twotoz/contributors: historical U85Q token-centering experiment.
 * Research only; this layout does not apply to the P64 firmware LUT. */
#pragma once
#include "dsp.h"

/* Controller address is I[3:0], Q[3:0], then two middle-sample signs. The archived U85Q tracker
 * uses the endpoint for its angle. Change top-five token bits only;
 * the shared worker's phase state and DAC bits must remain bit-exact. */
static inline void dc_live_center_lut(uint16_t *dst, const uint16_t *base,
                                      int i_mcells, int q_mcells)
{
    if (!i_mcells && !q_mcells) {
        for (unsigned a = 0; a < 1024; ++a) dst[a] = base[a];
        return;
    }
    const double dc_i = i_mcells / 1000., dc_q = q_mcells / 1000.;
    for (unsigned a = 0; a < 1024; ++a) {
        int i = (int)(a & 15u), q = (int)((a >> 4) & 15u);
        if (i > 7) i -= 16;
        if (q > 7) q -= 16;
        const double x = i + .5 - dc_i, y = q + .5 - dc_q;
        const double angle = atan2(y, x) + M_PI;
        unsigned token = (unsigned)floor(angle * 28. / (2. * M_PI)) % 28u;
        if (x * x + y * y < .64)
            token = 28u + (unsigned)floor(angle / (M_PI / 2.)) % 4u;
        dst[a] = (uint16_t)((base[a] & 2047u) | (token << 11));
    }
}
