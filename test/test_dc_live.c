/* C5VRX by Twotoz/contributors: live-DC evidence gates.
 * Uses the real archived VTX-off captures; not a physical sensitivity test. */
#include "dc_live.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t pack(double x, double y)
{
    int i = (int)floor(x), q = (int)floor(y);
    assert(i >= -8 && i <= 7 && q >= -8 && q <= 7);
    return (uint8_t)(((unsigned)i & 15u) << 4 | ((unsigned)q & 15u));
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    uint8_t raw[8192]; dsp_circle_t c = {0};
    for (unsigned k = 0; k < sizeof(raw); ++k) {
        double phase = 2. * M_PI * k / 997.;
        raw[k] = pack(1.5 + 4. * cos(phase), -.75 + 4. * sin(phase));
    }
    dsp_circle_sums(raw, sizeof(raw), &c);
    int di = 0, dq = 0; unsigned env = 0;
    assert(dc_live_fit(&c, 0, &di, &dq, &env));
    assert(abs(di - 1500) < 100 && abs(dq + 750) < 100 && env >= 300);
    assert(dc_live_measure(&c, 0, &di, &dq, &env));
    assert(!dc_live_fit(&c, 20, NULL, NULL, NULL));
    for (unsigned capture = 0; capture < 4; ++capture) {
        char path[512]; snprintf(path, sizeof(path), "%s/off_%02u.bin", argv[1], capture);
        FILE *f = fopen(path, "rb"); assert(f);
        size_t n = fread(raw, 1, sizeof(raw), f); fclose(f); assert(n == 8190);
        memset(&c, 0, sizeof(c)); dsp_circle_sums(raw, n, &c);
        /* Existing fit falsely accepts these small quantizer clouds. */
        assert(dsp_circle_dc(&c, &di, &dq, &env));
        assert(env >= 300 && !dc_live_fit(&c, 0, &di, &dq, &env));
        assert(!dc_live_measure(&c, 0, &di, &dq, &env));
        assert(di == (int)((double)c.x * 500. / c.n));
        assert(dq == (int)((double)c.y * 500. / c.n));
        assert(env >= 300); /* keep diagnostic: drift must not call it noise */
    }
    memset(&c, 0, sizeof(c));
    for (unsigned k = 0; k < sizeof(raw); ++k) {
        double phase = -.1 + .2 * k / sizeof(raw);
        raw[k] = pack(4. * cos(phase), 4. * sin(phase));
    }
    dsp_circle_sums(raw, sizeof(raw), &c);
    assert(!dc_live_fit(&c, 0, NULL, NULL, NULL));
    puts("PASS: real false-circle captures rejected and qualified center recovered");
    return 0;
}
