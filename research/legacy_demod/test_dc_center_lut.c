/* C5VRX by Twotoz/contributors: historical U85Q token-layout regression. */
#include "dc_center_lut.h"
#include "kp_tables.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    uint16_t shifted[1024];
    const int offsets[][2] = {{0, 0}, {1500, -750}, {-500, 1250}, {4000, -2000}};
    for (unsigned kp = 0; kp < KP_LEVELS; ++kp) {
        for (unsigned d = 0; d < sizeof(offsets) / sizeof(offsets[0]); ++d) {
            dc_live_center_lut(shifted, kp_tables[kp], offsets[d][0], offsets[d][1]);
            for (unsigned a = 0; a < 1024; ++a) {
                assert((shifted[a] & 2047u) == (kp_tables[kp][a] & 2047u));
                assert((shifted[a] >> 11) == (shifted[a & 255u] >> 11));
            }
            if (d == 0) assert(!memcmp(shifted, kp_tables[kp], sizeof(shifted)));
        }
    }
    puts("PASS: archived U85Q kp worker bits and zero-DC tables preserved");
    return 0;
}
