/* C5VRX by Twotoz and contributors: issue #200 manual DAC6 scaling.
 * 1x is bit-exact. High gains can clip; output impedance is not calibrated.
 * No phase/token bits may change when scaling DAC output. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define OUTPUT_LEVELS 10
static inline uint16_t output_entry(uint16_t value, unsigned level, bool *clipped)
{
    /* Keep the existing 1.0..2.5x maps; extend their half-step contrast
     * curve through 5.0x, plus 0.5x half-scale. Saturation is always limited to DAC6 bits. */
    static const int pivot[OUTPUT_LEVELS] = {21, 13, 16, 19, 22, 25, 28, 31, 34, 21};
    static const int scale[OUTPUT_LEVELS] = { 2,  3,  4,  5,  6,  7,  8,  9, 10,  1};
    if (level >= OUTPUT_LEVELS) level = 0;
    int twice = 2 * pivot[level] + scale[level] * ((int)(value & 63u) - 21);
    /* Round to nearest code, including negative values before saturation. */
    int code = twice >= 0 ? (twice + 1) / 2 : twice / 2;
    if (clipped) *clipped = code < 0 || code > 63;
    if (code < 0) code = 0;
    if (code > 63) code = 63;
    return (uint16_t)((value & 0xffc0u) | (unsigned)code);
}
