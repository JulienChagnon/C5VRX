/* C5VRX-5 by Twotoz and the C5VRX contributors: verified LUT16 updates.
 * ESP32-C5 TRM: CPU addressing follows the configured LUT width too. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define KP_LUT_ENTRIES 1024u
typedef bool (*kp_lut_write_fn)(void *ctx, unsigned entry, uint16_t value);
typedef void (*kp_lut_select_fn)(void *ctx, unsigned entry);
typedef void (*kp_lut_data_fn)(void *ctx, uint16_t value);
typedef uint32_t (*kp_lut_read_fn)(void *ctx);

/* A prepared program image already loaded the target table. Read every
 * entry without a second write; a mismatch must enter verified rollback. */
static inline bool kp_lut_verify(const uint16_t *want, void *ctx,
                                kp_lut_select_fn select_entry, kp_lut_read_fn read,
                                uint32_t *verify_fail)
{
    bool verified = true;
    for (unsigned entry = 0; entry < KP_LUT_ENTRIES; ++entry) {
        select_entry(ctx, entry);
        if ((read(ctx) & 0xffffu) != want[entry]) {
            ++*verify_fail;
            verified = false;
        }
    }
    return verified;
}

/* Selection refreshes the CPU read latch. After storing CFG1, select CFG0
 * again before verifying; the upper 16 read bits are outside LUT16. MMIO
 * callbacks supply ordering barriers around their register accesses. */
static inline bool kp_lut_write16(void *ctx, unsigned entry, uint16_t value,
                                  kp_lut_select_fn select_entry,
                                  kp_lut_data_fn store, kp_lut_read_fn read)
{
    select_entry(ctx, entry);
    store(ctx, value);
    select_entry(ctx, entry);
    return (read(ctx) & 0xffffu) == value;
}

/* The same full verification, but an entry is written only when the RAM
 * does not already hold it (the program load wrote its own table; adjacent
 * kp levels differ in ~1/3 of the entries). Each entry: select, read; on a
 * difference store, select, read again. The halted LUT stage was 2.7-2.9 ms
 * of the ~5.5 ms restart gap with select-store-select-read for all 1024
 * (board, range audit PR #230). Caller owns a halted core. */
static inline bool kp_lut_load_delta(uint16_t *shadow, const uint16_t *want, void *ctx,
                                     kp_lut_select_fn select_entry, kp_lut_data_fn store, kp_lut_read_fn read,
                                     uint32_t *writes, uint32_t *verify_fail)
{
    bool verified = true;
    for (unsigned entry = 0; entry < KP_LUT_ENTRIES; ++entry) {
        select_entry(ctx, entry);
        if ((read(ctx) & 0xffffu) == want[entry]) continue;
        ++*writes;
        store(ctx, want[entry]);
        select_entry(ctx, entry);
        if ((read(ctx) & 0xffffu) != want[entry]) {
            ++*verify_fail;
            verified = false;
        }
    }
    if (verified && shadow != want) memcpy(shadow, want, KP_LUT_ENTRIES * sizeof(*shadow));
    return verified;
}

/* Caller owns a halted core. Verify the entire candidate before publishing
 * its shadow. Failure leaves the last verified shadow intact for rollback. */
static inline bool kp_lut_load(uint16_t *shadow, const uint16_t *want,
                               kp_lut_write_fn write_entry, void *ctx,
                               uint32_t *writes, uint32_t *verify_fail)
{
    bool verified = true;
    for (unsigned entry = 0; entry < KP_LUT_ENTRIES; ++entry) {
        ++*writes;
        if (!write_entry(ctx, entry, want[entry])) {
            ++*verify_fail;
            verified = false;
        }
    }
    if (verified && shadow != want) memcpy(shadow, want, KP_LUT_ENTRIES * sizeof(*shadow));
    return verified;
}
