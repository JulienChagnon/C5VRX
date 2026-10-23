/* C5VRX-5 by Twotoz and the C5VRX contributors: verified LUT16 updates.
 * ESP32-C5 TRM: CPU addressing follows the configured LUT width too. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define TRANSPORT_LUT_ENTRIES 1024u
typedef void (*transport_lut_select_fn)(void *ctx, unsigned entry);
typedef uint32_t (*transport_lut_read_fn)(void *ctx);

/* A prepared program image already loaded the target table. Read every
 * entry without a second write; a mismatch must enter verified rollback. */
static inline bool transport_lut_verify(const uint16_t *want, void *ctx,
                                transport_lut_select_fn select_entry, transport_lut_read_fn read,
                                uint32_t *verify_fail)
{
    bool verified = true;
    for (unsigned entry = 0; entry < TRANSPORT_LUT_ENTRIES; ++entry) {
        select_entry(ctx, entry);
        if ((read(ctx) & 0xffffu) != want[entry]) {
            ++*verify_fail;
            verified = false;
        }
    }
    return verified;
}

