/* C5VRX-5 by Twotoz and the C5VRX contributors: LUT16 port regression.
 * Model width-dependent addressing and a read latch refreshed by CFG0. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "kp_lut_io.h"
#include "kp_tables.h"

typedef struct {
    uint16_t ram[1024];
    unsigned selected, writes;
    uint32_t read_latch;
    int fail_entry;
} port_t;

static void select_entry(void *ctx, unsigned entry)
{
    port_t *p = ctx;
    assert(entry < 1024);
    p->selected = entry;
    /* Even-address reads can include their neighbour in the upper half. */
    p->read_latch = p->ram[entry] | ((entry & 1u) ? 0u : (uint32_t)p->ram[entry + 1u] << 16);
}

static void store(void *ctx, uint16_t value)
{
    port_t *p = ctx;
    ++p->writes;
    if ((int)p->selected != p->fail_entry) p->ram[p->selected] = value;
    /* Writing CFG1 does not refresh read_latch. */
}

static uint32_t read_port(void *ctx) { return ((port_t *)ctx)->read_latch; }

static bool write_entry(void *ctx, unsigned entry, uint16_t value)
{
    return kp_lut_write16(ctx, entry, value, select_entry, store, read_port);
}

int main(void)
{
    uint16_t shadow[1024], want[1024];
    port_t port = {.fail_entry = -1};
    uint32_t fail = 0;
    memcpy(shadow, kp_tables[0], sizeof(shadow));
    memcpy(port.ram, shadow, sizeof(shadow));

    /* Reproduce stale verification without reselect; check the actual I/O
     * helper refreshes it and ignores unrelated upper read bits. */
    uint16_t changed = port.ram[0] ^ 0x55;
    select_entry(&port, 0);
    store(&port, changed);
    assert((read_port(&port) & 0xffffu) != changed);
    assert(write_entry(&port, 0, changed));
    assert(port.ram[1] == shadow[1]);
    assert(write_entry(&port, 0, shadow[0]));

    /* Whole-table verification, preserving distinct even/odd neighbours. */
    memcpy(want, shadow, sizeof(want));
    want[1] ^= 0x55;
    want[1023] ^= 0xaa;
    uint32_t writes = 0;
    assert(kp_lut_load(shadow, want, write_entry, &port, &writes, &fail));
    assert(writes == 1024 && !memcmp(shadow, want, sizeof(shadow)));
    assert(!memcmp(port.ram, want, sizeof(shadow)));

    /* A rejected entry must not publish a partially verified target. */
    uint16_t previous[1024];
    memcpy(previous, shadow, sizeof(shadow));
    want[0] ^= 0x11;
    want[7] ^= 0x22;
    port.fail_entry = 7;
    assert(!kp_lut_load(shadow, want, write_entry, &port, &writes, &fail));
    assert(fail == 1 && !memcmp(shadow, previous, sizeof(shadow)));
    port.fail_entry = -1;
    assert(kp_lut_load(shadow, previous, write_entry, &port, &writes, &fail));
    assert(!memcmp(port.ram, previous, sizeof(shadow)));  /* rollback */
    assert(kp_lut_load(shadow, want, write_entry, &port, &writes, &fail));

    /* Exercise all actual ladder tables both ways. */
    for (unsigned pass = 0; pass < 2; ++pass)
        for (unsigned k = 0; k < KP_LEVELS; ++k) {
            unsigned level = pass ? KP_LEVELS - 1 - k : k;
            assert(kp_lut_load(shadow, kp_tables[level], write_entry, &port, &writes, &fail));
            assert(!memcmp(shadow, kp_tables[level], sizeof(shadow)));
            assert(!memcmp(port.ram, kp_tables[level], sizeof(shadow)));
        }
    assert(fail == 1);

    /* Delta load: only differing entries are written, every entry verified. */
    for (unsigned k = 1; k < KP_LEVELS; ++k) {
        memcpy(port.ram, kp_tables[k - 1], sizeof(port.ram));
        uint32_t dw = 0, df = 0;
        unsigned differ = 0;
        for (unsigned i = 0; i < 1024; ++i) differ += kp_tables[k][i] != kp_tables[k - 1][i];
        assert(kp_lut_load_delta(shadow, kp_tables[k], &port, select_entry, store, read_port, &dw, &df));
        assert(dw == differ && df == 0 && !memcmp(port.ram, kp_tables[k], sizeof(shadow)));
        assert(!memcmp(shadow, kp_tables[k], sizeof(shadow)));
    }
    {   /* a stuck entry is caught, and the shadow is not published */
        memcpy(port.ram, kp_tables[0], sizeof(port.ram));
        memcpy(shadow, kp_tables[0], sizeof(shadow));
        uint32_t dw = 0, df = 0;
        unsigned stuck = 0;
        while (kp_tables[1][stuck] == kp_tables[0][stuck]) ++stuck;
        port.fail_entry = (int)stuck;
        assert(!kp_lut_load_delta(shadow, kp_tables[1], &port, select_entry, store, read_port, &dw, &df));
        assert(df == 1 && !memcmp(shadow, kp_tables[0], sizeof(shadow)));
        port.fail_entry = -1;
    }
    /* Single-load path reads all entries and never repairs a corrupt load
     * behind the restart state machine's back. Include the last entry. */
    for (unsigned k = 0; k < KP_LEVELS; ++k) {
        memcpy(port.ram, kp_tables[k], sizeof(port.ram));
        unsigned before = port.writes;
        uint32_t vf = 0;
        assert(kp_lut_verify(kp_tables[k], &port, select_entry, read_port, &vf));
        assert(vf == 0 && port.writes == before);
        port.ram[0] ^= 1;
        port.ram[1023] ^= 1;
        assert(!kp_lut_verify(kp_tables[k], &port, select_entry, read_port, &vf));
        assert(vf == 2 && port.writes == before);
    }
    puts("PASS: LUT16 addressing, refreshed readback, full verification, rollback, all kp tables, delta and single load");
    return 0;
}
