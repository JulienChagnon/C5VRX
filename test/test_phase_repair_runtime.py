#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: execute production live phase repair gates."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sphase_logic.h"
#define RF_LANE_ULTRAFINE 2
static bool s_startup, s_boot_phase_pending, owner = true;
static unsigned s_health_blocks, s_health_rejected, calls;
static int s_sphase_excess;
static const char *s_sphase_state;
static int64_t now;
static sphase_verdict_t result = SPHASE_SETTLED;
static bool quiet_owner(void) { return owner; }
static int rf_lane(void) { return 2; }
static int64_t esp_timer_get_time(void) { return now; }
static sphase_verdict_t sphase_scan(void) {
    ++calls;
    if (result == SPHASE_SETTLED) s_sphase_excess = 50;
    return result;
}
'''
SUFFIX = r'''
static void block(int excess) {
    now += 250000;
    ++s_health_blocks;
    s_sphase_excess = excess;
    phase_service();
}
int main(void) {
    s_startup = true;
    for (int i=0;i<4;++i) block(4000);
    assert(calls == 0);
    s_startup = false; s_boot_phase_pending = true;
    block(4000); assert(calls == 0);
    s_boot_phase_pending = false; owner = false;
    block(4000); assert(calls == 0);
    owner = true;
    block(4000); phase_service(); phase_service(); assert(calls == 0);
    block(50); /* healthy evidence breaks the streak */
    block(4000); block(4000); assert(calls == 0);
    block(4000); assert(calls == 1 && s_phase_repairs == 1);
    assert(!strcmp(s_sphase_state, "settled") && s_sphase_excess == 50);
    for (int i=0;i<39;++i) block(4000);
    assert(calls == 1); /* successful repairs share a ten-second cooldown */
    result = SPHASE_UNSETTLED;
    block(4000); assert(calls == 2 && s_phase_repair_fails == 1);
    assert(!strcmp(s_sphase_state, "unverified"));
    for (int i=0;i<119;++i) block(4000);
    assert(calls == 2);
    block(4000); assert(calls == 3 && s_phase_repair_fails == 2);
    for (int i=0;i<239;++i) block(4000);
    assert(calls == 3);
    result = SPHASE_SETTLED;
    block(4000); assert(calls == 4 && s_phase_repair_fails == 2);
    puts("PASS: production phase repair ownership, independent blocks, cooldown and failure backoff");
}
'''


class PhaseRepairTest(unittest.TestCase):
    def test_production_repair(self):
        source = (ROOT / 'main/calibration.c').read_text()
        begin = source.index('#define PHASE_REPAIR_EXCESS')
        end = source.index('/* VTX deviation', begin)
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / 'repair.c'
            binary = Path(directory) / 'repair'
            c.write_text(PREFIX + source[begin:end] + SUFFIX)
            subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall',
                            '-Wextra', '-Werror', '-I', str(ROOT / 'main'),
                            str(c), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
