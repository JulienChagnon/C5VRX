#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: actual T snapshot/age publication.

Compile the production AFC part of control_print; task boundaries are mocked.
Mutation on resume proves all printed evidence comes from the frozen copy.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PREFIX = r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "afc_ctrl.h"
#include "cn_estimate.h"
#define AFC_PORCH_TARGET_KHZ (-1000)
static uint64_t now=200000;
static uint32_t context=1,s_gain_epoch=2;
static bool suspended,mutate;
static char log_line[4096];
static int64_t esp_timer_get_time(void) { assert(suspended); return now; }
static uint32_t afc_context(void) { assert(suspended); return context; }
static void vTaskSuspendAll(void) { assert(!suspended); suspended=true; }
static int xTaskResumeAll(void);
static int record(const char *fmt,...) {
    assert(!suspended); va_list ap; va_start(ap,fmt);
    int n=vsnprintf(log_line,sizeof(log_line),fmt,ap); va_end(ap); return n;
}
#define printf record
'''
SUFFIX = r'''
#undef printf
static int xTaskResumeAll(void) {
    assert(suspended); suspended=false;
    if (mutate) { s_afc_evidence.qualified_us=0; s_afc_evidence.history.oldest_us=199000; }
    return 0;
}
int main(void) {
    control_print(); assert(strstr(log_line,"cn_x10=-990") && strstr(log_line,"qualified_age_ms=-1"));
    s_afc_evidence.context=1; s_afc_evidence.qualified_gain_epoch=2;
    s_afc_evidence.qualified_us=s_afc_evidence.sampled_us=100000;
    s_afc_evidence.raw_porch_khz=-780; s_afc_evidence.residual_khz=220;
    s_afc_evidence.history=(afc2_evidence_t){.n=16,.timestamped=16,.stable=true,.median_khz=210,
        .mad_khz=10,.oldest_us=10000,.newest_us=100000};
    mutate=true; control_print(); mutate=false;
    assert(strstr(log_line,"qualified_age_ms=100 sample_age_ms=100"));
    assert(strstr(log_line,"oldest_age_ms=190 newest_age_ms=100"));
    assert(strstr(log_line,"last_qualified_raw_porch_khz=-780 last_qualified_residual_khz=220"));
    s_afc_evidence.qualified_us=100000; context=2; control_print();
    assert(strstr(log_line,"context_current=0") && strstr(log_line,"qualified_age_ms=-1"));
    context=1; s_gain_epoch=3; control_print();
    assert(strstr(log_line,"qualified_gain_current=0") && strstr(log_line,"qualified_age_ms=-1"));
    s_gain_epoch=2; now=50000; control_print();
    assert(strstr(log_line,"qualified_age_ms=-1 sample_age_ms=-1"));
    assert(!suspended);
    puts("PASS: actual AFC T snapshot survives producer change after resume; ages advance, stale context/gain/future/unknown timestamps refused; no printing under task exclusion");
}
'''


class AfcTelemetryTest(unittest.TestCase):
    def test_actual_print(self):
        source = (ROOT / "main/control.c").read_text(encoding="utf8")
        start = source.index("typedef struct {", source.index("static unsigned s_afc_corrections"))
        end = source.index("/* Learned offset sign", start)
        declarations = source[start:end]
        start = source.index("void control_print(void)")
        end = source.index('    printf("CONTROL channel=', start)
        actual = source[start:end] + "}\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "afc.c"
            path.write_text(PREFIX + declarations + actual + SUFFIX, encoding="utf8")
            binary = Path(directory) / "afc"
            subprocess.run([os.environ.get("CC", "cc"), "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "main"), str(path), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
