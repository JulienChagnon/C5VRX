#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: actual core + receiver + NVS map path.

Extract production receiver bind/export and calibration persistence functions;
mock only task/hardware/NVS boundaries, link actual gain.c and arc_phy.c.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + "\n"


PREFIX = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "gain.h"
#include "gain_map_preset.h"
#include "cn_observation.h"
unsigned char phy_param[2048];
static direct_gain_t s_gain;
static uint8_t s_current_gain=59;
static dg_map_blob_t s_gain_map_saved, disk;
static bool s_gain_map_saved_valid, s_map_bound, active=true;
static uint32_t s_control_epoch=1, s_map_control, s_map_phy, s_map_table;
static uint32_t s_gain_epoch;
static int64_t s_last_gain_write_us;
static uint32_t s_map_imports, s_preset_imports, phy=2, table_epoch=3;
static arc_gain_table_t actual_table;
static dg_map_context_t live;
static unsigned suspended, suspends, resumes, stores, filter_reads;
static bool mutate;
static bool other_actuator, actuator;
static int64_t now=31000000;
typedef struct { int8_t code; uint8_t skirt; bool bw20; } rf_filter_t;
static rf_filter_t rf_filter(void) {
    if (mutate && ++filter_reads==2) ++live.filter_skirt;
    return (rf_filter_t){live.filter_code,live.filter_skirt,live.filter_bw20};
}
static uint16_t rf_frequency_mhz(void) { return live.freq_mhz; }
static unsigned rf_lane(void) { return live.lane_mode; }
static int rf_offset_khz(void) { return live.offset_khz; }
static uint32_t phy_generation(void) { return phy; }
static bool phy_try_actuator(uint32_t expected) {
    assert(!suspended); if (other_actuator || expected!=phy) return false;
    assert(!actuator); actuator=true; return true;
}
static void phy_end_actuator(void) { assert(suspended && actuator); actuator=false; }
static uint32_t rf_table_generation(void) { return table_epoch; }
static const arc_gain_table_t *rf_gain_table(void) { return &actual_table; }
static bool receiver_active(void) { return active; }
static void vTaskSuspendAll(void) { assert(!suspended); suspended=1; ++suspends; }
static int xTaskResumeAll(void) { assert(suspended); suspended=0; ++resumes; return 0; }
static int64_t esp_timer_get_time(void) { return now; }
static bool blob_store(const char *key, const void *blob, size_t bytes) {
    assert(!suspended && !strcmp(key,"gain_map") && bytes==sizeof(disk));
    disk=*(const dg_map_blob_t *)blob; ++stores; return true;
}
'''
SUFFIX = r'''
static void bind(void) { dg_map_context_t c=map_context(); gain_map_bind(&c,s_control_epoch,phy,table_epoch); }
static void learn(void) {
    s_gain.relative_power_q10[59]=1024; s_gain.relative_power_q10[60]=1500;
    s_gain.confidence[59]=7; s_gain.confidence[60]=9;
    s_gain.uncertainty_pm[59]=80; s_gain.uncertainty_pm[60]=123;
    s_gain.learned=12; s_gain.state=DG_HOLD;
}
static void assert_retained(void) {
    assert(s_gain.learned && s_gain.confidence[60]==9 && s_gain.relative_power_q10[60]==1500 && s_gain.uncertainty_pm[60]==123);
}
int main(void) {
    arc_gain_table_from_bytes(&actual_table,NULL,ARC_5G_GAIN_MAX);
    live=(dg_map_context_t){.freq_mhz=5865,.lane_mode=2,.iq_scale=4,.filter_skirt=32,.filter_bw20=1,.filter_code=8};
    phy_param[650]=4;
    const dg_map_context_t a=live; const arc_gain_table_t original=actual_table;
    bind(); learn(); dg_map_blob_t saved_a, out;
    assert(receiver_export_gain_map(&saved_a)==2 && saved_a.version==DG_MAP_VERSION && saved_a.context_valid);
    assert(saved_a.freq_mhz==5865 && saved_a.context.iq_scale==4);
    bind(); assert_retained(); /* tracking reset, identical transfer context */
    dg_reset(&s_gain,&actual_table,s_current_gain); assert_retained();
    ++s_control_epoch; ++phy; ++table_epoch; bind(); assert_retained();
    assert(receiver_export_gain_map(&out)==0); /* reset is ACQUIRE */
    s_gain.state=DG_HOLD; assert(receiver_export_gain_map(&out)==2);
    s_gain_map_saved=saved_a; s_gain_map_saved_valid=true;
    /* Every transfer field independently invalidates and refuses import. */
    for (unsigned field=0;field<7;++field) {
        live=a; phy_param[650]=a.iq_scale;
        switch(field) {
        case 0: live.freq_mhz=5805; break; case 1: live.offset_khz=-180; break;
        case 2: live.filter_code=9; break; case 3: live.filter_skirt=31; break;
        case 4: live.filter_bw20=0; break; case 5: live.lane_mode=1; break;
        case 6: phy_param[650]=5; break;
        }
        assert(receiver_export_gain_map(&out)==0);
        bind(); assert(!s_gain.learned && s_gain.confidence[60]==0);
        assert(!dg_import_map(&s_gain,&saved_a));
        live=a; phy_param[650]=a.iq_scale; bind(); assert_retained(); /* A -> B -> A imports A only */
    }
    /* Exact table identity, including old omitted start/RF/band fields. */
    for (unsigned field=0;field<5;++field) {
        actual_table=original;
        switch(field) {
        case 0: ++actual_table.spans[0]; break; case 1: ++actual_table.start[0]; break;
        case 2: ++actual_table.rf_code[8]; break; case 3: --actual_table.max_index; break;
        case 4: actual_table.band5=false; break;
        }
        assert(receiver_export_gain_map(&out)==0); bind(); assert(!s_gain.learned);
        assert(!dg_import_map(&s_gain,&saved_a));
        actual_table=original; bind(); assert_retained();
    }
    /* Old metadata, wrong duplicated labels, unset context all rejected. */
    dg_map_blob_t bad=saved_a; bad.version=4; assert(!dg_import_map(&s_gain,&bad));
    bad=saved_a; bad.version=5; assert(!dg_import_map(&s_gain,&bad)); /* before digital mode 0 */
    bad=saved_a; bad.context_valid=0; assert(!dg_import_map(&s_gain,&bad));
    bad=saved_a; bad.freq_mhz=5805; assert(!dg_import_map(&s_gain,&bad));
    bad=saved_a; bad.lane_mode=1; assert(!dg_import_map(&s_gain,&bad));
    /* Export excludes observer and rejects unknown/inactive/changed model. */
    s_gain.state=DG_HOLD; active=false; assert(!receiver_export_gain_map(&out)); active=true;
    other_actuator=true; assert(!receiver_export_gain_map(&out)); other_actuator=false;
    s_last_gain_write_us=now-499; assert(!receiver_export_gain_map(&out));
    s_last_gain_write_us=now+1; assert(!receiver_export_gain_map(&out)); s_last_gain_write_us=0;
    ++s_control_epoch; assert(!receiver_export_gain_map(&out)); bind(); s_gain.state=DG_HOLD;
    mutate=true; filter_reads=0; assert(!receiver_export_gain_map(&out)); mutate=false;
    live=a; bind(); s_gain.state=DG_HOLD;
    assert(receiver_export_gain_map(&out)==2 && !memcmp(&out,&saved_a,sizeof(out)));
    /* Saver never stamps current RF onto an old model, never writes NVS
     * under scheduler exclusion, and skips unchanged blobs. */
    s_gain_map_saved_valid=false; gain_map_service(); assert(stores==1 && disk.context.freq_mhz==5865);
    now+=31000000; gain_map_service(); assert(stores==1);
    live.freq_mhz=5805; now+=31000000; gain_map_service(); assert(stores==1);
    bind(); learn(); gain_map_service(); assert(stores==2 && disk.context.freq_mhz==5805 && disk.freq_mhz==5805);
    live=a; bind(); assert(!s_gain.learned); /* only B cached now, cannot import it as A */
    assert(suspends==resumes && !suspended);
    printf("INFO: host ABI map blob=%zu bytes; global controller=%zu bytes (not C5 WCET/stack proof)\n", sizeof(dg_map_blob_t),sizeof(direct_gain_t));
    puts("PASS: actual gain reset/import + receiver context/export + NVS saver: same context retained; all transfer/table changes rejected; A-B-A; v4 ignored; coherent publication");
}
'''


class GainMapContextTest(unittest.TestCase):
    def test_production_paths(self):
        runtime = (ROOT / "main/receiver.c").read_text(encoding="utf8")
        calibration = (ROOT / "main/calibration.c").read_text(encoding="utf8")
        snippets = "".join(function(runtime, name) for name in
                           ["static dg_map_context_t map_context(void)",
                            "unsigned receiver_export_gain_map(dg_map_blob_t *blob)",
                            "static void gain_map_bind("])
        snippets += function(calibration, "static void gain_map_service(void)")
        self.assertIn("gain_map_bind(&context, control, phy, table)", runtime)
        self.assertIn("!dg_map_matches(&s_gain, rf_gain_table(), &context)", runtime)
        self.assertNotIn('s_gain_map_saved.freq_mhz == rf_frequency_mhz()', runtime)
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "test.c"
            source.write_text(PREFIX + snippets + SUFFIX, encoding="utf8")
            binary = Path(folder) / "test"
            subprocess.run([os.environ.get("CC", "cc"), "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "main"), str(source), str(ROOT / "main/gain.c"),
                            str(ROOT / "main/arc_phy.c"), "-o", str(binary), "-lm"], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
