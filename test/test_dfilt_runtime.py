#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: execute production DFILT and RF restores.

Only the SDK, vendor helpers and MMIO boundary are mocked. Test both pinned
and unsupported builds, including failed readback and nested ownership.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + "\n"


SDK = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portMAX_DELAY 100
#define pdTRUE 1
typedef int StaticSemaphore_t;
typedef void *SemaphoreHandle_t;
static unsigned locks, hooks, writes;
static void *xSemaphoreCreateRecursiveMutexStatic(int *p){return p;}
static int xSemaphoreTakeRecursive(void *p,int timeout){(void)timeout;assert(p);++locks;return 1;}
static int xSemaphoreGiveRecursive(void *p){assert(p&&locks);--locks;return 1;}
#ifdef C5VRX_PHY_PINNED
static unsigned warnings, dc_hooks;
static bool fail_write, notified;
static uint32_t reg_word = 0xa5a5a5a5u, adc_word = 3u;
static uint32_t mmio_read(uint32_t address) {
    assert(address==0x600a0430u && locks);return reg_word;
}
#define REG_READ(address) mmio_read(address)
#endif
'''
VENDOR = r'''
static void vendor_mode(unsigned mode) {
    reg_word=(reg_word & ~DFILT_MASK) | (mode << DFILT_SHIFT);
}
void phy_rx_filter_mode(uint32_t mode) {
    assert(locks && phy_busy() && notified && (mode==0 || mode==4 || mode==8 || mode==12));
    ++writes; if(!fail_write) vendor_mode(mode);
}
static void invalidate(void){assert(locks);++hooks;notified=true;}
static unsigned s_gain=52;
typedef struct {bool bw20;int code;unsigned skirt;} rf_filter_t;
static rf_filter_t s_filter;
static bool s_setup_valid=true, s_setup_bw20;
static uint16_t s_freq=5865;
static int s_offset_khz;
typedef int esp_err_t;
#define ESP_OK 0
#define RF_OFFSET_MAX_KHZ 2500
#define ESP_ERR_NOT_SUPPORTED 1
static esp_err_t phy_11p_apply_default(void){assert(phy_busy());return ESP_OK;}
static int phy_11p_hw_enabled(void){return 1;}
static esp_err_t channel_setup(uint16_t mhz) {
    assert(phy_busy());vendor_mode(mhz>5830 ? 8 : 0);
    s_setup_bw20=s_filter.bw20;return ESP_OK;
}
static void capture_table(void){assert(phy_busy());}
static void phy_chip_set_chan_offset(int khz){assert(phy_busy());(void)khz;vendor_mode(8);}
bool rf_filter_applied(void);
static void post_gain(unsigned gain){assert(gain==s_gain && phy_busy());++dc_hooks;}
static void (*s_post_gain_hook)(unsigned)=post_gain;
static bool phy_dco_release(void){assert(phy_busy());return true;}
static void enable_continuous_modem(void){assert(phy_busy());}
static int lock_rx_only(void){assert(phy_busy());return 0;}
static void phy_disable_agc(void){assert(phy_busy());}
static void phy_rfagc_disable(void){assert(phy_busy());}
static void phy_wifi_fbw_sel(unsigned mode){assert(phy_busy());vendor_mode(mode ? 8 : 4);}
static void phy_force_rx_gain(bool force,unsigned gain){assert(force && gain==s_gain && phy_busy());}
static void apply_filter(void){assert(phy_busy());}
static int phy_filter_code(void){return s_filter.code;}
static int phy_filter_skirt(void){return (int)s_filter.skirt;}
#define ESP_ERROR_CHECK(expr) assert((expr)==0)
#define ESP_EARLY_LOGW(...) (++warnings)
'''
PINNED = r'''
int main(void) {
    phy_set_observation_hook(invalidate);
    for(unsigned mode=0;mode<16;++mode) {
        vendor_mode(mode); const uint32_t unrelated=reg_word & ~DFILT_MASK;
        const unsigned gen=phy_generation(), before=hooks;
        assert(phy_dfilt_hw_mode()==(int)mode && !locks && !phy_busy());
        notified=false;assert(phy_dfilt_apply_default());
        assert(phy_dfilt_hw_mode()==PHY_DFILT_MODE_DEFAULT && (reg_word & ~DFILT_MASK)==unrelated);
        assert(adc_word==3 && hooks==before+1 && phy_generation()==gen+1 && !locks);
    }
    for(unsigned width=0;width<2;++width) {
        s_filter.bw20=(bool)width;vendor_mode(8);
        const unsigned gen=phy_generation(), before=hooks;
        phy_begin();restore_lock();
        assert(phy_busy() && phy_generation()==gen && hooks==before+1);
        assert(phy_dfilt_hw_mode()==PHY_DFILT_MODE_DEFAULT && rf_filter_applied());
        phy_end();assert(!locks && phy_generation()==gen+1);
    }
    /* A dropped write must never be reported as applied. */
    fail_write=true;vendor_mode(8);
    assert(!phy_dfilt_apply_default() && !rf_filter_applied() && !locks);
    phy_begin();restore_lock();phy_end();
    assert(warnings==1 && !rf_filter_applied() && dc_hooks==3 && !locks);
    /* Low-bit corruption is also a mismatch, even with selector bits zero. */
    vendor_mode(1);assert(!rf_filter_applied());
    fail_write=false;assert(phy_dfilt_apply_default() && rf_filter_applied());
    assert(writes==21 && adc_word==3);
    for(unsigned width=0;width<2;++width) {
        rf_filter_t f={(bool)width,16,8};
        assert(rf_set_filter(f) && rf_filter_applied() && !locks);
        rf_set_offset_khz(250*(int)(width+1));
        assert(phy_dfilt_hw_mode()==PHY_DFILT_MODE_DEFAULT && !locks && !phy_busy());
    }
    fail_write=true;
    assert(!rf_set_filter(s_filter) && !rf_filter_applied() && !locks);
    /* Menu PHY FILT: 0/4/8/12 only, kept through every RF restore; a failed
     * readback keeps the previous mode; never another value. */
    fail_write=false;
    assert(phy_dfilt_next(12)==0 && phy_dfilt_next(0)==4 && phy_dfilt_next(4)==8 && phy_dfilt_next(8)==12);
    for(int m=0;m<=12;m+=4) {
        assert(phy_dfilt_select(m) && phy_dfilt_mode()==m && phy_dfilt_hw_mode()==m && !locks);
        vendor_mode(8);phy_begin();restore_lock();phy_end();
        assert(phy_dfilt_hw_mode()==m && rf_filter_applied() && !locks);
    }
    assert(!phy_dfilt_select(5) && !phy_dfilt_select(16) && phy_dfilt_mode()==12);
    assert(phy_dfilt_select(4));fail_write=true;
    assert(!phy_dfilt_select(0) && phy_dfilt_mode()==4);
    fail_write=false;assert(phy_dfilt_select(PHY_DFILT_MODE_DEFAULT) && !locks);
    puts("PASS: production DFILT readback, restore, failure and ownership");
}
'''
UNSUPPORTED = r'''
int main(void) {
    assert(!phy_dfilt_apply_default() && phy_dfilt_hw_mode()==-1);
    assert(!locks && !hooks && !writes && !phy_generation());
    puts("PASS: unsupported PHY refuses DFILT without MMIO/vendor calls");
}
'''
GAIN_PREFIX = r'''
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "gain.h"
#include "cn_auto.h"
unsigned char phy_param[2048];
static direct_gain_t s_gain;
static bool s_cna_gain_dirty;
static bool blob_load(const char *,void *,size_t);
static bool blob_store(const char *,const void *,size_t);
'''
GAIN_HARNESS = r'''
static gtab_blob_t disk;
static unsigned stores;
static bool blob_load(const char *key,void *data,size_t size) {
    assert(!strcmp(key,"gtab") && size==sizeof(disk));memcpy(data,&disk,size);return true;
}
static bool blob_store(const char *key,const void *data,size_t size) {
    assert(!strcmp(key,"gtab") && size==sizeof(disk));memcpy(&disk,data,size);++stores;return true;
}
int main(void) {
    arc_gain_table_t table;
    arc_gain_table_from_bytes(&table,NULL,ARC_5G_GAIN_MAX);
    dg_reset(&s_gain,&table,83);
    const int16_t old_model=s_gain.model_db_q8[82];
    disk=(gtab_blob_t){.version=GTAB_VERSION-1u,.rows=2};
    disk.th_x256[83]=1024;disk.th_x256[82]=512;
    /* VTX-on boot: no fresh noise pass; a table of the previous RF context
     * (11p with digital mode 0) must be ignored. */
    cna_apply_gain();
    assert(!s_gain_rows_measured && s_gain.model_db_q8[82]==old_model);
    assert(s_gain_th[83]==0 && s_gain_th[82]==0 && !stores);
    /* A table of the current context remains usable on the next VTX-on boot. */
    disk.version=GTAB_VERSION;s_gain_th_loaded=false;
    cna_apply_gain();assert(s_gain_rows_measured==2 && s_gain_th[83]==4);
    /* A clean startup pass replaces an old blob and publishes new data. */
    disk.version=GTAB_VERSION-1u;
    const float noise[DG_STATES]={[83]=3.0f+CNA_QUANT_CELLS2,[82]=1.0f+CNA_QUANT_CELLS2};
    receiver_gain_table_measured(noise);
    assert(stores==1 && disk.version==GTAB_VERSION && s_gain_th_new && s_cna_gain_dirty);
    cna_apply_gain();assert(s_gain_rows_measured==2 && s_gain_th[83]==3 && !s_gain_th_new);
    receiver_gain_table_measured(noise);assert(stores==1); /* unchanged data: no NVS wear */
    puts("PASS: production gtab rejects previous-context data on VTX-on boot; fresh save/load retained");
}
'''


class DigitalFilter(unittest.TestCase):
    def test_noise_derived_gain_migration(self):
        receiver = (ROOT / "main/receiver.c").read_text()
        production = receiver[receiver.index("#define GTAB_VERSION"):receiver.index("static void cna_probe(")]
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source, exe = base / "gain_migration.c", base / "gain_migration"
            source.write_text(GAIN_PREFIX + production + GAIN_HARNESS)
            subprocess.run(["cc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "main"), str(source), str(ROOT / "main/gain.c"),
                            str(ROOT / "main/arc_phy.c"), "-lm", "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_production(self):
        phy = (ROOT / "main/phy.c").read_text()
        ownership = phy.split("#ifdef C5VRX_PHY_PINNED", 1)[0]
        ownership = re.sub(r"^#include[^\n]*\n", "", ownership, flags=re.MULTILINE)
        digital = phy[phy.index("/* ---- Digital RX filter"):phy.index("/* ---- 11p receive baseline")]
        header = (ROOT / "main/phy.h").read_text()
        default = re.search(r"^#define PHY_DFILT_MODE_DEFAULT .+$", header, re.MULTILINE)[0]
        rf = (ROOT / "main/rf.c").read_text()
        for pinned in (True, False):
            with self.subTest(pinned=pinned), tempfile.TemporaryDirectory() as directory:
                base = Path(directory)
                source = base / "dfilt.c"
                content = SDK + default + "\n" + ownership + digital
                if pinned:
                    content += VENDOR + function(rf, "static void restore_lock(")
                    content += function(rf, "bool rf_filter_applied(")
                    content += function(rf, "bool rf_set_filter(")
                    content += function(rf, "void rf_set_offset_khz(") + PINNED
                else:
                    content += UNSUPPORTED
                source.write_text(content)
                exe = base / "dfilt"
                subprocess.run(["cc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                *(["-DC5VRX_PHY_PINNED=1"] if pinned else []),
                                str(source), "-o", str(exe)], check=True)
                subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
