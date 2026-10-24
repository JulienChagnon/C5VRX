#!/usr/bin/env python3
"""C5VRX/Twotoz: execute production 11p entry, rollback and analog profiles.

Mocks only SDK/vendor/MMIO. Fault every owned field and rollback; exercise
factory-relative wider profiles and repeated vendor restores at both widths.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from test_dfilt_runtime import function

ROOT = Path(__file__).resolve().parents[1]
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
static int xSemaphoreTakeRecursive(void *p,int t){(void)t;assert(p);++locks;return 1;}
static int xSemaphoreGiveRecursive(void *p){assert(p&&locks);--locks;return 1;}
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NOT_SUPPORTED 1
#define ESP_ERR_INVALID_RESPONSE 2
#define ESP_FAIL 3
#define PHY_FILTER_CALIBRATED (-1)
static void invalidate(void){assert(locks);++hooks;}
'''
HARDWARE = r'''
unsigned char phy_param[2048];
static uint32_t words[4];
static uint8_t bytes[8];
static int fail_entry=-1;
static bool fail_restore;
static unsigned index_of(uint32_t addr) {
    switch(addr) {
    case 0x600A7CE4: return 0; case 0x600A7030: return 1;
    case 0x600A7048: return 2; case 0x600A71C4: return 3;
    default: assert(false); return 0;
    }
}
static uint32_t read_mmio(uint32_t addr){assert(locks);return words[index_of(addr)];}
static void write_mmio(uint32_t addr,uint32_t v){assert(locks);words[index_of(addr)]=v;}
#define REG_READ(a) read_mmio(a)
#define REG_WRITE(a,v) write_mmio(a,v)
uint8_t phy_i2c_readReg(uint8_t b,uint8_t h,uint8_t r) {
    assert(locks&&b==0x67&&h==1&&r>=6&&r<=13);return bytes[r-6];
}
void phy_i2c_writeReg(uint8_t b,uint8_t h,uint8_t r,uint8_t v) {
    assert(locks&&b==0x67&&h==1&&r>=6&&r<=13);
    if(!fail_restore)bytes[r-6]=v;
}
'''
VENDOR = r'''
void phy_11p_set(uint8_t e,uint8_t m) {
    assert(locks && phy_busy() && e==1 && m==0 && hooks);++writes;
    phy_param[0x26]=1;phy_param[0x27]=0;
    for(unsigned i=0;i<4;++i)
        words[i]=(words[i]&~s_11p_fields[i].mask)|s_11p_fields[i].value;
    for(unsigned i=0;i<8;++i)bytes[i]=60;
    /* Unrelated live fields must survive a masked rollback. */
    words[0]^=0x80000000u;
    if(fail_entry>=0 && fail_entry<4)words[fail_entry]^=s_11p_fields[fail_entry].mask;
    if(fail_entry>=4 && fail_entry<12)bytes[fail_entry-4]=59;
    if(fail_entry==12)phy_param[0x26]=0;
    if(fail_entry==13)phy_param[0x27]=1;
}
static void factory(void) {
    phy_param[0x26]=5;phy_param[0x27]=7;
    for(unsigned i=0;i<4;++i)words[i]=0xa5a5a5a5u;
    for(unsigned i=0;i<8;++i)bytes[i]=(uint8_t)(20+i);
}
static bool s_bw20;
static unsigned dc_hooks;
static unsigned s_gain=52;
static bool phy_dco_release(void){assert(phy_busy());return true;}
static void enable_continuous_modem(void){assert(phy_busy());}
static int lock_rx_only(void){assert(phy_busy());return 0;}
static void phy_disable_agc(void){assert(phy_busy());}
static void phy_rfagc_disable(void){assert(phy_busy());}
static void phy_wifi_fbw_sel(unsigned mode){assert(phy_busy()&&mode==!s_bw20);factory();}
static void phy_force_rx_gain(bool f,unsigned g){assert(phy_busy()&&f&&g==s_gain);}
static bool phy_dfilt_apply_default(void){assert(phy_busy()&&phy_11p_hw_enabled()==1);return true;}
static void post_gain(unsigned g){assert(g==s_gain&&phy_busy()&&phy_filter_hw_mismatch(NULL,NULL)==0);++dc_hooks;}
static void (*s_post_gain_hook)(unsigned)=post_gain;
typedef struct {int code;unsigned skirt;bool bw20;} rf_filter_t;
static rf_filter_t s_filter={-1,0,false};
#define ESP_EARLY_LOGW(...) assert(false)
#define ESP_ERROR_CHECK(e) assert((e)==ESP_OK)
'''
HARNESS = r'''
int main(void) {
    phy_set_observation_hook(invalidate);
    factory();assert(phy_filter_capture_base());
    assert(phy_filter_calibrated_code()==20);
    for(int fault=0;fault<14;++fault) {
        factory();fail_entry=fault;
        const unsigned gen=phy_generation(), before=hooks;
        assert(phy_11p_apply_default()==ESP_ERR_INVALID_RESPONSE);
        assert(phy_generation()==gen+1&&hooks==before+1&&!locks&&!phy_busy());
        assert(phy_param[0x26]==5&&phy_param[0x27]==7);
        for(unsigned i=0;i<8;++i)assert(bytes[i]==20+i);
        for(unsigned i=0;i<4;++i)
            assert(words[i]==(0xa5a5a5a5u^(i==0?0x80000000u:0)));
    }
    factory();fail_entry=4;fail_restore=true;
    assert(phy_11p_apply_default()==ESP_FAIL && !locks && !phy_busy());
    fail_restore=false;fail_entry=-1;
    for(unsigned bw=0;bw<2;++bw) {
        s_filter.bw20=s_bw20=(bool)bw;
        for(unsigned repeat=0;repeat<3;++repeat) {
            const unsigned gen=phy_generation(), before=hooks;
            phy_begin();restore_lock();phy_end();
            assert(phy_generation()==gen+1&&hooks==before+3&&!locks);
            assert(phy_11p_hw_enabled()==1&&phy_filter_hw_mismatch(NULL,NULL)==0);
            for(unsigned i=0;i<8;++i)assert(bytes[i]==60);
        }
        /* Explicit wider profiles retain the factory reference, not 60+skirt. */
        s_filter.code=16;s_filter.skirt=8;
        phy_begin();restore_lock();phy_end();
        assert(bytes[0]==16&&bytes[1]==16);
        for(unsigned i=2;i<8;++i)assert(bytes[i]==20+i+8);
        assert(phy_11p_hw_enabled()==1&&phy_filter_hw_mismatch(NULL,NULL)==0);
        /* Return to the exact eight-byte 11p baseline. */
        s_filter.code=-1;s_filter.skirt=0;
        phy_begin();restore_lock();phy_end();
        for(unsigned i=0;i<8;++i)assert(bytes[i]==60);
        words[2]^=0x100u;assert(phy_11p_hw_enabled()==0);
    }
    assert(dc_hooks==10&&!locks&&!phy_busy());
    puts("PASS: production 11p entry, 14 faults, exact rollback, fatal rollback result, nested restore and analog profiles");
}
'''
UNSUPPORTED = r'''
int main(void) {
    phy_set_observation_hook(invalidate);
    assert(phy_11p_apply_default()==ESP_ERR_NOT_SUPPORTED&&phy_11p_hw_enabled()==-1);
    assert(!locks&&!hooks&&!writes&&!phy_generation());
    puts("PASS: unsupported PHY refuses 11p without vendor/MMIO access");
}
'''


class ElevenP(unittest.TestCase):
    def test_production(self):
        phy = (ROOT/'main/phy.c').read_text()
        ownership = re.sub(r'^#include[^\n]*\n','',phy.split('#ifdef C5VRX_PHY_PINNED',1)[0],flags=re.M)
        analog = phy[phy.index('#ifdef C5VRX_PHY_PINNED'):phy.index('/* ---- Digital RX filter')]
        p11 = phy[phy.index('/* ---- 11p receive baseline'):phy.index('/* ---- RX DC DACs')]
        rf = (ROOT/'main/rf.c').read_text()
        for pinned in (True,False):
            with self.subTest(pinned=pinned), tempfile.TemporaryDirectory() as directory:
                base=Path(directory)
                content=SDK+ownership
                if pinned:
                    content+=HARDWARE+function((ROOT/'main/dsp.h').read_text(),'static inline uint8_t dsp_filter_code(')
                    content+=analog+p11+VENDOR+function(rf,'static void apply_filter(')+function(rf,'static void restore_lock(')+HARNESS
                else:
                    content+=p11+UNSUPPORTED
                (base/'test.c').write_text(content)
                subprocess.run(['cc','-O2','-std=c11','-Wall','-Wextra','-Werror',
                                *(['-DC5VRX_PHY_PINNED=1'] if pinned else []),
                                str(base/'test.c'),'-o',str(base/'test')],check=True)
                subprocess.run([str(base/'test')],check=True)


if __name__=='__main__':unittest.main()
