#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: actual shared passive calibration monitor.
No automatic phase/DC actuator, no warning from unknown or missing maps.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "calibration_policy.h"
#include "dc_health.h"
#include "sphase_logic.h"
#define RF_LANE_ULTRAFINE 2
static cal_watch_t s_basemap_watch;
static uint32_t s_basemap_context;
static bool s_cal_notice;
static uint16_t s_cal_notice_freq,s_cal_notice_key;
static uint8_t s_cal_notice_gain;
static unsigned s_current_gain=83;
static unsigned rf_frequency_mhz(void){return 5865;}
static unsigned live_filter_key(void){return 42;}
static unsigned s_health_blocks,s_health_rejected;
static int64_t s_health_us,s_last_gain_write_us;
static int s_drift_avg[2],s_sphase_excess;
static const char *s_sphase_state="unverified";
static bool s_dc_urgent,s_startup,s_scan_active,s_menu_active;
static bool paused,busy,splash,copy_ok=true,qualified=true,dither=true,video,circle,change_context;
static int measured_i,measured_q,mismatch,phase=20;
static unsigned env=100;
static double m2=2,kurt=2;
static int64_t now=100000;
static unsigned context=1,collections,warnings;
static int64_t esp_timer_get_time(void){return now;}
static bool receiver_paused(void){return paused;}
static bool phy_busy(void){return busy;}
static bool menu_splash_active(void){return splash;}
static unsigned rf_lane(void){return 2;}
static uint32_t health_context(void){return context;}
static int phy_filter_hw_mismatch(void *a,void *b){(void)a;(void)b;return mismatch;}
static bool video_recent(void){return video;}
static unsigned dsp_quiet_limit(int clip){(void)clip;return 115;}
typedef struct {unsigned windows,envelope_x100;int dc_i,dc_q;bool dc_circle,dc_qualified,dc_noise_dither;
 double noise_m2,noise_m4;struct {int clip_pm;} m;} window_stats_t;
static bool collect_stats(unsigned n,window_stats_t *w,bool noise){
 assert(n==24 && noise);++collections;
 *w=(window_stats_t){.windows=copy_ok?n:0,.dc_i=measured_i,.dc_q=measured_q,
 .dc_circle=circle,.dc_qualified=qualified,.dc_noise_dither=dither,.envelope_x100=env,.noise_m2=m2*n,.noise_m4=kurt*m2*m2*n};
 if(change_context)++context;
 return copy_ok;
}
static int sphase_record(const window_stats_t *w){(void)w;s_sphase_excess=phase;return phase;}
/* Any accidental addition of an actuator to production becomes observable. */
unsigned actuations;
void rx_clock_slip(unsigned n){actuations+=n;}
void phy_dco_try_pair(void){++actuations;}
static int log_printf(const char *format,...){if(strstr(format,"CAL_NEEDED"))++warnings;return 0;}
#define printf log_printf
'''
SUFFIX = r'''
#undef printf
static void tick(void){now+=300000;drift_service();assert(!actuations);}
int main(void){
 tick();tick();tick();assert(s_basemap_watch.state==CAL_GOOD && !warnings);
 measured_i=2000;tick();tick();assert(!warnings);tick();assert(warnings==1);
 for(int k=0;k<8;++k){tick();}assert(warnings==1); /* no warning spam */
 measured_i=0;tick();tick();tick();assert(s_basemap_watch.state==CAL_GOOD);
 /* A single spike, ambiguous cloud, copy failure, and changing RF are not a bad map. */
 measured_i=3000;tick();measured_i=0;tick();assert(warnings==1);
 ++context;qualified=dither=false;measured_i=3000;tick();tick();tick();assert(warnings==1);
 qualified=dither=true;copy_ok=false;tick();tick();assert(warnings==1);
 copy_ok=true;change_context=true;tick();tick();assert(warnings==1);change_context=false;
 ++context;measured_i=0;phase=900;tick();tick();tick();
 assert(warnings==1 && !strcmp(s_sphase_state,"unverified")); /* high HF is not proof of bad map */
 /* Hardware readback is independently qualified even without a valid DC cloud. */
 ++context;copy_ok=false;mismatch=2;tick();tick();tick();assert(warnings==2);
 mismatch=0;copy_ok=qualified=true;measured_i=0;tick();tick();tick();
 unsigned prev=collections;s_menu_active=true;tick();assert(collections==prev);
 splash=true;tick();assert(collections==prev+1); /* can use existing boot raster */
 s_menu_active=splash=false;paused=true;prev=collections;tick();assert(collections==prev);
 paused=false;s_startup=true;tick();assert(collections==prev);
 /* Real low-gain noise can be below the circle-fit variance threshold but
  * still dither >=3 cells per axis; it must reveal a large DC park. */
 s_startup=false;++context;qualified=false;dither=true;measured_i=2200;m2=.9;kurt=2;
 tick();tick();tick();assert(warnings==3);
 puts("PASS: production monitor shares DC/noise/phase observations, warns only on sustained qualified bad evidence, no actuation");
}
'''
class SamplingRuntimeTest(unittest.TestCase):
 def test_actual_shared_monitor(self):
  source=(ROOT/'main/calibration.c').read_text()
  begin=source.index('static void drift_service(void)')
  end=source.index('/* Persist the measured gain map',begin)
  with tempfile.TemporaryDirectory() as d:
   folder=Path(d);(folder/'monitor.c').write_text(PREFIX+source[begin:end]+SUFFIX)
   subprocess.run([os.environ.get('CC','cc'),'-O2','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(folder/'monitor.c'),'-o',str(folder/'run')],check=True)
   subprocess.run([str(folder/'run')],check=True)
if __name__=='__main__':unittest.main()
