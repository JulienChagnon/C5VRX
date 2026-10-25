#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: production boot and dump framing boundaries."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

def run(source):
 with tempfile.TemporaryDirectory() as d:
  p=Path(d);(p/'t.c').write_text(source)
  subprocess.run(['cc','-O2','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(p/'t.c'),'-lm','-o',str(p/'t')],check=True)
  return subprocess.check_output([str(p/'t')],text=True)

class PolicyTest(unittest.TestCase):
 def test_actual_boot_never_calibrates(self):
  s=(ROOT/'main/calibration.c').read_text()
  body=s[s.index('static void calibration_boot(void)'):s.index('static void calibration_task(')]
  run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
static bool s_startup=true;
static int64_t s_boot_policy_ms,s_boot_phase_until;
static bool s_boot_phase_pending;
static unsigned selected,ready,live;
static int64_t esp_timer_get_time(void){return 1000;}
static void dco_select(void){++selected;}
static void dco_ready_note(void){++ready;}
static void profile_live_begin(void){++live;}
static int log_printf(const char *f,...){(void)f;return 0;}
#define printf log_printf
''' + body + r'''
int main(void){calibration_boot();assert(!s_startup && selected==1 && ready==1 && live==1 && s_boot_policy_ms==0);}
''')
 def test_warning_requires_evidence_and_recovers(self):
  run(r'''
#include <assert.h>
#include "calibration_policy.h"
int main(void){
 cal_watch_t w={0};
 for(int k=1;k<10;++k)assert(!cal_watch_add(&w,1,k*300000,false,0,4000,4000));
 assert(w.state==CAL_UNKNOWN);
 for(int k=10;k<12;++k)assert(!cal_watch_add(&w,1,k*300000,true,0,2000,0));
 assert(cal_watch_add(&w,1,12*300000,true,0,2000,0));assert(w.state==CAL_BAD);
 assert(!cal_watch_add(&w,2,13*300000,true,0,2000,0));assert(w.state==CAL_UNKNOWN);
 cal_watch_add(&w,2,14*300000,true,0,2000,0);
 assert(!cal_watch_add(&w,2,30*300000,true,0,2000,0));assert(w.state==CAL_UNKNOWN);
 for(int k=31;k<34;++k)cal_watch_add(&w,2,k*300000,true,0,0,0);
 assert(w.state==CAL_GOOD);
}
''')
 def test_actual_dump_line_integrity_and_bounds(self):
  s=(ROOT/'main/calibration.c').read_text()
  body=s[s.index('typedef struct { unsigned lines; uint32_t hash; bool failed; } cal_dump_t;'):s.index('static void cal_dump_table(')]
  out=run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static void vTaskDelay(unsigned n){assert(n==2);}
''' + body + r'''
int main(void){
 cal_dump_t d={.hash=2166136261u};
 cal_dump_line(&d,"{\"type\":\"device\",\"schema\":1}");
 cal_dump_line(&d,"{\"type\":\"capture\",\"live_context_unchanged\":true}");
 printf("END %u %08x\n",d.lines,d.hash);
 char huge[600];memset(huge,'x',599);huge[599]=0;
 cal_dump_line(&d,"%s",huge);assert(d.failed && d.lines==2);
}
''')
  lines=out.splitlines();hash_value=2166136261
  for n,line in enumerate(lines[:2]):
   self.assertTrue(line.startswith(f'CALDUMP_LINE n={n} '))
   payload=line.split(' ',2)[2]
   for b in (payload+'\n').encode():hash_value=((hash_value^b)*16777619)&0xffffffff
  self.assertEqual(lines[2],f'END 2 {hash_value:08x}')
 def test_dump_has_no_rf_actuator(self):
  s=(ROOT/'main/calibration.c').read_text();body=s[s.index('static void calibration_dump(void)'):s.index('static void calibration_boot(void)')]
  for name in ['rf_set_filter(', 'rf_set_channel(', 'receiver_set_gain(', 'phy_dco_search(', 'rx_clock_slip(', 'status_screen(', 'blob_store(']:
   self.assertNotIn(name,body)
 def test_menu_recommendation_is_latched_until_recovery(self):
  s=(ROOT/'main/calibration.c').read_text()
  body=s[s.index('unsigned calibration_dc_needed('):s.index('static void dc_health_print(')]
  run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
static bool s_cal_notice;
static uint16_t s_cal_notice_freq=5865,s_cal_notice_key=42;
static unsigned frequency=5865,key=42;
static unsigned rf_frequency_mhz(void){return frequency;}
static unsigned live_filter_key(void){return key;}
''' + body + r'''
int main(void){char label[24];
 assert(!calibration_dc_needed(label,sizeof(label))); /* missing data alone */
 s_cal_notice=true;assert(calibration_dc_needed(label,sizeof(label)));
 assert(calibration_dc_needed(NULL,0)); /* menu may stay open indefinitely */
 frequency=5805;assert(!calibration_dc_needed(NULL,0));
 frequency=5865;key=43;assert(!calibration_dc_needed(NULL,0));
 key=42;s_cal_notice=false;assert(!calibration_dc_needed(NULL,0));
}
''')
 def test_actual_boot_phase_guard(self):
  s=(ROOT/'main/calibration.c').read_text()
  body=s[s.index('static bool phase_slip(void)'):s.index('static sphase_verdict_t sphase_scan_n(')]
  run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
static int64_t s_probe_deadline=600000,now;
static bool splash=true,inside;
static unsigned writes;
static int64_t esp_timer_get_time(void){return now;}
static bool menu_splash_active(void){return splash;}
static void vTaskSuspendAll(void){inside=true;}
static int xTaskResumeAll(void){inside=false;return 0;}
static void rx_clock_slip(unsigned n){assert(n==1);if(s_probe_deadline)assert(inside && splash && now<s_probe_deadline);++writes;}
''' + body + r'''
int main(void){
 assert(phase_slip() && writes==1 && !inside);
 splash=false;assert(!phase_slip() && writes==1 && !inside);
 splash=true;now=600000;assert(!phase_slip() && writes==1 && !inside);
 s_probe_deadline=0;assert(phase_slip() && writes==2); /* explicit manual path */
}
''')
 def test_actual_splash_respects_application_deadline(self):
  s=(ROOT/'main/main.c').read_text()
  body=s[s.index('static void splash_wait_gain('):s.index('void app_main(void)')]
  run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define DG_HOLD 1
#define SPLASH_MIN_MS 3000
#define SPLASH_MAX_MS 4000
#define pdMS_TO_TICKS(x) (x)
static unsigned s_current_gain=83;
static struct {int state;unsigned holds;} s_gain={1,1};
static int64_t now;
static int64_t esp_timer_get_time(void){return now;}
static void vTaskDelay(unsigned ms){now+=ms*1000;}
static void menu_splash_progress(int p){(void)p;}
''' + body + r'''
int main(void){
 splash_wait_gain(0,0,1000000);assert(now==1000000); /* deadline overrides 3 s logo */
 now=0;splash_wait_gain(0,0,9000000);assert(now==3000000);
 now=0;s_gain.state=0;splash_wait_gain(0,0,9000000);assert(now==4000000);
}
''')

 def test_actual_noise_dither_collection(self):
  s=(ROOT/'main/calibration.c').read_text()
  typedef=s[s.index('typedef struct {\n    uint32_t windows;'):s.index('static int64_t s_boot_policy_ms')]
  body=s[s.index('static bool collect_stats('):s.index('static bool collect(unsigned windows')]
  run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "dsp.h"
#include "cn_auto.h"
#include "dc_live.h"
#define PROBE_BYTES 256u
#define PROBE_REGIONS 4u
#define PROBE_REGION_BYTES 64u
typedef struct {int clip_pm;} metrics_t;
static int64_t s_probe_deadline;
static int mode;
static int64_t esp_timer_get_time(void){return 1;}
static bool menu_splash_active(void){return true;}
static void vTaskDelay(unsigned n){assert(n==2);}
static metrics_t measure_window(const uint8_t *s,unsigned n){(void)s;(void)n;return (metrics_t){0};}
static bool copy_probe_window(uint8_t *s,void *node){
 (void)node;
 for(unsigned k=0;k<256;++k){
  int i=3,q=0;
  if(mode==0){i+=(k%4==0?-1:k%4==3?1:0);q=(k/4%4==0?-1:k/4%4==3?1:0);}
  if(mode==1){i+=(k%2?1:0);q=(k%2?1:0);}
  if(mode==2){i+=(k<2?-1:k%2?1:0);q=(k/4%4==0?-1:k/4%4==3?1:0);}
  s[k]=(uint8_t)(((i&15)<<4)|(q&15));
 }
 return true;
}
''' + typedef + body + r'''
int main(void){window_stats_t w;
 mode=0;assert(collect_stats(24,&w,true) && w.dc_noise_dither);
 mode=1;assert(collect_stats(24,&w,true) && !w.dc_noise_dither);
 mode=2;assert(collect_stats(24,&w,true) && !w.dc_noise_dither);
 mode=0;assert(collect_stats(24,&w,false) && !w.dc_noise_dither);
}
''')

if __name__=='__main__':unittest.main()
