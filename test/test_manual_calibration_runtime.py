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
  subprocess.run(['cc','-O2','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(p/'t.c'),'-o',str(p/'t')],check=True)
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
if __name__=='__main__':unittest.main()
