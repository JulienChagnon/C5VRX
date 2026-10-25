#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: actual main/PR DC searches on a mock PHY.
Reports scheduled wait time and observation count, NOT measured RF wall time.
"""
from pathlib import Path
import argparse
import json
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
PREFIX=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dsp.h"
#include "phy.h"
#define C5VRX_PHY_PINNED
static int codes[2]={220,200},target[2]={225,186};
static unsigned probes,wait_ms;
static void phy_pbus_force_test(unsigned b,unsigned bank,unsigned v){if(bank==2&&(b==2||b==3))codes[b-2]=v;}
void phy_begin(void){}
void phy_end(void){}
'''
MOCK=r'''
static void read_live(uint16_t live[PBUS_BLOCKS][2]){memset(live,0,sizeof(uint16_t)*PBUS_BLOCKS*2);live[2][0]=live[2][1]=codes[0];live[3][0]=live[3][1]=codes[1];}
static void enter_debug(uint16_t live[PBUS_BLOCKS][2]){(void)live;}
static void pbus_workmode_keep_gain(void){}
'''
SUFFIX=r'''
static bool measure(int dc[2],unsigned ms){++probes;wait_ms+=ms;dc[0]=(codes[0]-target[0])*50+(codes[1]-target[1])*5;dc[1]=(codes[1]-target[1])*50;return true;}
static bool slow(int dc[2]){return measure(dc,178);}
static bool fast(int dc[2]){return measure(dc,82);}
static bool flat(int dc[2]){++probes;wait_ms+=26;dc[0]=codes[0]<=target[0]?-500:500;dc[1]=codes[1]<=target[1]?-500:500;return true;}
int main(void){
 phy_dco_result_t r;int seed[2]={225,186};
 for(int k=0;k<3;++k){
  codes[0]=220;codes[1]=200;probes=wait_ms=0;
  if(k==2){FLAT_CALL;}else{MATRIX_CALL;}
  assert(r.measured && codes[0]==220 && codes[1]==200);
  printf("BENCH case=%d observations=%u wait_ms=%u residual_i=%d residual_q=%d\n",k,probes,wait_ms,r.residual_mcells[0],r.residual_mcells[1]);
 }
 (void)fast;
}
'''
def run(source,modern):
 helpers=source[source.index('#define DCO_PROBE'):source.index('/* phy_pbus_workmode()')]
 helpers=helpers.replace('static int s_dco_codes[2];','').replace('static bool s_dco_held;','')
 body=source[source.index('esp_err_t phy_dco_search(bool'):source.index('esp_err_t phy_dco_hold(')]
 call='phy_dco_search_verified(fast,slow,k==0?seed:NULL,&r)' if modern else 'phy_dco_search_from(slow,k==0?seed:NULL,&r)'
 flat='phy_dco_search_flat_from(flat,seed,&r)' if modern else 'phy_dco_search_flat(flat,&r)'
 harness=PREFIX+helpers+MOCK+body+SUFFIX.replace('MATRIX_CALL',call).replace('FLAT_CALL',flat)
 with tempfile.TemporaryDirectory() as d:
  p=Path(d);(p/'b.c').write_text(harness)
  (p/'esp_err.h').write_text('typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NOT_SUPPORTED 1\n#define ESP_ERR_INVALID_ARG 2\n#define ESP_ERR_INVALID_RESPONSE 3\n')
  subprocess.run(['cc','-O2','-std=c11','-Wall','-Wextra','-Werror','-I',str(p),'-I',str(ROOT/'main'),str(p/'b.c'),'-lm','-o',str(p/'b')],check=True)
  lines=subprocess.check_output([str(p/'b')],text=True).splitlines()
 result=[]
 for line in lines:
  if line.startswith('BENCH '):result.append({k:int(v) for k,v in (field.split('=') for field in line.split()[1:])})
 return result

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--base',default='origin/main');parser.add_argument('--output',type=Path)
 args=parser.parse_args()
 old=subprocess.check_output(['git','show',f'{args.base}:main/phy.c'],cwd=ROOT,text=True)
 new=(ROOT/'main/phy.c').read_text()
 base=run(old,False);pr=run(new,True)
 names=['centred_neighbour','coupled_correction','flat_local_bracket']
 result={'evidence':'deterministic mock PHY; scheduled waits only; not hardware timing or RF quality',
 'base_commit':subprocess.check_output(['git','rev-parse',args.base],cwd=ROOT,text=True).strip(),
 'cases':[{ 'case':names[i],'main':a,'pr':b,'wait_speedup':round(a['wait_ms']/b['wait_ms'],3)} for i,(a,b) in enumerate(zip(base,pr))],
 'boot_policy':{'pr_automatic_sweeps':0,'pr_automatic_searches':0,'pr_boot_phase_budget_ms':600,'pr_automatic_dc_trim':0,
                'note':'normal boot loads baseline/saved settings; shared passive checks run independently, not a blocking sweep'}}
 text=json.dumps(result,indent=2)+'\n'
 if args.output:args.output.write_text(text)
 print(text,end='')
if __name__=='__main__':main()
