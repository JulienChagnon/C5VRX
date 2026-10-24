"""C5VRX/Twotoz: actual restart lease, native revocation and clock ownership.

SDK/session boundaries are mocked; this proves software ordering only.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
PREFIX=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "transport_recovery.h"
#define IRAM_ATTR
#define ESP_OK 0
static bool s_started,s_have_verified,s_lut_known,s_verified_flag,s_flag_format;
static bool s_transport_fault,s_stream_generation_overflow,s_menu_active,s_scan_active;
static bool s_rx_enabled,s_tx_enabled,s_bs_enabled;
static bool s_output_probe_busy;
static unsigned s_output,s_verified_output;
static int s_rx_node_count;
static uint32_t s_stream_generation;
static bool pin_clamped,observer_active,output_end_ok,observer_end_ok;
static unsigned invalidates,output_ends,observer_ends,source_calls;
static unsigned pin_applies,tx_enables;static int tx_enable_error;
static int enable_tx(void){++tx_enables;return tx_enable_error;}
static bool flag_pin_apply(bool flag){++pin_applies;pin_clamped=flag;return true;}
static char events[128];
static void event(char c){size_t n=strlen(events);assert(n+1<sizeof(events));events[n]=c;events[n+1]=0;}
static bool flag_pin_clamped(void){return pin_clamped;}
static void sync_live_output_phase(unsigned phase){assert(phase<6u);}
static bool sync_output_live_active(void){return observer_active;}
static bool sync_live_output_end(void){++output_ends;event('E');return output_end_ok;}
static bool sync_output_live_end(void){++observer_ends;event('O');if(observer_end_ok)observer_active=false;return observer_end_ok;}
bool transport_sync_live_cancel(void);
'''
MIDDLE=r'''
static void sync_output_live_invalidate(void){
 assert(s_live_owner->coasting_restart);++invalidates;event('I');
}
static bool sync_output_live_source(void *arg,uint32_t *epoch){
 (void)arg;++source_calls;if(epoch)*epoch=99;return true;
}
typedef struct {bool flag;} restart_t;
static bool stop_part(void *context,tr_part_t part){
 (void)context;
 if(s_live_owner->coasting_restart)assert(invalidates&&!output_ends&&!observer_ends);
 else assert(!s_live_owner->authorized&&output_ends&&observer_ends);
 event(part==TR_TX?'T':part==TR_BS?'B':'R');return true;
}
'''
SUFFIX=r'''
static sync_live_owner_t owner;
static void reset(void){
 memset(&owner,0,sizeof(owner));s_live_owner=&owner;
 owner.requested=owner.authorized=owner.output_requested=true;
 owner.context=(sync_live_context_t){7,4};owner.stream_generation=7;
 s_stream_generation=7;s_started=s_have_verified=s_lut_known=s_verified_flag=s_flag_format=true;
 s_transport_fault=s_stream_generation_overflow=s_menu_active=s_scan_active=false;
 s_rx_enabled=s_tx_enabled=s_bs_enabled=true;
 s_output=s_verified_output=4;s_rx_node_count=9;s_output_probe_busy=true;
 pin_clamped=observer_active=output_end_ok=observer_end_ok=true;
 invalidates=output_ends=observer_ends=source_calls=pin_applies=tx_enables=0;tx_enable_error=0;events[0]=0;
 assert(transport_sync_live_valid(7));
}
int main(void){unsigned checks=0;
 reset();assert(actual_tx_enable(true)&&tx_enables==1&&!pin_applies&&pin_clamped);++checks;
 reset();s_verified_flag=false;pin_clamped=false;
 assert(actual_tx_enable(true)&&tx_enables==1&&pin_applies==1&&pin_clamped);++checks;
 reset();assert(actual_tx_enable(false)&&pin_applies==1&&!pin_clamped);++checks;
 reset();tx_enable_error=1;assert(!actual_tx_enable(true)&&!pin_applies);++checks;
 reset();uint32_t epoch=0;assert(transport_sync_native_source(NULL,&epoch)&&epoch==99);++checks;
 /* Pending selections must not destroy the current verified output lease. */
 s_output=8;assert(transport_sync_live_valid(7));++checks;
 restart_t request={true};assert(quiesce(&request));
 assert(!strcmp(events,"ITBR")&&owner.coasting_restart&&owner.authorized);
 assert(!s_started&&!s_rx_enabled&&!s_tx_enabled&&!s_bs_enabled&&!s_rx_node_count);
 assert(transport_sync_live_valid(7)&&!transport_sync_live_valid(8));
 unsigned calls=source_calls;assert(!transport_sync_native_source(NULL,&epoch)&&source_calls==calls);++checks;
 ++s_stream_generation;assert(transport_sync_live_valid(7)); /* lease != DMA generation */
 s_verified_output=8;
 s_started=s_rx_enabled=s_tx_enabled=s_bs_enabled=true;
 transport_sync_restart_finish(true);
 assert(!owner.coasting_restart&&owner.context.generation==7&&owner.stream_generation==8);
 assert(owner.context.output==8);
 assert(transport_sync_live_valid(7)&&!transport_sync_live_valid(8));
 assert(!output_ends&&!observer_ends&&observer_active);++checks;
 /* Rollback performs a second quiesce on the same clock, then rebinds the
  * previous verified configuration to the replacement DMA generation. */
 reset();assert(quiesce(&request));++s_stream_generation;
 assert(quiesce(&request));++s_stream_generation;
 s_started=s_rx_enabled=s_tx_enabled=s_bs_enabled=true;
 transport_sync_restart_finish(true);
 assert(owner.context.generation==7&&owner.stream_generation==9&&transport_sync_live_valid(7));
 assert(invalidates==2&&!output_ends&&!observer_ends&&observer_active);++checks;
 reset();assert(quiesce(&request));transport_sync_restart_finish(false);
 assert(!owner.authorized&&!owner.coasting_restart&&!observer_active&&output_ends==1&&observer_ends==1);++checks;
 /* Exclusions are enforced even inside the coasting interval. */
 for(unsigned fault=0;fault<11;++fault){
  reset();assert(transport_sync_restart_begin(true));
  switch(fault){
   case 0:s_menu_active=true;break;case 1:s_scan_active=true;break;
   case 2:s_transport_fault=true;break;case 3:owner.fault=true;break;
   case 4:s_stream_generation_overflow=true;break;case 5:s_verified_flag=false;break;
   case 6:s_flag_format=false;break;case 7:s_lut_known=false;break;
   case 8:s_have_verified=false;break;case 9:pin_clamped=false;break;
   case 10:owner.authorized=false;break;
  }
  assert(!transport_sync_live_valid(7));++checks;
 }
 /* None of those states may acquire an initial coasting exception either. */
 for(unsigned fault=0;fault<14;++fault){
  reset();bool flag=true;
  switch(fault){
   case 0:s_menu_active=true;break;case 1:s_scan_active=true;break;
   case 2:s_transport_fault=true;break;case 3:owner.fault=true;break;
   case 4:s_stream_generation_overflow=true;break;case 5:s_verified_flag=false;break;
   case 6:s_flag_format=false;break;case 7:s_lut_known=false;break;
   case 8:s_have_verified=false;break;case 9:pin_clamped=false;break;
   case 10:owner.authorized=false;break;case 11:owner.requested=false;break;
   case 12:observer_active=false;break;case 13:flag=false;break;
  }
  assert(!transport_sync_restart_begin(flag)&&!owner.coasting_restart&&!invalidates);++checks;
 }
 /* Format exit, scan and menu take the ordinary stop path in quiesce. */
 for(unsigned reason=0;reason<3;++reason){
  reset();request.flag=reason!=0;s_menu_active=reason==1;s_scan_active=reason==2;
  assert(quiesce(&request));
  assert(!owner.coasting_restart&&!owner.authorized&&!observer_active);
  assert(!strcmp(events,"EOTBR"));++checks;
 }
 reset();assert(transport_sync_restart_begin(true));pin_clamped=false;
 transport_sync_restart_finish(true);assert(!owner.authorized&&!observer_active);++checks;
 reset();assert(transport_sync_restart_begin(true));output_end_ok=false;
 transport_sync_restart_finish(false);assert(owner.fault&&!owner.authorized&&!owner.coasting_restart);++checks;
 reset();s_stream_generation=8;assert(!transport_sync_live_valid(7));++checks;
 reset();s_live_owner=NULL;assert(!transport_sync_restart_begin(true)&&!transport_sync_live_valid(7));++checks;
 puts("PASS: actual restart lease, source revocation, clock preservation, rollback rebind and exclusion guards");
 printf("%u ownership scenarios; no physical timing claim\n",checks);
}
'''

def function(source,signature):
    start=source.index(signature+'\n{');brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]+'\n'

def main():
    source=(ROOT/'main/transport.c').read_text()
    a=source.index('typedef struct {\n    uint32_t generation;')
    context=source[a:source.index('} sync_live_context_t;',a)+len('} sync_live_context_t;')]
    a=source.index('typedef struct {\n    sync_live_context_t context;')
    owner=source[a:source.index('static sync_live_owner_t *s_live_owner;',a)+len('static sync_live_owner_t *s_live_owner;')]
    names=['static bool IRAM_ATTR transport_sync_live_valid(uint32_t generation)',
        'static bool transport_sync_restart_begin(bool flag)',
        'static void transport_sync_restart_finish(bool verified)',
        'bool transport_sync_live_cancel(void)',
        'static bool IRAM_ATTR transport_sync_native_source(void *arg,uint32_t *epoch)',
        'static bool quiesce(void *ctx)']
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory);unit=path/'run.c';binary=path/'run'
        stage_start=source.index('    case TR_TX_ENABLE:',source.index('static bool restart_stage('))
        stage=source[stage_start:source.index('    case TR_RX_START:',stage_start)]
        tx_helper=('static bool actual_tx_enable(bool flag){restart_t value={flag};restart_t *r=&value;'
            'int err=0;switch(TR_TX_ENABLE){'+stage+'default:return false;}return err==ESP_OK;}\n')
        unit.write_text(PREFIX+context+owner+MIDDLE+''.join(function(source,n) for n in names)+tx_helper+SUFFIX)
        subprocess.run([os.environ.get('CC','cc'),'-O2','-std=c11','-Wall','-Wextra','-Werror',
            '-I',str(ROOT/'main'),str(unit),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)

if __name__=='__main__':main()
