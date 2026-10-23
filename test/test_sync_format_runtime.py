"""C5VRX/Twotoz: execute actual flag-format publication and rollback branches.

Uses real restart coordinator, LUT tables, output scaling and comparator mapping.
SDK stages/pin voltage are fixtures; this is not a physical CVBS proof.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PREFIX = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "p64_tables.h"
#include "sync_flag.h"
#include "transport_recovery.h"
#include "transport_guard.h"
#include "transport_lut.h"
static const uint8_t s_p64[300]={1,0,3,8,0,2,1,[12]=4};
static bool s_menu_active;
static transport_guard_t s_switch_guard;
static int64_t now=10000000;
static int64_t esp_timer_get_time(void){return now;}
static bool s_started,s_transport_fault,s_lut_known,s_have_verified;
static bool s_flag_format,s_verified_flag;
static unsigned s_output,s_verified_output,s_output_clipped;
static uint16_t s_lut_live[1024],before[1024];
static uint32_t s_restart_errors,s_rollback_fail,s_rollbacks,s_control_epoch;
static unsigned settings_writes,cancels,acquires,stages,checks;
static unsigned restart_finishes;static bool restart_finish_ok;
/* Resident clock ownership is exercised separately in restart runtime tests. */
void transport_sync_restart_finish(bool ok){
 ++restart_finishes;restart_finish_ok=ok;
 if(ok){assert(s_started&&s_lut_known&&!s_transport_fault);
  assert(!memcmp(s_lut_work,s_lut_live,sizeof(s_lut_live)));}
 else assert(s_transport_fault&&!s_lut_known);
}
static int fail_forward=-1,fail_rollback=-1;
static bool physical_clamp,cancel_ok,acquire_ok;
static unsigned before_output;
static bool before_flag;
static struct {bool active;} owner_storage,*s_flag_pin;
static bool transport_sync_live_cancel(void){++cancels;return cancel_ok;}
static bool flag_pin_acquire(void){
 assert(!s_flag_pin);++acquires;if(!acquire_ok)return false;
 owner_storage.active=true;s_flag_pin=&owner_storage;return true;
}
static bool flag_pin_restored(void){return s_flag_pin&&s_flag_pin->active&&!physical_clamp;}
static bool flag_pin_clamped(void){return s_flag_pin&&s_flag_pin->active&&physical_clamp;}
static void settings_save_output(unsigned output){assert(output==s_verified_output);++settings_writes;}
static const char *transport_program_name(void){return "normal";}
static const char *transport_output_name(void){return "fixture";}
static int quiet_printf(const char *format,...){(void)format;return 1;}
#define printf quiet_printf
'''
STAGES = r'''
    /* Production rollback assignments above run before mocked SDK stages. */
    assert(!memcmp(s_lut_live,before,sizeof(before)));
    assert(s_verified_output==before_output &&
        s_verified_flag==before_flag);
    assert(r->program==tracker_program());
    assert(!memcmp(s_program_work.prefix,r->program,sizeof(s_program_work.prefix)));
    if(stage==TR_LUT_VERIFY){
        for(unsigned i=0;i<1024;++i){
            bool clipped;
            const uint16_t *canonical=p64_table;
            uint16_t expected=r->flag?sync_flag_entry(canonical[i],r->output,&clipped):
                output_entry(canonical[i],r->output,&clipped);
            assert(s_lut_work[i]==expected);++checks;
        }
    }
    if(stage==TR_TX_ENABLE){assert(!r->flag||s_flag_pin);physical_clamp=r->flag;}
    ++stages;
    return (int)stage!=(rollback?fail_rollback:fail_forward);
}
static bool quiesce(void *context){(void)context;s_started=false;return true;}
'''
SUFFIX = r'''
#undef printf
static void snapshot(void){
 memcpy(before,s_lut_live,sizeof(before));now+=6000000;
 before_output=s_verified_output;before_flag=s_verified_flag;
}
static void reset(unsigned level,unsigned output){
 (void)level;
 s_menu_active=false;memset(&s_switch_guard,0,sizeof(s_switch_guard));now=10000000;
 s_started=s_lut_known=s_have_verified=true;s_transport_fault=false;
 s_flag_format=s_verified_flag=false;
 s_output=s_verified_output=output;
 s_output_clipped=0;
 for(unsigned i=0;i<1024;++i){bool clipped;
  s_lut_live[i]=output_entry(p64_table[i],output,&clipped);s_output_clipped+=clipped;
 }
 s_flag_pin=NULL;owner_storage.active=false;physical_clamp=false;
 cancel_ok=acquire_ok=true;fail_forward=fail_rollback=-1;
 settings_writes=cancels=acquires=stages=restart_finishes=0;restart_finish_ok=false;
 s_restart_errors=s_rollback_fail=s_rollbacks=s_control_epoch=0;snapshot();
}
static void verify(bool flag,unsigned level,unsigned output){
 (void)level;
 assert(s_lut_known&&!s_transport_fault&&s_have_verified&&s_started);
 assert(restart_finishes&&restart_finish_ok);
 assert(s_verified_flag==flag&&s_flag_format==flag);
 assert(s_verified_output==output&&physical_clamp==flag);
 assert(flag?(s_flag_pin&&s_flag_pin->active):!s_flag_pin);
 unsigned clipped_count=0;
 for(unsigned i=0;i<1024;++i){bool clipped;
  const uint16_t *canonical=p64_table;
  uint16_t expected=flag?sync_flag_entry(canonical[i],output,&clipped):
    output_entry(canonical[i],output,&clipped);
  assert(s_lut_live[i]==expected);clipped_count+=clipped;++checks;
 }
 assert(s_output_clipped==clipped_count);
}
static unsigned selected, reads;
static unsigned corrupt;
static void select_entry(void *ctx,unsigned entry){(void)ctx;selected=entry;}
static uint32_t read_entry(void *ctx){(void)ctx;++reads;return p64_table[selected] ^ (selected==corrupt?1u:0u);}
int main(void){
 uint32_t failures=0;corrupt=1024;
 assert(transport_lut_verify(p64_table,NULL,select_entry,read_entry,&failures));
 assert(reads==1024&&!failures);
 for(corrupt=0;corrupt<1024;++corrupt){
  reads=0;assert(!transport_lut_verify(p64_table,NULL,select_entry,read_entry,&failures));
  assert(reads==1024&&failures==corrupt+1);++checks;
 }
 /* Exhaustive supported formats: comparison uses canonical code even at5x. */
 for(unsigned level=0;level<1;++level)
 for(unsigned output=0;output<OUTPUT_LEVELS;++output){
  reset(level,output);assert(transport_flag_format(true));verify(true,level,output);
  assert(!settings_writes&&acquires==1);
  snapshot();unsigned calls=stages;assert(transport_flag_format(true)&&stages==calls);
  assert(resume_live(s_lut_live));verify(true,level,output); /* no double scaling */
  snapshot();assert(transport_flag_format(false));verify(false,level,output);
  assert(!settings_writes);
 }
 /* While flagged, ordinary output/kp/LP updates must rebuild encoded LUTs. */
 reset(0,0);assert(transport_flag_format(true));
 for(unsigned output=0;output<OUTPUT_LEVELS;++output)
 for(unsigned level=0;level<1;++level){
  snapshot();s_output=output;
  assert(resume_live(p64_table));verify(true,level,output);
 }
 snapshot();assert(transport_flag_format(false));verify(false,0,s_output);
 /* Failure after any forward stage may not publish a partially encoded LUT.
  * Real rollback prefix restores selected configuration and pin SDK fixture. */
 for(int stage=0;stage<TR_STAGES;++stage)for(unsigned exit=0;exit<2;++exit){
  reset(2,8);if(exit){assert(transport_flag_format(true));snapshot();}
  fail_forward=stage;
  assert(!transport_flag_format(!exit));verify(exit,2,8);
  assert(s_restart_errors==1&&s_rollbacks==1&&!s_rollback_fail&&!settings_writes);
  assert(!memcmp(before,s_lut_live,sizeof(before)));++checks;
 }
 /* A failed attempted configuration change while flagged restores all three
  * user-facing selections and the original encoded shadow. */
 reset(2,4);assert(transport_flag_format(true));snapshot();
 fail_forward=TR_TX_START;s_output=8;
 assert(!resume_live(p64_table));verify(true,2,4);
 assert(s_output==4&&!settings_writes);++checks;
 /* Failed verified recovery latches unknown and keeps pin ownership retained. */
 for(unsigned exit=0;exit<2;++exit){
  reset(3,9);if(exit){assert(transport_flag_format(true));snapshot();}
  fail_forward=TR_TX_START;fail_rollback=TR_TX_START;
  assert(!transport_flag_format(!exit));
  assert(s_transport_fault&&!s_lut_known&&s_rollback_fail==1&&s_flag_pin);
  assert(restart_finishes==(exit?2u:1u)&&!restart_finish_ok);
  assert(!memcmp(before,s_lut_live,sizeof(before))&&!settings_writes);
  unsigned previous=stages;assert(!transport_flag_format(exit)&&stages==previous);++checks;
 }
 reset(0,0);cancel_ok=false;assert(!transport_flag_format(true)&&!acquires&&!stages);++checks;
 reset(0,0);acquire_ok=false;assert(!transport_flag_format(true)&&!stages&&!s_flag_pin);++checks;
 /* Cold boot loads only the complete P64 image. */
 reset(0,0);s_have_verified=s_started=s_lut_known=false;
 assert(resume_live(NULL));verify(false,0,0);snapshot();
 assert(tracker_program()==s_p64);
 /* Profile and OUTPUT changes share the guard, including rollback. */
 assert(transport_restart_profile());verify(false,0,0);
 unsigned untouched=stages;
 assert(!transport_set_output(4)&&!transport_restart_profile()&&stages==untouched);
 snapshot();assert(transport_set_output(4));verify(false,0,4);
 for(int stage=0;stage<TR_STAGES;++stage){
  reset(0,4);fail_forward=stage;assert(!transport_restart_profile());
  verify(false,0,4);assert(!memcmp(before,s_lut_live,sizeof(before)));
  assert(!transport_set_output(6)&&!transport_restart_profile());
 }
 reset(0,4);fail_forward=fail_rollback=TR_TX_START;
 assert(!transport_restart_profile()&&s_transport_fault&&!s_lut_known);
 assert(!transport_set_output(6)&&!transport_restart_profile());
 printf("PASS: %u actual format/publication/recovery checks; no physical claim\n",checks);
}
'''

def main():
    source=(ROOT/'main/transport.c').read_text()
    def segment(start,stop):
        a=source.index(start);return source[a:source.index(stop,a)]
    tracker=segment('static const void *tracker_program(void)', '/* 6-bit resistor DAC')
    image=segment('typedef struct {\n    uint32_t prefix[', 'static uint32_t s_lut_writes')
    prefix=PREFIX
    prefix=prefix.replace('static uint16_t s_lut_live[1024],before[1024];',
                          image+'\nstatic uint16_t s_lut_live[1024],before[1024];')
    # Keep only production selectors; state lives in the fixture above.
    tracker=tracker.replace('static bool s_flag_format,s_verified_flag;','')
    words=segment('static bool program_words(', '/* The analog profile has already changed')
    a=source.index('typedef struct {\n    const uint8_t *program;')
    restart=source[a:source.index('} restart_t;',a)+len('} restart_t;')]
    rollback=segment('static bool restart_stage(void *ctx, tr_stage_t stage, bool rollback)', '    esp_err_t err = ESP_OK;\n    switch (stage)')
    publication=segment('static bool resume_live(const uint16_t *restore_lut)\n{','void transport_resume_live(void)')
    def function(signature):
        a=source.index(signature);end=source.index('{',a)+1;depth=1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        return source[a:end]+'\n'
    selectors='static void stage_hooks(void){}\n'
    selectors+=''.join(function(signature) for signature in (
        'bool transport_set_output(unsigned level)', 'bool transport_restart_profile(void)'))
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory);unit=path/'run.c';binary=path/'run'
        unit.write_text(prefix+tracker+words+restart+rollback+STAGES+'static void stage_hooks(void);\n'+publication+selectors+SUFFIX)
        subprocess.run([os.environ.get('CC','cc'),'-O2','-std=c11','-Wall','-Wextra','-Werror',
            '-I',str(ROOT/'main'),str(unit),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)

if __name__=='__main__':main()
