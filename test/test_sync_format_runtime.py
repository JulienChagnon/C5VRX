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
#include "kp_tables.h"
#include "p64_tables.h"
#include "sync_flag.h"
#include "transport_recovery.h"
#include "kp_switch_guard.h"
static const uint8_t s_u85q[300]={1,0,3,8,0,2,1,[12]=1};
static const uint8_t s_u85q_lp[300]={1,0,3,8,0,2,1,[12]=2};
static const uint8_t s_u85q_lp_flag[300]={1,0,3,8,0,2,1,[12]=3};
static const uint8_t s_u85q_64[300]={1,0,3,8,0,2,1,[12]=4};
static bool s_demod_p64,s_verified_p64;
static bool s_menu_active;
static kp_switch_guard_t s_switch_guard;
static int64_t now=10000000;
static int64_t esp_timer_get_time(void){return now;}
static bool s_started,s_transport_fault,s_lut_known,s_have_verified;
static bool s_prog_lp,s_flag_format,s_verified_lp,s_verified_flag;
static unsigned s_output,s_verified_output,s_output_clipped;
static uint16_t s_lut_live[1024],before[1024];
static uint8_t s_kp_target;
static int s_verified_level;
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
static int before_level;static unsigned before_output;
static bool before_lp,before_flag,before_p64;
static struct {bool active;} owner_storage,*s_flag_pin;
static bool transport_sync_live_cancel(void){++cancels;return cancel_ok;}
static bool flag_pin_acquire(void){
 assert(!s_flag_pin);++acquires;if(!acquire_ok)return false;
 owner_storage.active=true;s_flag_pin=&owner_storage;return true;
}
static bool flag_pin_restored(void){return s_flag_pin&&s_flag_pin->active&&!physical_clamp;}
static bool flag_pin_clamped(void){return s_flag_pin&&s_flag_pin->active&&physical_clamp;}
static void settings_save_output(unsigned output){assert(output==s_verified_output);++settings_writes;}
static const char *transport_program_name(void){return s_prog_lp?"LP":"normal";}
static const char *transport_output_name(void){return "fixture";}
static int quiet_printf(const char *format,...){(void)format;return 1;}
#define printf quiet_printf
'''
STAGES = r'''
    /* Production rollback assignments above run before mocked SDK stages. */
    assert(!memcmp(s_lut_live,before,sizeof(before)));
    assert(s_verified_level==before_level && s_verified_output==before_output &&
        s_verified_lp==before_lp && s_verified_flag==before_flag && s_verified_p64==before_p64);
    assert(r->program==tracker_format_program(r->lp,r->flag));
    assert(!memcmp(s_program_work.prefix,r->program,sizeof(s_program_work.prefix)));
    if(stage==TR_LUT_VERIFY){
        for(unsigned i=0;i<1024;++i){
            bool clipped;
            const uint16_t *canonical=tracker_lut(r->level);
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
 memcpy(before,s_lut_live,sizeof(before));before_level=s_verified_level;
 before_output=s_verified_output;before_lp=s_verified_lp;before_flag=s_verified_flag;
 before_p64=s_verified_p64;
}
static void reset(unsigned level,unsigned output,bool lp){
 s_demod_p64=s_verified_p64=false;
 s_menu_active=false;memset(&s_switch_guard,0,sizeof(s_switch_guard));now=10000000;
 s_started=s_lut_known=s_have_verified=true;s_transport_fault=false;
 s_prog_lp=s_verified_lp=lp;s_flag_format=s_verified_flag=false;
 s_output=s_verified_output=output;s_verified_level=(int)level;s_kp_target=(uint8_t)level;
 s_output_clipped=0;
 for(unsigned i=0;i<1024;++i){bool clipped;
  s_lut_live[i]=output_entry(kp_tables[level][i],output,&clipped);s_output_clipped+=clipped;
 }
 s_flag_pin=NULL;owner_storage.active=false;physical_clamp=false;
 cancel_ok=acquire_ok=true;fail_forward=fail_rollback=-1;
 settings_writes=cancels=acquires=stages=restart_finishes=0;restart_finish_ok=false;
 s_restart_errors=s_rollback_fail=s_rollbacks=s_control_epoch=0;snapshot();
}
static void verify(bool flag,unsigned level,unsigned output,bool lp){
 assert(s_lut_known&&!s_transport_fault&&s_have_verified&&s_started);
 assert(restart_finishes&&restart_finish_ok);
 assert(s_verified_flag==flag&&s_flag_format==flag&&s_verified_level==(int)level);
 assert(s_verified_lp==lp&&s_verified_output==output&&physical_clamp==flag);
 assert(flag?(s_flag_pin&&s_flag_pin->active):!s_flag_pin);
 unsigned clipped_count=0;
 for(unsigned i=0;i<1024;++i){bool clipped;
  const uint16_t *canonical=tracker_lut((int)level);
  uint16_t expected=flag?sync_flag_entry(canonical[i],output,&clipped):
    output_entry(canonical[i],output,&clipped);
  assert(s_lut_live[i]==expected);clipped_count+=clipped;++checks;
 }
 assert(s_output_clipped==clipped_count);
}
int main(void){
 /* Exhaustive supported formats: comparison uses canonical code even at5x. */
 for(unsigned lp=0;lp<2;++lp)for(unsigned level=0;level<KP_LEVELS;++level)
 for(unsigned output=0;output<OUTPUT_LEVELS;++output){
  reset(level,output,lp);assert(transport_flag_format(true));verify(true,level,output,lp);
  assert(!settings_writes&&acquires==1);
  snapshot();unsigned calls=stages;assert(transport_flag_format(true)&&stages==calls);
  assert(resume_live(s_lut_live));verify(true,level,output,lp); /* no double scaling */
  snapshot();assert(transport_flag_format(false));verify(false,level,output,lp);
  assert(!settings_writes);
 }
 /* While flagged, ordinary output/kp/LP updates must rebuild encoded LUTs. */
 reset(0,0,false);assert(transport_flag_format(true));
 for(unsigned output=0;output<OUTPUT_LEVELS;++output)
 for(unsigned level=0;level<KP_LEVELS;++level){
  snapshot();s_output=output;s_prog_lp=((level+output)&1u)!=0;
  assert(resume_live(kp_tables[level]));verify(true,level,output,s_prog_lp);
 }
 snapshot();assert(transport_flag_format(false));verify(false,s_verified_level,s_output,s_prog_lp);
 /* Failure after any forward stage may not publish a partially encoded LUT.
  * Real rollback prefix restores selected configuration and pin SDK fixture. */
 for(int stage=0;stage<TR_STAGES;++stage)for(unsigned exit=0;exit<2;++exit){
  reset(2,8,true);if(exit){assert(transport_flag_format(true));snapshot();}
  fail_forward=stage;
  assert(!transport_flag_format(!exit));verify(exit,2,8,true);
  assert(s_restart_errors==1&&s_rollbacks==1&&!s_rollback_fail&&!settings_writes);
  assert(!memcmp(before,s_lut_live,sizeof(before)));++checks;
 }
 /* A failed attempted configuration change while flagged restores all three
  * user-facing selections and the original encoded shadow. */
 reset(2,4,false);assert(transport_flag_format(true));snapshot();
 fail_forward=TR_TX_START;s_output=8;s_prog_lp=true;
 assert(!resume_live(kp_tables[6]));verify(true,2,4,false);
 assert(s_output==4&&!s_prog_lp&&!settings_writes);++checks;
 /* Failed verified recovery latches unknown and keeps pin ownership retained. */
 for(unsigned exit=0;exit<2;++exit){
  reset(3,9,false);if(exit){assert(transport_flag_format(true));snapshot();}
  fail_forward=TR_TX_START;fail_rollback=TR_TX_START;
  assert(!transport_flag_format(!exit));
  assert(s_transport_fault&&!s_lut_known&&s_rollback_fail==1&&s_flag_pin);
  assert(restart_finishes==(exit?2u:1u)&&!restart_finish_ok);
  assert(!memcmp(before,s_lut_live,sizeof(before))&&!settings_writes);
  unsigned previous=stages;assert(!transport_flag_format(exit)&&stages==previous);++checks;
 }
 reset(0,0,false);cancel_ok=false;assert(!transport_flag_format(true)&&!acquires&&!stages);++checks;
 reset(0,0,false);acquire_ok=false;assert(!transport_flag_format(true)&&!stages&&!s_flag_pin);++checks;
 /* P64 uses its own program even with the comparator format enabled.
  * Every output setting must preserve all six state bits and four tokens. */
 for(unsigned output=0;output<OUTPUT_LEVELS;++output){
  reset(2,output,false);assert(transport_set_p64(true));
  assert(s_verified_p64&&s_demod_p64&&tracker_format_program(true,true)==s_u85q_64);
  verify(false,2,output,false);snapshot();
  assert(transport_flag_format(true));verify(true,2,output,false);snapshot();
  assert(resume_live(s_lut_live));verify(true,2,output,false);snapshot();
  assert(transport_flag_format(false));verify(false,2,output,false);
 }
 /* Mode changes obey the shared cooldown and publish only after a full
  * verified restart; a failure restores the prior program and table. */
 for(unsigned previous=0;previous<2;++previous)for(int stage=0;stage<TR_STAGES;++stage){
  reset(3,4,true);if(previous){assert(transport_set_p64(true));snapshot();now+=6000000;}
  assert(transport_flag_format(true));snapshot();fail_forward=stage;
  assert(!transport_set_p64(!previous));
  assert(s_demod_p64==(bool)previous&&s_verified_p64==(bool)previous);
  assert(s_prog_lp==before_lp&&!memcmp(before,s_lut_live,sizeof(before)));
  assert(!transport_set_p64(!previous)); /* failed attempt also starts cooldown */
 }
 reset(2,0,false);assert(transport_set_p64(true));snapshot();
 assert(!transport_set_p64(false)&&s_demod_p64);
 now+=6000000;assert(transport_set_p64(false)&&!s_verified_p64);verify(false,2,0,false);snapshot();
 now+=6000000;assert(transport_set_p64(true));snapshot();now+=6000000;
 assert(transport_set_output(4)&&s_verified_p64);verify(false,2,4,false);
 /* Menu staging and initial boot both load the selected complete image. */
 reset(1,0,false);s_menu_active=true;assert(transport_set_p64(true));
 assert(!stages&&!s_verified_p64);s_menu_active=false;assert(resume_live(NULL));
 verify(false,1,0,false);assert(s_verified_p64);
 reset(0,0,false);s_have_verified=s_started=s_lut_known=false;s_demod_p64=true;
 assert(resume_live(NULL)&&s_verified_p64);verify(false,0,0,false);
 unsigned untouched=stages;snapshot();
 assert(transport_kp_level()==-1&&!transport_kp_set(3)&&!transport_set_lp(true));
 assert(!transport_apply_auto(3,true)&&stages==untouched);
 assert(!transport_apply_auto(KP_LEVELS,false)&&stages==untouched);
 assert(transport_apply_auto(3,false)&&s_kp_target==3&&stages>untouched);
 verify(false,3,0,false);snapshot();untouched=stages;
 assert(!transport_set_output(4)&&!transport_set_p64(false)&&stages==untouched);
 assert(!transport_apply_auto(4,false)); /* all live actions share profile cooldown */
 assert(!memcmp(before,s_lut_live,sizeof(before))&&s_verified_p64);
 /* Profile restarts retain P64, including every verified rollback path. */
 for(int stage=0;stage<TR_STAGES;++stage){
  reset(2,0,false);assert(transport_set_p64(true));snapshot();now+=6000000;
  fail_forward=stage;assert(!transport_apply_auto(6,false));
  assert(s_verified_p64&&s_demod_p64&&s_verified_level==2&&s_kp_target==2);
  assert(!memcmp(before,s_lut_live,sizeof(before))&&!transport_set_output(4));
 }
 /* Terminal failures cannot advertise the attempted decoder or allow a
  * subsequent mode change after verified recovery also failed. */
 for(unsigned previous=0;previous<2;++previous){
  reset(0,0,false);if(previous){assert(transport_set_p64(true));snapshot();now+=6000000;}
  fail_forward=fail_rollback=TR_TX_START;
  assert(!transport_set_p64(!previous)&&s_transport_fault&&!s_lut_known);
  assert(s_verified_p64==(bool)previous&&!transport_set_p64(!previous));
 }
 printf("PASS: %u actual format/publication/recovery checks; no physical claim\n",checks);
}
'''

def main():
    source=(ROOT/'main/transport.c').read_text()
    def segment(start,stop):
        a=source.index(start);return source[a:source.index(stop,a)]
    tracker=segment('static const void *tracker_program(bool lp)', '/* 6-bit resistor DAC')
    image=segment('typedef struct {\n    uint32_t prefix[', 'static uint32_t s_kp_writes')
    prefix=PREFIX.replace('static uint16_t s_lut_live[1024],before[1024];',
                          image+'\nstatic uint16_t s_lut_live[1024],before[1024];')
    # Keep only production selectors; state lives in the fixture above.
    tracker=tracker.replace('static bool s_flag_format,s_verified_flag;','')
    words=segment('static bool program_words(', '/* All live kp/program writes')
    a=source.index('typedef struct {\n    const uint8_t *program;')
    restart=source[a:source.index('} restart_t;',a)+len('} restart_t;')]
    rollback=segment('static bool restart_stage(void *ctx, tr_stage_t stage, bool rollback)', '    esp_err_t err = ESP_OK;\n    switch (stage)')
    publication=segment('static bool resume_live(const uint16_t *restore_lut)\n{','void transport_resume_live(void)')
    def function(signature):
        a=source.index(signature);end=source.index('{',a)+1;depth=1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        return source[a:end]+'\n'
    selectors='static void stage_hooks(void){}\nstatic bool switch_live(int level,bool lp){assert(!s_demod_p64);(void)level;(void)lp;return false;}\n'
    selectors+=''.join(function(signature) for signature in (
        'bool transport_set_p64(bool enabled)', 'bool transport_set_output(unsigned level)',
        'bool transport_kp_set(int level)', 'int transport_kp_level(void)',
        'bool transport_set_lp(bool lp)', 'bool transport_apply_auto(int level, bool lp)'))
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory);unit=path/'run.c';binary=path/'run'
        unit.write_text(prefix+tracker+words+restart+rollback+STAGES+publication+selectors+SUFFIX)
        subprocess.run([os.environ.get('CC','cc'),'-O2','-std=c11','-Wall','-Wextra','-Werror',
            '-I',str(ROOT/'main'),str(unit),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)

if __name__=='__main__':main()
