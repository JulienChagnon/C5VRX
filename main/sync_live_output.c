/* C5VRX by Twotoz/contributors. Camera-period increments run entirely in
 * the alarm ISR: no task refill or flash planner is needed during holdover.
 * Vertical tips separated by4.7us share a bounded ISR, exposing real DAC data
 * in every gap. No hard WCET or physical monitor-lock claim follows.
 *
 * One owner (2026-10-18): the camera-phase guard (sync_phase_guard.h) alone
 * moves this grid - acquisition, H/V re-anchors, trims, the holdover PLL,
 * per-type repair trust, camera presence and the PAL/NTSC choice - from the
 * same native pulses. The probe's strict H/V model no longer drives output:
 * it never qualified a camera on the board and duplicated every decision. */
#include "sync_live_output.h"
#include "sync_edge_clock.h"
#include "sync_live_clock.h"
#include "sync_phase_guard.h"
#include "sync_clock_pll.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_clk.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include <stdlib.h>
#include <string.h>

#define SLO_MASK UINT32_C(0x01801b00)
#define SLO_LEAD 240u
#define SLO_LATE 160u
/* Flag path: at most 1us late and at least 1us of useful tip remaining.
 * All widths terminate on the original absolute end, including late arrivals. */
#define SLO_FLAG_LATE 40u
#define SLO_FLAG_MIN 20u
#define SLO_PHASE_WINDOW 80u
#define SLO_BATCH_CYCLES (224u*240u)
#define SLO_CATCHUP_LEAD 80u /*4us leaves driver/ISR epilogue time before retry*/
/* Camera presence: camera H the guard holds in phase (its per-line phase
 * histogram stands clearly above the noise level, which random static
 * cannot build) within one field. Native pulses reach the task in
 * batches, so the hold covers one field. */
#define SLO_CAMERA_HOLD 400000u /* 20ms */
/* Line periods for the guard's other-standard histogram (ticks). */
#define SLO_LINE_PAL 1280
#define SLO_LINE_NTSC 1271
/* Per-type repair authority (operator review 2026-10-18): a synthetic H tip
 * needs camera H seen in phase on this grid, a V/EQ/broad tip additionally
 * camera V confirmed at this grid's broad slot, both within the holdover
 * window. Without that evidence nothing is clamped: untouched live CVBS
 * beats a possibly conflicting second sync train. 5 s of holdover drifts
 * < 5 us with the holdover PLL. */
#define SLO_TRUST_TICKS 100000000u
/* Same 9-ms delivery bound as the native observer model. This measures
 * source coverage independently of whether H/V can be found in the samples. */
#define SLO_OBSERVER_HOLD 180000u
typedef struct {uint64_t start;uint16_t width;uint8_t kind;uint16_t index;uint64_t half;} slo_tip_t;
typedef struct {
    sync_live_gate_t *planner;
    slc_t clock;
    slo_tip_t tip;
    uint32_t saved_mux[6],saved_pad[6],generation;
    slo_live_fn live;slo_native_fn native;void *native_arg;
    slo_source_fn source;
    uint32_t source_epoch;
    uint64_t rise,alarm_tick;
    bool flag,source_seen,low_seen,last_high,rise_valid;
    bool native_authority,native_inverted;
    volatile uint32_t publication;
    uint32_t applied;
    slc_t pending_clock;
    slo_tip_t pending_tip;
    slg_standard_t standard;
    uint8_t pending_mode;
    int32_t pending_shift;
    uint32_t pending_whole;
    uint16_t pending_increment,pending_denominator;
    /* Long-baseline camera period; 0 until the PLL has one. */
    scp_t pll;
    uint64_t pll_period_q;
    /* Task-owned camera-phase guard; the ISR reads only `yield`. */
    spg_t guard;
    uint64_t guard_after,v_after,last_broad;
    volatile bool yield;
    /* Task writes; the ISR needs both the 32-bit tick and the task's
     * freshness flag, so a torn or wrapped tick never reads as present. */
    volatile uint32_t camera_tick32;
    volatile bool camera_fresh;
    /* Task-owned evidence ages (last in-phase camera H, last confirmed V);
     * the ISR reads only the two flags. */
    uint64_t h_trust_tick,v_trust_tick;
    volatile bool h_trusted,v_trusted;
    uint64_t observation_end;
    uint64_t grid_after; /* ISR: first source timestamp described by this grid */
    volatile uint32_t observation_tick32;
    volatile bool observation_seen;
    slo_status_t status;
    bool saved,configured,quarantined;
} slo_t;
static void slo_trust(slo_t *s,uint64_t now);

static inline __attribute__((always_inline)) bool observer_current(const slo_t *s,uint64_t now)
{
    return s->observation_seen && (uint32_t)((uint32_t)now-s->observation_tick32)<=SLO_OBSERVER_HOLD;
}

/* Task-side revocation: fresh samples cannot revive old H/V authority after
 * a coverage gap. Keep the clock, but reacquire on post-gap source data. */
static void observer_lost(slo_t *s,uint64_t now)
{
    s->native_authority=false;__sync_synchronize();
    s->observation_seen=false;s->h_trust_tick=s->v_trust_tick=0;
    s->h_trusted=s->v_trusted=false;s->camera_fresh=false;
    s->status.h_trusted=s->status.v_trusted=s->status.camera_present=s->status.observer_fresh=false;
    s->status.guard_yield=false;
    s->guard=(spg_t){0};s->yield=false;s->pll.active=false;
    s->source_seen=false;s->low_seen=false;s->rise_valid=false;
    s->guard_after=s->v_after=now;s->last_broad=0;
    ++s->status.observer_gaps;
}
enum {SLO_PUB_RESEED=1,SLO_PUB_JUMP_H,SLO_PUB_JUMP_V,SLO_PUB_PERIOD};
static slo_t *s_output;
static DRAM_ATTR volatile unsigned s_output_phase;
void sync_live_output_phase(unsigned phase)
{ s_output_phase=phase<6u?phase:0u; }
static const unsigned pins[6]={23,24,11,12,8,9};
static const uint32_t pads[6]={IO_MUX_GPIO23_REG,IO_MUX_GPIO24_REG,IO_MUX_GPIO11_REG,
    IO_MUX_GPIO12_REG,IO_MUX_GPIO8_REG,IO_MUX_GPIO9_REG};

static void IRAM_ATTR fail(slo_t *s,uint32_t code)
{
    /* Never energize pads here if ownership has already changed. Normal
     * pulse completion always enables them before any next decision. */
    s->status.fault=code;s->status.active=false;
    (void)sync_edge_clock_alarm_off();
}

/* GPIO2 mirrors internal DATA0 even while DAC OE is suppressed. An active
 * sync level already present at admission cannot become evidence until a new
 * inactive/active transition is seen. Every source epoch, marker or invalidation clears it. */
/* True when the comparator shows sync level now, or the source cannot be
 * read (fail safe: never clamp over an unknown camera state). */
static bool IRAM_ATTR flag_level(slo_t *s)
{
    uint32_t epoch=0;
    if(!s->source(s->native_arg,&epoch))return true;
    return ((REG_READ(GPIO_IN_REG)&(1u<<2))!=0)==s->native_inverted;
}

static bool IRAM_ATTR flag_native(slo_t *s,uint64_t now,const slo_tip_t *tip)
{
    if(!s->native_authority)return false;
    uint32_t epoch=0;
    if(!s->source(s->native_arg,&epoch)) {
        s->source_seen=false;s->low_seen=false;s->rise_valid=false;
        return false;
    }
    bool high=((REG_READ(GPIO_IN_REG)&(1u<<2))!=0)==s->native_inverted;
    if(!s->source_seen || epoch!=s->source_epoch) {
        s->source_seen=true;s->source_epoch=epoch;
        s->low_seen=!high;s->last_high=high;s->rise_valid=false;
        return false;
    }
    if(!high)s->low_seen=true;
    if(high && !s->last_high && s->low_seen) {
        s->rise=now;s->rise_valid=true;s->low_seen=false;
    }
    s->last_high=high;
    if(!high || !s->rise_valid)return false;
    uint64_t distance=s->rise>=tip->start?s->rise-tip->start:tip->start-s->rise;
    return distance<=SLO_PHASE_WINDOW && now<tip->start+tip->width;
}

static bool IRAM_ATTR output_alarm_run(slo_t *s,uint64_t count,uint32_t batch_start)
{
    uint32_t n=0;
    for(;;) {
        if(!s->status.active)return false;
        if(!s->live(s->generation)){fail(s,1);return false;}
        slo_tip_t tip=s->tip;
        uint64_t now;
        if(!sync_edge_clock_read(&now)){fail(s,3);return false;}
        if(!n) {
            uint64_t scheduled=s->alarm_tick;
            uint64_t delay=count>=scheduled?count-scheduled:0u;
            uint64_t callback=now>=count?now-count:0u;
            if(delay>s->status.max_irq_delay_ticks)
                s->status.max_irq_delay_ticks=delay>UINT32_MAX?UINT32_MAX:(uint32_t)delay;
            if(callback>s->status.max_callback_ticks)
                s->status.max_callback_ticks=callback>UINT32_MAX?UINT32_MAX:(uint32_t)callback;
        }
        /* No repair authority: leave CVBS alone and skip the early-alarm
         * spin. Still advance the clock and consume pending publications. */
        if(now<tip.start) {
            if(!s->native_authority)goto advance;
            if(s->yield){++s->status.yielded;goto advance;}
            if(!(tip.kind==SLG_H?s->h_trusted:s->v_trusted)) {
                ++s->status.camera_yields;goto advance;
            }
            if(s->flag) {
                uint32_t epoch;
                if(!s->source(s->native_arg,&epoch)){++s->status.source_yields;goto advance;}
                if(!observer_current(s,now)){++s->status.observer_yields;goto advance;}
            }
        }
        bool native=false;
        uint32_t wait=esp_cpu_get_cycle_count(),waited=0;
        if(now<tip.start) {
            while(now<tip.start) {
                if(s->flag)native=flag_native(s,now,&tip);
                if(esp_cpu_get_cycle_count()-wait>(SLO_LEAD+SLO_LATE)*12u ||
                   !sync_edge_clock_read(&now)){fail(s,4);return false;}
            }
            waited=esp_cpu_get_cycle_count()-wait;
            s->status.wait_cycles+=waited;
        }
        uint64_t late=now-tip.start;
        if(late>s->status.max_late_ticks)
            s->status.max_late_ticks=late>UINT32_MAX?UINT32_MAX:(uint32_t)late;
        if(late>(s->flag?SLO_FLAG_LATE:SLO_LATE) ||
           (s->flag && late+SLO_FLAG_MIN>=tip.width)) {
            ++s->status.missed;s->status.miss_kind=tip.kind;
            unsigned phase=s_output_phase;
            if(phase>=6u)phase=0u;
            s->status.miss_phase=phase;
            if(s->status.misses_by_phase[phase]!=UINT32_MAX)++s->status.misses_by_phase[phase];
            goto advance;
        }
        if(s->flag) {
            uint32_t epoch;
            if(!s->source(s->native_arg,&epoch)){++s->status.source_yields;goto advance;}
            if(!observer_current(s,now)){++s->status.observer_yields;goto advance;}
            native=flag_native(s,now,&tip);
        }
        else native=s->native && s->native(tip.start,tip.width,(slg_kind_t)tip.kind,s->native_arg);
        if(!s->status.active || !s->live(s->generation)){fail(s,1);return false;}
        const bool trusted=tip.kind==SLG_H?s->h_trusted:s->v_trusted;
        /* An asserted live sync level always owns this interval. Historical
         * camera presence cannot grant permission to overwrite that level. */
        if(!native && s->flag && s->native_authority && trusted)native=flag_level(s);
        if(native){++s->status.suppressed;s->status.suppressed_wait_cycles+=waited;}
        else if(!s->native_authority){}
        else if(s->yield)++s->status.yielded;
        else if(!trusted)++s->status.camera_yields;
        else {
            REG_WRITE(GPIO_ENABLE_W1TC_REG,SLO_MASK);
            const uint32_t clamp=esp_cpu_get_cycle_count();
            bool camera_owned=false;
            if(s->flag) {
                /* Restore native video promptly when a correctly phased source
                 * edge arrives; never extend a late repair beyond tip end. */
                uint32_t hold=esp_cpu_get_cycle_count();
                while(now<tip.start+tip.width) {
                    /* Native level or lost witness ownership immediately ends
                     * a repair, even after the camera-presence timer expired. */
                    if(flag_native(s,now,&tip) || flag_level(s)){camera_owned=true;break;}
                    if(esp_cpu_get_cycle_count()-hold>(uint32_t)tip.width*12u ||
                       !sync_edge_clock_read(&now))break;
                }
            } else {
                uint32_t start=esp_cpu_get_cycle_count();
                while(esp_cpu_get_cycle_count()-start<(uint32_t)tip.width*12u){}
            }
            REG_WRITE(GPIO_ENABLE_W1TS_REG,SLO_MASK);
            s->status.hold_cycles+=esp_cpu_get_cycle_count()-clamp;
            /* A sliver the camera's own pulse cut short is its pulse. */
            if(camera_owned)++s->status.suppressed;
            else ++s->status.repaired;
        }
advance:
        ++n;
        slg_pulse_t pulse;
        if(!slc_next(&s->clock,&pulse)){++s->status.queue_low;fail(s,2);return false;}
        slo_tip_t next={pulse.start,(uint16_t)pulse.width,(uint8_t)pulse.kind,pulse.index,pulse.half};
        /* Single-core task publication cannot run concurrently with ISR.
         * An interrupted writer is refused; no ISR spins on task state. */
        uint32_t publication=s->publication;
        if(!(publication&1u) && publication!=s->applied) {
            __sync_synchronize();
            slc_t candidate=s->pending_clock;slo_tip_t candidate_tip=s->pending_tip;
            if(publication==s->publication) {
                s->applied=publication;
                uint8_t mode=s->pending_mode;
                if(mode==SLO_PUB_JUMP_H) {
                    /* A camera-phase re-anchor shifts the whole grid (H and
                     * V together) on the next tip; an expired tip is skipped
                     * by admission, never emitted late. */
                    int32_t shift=s->pending_shift;
                    if(shift>=0) {next.start+=(uint32_t)shift;s->clock.tick+=(uint32_t)shift;}
                    else if(next.start>(uint32_t)-shift && s->clock.tick>(uint32_t)-shift) {
                        next.start-=(uint32_t)-shift;s->clock.tick-=(uint32_t)-shift;
                    }
                }
                if(mode==SLO_PUB_PERIOD) {
                    s->clock.whole=s->pending_whole;
                    s->clock.increment=s->pending_increment;
                    s->clock.denominator=s->pending_denominator;s->clock.remainder=0;
                } else if(mode==SLO_PUB_JUMP_V) {
                    /* Move the field position by whole halflines; physical
                     * H tips keep their absolute half parity. */
                    int32_t index=((int32_t)s->clock.index-s->pending_shift)%(int32_t)s->clock.halves;
                    if(index<0)index+=s->clock.halves;
                    s->clock.index=(uint16_t)index;
                } else if(mode==SLO_PUB_RESEED && candidate_tip.start>now+SLO_LEAD) {
                    /* Other standard, nominal phase, no authority: the guard
                     * re-acquires the camera on it before anything is
                     * clamped. Edges of the old grid are discarded. */
                    s->clock=candidate;next=candidate_tip;
                    s->standard=candidate.halves==625u?SLG_PAL:SLG_NTSC;
                    s->source_seen=false;s->low_seen=false;s->rise_valid=false;
                }
                s->grid_after=next.start;
            }
        }
        s->tip=next;
        if(!sync_edge_clock_read(&now)){fail(s,3);return false;}
        if(next.start>now && next.start-now>SLO_LEAD+60u) {
            if(n>1u)++s->status.batches;
            s->alarm_tick=next.start-SLO_LEAD;
            if(!sync_edge_clock_alarm_at(s->alarm_tick))fail(s,7);
            return false;
        }
        if(n>=8u || esp_cpu_get_cycle_count()-batch_start>SLO_BATCH_CYCLES) {
            /* A cache/critical-section delay can leave several expired tips.
             * Keep the exact NEXT tip/iterator, restore task execution, then
             * skip the remaining backlog in another bounded ISR. Admission
             * still rejects every expired tip; no stale pulse is replayed. */
            if(now>UINT64_MAX-SLO_CATCHUP_LEAD){fail(s,7);return false;}
            if(s->status.budget_yields!=UINT32_MAX)++s->status.budget_yields;
            s->alarm_tick=now+SLO_CATCHUP_LEAD;
            if(!sync_edge_clock_alarm_at(s->alarm_tick))fail(s,7);
            return false;
        }
    }
}

/* Aggregate CPU occupancy (PR #218 review): total and worst ISR cycles,
 * the early-alarm spin, the part of it a native pulse then made useless,
 * and the clamp holds. Counters only; nothing is printed from the ISR. */
static bool IRAM_ATTR output_alarm(uint64_t count,void *arg)
{
    slo_t *s=arg;
    const uint32_t start=esp_cpu_get_cycle_count();
    const bool again=output_alarm_run(s,count,start);
    const uint32_t spent=esp_cpu_get_cycle_count()-start;
    ++s->status.isr_calls;s->status.isr_cycles+=spent;
    if(spent>s->status.max_isr_cycles)s->status.max_isr_cycles=spent;
    return again;
}

bool sync_live_output_poll(void)
{
    slo_t *s=s_output;
    if(!s || !s->status.active || s->quarantined)return false;
    if(!s->live(s->generation))return false;
    return true;
}

bool sync_live_output_end(void)
{
    slo_t *s=s_output;
    if(!s)return true;
    if(s->quarantined)return false;
    s->status.active=false;__sync_synchronize();
    if(!sync_edge_clock_alarm_bind(NULL,NULL)){s->quarantined=true;return false;}
    if(s->configured) {
        REG_WRITE(GPIO_ENABLE_W1TS_REG,SLO_MASK);
        for(unsigned i=0;i<6u;++i) {
            REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG+4u*pins[i],s->saved_mux[i]);
            REG_WRITE(pads[i],s->saved_pad[i]);
        }
        for(unsigned i=0;i<6u;++i)
            if(REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG+4u*pins[i])!=s->saved_mux[i] ||
               REG_READ(pads[i])!=s->saved_pad[i]){s->quarantined=true;return false;}
        if((REG_READ(GPIO_ENABLE_REG)&SLO_MASK)!=SLO_MASK){s->quarantined=true;return false;}
    }
    sync_live_gate_destroy(s->planner);free(s);s_output=NULL;return true;
}

static bool output_begin(slg_standard_t fallback,
    uint32_t generation,slo_live_fn live,slo_native_fn native,slo_source_fn source,void *arg)
{
    if(s_output || !live || !live(generation) || esp_clk_cpu_freq()!=240000000)return false;
    uint64_t now;
    if(!sync_edge_clock_read(&now) || now>UINT64_MAX-20000u)return false;
    slo_t *s=heap_caps_calloc(1,sizeof(*s),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!s)return false;
    s_output=s;s->live=live;s->generation=generation;s->native=native;s->native_arg=arg;
    s->source=source;s->flag=source!=NULL;
    s->native_inverted=true; /* authority comes from the guard only */
    const slg_config_t config={SLO_LEAD,s->flag?SLO_FLAG_LATE:SLO_LATE,
        s->flag?40u:100u,s->flag?40u:100u,UINT32_MAX,64u};
    s->planner=sync_live_gate_create(&config);
    if(!s->planner)goto failed;
    if(!sync_live_gate_nominal(s->planner,fallback,now+20000u,0u,0u,now))goto failed;
    if(!sync_live_gate_arm(s->planner))goto failed;
    slg_clock_seed_t seed;slg_pulse_t first;
    if(!sync_live_gate_export(s->planner,&seed) || !slc_init(&s->clock,&seed) ||
       !slc_next(&s->clock,&first))goto failed;
    s->tip=(slo_tip_t){first.start,(uint16_t)first.width,(uint8_t)first.kind,first.index,first.half};
    s->standard=seed.standard;
    s->status.standard=(uint8_t)s->standard;
    /* Retain the task-owned planner for allocation-free timing updates.
     * ISR execution consumes only the exported incremental clock. */
    if((REG_READ(GPIO_ENABLE_REG)&SLO_MASK)!=SLO_MASK)goto failed;
    for(unsigned i=0;i<6u;++i) {
        uint32_t reg=GPIO_FUNC0_OUT_SEL_CFG_REG+4u*pins[i];
        s->saved_mux[i]=REG_READ(reg);s->saved_pad[i]=REG_READ(pads[i]);
        if((s->saved_mux[i]&GPIO_FUNC0_OUT_SEL_M)!=(s->flag && !i?256u:48u+i) ||
           (s->saved_mux[i]&(GPIO_FUNC0_OUT_INV_SEL|GPIO_FUNC0_OE_INV_SEL)))goto failed;
    }
    if(s->flag && (REG_READ(GPIO_OUT_REG)&(1u<<23)))goto failed;
    s->saved=true;s->status.active=true;
    if(!sync_live_output_poll())goto failed;
    if(!sync_edge_clock_alarm_bind(output_alarm,s))goto failed;
    s->configured=true;
    for(unsigned i=0;i<6u;++i) {
        /* The flag owner proves GPIO23's complete clamp identity on every
         * admission. Keep its IOMUX bits unchanged while borrowing its OE. */
        if(!s->flag || i)REG_WRITE(pads[i],s->saved_pad[i]&~(FUN_PU|FUN_PD));
        REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG+4u*pins[i],s->saved_mux[i]|GPIO_FUNC0_OE_SEL);
    }
    __sync_synchronize();
    s->alarm_tick=s->tip.start-SLO_LEAD;
    if(!live(generation) || !sync_edge_clock_alarm_at(s->alarm_tick))goto failed;
    return true;
failed:
    (void)sync_live_output_end();return false;
}

void sync_live_output_status(slo_status_t *out)
{
    if(!out)return;
    slo_t *s=s_output;
    if(!s){*out=(slo_status_t){0};return;}
    /* The single-core ISR updates 64-bit cycle totals in two 32-bit stores.
     * Retry if it interrupted this task-side snapshot; no IRQ masking or
     * extra interrupt-path work is needed. Calls advances on every return. */
    uint32_t calls;
    do {
        calls=s->status.isr_calls;__sync_synchronize();
        *out=s->status;__sync_synchronize();
    } while(calls!=s->status.isr_calls);
}
void sync_live_output_native_polarity(bool inverted)
{
    slo_t *s=s_output;
    if(!s || s->native_inverted==inverted)return;
    s->native_inverted=inverted;
    s->source_seen=false;s->low_seen=false;s->rise_valid=false;
}

bool sync_live_output_begin(slg_standard_t fallback,
    uint32_t generation,slo_live_fn live,slo_native_fn native,void *arg)
{ return output_begin(fallback,generation,live,native,NULL,arg); }

bool sync_live_output_flag_begin(slg_standard_t fallback,
    uint32_t generation,slo_live_fn live,slo_source_fn source,void *arg)
{ return source && output_begin(fallback,generation,live,NULL,source,arg); }

/* The camera runs the other standard: restart on its nominal grid. */
static bool slo_reseed(slo_t *s,uint64_t now)
{
    if((s->publication&1u) || s->publication!=s->applied || now>UINT64_MAX-20000u)return false;
    slg_standard_t other=s->standard==SLG_PAL?SLG_NTSC:SLG_PAL;
    slg_clock_seed_t seed;slc_t candidate;slg_pulse_t pulse;
    if(!sync_live_gate_nominal(s->planner,other,now+20000u,0u,0u,now) || !sync_live_gate_arm(s->planner) ||
       !sync_live_gate_export(s->planner,&seed) || !slc_init(&candidate,&seed) ||
       !slc_next(&candidate,&pulse))return false;
    s->native_authority=false;
    ++s->publication;__sync_synchronize();
    s->pending_mode=SLO_PUB_RESEED;s->pending_clock=candidate;
    s->pending_tip=(slo_tip_t){pulse.start,(uint16_t)pulse.width,(uint8_t)pulse.kind,pulse.index,pulse.half};
    __sync_synchronize();++s->publication;
    s->guard=(spg_t){0};s->yield=false;s->guard_after=s->v_after=now+20000u;
    s->pll.active=false;s->pll_period_q=0;
    s->h_trust_tick=s->v_trust_tick=0;s->camera_fresh=false;
    ++s->status.std_switches;
    return true;
}

/* Bounded task-side publication; refused while another is unapplied so a
 * re-anchor can never overwrite a pending reseed. */
static bool guard_publish(slo_t *s,uint8_t mode,int32_t value)
{
    uint64_t now;
    if((s->publication&1u) || s->publication!=s->applied || !sync_edge_clock_read(&now))return false;
    ++s->publication;__sync_synchronize();
    s->pending_mode=mode;s->pending_shift=value;
    __sync_synchronize();++s->publication;
    /* Pulses observed before the ISR applies it belong to the old grid.
     * A trim moves it by a few ticks, well inside one histogram bin. */
    if(mode==SLO_PUB_JUMP_V)s->v_after=now+20000u;
    else if(s->guard.yield_h)s->guard_after=now+20000u;
    if(mode==SLO_PUB_JUMP_V)++s->status.guard_v_jumps;
    else if(mode==SLO_PUB_JUMP_H && s->guard.yield_h)++s->status.guard_h_jumps; /* trims counted apart */
    return true;
}

static void slo_trust(slo_t *s,uint64_t now)
{
    bool h=s->native_authority && s->h_trust_tick && now>=s->h_trust_tick &&
        now-s->h_trust_tick<=SLO_TRUST_TICKS;
    bool v=h && s->v_trust_tick && now>=s->v_trust_tick && now-s->v_trust_tick<=SLO_TRUST_TICKS;
    s->h_trusted=h;s->v_trusted=v;
    s->status.h_trusted=h;s->status.v_trusted=v;
}

static bool guard_publish_period(slo_t *s,uint64_t period_q)
{
    if((s->publication&1u) || s->publication!=s->applied)return false;
    ++s->publication;__sync_synchronize();
    s->pending_mode=SLO_PUB_PERIOD;
    s->pending_whole=(uint32_t)(period_q/SCP_DEN);
    s->pending_increment=(uint16_t)(period_q%SCP_DEN);s->pending_denominator=SCP_DEN;
    __sync_synchronize();++s->publication;
    ++s->status.pll_updates;
    return true;
}

void sync_live_output_native_pulse(uint64_t tick,uint32_t width,bool high)
{
    slo_t *s=s_output;
    if(!s || !s->flag || !s->status.active || s->quarantined)return;
    uint64_t now;
    if(!sync_edge_clock_read(&now))return;
    /* Every completed marker-excluded run is coverage, including non-sync
     * runs in real snow. Delayed H/V must not move today's grid or renew its
     * authority. Only the phase bracket's edge uncertainty may be future. */
    if(tick>UINT64_MAX-width || (tick+width>now && tick+width-now>SLO_PHASE_WINDOW) ||
       (now>=tick+width && now-(tick+width)>SLO_OBSERVER_HOLD)) {
        ++s->status.stale_native;
        if(s->observation_seen)observer_lost(s,now);
        return;
    }
    if(s->observation_seen && now>=s->observation_end &&
       now-s->observation_end>SLO_OBSERVER_HOLD)observer_lost(s,now);
    if(!s->observation_seen || tick+width>s->observation_end) {
        s->observation_end=tick+width;s->observation_tick32=(uint32_t)(tick+width);
        __sync_synchronize();s->observation_seen=true;
    }
    /* Without authority the grid is a nominal clock that emits nothing;
     * the guard still aligns it to a coherent camera and may grant it. */
    if(high!=s->native_inverted || tick<s->guard_after || tick<s->grid_after ||
       s->publication!=s->applied || (s->publication&1u))return;
    bool h=width>=75u && width<=240u,broad=width>=250u && width<=620u;
    if(!h && !broad)return;
    slo_tip_t tip;slc_t clock;
    do {tip=s->tip;clock=s->clock;__sync_synchronize();} while(tip.start!=s->tip.start);
    if(!clock.denominator || !clock.halves)return;
    /* Halfline in 1/denominator ticks, exactly as the ISR iterator steps. */
    int64_t den=clock.denominator,half=(int64_t)clock.whole*den+clock.increment;
    int64_t delta=((int64_t)tick-(int64_t)tip.start)*den;
    spg_action_t action={SPG_NONE,0};
    if(h) {
        /* H lines sit on halves with the clock's parity. */
        int64_t base=(tip.half&1u)==clock.parity?delta:delta-half;
        int32_t offset=(int32_t)(spg_wrap(base,2*half)/den);
        action=spg_h(&s->guard,offset,(int32_t)(2*half/den),
                     s->standard==SLG_PAL?SLO_LINE_NTSC:SLO_LINE_PAL,tick);
        bool camera=spg_in_phase(&s->guard) && s->guard.h_tick==tick;
        if(camera) {
            /* In-phase camera H: present now, trusted for holdover. */
            s->h_trust_tick=tick;
            s->camera_tick32=(uint32_t)tick;__sync_synchronize();s->camera_fresh=true;
            ++s->status.camera_lines;
        }
        /* Only camera H the guard holds in phase measures the period; in
         * static a random pulse in the window is not camera evidence. */
        if(s->native_authority && camera && action.kind==SPG_NONE) {
            uint64_t period_q=scp_period_q(clock.whole,clock.increment,clock.denominator);
            /* RMT delivers historical runs. The PLL baseline belongs to the
             * observed H, not the ISR's current (possibly many lines later)
             * tip. Mixing those two times turns delivery jitter into drift. */
            int64_t dhalf=delta>=0?(delta+half/2)/half:-((-delta+half/2)/half);
            if((dhalf<0 && tip.half<(uint64_t)-dhalf) ||
               (dhalf>=0 && tip.half>UINT64_MAX-(uint64_t)dhalf))return;
            uint64_t observed_half=dhalf<0?tip.half-(uint64_t)-dhalf:tip.half+(uint64_t)dhalf;
            if(!s->pll.active || s->pll.period_q!=period_q)scp_start(&s->pll,period_q,observed_half,offset);
            else {
                scp_aligned(&s->pll);
                uint64_t estimate;
                if(s->publication==s->applied && !(s->publication&1u) &&
                   scp_estimate(&s->pll,observed_half,offset,&estimate) &&
                   guard_publish_period(s,estimate))s->pll_period_q=estimate;
            }
        }
    } else {
        if(tick<s->v_after)return;
        bool first=tick<s->last_broad || tick-s->last_broad>3000u;
        s->last_broad=tick;
        if(!first)return;
        int64_t halves=delta>=0?(delta+half/2)/half:-((-delta+half/2)/half);
        int32_t index=(int32_t)(((int64_t)tip.index+halves)%clock.halves);
        if(index<0)index+=clock.halves;
        int32_t offset=spg_wrap((int64_t)index-clock.section,clock.halves);
        action=spg_v(&s->guard,offset,tick,(uint64_t)((clock.halves*half+den/2)/den));
        if(!offset && !s->guard.yield_h && s->guard.v_good>=SPG_V_LOCK)s->v_trust_tick=tick;
    }
    if(action.kind==SPG_STD) {
        /* The camera's line period is the other standard's. */
        if(slo_reseed(s,tick))return;
    } else if(action.kind==SPG_JUMP_H) {
        bool trim=!s->guard.yield_h;
        if(guard_publish(s,SLO_PUB_JUMP_H,action.value)) {
            if(trim){++s->status.guard_trims;scp_trim(&s->pll,action.value);spg_shift(&s->guard,action.value);}
            else {
                s->pll.active=false; /* re-anchor: the window no longer measures drift */
                s->h_trust_tick=s->v_trust_tick=0; /* moved grid: confirm before repairing */
                slo_trust(s,tick);
                spg_reset(&s->guard);
            }
        }
    } else if(action.kind==SPG_JUMP_V && guard_publish(s,SLO_PUB_JUMP_V,action.value)) {
        spg_reset_v(&s->guard); /* H positions are unchanged by a halfline index shift */
        s->v_trust_tick=0;slo_trust(s,tick);
    }
    s->yield=spg_yield(&s->guard,tick);
    /* Grant only on the grid the evidence describes: nothing unapplied. */
    if(!s->native_authority && spg_acquired(&s->guard,tick) &&
       s->publication==s->applied && !(s->publication&1u)) {
        s->native_authority=true;
        s->h_trust_tick=s->v_trust_tick=tick;
        ++s->status.guard_acquired;
    }
    slo_trust(s,now);
}

void sync_live_output_guard_tick(void)
{
    slo_t *s=s_output;
    uint64_t now;
    if(!s || !s->flag || !sync_edge_clock_read(&now))return;
    s->status.standard=(uint8_t)s->standard;
    if(s->observation_seen && !observer_current(s,now))observer_lost(s,now);
    s->status.observer_fresh=observer_current(s,now);
    s->status.sample_age_ticks=!s->observation_seen || now<s->observation_end ? UINT32_MAX :
        now-s->observation_end>UINT32_MAX?UINT32_MAX:(uint32_t)(now-s->observation_end);
    s->yield=spg_yield(&s->guard,now);
    s->status.guard_yield=s->yield;
    if(s->camera_fresh && (now<s->h_trust_tick || now-s->h_trust_tick>=SLO_CAMERA_HOLD))s->camera_fresh=false;
    s->status.camera_present=s->camera_fresh;
    slo_trust(s,now);
    s->status.pll_rejected=s->pll.rejected;
    /* Signed fine-period offset versus nominal, in parts per billion. */
    if(s->pll_period_q) {
        uint64_t nominal=s->standard==SLG_PAL?640u*(uint64_t)SCP_DEN:(5720u*(uint64_t)SCP_DEN+4u)/9u;
        s->status.pll_ppb=(int32_t)(((int64_t)s->pll_period_q-(int64_t)nominal)*1000000000/(int64_t)nominal);
    }
}

