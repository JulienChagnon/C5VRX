/* C5VRX by Twotoz and contributors: bounded input-only RMT DACbit4 probe.
 * This manual observer and its RX ISR stay in flash to preserve receiver
 * task SRAM. Cache-disabled intervals may lose observations; this module
 * supplies candidate clock evidence only; it never schedules output pulses.
 * Captures durations, not absolute GPTIMER timestamps. Bright-picture aliases
 * and generated gates are not camera evidence. Owner must exclude gates.
 * GPIO2 (unused MTMS pad) mirrors PARLIO DATA4 independently of DAC OE.
 * Only the isolated GPIO2 data bit may be driven for a private clock marker.
 * No raw IQ, LUT, DMA descriptors, PHY registers or DAC output data writes. */
#include "sync_output_probe.h"
#include "sync_edge_clock.h"
#include "sync_phase_bracket.h"
#include "sync_native_track.h"
#include "sync_native_flag_track.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/rmt_rx.h"
#include "driver/rmt_common.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

#define SOP_GPIO 2u
#define SOP_MASK (1u << SOP_GPIO)
_Static_assert((SOP_MASK & UINT32_C(0x01801b00)) == 0u,
               "native witness must be outside all six DAC output enables");
#define SOP_SYMBOLS 48u
#define SOP_WINDOW_US 50000
#define SOP_ARM_MAX_US 700 /* below1ms RMT idle; permits bounded224us output IRQ */
#define SOP_CLOCK_MIN_NS 1000u /* reject sub-us chatter; retain2.3us EQ lows */
#define SOP_MARKER_MAX_TICKS 6000u /*16us marker plus a bounded vertical ISR */
/* Two 48-symbol ping-pong blocks plus control-loop polling span ~8ms of H.
 * This bounds delivery staleness, not per-line missing-sync detection. */
#define SOP_MODEL_MAX_AGE_TICKS 180000u

#define SOP_RX_RING 12u /* RTC_FAST doubles as boot heap: 16 caused ESP_ERR_NO_MEM in control_start */
typedef struct {
    rmt_symbol_word_t symbols[SOP_SYMBOLS];
    uint8_t count;
    bool is_last,bad;
} sop_rx_batch_t;
typedef struct {
    sop_rx_batch_t batches[SOP_RX_RING];
    volatile uint32_t head,tail;
    volatile bool overflow;
} sop_rx_ring_t;
static RTC_FAST_ATTR sop_rx_ring_t s_rx_ring;

/* Allocate after receiver startup: even a small BSS increase can prevent
 * its contiguous control-task stack allocation. Keep storage alive permanently
 * once acquired, including any quarantined callback ownership. */
typedef struct {
    uint64_t rise_before,rise_after,fall_before,fall_after;
    sync_phase_bracket_t native_phase,calibrated_phase;
    sync_native_track_t native;
    sync_native_track_t inverted;
    sync_native_flag_track_t flag_native,flag_inverted;
    snt_trace_t trace;
    uint32_t native_qualified_epochs;
    bool native_phase_checked,native_reported;
    int64_t rate_started;
    uint32_t rate_callbacks;
    volatile uint32_t sequence;
    bool resident,trace_flag;
} sop_native_t;
static sop_native_t *s_native;

typedef struct {
    uint32_t callbacks, symbols, halves, epochs, endings, low, h, broad;
    uint32_t h_periods, pal32, ntsc32, broad5, broad6, pal_fields, ntsc_fields;
    uint32_t bad_data, elapsed, run_start, run_width, previous_h, train_start;
    uint32_t previous_broad, broad_start, previous_group, previous_group_count;
    uint32_t h_train, broad_train;
    uint32_t generation;
    uint32_t discard_before,first_duration;
    bool first_level, marker_required, marker_invalid;
    uint32_t prefix[3];
    unsigned prefix_n;
    bool (*live)(uint32_t);
    volatile bool armed, stop, ended, cancelled, rate_limited;
    bool run_valid, run_level, run_edge;
    bool have_h, have_broad, have_group;
} sop_context_t;

/* Keep ISR pointers/buffers alive even if an SDK cleanup operation fails.
 * A quarantined channel prevents subsequent calls from reusing its storage. */
static DRAM_ATTR sop_context_t s_probe;
static DRAM_ATTR rmt_symbol_word_t s_symbols[SOP_SYMBOLS];
static rmt_channel_handle_t s_quarantined;
static bool s_busy;
static bool s_clock_fault;
/* ISR consumers use only this DRAM publication, never the task-owned session
 * or private marker. An epoch change invalidates any previously sampled edge. */
static DRAM_ATTR volatile uint32_t s_source_epoch;
static DRAM_ATTR volatile bool s_source_ready;
static void sop_source_pause(void)
{
    s_source_ready=false;
    __sync_synchronize();
    ++s_source_epoch;
    __sync_synchronize();
}
bool IRAM_ATTR sync_output_live_source(void *arg,uint32_t *epoch)
{
    (void)arg;
    uint32_t before=s_source_epoch;
    __sync_synchronize();
    bool ready=s_source_ready && s_probe.armed && !s_probe.stop && !s_probe.ended;
    __sync_synchronize();
    if(epoch)*epoch=before;
    return ready && before==s_source_epoch;
}

static inline bool sop_near(uint32_t actual, uint32_t expected, uint32_t tolerance)
{
    return actual >= expected - tolerance && actual <= expected + tolerance;
}

static void sop_group(sop_context_t *s)
{
    if (!s->have_broad) return;
    uint32_t n = s->broad_train;
    if (n == 5u) ++s->broad5;
    if (n == 6u) ++s->broad6;
    if ((n == 5u || n == 6u) && s->have_group && s->previous_group_count == n) {
        uint32_t delta = s->broad_start - s->previous_group;
        if (n == 5u && sop_near(delta, 400000u, 1000u)) ++s->pal_fields;
        if (n == 6u && sop_near(delta, 333667u, 1000u)) ++s->ntsc_fields;
    }
    s->have_group = n == 5u || n == 6u;
    s->previous_group = s->broad_start;
    s->previous_group_count = n;
    s->have_broad = false;
}

/* Resident flag sessions forward every native pulse (common clock) to the
 * live output's camera-phase guard. Task context only (drain). */
static void (*s_pulse_hook)(uint64_t tick,uint32_t width,bool high);
void sync_output_live_pulse_hook(void (*hook)(uint64_t tick,uint32_t width,bool high))
{ s_pulse_hook=hook; }

static void sop_native_run(sop_context_t *s,uint32_t start,uint32_t width,bool inverted)
{
    if(start<s->discard_before)return; /* private marker/handoff is not camera sync */
    if(s_native && s->marker_required && !s_native->native_phase_checked && s->prefix_n==3u) {
        uint32_t rise=s->prefix[0]+s->prefix[1];
        s_native->native_phase=sync_phase_bracket(rise,rise+s->prefix[2],
            s_native->rise_before,s_native->rise_after,s_native->fall_before,s_native->fall_after);
        s_native->native_phase_checked=true;
    }
    sync_phase_bracket_t phase=s_native?s_native->native_phase:(sync_phase_bracket_t){0};
    if(!phase.valid && s_native && s_native->calibrated_phase.valid)
        phase=s_native->calibrated_phase;
    if(s_native && s->marker_required && phase.valid) {
        int64_t offset=phase.lo+(phase.hi-phase.lo)/2;
        if(offset>=-(int64_t)start && offset<=INT64_MAX-(int64_t)start) {
            if(s_native->trace_flag && inverted && offset+start<=UINT32_MAX)
                snt_trace_flag_high(&s_native->trace,s->epochs,(uint32_t)(offset+start),width);
            if(s_native->trace_flag) {
                sync_native_flag_high(inverted?&s_native->flag_inverted:&s_native->flag_native,
                                      (uint64_t)(offset+start),width);
                if(s_native->resident && s_pulse_hook)s_pulse_hook((uint64_t)(offset+start),width,inverted);
            }
            else sync_native_low(inverted?&s_native->inverted:&s_native->native,(uint64_t)(offset+start),width);
            bool qualified=s_native->native.valid || s_native->inverted.valid;
            if(qualified && !s_native->native_reported) {
                ++s_native->native_qualified_epochs;s_native->native_reported=true;
            }
        }
    }
}

static void sop_low(sop_context_t *s, uint32_t start, uint32_t width)
{
    if(start<s->discard_before)return;
    if(s_native && s_native->resident)snt_trace_low(&s_native->trace,s->epochs,start,width);
    sop_native_run(s,start,width,false);
    ++s->low;
    if (width >= 76u && width <= 112u) {
        ++s->h;
        sop_group(s);
        if (s->have_h) {
            uint32_t delta = start - s->previous_h;
            if (delta >= 1260u && delta <= 1290u) {
                ++s->h_periods;
                if (++s->h_train == 32u) {
                    uint32_t fit = start - s->train_start;
                    if (sop_near(fit, 40960u, 32u)) ++s->pal32;
                    if (sop_near(fit, 40676u, 32u)) ++s->ntsc32;
                    s->train_start = start; s->h_train = 0u;
                }
            } else { s->train_start = start; s->h_train = 0u; }
        } else { s->train_start = start; s->h_train = 0u; }
        s->previous_h = start; s->have_h = true;
    } else if (width >= 520u && width <= 580u) {
        ++s->broad;
        s->have_h = false; s->h_train = 0u;
        if (!s->have_broad || !sop_near(start - s->previous_broad, 638u, 8u)) {
            sop_group(s);
            s->broad_start = start; s->broad_train = 0u;
        }
        s->have_broad = true; s->previous_broad = start; ++s->broad_train;
    }
}

/* An RMT symbol has two ordered level durations. Merge identical adjacent
 * levels, including across callbacks. Never classify a truncated first run
 * or the unfinished final run. 50ms fits comfortably in32-bit tick counters. */
static bool sop_half(sop_context_t *s, uint32_t duration, bool level)
{
    if (!duration) return false;  /* RMT end marker, not a zero-length edge. */
    if(s->prefix_n<3u)s->prefix[s->prefix_n++]=duration;
    if(!s->first_duration){
        s->first_duration=duration;s->first_level=level;
        if(s->marker_required && (!level || duration<280u || duration>SOP_MARKER_MAX_TICKS)) {
            s->marker_invalid=true;s->stop=true;return false;
        }
    }
    if (s->elapsed > UINT32_MAX - duration || s->run_width > UINT32_MAX - duration) {
        ++s->bad_data; s->stop = true; return false;
    }
    if (!s->run_valid) {
        s->run_valid = true; s->run_level = level;
        s->run_start = s->elapsed; s->run_width = 0u; s->run_edge = false;
    } else if (level != s->run_level) {
        if (s->run_edge) {
            if(!s->run_level)sop_low(s,s->run_start,s->run_width);
            else sop_native_run(s,s->run_start,s->run_width,true);
        }
        s->run_start = s->elapsed; s->run_width = 0u;
        s->run_level = level; s->run_edge = true;
    }
    s->run_width += duration; s->elapsed += duration; ++s->halves;
    return true;
}

static void sop_parse_batch(sop_context_t *s,const sop_rx_batch_t *b)
{
    if(s->stop)return;
    if(b->bad) {
        ++s->bad_data; s->stop = true; return;
    }
    s->symbols += b->count;
    bool marker = false;
    for (size_t i = 0; i < b->count; ++i) {
        rmt_symbol_word_t word = b->symbols[i];
        if (!sop_half(s, word.duration0, word.level0)) { marker = true; break; }
        if (!sop_half(s, word.duration1, word.level1)) { marker = true; break; }
    }
    if (marker && !b->is_last && !s->stop) {
        ++s->bad_data; s->stop = true;
    }
    if (b->is_last) {
        ++s->endings; s->ended = true;
        sop_group(s);
        /* No evidence may bridge an idle stop and a later receive epoch. */
        s->run_valid = s->have_h = s->have_broad = s->have_group = false;
        s->h_train = s->broad_train = 0u;
    }
}

static void sop_drain(sop_context_t *s)
{
    /* IRQs may publish while we parse. Leave new batches for the next poll
     * instead of chasing a noisy producer while holding the PHY lease. */
    const uint32_t head=s_rx_ring.head;
    while(s_rx_ring.tail!=head) {
        uint32_t tail=s_rx_ring.tail;
        __sync_synchronize();
        sop_rx_batch_t batch=s_rx_ring.batches[tail%SOP_RX_RING];
        __sync_synchronize();
        s_rx_ring.tail=tail+1u;
        sop_parse_batch(s,&batch);
    }
}

/* Keep the RMT RX callback tiny and IRAM-resident so ESP-IDF's critical
 * section inside rmt_rx_default_isr never delays GPTIMER sync output alarms. */
static bool IRAM_ATTR sop_received_inner(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t *event, void *arg)
{
    (void)channel;
    sop_context_t *s = arg;
    if (!s->armed || s->stop) return false;
    int64_t rate_now=esp_timer_get_time();
    if(s_native && s_native->resident && rate_now-s_native->rate_started>=SOP_WINDOW_US) {
        s_native->rate_started=rate_now;s_native->rate_callbacks=0;
    }
    if((s_native && s_native->resident?s_native->rate_callbacks:s->callbacks)>=256u){s->rate_limited=true;s->stop=true;return false;}
    if(s_native && s_native->resident)++s_native->rate_callbacks;
    if (!s->live(s->generation)) {
        s->cancelled = true; s->stop = true; return false;
    }
    ++s->callbacks;
    uint32_t head=s_rx_ring.head;
    if (!event || event->num_symbols > SOP_SYMBOLS ||
        (event->num_symbols && !event->received_symbols) || s->ended) {
        ++s->bad_data; s->stop = true; return false;
    }
    if (s_rx_ring.overflow || head - s_rx_ring.tail >= SOP_RX_RING) {
        s_rx_ring.overflow = true; return false;
    }
    sop_rx_batch_t *b=&s_rx_ring.batches[head%SOP_RX_RING];
    b->count=(uint8_t)event->num_symbols;
    b->is_last=event->flags.is_last;
    b->bad=false;
    for(size_t i=0;i<event->num_symbols;++i)b->symbols[i]=event->received_symbols[i];
    if(event->flags.is_last)s->ended=true;
    __sync_synchronize();
    s_rx_ring.head=head+1u;
    return false;
}

/* Single-core C5: snapshots reject a callback-interrupted copy. All callback
 * storage remains owned even after a failed disable/detach/delete. */
static bool IRAM_ATTR sop_received(rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *event,void *arg)
{
    if(s_native)++s_native->sequence;
    __sync_synchronize();
    bool wake=sop_received_inner(channel,event,arg);
    __sync_synchronize();
    if(s_native)++s_native->sequence;
    return wake;
}

static void sop_epoch_mode(sop_context_t *s,bool preserve_native)
{
    s_rx_ring.head=s_rx_ring.tail=0u;
    s_rx_ring.overflow=false;
    ++s->epochs;
    s->elapsed = s->run_width = 0u;
    s->run_valid = s->run_edge = s->have_h = s->have_broad = s->have_group = false;
    s->h_train = s->broad_train = 0u; s->ended = false;
    s->prefix_n=0;s->first_duration=0;s->first_level=false;
    memset(s->prefix,0,sizeof(s->prefix));
    if(s_native) {
        s_native->native_phase=(sync_phase_bracket_t){0};
        if(!preserve_native) {
            sync_native_reset(&s_native->native);
            sync_native_reset(&s_native->inverted);
            if(s_native->trace_flag) {
                sync_native_flag_init(&s_native->flag_native,&s_native->native);
                sync_native_flag_init(&s_native->flag_inverted,&s_native->inverted);
            }
        }
        s_native->native_phase_checked=false;s_native->native_reported=false;
    }
    s->discard_before=s->marker_required?UINT32_MAX:0u;
}
static void sop_epoch(sop_context_t *s)
{ sop_epoch_mode(s,false); }

typedef struct {
    uint32_t valid,refused,pending,max_span,regressions;
    bool have_origin;
    int64_t last_lo,last_hi;
} sop_clock_stats_t;

static sync_phase_bracket_t sop_score_clock(sop_clock_stats_t *stats,
    uint64_t rb,uint64_t ra,uint64_t fb,uint64_t fa)
{
    sync_phase_bracket_t r={0};
    if(s_probe.prefix_n<3u){++stats->pending;return r;}
    uint32_t rise=s_probe.prefix[0]+s_probe.prefix[1];
    r=sync_phase_bracket(rise,rise+s_probe.prefix[2],rb,ra,fb,fa);
    if(r.valid && stats->have_origin && r.lo<=stats->last_hi) {
        ++stats->regressions;r.valid=false;
    }
    if(r.valid) {
        ++stats->valid;stats->have_origin=true;
        stats->last_lo=r.lo;stats->last_hi=r.hi;
        uint32_t span=(uint32_t)(r.hi-r.lo);
        if(span>stats->max_span)stats->max_span=span;
    } else ++stats->refused;
    return r;
}

typedef struct {
    uint32_t generation;
    bool (*live)(uint32_t);
    bool edge_clock, flag_source;
    uint32_t iomux;
    uint32_t matrix;
    uint32_t enabled;
    uint32_t mirror_iomux;
    uint32_t mirror_matrix;
    uint32_t mirror_pin;
    uint32_t mirror_enable;
    uint32_t mirror_output;
    uint32_t witness_iomux;
    uint32_t witness_matrix;
    rmt_channel_handle_t channel;
    bool enabled_rx;
    bool pin_changed;
    bool clock_attempted;
    bool clock_read_ok;
    bool clock_closed;
    uint64_t clock_ticks;
    uint32_t common_filter_ns;
    int64_t arm_us;
    uint64_t rise_before;
    uint64_t rise_after;
    uint64_t fall_before;
    uint64_t fall_after;
    bool clock_link_armed;
    bool phase_scored;
    uint32_t clock_rearms;
    uint64_t epoch_quiet;
    uint64_t native_floor;
    sop_clock_stats_t clock_stats;
    sync_phase_bracket_t phase;
    const char *failure;
    esp_err_t err;
    esp_err_t cleanup;
    rmt_receive_config_t receive;
    int64_t began;
    int64_t epoch_started;
    bool resident;
    volatile bool invalidated;
    bool forget_native,fresh_h_fit;
    sync_output_live_model_t retained;
    bool (*before_close)(void);
} sop_session_t;
static sop_session_t *s_session;
static bool sop_session_end(sop_session_t *s);

static bool sop_session_arm(sop_session_t *s)
{
    sop_source_pause();
    s->epoch_started=esp_timer_get_time();
    s->phase_scored=false;s->phase=(sync_phase_bracket_t){0};
    s->rise_before=s->rise_after=s->fall_before=s->fall_after=0;
    if(s->edge_clock) {
        /* RX idle expiry can run before its first edge. Do not let a gain
         * task preempt the short RX/ETM/mirror arm sequence. These APIs do
         * not allocate or wait; interrupts continue and a time/ended guard
         * refuses even an interrupt-delayed arm before accepting evidence. */
        bool ready=false;
        vTaskSuspendAll();
        int64_t arm_start=esp_timer_get_time();
        s->err=rmt_receive(s->channel,s_symbols,sizeof(s_symbols),&s->receive);
        if(s->err!=ESP_OK)s->failure="receive";
        else if(!s->clock_link_armed && !sync_edge_clock_arm())s->failure="clock_arm";
        else if(esp_timer_get_time()-arm_start>SOP_ARM_MAX_US || s_probe.ended)s->failure="arm_gap";
        else {
            ready=true;
            s->clock_link_armed=true;
        }
        s->arm_us=esp_timer_get_time()-arm_start;
        if(s->arm_us>SOP_ARM_MAX_US){ready=false;s->failure="arm_gap";}
        if(ready) {
            const bool quiet_read=sync_edge_clock_read(&s->epoch_quiet);
            if(!quiet_read || (!s->clock_rearms?s->epoch_quiet!=0u:s->epoch_quiet<=s->native_floor)) {
                if(quiet_read && s->clock_rearms)++s->clock_stats.regressions;
                ready=false;s->failure=s->clock_rearms?"clock_regression":"clock_not_quiet";
            }
        }
        if(ready) {
            /* The first private HIGH marker starts the ETM clock. RMT's own
             * fixed input filter can delay its origin; the second marker
             * measures that offset rather than assuming simultaneous taps.
             * Subsequent ETM events are idempotent START, never reload. */
            REG_WRITE(GPIO_OUT_W1TS_REG,SOP_MASK);
            esp_rom_delay_us(16);
            REG_WRITE(GPIO_OUT_W1TC_REG,SOP_MASK);
            esp_rom_delay_us(16);
            if(!sync_edge_clock_enable_filter()){ready=false;s->failure="clock_filter";}
            else {
                esp_rom_delay_us(4);
                /* A second, filtered HIGH marker measures the whole native
                 * input path's delay. Both physical edges are bracketed by
                 * timer reads and flushed GPIO data readbacks. The marker
                 * remains confined to GPIO2 and below the source barrier. */
                portDISABLE_INTERRUPTS();
                bool rise_ok=sync_edge_clock_read(&s->rise_before);
                if(rise_ok) {
                    __sync_synchronize();REG_WRITE(GPIO_OUT_W1TS_REG,SOP_MASK);
                    bool high=(REG_READ(GPIO_OUT_REG)&SOP_MASK)!=0u;__sync_synchronize();
                    rise_ok=high && sync_edge_clock_read(&s->rise_after);
                }
                portENABLE_INTERRUPTS();
                if(!rise_ok){ready=false;s->failure="clock_rise_after";}
                if(ready) {
                    esp_rom_delay_us(16);
                    portDISABLE_INTERRUPTS();
                    bool fall_ok=sync_edge_clock_read(&s->fall_before);
                    if(fall_ok) {
                        __sync_synchronize();REG_WRITE(GPIO_OUT_W1TC_REG,SOP_MASK);
                        bool low=(REG_READ(GPIO_OUT_REG)&SOP_MASK)==0u;__sync_synchronize();
                        fall_ok=low && sync_edge_clock_read(&s->fall_after);
                    }
                    portENABLE_INTERRUPTS();
                    if(!fall_ok){ready=false;s->failure="clock_fall_after";}
                }
                if(ready)esp_rom_delay_us(16);
                if(ready)REG_WRITE(GPIO_FUNC2_OUT_SEL_CFG_REG,(s->witness_matrix&~GPIO_FUNC2_OUT_SEL_M)|(s->flag_source?48u:52u));
                uint64_t native_start;
                if(ready && (!sync_edge_clock_read(&native_start) || native_start<=s->epoch_quiet ||
                    native_start-s->epoch_quiet>UINT32_MAX-64u)) {
                    ready=false;s->failure="clock_native_start";
                } else if(ready) {
                    s->native_floor=native_start;
                    s_native->rise_before=s->rise_before;s_native->rise_after=s->rise_after;
                    s_native->fall_before=s->fall_before;s_native->fall_after=s->fall_after;
                    __sync_synchronize();
                    /* RMT restarts at zero; the output timer never restarts.
                     * Its pre-marker read gives a conservative local barrier. */
                    s_probe.discard_before=(uint32_t)(native_start-s->epoch_quiet)+s->common_filter_ns/50u+4u;
                }
            }
        }
        (void)xTaskResumeAll();
        if(!ready)return false;
        if(s->resident && s->flag_source) {
            /* Let the common input filter clear the private LOW marker before
             * granting a direct GPIO sampler camera-source ownership. */
            esp_rom_delay_us((s->common_filter_ns+999u)/1000u+1u);
            __sync_synchronize();
            s_source_ready=s_probe.armed && !s_probe.ended && !s_probe.stop &&
                (REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG)&GPIO_FUNC2_OUT_SEL_M)==48u;
        }
    } else {
        s->err=rmt_receive(s->channel,s_symbols,sizeof(s_symbols),&s->receive);
        if(s->err!=ESP_OK){s->failure="receive";return false;}
    }
    return true;
}

static bool sop_session_begin(uint32_t generation, bool (*live)(uint32_t), bool edge_clock, bool resident, bool flag_source)
{
    if(s_busy || s_quarantined || s_clock_fault)return false;
    if(!s_session)s_session=calloc(1,sizeof(*s_session));
    if(!s_session)return false;
    sop_session_t *s=s_session;
    memset(s,0,sizeof(*s));
    s->generation=generation;s->live=live;s->edge_clock=edge_clock;s->resident=resident;s->flag_source=flag_source;
    s->clock_closed=true;s->arm_us=-1;s->failure="none";
    if (s_busy || s_quarantined || s_clock_fault) {
        printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 refusal=busy_or_quarantined\n"); return false;
    }
    if (!s->live || !s->live(s->generation)) {
        printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 refusal=not_live\n"); return false;
    }
    if(s->edge_clock && !s_native) {
        s_native=calloc(1,sizeof(*s_native));
        if(!s_native){printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 refusal=native_memory\n");return false;}
    }
    if(s->edge_clock)memset(s_native,0,sizeof(*s_native));
    s_busy = true;
    s->iomux = REG_READ(IO_MUX_GPIO8_REG);
    s->matrix = REG_READ(GPIO_FUNC8_OUT_SEL_CFG_REG);
    s->enabled = REG_READ(GPIO_ENABLE_REG) & (1u << 8);
    /* PARLIO DATA4 index52, push-pull, no inversion. Either peripheral or
     * GPIO-controlled OE is valid: this input-only observer changes neither.
     * The root caller separately verifies the decoder and output context. */
    bool pin_ok = (s->matrix & GPIO_FUNC8_OUT_SEL_M) == 52u &&
        !(s->matrix & (GPIO_FUNC8_OUT_INV_SEL_M | GPIO_FUNC8_OE_INV_SEL_M)) &&
        s->enabled &&
        !(REG_READ(GPIO_PIN8_REG) & GPIO_PIN8_PAD_DRIVER_M) &&
        ((s->iomux & MCU_SEL_M) >> MCU_SEL_S) == 1u;
    if (!pin_ok) {
        printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 refusal=pin_context matrix=0x%08lx iomux=0x%08lx enable=0x%08lx pad=0x%08lx\n",
            (unsigned long)s->matrix, (unsigned long)s->iomux, (unsigned long)s->enabled,
            (unsigned long)REG_READ(GPIO_PIN8_REG));
        s_busy = false; return false;
    }
    /* The native witness must continue seeing PARLIO while all six DAC pads
     * are gated. GPIO2 is not wired to the DAC and must be unused/disabled.
     * Its MTMS role is already unavailable in this RX GPIO3/4/5 topology.
     * Refuse an occupied output rather than taking it from another owner. */
    s->mirror_iomux = REG_READ(IO_MUX_GPIO2_REG);
    s->mirror_matrix = REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG);
    s->mirror_pin = REG_READ(GPIO_PIN2_REG);
    s->mirror_enable = REG_READ(GPIO_ENABLE_REG) & SOP_MASK;
    s->mirror_output = REG_READ(GPIO_OUT_REG) & SOP_MASK;
    if (s->mirror_enable || (s->mirror_pin & GPIO_PIN2_PAD_DRIVER_M)) {
        printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 refusal=mirror_occupied\n");
        s_busy = false; return false;
    }
    s->witness_iomux = (s->mirror_iomux & ~(MCU_SEL_M | FUN_PU | FUN_PD)) |
        (1u << MCU_SEL_S) | FUN_IE;
    s->witness_matrix = (s->mirror_matrix & ~(GPIO_FUNC2_OUT_SEL_M |
        GPIO_FUNC2_OUT_INV_SEL_M | GPIO_FUNC2_OE_INV_SEL_M)) | (s->edge_clock?256u:(s->flag_source?48u:52u)) | GPIO_FUNC2_OE_SEL_M;
    memset(&s_probe, 0, sizeof(s_probe));
    s_probe.generation = s->generation; s_probe.live = s->live;
    s_probe.marker_required=s->edge_clock;
    if(s_native){s_native->resident=resident;s_native->trace_flag=flag_source;s_native->rate_started=esp_timer_get_time();}
    if(s->edge_clock)s_probe.discard_before=UINT32_MAX; /* IRQs may precede handoff publication. */

    const rmt_rx_channel_config_t cfg = {.gpio_num = SOP_GPIO,
        .clk_src = RMT_CLK_SRC_PLL_F80M, .resolution_hz = 20000000u,
        .mem_block_symbols = 96u, .intr_priority = 2};
    const rmt_rx_event_callbacks_t callbacks = {.on_recv_done = sop_received};
    s->receive = (rmt_receive_config_t) {.signal_range_min_ns = s->edge_clock?SOP_CLOCK_MIN_NS:2500u,
        .signal_range_max_ns = s->edge_clock?1000000u:100000u, .flags = {.en_partial_rx = true}};
    s->began = esp_timer_get_time();
    if(s->edge_clock)REG_WRITE(GPIO_OUT_W1TC_REG,SOP_MASK);
    REG_WRITE(IO_MUX_GPIO2_REG, s->witness_iomux);
    REG_WRITE(GPIO_FUNC2_OUT_SEL_CFG_REG, s->witness_matrix);
    REG_WRITE(GPIO_ENABLE_W1TS_REG, SOP_MASK);
    if (REG_READ(IO_MUX_GPIO2_REG) != s->witness_iomux ||
        REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG) != s->witness_matrix ||
        (REG_READ(GPIO_ENABLE_REG) & SOP_MASK) != SOP_MASK) {
        s->failure = "mirror_readback"; goto failed;
    }
    s->err = rmt_new_rx_channel(&cfg, &s->channel);
    if (s->err != ESP_OK) { s->failure = "new_rx"; goto failed; }
    /* SDK may set FUN_IE and reselect the same GPIO function. All other
     * matrix/OE and IO MUX bits must remain unchanged before enabling RX. */
    s->pin_changed = REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG) != s->witness_matrix ||
        (REG_READ(GPIO_ENABLE_REG) & SOP_MASK) != SOP_MASK ||
        REG_READ(IO_MUX_GPIO2_REG) != s->witness_iomux;
    if (s->pin_changed) { s->failure = "pin_mutated"; goto failed; }
    s->err = rmt_rx_register_event_callbacks(s->channel, &callbacks, &s_probe);
    if (s->err != ESP_OK) { s->failure = "callbacks"; goto failed; }
    if(s->edge_clock) {
        s->clock_attempted=true;
        if(!sync_edge_clock_prepare()){s->failure="clock_prepare";goto failed;}
        s->common_filter_ns=sync_edge_clock_filter_ns();
        esp_rom_delay_us(50); /* Private GPIO data drives a quiet LOW; DAC is untouched. */
    }
    if (!s->live(s->generation)) { s_probe.cancelled = true; s->failure = "not_live"; goto failed; }
    s->err = rmt_enable(s->channel);
    if (s->err != ESP_OK) { s->failure = "enable"; goto failed; }
    s->enabled_rx = true;
    s_probe.armed = true;
    sop_epoch(&s_probe);
    if(!sop_session_arm(s))goto failed;
    return true;
failed:
    (void)sop_session_end(s);
    return false;
}

/* Same-camera gain/DC/filter changes invalidate observations, not established
 * camera clock authority. Restore only trusted fields after clearing every
 * acquisition candidate and incomplete vertical group. */
static void sop_flag_clock_reset(sync_native_flag_track_t *f)
{
    sync_native_track_t *n=f->native;
    bool trusted=n->valid,selected=n->have_selected;
    bool have_group=n->have_group,shifted=f->group_shifted;
    uint64_t group=n->group;
    uint8_t group_n=n->group_n;
    uint64_t h=n->h,field=n->field;
    uint32_t line=n->selected_line_ticks_x32,fields=n->qualified_fields;
    uint8_t standard=n->standard;
    sync_native_reset(n);sync_native_flag_init(f,n);
    /* A fully completed first V group is evidence, not an unfinished pulse
     * train. Keep it on this same clock so 20ms AGC cannot starve a 33ms pair.
     * snft_finish still demands an independent fresh H/V group and its exact
     * inter-field timing/phase agreement before granting authority. */
    n->have_group=have_group;n->group=group;n->group_n=group_n;
    f->group_shifted=shifted;
    if(trusted) {
        n->valid=true;n->standard=standard;n->h=h;n->field=field;
        n->line_ticks_x32=n->selected_line_ticks_x32=line;
        n->qualified_fields=fields;n->have_selected=selected;
    }
}

static bool sop_session_poll(sop_session_t *s)
{
    if(!s_busy)return false;
    if(!s->live(s->generation)){s_probe.cancelled=true;s_probe.stop=true;s->failure="not_live";return false;}
    sop_drain(&s_probe);
    if(s_probe.stop)return false;
    /* Only the comparator's ordinary idle ending may retain a completed
     * V candidate on this same uninterrupted absolute clock. PHY invalidation,
     * proactive rollover and failed marker calibration remain full resets. */
    bool flag_idle=s->flag_source && (s_probe.ended || s_rx_ring.overflow) && !s->invalidated &&
        esp_timer_get_time()-s->epoch_started<180000000;
    /* Abort and rearm before the 32-bit RMT-local elapsed counter wraps.
     * This is a source discontinuity, never synthetic camera evidence. */
    /* A native idle ending is a hardware receive boundary too. Use the same
     * full RX disable/enable boundary as an invalidation, including the SDK's
     * interrupt-status clear, before reusing the channel and private marker.
     * Keep the independent absolute timer running throughout. */
    if(s_probe.ended || s_rx_ring.overflow || s->invalidated || esp_timer_get_time()-s->epoch_started>=180000000) {
        sop_source_pause();
        s_probe.armed=false;
        if(rmt_disable(s->channel)!=ESP_OK){s->failure="rollover_disable";return false;}
        s->enabled_rx=false;
        sop_drain(&s_probe);
        if(s_probe.stop)return false;
        if(rmt_enable(s->channel)!=ESP_OK){s->failure="rollover_enable";return false;}
        s->enabled_rx=true;s_probe.ended=true;
    }
    if(s_probe.ended) {
        /* Retain the last qualified clock for callers' holdover, separately
         * from the fresh receiver evidence about to be reset. */
        sync_output_live_model_t retained;
        if(sync_output_live_snapshot(&retained) && retained.standard)
            s->retained=retained;
        if(s->edge_clock) {
            s->phase=sop_score_clock(&s->clock_stats,s->rise_before,s->rise_after,s->fall_before,s->fall_after);
            s->phase_scored=true;
            if(s->clock_stats.regressions){s->failure="clock_regression";return false;}
            s_probe.armed=false;
            REG_WRITE(GPIO_OUT_W1TC_REG,SOP_MASK);
            REG_WRITE(GPIO_FUNC2_OUT_SEL_CFG_REG,s->witness_matrix);
            esp_rom_delay_us(4);
            if(!sync_edge_clock_disable_filter()){s->failure="clock_filter_pause";return false;}
            ++s->clock_rearms;
        }
        if(s->forget_native)s->clock_stats.have_origin=false;
        bool phase_ok=s->phase.valid || (s->clock_stats.have_origin && !s->clock_stats.regressions);
        if(s_native && phase_ok) {
            s_native->calibrated_phase=(sync_phase_bracket_t){
                .lo=s->phase.valid?s->phase.lo:s->clock_stats.last_lo,
                .hi=s->phase.valid?s->phase.hi:s->clock_stats.last_hi,
                .valid=true
            };
        }
        bool retain_clock=s->resident && s->flag_source && s->invalidated &&
            !s->forget_native && phase_ok;
        if(retain_clock) {
            sop_flag_clock_reset(&s_native->flag_native);
            sop_flag_clock_reset(&s_native->flag_inverted);
            s->fresh_h_fit=true;
        }
        sop_epoch_mode(&s_probe,retain_clock || (flag_idle && phase_ok));
        s_probe.armed=true;s->invalidated=false;s->forget_native=false;
        return sop_session_arm(s);
    }
    return true;
}

static bool sop_session_end(sop_session_t *s)
{
    sop_source_pause();
    s_probe.armed = false;
    sop_drain(&s_probe);
    if(s->before_close && !s->before_close()) {
        s_clock_fault=true;
        return false; /* Keep dependent callback storage/timer owned. */
    }
    if(s->edge_clock && s_probe.epochs && !s->phase_scored)
        s->phase=sop_score_clock(&s->clock_stats,s->rise_before,s->rise_after,s->fall_before,s->fall_after);
    if(s->edge_clock && s->clock_attempted)s->clock_read_ok=sync_edge_clock_read(&s->clock_ticks);
    if (s_probe.bad_data) s->failure = "bad_data";
    else if (s_probe.marker_invalid) s->failure = "bad_marker";
    else if (s_probe.cancelled) s->failure = "not_live";
    else if (s_probe.rate_limited) s->failure = "event_rate";
    if (s->channel) {
        if (s->enabled_rx) {
            esp_err_t stopped = rmt_disable(s->channel);
            if (stopped != ESP_OK) s->cleanup = stopped;
            else s->enabled_rx=false;
        }
        const rmt_rx_event_callbacks_t no_callbacks = {0};
        esp_err_t detached = rmt_rx_register_event_callbacks(s->channel, &no_callbacks, NULL);
        if (detached != ESP_OK) s->cleanup = detached;
        if(s->cleanup==ESP_OK) {
            esp_err_t deleted = rmt_del_channel(s->channel);
            if(deleted!=ESP_OK)s->cleanup=deleted;
        }
        if(s->cleanup!=ESP_OK)s_quarantined=s->channel;
        else s->channel=NULL;
    }
    /* Restore the unused mirror pad even on allocation/setup errors.
     * Only the private GPIO2 marker data bit is restored; no DAC data writes. */
    REG_WRITE(GPIO_ENABLE_W1TC_REG, SOP_MASK);
    if(s->clock_attempted) {
        s->clock_closed=sync_edge_clock_close();
        if(!s->clock_closed)s_clock_fault=true;
        uint32_t rmt_rise=s_probe.prefix[0]+s_probe.prefix[1];
        uint32_t rmt_fall=rmt_rise+s_probe.prefix[2];
        if(!s->resident && !s->flag_source)printf("SYNC_PHASE_BRACKET manual=1 absolute_time=0 camera_qualified=0 prefix_n=%u rmt_rise=%lu rmt_fall=%lu rise_before=%llu rise_after=%llu fall_before=%llu fall_after=%llu valid=%u offset_lo=%lld offset_hi=%lld\n",
            s_probe.prefix_n,(unsigned long)rmt_rise,(unsigned long)rmt_fall,
            (unsigned long long)s->rise_before,(unsigned long long)s->rise_after,
            (unsigned long long)s->fall_before,(unsigned long long)s->fall_after,
            (unsigned)s->phase.valid,(long long)s->phase.lo,(long long)s->phase.hi);
        if(!s->resident && !s->flag_source)printf("SYNC_CLOCK_EPOCHS manual=1 camera_qualified=0 rearms=%lu valid=%lu refused=%lu pending=%lu regressions=%lu max_span_ticks=%lu last_origin_lo=%lld last_origin_hi=%lld timer_resets=0\n",
            (unsigned long)s->clock_rearms,(unsigned long)s->clock_stats.valid,
            (unsigned long)s->clock_stats.refused,(unsigned long)s->clock_stats.pending,
            (unsigned long)s->clock_stats.regressions,(unsigned long)s->clock_stats.max_span,
            (long long)s->clock_stats.last_lo,(long long)s->clock_stats.last_hi);
        const sync_native_track_t *nt=s_native->inverted.valid && !s_native->native.valid?
            &s_native->inverted:&s_native->native;
        if(!s->resident && !s->flag_source)printf("SYNC_NATIVE_TIMING manual=1 camera_qualified=0 timing_valid=%u standard=%u qualified_epochs=%lu fields=%lu line_ticks_x32=%lu field_tick=%llu phase_span_ticks=%lld\n",
            (unsigned)(nt->valid && !(s_native->inverted.valid && s_native->native.valid)),(unsigned)nt->standard,
            (unsigned long)s_native->native_qualified_epochs,
            (unsigned long)nt->qualified_fields,
            (unsigned long)nt->line_ticks_x32,
            (unsigned long long)nt->field,
            s_native->native_phase.valid?(long long)(s_native->native_phase.hi-s_native->native_phase.lo):-1LL);
        if(!s->resident && !s->flag_source)printf("SYNC_EDGE_CLOCK manual=1 absolute_time=0 camera_qualified=0 common_filter_ns=%lu timer_hz=20000000 ticks=%llu read_ok=%u edges_seen=%u closed=%u arm_us=%lld marker_ticks=%lu marker_level=%u marker_ok=%u native_after_ticks=%lu error=%s\n",
            (unsigned long)s->common_filter_ns,(unsigned long long)s->clock_ticks,
            (unsigned)s->clock_read_ok,(unsigned)(s_probe.halves!=0u),
            (unsigned)s->clock_closed,(long long)s->arm_us,(unsigned long)s_probe.first_duration,
            (unsigned)s_probe.first_level,(unsigned)(s_probe.first_level &&
                s_probe.first_duration>=280u && s_probe.first_duration<=SOP_MARKER_MAX_TICKS),
            (unsigned long)s_probe.discard_before,sync_edge_clock_error());
    }
    if(s->edge_clock && (REG_READ(GPIO_OUT_REG)&SOP_MASK)!=s->mirror_output)
        REG_WRITE(s->mirror_output?GPIO_OUT_W1TS_REG:GPIO_OUT_W1TC_REG,SOP_MASK);
    if (REG_READ(IO_MUX_GPIO2_REG) != s->mirror_iomux) REG_WRITE(IO_MUX_GPIO2_REG, s->mirror_iomux);
    if (REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG) != s->mirror_matrix)
        REG_WRITE(GPIO_FUNC2_OUT_SEL_CFG_REG, s->mirror_matrix);
    bool restored = REG_READ(IO_MUX_GPIO2_REG) == s->mirror_iomux &&
        REG_READ(GPIO_FUNC2_OUT_SEL_CFG_REG) == s->mirror_matrix &&
        REG_READ(GPIO_PIN2_REG) == s->mirror_pin &&
        (REG_READ(GPIO_ENABLE_REG) & SOP_MASK) == s->mirror_enable &&
        (REG_READ(GPIO_OUT_REG) & SOP_MASK) == s->mirror_output &&
        REG_READ(IO_MUX_GPIO8_REG) == s->iomux &&
        REG_READ(GPIO_FUNC8_OUT_SEL_CFG_REG) == s->matrix &&
        (REG_READ(GPIO_ENABLE_REG) & (1u << 8)) == s->enabled;
    bool exclusion = !s_probe.cancelled && s->live(s->generation);
    if(!s->resident && !s->flag_source)printf("SYNC_OUTPUT_PROBE source=native_dac manual=1 gpio=2 mirror_source_gpio=%u bit=%u output_x100=%u "
        "self_gate_excluded=%u absolute_time=0 camera_qualified=0 restored=%u "
        "pin_mutated=%u error=%s api=%d cleanup=%d duration_us=%lld "
        "callbacks=%lu symbols=%lu halves=%lu epochs=%lu endings=%lu low=%lu "
        "h=%lu broad=%lu h_periods=%lu pal32=%lu ntsc32=%lu broad5=%lu broad6=%lu "
        "pal_fields=%lu ntsc_fields=%lu bad_data=%lu cancelled=%u\n",
        s->flag_source?23u:8u,s->flag_source?0u:4u,s->flag_source?0u:100u,
        (unsigned)exclusion, (unsigned)restored, (unsigned)s->pin_changed,
        s->failure, (int)s->err, (int)s->cleanup, (long long)(esp_timer_get_time() - s->began),
        (unsigned long)s_probe.callbacks, (unsigned long)s_probe.symbols,
        (unsigned long)s_probe.halves, (unsigned long)s_probe.epochs,
        (unsigned long)s_probe.endings, (unsigned long)s_probe.low,
        (unsigned long)s_probe.h, (unsigned long)s_probe.broad,
        (unsigned long)s_probe.h_periods, (unsigned long)s_probe.pal32,
        (unsigned long)s_probe.ntsc32, (unsigned long)s_probe.broad5,
        (unsigned long)s_probe.broad6, (unsigned long)s_probe.pal_fields,
        (unsigned long)s_probe.ntsc_fields, (unsigned long)s_probe.bad_data,
        (unsigned)s_probe.cancelled);
    /* A failed pin readback is an ownership fault even when every SDK
     * cleanup call succeeded. Never reuse a partly restored observer. */
    if(!restored || s->cleanup!=ESP_OK)s_clock_fault=true;
    if(s->resident)printf("SYNC_LIVE_END error=%s api=%d cleanup=%d restored=%u clock_closed=%u callbacks=%lu marker_ticks=%lu arm_us=%lld\n",
        s->failure,(int)s->err,(int)s->cleanup,(unsigned)restored,(unsigned)s->clock_closed,
        (unsigned long)s_probe.callbacks,(unsigned long)s_probe.first_duration,(long long)s->arm_us);
    s_busy = false;
    return restored && s->cleanup == ESP_OK && s->clock_closed;
}

bool sync_output_live_begin(uint32_t generation,bool (*live)(uint32_t))
{ return sop_session_begin(generation,live,true,true,false); }
bool sync_output_live_begin_flag(uint32_t generation,bool (*live)(uint32_t))
{ return sop_session_begin(generation,live,true,true,true); }
bool sync_output_live_active(void)
{ return s_busy && s_session && s_session->resident; }
void sync_output_live_before_close(bool (*callback)(void))
{ if(sync_output_live_active())s_session->before_close=callback; }
bool sync_output_live_poll(void)
{
    if(!sync_output_live_active())return false;
    if(sop_session_poll(s_session))return true;
    (void)sop_session_end(s_session);return false;
}
bool sync_output_live_end(void)
{
    if(!s_busy)return !s_quarantined && !s_clock_fault;
    if(!sync_output_live_active())return false;
    return sop_session_end(s_session);
}

void sync_output_live_invalidate(void)
{
    if(!sync_output_live_active())return;
    sop_source_pause();
    s_probe.armed=false;
    __sync_synchronize();
    s_session->invalidated=true;
}

/* Advance a trusted V anchor by whole fields on its rational halfline grid.
 * A field contains an odd halfline count; retaining that count (rather than
 * rounding to whole H lines) also preserves interlace H parity. Split the
 * arithmetic before multiplying so even near-UINT64_MAX timer values are safe. */
static uint64_t sop_coast_field(uint64_t field,uint64_t h,uint32_t line,uint8_t standard)
{
    if(h<=field || !line || (standard!=1u && standard!=2u))return field;
    uint64_t numerator=(uint64_t)(standard==2u?625u:525u)*line;
    uint64_t age=h-field;
    uint64_t fields=(age/numerator)*64u+((age%numerator)*64u+31u)/numerator;
    uint64_t advance=(fields/64u)*numerator+((fields%64u)*numerator+32u)/64u;
    return field+advance; /* advance<=age by construction */
}

void sync_output_live_forget(void)
{
    if(!sync_output_live_active())return;
    s_session->forget_native=true;
    sync_output_live_invalidate();
}

bool sync_output_live_snapshot(sync_output_live_model_t *out)
{
    if(!out)return false;
    *out=(sync_output_live_model_t){0};
    if(!sync_output_live_active() || !s_native)return false;
    for(unsigned attempt=0;attempt<3u;++attempt) {
        uint32_t seq=s_native->sequence;
        if(seq&1u)continue;
        __sync_synchronize();
        sop_drain(&s_probe);
        sync_native_track_t native=s_native->native;
        sync_native_track_t inverted=s_native->inverted;
        bool high_pulse=s_session->flag_source?!native.valid || inverted.valid:
            (inverted.valid && !native.valid);
        bool ambiguous=!s_session->flag_source && inverted.valid && native.valid;
        if(high_pulse)native=inverted;
        sync_phase_bracket_t phase=s_native->native_phase;
        if(!phase.valid && native.valid && s_session->clock_stats.have_origin && !s_session->clock_stats.regressions)
            phase=(sync_phase_bracket_t){.lo=s_session->clock_stats.last_lo,.hi=s_session->clock_stats.last_hi,.valid=true};
        bool armed=s_probe.armed,invalidated=s_session->invalidated;
        uint64_t now;
        if(!sync_edge_clock_read(&now))return false;
        __sync_synchronize();
        if(seq!=s_native->sequence)continue;
        *out=s_session->retained;
        if(native.valid && phase.valid && !ambiguous) {
            out->standard=native.standard;
            out->last_h=native.h;out->field=native.field;
            out->line_ticks_x32=native.have_selected?native.selected_line_ticks_x32:native.line_ticks_x32;
            out->phase_lo=phase.lo;out->phase_hi=phase.hi;
            out->inverted=high_pulse;
        }
        out->generation=s_session->generation;out->now=now;
        bool source=s_session->flag_source?sync_output_live_source(NULL,NULL):armed;
        out->tracking=armed && !invalidated && source && !ambiguous && native.valid &&
            native.have_selected && phase.valid && native.field &&
            (!s_session->fresh_h_fit || (native.h_fit && native.h_standard==native.standard)) &&
            (native.standard==1u || native.standard==2u) &&
            now>=native.h && now-native.h<=SOP_MODEL_MAX_AGE_TICKS &&
            s_session->live(s_session->generation);
        out->valid=armed && !invalidated && !ambiguous && native.valid && phase.valid &&
            now>=native.last && now-native.last<=SOP_MODEL_MAX_AGE_TICKS &&
            now>=native.field && now-native.field<=810000u &&
            s_session->live(s_session->generation);
        if(out->tracking && !out->valid)
            out->field=sop_coast_field(native.field,native.h,out->line_ticks_x32,native.standard);
        return true;
    }
    return false;
}

bool sync_output_live_diagnostics(char *out,size_t size)
{
    if(!out || !size)return false;
    out[0]=0;
    if(!sync_output_live_active() || !s_native)return false;
    for(unsigned attempt=0;attempt<3u;++attempt) {
        uint32_t seq=s_native->sequence;
        if(seq&1u)continue;
        __sync_synchronize();
        sop_drain(&s_probe);
        sop_context_t probe=s_probe;
        sync_native_track_t native=s_native->native;
        sync_phase_bracket_t phase=s_native->native_phase;
        bool inverted=s_session->flag_source?!native.valid || s_native->inverted.valid:
            (s_native->inverted.valid && !native.valid);
        bool ambiguous=!s_session->flag_source && s_native->inverted.valid && native.valid;
        if(inverted)native=s_native->inverted;
        bool phase_checked=s_native->native_phase_checked;
        if(!phase.valid && s_session->clock_stats.have_origin && !s_session->clock_stats.regressions) {
            phase=(sync_phase_bracket_t){.lo=s_session->clock_stats.last_lo,.hi=s_session->clock_stats.last_hi,.valid=true};
            phase_checked=true;
        }
        uint32_t qualified_epochs=s_native->native_qualified_epochs;
        uint32_t rearms=s_session->clock_rearms;
        bool invalidated=s_session->invalidated;
        __sync_synchronize();
        if(seq!=s_native->sequence)continue;
        int n=snprintf(out,size,"SYNC_LIVE_DIAG callbacks=%lu symbols=%lu epochs=%lu endings=%lu low=%lu h=%lu broad=%lu h_periods=%lu pal32=%lu ntsc32=%lu broad5=%lu broad6=%lu prefix_n=%u marker_ticks=%lu barrier=%lu elapsed=%lu phase_checked=%u phase_valid=%u phase_lo=%lld phase_hi=%lld native_valid=%u native_standard=%u fields=%lu qualified_epochs=%lu rearms=%lu invalidated=%u inverted=%u ambiguous=%u\n",
            (unsigned long)probe.callbacks,(unsigned long)probe.symbols,
            (unsigned long)probe.epochs,(unsigned long)probe.endings,
            (unsigned long)probe.low,(unsigned long)probe.h,(unsigned long)probe.broad,
            (unsigned long)probe.h_periods,(unsigned long)probe.pal32,(unsigned long)probe.ntsc32,
            (unsigned long)probe.broad5,(unsigned long)probe.broad6,probe.prefix_n,
            (unsigned long)probe.first_duration,(unsigned long)probe.discard_before,
            (unsigned long)probe.elapsed,(unsigned)phase_checked,
            (unsigned)phase.valid,(long long)phase.lo,
            (long long)phase.hi,(unsigned)(native.valid && !ambiguous),
            (unsigned)native.standard,(unsigned long)native.qualified_fields,
            (unsigned long)qualified_epochs,(unsigned long)rearms,(unsigned)invalidated,(unsigned)inverted,(unsigned)ambiguous);
        if(n>=0 && (size_t)n<size)return true;
        out[0]=0;return false;
    }
    return false;
}


