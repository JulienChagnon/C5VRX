/* C5VRX by Twotoz/contributors: conditional tips on the native timebase.
 * Caller serializes task APIs and cancels before changing transport/pad owner.
 * No task/PHY lease is owned for the lifetime. Analogue/monitor proof is separate. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "sync_live_gate.h"

typedef bool (*slo_live_fn)(uint32_t generation);
/* IRAM source guard: false during markers, RX rearm and receive mutations. */
typedef bool (*slo_source_fn)(void *arg,uint32_t *epoch);
typedef bool (*slo_native_fn)(uint64_t start,uint32_t width,slg_kind_t kind,void *arg);
typedef struct {
    uint32_t repaired,suppressed,batches,max_late_ticks,queue_low,fault,missed,miss_kind;
    uint32_t max_irq_delay_ticks,max_callback_ticks,miss_phase,misses_by_phase[6],budget_yields;
    /* Camera-phase guard: tips left to a present off-grid camera, grid
     * re-anchors (H incl. fine trims, V) from coherent native pulses. */
    uint32_t yielded,guard_h_jumps,guard_v_jumps,guard_trims;
    /* Camera grids granted by the guard (coherent H + field-spaced V). */
    uint32_t guard_acquired;
    /* Holdover PLL: fine-period updates/rejections, offset vs nominal. */
    uint32_t pll_updates,pll_rejected;
    int32_t pll_ppb;
    /* camera_yields: tips left untouched for lack of fresh matching H/V
     * evidence (the type-specific trust below); camera_lines: in-phase
     * camera H pulses; std_switches: PAL/NTSC restarts by the guard. */
    uint32_t camera_yields,camera_lines,std_switches;
    /* Missing source coverage is not evidence of missing camera sync. */
    uint32_t observer_yields,source_yields,observer_gaps,stale_native,sample_age_ticks;
    /* CPU cycles (240 MHz): ISR total and worst single call, early-alarm
     * spin, the spin before tips a native pulse then suppressed, clamps. */
    uint32_t isr_calls,max_isr_cycles;
    uint64_t isr_cycles,wait_cycles,suppressed_wait_cycles,hold_cycles;
    uint8_t standard; /* Actual guard grid: 1 NTSC, 2 PAL. */
    bool observer_fresh;
    bool active,guard_yield,camera_present,h_trusted,v_trusted;
} slo_status_t;
/* Starts on the nominal grid of `fallback` without authority; the guard
 * acquires the camera. Native callback must be cache-safe/IRAM; NULL admits
 * no native suppression evidence. */
bool sync_live_output_begin(slg_standard_t fallback,
    uint32_t generation,slo_live_fn live,slo_native_fn native,void *arg);
bool sync_live_output_poll(void);
bool sync_live_output_end(void);
void sync_live_output_status(slo_status_t *out);
/* Task-side diagnostic tag: 0 other,1 observer,2 snapshot,3 reserved,
 *4 transport restart,5 reserved. ISR records only bounded counters. */
void sync_live_output_phase(unsigned phase);

/* Production comparator source: HIGH sync on GPIO2, DAC GPIO23 clamped LOW. */
bool sync_live_output_flag_begin(slg_standard_t fallback,
    uint32_t generation,slo_live_fn live,slo_source_fn source,void *arg);
void sync_live_output_native_polarity(bool inverted);
/* Task context, every native comparator pulse on the common clock (`high`
 * is the comparator level). The only input that moves the grid: the guard
 * acquires, re-anchors, trims and times it (sync_phase_guard.h). */
void sync_live_output_native_pulse(uint64_t tick,uint32_t width,bool high);
/* Control tick: ends a yield once the camera has been gone for 100 ms. */
void sync_live_output_guard_tick(void);
