/* C5VRX by Twotoz and contributors: native DAC timing observer for the sync
 * flywheel. GPIO2 must be unused; it mirrors PARLIO DATA4 independently of
 * the six DAC OEs. GPIO8 and all DAC settings remain unchanged by this
 * observer. The live predicate is called from ISR context and must be IRAM
 * safe. No camera-lock or analogue waveform claim follows from it. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "sync_native_trace.h"

/* Resident observer. Task context only; caller serializes begin/poll/end
 * against transport mutations with short ownership, never a resident lease.
 * At1x the resident DATA4 observation can be investigated on P64
 * program; absence of a qualified model alone does not authorize repairing
 * an unqualified strong-signal program. No output gates are enabled here.
 * Invalidate BEFORE a receive
 * mutation; poll rearms fresh evidence on the same running timer. Snapshot
 * freshness includes RMT delivery latency; it is not a per-line loss flag.
 * Snapshot coordinates remain available with valid=false during holdover.
 * A failed end quarantines driver ownership and prevents subsequent begin. */
typedef struct {
    uint64_t last_h, field, now; /* field may be predicted only when tracking&&!valid. */
    uint32_t generation, line_ticks_x32;
    int64_t phase_lo, phase_hi;
    uint8_t standard; /* Native qualifier IDs: PAL=2, NTSC=1. */
    bool valid;    /* Fresh observed V plus native timing evidence. */
    bool tracking; /* Fresh selected H on trusted camera clock; V may coast. */
    bool inverted; /* Qualified sync is a HIGH run on native DATA4. */
} sync_output_live_model_t;
bool sync_output_live_begin(uint32_t generation, bool (*live)(uint32_t));
/* Verified comparator stream (signal48 HIGH sync), independent of DAC scale.
 * Caller proves flag LUT/program binding and GPIO23 clamp in live(). */
bool sync_output_live_begin_flag(uint32_t generation, bool (*live)(uint32_t));
/* IRAM-safe sampler admission. False throughout private marker, mutation and
 * teardown. Discard edge history whenever false or epoch changes. */
bool sync_output_live_source(void *arg,uint32_t *epoch);
bool sync_output_live_poll(void);
bool sync_output_live_end(void);
bool sync_output_live_active(void);
/* Task-side teardown dependency: stop any consumer of the shared clock and
 * restore DAC ownership BEFORE observer teardown checks/clock deletion. */
void sync_output_live_before_close(bool (*callback)(void));
/* Same-camera mutation: flag mode keeps trusted clock/completed V evidence,
 * clears partial observations, and requires a new H fit before tracking. */
void sync_output_live_invalidate(void);
/* Different RF channel/camera: discard native authority as well as partials.
 * Published old coordinates remain available only as nontracking holdover. */
void sync_output_live_forget(void);
bool sync_output_live_snapshot(sync_output_live_model_t *out);
/* Task-side diagnostic snapshot, under the same serialized ownership. */
bool sync_output_live_diagnostics(char *out,size_t size);
/* Resident flag session: receives each native pulse during drain (task). */
void sync_output_live_pulse_hook(void (*hook)(uint64_t tick,uint32_t width,bool high));
