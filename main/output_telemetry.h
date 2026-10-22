/* C5VRX-5 by Twotoz and the C5VRX contributors: task-side rollup of
 * output-observer telemetry. The real-time ISR only owns the cumulative
 * counters supplied here; this helper adds no line-rate or interrupt work. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t repaired, suppressed, camera_lines, observer_gaps;
    uint8_t standard; /* 0 unknown, 1 NTSC, 2 PAL */
    bool active, observer_fresh, camera_present, h_trusted, v_trusted;
} output_telem_sample_t;

typedef struct {
    uint32_t samples, fresh_samples, h_locked_samples, v_locked_samples;
    uint32_t native_h_lines, repaired, suppressed, observer_gaps;
    uint32_t last_native_h_lines, last_repaired, last_suppressed, last_observer_gaps;
    uint32_t unlock_started_ms, longest_unlock_ms;
    uint8_t standard;
    bool counters_active, unlocking, unlock_reported;
} output_telem_acc_t;

typedef struct {
    uint32_t samples, fresh_samples, h_locked_samples, v_locked_samples;
    uint32_t native_h_lines, repaired, suppressed, observer_gaps, longest_unlock_ms;
    uint8_t standard;
} output_telem_roll_t;

static inline uint32_t output_telem_delta(uint32_t now, uint32_t before, bool continuous)
{
    return continuous && now >= before ? now - before : now;
}

/* Returns true once when a continuously observed native-H outage reaches
 * 100 ms. Unknown/stale coverage ends the run instead of counting as unlock. */
static inline bool output_telem_add(output_telem_acc_t *a, const output_telem_sample_t *s, uint32_t now_ms,
                                    bool observation_available)
{
    ++a->samples;
    if (s->active) {
        a->native_h_lines += output_telem_delta(s->camera_lines, a->last_native_h_lines, a->counters_active);
        a->repaired += output_telem_delta(s->repaired, a->last_repaired, a->counters_active);
        a->suppressed += output_telem_delta(s->suppressed, a->last_suppressed, a->counters_active);
        a->observer_gaps += output_telem_delta(s->observer_gaps, a->last_observer_gaps, a->counters_active);
        a->last_native_h_lines = s->camera_lines;
        a->last_repaired = s->repaired;
        a->last_suppressed = s->suppressed;
        a->last_observer_gaps = s->observer_gaps;
        if (s->standard == 1u || s->standard == 2u) a->standard = s->standard;
    }
    a->counters_active = s->active;

    /* Cached status is not current evidence when the owner could not service
     * the observer (busy PHY or fault). Keep cumulative-counter continuity. */
    const bool fresh = observation_available && s->active && s->observer_fresh;
    const bool h_locked = fresh && s->camera_present && s->h_trusted;
    if (fresh) {
        ++a->fresh_samples;
        a->h_locked_samples += h_locked;
        /* A held V trust bit is not evidence after native H disappeared. */
        a->v_locked_samples += s->camera_present && s->v_trusted;
    }
    if (!fresh || h_locked) {
        a->unlocking = false;
        a->unlock_reported = false;
        return false;
    }
    if (!a->unlocking) {
        a->unlocking = true;
        a->unlock_started_ms = now_ms;
    }
    const uint32_t duration = now_ms - a->unlock_started_ms;
    if (duration > a->longest_unlock_ms) a->longest_unlock_ms = duration;
    if (duration >= 100u && !a->unlock_reported) {
        a->unlock_reported = true;
        return true;
    }
    return false;
}

static inline void output_telem_roll(output_telem_acc_t *a, uint32_t now_ms, output_telem_roll_t *out)
{
    *out = (output_telem_roll_t){
        .samples = a->samples, .fresh_samples = a->fresh_samples,
        .h_locked_samples = a->h_locked_samples, .v_locked_samples = a->v_locked_samples,
        .native_h_lines = a->native_h_lines, .repaired = a->repaired, .suppressed = a->suppressed,
        .observer_gaps = a->observer_gaps, .longest_unlock_ms = a->longest_unlock_ms,
        .standard = a->standard,
    };
    a->samples = a->fresh_samples = a->h_locked_samples = a->v_locked_samples = 0;
    a->native_h_lines = a->repaired = a->suppressed = a->observer_gaps = 0;
    a->longest_unlock_ms = 0;
    a->standard = 0;
    if (a->unlocking) a->unlock_started_ms = now_ms;
}

static inline uint8_t output_telem_pct(uint32_t part, uint32_t whole)
{
    if (!whole) return 0xffu; /* explicitly unknown */
    uint64_t pct = ((uint64_t)part * 100u + whole / 2u) / whole;
    return (uint8_t)(pct > 100u ? 100u : pct);
}

static inline uint16_t output_telem_sat16(uint32_t value)
{
    return (uint16_t)(value > 65535u ? 65535u : value);
}
