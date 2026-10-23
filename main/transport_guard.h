/* C5VRX by Twotoz and contributors: shared live restart cooldown. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define TRANSPORT_SWITCH_GAP_US 5000000LL
typedef struct { bool switched; int64_t last_switch_us; } transport_guard_t;
static inline bool transport_guard_ready(const transport_guard_t *g, int64_t now)
{ return !g->switched || now - g->last_switch_us >= TRANSPORT_SWITCH_GAP_US; }
static inline void transport_guard_mark(transport_guard_t *g, int64_t now)
{ g->switched = true; g->last_switch_us = now; }
