/* C5VRX-5 by Twotoz and the C5VRX contributors: realtime transport.
 * MODEM_DIAG -> PARLIO RX 40 MS/s -> 32 KiB cyclic ring -> TX BitScrambler
 * (P64, or U85Q fallback) -> PARLIO TX 40 MHz -> 6-bit resistor DAC. Both GDMA rings run
 * with interrupts off and cleared suc_eof (espressif/esp-idf#19091), so no
 * DMA boundary is a stream boundary; the CPU never paces samples. */
#include "receiver.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "driver/bitscrambler.h"
#include "driver/parlio_rx.h"
#include "driver/parlio_tx.h"
#include "sync_output_probe.h"
#include "sync_live_output.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/misc.h"
#include "hal/bitscrambler_ll.h"
#include "kp_lut_io.h"
#include "kp_switch_guard.h"
#include "transport_recovery.h"
#include "output_level.h"
#include "sync_flag.h"
#include "hal/parlio_ll.h"
#include "kp_tables.h"
#include "p64_tables.h"
#include "snapshot.h"
#include "output_telemetry.h"
#include "control_window.h"
#include "phase8_gain_lut.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_clk.h"
#include "soc/soc_caps.h"
#include "phy.h"
#include "soc/ahb_dma_struct.h"
#include "soc/bitscrambler_struct.h"
#include "soc/parl_io_struct.h"
#include "soc/pcr_struct.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

/* U85Q UNTRAINED (model 1a09f5c092e8, C5VRX PR #190): closed-form 32-phase
 * tracker on the PAIR4411 shared word, origin cells as coarse quadrant
 * tokens, linear -6..+9 MHz output window. docs/FINDINGS.md. */
BITSCRAMBLER_PROGRAM(s_u85q, "u85q");
/* Calmer loop gains (the MATCHED profiles' kp 0.6 / 0.5) are LUTs of this
 * same program: u85q_kp060/kp050.bsasm have identical instructions and
 * their LUTs are kp_tables.h levels 5 and 7 bit-exactly, so they are not
 * separate programs here and the kp/LP ladder runs on every profile. */
/* U85Q-LP: U85Q plus a post-detection IIR low-pass (pole 1/2) in counter A,
 * same LUT and two bundles (research/u85q_lp.py, README sections 12/16).
 * transport_set_lp() uses the normal pipeline start sequence; a live
 * instruction rewrite left the board video black while USB kept running. */
BITSCRAMBLER_PROGRAM(s_u85q_lp, "u85q_lp");
BITSCRAMBLER_PROGRAM(s_u85q_lp_flag, "u85q_lp_flag");
BITSCRAMBLER_PROGRAM(s_u85q_64, "u85q_64");
/* PR #250 P64 champion: 64 states / 16 tokens. Keep this identity with
 * the verified program and LUT through menu, OUTPUT and rollback. */
/* U85Q remains the reference at boot. The first board comparison had no
 * VTX; P64-R1 needs explicit live-signal qualification, not noise viewing. */
static bool s_demod_p64 = false, s_verified_p64 = false;
static bool s_started;   /* live TX running (cleared while menu/status/recovery owns TX) */
static bool s_prog_lp;   /* U85Q-LP instructions loaded */
static const void *tracker_program(bool lp)
{ return s_demod_p64 ? s_u85q_64 : lp ? s_u85q_lp : s_u85q; }
static bool s_flag_format,s_verified_flag;
static const void *tracker_format_program(bool lp,bool flag)
{ return !s_demod_p64 && flag && lp?s_u85q_lp_flag:tracker_program(lp); }
static const uint16_t *tracker_lut(int level)
{ return s_demod_p64 ? p64_table : kp_tables[level]; }

/* 6-bit resistor DAC, LSB first (XIAO D4..D9; 8.2k/3.9k/2k/1k/470R/240R
 * plus 200R). Do not change the tested pin order. */
static const int s_dac_gpio[8] = {23, 24, 11, 12, 8, 9, -1, -1};

DMA_ATTR __attribute__((aligned(64))) uint8_t s_raw_ring[RAW_RING_BYTES];
static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_rx_delimiter;
static parlio_tx_unit_handle_t s_tx;
static bitscrambler_handle_t s_bs;
static bool s_tx_enabled;
static bool s_rx_enabled, s_bs_enabled;
static bool s_transport_fault;
static uint32_t s_stream_generation;
static bool s_stream_generation_overflow;
static unsigned s_output, s_verified_output, s_output_clipped;
static int s_verified_level;
static bool s_verified_lp, s_have_verified;
static uint32_t s_restart_errors, s_rollbacks, s_rollback_fail;
#define LIVE_TX_NODES ((RAW_RING_BYTES + 4091u) / 4092u)
DMA_ATTR __attribute__((aligned(64))) static dma_descriptor_t s_live_tx_nodes[LIVE_TX_NODES];
int s_rx_dma_ch = -1, s_tx_dma_ch = -1;
ring_node_t s_rx_nodes[MAX_RING_DESCRIPTORS];
int s_rx_node_count;
static uint32_t s_tx_empty, s_rx_overflow, s_dma_faults, s_bs_overload;
static struct {
    uint32_t calls, not_ready, source_invalid, pointer_invalid, lapped, timed_out;
    uint32_t max_us, max_advance;
} s_control_copy;

void sync_dma_c2m(const void *addr, size_t size)
{
    if (!addr || !size) return;
    uint32_t start = (uint32_t)addr & ~63u, end = ((uint32_t)addr + size + 63u) & ~63u;
    (void)esp_cache_msync((void *)start, end - start,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

void sync_dma_m2c(const void *addr, size_t size)
{
    if (!addr || !size) return;
    uint32_t start = (uint32_t)addr & ~63u, end = ((uint32_t)addr + size + 63u) & ~63u;
    (void)esp_cache_msync((void *)start, end - start,
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static esp_err_t enable_tx(void)
{
    esp_err_t err = parlio_tx_unit_enable(s_tx);
    if (err == ESP_OK) s_tx_enabled = true;
    return err;
}

static esp_err_t disable_tx(void)
{
    esp_err_t err = parlio_tx_unit_disable(s_tx);
    if (err == ESP_OK) s_tx_enabled = false;
    return err;
}

static esp_err_t prepare_rx(void)
{
    const parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .data_width = 8u,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz = IQ_RATE_HZ,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        /* Same order as rf.c s_iq_pins: Q bits 0..3, I bits 4..7. */
        .data_gpio_nums = {GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
                           GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4},
        .flags = {.free_clk = true, .clk_gate_en = false, .allow_pd = false},
    };
    esp_err_t err = parlio_new_rx_unit(&cfg, &s_rx);
    if (err != ESP_OK) return err;
    const parlio_rx_soft_delimiter_config_t delim = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = sizeof(s_raw_ring),
        .timeout_ticks = 0u,
    };
    err = parlio_new_rx_soft_delimiter(&delim, &s_rx_delimiter);
    if (err != ESP_OK) return err;
    err = parlio_rx_unit_enable(s_rx, false);
    if (err == ESP_OK) s_rx_enabled = true;
    return err;
}

static esp_err_t prepare_tx(void)
{
    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = 40000000u,
        .data_width = 8u,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .trans_queue_depth = 1u,
        .max_transfer_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (unsigned k = 0; k < 8u; ++k) cfg.data_gpio_nums[k] = s_dac_gpio[k];
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (err != ESP_OK) return err;
    const bitscrambler_config_t bs = {.dir = BITSCRAMBLER_DIR_TX, .attach_to = SOC_BITSCRAMBLER_ATTACH_PARL_IO};
    err = bitscrambler_new(&bs, &s_bs);
    if (err != ESP_OK) return err;
    return enable_tx();
}

static esp_err_t start_rx(void)
{
    esp_err_t err = parlio_rx_soft_delimiter_start_stop(s_rx, s_rx_delimiter, true);
    if (err != ESP_OK) return err;
    const parlio_receive_config_t cfg = {
        .delimiter = s_rx_delimiter,
        .flags = {.partial_rx_en = true, .indirect_mount = false},
    };
    return parlio_rx_unit_receive(s_rx, s_raw_ring, sizeof(s_raw_ring), &cfg);
}

/* Use an owned fixed raw-ring TX chain, just as the menu uses an owned
 * raster chain. Re-submitting IDF loop transactions after repeated P hung
 * in its unbounded FIFO-ready spin. This chain never remounts driver loop
 * buffers, and the readiness wait yields with a bounded deadline. */
static esp_err_t start_tx(void)
{
    size_t offset = 0;
    for (unsigned k = 0; k < LIVE_TX_NODES; ++k) {
        dma_descriptor_t *d = &s_live_tx_nodes[k];
        size_t length = sizeof(s_raw_ring) - offset;
        if (length > 4092u) length = 4092u;
        memset(d, 0, sizeof(*d));
        d->dw0.size = length;
        d->dw0.length = length;
        d->dw0.owner = 1;
        d->buffer = s_raw_ring + offset;
        d->next = &s_live_tx_nodes[(k + 1u) % LIVE_TX_NODES];
        offset += length;
    }
    sync_dma_c2m(s_live_tx_nodes, sizeof(s_live_tx_nodes));
    parlio_ll_tx_enable_clock(&PARL_IO, false);
    parlio_ll_tx_reset_clock(&PARL_IO);
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_set_idle_data_value(&PARL_IO, DAC_IDLE_CODE);
    parlio_ll_tx_set_eof_condition(&PARL_IO, PARLIO_LL_TX_EOF_COND_DATA_LEN);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 1);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    AHB_DMA.out_link_addr[s_tx_dma_ch].val = (uint32_t)&s_live_tx_nodes[0];
    AHB_DMA.channel[s_tx_dma_ch].out.out_link.outlink_start_chn = 1;
    int64_t deadline = esp_timer_get_time() + 20000;
    while (!parlio_ll_tx_is_ready(&PARL_IO)) {
        if (esp_timer_get_time() >= deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);   /* taskYIELD alone would still starve lower-priority IDLE */
    }
    parlio_ll_tx_start(&PARL_IO, true);
    parlio_ll_tx_enable_clock(&PARL_IO, true);
    return ESP_OK;
}

/* The CPU LUT port follows lut_mode (ESP32-C5 TRM 44.3.1). On this
 * board, writes/readback under RUN or PAUSE fail; HALT with the normal
 * load/reset/start sequence succeeds. Do not claim a seamless live update. */
static uint16_t s_lut_live[1024];   /* what the LUT RAM holds */
static bool s_lut_known;            /* shadow valid: U85Q program, LUT16 */
static uint8_t s_kp_target;
/* IDF V1 image: 3 header words, 8 instructions of 9 words, LUT16.
 * Reuse the target LUT storage; only the 300-byte prefix is additional.
 * Prepare before halting so the driver writes the target exactly once. */
typedef struct {
    uint32_t prefix[3u + 8u * 9u];
    uint16_t lut[1024];
} tracker_image_t;
static tracker_image_t s_program_work;
#define s_lut_work s_program_work.lut
_Static_assert(offsetof(tracker_image_t, lut) == 300u, "BitScrambler V1 LUT offset");
static uint32_t s_kp_writes, s_kp_verify_fail;
static uint32_t s_kp_switches, s_kp_restart_fail, s_kp_restart_us;
static bool resume_live(const uint16_t *restore_lut);
static bool switch_live(int level, bool lp);
static kp_switch_guard_t s_switch_guard;

bool transport_kp_set(int level)
{
    if (s_demod_p64) return false;  /* P64 is one fixed table, no kp ladder. */
    if (s_transport_fault || !s_lut_known || level < 0 || level >= KP_LEVELS) return false;
    s_kp_target = (uint8_t)level;
    return true;
}

/* Verified level, -1 while a different target is pending or unknown. */
int transport_kp_level(void)
{
    if (s_demod_p64 || !s_lut_known || s_transport_fault) return -1;
    return s_verified_level;
}

int transport_kp_target(void) { return s_lut_known ? s_kp_target : -1; }

static void select_lut_entry(void *ctx, unsigned entry)
{
    (void)ctx;
    BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg0.lut_idx = entry;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static void store_lut_entry(void *ctx, uint16_t value)
{
    (void)ctx;
    BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg1.lut = value;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static uint32_t read_lut_entry(void *ctx)
{
    (void)ctx;
    return BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg1.lut;
}

static bool write_lut_entry(void *ctx, unsigned entry, uint16_t value)
{
    return kp_lut_write16(ctx, entry, value, select_lut_entry, store_lut_entry, read_lut_entry);
}

/* Apply one complete target using the normal pipeline restart. Copy the
 * shadow only after the halted RAM verifies; a failed load resumes baseline
 * video instead of repeatedly retrying writes under the running core. */
static int64_t stage_now(void);
static void stage_note(int stage, int64_t us);
static void stage_hooks(void) { tr_hook_now = stage_now; tr_hook_note = stage_note; }

unsigned transport_kp_tick(void)
{
    stage_hooks();
    if (s_demod_p64) return 0;
    if (!s_lut_known || !s_started || s_menu_active || transport_kp_level() == s_kp_target) return 0;
    uint32_t before = s_kp_writes;
    (void)switch_live(s_kp_target, s_prog_lp);
    return s_kp_writes - before;
}

/* A paused instruction rewrite did not reliably resume video on the board.
 * Program changes therefore use the existing stop/load/reset/start topology,
 * with the verified current kp LUT restored before the core starts. This has
 * a brief video interruption; it is not a sample-seamless program handover. */
static uint32_t s_prog_switches, s_prog_verify_fail, s_prog_restart_fail;
static uint32_t s_prog_restart_us;

static bool program_words(const uint8_t *bin, const uint8_t **inst)
{
    /* bitscrambler_program_hdr_t (esp_driver_bitscrambler): version, hw_rev,
     * hdr_len (words), inst_ct, ... */
    if (bin[0] != 1u || bin[2] != 3u || bin[3] != 8u || bin[4] != 0u || bin[5] != 2u || bin[6] != 1u)
        return false;
    *inst = bin + (size_t)bin[2] * 4u;
    return true;
}

/* All live kp/program writes share one cooldown. Manual P, automatic LP,
 * manual k and mode changes cannot bypass it or alternate two timers. */
static bool switch_live(int level, bool lp)
{
    if (s_demod_p64) return false;
    if (!s_lut_known || !s_started || s_menu_active) return false;
    if (level >= KP_LEVELS || level < -1) return false;
    const uint16_t *lut = level < 0 ? s_lut_live : kp_tables[level];
    bool kp_changed = level >= 0 && level != s_verified_level;
    bool lp_changed = lp != s_prog_lp;
    if (!kp_changed && !lp_changed) return true;
    int64_t began = esp_timer_get_time();
    /* Into LP is the rescue: short gap; everything else keeps the 5 s. */
    if (!(lp_changed && lp ? kp_switch_ready_rescue(&s_switch_guard, began) : kp_switch_ready(&s_switch_guard, began)))
        return false;
    bool previous = s_prog_lp;
    s_prog_lp = lp;
    bool ok = resume_live(lut);
    if (!ok) {
        s_prog_lp = previous;
        if (kp_changed) ++s_kp_restart_fail;
        if (lp_changed) ++s_prog_restart_fail;
    } else {
        if (level >= 0) s_kp_target = (uint8_t)level;
        if (kp_changed) ++s_kp_switches;
        if (lp_changed) ++s_prog_switches;
    }
    int64_t finished = esp_timer_get_time();
    kp_switch_mark(&s_switch_guard, finished);  /* failures interrupted video too */
    if (kp_changed) s_kp_restart_us = (uint32_t)(finished - began);
    if (lp_changed) s_prog_restart_us = (uint32_t)(finished - began);
    {
        uint8_t e[6] = {(uint8_t)(level < 0 ? 0xffu : (unsigned)level), lp, ok};
        const uint32_t us = (uint32_t)(finished - began);
        e[3] = (uint8_t)us; e[4] = (uint8_t)(us >> 8); e[5] = (uint8_t)(us >> 16);
        flightlog_event(FLOG_E_KP, e, sizeof(e));
    }
    return ok;
}

bool transport_apply_auto(int level, bool lp)
{
    /* A profile's analog filter has changed before this call. P64 keeps
     * its fixed table, but still needs the guarded, verified restart and
     * cooldown shared with OUTPUT and decoder changes. */
    if (s_demod_p64) {
        if (s_transport_fault || !s_lut_known || !s_started || s_menu_active ||
            lp || level < 0 || level >= KP_LEVELS) return false;
        if (!kp_switch_ready(&s_switch_guard, esp_timer_get_time())) return false;
        const uint8_t previous = s_kp_target;
        s_kp_target = (uint8_t)level;  /* retain the profile's U85Q fallback */
        stage_hooks();
        const bool ok = resume_live(p64_table);
        if (!ok) s_kp_target = previous;
        kp_switch_mark(&s_switch_guard, esp_timer_get_time());
        return ok;
    }
    stage_hooks();
    return switch_live(level, lp);
}

uint32_t transport_switch_wait_ms(void)
{
    int64_t now = esp_timer_get_time();
    return kp_switch_ready(&s_switch_guard, now) ? 0 :
           (uint32_t)((KP_SWITCH_GAP_US - (now - s_switch_guard.last_switch_us) + 999) / 1000);
}

bool transport_set_lp(bool lp)
{
    if (s_demod_p64) return false;
    if (lp == s_prog_lp) return true;
    /* Menu selection does not touch live RAM; applied at menu exit. */
    if (s_menu_active) { s_prog_lp = lp; return true; }
    return switch_live(-1, lp);
}

bool transport_lp(void) { return s_prog_lp; }

const char *transport_program_name(void)
{ return s_demod_p64 ? P64_MODEL_ID : s_prog_lp ? "U85Q-LP" : "U85Q"; }
bool transport_p64(void) { return s_demod_p64; }

bool transport_set_p64(bool enabled)
{
    if (s_transport_fault) return false;
    if (enabled == s_demod_p64) return true;
    if (s_menu_active || !s_started) {
        s_demod_p64 = enabled;
        s_prog_lp = false;
        return true;
    }
    if (!s_lut_known || !kp_switch_ready(&s_switch_guard, esp_timer_get_time())) return false;
    const bool previous = s_demod_p64, previous_lp = s_prog_lp;
    s_demod_p64 = enabled;
    s_prog_lp = false;
    const bool ok = resume_live(tracker_lut(s_kp_target));
    if (!ok) { s_demod_p64 = previous; s_prog_lp = previous_lp; }
    kp_switch_mark(&s_switch_guard, esp_timer_get_time());
    return ok;
}
bool transport_faulted(void) { return s_transport_fault; }
uint32_t transport_restart_errors(void) { return s_restart_errors; }
unsigned transport_output(void) { return s_output; }
const char *transport_output_name(void)
{
    static const char *const names[OUTPUT_LEVELS] = {
        "1.0x", "1.5x", "2.0x", "2.5x", "3.0x", "3.5x", "4.0x", "4.5x", "5.0x", "0.5x"
    };
    return names[s_output];
}
bool transport_set_output(unsigned level)
{
    if (s_transport_fault || level >= OUTPUT_LEVELS) return false;
    if (level == s_output) return true;
    if (s_menu_active || !s_started) { s_output = level; return true; }
    if (!kp_switch_ready(&s_switch_guard, esp_timer_get_time())) return false;
    s_output = level;
    bool ok = resume_live(s_lut_known ? tracker_lut(s_verified_level) : NULL);
    kp_switch_mark(&s_switch_guard, esp_timer_get_time());
    return ok;
}

/* Selected configuration stays separate from the last fully verified live
 * configuration. Publication waits for both DMA starts, including readback
 * of the complete program and LUT on boot/menu/rollback as well as P/k. */
typedef struct {
    const uint8_t *program;
    int level;
    bool lp, flag;
    unsigned output, clipped;
} restart_t;

/* Comparator-flag DAC pin ownership for the live flywheel format. Ordinary
 * verified state and s_lut_live remain the restoration target. */
typedef struct {
    uint32_t iomux,matrix,pad,enable,output,clamp_matrix,generation;
    bool active,saved,clamped;
} flag_pin_t;
static flag_pin_t *s_flag_pin;
static flag_pin_t *s_flag_storage;
#define FLAG_PIN_MASK (UINT32_C(1)<<23)

static bool IRAM_ATTR flag_pin_clamped(void)
{
    const flag_pin_t *s=s_flag_pin;
    return s && s->saved && s->clamped &&
        REG_READ(IO_MUX_GPIO23_REG)==s->iomux &&
        REG_READ(GPIO_FUNC23_OUT_SEL_CFG_REG)==s->clamp_matrix &&
        REG_READ(GPIO_PIN23_REG)==s->pad &&
        (REG_READ(GPIO_ENABLE_REG)&FLAG_PIN_MASK)==FLAG_PIN_MASK &&
        !(REG_READ(GPIO_OUT_REG)&FLAG_PIN_MASK);
}
static bool flag_pin_restored(void)
{
    const flag_pin_t *s=s_flag_pin;
    return s && s->saved && !s->clamped &&
        REG_READ(IO_MUX_GPIO23_REG)==s->iomux &&
        REG_READ(GPIO_FUNC23_OUT_SEL_CFG_REG)==s->matrix &&
        REG_READ(GPIO_PIN23_REG)==s->pad &&
        (REG_READ(GPIO_ENABLE_REG)&FLAG_PIN_MASK)==s->enable &&
        (REG_READ(GPIO_OUT_REG)&FLAG_PIN_MASK)==s->output;
}
/* Called only while TX is halted, after enable() and before TX_START.
 * Disable this one pad before restoring its latent GPIO output latch. */
static bool flag_pin_apply(bool flag)
{
    flag_pin_t *s=s_flag_pin;
    if(!s || !s->active)return !flag;
    if(!s->saved)return false;
    REG_WRITE(GPIO_ENABLE_W1TC_REG,FLAG_PIN_MASK);
    REG_WRITE(flag?GPIO_OUT_W1TC_REG:s->output?GPIO_OUT_W1TS_REG:GPIO_OUT_W1TC_REG,FLAG_PIN_MASK);
    REG_WRITE(IO_MUX_GPIO23_REG,s->iomux);
    REG_WRITE(GPIO_FUNC23_OUT_SEL_CFG_REG,flag?s->clamp_matrix:s->matrix);
    REG_WRITE(GPIO_PIN23_REG,s->pad);
    if(flag || s->enable)REG_WRITE(GPIO_ENABLE_W1TS_REG,FLAG_PIN_MASK);
    s->clamped=flag;
    if(flag)return flag_pin_clamped();
    return flag_pin_restored();
}

/* Pin identity is acquired once, before any comparator program can run.
 * The small ownership record is independent of optional diagnostic buffers. */
static bool flag_pin_acquire(void)
{
    if(s_flag_pin)return false;
    if(!s_flag_storage)s_flag_storage=calloc(1,sizeof(*s_flag_storage));
    flag_pin_t *s=s_flag_storage;
    if(!s)return false;
    memset(s,0,sizeof(*s));
    s->iomux=REG_READ(IO_MUX_GPIO23_REG);s->matrix=REG_READ(GPIO_FUNC23_OUT_SEL_CFG_REG);
    s->pad=REG_READ(GPIO_PIN23_REG);s->enable=REG_READ(GPIO_ENABLE_REG)&FLAG_PIN_MASK;
    s->output=REG_READ(GPIO_OUT_REG)&FLAG_PIN_MASK;
    if((s->matrix&GPIO_FUNC23_OUT_SEL_M)!=48u ||
       (s->matrix&(GPIO_FUNC23_OUT_INV_SEL_M|GPIO_FUNC23_OE_INV_SEL_M)) ||
       (s->pad&GPIO_PIN23_PAD_DRIVER_M) || !s->enable ||
       ((s->iomux&MCU_SEL_M)>>MCU_SEL_S)!=1u)return false;
    s->clamp_matrix=(s->matrix&~(GPIO_FUNC23_OUT_SEL_M|GPIO_FUNC23_OUT_INV_SEL_M|
        GPIO_FUNC23_OE_INV_SEL_M))|256u|GPIO_FUNC23_OE_SEL_M;
    s->saved=true;s->active=true;s_flag_pin=s;
    return true;
}

static bool stop_part(void *ctx, tr_part_t part)
{
    (void)ctx;
    esp_err_t err = part == TR_TX ? disable_tx() : part == TR_BS ? bitscrambler_disable(s_bs) :
                    parlio_rx_unit_disable(s_rx);
    if (err != ESP_OK) printf("TRANSPORT teardown part=%d error=%s\n", part, esp_err_to_name(err));
    return err == ESP_OK;
}
static bool transport_sync_restart_begin(bool flag);
static void transport_sync_restart_finish(bool verified);
/* Where a restart's picture gap goes (T TRANSPORT_STAGES_US, range audit
 * PR #230): the last restart's time per stage. */
static uint32_t s_stage_us[TR_STAGES], s_quiesce_us;
static int64_t stage_now(void) { return esp_timer_get_time(); }
static void stage_note(int stage, int64_t us)
{
    if (stage < 0) s_quiesce_us = (uint32_t)us;
    else if (stage < TR_STAGES) s_stage_us[stage] = (uint32_t)us;
}
static bool quiesce(void *ctx)
{
    const restart_t *r=ctx;
    if(!transport_sync_restart_begin(r && r->flag) && !transport_sync_live_cancel())return false;
    bool ok = tr_stop_parts(ctx, stop_part, &s_tx_enabled, &s_bs_enabled, &s_rx_enabled, true);
    s_started = false;
    s_rx_node_count = 0; /* observers cannot consume stopped/stale DMA */
    return ok;
}

static esp_err_t start_rings_tail(void);
static bool restart_stage(void *ctx, tr_stage_t stage, bool rollback)
{
    restart_t *r = ctx;
    if (stage == TR_BS_ENABLE && rollback) {
        s_demod_p64 = s_verified_p64;
        s_prog_lp = s_verified_lp;
        s_output = s_verified_output;
        r->lp = s_verified_lp;
        r->level = s_verified_level;
        r->output = s_verified_output;
        r->flag = s_verified_flag;
        s_flag_format=s_verified_flag;
        r->program = tracker_format_program(r->lp,r->flag);
        const uint8_t *inst;
        if (!program_words(r->program, &inst)) return false;
        memcpy(s_program_work.prefix, r->program, sizeof(s_program_work.prefix));
        memcpy(s_lut_work, s_lut_live, sizeof(s_lut_work));
    }
    esp_err_t err = ESP_OK;
    switch (stage) {
    case TR_BS_ENABLE:
        err = bitscrambler_enable(s_bs);
        if (err == ESP_OK) s_bs_enabled = true;
        break;
    case TR_PROGRAM_LOAD:
        err = bitscrambler_load_program(s_bs, &s_program_work);
        if (err == ESP_OK) s_kp_writes += KP_LUT_ENTRIES;
        break;
    case TR_INSTRUCTION_VERIFY: {
        const uint8_t *inst;
        if (!program_words(r->program, &inst)) { ++s_prog_verify_fail; return false; }
        bool verified = true;
        for (unsigned i = 0; i < 8u; ++i)
            for (unsigned w = 0; w < BITSCRAMBLER_LL_INST_LEN_WORDS; ++w) {
                uint32_t expected;
                memcpy(&expected, inst + (i * BITSCRAMBLER_LL_INST_LEN_WORDS + w) * 4u, 4);
                /* Instructions have 257 bits: only bit 0 of word 8 exists. */
                uint32_t mask = w == 8u ? 1u : UINT32_MAX;
                uint32_t actual = bitscrambler_ll_instmem_read(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX, i, w);
                if ((actual & mask) != (expected & mask)) { ++s_prog_verify_fail; verified = false; }
            }
        if (!verified) {
            printf("TRANSPORT restart instruction_verify failed rollback=%u\n", rollback);
            return false;
        }
        break;
    }
    case TR_LUT_VERIFY:
        if (!kp_lut_verify(s_lut_work, NULL, select_lut_entry, read_lut_entry, &s_kp_verify_fail)) {
            printf("TRANSPORT restart lut_verify failed rollback=%u\n", rollback);
            return false;
        }
        break;
    case TR_BS_RESET: err = bitscrambler_reset(s_bs); break;
    case TR_BS_START: err = bitscrambler_start(s_bs); break;
    case TR_RX_ENABLE:
        err = parlio_rx_unit_enable(s_rx, false);
        if (err == ESP_OK) s_rx_enabled = true;
        break;
    case TR_TX_ENABLE:
        err = enable_tx();
        /* A flag-to-flag restart retains the independent sync clock and pad
         * owner. Do not interrupt its clamp while changing the stream. */
        if(err==ESP_OK && !(r->flag && s_verified_flag && flag_pin_clamped()) &&
           !flag_pin_apply(r->flag))return false;
        break;
    case TR_RX_START: err = start_rx(); break;
    case TR_TX_START: err = start_rings_tail(); break;
    default: return false;
    }
    if (err != ESP_OK)
        printf("TRANSPORT restart stage=%d rollback=%u error=%s\n", stage, rollback, esp_err_to_name(err));
    return err == ESP_OK;
}


static int find_dma(bool rx)
{
    for (int k = 0; k < 3; ++k) {
        if (rx && AHB_DMA.channel[k].in.in_peri_sel.peri_in_sel_chn == 9) return k;
        if (!rx && AHB_DMA.channel[k].out.out_peri_sel.peri_out_sel_chn == 9) return k;
    }
    return -1;
}

/* Clear suc_eof on every ring descriptor: the ring becomes endless. */
static int clear_eof(int ch, bool rx)
{
    if (ch < 0 || ch >= 3) return 0;
    uint32_t first = rx ? AHB_DMA.channel[ch].in.in_dscr_bf0.val : AHB_DMA.channel[ch].out.out_dscr_bf0.val;
    if (first < 0x40800000u || first >= 0x40860000u) return 0;
    dma_descriptor_t *d = (dma_descriptor_t *)(uintptr_t)first;
    int count = 0;
    while (d && count < MAX_RING_DESCRIPTORS) {
        d->dw0.suc_eof = 0;
        sync_dma_c2m(d, sizeof(*d));
        if (rx) {
            s_rx_nodes[count].dscr = d;
            s_rx_nodes[count].buffer = (uint8_t *)d->buffer;
            s_rx_nodes[count].length = d->dw0.size ? d->dw0.size : CONTROL_BYTES;
        }
        d = d->next;
        ++count;
        if ((uintptr_t)d == first) break;
    }
    if (rx) s_rx_node_count = count;
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    return count;
}

static void quiet_interrupts(void)
{
    for (int k = 0; k < 3; ++k) { AHB_DMA.in_intr[k].ena.val = 0; AHB_DMA.out_intr[k].ena.val = 0; }
    PARL_IO.int_ena.val = 0;
    /* External-enable EOF selection: soft-delimiter RX never stalls on EOF. */
    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 1;
}

/* RX first, TX three quarters of a ring later (nominal614us). Compared with
 * half a ring this buys8192 bytes for whole-descriptor/write-history guards,
 * for205us more video latency and no extra memory. Actual frontiers, never
 * this requested delay, decide whether a future keeper write is admissible. */
static esp_err_t start_rings_tail(void)
{
    /* Invalidate every old DMA epoch at entry, including a failed start or
     * rollback replacement. This changes no stream data or clock setting. */
    if (++s_stream_generation == 0) s_stream_generation_overflow = true;
    quiet_interrupts();
    /* Make RX endless BEFORE any wait for TX. start_tx may wait/yield for
     * FIFO readiness; deferring this until afterward could let RX reach EOF. */
    s_rx_dma_ch = find_dma(true);
    if (s_rx_dma_ch < 0) return ESP_ERR_NOT_FOUND;
    const int rx_channel = s_rx_dma_ch;
    if (clear_eof(s_rx_dma_ch, true) < 3) return ESP_ERR_INVALID_STATE;
    esp_rom_delay_us((uint64_t)LIVE_PIPELINE_LAG_BYTES * 1000000ULL / IQ_RATE_HZ);
    s_tx_dma_ch = find_dma(false);
    if (s_tx_dma_ch < 0) return ESP_ERR_NOT_FOUND;
    const int tx_channel = s_tx_dma_ch;
    esp_err_t err = start_tx();
    if (err != ESP_OK) return err;
    int rx_after = find_dma(true), tx_after = find_dma(false);
    if (rx_after < 0 || tx_after < 0) return ESP_ERR_NOT_FOUND;
    if (rx_after != rx_channel || tx_after != tx_channel) return ESP_ERR_INVALID_STATE;
    quiet_interrupts();
    if (clear_eof(s_tx_dma_ch, false) < 1)
        return ESP_ERR_INVALID_STATE;
    PARL_IO.int_clr.val = UINT32_MAX;
    if (s_rx_dma_ch >= 0) AHB_DMA.in_intr[s_rx_dma_ch].clr.val = UINT32_MAX;
    if (s_tx_dma_ch >= 0) AHB_DMA.out_intr[s_tx_dma_ch].clr.val = UINT32_MAX;
    BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].val = 1u << 31;
    return ESP_OK;
}

esp_err_t transport_start(void)
{
    memset(s_raw_ring, 0, sizeof(s_raw_ring));
    sync_dma_c2m(s_raw_ring, sizeof(s_raw_ring));
    esp_err_t err = prepare_rx();
    if (err == ESP_OK) err = prepare_tx();
    if (err != ESP_OK) {
        (void)quiesce(NULL);
        s_transport_fault = true;
        return err;
    }
    return resume_live(NULL) ? ESP_OK : ESP_FAIL;
}

/* Start level of a profile (kp_tables.h), LP off. Never under live video:
 * only before the pipeline starts or while the menu / a status screen owns
 * TX; the LUT is then loaded and verified by the normal start sequence. A
 * live profile switch uses transport_apply_auto() (guarded restart). */
bool transport_set_base_level(int level)
{
    if (level < 0 || level >= KP_LEVELS) return false;
    if (s_started && !s_menu_active) {
        printf("BASE level change refused: live video\n");
        return false;
    }
    s_kp_target = (uint8_t)level;
    s_prog_lp = false;
    return true;
}

/* Live -> menu: the raster takes TX, but RX keeps collecting IQ for filter,
 * DC and profile calibration. Full recovery quiesce stops RX as well. */
void transport_stop_tx(void)
{
    if(!transport_sync_live_cancel()){s_transport_fault=true;return;}
    bool ok = tr_stop_parts(NULL, stop_part, &s_tx_enabled, &s_bs_enabled, &s_rx_enabled, false);
    s_started = false;
    if (!ok) {
        s_transport_fault = true;
        printf("TRANSPORT ERROR: teardown failed; live recovery latched OFF, USB remains available\n");
    }
}

/* Start a standalone descriptor chain (menu raster) on the TX GDMA. */
void transport_start_chain(dma_descriptor_t *first)
{
    if(!transport_sync_live_cancel()){s_transport_fault=true;return;}
    if (s_transport_fault || enable_tx() != ESP_OK) return;
    s_tx_dma_ch = find_dma(false);
    if (s_tx_dma_ch < 0) { (void)disable_tx(); s_transport_fault = true; return; }
    AHB_DMA.out_intr[s_tx_dma_ch].ena.val = 0;
    AHB_DMA.out_intr[s_tx_dma_ch].clr.val = UINT32_MAX;
    PARL_IO.int_ena.val = 0;
    parlio_ll_tx_enable_clock(&PARL_IO, false);
    parlio_ll_tx_reset_clock(&PARL_IO);
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_set_idle_data_value(&PARL_IO, DAC_IDLE_CODE);
    parlio_ll_tx_set_eof_condition(&PARL_IO, PARLIO_LL_TX_EOF_COND_DATA_LEN);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 1);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    AHB_DMA.out_link_addr[s_tx_dma_ch].val = (uint32_t)first;
    AHB_DMA.channel[s_tx_dma_ch].out.out_link.outlink_start_chn = 1;
    /* Wait up to 50 ms, yielding; PARLIO outputs the idle code meanwhile. */
    int64_t deadline = esp_timer_get_time() + 50000;
    while (!parlio_ll_tx_is_ready(&PARL_IO) && esp_timer_get_time() < deadline) vTaskDelay(1);
    if (!parlio_ll_tx_is_ready(&PARL_IO)) {
        (void)disable_tx(); s_transport_fault = true;
        printf("TRANSPORT ERROR: menu TX timeout, USB remains available\n");
        return;
    }
    parlio_ll_tx_start(&PARL_IO, true);
    parlio_ll_tx_enable_clock(&PARL_IO, true);
}

void transport_stop_chain(void)
{
    if(!transport_sync_live_cancel()){s_transport_fault=true;return;}
    if (s_tx_enabled && disable_tx() != ESP_OK) s_transport_fault = true;
}

/* Menu -> live: the exact boot topology again. */
static bool resume_live(const uint16_t *restore_lut)
{
    if (s_transport_fault) return false;
    restart_t r = {.program=tracker_format_program(s_prog_lp,s_flag_format), .level=s_kp_target,
        .lp=s_prog_lp,.flag=s_flag_format,.output=s_output};
    const uint8_t *program_inst;
    if (!program_words(r.program, &program_inst)) return false;
    memcpy(s_program_work.prefix, r.program, sizeof(s_program_work.prefix));
    if (!restore_lut) restore_lut = tracker_lut(s_kp_target);
    if(restore_lut==s_lut_live && s_flag_format!=s_verified_flag)
        restore_lut=tracker_lut(s_verified_level);
    if (restore_lut == s_lut_live) {
        memcpy(s_lut_work, s_lut_live, sizeof(s_lut_work));
        r.level = s_verified_level;
        r.clipped = s_output_clipped;
    } else {
        const uint8_t *inst;
        if (!program_words(r.program, &inst) || r.program[6] != 1u) return false;
        for (unsigned i = 0; i < 1024u; ++i) {
            bool clipped;
            s_lut_work[i] = r.flag?sync_flag_entry(restore_lut[i],s_output,&clipped):
                                  output_entry(restore_lut[i], s_output, &clipped);
            r.clipped += clipped;
        }
        for (int k = 0; k < KP_LEVELS; ++k) if (restore_lut == kp_tables[k]) r.level = k;
    }
    tr_result_t result = tr_restart(&r, quiesce, restart_stage, s_have_verified);
    if (result != TR_APPLIED) ++s_restart_errors;
    if (result == TR_LATCHED) {
        ++s_rollback_fail;
        s_transport_fault = true;
        s_lut_known = false;
        transport_sync_restart_finish(false);
        printf("TRANSPORT ERROR: verified recovery failed; live OFF latched, USB/reflash available\n");
        return false;
    }
    s_started = true;
    if (result == TR_ROLLED_BACK) {
        ++s_rollbacks;
        s_kp_target = (uint8_t)s_verified_level;
        transport_sync_restart_finish(true);
        printf("TRANSPORT rollback=verified program=%s kp_level=%d output=%s\n",
               transport_program_name(), s_verified_level, transport_output_name());
        return false;
    }
    memcpy(s_lut_live, s_lut_work, sizeof(s_lut_live));
    const bool output_changed = s_have_verified && s_verified_output != r.output;
    s_verified_lp = r.lp;
    s_verified_p64 = s_demod_p64;
    s_verified_flag=r.flag;
    s_verified_level = r.level;
    s_verified_output = r.output;
    s_output_clipped = r.clipped;
    s_have_verified = true;
    s_lut_known = true;
    s_kp_target = (uint8_t)r.level;
    transport_sync_restart_finish(true);
    /* Automatic kp/program changes never write NVS. A failed restart or
     * rollback returns above and cannot replace the remembered output. */
    if (output_changed) settings_save_output(r.output);
    return true;
}

static bool transport_flag_format(bool flag)
{
    if(s_transport_fault || !s_started || !s_have_verified || !s_lut_known)return false;
    if(s_verified_flag==flag)return true;
    if(!transport_sync_live_cancel())return false;
    if(flag && !flag_pin_acquire())return false;
    s_flag_format=flag;
    bool ok=resume_live(tracker_lut(s_verified_level));
    if(!s_transport_fault && !s_verified_flag) {
        if(!flag_pin_restored()){s_transport_fault=true;s_lut_known=false;return false;}
        s_flag_pin->active=false;s_flag_pin=NULL;
    } else if(!s_transport_fault && !flag_pin_clamped()) {
        s_transport_fault=true;s_lut_known=false;return false;
    }
    return ok;
}

void transport_resume_live(void)
{
    if (!resume_live(NULL)) printf("TRANSPORT menu return failed; inspect T recovery status\n");
    kp_switch_mark(&s_switch_guard, esp_timer_get_time());
}

void transport_poll_faults(void)
{
    if (PARL_IO.int_raw.tx_fifo_rempty_int_raw) { ++s_tx_empty; PARL_IO.int_clr.tx_fifo_rempty_int_clr = 1; }
    if (PARL_IO.int_raw.rx_fifo_wovf_int_raw) { ++s_rx_overflow; PARL_IO.int_clr.rx_fifo_wovf_int_clr = 1; }
    if (s_rx_dma_ch >= 0) {
        uint32_t raw = AHB_DMA.in_intr[s_rx_dma_ch].raw.val & 0xfcu;
        if (raw) { ++s_dma_faults; AHB_DMA.in_intr[s_rx_dma_ch].clr.val = raw; }
    }
    if (s_tx_dma_ch >= 0) {
        uint32_t raw = AHB_DMA.out_intr[s_tx_dma_ch].raw.val & 0x7cu;
        if (raw) { ++s_dma_faults; AHB_DMA.out_intr[s_tx_dma_ch].clr.val = raw; }
    }
    if (BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].eof_overload) {
        ++s_bs_overload;
        BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].val = 1u << 31;
    }
}

uint32_t transport_fault_count(void)
{
    return s_tx_empty + s_rx_overflow + s_dma_faults + s_bs_overload;
}

/* Transport identity the live flywheel was started on. */
typedef struct {
    uint32_t generation;
    int level;
    unsigned output;
    bool lp;
} sync_live_context_t;

static bool s_output_probe_busy;

/* Allocate after startup so observer state cannot fragment the control stack
 * allocation. Lifecycle is serialized by short PHY ownership, not held while
 * the resident peripheral observes video or the control task sleeps. */
typedef struct {
    sync_live_context_t context;
    bool requested,authorized,fault,output_requested;
    bool coasting_restart;
    uint32_t stream_generation;
    int64_t retry_us;
    size_t channel;
    uint32_t starts,stops,failures;
} sync_live_owner_t;
static sync_live_owner_t *s_live_owner;
static output_telem_acc_t s_output_telem;
static bool s_output_observation_available;

void transport_flog_identity(uint8_t *demod, uint8_t *output_level, bool *flag_format)
{
    const bool verified = s_started && s_tx_enabled && s_bs_enabled && s_have_verified && s_lut_known &&
                          !s_transport_fault;
    if (demod) *demod = verified ? (s_verified_lp ? FLOG_DEMOD_U85Q_LP_1A09F5C092E8 :
                                                    FLOG_DEMOD_U85Q_1A09F5C092E8) : FLOG_DEMOD_UNKNOWN;
    if (output_level) *output_level = verified ? (uint8_t)s_verified_output : 0xffu;
    if (flag_format) *flag_format = verified && s_verified_flag;
}

/* Existing ISR/RMT counters only: one bounded task-side rollup sample per
 * 2-ms owner tick. Missing coverage terminates an unlock run as UNKNOWN. */
static void transport_output_telem_sample(bool observation_available)
{
    s_output_observation_available = observation_available;
    slo_status_t status;
    sync_live_output_status(&status);
    const output_telem_sample_t sample = {
        .repaired = status.repaired, .suppressed = status.suppressed,
        .camera_lines = status.camera_lines, .observer_gaps = status.observer_gaps,
        .standard = status.standard, .active = status.active,
        .observer_fresh = status.observer_fresh, .camera_present = status.camera_present,
        .h_trusted = status.h_trusted, .v_trusted = status.v_trusted,
    };
    const uint32_t gaps_before = s_output_telem.observer_gaps;
    const bool unlock = output_telem_add(&s_output_telem, &sample, (uint32_t)(esp_timer_get_time() / 1000),
                                         observation_available);
    const uint32_t new_gaps = s_output_telem.observer_gaps - gaps_before;
    uint8_t demod, level;
    transport_flog_identity(&demod, &level, NULL);
    const uint8_t state = (uint8_t)((status.observer_fresh ? 1u : 0u) | (status.h_trusted ? 2u : 0u) |
                                    (status.v_trusted ? 4u : 0u) | (status.camera_present ? 8u : 0u));
    if (new_gaps) {
        const uint8_t e[4] = {flog_sat8(new_gaps), demod, status.standard, state};
        flightlog_event(FLOG_E_OUTPUT_GAP, e, sizeof(e));
    }
    if (unlock) {
        const uint8_t e[4] = {demod, status.standard, state, level};
        flightlog_event(FLOG_E_OUTPUT_UNLOCK, e, sizeof(e));
        flightlog_snapshot_request(FLOG_SNAP_OUTPUT_UNLOCK);
    }
}

void transport_flog_output(transport_flog_output_t *out)
{
    if (!out) return;
    slo_status_t status;
    sync_live_output_status(&status);
    if (!s_output_observation_available) {
        status.observer_fresh = status.h_trusted = status.v_trusted = status.camera_present = false;
    }
    output_telem_roll_t roll;
    output_telem_roll(&s_output_telem, (uint32_t)(esp_timer_get_time() / 1000), &roll);
    bool flag_format;
    transport_flog_identity(&out->demod, &out->output_level, &flag_format);
    out->flags = (uint8_t)((out->demod ? FLOG_OUT_DEMOD_VERIFIED : 0u) |
                           (transport_sync_flywheel_enabled() ? FLOG_OUT_FLYWHEEL_REQUESTED : 0u) |
                           (status.active ? FLOG_OUT_FLYWHEEL_ACTIVE : 0u) |
                           (status.observer_fresh ? FLOG_OUT_OBSERVER_FRESH : 0u) |
                           (status.h_trusted ? FLOG_OUT_H_TRUSTED : 0u) |
                           (status.v_trusted ? FLOG_OUT_V_TRUSTED : 0u) |
                           (status.camera_present ? FLOG_OUT_CAMERA_PRESENT : 0u) |
                           (flag_format ? FLOG_OUT_FLAG_FORMAT : 0u));
    out->standard = roll.standard ? roll.standard : status.standard;
    out->samples = output_telem_sat16(roll.samples);
    out->fresh_samples = output_telem_sat16(roll.fresh_samples);
    out->h_lock_pct = output_telem_pct(roll.h_locked_samples, roll.fresh_samples);
    out->v_lock_pct = output_telem_pct(roll.v_locked_samples, roll.fresh_samples);
    out->native_h_lines = output_telem_sat16(roll.native_h_lines);
    out->repaired_tips = output_telem_sat16(roll.repaired);
    out->suppressed_tips = output_telem_sat16(roll.suppressed);
    out->longest_h_unlock_ms = output_telem_sat16(roll.longest_unlock_ms);
    out->observer_gaps = flog_sat8(roll.observer_gaps);
}

static bool IRAM_ATTR transport_sync_live_valid(uint32_t generation)
{
    const sync_live_owner_t *s=s_live_owner;
    if(!(s && s->authorized && !s->fault && s_have_verified &&
        s_lut_known && s_verified_flag && s_flag_format && flag_pin_clamped() &&
        !s_transport_fault && !s_stream_generation_overflow &&
        !s_menu_active && !s_scan_active && generation==s->context.generation))return false;
    /* The peripheral lease owns the absolute output clock; a replacement DMA
     * stream gets a separate generation. Only this bounded verified restart
     * may keep sync running while the stream itself is stopped. */
    if(s->coasting_restart)return true;
    return s_started && s_rx_enabled && s_tx_enabled && s_bs_enabled &&
        s_stream_generation==s->stream_generation &&
        s_verified_level==s->context.level && s_verified_lp==s->context.lp &&
        s_verified_output==s->context.output;
}

static bool transport_sync_restart_begin(bool flag)
{
    sync_live_owner_t *s=s_live_owner;
    if(!flag || !s || !s->requested || !s->authorized || s->fault ||
       !s_verified_flag || !s_flag_format || s_transport_fault ||
       s_menu_active || s_scan_active || !flag_pin_clamped() ||
       !sync_output_live_active())return false;
    if(!s->coasting_restart && !transport_sync_live_valid(s->context.generation))return false;
    s->coasting_restart=true;
    sync_live_output_phase(4u);
    sync_output_live_invalidate(); /* revoke source before stopping any producer */
    return true;
}

static void transport_sync_restart_finish(bool verified)
{
    sync_live_owner_t *s=s_live_owner;
    if(!s || !s->coasting_restart)return;
    if(!verified || !s_verified_flag || !s_flag_format || !flag_pin_clamped()) {
        (void)transport_sync_live_cancel();
        return;
    }
    s->context.level=s_verified_level;s->context.lp=s_verified_lp;
    s->context.output=s_verified_output;s->stream_generation=s_stream_generation;
    __sync_synchronize();
    s->coasting_restart=false;
    sync_live_output_phase(0u);
}

bool transport_sync_live_cancel(void)
{
    if(!s_live_owner)return true;
    s_live_owner->authorized=false;
    s_live_owner->coasting_restart=false;
    sync_live_output_phase(0u);
    __sync_synchronize();
    if(!sync_live_output_end()){s_live_owner->fault=true;return false;}
    bool active=sync_output_live_active();
    bool ok=sync_output_live_end();
    if(ok)s_output_probe_busy=false;
    if(active)++s_live_owner->stops;
    if(!ok){s_live_owner->fault=true;++s_live_owner->failures;}
    return ok;
}

static volatile bool s_sync_live_hold;
void transport_sync_live_hold(bool on) { s_sync_live_hold=on; }

bool transport_sync_live_release(void)
{
    if(!s_live_owner || !sync_output_live_active())return true;
    if(!phy_try_actuator(phy_generation()))return false;
    bool ok=transport_sync_live_cancel();
    phy_end_actuator();
    return ok && !sync_output_live_active();
}

/* Saved user setting (menu FLYWHEEL, console U); ON when never saved
 * (operator, 2026-10-18, after the PR #218 witness fix held on the board).
 * A saved OFF is respected and never allocates the observer/output, so
 * its heap stays free for the menu. */
static bool s_flywheel_boot=true;
void transport_sync_flywheel_set_boot(bool on) { s_flywheel_boot=on; }
bool transport_sync_flywheel_enabled(void)
{
    return s_live_owner ? s_live_owner->output_requested : s_flywheel_boot;
}

void transport_sync_flywheel_set(bool on, bool save)
{
    if(on && !s_live_owner)s_live_owner=calloc(1,sizeof(*s_live_owner));
    if(on && !s_live_owner){printf("SYNC_FLYWHEEL refused=memory\n");return;}
    s_flywheel_boot=on;
    if(s_live_owner) {
        /* The control owner stops/starts it on its next tick. */
        s_live_owner->output_requested=on;
        s_live_owner->requested=on;
    }
    if(save)settings_save_flywheel(on);
    printf("SYNC_FLYWHEEL requested=%u saved=%u\n",(unsigned)on,(unsigned)save);
}

void transport_sync_flywheel_toggle(void)
{
    transport_sync_flywheel_set(!transport_sync_flywheel_enabled(),true);
}

static bool IRAM_ATTR transport_sync_native_source(void *arg,uint32_t *epoch)
{
    (void)arg;
    /* Source validity only (markers, RX rearm, receive mutations). Gating
     * on the probe's strict model hid every native edge from the ISR on the
     * board (camera_qualified=0): tips were repaired over the camera.
     * A coasting restart has stopped the producers: nothing is native. */
    return s_live_owner && !s_live_owner->coasting_restart && sync_output_live_source(NULL,epoch);
}

void transport_sync_live_tick(void)
{
    /* Saved setting (default ON): allocate lazily on the first control tick
     * (after receiver startup, same as pressing U) so startup RAM is unchanged. */
    static bool boot_default_applied;
    if(!boot_default_applied) {
        boot_default_applied=true;
        if(!s_live_owner && s_flywheel_boot) {
            s_live_owner=calloc(1,sizeof(*s_live_owner));
            if(s_live_owner) {
                s_live_owner->requested=s_live_owner->output_requested=true;
                printf("SYNC_FLYWHEEL requested=1 boot=saved_or_default_on\n");
            } else printf("SYNC_FLYWHEEL refused=memory\n");
        } else if(!s_flywheel_boot) printf("SYNC_FLYWHEEL requested=0 boot=saved_off\n");
    }
    sync_live_owner_t *s=s_live_owner;
    if(!s || s->fault){transport_output_telem_sample(false);return;}
    if(!phy_try_actuator(phy_generation())){transport_output_telem_sample(false);return;}
    bool eligible=s->requested && !s_sync_live_hold && !calibration_in_startup() && !receiver_paused() &&
        !s_menu_active && !s_scan_active && s_started && s_have_verified && s_lut_known && !s_transport_fault;
    bool active=sync_output_live_active();
    if(active && (!eligible || !transport_sync_live_valid(s->context.generation))) {
        (void)transport_sync_live_cancel();active=false;
    }
    int64_t now=esp_timer_get_time();
    if(!s->requested && s_verified_flag && !active && now>=s->retry_us) {
        if(!transport_flag_format(false)){++s->failures;s->retry_us=now+1000000;}
    }
    if(eligible && !s_verified_flag && !active && now>=s->retry_us) {
        if(!transport_flag_format(true)) {++s->failures;s->retry_us=now+1000000;}
    }
    if(eligible && s_verified_flag && !s->fault && !active && now>=s->retry_us && !s_output_probe_busy) {
        s->context=(sync_live_context_t){s_stream_generation,s_verified_level,s_verified_output,s_verified_lp};
        s->stream_generation=s_stream_generation;
        s->authorized=true;
        phy_set_observation_hook(sync_output_live_invalidate);
        sync_output_live_pulse_hook(sync_live_output_native_pulse);
        if(sync_output_live_begin_flag(s->context.generation,transport_sync_live_valid)) {
            sync_output_live_before_close(sync_live_output_end);
            s_output_probe_busy=true;++s->starts;active=true;
            s->channel=rf_channel_index();
        } else {s->authorized=false;++s->failures;s->retry_us=now+1000000;}
    }
    if(active && s->channel!=rf_channel_index()) {
        sync_output_live_forget();s->channel=rf_channel_index();
    }
    sync_live_output_phase(1u);
    if(active && !sync_output_live_poll()) {
        ++s->failures;(void)transport_sync_live_cancel();s->retry_us=now+1000000;
    }
    if(active && sync_output_live_active()) {
        sync_live_output_phase(2u);
        slo_status_t output;sync_live_output_status(&output);
        /* The snapshot drains native pulses into the output's guard, which
         * alone owns the grid; the probe's H/V model only reports which
         * comparator polarity carries sync. */
        sync_output_live_model_t m;
        bool have=sync_output_live_snapshot(&m);
        if(have && m.tracking)sync_live_output_native_polarity(m.inverted);
        if(!s->output_requested && output.active) {
            if(!sync_live_output_end())s->fault=true;
        } else if(s->output_requested && have && !output.active && !output.fault) {
            slg_standard_t fallback=s_std_mode==STD_MODE_PAL?SLG_PAL:
                s_std_mode==STD_MODE_NTSC?SLG_NTSC:s_video_std==VIDEO_STD_NTSC?SLG_NTSC:SLG_PAL;
            if(!sync_live_output_flag_begin(fallback,s->context.generation,transport_sync_live_valid,
                transport_sync_native_source,NULL))++s->failures;
        }
        sync_live_output_guard_tick();
        sync_live_output_status(&output);
        if(output.active && !sync_live_output_poll())++s->failures;
    }
    sync_live_output_phase(0u);
    phy_end_actuator();
    transport_output_telem_sample(true);
}

void transport_sync_live_print(void)
{
    sync_live_owner_t *s=s_live_owner;
    printf("SYNC_LIVE requested=%u active=%u fault=%u starts=%lu stops=%lu failures=%lu boot_default=ON\n",
        s?(unsigned)s->requested:0u,(unsigned)sync_output_live_active(),s?(unsigned)s->fault:0u,
        s?(unsigned long)s->starts:0ul,s?(unsigned long)s->stops:0ul,s?(unsigned long)s->failures:0ul);
    sync_output_live_model_t model;
    bool have_model=false,have_diag=false;
    char diagnostic[768];
    if(phy_try_actuator(phy_generation())) {
        have_model=sync_output_live_snapshot(&model);
        have_diag=sync_output_live_diagnostics(diagnostic,sizeof(diagnostic));
        phy_end_actuator();
    }
    if(have_diag)printf("%s",diagnostic);
    if(have_model)
        printf("SYNC_LIVE_MODEL valid=%u tracking=%u standard=%u line_ticks_x32=%lu last_h=%llu field=%llu now=%llu phase_span_ticks=%lld inverted=%u boot_default=ON\n",
            (unsigned)model.valid,(unsigned)model.tracking,(unsigned)model.standard,(unsigned long)model.line_ticks_x32,
            (unsigned long long)model.last_h,(unsigned long long)model.field,
            (unsigned long long)model.now,(long long)(model.phase_hi-model.phase_lo),(unsigned)model.inverted);
    slo_status_t output;sync_live_output_status(&output);
    printf("SYNC_FLYWHEEL requested=%u active=%u repaired=%lu suppressed=%lu batches=%lu late_ticks=%lu missed=%lu miss_kind=%lu queue_low=%lu fault=%lu "
        "irq_delay_ticks=%lu callback_ticks=%lu budget_yields=%lu miss_phase=%lu miss_other=%lu miss_observer=%lu miss_snapshot=%lu miss_update=%lu miss_restart=%lu miss_reserved=%lu "
        "guard_yield=%u yielded=%lu h_jumps=%lu v_jumps=%lu trims=%lu pll_updates=%lu pll_rejected=%lu pll_ppb=%ld acquired=%lu camera_present=%u camera_lines=%lu camera_yields=%lu h_trusted=%u v_trusted=%u std_switches=%lu monitor_lock_proven=0\n",
        s?(unsigned)s->output_requested:0u,(unsigned)output.active,
        (unsigned long)output.repaired,(unsigned long)output.suppressed,(unsigned long)output.batches,
        (unsigned long)output.max_late_ticks,(unsigned long)output.missed,(unsigned long)output.miss_kind,
        (unsigned long)output.queue_low,(unsigned long)output.fault,
        (unsigned long)output.max_irq_delay_ticks,(unsigned long)output.max_callback_ticks,
        (unsigned long)output.budget_yields,
        (unsigned long)output.miss_phase,(unsigned long)output.misses_by_phase[0],
        (unsigned long)output.misses_by_phase[1],(unsigned long)output.misses_by_phase[2],
        (unsigned long)output.misses_by_phase[3],(unsigned long)output.misses_by_phase[4],
        (unsigned long)output.misses_by_phase[5],
        (unsigned)output.guard_yield,(unsigned long)output.yielded,(unsigned long)output.guard_h_jumps,
        (unsigned long)output.guard_v_jumps,(unsigned long)output.guard_trims,
        (unsigned long)output.pll_updates,(unsigned long)output.pll_rejected,(long)output.pll_ppb,
        (unsigned long)output.guard_acquired,(unsigned)output.camera_present,
        (unsigned long)output.camera_lines,(unsigned long)output.camera_yields,
        (unsigned)output.h_trusted,(unsigned)output.v_trusted,(unsigned long)output.std_switches);
    printf("SYNC_FLYWHEEL_WITNESS observer_fresh=%u sample_age_ticks=%lu observer_yields=%lu source_yields=%lu "
        "observer_gaps=%lu stale_native=%lu\n",(unsigned)output.observer_fresh,(unsigned long)output.sample_age_ticks,
        (unsigned long)output.observer_yields,(unsigned long)output.source_yields,
        (unsigned long)output.observer_gaps,(unsigned long)output.stale_native);
    /* ISR occupancy since the previous print (task side, 240 MHz cycles);
     * a restarted output (counters went back) starts a new interval. */
    static uint64_t prev_us,prev_isr,prev_wait,prev_wasted,prev_hold;
    const uint64_t now_us=(uint64_t)esp_timer_get_time();
    if(output.isr_cycles<prev_isr || output.wait_cycles<prev_wait || output.suppressed_wait_cycles<prev_wasted ||
       output.hold_cycles<prev_hold)prev_us=0;
    const uint64_t span=prev_us && now_us>prev_us ? (now_us-prev_us)*240u : 0u;
    #define SLO_PERMILLE(x,p) (span ? (unsigned long)(((x)-(p))*1000u/span) : 0ul)
    printf("SYNC_FLYWHEEL_LOAD active=%u interval_ms=%lu isr_calls=%lu max_isr_us=%lu isr_permille=%lu wait_permille=%lu "
        "suppressed_wait_permille=%lu hold_permille=%lu measured_not_estimated=1\n",(unsigned)output.active,
        (unsigned long)(span/240000u),(unsigned long)output.isr_calls,(unsigned long)(output.max_isr_cycles/240u),
        SLO_PERMILLE(output.isr_cycles,prev_isr),SLO_PERMILLE(output.wait_cycles,prev_wait),
        SLO_PERMILLE(output.suppressed_wait_cycles,prev_wasted),SLO_PERMILLE(output.hold_cycles,prev_hold));
    #undef SLO_PERMILLE
    prev_us=now_us;prev_isr=output.isr_cycles;prev_wait=output.wait_cycles;
    prev_wasted=output.suppressed_wait_cycles;prev_hold=output.hold_cycles;
}



void transport_print(void)
{
    const char *verified_demod = s_have_verified && s_lut_known && !s_transport_fault ?
        (s_verified_p64 ? P64_MODEL_ID : s_verified_lp ? "U85Q-LP" : "U85Q") : "unknown";
    printf("DEMOD selected=%s verified=%s p64_model=%s p64_table_sha256=%s\n",
           transport_program_name(), verified_demod, P64_MODEL_ID, P64_TABLE_SHA256);
    printf("TRANSPORT_STAGES_US quiesce=%lu bs_enable=%lu program_load=%lu instruction_verify=%lu lut_verify=%lu "
           "bs_reset=%lu bs_start=%lu rx_enable=%lu tx_enable=%lu rx_start=%lu tx_start=%lu (last restart)\n",
           (unsigned long)s_quiesce_us, (unsigned long)s_stage_us[TR_BS_ENABLE], (unsigned long)s_stage_us[TR_PROGRAM_LOAD],
           (unsigned long)s_stage_us[TR_INSTRUCTION_VERIFY], (unsigned long)s_stage_us[TR_LUT_VERIFY],
           (unsigned long)s_stage_us[TR_BS_RESET], (unsigned long)s_stage_us[TR_BS_START],
           (unsigned long)s_stage_us[TR_RX_ENABLE], (unsigned long)s_stage_us[TR_TX_ENABLE],
           (unsigned long)s_stage_us[TR_RX_START], (unsigned long)s_stage_us[TR_TX_START]);
    printf("VIDEO_OUTPUT source=%s standard=%s\n", s_menu_active ? "MENU" : s_started ? "LIVE" : "OFF",
           s_video_std == VIDEO_STD_PAL ? "PAL" : "NTSC");
    printf("OUTPUT level=%s boot_restore=saved default=1.0x clipped_lut_entries=%u recovery_errors=%lu rollbacks=%lu "
           "rollback_fail=%lu fault_latched=%u live=%u\n", transport_output_name(), s_output_clipped,
           (unsigned long)s_restart_errors, (unsigned long)s_rollbacks, (unsigned long)s_rollback_fail,
           s_transport_fault, s_started);
    printf("TRANSPORT rx_descriptors=%d tx_empty=%lu rx_overflow=%lu dma_faults=%lu bs_overload=%lu "
           "kp_level=%d kp_target=%d kp_writes=%lu kp_verify_fail=%lu program=%s flag_format=%u prog_switches=%lu "
           "switch_wait_ms=%lu kp_switches=%lu kp_restart_fail=%lu kp_restart_us=%lu prog_verify_fail=%lu prog_restart_fail=%lu prog_restart_us=%lu\n",
           s_rx_node_count, (unsigned long)s_tx_empty, (unsigned long)s_rx_overflow,
           (unsigned long)s_dma_faults, (unsigned long)s_bs_overload, transport_kp_level(),
           transport_kp_target(), (unsigned long)s_kp_writes, (unsigned long)s_kp_verify_fail,
           transport_program_name(),(unsigned)s_verified_flag, (unsigned long)s_prog_switches,
           (unsigned long)transport_switch_wait_ms(), (unsigned long)s_kp_switches, (unsigned long)s_kp_restart_fail, (unsigned long)s_kp_restart_us,
           (unsigned long)s_prog_verify_fail,
           (unsigned long)s_prog_restart_fail, (unsigned long)s_prog_restart_us);
    printf("CONTROL_COPY calls=%lu not_ready=%lu source_invalid=%lu pointer_invalid=%lu "
           "lapped=%lu timed_out=%lu max_us=%lu max_advance=%lu\n",
           (unsigned long)s_control_copy.calls, (unsigned long)s_control_copy.not_ready,
           (unsigned long)s_control_copy.source_invalid, (unsigned long)s_control_copy.pointer_invalid,
           (unsigned long)s_control_copy.lapped, (unsigned long)s_control_copy.timed_out,
           (unsigned long)s_control_copy.max_us, (unsigned long)s_control_copy.max_advance);
}

int ring_node_index(uint32_t addr)
{
    for (int k = 0; k < s_rx_node_count; ++k)
        if ((uintptr_t)s_rx_nodes[k].dscr == addr) return k;
    return -1;
}

static uint32_t rx_active_addr(void) { return AHB_DMA.channel[s_rx_dma_ch].in.in_dscr_bf0.val; }

static bool completed_plan(int active, size_t need, control_copy_plan_t *out)
{
    if (active < 0 || s_rx_node_count > MAX_RING_DESCRIPTORS) return false;
    size_t lengths[MAX_RING_DESCRIPTORS];
    for (int k = 0; k < s_rx_node_count; ++k) lengths[k] = s_rx_nodes[k].length;
    return control_copy_plan(lengths, (unsigned)s_rx_node_count, (unsigned)active, need, out);
}


/* The newest completed descriptor, copied and proven not lapped. */
bool copy_control_window(uint8_t *dst, size_t bytes)
{
    ++s_control_copy.calls;
    if (s_rx_dma_ch < 0 || s_rx_node_count < 3) { ++s_control_copy.not_ready; return false; }
    /* The control task (priority 3) was preempted mid-copy by the gain
     * observer (same priority, up to ~0.5 ms per window) and the sentinel
     * (4) often enough that 30-44 % of control windows failed the overrun
     * check on the board (T LADDER copy_fail, 2026-10-13). The copy itself
     * takes tens of microseconds: hold the scheduler for it. Interrupts
     * (GDMA, timers) still run. */
    vTaskSuspendAll();
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) {
        ++s_control_copy.pointer_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    control_copy_plan_t plan;
    if (!completed_plan(active, bytes, &plan)) {
        ++s_control_copy.source_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring ||
        src + bytes > s_raw_ring + sizeof(s_raw_ring)) {
        ++s_control_copy.source_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    sync_dma_m2c(src, bytes);
    memcpy(dst, src, bytes);
    int after = ring_node_index(rx_active_addr());
    const int64_t took = esp_timer_get_time() - start;
    (void)xTaskResumeAll();
    if ((uint64_t)took > s_control_copy.max_us) s_control_copy.max_us = (uint32_t)took;
    int advance = after < 0 ? s_rx_node_count : (after - active + s_rx_node_count) % s_rx_node_count;
    if (after < 0) { ++s_control_copy.pointer_invalid; return false; }
    if ((uint32_t)advance > s_control_copy.max_advance) s_control_copy.max_advance = (uint32_t)advance;
    if ((unsigned)advance >= plan.distance) { ++s_control_copy.lapped; return false; }
    if (!control_copy_current(&plan, (unsigned)s_rx_node_count, (unsigned)active, (unsigned)after,
                              (uint64_t)took, IQ_RATE_HZ)) { ++s_control_copy.timed_out; return false; }
    return true;
}

/* Four separated 64-byte regions of the newest completed descriptor. */
bool copy_probe_window(uint8_t dst[PROBE_BYTES], int *node)
{
    static const size_t offset[PROBE_REGIONS] = {512u, 1536u, 2560u, 4028u};
    if (s_rx_dma_ch < 0 || s_rx_node_count < 2) return false;
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) return false;
    control_copy_plan_t plan;
    if (!completed_plan(active, CONTROL_BYTES, &plan)) return false;
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring ||
        src + CONTROL_BYTES > s_raw_ring + sizeof(s_raw_ring)) return false;
    for (unsigned r = 0; r < PROBE_REGIONS; ++r) {
        sync_dma_m2c(src + offset[r], PROBE_REGION_BYTES);
        memcpy(dst + r * PROBE_REGION_BYTES, src + offset[r], PROBE_REGION_BYTES);
    }
    const uint64_t took = (uint64_t)(esp_timer_get_time() - start);
    const int after = ring_node_index(rx_active_addr());
    if (took > 300 || after < 0 || !control_copy_current(&plan, (unsigned)s_rx_node_count,
        (unsigned)active, (unsigned)after, took, IQ_RATE_HZ)) return false;
    if (node) *node = (int)plan.index;
    return true;
}

bool copy_sentinel_window(uint8_t dst[64])
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 3) return false;
    const int64_t start = esp_timer_get_time();
    const int active = ring_node_index(rx_active_addr());
    control_copy_plan_t plan;
    if (!completed_plan(active, 1984u + 64u, &plan)) return false;
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring || src + 2048u > s_raw_ring + sizeof(s_raw_ring)) return false;
    sync_dma_m2c(src + 1984u, 64u);
    memcpy(dst, src + 1984u, 64u);
    const int after = ring_node_index(rx_active_addr());
    return after >= 0 && control_copy_current(&plan, (unsigned)s_rx_node_count,
        (unsigned)active, (unsigned)after, (uint64_t)(esp_timer_get_time() - start), IQ_RATE_HZ);
}

/* 8190 contiguous bytes (204.75 us) ending where the active descriptor
 * began, valid only if RX could not have reached them during the copy. */
bool copy_snapshot(uint8_t *dst)
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 4) return false;
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) return false;
    uintptr_t base = (uintptr_t)s_raw_ring, first = (uintptr_t)s_rx_nodes[0].buffer;
    if (first < base || first - base >= sizeof(s_raw_ring)) return false;
    size_t total = 0;
    for (int k = 0; k < s_rx_node_count; ++k) {
        uintptr_t a = (uintptr_t)s_rx_nodes[k].buffer;
        if (a < base || !c5v4_snapshot_segment(sizeof(s_raw_ring), first - base, total, a - base,
                                               s_rx_nodes[k].length)) return false;
        total += s_rx_nodes[k].length;
    }
    if (total != sizeof(s_raw_ring)) return false;
    size_t end = (size_t)(s_rx_nodes[active].buffer - s_raw_ring);
    c5v4_snapshot_plan_t plan;
    if (!c5v4_snapshot_plan(total, end, s_rx_nodes[active].length, C5V4_LEVEL_SAMPLE_BYTES, &plan)) return false;
    sync_dma_m2c(s_raw_ring + plan.offset, plan.first);
    memcpy(dst, s_raw_ring + plan.offset, plan.first);
    size_t rest = C5V4_LEVEL_SAMPLE_BYTES - plan.first;
    if (rest) { sync_dma_m2c(s_raw_ring, rest); memcpy(dst + plan.first, s_raw_ring, rest); }
    uint64_t elapsed = (uint64_t)(esp_timer_get_time() - start);
    int later = ring_node_index(rx_active_addr());
    if (later < 0) return false;
    size_t after = (size_t)(s_rx_nodes[later].buffer - s_raw_ring);
    return c5v4_snapshot_current(total, C5V4_LEVEL_SAMPLE_BYTES, (after + total - end) % total,
                                 s_rx_nodes[later].length, elapsed, plan.safe_bytes, IQ_RATE_HZ);
}

/* Endpoint bytes (odd ring byte of each pair) of the newest `n` pairs. */
bool copy_recent_endpoints(uint8_t *dst, size_t n)
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 2 || 2u * n > sizeof(s_raw_ring) / 2u) return false;
    int idx = ring_node_index(rx_active_addr());
    if (idx < 0) return false;
    uint8_t *buf = s_rx_nodes[idx].buffer;
    if (buf < s_raw_ring || buf >= s_raw_ring + sizeof(s_raw_ring)) return false;
    size_t end = (size_t)(buf - s_raw_ring) & ~(size_t)1u;
    size_t pos = (end + sizeof(s_raw_ring) - 2u * n) % sizeof(s_raw_ring);
    sync_dma_m2c(s_raw_ring, sizeof(s_raw_ring));
    for (size_t k = 0; k < n; ++k) {
        dst[k] = s_raw_ring[pos + 1u];
        pos = (pos + 2u) % sizeof(s_raw_ring);
    }
    return true;
}

/* Sampling phase: holding the RX divider one count higher for ~1 us moves
 * the RX edge against the MODEM_DIAG update by a few 4.17 ns ticks
 * (zerowidth PR #3 does this on its sample clock). */
void rx_clock_slip(uint32_t us)
{
    phy_observation_changed();
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    uint32_t div = PCR.parl_clk_rx_conf.parl_clk_rx_div_num;
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_rx_conf, parl_clk_rx_div_num, div + 1u);
    esp_rom_delay_us(us);
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_rx_conf, parl_clk_rx_div_num, div);
    portEXIT_CRITICAL(&mux);
}

metrics_t measure_window(const uint8_t *s, size_t n)
{
    metrics_t m = {0};
    uint16_t hist[129] = {0};
    int clip = 0, origin = 0, coherent = 0, pi = 0, pq = 0;
    for (size_t k = 0; k < n; ++k) {
        int q = (int8_t)((s[k] & 0x0fu) << 4) >> 4, i = (int8_t)(s[k] & 0xf0u) >> 4;
        if (i == -8 || i == 7 || q == -8 || q == 7) ++clip;
        int p = i * i + q * q;
        if (p <= 4) ++origin;
        if (k) {
            int dot = i * pi + q * pq, cross = q * pi - i * pq;
            if (p >= 8 && dot > 0 && (cross < 0 ? -cross : cross) <= dot) ++coherent;
        }
        ++hist[p > 128 ? 128 : p];
        pi = i; pq = q;
    }
    unsigned cumulative = 0;
    for (unsigned p = 0; p <= 128u; ++p) {
        cumulative += hist[p];
        if (cumulative >= (n + 1u) / 2u) { m.p_median = (int)p; break; }
    }
    m.q_phase = n > 1u ? coherent * 100 / (int)(n - 1u) : 0;
    m.clip_pm = n ? clip * 1000 / (int)n : 0;
    m.origin_pm = n ? origin * 1000 / (int)n : 1000;
    return m;
}

/* Bench (console H, range audit PR #230): can the LUT be written while the
 * BitScrambler runs? The live writes of 2026-10-12 failed verification, but
 * they predate the LUT16 CPU addressing fix (kp_lut_io.h): never retested.
 * 1. the active values again, live: readback mismatches;
 * 2. another kp level live, read back; 3. the active values back, live.
 * The transport state (shadow, verified level) is not changed: a pure
 * measurement. Video may show the other kp level for ~1 ms. */
void transport_live_lut_test(void)
{
    if (s_demod_p64) { printf("LIVE_LUT refused: P64 fixed table; select U85Q with m\n"); return; }
    if (!s_lut_known || !s_started || s_menu_active || s_transport_fault || s_verified_level < 0) {
        printf("LIVE_LUT skip (not running, menu, fault or level unknown)\n");
        return;
    }
    const int other = s_verified_level + 1 < KP_LEVELS ? s_verified_level + 1 : s_verified_level - 1;
    static uint16_t want[KP_LUT_ENTRIES];
    uint32_t mism[3] = {0, 0, 0}, us[3] = {0, 0, 0}, differ = 0;
    for (unsigned i = 0; i < KP_LUT_ENTRIES; ++i) {
        bool clipped;
        want[i] = s_flag_format ? sync_flag_entry(kp_tables[other][i], s_output, &clipped)
                                : output_entry(kp_tables[other][i], s_output, &clipped);
        differ += want[i] != s_lut_live[i];
    }
    const uint16_t *pass[3] = {s_lut_live, want, s_lut_live};
    for (int p = 0; p < 3; ++p) {
        const int64_t t0 = esp_timer_get_time();
        for (unsigned i = 0; i < KP_LUT_ENTRIES; ++i)
            if (!write_lut_entry(NULL, i, pass[p][i])) ++mism[p];
        us[p] = (uint32_t)(esp_timer_get_time() - t0);
    }
    /* A last full readback of the restored values. */
    uint32_t final_mism = 0;
    for (unsigned i = 0; i < KP_LUT_ENTRIES; ++i) {
        select_lut_entry(NULL, i);
        if ((read_lut_entry(NULL) & 0xffffu) != s_lut_live[i]) ++final_mism;
    }
    printf("LIVE_LUT level=%d other=%d differ=%lu same_live: mismatch=%lu us=%lu | other_live: mismatch=%lu us=%lu | "
           "restore_live: mismatch=%lu us=%lu | final_readback_mismatch=%lu\n", s_verified_level, other,
           (unsigned long)differ, (unsigned long)mism[0], (unsigned long)us[0], (unsigned long)mism[1],
           (unsigned long)us[1], (unsigned long)mism[2], (unsigned long)us[2], (unsigned long)final_mism);
}
