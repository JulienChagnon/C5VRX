/* C5VRX-5 by Twotoz and the C5VRX contributors: on-screen menu.
 * A standalone PAL/NTSC raster (menu_raster.c) owns TX while the menu is
 * open; live video resumes with the exact boot topology on exit. The GDMA
 * scatter chain is allocated in 384-byte chunks only while the menu is open
 * (NTSC ~18 KiB, PAL ~22 KiB of internal SRAM). */
#include "receiver.h"
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "rf.h"
#include "dac_check.h"

void transport_start_chain(dma_descriptor_t *first);
void transport_stop_chain(void);

/* 32 x 12 B = 384 B, word aligned. With live tasks running the internal
 * heap has >20 KiB free but in pieces: 64-byte-aligned 1.5-KiB and 768-B
 * chunks failed after ~14 KiB (board 2026-10-18, NO_MEM at 7-8 KiB free).
 * NTSC needs 1499 nodes, PAL 1811. Internal SRAM is not cached on the C5
 * and sync_dma_c2m rounds itself, so cache-line alignment buys nothing. */
#define NODE_CHUNK  32u
#define NODE_CHUNKS ((MENU_MAX_NODES + NODE_CHUNK - 1u) / NODE_CHUNK)

enum {
    UI_ROOT = 22, UI_HEADER = 24, UI_PANEL = 26, UI_PANEL_2 = 29, UI_DIVIDER = 33,
    UI_MUTED = 39, UI_SELECTED = 43, UI_SELECTED_EDGE = 48, UI_STRONG = 54, UI_WHITE = 60,
};
#include "menu_font.h"

static DMA_ATTR __attribute__((aligned(64))) menu_raster_t s_raster;
static dma_descriptor_t *s_chunks[NODE_CHUNKS];
static unsigned s_chunk_count, s_node_capacity, s_node_count;
volatile int s_menu_cursor;
/* Status screen: a full-screen message on the menu raster (calibration). */
static volatile bool s_status;
static char s_status_title[40], s_status_line1[48], s_status_line2[48];
static volatile int s_status_percent;
/* DAC TEST: the current step, and whether the whole raster is a flat
 * meter level (sync and burst overwritten) instead of the menu picture. */
static dac_step_t s_dac_step = DAC_STEP_PATTERN;
static bool s_raster_flat;
static uint8_t s_flat_code;

static const uint8_t s_icons[MENU_ITEMS][8] = {
    {0x10, 0x38, 0x54, 0x10, 0x10, 0x38, 0x7c, 0x00},   /* band */
    {0x7e, 0x42, 0x5a, 0x5a, 0x5a, 0x42, 0x7e, 0x00},   /* channel */
    {0x00, 0x40, 0x50, 0x54, 0x55, 0x55, 0x55, 0x00},   /* scan */
    {0x7e, 0x42, 0x42, 0x42, 0x7e, 0x18, 0x3c, 0x00},   /* video */
    {0x08, 0x08, 0x1c, 0x7f, 0x1c, 0x08, 0x08, 0x00},   /* calibrate */
    {0x00, 0x08, 0x18, 0x38, 0x78, 0x38, 0x18, 0x08},   /* output */
    {0x00, 0x02, 0x06, 0x0e, 0x1e, 0x3e, 0x7e, 0x00},   /* gain */
    {0x00, 0x03, 0x03, 0x0f, 0x0f, 0x3f, 0x3f, 0x00},   /* dac test */
    {0x7c, 0x44, 0x04, 0x1f, 0x04, 0x44, 0x7c, 0x00},   /* exit */
};
static const char *const s_items[MENU_ITEMS] = {"BAND", "CHANNEL", "SCAN", "VIDEO", "CALIBRATE", "OUTPUT", "GAIN", "DAC TEST",
                                                "EXIT"};

static void rect(int x, int y, int w, int h, uint8_t code)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > (int)MENU_UI_WIDTH ? (int)MENU_UI_WIDTH : x + w;
    int y1 = y + h > (int)MENU_UI_LINES ? (int)MENU_UI_LINES : y + h;
    if (x1 <= x0 || y1 <= y0) return;
    unsigned sx0 = (unsigned)x0 * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN;
    unsigned sx1 = (unsigned)x1 * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN;
    for (int yy = y0; yy < y1; ++yy) memset(&s_raster.ui[yy][sx0], code, sx1 - sx0);
}

static void glyph(char ch, int x, int y, uint8_t code, int scale)
{
    unsigned index = (ch >= 32 && ch <= 126) ? (unsigned)ch - 32u : 0u;
    for (int gy = 0; gy < 8; ++gy)
        for (int gx = 0; gx < 8; ++gx)
            if (s_font8x8[index][gy] & (0x80u >> gx)) rect(x + gx * scale, y + gy * scale, scale, scale, code);
}

static void text(const char *s, int x, int y, uint8_t code, int scale)
{
    for (; s && *s && x < (int)MENU_UI_WIDTH; ++s, x += 8 * scale) glyph(*s, x, y, code, scale);
}

static void text_right(const char *s, int right, int y, uint8_t code)
{
    text(s, right - 8 * (int)strlen(s), y, code, 1);
}

static void value_box(int x, int y, int w, const char *label, const char *value)
{
    rect(x, y, w, 10, UI_PANEL_2);
    rect(x, y, w, 1, UI_DIVIDER);
    text(label, x + 3, y + 1, UI_MUTED, 1);
    text_right(value, x + w - 3, y + 1, UI_WHITE);
}

static void meter(int x, int y, int w, int value)
{
    value = value < 0 ? 0 : value > 100 ? 100 : value;
    rect(x, y, w, 5, UI_ROOT);
    rect(x + 1, y + 1, w - 2, 3, UI_PANEL_2);
    if (value) rect(x + 1, y + 1, (w - 2) * value / 100, 3, UI_STRONG);
}

static const char *std_name(video_standard_t s) { return s == VIDEO_STD_PAL ? "PAL" : "NTSC"; }

static void title(const char *t, const char *hint)
{
    text(t, 100, 10, UI_WHITE, 1);
    if (hint) text_right(hint, 376, 10, UI_MUTED);
    rect(100, 19, 276, 1, UI_DIVIDER);
}

static bool s_interlaced = true;

/* Back from a flat meter level to the menu picture: sync, burst and blank
 * segments are rebuilt in place; the descriptor chain is unchanged. */
static void raster_restore(void)
{
    if (!s_raster_flat) return;
    menu_raster_init(&s_raster, s_video_std, s_interlaced);
    s_raster_flat = false;
    sync_dma_c2m(&s_raster, sizeof(s_raster));
}

/* Every byte the chain sends becomes one DAC code: a constant node voltage
 * for a multimeter. The display loses sync meanwhile. */
static void raster_flat(uint8_t code)
{
    if (s_raster_flat && s_flat_code == code) return;
    memset(&s_raster, code, sizeof(s_raster));
    s_raster_flat = true;
    s_flat_code = code;
    sync_dma_c2m(&s_raster, sizeof(s_raster));
}

void menu_dac_step(void)
{
    s_dac_step = dac_check_next(s_dac_step);
    const uint8_t code = dac_check_code(s_dac_step);
    if (s_dac_step == DAC_STEP_PATTERN) {
        printf("DACTEST pattern: a smooth ramp, then six bars whose contrast doubles from D4 to D9\n");
    } else {
        if (s_dac_step == DAC_STEP_BIT0) {
            printf("DACTEST meter steps: unplug the display, measure CVBS to GND; expected mV at 3.30 V:");
            for (unsigned k = 0; k < DAC_CHECK_BITS; ++k)
                printf(" %s=%d", dac_check_pins[k], dac_check_expected_mv((uint8_t)(1u << k), DAC_CHECK_VDD_MV));
            printf(" ALL=%d NONE=0 (scale by your 3V3 pin / 3.30)\n", dac_check_expected_mv(63, DAC_CHECK_VDD_MV));
        }
        printf("DACTEST step=%s code=%u expect_mv=%d (record it for tools/dac_check.py)\n",
               dac_check_step_name(s_dac_step), code, dac_check_expected_mv(code, DAC_CHECK_VDD_MV));
    }
    if (s_menu_active) menu_render();
}

/* DAC TEST page, in the content pane like every other item: a ramp over
 * every level from blanking to the top code, then one bar per bit whose
 * halves differ in that bit alone. */
static void render_dac_pattern(void)
{
    /* Ramp, written per sample: 44 levels of ~26 samples (0.65 us). */
    const unsigned x0 = 100u * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN;
    const unsigned x1 = 376u * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN;
    for (unsigned y = 21; y < 33; ++y)
        for (unsigned x = x0; x < x1; ++x) s_raster.ui[y][x] = dac_check_ramp_code(x - x0, x1 - x0);
    for (unsigned k = 0; k < DAC_CHECK_BITS; ++k) {
        const int x = 100 + (int)k * 46;
        rect(x, 36, 21, 10, dac_check_bar_base(k));
        rect(x + 21, 36, 21, 10, dac_check_bar_top(k));
        text(dac_check_pins[k], x + 13, 48, UI_WHITE, 1);
    }
}

static void render_status(void)
{
    raster_restore();
    memset(s_raster.ui, UI_ROOT, sizeof(s_raster.ui));
    rect(0, 0, MENU_UI_WIDTH, 8, UI_HEADER);
    text("C5VRX-5", 6, 0, UI_WHITE, 1);
    text(rf_channel()->name, 96, 0, UI_MUTED, 1);
    text(s_status_title, (MENU_UI_WIDTH - 16 * (int)strlen(s_status_title)) / 2, 12, UI_WHITE, 2);
    text(s_status_line1, (MENU_UI_WIDTH - 8 * (int)strlen(s_status_line1)) / 2, 31, UI_WHITE, 1);
    text(s_status_line2, (MENU_UI_WIDTH - 8 * (int)strlen(s_status_line2)) / 2, 40, UI_MUTED, 1);
    if (s_status_percent >= 0) meter(40, 50, MENU_UI_WIDTH - 80, s_status_percent);
    sync_dma_c2m(s_raster.ui, sizeof(s_raster.ui));
}

bool menu_status_active(void) { return s_status; }

/* Show (or update) a full-screen status message; percent < 0 hides the bar.
 * The menu raster owns TX meanwhile, like the menu itself. */
void status_screen(const char *title, const char *line1, const char *line2, int percent)
{
    snprintf(s_status_title, sizeof(s_status_title), "%s", title);
    snprintf(s_status_line1, sizeof(s_status_line1), "%s", line1);
    snprintf(s_status_line2, sizeof(s_status_line2), "%s", line2);
    s_status_percent = percent;
    if (!s_status) {
        s_status = true;
        if (!s_menu_active && !menu_open()) {
            s_status = false;
            printf("STATUS screen refused (menu raster could not take TX): %s\n", title);
            return;
        }
    }
    render_status();
}

void status_screen_end(void)
{
    if (!s_status) return;
    s_status = false;
    menu_close();
}

void menu_render(void)
{
    char buf[32];
    const fpv_channel_t *ch = rf_channel();
    if (s_status) { render_status(); return; }
    if (s_menu_cursor != MENU_DAC) s_dac_step = DAC_STEP_PATTERN;   /* leaving the item ends the meter */
    if (s_dac_step != DAC_STEP_PATTERN) { raster_flat(dac_check_code(s_dac_step)); return; }
    raster_restore();
    memset(s_raster.ui, UI_ROOT, sizeof(s_raster.ui));
    rect(0, 0, MENU_UI_WIDTH, MENU_UI_LINES, UI_PANEL);
    /* Header: name, channel, frequency, gain, signal, standard. */
    rect(0, 0, MENU_UI_WIDTH, 8, UI_HEADER);
    text("C5VRX-5", 6, 0, UI_WHITE, 1);
    text(ch->name, 96, 0, UI_WHITE, 1);
    snprintf(buf, sizeof(buf), "%uM", ch->freq_mhz);
    text(buf, 128, 0, UI_MUTED, 1);
    snprintf(buf, sizeof(buf), "G%u", s_current_gain);
    text(buf, 208, 0, UI_WHITE, 1);
    /* Range warnings take the place of the link state: they cost range
     * without any sign in the picture. */
    char dc_need[24];
    const bool dc_cal = calibration_dc_needed(dc_need, sizeof(dc_need)) != 0;
    const bool gain_fixed = receiver_gain_mode() != GAIN_MODE_AUTO;
    if (dc_cal || gain_fixed) text(dc_cal ? "DC CAL!" : "GAIN!", 248, 0, UI_WHITE, 1);
    else text(s_afc_locked ? "VIDEO" : "SEARCH", 248, 0, UI_MUTED, 1);
    int bars = s_signal_strength <= 0 ? 0 : s_signal_strength >= 100 ? 5 : (s_signal_strength + 19) / 20;
    for (int i = 0; i < 5; ++i) rect(320 + i * 3, 7 - (2 + i), 2, 2 + i, i < bars ? UI_WHITE : UI_DIVIDER);
    text(std_name(s_video_std), 344, 0, UI_WHITE, 1);
    /* Navigation. */
    rect(0, 8, 92, MENU_UI_LINES - 8, UI_ROOT);
    rect(91, 8, 1, MENU_UI_LINES - 8, UI_DIVIDER);
    /* 56 lines: an 8-line header leaves room for six 8-line items, so the
     * list scrolls with the cursor (seven items put EXIT below the screen). */
    const int NAV_ROWS = ((int)MENU_UI_LINES - 8) / 8;
    int first = s_menu_cursor - (NAV_ROWS - 1);
    if (first < 0) first = 0;
    if (first > MENU_ITEMS - NAV_ROWS) first = MENU_ITEMS - NAV_ROWS > 0 ? MENU_ITEMS - NAV_ROWS : 0;
    for (int i = first; i < MENU_ITEMS && i < first + NAV_ROWS; ++i) {
        int y = 8 + (i - first) * 8;
        bool on = i == s_menu_cursor;
        if (on) { rect(0, y, 91, 8, UI_SELECTED); rect(0, y, 3, 8, UI_WHITE); rect(3, y, 88, 1, UI_SELECTED_EDGE); }
        uint8_t ink = on ? UI_WHITE : UI_MUTED;
        for (int gy = 0; gy < 8; ++gy)
            for (int gx = 0; gx < 8; ++gx)
                if (s_icons[i][gy] & (0x80u >> gx)) rect(5 + gx, y + gy, 1, 1, ink);
        text(s_items[i], 18, y, ink, 1);
    }
    if ((int)MENU_ITEMS > NAV_ROWS) {   /* 1-pixel scroll thumb next to the divider: part of the list shown */
        const int track = (int)MENU_UI_LINES - 8;
        rect(90, 8 + first * track / MENU_ITEMS, 1, NAV_ROWS * track / MENU_ITEMS, UI_STRONG);
    }
    switch (s_menu_cursor) {
    case MENU_BAND:
        title("BAND", "HOLD: NEXT BAND");
        value_box(100, 22, 276, "ACTIVE BAND", rf_band_name((unsigned)(rf_channel_index() / 8u)));
        value_box(100, 34, 276, "CHANNEL", ch->name);
        text("PRESS: NEXT ITEM", 100, 47, UI_MUTED, 1);
        break;
    case MENU_CHANNEL:
        title("CHANNEL", "HOLD: NEXT CHANNEL");
        rect(100, 22, 108, 23, UI_PANEL_2);
        rect(100, 22, 1, 23, UI_WHITE);
        text("ACTIVE", 108, 23, UI_MUTED, 1);
        text(ch->name, 108, 29, UI_WHITE, 2);
        rect(216, 22, 160, 23, UI_ROOT);
        text("CENTER MHZ", 224, 23, UI_MUTED, 1);
        snprintf(buf, sizeof(buf), "%u", ch->freq_mhz);
        text(buf, 224, 29, UI_WHITE, 2);
        text("SIGNAL", 100, 47, UI_MUTED, 1);
        meter(156, 48, 120, s_signal_strength);
        {   /* the level at the chip pin (rx_level.h) */
            const rx_level_t lv = receiver_level();
            if (lv.valid && lv.above_noise) snprintf(buf, sizeof(buf), "%d DBM", (lv.carrier_dbm_x10 - 5) / 10);
            else snprintf(buf, sizeof(buf), lv.valid ? "NOISE" : "--");
        }
        text_right(buf, 376, 47, UI_WHITE);
        break;
    case MENU_SCAN:
        title("SCAN", s_scan_active ? "SCANNING" : "HOLD: SCAN 48 CHANNELS");
        text("FINDS THE STRONGEST VIDEO", 100, 24, UI_WHITE, 1);
        text("CENTRED ON A CHANNEL", 100, 33, UI_MUTED, 1);
        meter(100, 47, 216, s_scan_active ? (int)s_scan_progress : 0);
        snprintf(buf, sizeof(buf), "%u%%", s_scan_active ? s_scan_progress : 0u);
        text_right(buf, 376, 45, UI_WHITE);
        break;
    case MENU_VIDEO:
        title("VIDEO STANDARD", "HOLD: CHANGE");
        value_box(100, 22, 276, "STANDARD", s_std_mode == STD_MODE_AUTO ? "AUTO" :
                                            s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC");
        value_box(100, 34, 276, "DETECTED", s_detected_std_valid ? std_name(s_detected_std) : "NO VIDEO");
        text("AUTO FOLLOWS THE CAMERA", 100, 47, UI_MUTED, 1);
        break;
    case MENU_OUTPUT:
        title("OUTPUT LEVEL", "HOLD: CHANGE");
        value_box(100, 22, 276, "LEVEL", transport_output_name());
        text("SAVED ON EXIT   LOAD: 75 OHM", 100, 38, UI_MUTED, 1);
        text(transport_output() == 9 ? "LOW SWING REDUCES CLIPPING" :
             (transport_output() ? "HIGH LEVEL MAY CLIP VIDEO" : "INCREASE FOR WEAK SYNC"), 100, 47, UI_WHITE, 1);
        break;
    case MENU_GAIN: {
        char mode[16];
        title("RX GAIN", "HOLD: CHANGE");
        value_box(100, 22, 276, "MODE", receiver_gain_mode_name(receiver_gain_mode(), mode, sizeof(mode)));
        snprintf(buf, sizeof(buf), "G%u", s_current_gain);
        value_box(100, 34, 276, "ACTIVE GAIN", buf);
        text(receiver_gain_mode() == GAIN_MODE_AUTO ? "AUTO FOLLOWS THE SIGNAL (KEEL-1)" : "FIXED: COSTS RANGE, USE AUTO",
             100, 47, UI_WHITE, 1);
        break;
    }
    case MENU_DAC:
        title("DAC TEST", "HOLD: METER STEPS");
        render_dac_pattern();
        break;
    case MENU_CALIBRATE:
        title("CALIBRATE", "HOLD: CALIBRATE NOW");
        if (dc_cal) {
            char line[48];
            snprintf(line, sizeof(line), "DC CAL NEEDED: %s", dc_need);
            text(line, 100, 24, UI_WHITE, 1);
            text("REPAIRED AT NEXT BOOT, OR HOLD", 100, 33, UI_MUTED, 1);
        } else {
            text("ON THE GROUND ONLY: VIDEO STOPS", 100, 24, UI_WHITE, 1);
            text("FOR A FEW MIN. VTX OFF IS BEST", 100, 33, UI_MUTED, 1);
        }
        text("NEVER RUNS BY ITSELF IN FLIGHT", 100, 47, UI_MUTED, 1);
        break;
    default:
        title("EXIT", NULL);
        rect(100, 23, 276, 20, UI_PANEL_2);
        rect(100, 23, 1, 20, UI_WHITE);
        text("BACK TO LIVE VIDEO", 116, 25, UI_WHITE, 1);
        text("HOLD TO EXIT", 116, 34, UI_MUTED, 1);
        text("CLOSES BY ITSELF AFTER 30 S", 100, 47, UI_MUTED, 1);
        break;
    }
    sync_dma_c2m(s_raster.ui, sizeof(s_raster.ui));
}

static dma_descriptor_t *node(unsigned index) { return &s_chunks[index / NODE_CHUNK][index % NODE_CHUNK]; }

static bool count_segment(void *ctx, const uint8_t *data, unsigned length)
{
    unsigned *count = ctx;
    if (*count >= MENU_MAX_NODES || length > 4092 || ((uintptr_t)data & 3) || (length & 3)) return false;
    ++*count;
    return true;
}

static bool append_segment(void *ctx, const uint8_t *data, unsigned length)
{
    (void)ctx;
    if (s_node_count >= s_node_capacity || length > 4092 || ((uintptr_t)data & 3) || (length & 3)) return false;
    dma_descriptor_t *d = node(s_node_count++);
    memset(d, 0, sizeof(*d));
    d->dw0.size = length;
    d->dw0.length = length;
    d->dw0.owner = 1;
    d->buffer = (void *)data;
    return true;
}

static void free_nodes(void)
{
    for (unsigned k = 0; k < s_chunk_count; ++k) { heap_caps_free(s_chunks[k]); s_chunks[k] = NULL; }
    s_chunk_count = s_node_capacity = s_node_count = 0;
}

/* Count the raster, reserve exactly that (grow only), link a cycle.
 * Interlaced first; the progressive field only when its chain cannot be
 * reserved (half the descriptors, but not every display keeps it). */
static esp_err_t reserve_chain(video_standard_t standard, bool interlaced, unsigned *required)
{
    menu_raster_init(&s_raster, standard, interlaced);
    s_raster_flat = false;
    *required = 0;
    if (!menu_raster_emit(&s_raster, standard, interlaced, count_segment, required) || !*required)
        return ESP_ERR_INVALID_SIZE;
    unsigned chunks = (*required + NODE_CHUNK - 1u) / NODE_CHUNK;
    while (s_chunk_count < chunks) {
        dma_descriptor_t *c = heap_caps_aligned_alloc(4u, NODE_CHUNK * sizeof(dma_descriptor_t),
                                                      MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL);
        if (!c) return ESP_ERR_NO_MEM;
        s_chunks[s_chunk_count++] = c;
        s_node_capacity += NODE_CHUNK;
    }
    return ESP_OK;
}

static esp_err_t build_chain(video_standard_t standard)
{
    unsigned required = 0;
    bool interlaced = true;
    esp_err_t err = reserve_chain(standard, true, &required);
    if (err == ESP_ERR_NO_MEM) {
        interlaced = false;
        err = reserve_chain(standard, false, &required);
        if (err == ESP_OK) printf("[MENU] progressive fallback: interlaced chain did not fit\n");
    }
    if (err != ESP_OK) return err;
    s_interlaced = interlaced;
    s_node_count = 0;
    if (!menu_raster_emit(&s_raster, standard, interlaced, append_segment, NULL) || s_node_count != required)
        return ESP_ERR_INVALID_SIZE;
    for (unsigned k = 0; k < s_node_count; ++k) node(k)->next = node((k + 1u) % s_node_count);
    s_video_std = standard;
    menu_render();
    sync_dma_c2m(&s_raster, sizeof(s_raster));
    for (unsigned k = 0; k < s_chunk_count; ++k) sync_dma_c2m(s_chunks[k], NODE_CHUNK * sizeof(dma_descriptor_t));
    return ESP_OK;
}

/* A short flat DAC level without sync between live video and the menu
 * raster (both ways): the goggle drops its lock on the camera timing and
 * locks fresh, instead of showing the menu briefly, inverting and going
 * black (operator, 2026-10-18). Not in flight: only menu/status entry. */
#define MENU_RELOCK_GAP_MS 150u

static void entry_start(void)
{
    s_menu_active = true;
    transport_stop_tx();
    vTaskDelay(pdMS_TO_TICKS(MENU_RELOCK_GAP_MS));
    transport_start_chain(node(0));
}

bool menu_open(void)
{
    if (s_menu_active) return true;
    /* Allocate and render before touching live video: if memory is short,
     * flight video keeps running. */
    const esp_err_t err = build_chain(menu_standard());
    if (err != ESP_OK) {
        ESP_LOGE("menu", "unavailable: %s (internal free=%u dma_free=%u dma_largest=%u chunks=%u)",
                 esp_err_to_name(err), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL),
                 s_chunk_count);
        free_nodes();
        return false;
    }
    entry_start();
    return true;
}

void menu_close(void)
{
    if (!s_menu_active) return;
    transport_stop_chain();
    vTaskDelay(pdMS_TO_TICKS(MENU_RELOCK_GAP_MS));
    transport_resume_live();
    free_nodes();       /* GDMA no longer references the chain */
    s_menu_active = false;
    s_dac_step = DAC_STEP_PATTERN;
    ++s_control_epoch;
}

void menu_set_standard_mode(std_mode_t mode)
{
    s_std_mode = mode;
    if (!s_menu_active) return;
    video_standard_t want = menu_standard();
    if (want == s_video_std) { menu_render(); return; }
    video_standard_t previous = s_video_std;
    transport_stop_chain();
    /* PAL needs ~312 more descriptors; without them keep the old standard. */
    if (build_chain(want) != ESP_OK) ESP_ERROR_CHECK(build_chain(previous));
    transport_start_chain(node(0));
}

