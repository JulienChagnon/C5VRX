/* C5VRX-5 by Twotoz and the C5VRX contributors: menu picture (menu_ui.h).
 *
 * Layout on the 384 x 56 grid: one settings list, all nine rows visible at
 * once (a row of text plus a row of air each), label left and value right,
 * the selected row on a white pill. A live card on the right keeps the
 * frequency, signal and level in view, and says in one word what a long
 * press does. One button: press = next row, hold = change it. */
#include "menu_ui.h"
#include <stdio.h>
#include <string.h>
#include "dac_check.h"
#include "menu_font.h"
#include "menu_logo.h"

unsigned menu_ui_overflow;

static uint8_t (*s_ui)[MENU_UI_BYTES];

#define W ((int)MENU_UI_WIDTH)
#define H ((int)MENU_UI_LINES)
#define TEXT_H ((int)MENU_TEXT_ROWS)
#define BIG_H ((int)MENU_DISPLAY_ROWS)

/* Settings list and live card. */
#define LIST_W 232
#define LIST_PITCH 6
#define CARD_X 248
#define CARD_R (W - 4)
#define CARD_W (CARD_R - CARD_X)

static unsigned sample(int x) { return (unsigned)x * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN; }

static void rect(int x, int y, int w, int h, uint8_t code)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > W ? W : x + w, y1 = y + h > H ? H : y + h;
    if (x1 <= x0 || y1 <= y0) return;
    const unsigned s0 = sample(x0), s1 = sample(x1);
    for (int yy = y0; yy < y1; ++yy) memset(&s_ui[yy][s0], code, s1 - s0);
}

/* Rounded box: the outer rows are pulled in, which on this grid reads as
 * a radius of about one row. */
static void pill(int x, int y, int w, int h, uint8_t code)
{
    if (h < 3) { rect(x, y, w, h, code); return; }
    const int r = h >= 7 ? 4 : 3, step = h >= 7 ? 1 : 0;   /* taller pills: a second, softer step */
    rect(x + r, y, w - 2 * r, 1, code);
    rect(x + r, y + h - 1, w - 2 * r, 1, code);
    if (step) { rect(x + 1, y + 1, w - 2, 1, code); rect(x + 1, y + h - 2, w - 2, 1, code); }
    rect(x, y + 1 + step, w, h - 2 - 2 * step, code);
}

/* ---- text ---- */

typedef enum { FONT_TEXT, FONT_BIG } font_t;
typedef enum { ALIGN_LEFT, ALIGN_CENTER, ALIGN_RIGHT } align_t;

static unsigned glyph_index(char c)
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    return (c >= 32 && c <= 95) ? (unsigned)(c - 32) : (unsigned)('?' - 32);
}

static int glyph_width(font_t f, char c)
{
    const unsigned i = glyph_index(c);
    return f == FONT_BIG ? s_menu_display_font[i].width : s_menu_text_font[i].width;
}

static int default_track(font_t f) { return f == FONT_BIG ? 5 : 2; }

static int text_width_track(const char *s, font_t f, int track)
{
    int w = 0;
    for (const char *p = s; p && *p; ++p) w += glyph_width(f, *p) + (p[1] ? track : 0);
    return w;
}

static int text_width(const char *s, font_t f) { return text_width_track(s, f, default_track(f)); }

/* One row mask as runs, clipped at clip_r. */
static void mask_row(uint32_t bits, int x, int y, int clip_r, uint8_t code)
{
    for (int b = 0; bits; ) {
        if (!(bits & 1u)) { bits >>= 1; ++b; continue; }
        int start = b;
        while (bits & 1u) { bits >>= 1; ++b; }
        int x0 = x + start, x1 = x + b;
        if (x1 > clip_r) x1 = clip_r;
        if (x1 > x0) rect(x0, y, x1 - x0, 1, code);
    }
}

static void glyph5(const menu_glyph5_t *g, int x, int y, int clip_r, uint8_t code)
{
    for (int r = 0; r < TEXT_H; ++r) mask_row(g->rows[r], x, y + r, clip_r, code);
}

static int draw(const char *s, int x, int y, font_t f, int track, int clip_r, uint8_t code)
{
    for (const char *p = s; p && *p && x < clip_r; ++p) {
        const unsigned i = glyph_index(*p);
        if (f == FONT_BIG)
            for (int r = 0; r < BIG_H; ++r) mask_row(s_menu_display_font[i].rows[r], x, y + r, clip_r, code);
        else
            glyph5(&s_menu_text_font[i], x, y, clip_r, code);
        x += glyph_width(f, *p) + track;
    }
    return x - track;
}

/* Text in a box [x, x + w): tracking tightens before anything is clipped;
 * a clip counts as an overflow. Returns the right edge of the ink. */
static int text_box(const char *s, int x, int y, int w, align_t a, font_t f, uint8_t code)
{
    int track = default_track(f), tw = text_width_track(s, f, track);
    while (tw > w && track > 0) tw = text_width_track(s, f, --track);
    if (tw > w) { ++menu_ui_overflow; tw = w; }
    const int left = a == ALIGN_LEFT ? x : a == ALIGN_CENTER ? x + (w - tw) / 2 : x + w - tw;
    return draw(s, left, y, f, track, x + w, code);
}

static int text(const char *s, int x, int y, uint8_t code)
{
    return text_box(s, x, y, W - x, ALIGN_LEFT, FONT_TEXT, code);
}

/* ---- components ---- */

/* A horizontal meter: grey track, white fill, two rows. */
static void meter(int x, int y, int w, int percent)
{
    percent = percent < 0 ? 0 : percent > 100 ? 100 : percent;
    rect(x, y, w, 2, MENU_INK_TRACK);
    if (percent) rect(x, y, w * percent / 100, 2, MENU_INK_WHITE);
}

/* Signal bars: five, one row taller each, bottom aligned at y + 4. */
static void signal_bars(int x, int y, int signal)
{
    const int bars = signal <= 0 ? 0 : signal >= 100 ? 5 : (signal + 19) / 20;
    for (int i = 0; i < 5; ++i) rect(x + i * 5, y + 4 - i, 3, i + 1, i < bars ? MENU_INK_WHITE : MENU_INK_TRACK);
}

/* Small inverted badge (warnings, HOLD key). Returns its right edge. */
static int badge(const char *s, int x, int y, uint8_t fill, uint8_t ink)
{
    const int w = text_width(s, FONT_TEXT) + 10;
    pill(x, y, w, 7, fill);
    text_box(s, x + 5, y + 1, w - 10, ALIGN_LEFT, FONT_TEXT, ink);
    return x + w;
}

/* ---- settings list ---- */

static const char *const s_items[MENU_ITEMS] = {
    "BAND", "CHANNEL", "SCAN", "VIDEO", "CALIBRATE", "OUTPUT", "GAIN", "DAC TEST", "EXIT",
};

/* The value a row shows: main text plus an optional grey note (the live
 * state behind an AUTO setting). Actions show none. */
static void row_value(const menu_view_t *v, int item, const char **value, const char **note)
{
    static char band[2];
    *value = *note = NULL;
    switch (item) {
    case MENU_BAND:
        band[0] = v->band < v->band_count ? v->band_letters[v->band] : '?';
        band[1] = 0;
        *value = band;
        break;
    case MENU_CHANNEL: *value = v->channel; break;
    case MENU_VIDEO:
        *value = v->std_mode == 1 ? "NTSC" : v->std_mode == 2 ? "PAL" : "AUTO";
        if (v->std_mode == 0) *note = v->detected_valid ? (v->detected_pal ? "PAL" : "NTSC") : "--";
        break;
    case MENU_CALIBRATE: if (v->dc_cal) *value = "DC!"; break;
    case MENU_OUTPUT: *value = v->output_name; break;
    case MENU_GAIN: *value = v->gain_mode; break;
    default: break;
    }
}

static void list(const menu_view_t *v)
{
    for (int i = 0; i < MENU_ITEMS; ++i) {
        const int y = 1 + i * LIST_PITCH;
        const bool on = i == v->cursor;
        if (on) pill(0, y - 1, LIST_W, 7, MENU_INK_WHITE);
        const char *value, *note;
        row_value(v, i, &value, &note);
        int right = LIST_W - 8;
        if (value) right = text_box(value, 0, y, right, ALIGN_RIGHT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_WHITE);
        if (note) {
            const int nw = text_width(note, FONT_TEXT), vw = text_width(value, FONT_TEXT);
            text_box(note, right - vw - 8 - nw, y, nw, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
        }
        text_box(s_items[i], 9, y, 100, ALIGN_LEFT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_GREY);
    }
}

/* ---- live card ---- */

/* Link state (or a range warning in its place: both cost range without
 * any sign in the picture) beside the signal bars. */
static void card_status(const menu_view_t *v)
{
    signal_bars(CARD_X, 1, v->signal);
    if (v->dc_cal || v->gain_fixed) badge(v->dc_cal ? "DC CAL!" : "GAIN!", CARD_X + 31, 0, MENU_INK_WHITE, MENU_INK_BLACK);
    else text(v->video ? "VIDEO" : "SEARCH", CARD_X + 32, 1, v->video ? MENU_INK_WHITE : MENU_INK_GREY);
}

static void card_tuning(const menu_view_t *v)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", v->freq_mhz);
    text_box(buf, CARD_X, 12, CARD_W, ALIGN_LEFT, FONT_BIG, MENU_INK_WHITE);
    int x = text(v->channel, CARD_X, 26, MENU_INK_WHITE) + 8;
    char band[16];   /* "RACEBAND R" -> "RACEBAND": the letter is in the channel */
    snprintf(band, sizeof(band), "%s", v->band_name ? v->band_name : "");
    char *space = strchr(band, ' ');
    if (space) *space = 0;
    text_box(band, x, 26, CARD_R - x, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    if (v->level_valid && v->level_above_noise) snprintf(buf, sizeof(buf), "%d DBM", v->level_dbm);
    else snprintf(buf, sizeof(buf), v->level_valid ? "NOISE" : "NO LEVEL");
    meter(CARD_X, 35, CARD_W, v->signal);
    text_box(buf, CARD_X, 39, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
}

static void card_scan(const menu_view_t *v)
{
    char buf[8];   /* the number in the display font, its % in text size */
    snprintf(buf, sizeof(buf), "%u", v->scan_progress > 100u ? 100u : v->scan_progress);
    const int x = text_box(buf, CARD_X, 12, CARD_W - 16, ALIGN_LEFT, FONT_BIG, MENU_INK_WHITE);
    text("%", x + 4, 18, MENU_INK_GREY);
    text_box("SCANNING", CARD_X, 26, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    meter(CARD_X, 35, CARD_W, (int)v->scan_progress);
}

/* DAC TEST: a ramp over every level from blanking to the top code, then one
 * bar per bit whose halves differ in that bit alone. Measurement codes,
 * not the UI palette. */
static void card_dac(void)
{
    const unsigned x0 = sample(CARD_X), x1 = sample(CARD_R);
    for (unsigned y = 11; y < 21; ++y)
        for (unsigned x = x0; x < x1; ++x) s_ui[y][x] = dac_check_ramp_code(x - x0, x1 - x0);
    const int pitch = CARD_W / DAC_CHECK_BITS, half = (pitch - 4) / 2;
    for (unsigned k = 0; k < DAC_CHECK_BITS; ++k) {
        const int x = CARD_X + (int)k * pitch;
        rect(x, 24, half, 9, dac_check_bar_base(k));
        rect(x + half, 24, half, 9, dac_check_bar_top(k));
        text_box(dac_check_pins[k] + 1, x, 36, 2 * half, ALIGN_CENTER, FONT_TEXT, MENU_INK_GREY);   /* "4" of D4 */
    }
}

/* What a long press does, one word. */
static const char *hold_action(const menu_view_t *v)
{
    switch (v->cursor) {
    case MENU_BAND: case MENU_CHANNEL: case MENU_OUTPUT: case MENU_GAIN: return "NEXT";
    case MENU_SCAN: return v->scan_active ? NULL : "SCAN";
    case MENU_VIDEO: return "CHANGE";
    case MENU_CALIBRATE: return "START";
    case MENU_DAC: return "STEP";
    default: return "EXIT";
    }
}

void menu_ui_render(menu_ui_rows_t ui, const menu_view_t *v)
{
    s_ui = ui;
    menu_ui_overflow = 0;
    memset(ui, MENU_INK_BLACK, sizeof(menu_ui_rows_t));
    list(v);
    pill(CARD_X - 7, 0, W - CARD_X + 7, H, MENU_INK_PANEL);
    card_status(v);
    if (v->cursor == MENU_DAC) card_dac();
    else if (v->scan_active) card_scan(v);
    else card_tuning(v);
    const char *hold = hold_action(v);
    if (hold) {
        const int x = badge("HOLD", CARD_X, 48, MENU_INK_WHITE, MENU_INK_BLACK);
        text_box(hold, x + 6, 49, CARD_R - x - 6, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    }
}

/* Full-screen status (calibration and the like): the title in the display
 * font when it fits, two lines and a progress bar. */
void menu_ui_render_status(menu_ui_rows_t ui, const menu_status_view_t *v)
{
    char buf[24];
    s_ui = ui;
    menu_ui_overflow = 0;
    memset(ui, MENU_INK_BLACK, sizeof(menu_ui_rows_t));
    const int x = 12, w = W - 24;
    text("C5VRX", x, 1, MENU_INK_WHITE);
    snprintf(buf, sizeof(buf), "%s  %u", v->channel ? v->channel : "", v->freq_mhz);
    text_box(buf, x, 1, w, ALIGN_RIGHT, FONT_TEXT, MENU_INK_GREY);
    rect(x, 9, w, 1, MENU_INK_RULE);
    if (text_width(v->title, FONT_BIG) <= w) {
        text_box(v->title, x, 14, w, ALIGN_CENTER, FONT_BIG, MENU_INK_WHITE);
    } else {   /* too long for the display font: a white title pill */
        const int tw = text_width(v->title, FONT_TEXT) + 16, tx = (W - tw) / 2;
        pill(tx, 16, tw, 7, MENU_INK_WHITE);
        text_box(v->title, tx + 8, 17, tw - 16, ALIGN_LEFT, FONT_TEXT, MENU_INK_BLACK);
    }
    text_box(v->line1, x, 30, w, ALIGN_CENTER, FONT_TEXT, MENU_INK_WHITE);
    text_box(v->line2, x, 37, w, ALIGN_CENTER, FONT_TEXT, MENU_INK_GREY);
    if (v->percent >= 0) {
        const int bw = 240, bx = (W - bw) / 2;
        meter(bx, 47, bw, v->percent);
        snprintf(buf, sizeof(buf), "%d%%", v->percent > 100 ? 100 : v->percent);
        text(buf, bx + bw + 8, 46, MENU_INK_WHITE);
    }
}

void menu_ui_render_splash(menu_ui_rows_t ui, int percent)
{
    s_ui = ui;
    menu_ui_overflow = 0;
    memset(ui, MENU_INK_BLACK, sizeof(menu_ui_rows_t));
    const int x0 = (W - (int)MENU_LOGO_WIDTH) / 2, y0 = (H - (int)MENU_LOGO_ROWS) / 2 - 3;
    for (int y = 0; y < (int)MENU_LOGO_ROWS; ++y)
        for (int x = 0; x < (int)MENU_LOGO_WIDTH; ++x)
            if (s_menu_logo[y][x] != MENU_INK_BLACK) rect(x0 + x, y0 + y, 1, 1, s_menu_logo[y][x]);
    const int bw = 120;
    meter((W - bw) / 2, y0 + (int)MENU_LOGO_ROWS + 8, bw, percent);
}
