#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: exercise actual settings.c across reboot.

Only NVS and receiver hardware boundaries are stubbed; persistence decisions
are compiled from the production source. No ESP-IDF SDK emulation required.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
RECEIVER = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum { STD_MODE_AUTO, STD_MODE_NTSC, STD_MODE_PAL } std_mode_t;
extern std_mode_t s_std_mode;
bool transport_set_output(unsigned level);
bool blob_load(const char *, void *, size_t);
bool blob_store(const char *, const void *, size_t);
void settings_load(void);
void settings_save(void);
void settings_save_output(unsigned);
void receiver_set_gain_mode(uint8_t mode, bool save);
#define SETTINGS_PREFS_VERSION 1u
typedef struct { uint8_t version; int8_t filter_select; uint8_t gain_mode, dfilt_mode; } settings_prefs_t;
bool settings_prefs_load(settings_prefs_t *p);
void settings_prefs_store(const settings_prefs_t *p);
'''
NVS = r'''
#pragma once
#include <stddef.h>
typedef int nvs_handle_t;
typedef int esp_err_t;
#define ESP_OK 0
#define NVS_READONLY 0
#define NVS_READWRITE 1
esp_err_t nvs_open(const char *, int, nvs_handle_t *);
esp_err_t nvs_get_blob(nvs_handle_t, const char *, void *, size_t *);
esp_err_t nvs_set_blob(nvs_handle_t, const char *, const void *, size_t);
esp_err_t nvs_commit(nvs_handle_t);
void nvs_close(nvs_handle_t);
const char *esp_err_to_name(esp_err_t);
'''
RF = r'''
#pragma once
#include <stdbool.h>
#define FPV_CHANNELS 48
unsigned rf_channel_index(void);
bool rf_set_channel(unsigned);
'''
HARNESS = r'''
#include "receiver.h"
#include "nvs.h"
#include "rf.h"
#include "output_level.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { unsigned char bytes[16]; size_t n; } slot_t;
static slot_t slots[5], pending;
static int pending_index = -1, writes, commits, fail_open, fail_set, fail_commit;
static unsigned output, channel, output_applied, gain_mode, gain_restores;
std_mode_t s_std_mode;
static int index_of(const char *key) {
    if (!strcmp(key, "settings")) return 0;
    if (!strcmp(key, "output")) return 1;
    if (!strcmp(key, "gain_mode")) return 2;
    if (!strcmp(key, "flywheel")) return 3;
    if (!strcmp(key, "prefs")) return 4;
    assert(0); return -1;
}
esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *h) {
    assert(!strcmp(ns, "c5vrx5"));
    if (fail_open) return 1;
    *h = mode; return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *data, size_t *n) {
    (void)h; slot_t *s = &slots[index_of(key)];
    if (!s->n || *n < s->n) return 1;
    memcpy(data, s->bytes, s->n); *n = s->n; return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *data, size_t n) {
    assert(h == NVS_READWRITE && n <= sizeof(pending.bytes));
    ++writes;
    if (fail_set) return 1;
    pending_index = index_of(key); pending.n = n;
    memcpy(pending.bytes, data, n); return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) {
    assert(h == NVS_READWRITE); ++commits;
    if (fail_commit) return 1;
    assert(pending_index >= 0); slots[pending_index] = pending;
    pending_index = -1; return ESP_OK;
}
void nvs_close(nvs_handle_t h) { (void)h; pending_index = -1; }
const char *esp_err_to_name(esp_err_t e) { (void)e; return "TEST_FAILURE"; }
unsigned rf_channel_index(void) { return channel; }
bool rf_set_channel(unsigned c) { channel = c; return true; }
bool transport_set_output(unsigned o) { assert(o < OUTPUT_LEVELS); output = o; ++output_applied; return true; }
void receiver_set_gain_mode(uint8_t mode, bool save) { assert(!save); gain_mode = mode; ++gain_restores; }
static void boot(void) { output = 0; channel = 0; s_std_mode = STD_MODE_AUTO; output_applied = 0;
    gain_mode = 0; gain_restores = 0; settings_load(); }
static void seed(unsigned slot, const unsigned char *bytes, size_t n) {
    assert(n <= sizeof(slots[slot].bytes)); slots[slot].n = n; memcpy(slots[slot].bytes, bytes, n);
}
int main(void) {
    boot(); assert(output == 0 && output_applied == 0 && channel == 0 && s_std_mode == STD_MODE_AUTO);
    for (unsigned level = 0; level < OUTPUT_LEVELS; ++level) {
        slots[1].n = 0; writes = commits = 0;
        settings_save_output(level); assert(writes == 1 && commits == 1);
        boot(); assert(output == level && output_applied == 1);
        settings_save_output(level); assert(writes == 1 && commits == 1);
    }
    const unsigned char legacy[] = {1, 17, STD_MODE_PAL, 0x5a};
    seed(0, legacy, sizeof(legacy)); slots[1].n = 0;
    boot(); assert(channel == 17 && s_std_mode == STD_MODE_PAL && output == 0);
    settings_save_output(3); boot();
    assert(channel == 17 && s_std_mode == STD_MODE_PAL && output == 3);
    assert(slots[0].n == sizeof(legacy) && !memcmp(slots[0].bytes, legacy, sizeof(legacy)));
    writes = commits = 0; settings_save(); assert(!writes && !commits);
    channel = 24; s_std_mode = STD_MODE_NTSC; settings_save();
    assert(writes == 1 && commits == 1); boot();
    assert(channel == 24 && s_std_mode == STD_MODE_NTSC && output == 3);
    for (unsigned bad = OUTPUT_LEVELS; bad < 256; ++bad) {
        const unsigned char b = (unsigned char)bad; seed(1, &b, 1);
        boot(); assert(output == 0 && output_applied == 0 && channel == 24);
    }
    const unsigned char too_long[] = {2, 0}; seed(1, too_long, 2);
    boot(); assert(output == 0 && output_applied == 0);
    const unsigned char valid_output = 2; seed(1, &valid_output, 1);
    const unsigned char unknown_settings[] = {2, 17, STD_MODE_PAL, 0};
    seed(0, unknown_settings, 4); boot();
    assert(output == 2 && channel == 0 && s_std_mode == STD_MODE_AUTO);
    slots[0].n = 0; boot(); assert(output == 2 && output_applied == 1);
    const unsigned char invalid_settings[] = {1, FPV_CHANNELS, 255, 0};
    seed(0, invalid_settings, 4); boot();
    assert(output == 2 && channel == 0 && s_std_mode == STD_MODE_AUTO);
    writes = commits = 0; settings_save_output(OUTPUT_LEVELS); settings_save_output(1000);
    assert(!writes && !commits); boot(); assert(output == 2);
    fail_open = 1; boot(); assert(output == 0 && !output_applied); settings_save_output(1);
    fail_open = 0; boot(); assert(output == 2);
    writes = commits = 0; fail_set = 1; settings_save_output(1);
    assert(writes == 1 && !commits); fail_set = 0; boot(); assert(output == 2);
    fail_commit = 1; settings_save_output(1); fail_commit = 0; boot(); assert(output == 2);
    settings_save_output(1); boot(); assert(output == 1);
    /* Gain mode: never restored, every boot starts AUTO, even with a fixed
     * mode left in NVS by an earlier firmware. */
    boot(); assert(!gain_restores);
    const unsigned char old_fixed = 42; seed(2, &old_fixed, 1); boot(); assert(!gain_restores && gain_mode == 0);
    /* A "flywheel" key left by an earlier firmware is ignored (flywheel removed). */
    const unsigned char old_flywheel = 1; seed(3, &old_flywheel, 1);
    writes = commits = 0; boot(); assert(!writes && !commits && output == 1 && gain_mode == 0);
    /* Menu prefs (FILTER, GAIN model, PHY FILT): saved once, read back after
     * a reboot, unchanged values not rewritten, another version ignored. */
    settings_prefs_t p = {SETTINGS_PREFS_VERSION, 1, 254, 0}, q;
    assert(!settings_prefs_load(&q));
    writes = commits = 0; settings_prefs_store(&p); assert(writes == 1 && commits == 1);
    boot(); assert(settings_prefs_load(&q) && !memcmp(&p, &q, sizeof(p)));
    settings_prefs_store(&p); assert(writes == 1);
    p.dfilt_mode = 8; settings_prefs_store(&p); assert(writes == 2);
    assert(settings_prefs_load(&q) && q.dfilt_mode == 8 && q.filter_select == 1 && q.gain_mode == 254);
    const unsigned char future[] = {2, 1, 0, 12}; seed(4, future, sizeof(future));
    assert(!settings_prefs_load(&q));
    puts("PASS: actual settings.c output save/reboot, legacy compatibility, invalid data, wear and NVS failures");
    return 0;
}
'''


class SettingsTest(unittest.TestCase):
    def test_actual_production_source(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            shutil.copyfile(ROOT / "main/settings.c", folder / "settings.c")
            shutil.copyfile(ROOT / "main/output_level.h", folder / "output_level.h")
            for file, text in (("receiver.h", RECEIVER), ("nvs.h", NVS), ("rf.h", RF), ("harness.c", HARNESS)):
                (folder / file).write_text(text, encoding="utf-8")
            binary = folder / "test_settings"
            subprocess.run([os.environ.get("CC", "cc"), "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(folder), str(folder / "settings.c"), str(folder / "harness.c"),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
