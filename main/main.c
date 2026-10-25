/* C5VRX-5 by Twotoz and the C5VRX contributors: ESP32-C5 analog 5.8 GHz FPV
 * receiver. https://github.com/Twotoz/C5VRX - https://twotoz.github.io/C5VRX/ */
#include "receiver.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "rf.h"

volatile const char *s_boot_stage = "start";

/* The boot logo while the receiver tasks start and Direct Gain makes its
 * first decision (a hold, also without a carrier). The bar fills evenly
 * over SPLASH_MIN_MS; the logo stays longer, up to SPLASH_MAX_MS, only
 * while the gain has not held yet, so live video starts on a found gain. */
#define SPLASH_MIN_MS 3000
#define SPLASH_MAX_MS 4000

static void splash_wait_gain(int64_t t0, uint32_t holds, int64_t deadline)
{
    int settled_ms = -1;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        const int ms = (int)((esp_timer_get_time() - t0) / 1000);
        if (settled_ms < 0 && s_gain.state == DG_HOLD && s_gain.holds != holds) settled_ms = ms;
        menu_splash_progress(ms * 100 / SPLASH_MIN_MS);
        if ((settled_ms >= 0 && ms >= SPLASH_MIN_MS) || ms >= SPLASH_MAX_MS || esp_timer_get_time() >= deadline) break;
    }
    if (settled_ms >= 0) printf("SPLASH gain G%u held after %d ms\n", s_current_gain, settled_ms);
    else printf("SPLASH ended at %d ms before a gain hold\n", SPLASH_MAX_MS);
}

void app_main(void)
{
    const int64_t app_t0 = esp_timer_get_time();
    /* USB output never blocks a task when no host is attached. */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    fcntl(fileno(stdout), F_SETFL, fcntl(fileno(stdout), F_GETFL, 0) | O_NONBLOCK);
    usb_serial_jtag_vfs_use_nonblocking();
    /* The console first: flashing (download key) and diagnostics must stay
     * reachable whatever a later boot step does. */
    console_start();
    firmware_print();   /* first log line names the exact build */
    if(!control_prepare()) {
        s_boot_stage="control_memory";
        printf("CONTROL stack reservation failed; USB status/download remain available\n");
        return;
    }
    s_boot_stage = "rf";
    ESP_ERROR_CHECK(rf_start());
    printf("BOOT_TIMING radio_ms=%lld\n", (esp_timer_get_time() - app_t0) / 1000);
    s_boot_stage = "settings";
    settings_load();
    s_boot_stage = "flightlog";
    flightlog_start();
    /* Start at the first index of the highest RF stage; Direct Gain takes
     * over at once (table maximum without a carrier). */
    receiver_set_gain(arc_gain_highest_rf_stage_start(rf_gain_table()));
    s_boot_stage = "calibration";
    calibration_start();
    /* The receive profile (filter class + tracker) of this channel is
     * chosen before live TX starts; it never changes under live video. */
    profile_load();
    profile_boot();
    s_boot_stage = "transport";
    if (transport_start() != ESP_OK) {
        s_boot_stage = "transport_fault";
        printf("TRANSPORT boot failed: live OFF; USB status/download remain available\n");
        return;
    }
    s_boot_stage = "splash";
    const int64_t splash_t0 = esp_timer_get_time();
    const uint32_t splash_holds = s_gain.holds;
    /* Leave one second of a ten-second target for final TX handover.
     * Never add a fixed warning screen after the normal splash. */
    const int64_t ready_deadline = app_t0 + 9000000;
    const bool splash = esp_timer_get_time() < ready_deadline && menu_splash_begin();
    s_boot_stage = "tasks";
    receiver_start();
    control_start();
    if (splash) splash_wait_gain(splash_t0, splash_holds, ready_deadline);
    menu_splash_end();
    s_boot_stage = "ready";
    printf("BOOT_TIMING ready_ms=%lld target_ms=10000\n", (esp_timer_get_time() - app_t0) / 1000);
    printf("C5VRX5 %s ready: %s %u MHz, %s, ultrafine IQ, KEEL-1 ('h' for keys) fw=%s\n",
           esp_app_get_description()->version, rf_channel()->name, rf_frequency_mhz(),
           transport_program_name(), firmware_id());
}
