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

/* The boot logo stays while Direct Gain makes its first decision (a hold,
 * also without a carrier), so live video starts on a found gain: at least
 * SPLASH_MIN_MS for the logo to be seen, at most SPLASH_MAX_MS. The bar
 * follows the wait and fills once the gain holds. */
#define SPLASH_MIN_MS 3000
#define SPLASH_MAX_MS 4000

static void splash_wait_gain(void)
{
    const uint32_t holds = s_gain.holds;
    const int64_t t0 = esp_timer_get_time();
    int settled_ms = -1;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        const int ms = (int)((esp_timer_get_time() - t0) / 1000);
        if (settled_ms < 0 && s_gain.state == DG_HOLD && s_gain.holds != holds) settled_ms = ms;
        if ((settled_ms >= 0 && ms >= SPLASH_MIN_MS) || ms >= SPLASH_MAX_MS) break;
        menu_splash_progress(settled_ms >= 0 ? 100 : 30 + 60 * ms / SPLASH_MAX_MS);
    }
    menu_splash_progress(100);
    if (settled_ms >= 0) printf("SPLASH gain G%u held after %d ms\n", s_current_gain, settled_ms);
    else printf("SPLASH ended at %d ms before a gain hold\n", SPLASH_MAX_MS);
}

void app_main(void)
{
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
    const bool splash = menu_splash_begin();
    s_boot_stage = "tasks";
    receiver_start();
    menu_splash_progress(20);
    control_start();
    menu_splash_progress(30);
    if (splash) splash_wait_gain();
    menu_splash_end();
    s_boot_stage = "ready";
    printf("C5VRX5 %s ready: %s %u MHz, %s, ultrafine IQ, KEEL-1 ('h' for keys) fw=%s\n",
           esp_app_get_description()->version, rf_channel()->name, rf_frequency_mhz(),
           transport_program_name(), firmware_id());
}
