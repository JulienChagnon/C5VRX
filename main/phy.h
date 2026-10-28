/* C5VRX-5 by Twotoz and the C5VRX contributors: owned access to the pinned
 * ESP32-C5 PHY (from legacy/c5vrx4/main/phy_rx_lab.c). Writes outside the public
 * API run only on the libphy whose SHA256 the build verified
 * (C5VRX_PHY_PINNED); on any other PHY they refuse and return false. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Recursive PHY transaction. Every end publishes a new generation, so a
 * control decision taken before a retune is never applied after it. */
void phy_begin(void);
void phy_end(void);
/* Recursive read-only ownership: serialize a diagnostic snapshot with PHY
 * writers without invalidating observations or advancing the generation. */
void phy_read_begin(void);
void phy_read_end(void);
uint32_t phy_generation(void);
bool phy_busy(void);
/* Non-blocking gain/offset ownership without a new generation. */
bool phy_try_actuator(uint32_t expected_generation);
bool phy_try_actuator_wait(uint32_t expected_generation, uint32_t wait_ticks);   /* bounded wait for the lock */
void phy_end_actuator(void);
/* Receive observers invalidate buffered evidence BEFORE a physical change.
 * Hook must be nonblocking, allocation-free, and must not acquire PHY ownership.
 * Register once during observer setup; NULL removes it. */
void phy_set_observation_hook(void (*hook)(void));
void phy_observation_changed(void);

/* Analog RX filter, after ESPARGOS esp-sdr (GPL-3.0, commit ac627b0b):
 * BBTOP 0x67 regs 6/7 hold the RX0 capacitor code (absolute low six bits),
 * regs 8..13 the second stage (calibrated bytes + skirt offset). The
 * factory bytes are captured once after PHY init for explicit profiles.
 * CALIBRATED code / zero skirt now select the 11p baseline byte 60. */
#define PHY_FILTER_CALIBRATED (-1)
bool phy_filter_capture_base(void);
bool phy_filter_set_code(int code);
bool phy_filter_set_skirt(int offset);
int phy_filter_code(void);
int phy_filter_skirt(void);
int phy_filter_calibrated_code(void);
/* Hardware read-back of BBTOP 0x67 regs 6..13 against the bytes the shadow
 * code/skirt imply (60 for the 11p defaults). Write-time readback alone
 * cannot see a later vendor write (gain force, channel setup) restoring other bytes. Returns
 * the number of mismatching registers (0 = live filter as set), -1 when
 * unsupported; hw[8]/want[8] receive the bytes when not NULL. */
int phy_filter_hw_mismatch(uint8_t hw[8], uint8_t want[8]);

/* Fixed production digital RX filter, separate from the analog
 * capacitor/skirt setting. Mode 12 (0x600A0430[21:18]): VTX-on lab
 * (console S suite 3, 5865 MHz, 2026-10-26) C/N +0.20/+0.28/+0.52 dB and
 * fewer FM clicks than mode 0 in three runs; mode 0 had only been chosen on
 * VTX-off noise (PR #244), where 12 reads ~1 dB more noise. Other modes
 * remain lab research. The vendor-selected ADC rate is preserved.
 * Unsupported PHYs refuse the write and report -1 for readback. */
#define PHY_DFILT_MODE_DEFAULT 12
/* Applies the selected mode (PHY_DFILT_MODE_DEFAULT unless the operator
 * chose another in menu PHY FILT); every RF restore reapplies it. */
bool phy_dfilt_apply_default(void);
int phy_dfilt_hw_mode(void);
/* Operator choice among the bench-safe modes 0/4/8/12 (PR #244, #279), kept
 * across reboots by the settings prefs; the noise maps and the C/N reference
 * are measured in PHY_DFILT_MODE_DEFAULT. Returns false for another value or
 * a failed readback (then the previous mode is restored). */
bool phy_dfilt_select(int mode);
int phy_dfilt_mode(void);
int phy_dfilt_next(int mode);

/* PR #254 11p baseline, pinned helper (1,0). Verified full entry and exact
 * saved-state rollback; ESP_ERR_INVALID_RESPONSE = entry failed/restored,
 * ESP_FAIL = rollback failed. Callers must stop on either failure.
 * hw_enabled checks flags/MMIO; analog overrides have separate readback. */
esp_err_t phy_11p_apply_default(void);
int phy_11p_hw_enabled(void);

/* RX DC DACs (PBUS blocks 2/3 bank 2). A hold forces one pair in PBUS
 * debug mode with every owned live word re-asserted; any gain write or PHY
 * restore releases it first (rf.c) and the post-gain hook re-holds. */
typedef struct {
    bool measured;
    int codes[2];
    int residual_mcells[2];
} phy_dco_result_t;
esp_err_t phy_dco_search(bool (*measure)(int dc[2]), phy_dco_result_t *res);
/* The same search started from given codes (a converged neighbour row) instead
 * of the live ones: start NULL = phy_dco_search. */
esp_err_t phy_dco_search_from(bool (*measure)(int dc[2]), const int start[2], phy_dco_result_t *res);
/* Short search probes, full independent measurement at the chosen codes.
 * Failed verification leaves measured=false and restores every owned word. */
esp_err_t phy_dco_search_verified(bool (*measure)(int dc[2]), bool (*verify)(int dc[2]),
                                 const int start[2], phy_dco_result_t *res);
/* The same codes for a row whose receiver noise is below one quantizer step:
 * the IQ is one fixed code, so the DC is found per axis by bisection on the
 * code where that fixed code flips (DC within half a cell). measured=false
 * when an axis has no flip within the search range. */
esp_err_t phy_dco_search_flat(bool (*measure)(int dc[2]), phy_dco_result_t *res);
esp_err_t phy_dco_search_flat_from(bool (*measure)(int dc[2]), const int start[2], phy_dco_result_t *res);
esp_err_t phy_dco_hold(int code_i, int code_q);
bool phy_dco_release(void);
bool phy_dco_held(void);
/* PBUS DC words, read back: bank 1 (written by the vendor PHY) and bank 2
 * (the pair the hold forces), I then Q. false without the pinned PHY. */
bool phy_dco_banks(int bank1[2], int bank2[2]);
/* Bank 1 pinned per DC table (phy.c pin_bank1): a new table (channel,
 * filter, gain span lo..hi) takes the next hold's bank 1 and forces it on
 * every later hold. Status: the pinned pair, the last differing live pair
 * the vendor had written (-1 none) and how often it was overridden. */
void phy_dco_bank1_reset(uint8_t lo, uint8_t hi);
uint32_t phy_dco_bank1_status(int pin[2], int moved[2]);
/* The vendor's bank-1 source table, phy_param+0x180: 7 frequency groups x
 * 9 RF stages x {I, Q}, written by the boot RX DC calibration only. *group:
 * the group of the vendor's current frequency. NULL without the pinned PHY. */
#define PHY_VBANK1_GROUPS 7u
#define PHY_VBANK1_STAGES 9u
const int16_t *phy_vendor_bank1_table(unsigned *group);
#define PHY_DCO_TRY_MAX 8   /* codes per verified live trial (phy_dco_try_pair) */
bool phy_dco_nudge(int di, int dq, int out[2]);
/* Sample the actual current DAC pair twice; no mode change or assumed codes.
 * try_pair requires the caller's PHY actuator lease and an exact actual
 * RF/BB/fine match to packed_gain. A one-code trial may enter debug hold.
 * ESP_OK: target verified; ESP_ERR_INVALID_STATE: refused before writes;
 * ESP_FAIL: target failed, original pair verified; ESP_ERR_INVALID_RESPONSE:
 * even original restoration failed. Caller must invalidate sample epochs on
 * any attempted write and keep a last validated pair until remeasurement. */
bool phy_dco_read_pair(int out[2]);
esp_err_t phy_dco_try_pair(uint32_t packed_gain, const int expected[2], const int target[2]);
