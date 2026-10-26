/* C5VRX-5 by Twotoz and the C5VRX contributors: bench-only VTX-on PHY
 * sensitivity lab (console S + suite key). Extends the PR #254 VTX-off noise
 * lab; see docs/PHY_SENSITIVITY_LAB.md for the candidate inventory, the
 * test plan and the evidence limits. Nothing is saved; every write is
 * restored and verified, a failed restore reboots. */
#pragma once
#include <stdbool.h>

/* Suite keys: '0' shams, '1' NF-auto, '2' IQ correction, '3' digital
 * filter, '4' packet/RX policy, '5' clocks, '6' ADC/LO-cap/ULP, '7' analog
 * filter code, '8' gain rows, '9' DAC output off, 'a' all of 1..9. */
bool sense_lab_suite_valid(char suite);
void sense_lab_request(char suite);
void sense_lab_help(void);
/* Calibration task: runs a pending request. quiet = no menu, scan, pause or
 * PHY owner. */
bool sense_lab_pending(void);
void sense_lab_run(bool quiet);
