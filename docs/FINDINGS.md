# C5VRX-5 findings

C5VRX-5 keeps only what the C5VRX-4 research on PR #190 (branch
`codex/c5vrx4-pair-range`) showed to help the picture. The full study, its
scripts and data live in that branch under `v4/docs/PICTURE_CLIFF_STUDY.md`,
`v4/tools/dsp_search/` and `v4/docs/data/`. This page is the summary.
Synthetic results are host simulations; board results are labelled as such.
Measured range is not established.

## The problem: the whole picture dropped out

With slightly bad video the C5VRX-4 picture did not get noisy, it vanished.
A goggle model (sync low-pass, slicer, H-PLL flywheel, ideal and peak-clamp
AGC) over full fields showed why: the trained 32-phase trackers (PAIR,
RANGE32) slip on VTXs with a larger deviation (>= ~1.05) or an off-centre
carrier and never re-acquire. They lost the whole picture in 8-25 % of cases
even at 20-25 dB C/N. The board logs added two contributors: the DC search
running under weak video, and gain hunting at the range edge.

## Where information is lost, and where clicks come from

An information ledger added one loss at a time to identical signals:

- The 4-bit IQ lanes, folding included, lose essentially nothing against
  the analog IQ. Using one sample per 50 ns span is not a loss either.
- Clicks are FM threshold physics: the noise phasor circles the origin. The
  ideal analog discriminator (an RX5808) has them too. The IQ shows when
  they happen: the sample lands in the origin cells.
- The loss is the 32-sector phase lattice (625 kHz per output step). A
  tracker with clipping recovers it; the flashed LUT reaches its own float
  bound below 14 dB C/N.

## U85Q: the demodulator

An untrained search chose among closed-form formulas only (no fitted LUT
entry), with held-out confirmation:

- state: 32 tracked phases, `p' = p + 0.85 * wrap(observation - p)`;
- observation: the PAIR4411 shared word (sample A full byte + sample B
  signs);
- origin cells: four coarse quadrant tokens with their own gain 0.5. Erasing
  them (U85E) was sharper but failed under multipath, because fades put
  real signal into the origin cells;
- output: the tracked advance, linear over -6..+9 MHz onto the DAC.

Held-out results (picture = correctly displayed lines, mean):

| model | nominal picture / SINAD | mixed robustness picture / SINAD |
| --- | --- | --- |
| **U85Q** | 0.998 / 3.55 dB | **0.893** / 1.93 dB |
| U85E (origin erasure) | 0.995 / 4.26 dB | 0.706 / 2.48 dB |
| HC50 (stateless) | 0.981 / 2.17 dB | 0.872 / 0.76 dB |

The mixed set covers echo + fading, carrier +-2 MHz, DC, I/Q 1.4/20 deg and
a CW spur. Trained PAIR/RANGE32 scored 0.83 / 0.82 on the nominal set, with
0.00 in their worst cases. Under multipath U85Q matches its float upper
bound; what remains there is in the signal, not the LUT.

The output window is the one trade-off left: narrower is sharper but clips
the sync tip when the carrier is off centre.

## Board validation (2026-10-11, real video, A1)

U85Q with the narrow filter and the DC latch, 12 snapshots:

- the carrier's sync tip sat at -5.0 MHz and white at +6.25 MHz. U85Q's
  -6..+9 MHz window holds both; U85E's -3.5..+7.5 MHz clipped the sync tip,
  so a narrower window is not safe;
- clicks per 1000 samples: U85Q 83.0, U85E 85.7, HC50 101.5.

## AFC target matched to U85Q (C5VRX-5)

C5VRX-4 centred the VTX blanking level at -436 kHz, the design centre of
the trained trackers. U85Q's window is -6..+9 MHz, and the board's real VTX
swung wider than the deviations searched (up to 1.4). Shifting the AFC
porch target, held-out simulation (board conditions with +-1 MHz carrier
error, stress, 20 cases per cell at C/N 4-20):

| porch target | picture (all) | SINAD | deviation 1.4 | deviation 1.8 |
| --- | ---: | ---: | ---: | ---: |
| -436 kHz (C5VRX-4) | 0.908 | 2.42 dB | 1.00 | 0.64 |
| **-1036 kHz (C5VRX-5: -1.0 MHz)** | **0.969** | **3.27 dB** | 0.98 | 0.90 |
| -1436 kHz | 0.961 | 3.89 dB | 0.95 | 0.91 |

The larger shift buys more SINAD but starts to drop pictures; -1.0 MHz is
the balanced choice. Board confirmation is pending.

The Direct Gain V5 band also fits U85Q: same simulation, mean over C/N
4-20 and deviation 0.69-1.4, by IQ radius (cells):

| radius | 3.0 | 3.6 | 4.2 | 5.0 | 5.7 | 6.5 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| picture | 0.992 | 0.997 | 0.987 | 0.995 | 0.993 | 0.980 |
| SINAD dB | 3.16 | 3.09 | 3.32 | 3.03 | 2.72 | 1.75 |

The healthy band (P50 13..32, radius ~3.6..5.7, targets 4.1 / 5.2) sits on
the flat part; folding only costs above it. The gain controller is kept.

## The narrow analog filter

FM threshold follows the noise bandwidth before the demodulator. The board
ran ~19.4 MHz (noise bandwidth 20.7 MHz) where Carson's rule for typical
VTX video is ~13-14 MHz. Host results, U85E:

| filter | lock at 2 dB | lock at 4 dB | mean SINAD |
| --- | ---: | ---: | ---: |
| ~20 MHz | 0.66 | 0.92 | 4.26 dB |
| 14 MHz | 0.95 | 0.99 | 4.55 dB |
| 12 MHz | 0.98 | 0.99 | 4.50 dB |

14 MHz costs <= 0.3 dB on a strong signal for every tested deviation, and
helps every weak case. C5VRX-5 measures the setting per board (VTX off,
once) and keeps the one with the lowest noise bandwidth that still covers
14 MHz. (Later lowered to a 12 MHz default class, plus 10 MHz narrow and
16 MHz wide classes; `calibration.c` FILTER_MIN_KHZ.)

## Receive-chain fixes carried over

- DC: the per-gain DC DAC pair is searched only in confirmed quiet, never
  on a channel where a carrier was seen this boot, and held after every gain
  write (board: DC median ~134 LSB without the hook, ~11 with it).
- Sampling phase: about 3 boots in 10 sampled MODEM_DIAG mid-transition;
  a fourth-difference probe finds it and slips the RX clock.
- Direct Gain V5: healthy-band zero-write zone, 20 ms persistence for
  moderate excursions, burst gate and staged overload recovery.

## What did not help (kept out)

Trained LUT demods, FusionDemod/AutoFit program swaps (3.8-5.1 ms engine
halts), the sync flywheel, the level servo, adaptive lanes, the radius
boost, the vendor RX recalibration (grain on strong pictures), one lookup per
IQ40 sample, frequency-only trackers, MMSE outputs with a narrow window.

## What is left

Within this demodulator class the C5 is close to its ceiling. The remaining
dB are outside the LUT: the RF front end (LNA, filter, antenna, diversity) or
hardware with line memory at sample rate. A flight comparison against an
RX5808 is the open measurement.

## PR #196 ladder and LP handover (2026-10-12)

C5VRX by Twotoz and the C5VRX contributors. Initial operator evidence:
video appeared stable during five requested kp steps, but telemetry reached
1759 verification failures in 1760 writes. The old software shadow still
acknowledged them, so reported levels did not prove hardware contents.
Paused instruction rewriting with P produced black video while USB lived.

The first candidate (65946f1) inferred a fixed 32-bit CPU LUT port from
ESP-IDF's LL comment. Physical tests disproved it. The ESP32-C5 TRM states
that host addressing/data size must follow the configured LUT width too.
Stopped-core probes showed correct LUT16 low-half reads and neighbour bits
in some upper read halves; mode 32 accepted packed writes. Keep LUT16 and
compare only the low 16 bits. RUN writes returned unrelated RAM values and
failed almost entirely; PAUSE-only updates likewise failed and caused TX
empty events. HALT with the normal load/reset/start topology verified every
entry. The exact internal arbitration/read-latch cause is not established.
Source: [Espressif ESP32-C5 TRM, BitScrambler memory access](https://documentation.espressif.com/esp32-c5_technical_reference_manual_en.pdf).

The corrected path loads the complete target while halted, reselects each
CPU address before readback, and only publishes the shadow after all 1024
LUT16 entries verify. Failure restores baseline video. Both kp and LP
changes briefly restart output; no sample-seamless handover is claimed.
`test_kp_lut_io.c` models LUT16, address refresh, unrelated upper read bits,
a failed target and rollback, and every real kp table in both directions.
`kp_writes` counts attempted LUT16 writes.

Operator `lutfix2` telemetry reached 32,768 writes with zero verification
errors, one kp change and 31 program switches, zero transport/restart faults.
P switches measured about 5.5 ms; kp about 6.3 ms. The operator could barely
see the change. These timings belong to that build/setup, not a universal
latency or flight/range result. Continued P spam eventually hung in IDF
`parlio_tx_do_transaction` at its unbounded FIFO-ready spin (MEPC 408074e0,
control task, repeated IDLE watchdog warnings, USB heartbeat still alive).

Live TX now uses an owned nine-descriptor cycle covering the same 32-KiB
raw ring, instead of re-submitting driver loop transactions. Descriptor
ownership/zero EOF are restored with TX stopped; the FIFO-ready wait has a 20-ms timeout and
vTaskDelay so IDLE can run. A timeout returns failure and attempts baseline
recovery. The original boot/menu/RX topology and DAC pins are retained.

Every actual live change shares a five-second cooldown; failed restarts
also consume it. AUTO requires an unchanged combined kp/program target for
two seconds of usable observations in the same context. A combined target
uses one restart, not separate kp and program interruptions. P during
cooldown is refused and requires a new press later. Policy/mode reset
cannot erase the physical cooldown. `test_kp_switch_guard.c` checks boundary
times, target oscillation, coverage gaps and context changes. Diagnostics
expose switch_wait_ms, kp/prog restart times and failure counters.

Guarded owned-ring build hardware test (COM10, XIAO ESP32-C5 rev 1.0,
IDF 6.0.2, 2026-10-12): six bursts of 32 P requests, spaced 5.5 s apart,
produced six accepted program changes and 90 explicit cooldown refusals;
excess commands can be dropped by the bounded queues. Three manual kp
steps then reached verified levels 1, 2 and 3. Final totals: 9216 LUT16
writes, zero kp/prog verification failures, zero restart failures, zero
TX-empty/RX-overflow/DMA/BitScrambler faults, and no watchdog/abort. Program
restarts measured 5.94-6.19 ms, kp 5.78 ms. The observer remained alive.
This proves that tested manual sequence, not all flight conditions or the
AUTO target curve; continuous-video quality and the two-second AUTO dwell
still need field observation. Firmware was app-only flashed with a matching
partition layout and NVS/PHY left intact.

Sampling drift was observed separately and is not claimed fixed here.

## Review follow-up: recovery, calibration and output level (2026-10-12)

PR comments 6123181977 and 6123278553 identify remaining safety and
compatibility gaps. The first comment reviewed obsolete 65946f1; its
32-bit LUT-port endorsement was contradicted by the physical tests above.
The corrected LUT16/HALT access is retained.

`transport_recovery.h` drives the production restart: checked teardown,
BS enable, program load, instruction readback, complete LUT16 write/readback,
BS reset/start, RX/TX enable, RX start and bounded TX start. The last good
LUT/configuration is published only after the whole chain succeeds. Each
failed candidate is cleaned up before exactly one fully verified rollback;
failed rollback or teardown latches live OFF and reports an error without
aborting/retrying indefinitely. USB runs independently. Menu return and boot
use this same verification path. `test_transport_recovery.c` injects every
candidate and rollback stage failure, initial-load failure and failed cleanup,
and verifies publication only after successful final DMA start. These are
host coordinator tests, not physical SDK failure-injection evidence.

Manual P/k save AUTO OFF once, and AUTO is stopped/reconciled after restart
errors. Missing/invalid settings select AUTO kp+LP by the operator's explicit
decision; existing saved OFF is respected. Invalid probe/menu/scan windows
clear target confirmation. Five-second backend cooldown and two-second
continuous usable-evidence confirmation remain in force.

Issue #199 has no attached raw snapshot. A matching synthetic +1.23/-3.26
cell DC and 1.7-cell complex noise test now runs the exact shared quiet and
filter-measurement helpers, with no threshold change and no prior DC search.
100 synthetic bandwidth settings complete without a definite-carrier abort
and leave eligible quiet candidates. A too-narrow/noisy setting can be
skipped, just as in the real sweep. Weak FM (-1..3 dB model C/N) is refused
as quiet; failed/short probes, recent sync, origin and rails retain separate
refusals. These 100 synthetic settings are not the actual PHY register sweep.
The reporter's complete physical sweep and subsequent DCO remain pending.

Issue #200 manual OUTPUT is integrated via g/menu. Boot is always 1x;
1.5/2/2.5x change only DAC6 low bits. Transformation is applied to original
kp/program tables, never cumulatively to an already-scaled table. Verified
level/program/output survive menu return and rollback; T reports clipping
as the count of transformed LUT entries, not live sample clipping. HDZero
acceptance with the reporter's low-deviation VTX and a wide VTX is pending.

`research/output_level.py` uses the existing bit-exact U85Q and LP models
over PAL/NTSC, deviations .69/1.0/1.7 and C/N 6/30 dB, plus an unequal
sync/picture swing approximating issue #200 (-2.82/-1.5/+3.7 MHz): 160 model rows,
including four manual output levels and an ideal sync/white-referenced
candidate. All eight kp LUTs additionally pass identity and metadata
preservation checks. The DAC model assumes ideal 3.3 V GPIO, nominal
resistors, 200-ohm series resistor and 75-ohm termination (maximum .623 V
loaded). This differs from the issue reporter's nominal voltage model:
neither is a measured board voltage, and actual output impedance/load must
be established before interpreting compliance or selecting a voltage target.

Example, PAL strong carrier, U85Q, deviation 1.7: manual 2x clips 23.65%
of pre-LP samples and luma SINAD falls from 15.63 to 11.55 dB. The ideal
sync/white-reference candidate selects 1.19x, clips .64% and scores 15.68 dB.
At deviation .69, the nominal loaded sync depth rises from .060 V (1x) to
.127 V with the ideal 2.5x candidate, with .66% clipped samples. The high
gain cap prevents this example from reaching a nominal .286-V sync target;
gain does not increase the DAC's physical voltage span. Noise and transient
excursions can still clip even with ideal reference headroom.

The existing goggle metric fits analogue amplitude away, so its normalised
picture score cannot prove AV-input compatibility. The simulation separately
reports unnormalised nominal depth/swing/clipping. It demonstrates potential
benefit and the harm of blind 2x, not a tested live estimator, HDZero model,
actual sensitivity gain or flight-safe automatic output. Automatic OUTPUT
is consequently not enabled. A future implementation needs reliable sync,
porch **and white/burst headroom** measurements, context resets, failed-write
rollback, at least 2 s stable evidence and the shared 5 s application gap;
it must hold on invalid/weak observations rather than follow a fade.

Follow-up board evidence: firmware `5.0.0-pr.196-output2` passed all eight
kp levels in both directions including wraps, two P-spam bursts, 1x/2x menu
return, 2x preservation across kp/LP and an OFF-persistence reboot. Before
reboot: 16 kp and 3 program changes, 25,600 total verified LUT writes,
zero verification/restart/transport faults and no watchdog/abort. Kp changes
were about 4.2-4.4 ms and program changes about 3.8 ms in this VTX-off run.
After testing, AUTO kp+LP was explicitly restored and OUTPUT was 1x.

Manual F first caught a follow-up regression: full recovery teardown was
also stopping RX during the menu/status raster, so all calibrator copies
failed. The correction keeps RX and its descriptor metadata alive while
normal menu/status screens own TX; failed pipeline recovery still stops all
enabled parts. The production stop-parts helper now has a host regression
assertion for retained RX plus complete cleanup even when a stop fails.

Repeating F on the board (operator confirmed VTX OFF) then measured all
100 settings, plus calibrated baseline/recheck (102 setting log records),
and stored normal BW40 code48/skirt8, width 13.125 MHz, NBW13.280 MHz;
narrow code60/skirt0, width11.875 MHz, NBW11.899 MHz; wide code24/skirt0,
width18.125 MHz, NBW18.245 MHz, calibrated NBW20.612 MHz. It subsequently
searched 12 DC rows and measured a floor at gain72; no missing rows remained.
The bounded startup finished in 119.205 s without no_quiet_channel,
sample_failure or new transport/verification faults. Sampling phase still
reported unsettled and is an explicitly added investigation, not a passed
test. This is this board's VTX-off acceptance, not the #199 reporter's capture.

Multi-Rescue proposal test (`research/rescue_study.py`, data/rescue_study.json
and standalone plots): 64 identical-Q4 cases x 17 candidates = 1088 results,
with eight kp values for U85Q/LP and a floating adjacent-FM same-Q4 reference.
Clean cases have four seeds (two PAL/two NTSC), deviations .69/1.0/1.4/1.7,
C/N1/4/30 dB and the existing matched filter/baseline. CFO +/-750 kHz, DC1
cell and 3-ms/-30-dB fades use two seeds. Ranking uses worst-seed absolute
model line-lock >=.90, no more than .02 lock loss vs baseline, relative burst
SINAD >= baseline-1 dB and no additional clean DAC-rail exposure, plus <=1 dB
strong-signal loss. This relative burst gate does not prove decoded colour;
negative burst SINAD may still pass if the baseline is equally impaired.

Examples: .69/CN1 LP kp.50 gives +1.98 dB worst luma SINAD vs MATCHED,
worst modeled lock .960; 1.0/CN4 LP kp.70 gives +1.17 dB, lock .995;
1.4/CN4 LP kp.85 gives +1.79 dB, lock .989. No candidate meets the gates
at 1.4/1.7 CN1, the modeled 3-ms fades, or wide/DC1 at CN4. These are
conditional noise/detail improvements, not attenuation-margin measurements
or recognisable moving-video proof. Four new hardware programs are not
justified by this result. The matched-topology restriction is reconciled
since (research README section 17); no universal automatic rescue claim.

The first non-oracle OUTPUT probe (`research/output_probe.c`) runs the
production AFC/Direct Gain helpers passively. It rejects all modeled CN-10,
CN1 and two windows of a real board VTX-off snapshot. Strong narrow PAL
accepts191/196 windows; median sync bias .86 kHz, p95-p5 spread15 kHz,
candidate gain1.61 with spread.039. The conservative five-sync-span white
reserve refuses normal/wide candidates; the global C/N proxy also
underestimates wide strong-carrier confidence. This prototype therefore
does not qualify a universal automatic OUTPUT servo. White percentiles
from a few lines cannot guarantee headroom for a subsequent bright scene.

## The ladder froze at the range edge (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. The kp + LP ladder only helps if
the control loop lets it reach the edge, and two parts of `control.c`
stopped it there (research README section 18, `research/ladder_gate.py`):

- The ladder and its switch confirmation shared the AFC window gate, which
  includes the envelope-stationarity test. Receiver noise near the FM
  threshold fails it: on the 12 MHz class 46 % of control windows pass at
  0 dB C/N and 85 % at 3 dB (sim.py IQ). Each failure held the ladder and
  restarted the confirmation. `kp_ladder_window_usable()` now gates the
  ladder on settle, gain write and context only; the AFC keeps the test.
- The confirmation wanted the exact (kp level, program) for 2 s while the
  ladder steps a level every 0.2-1 s. `kp_switch_confirm_toward()` confirms
  the program and the direction instead and applies the latest level;
  oscillation around the running pair still never switches.

Control-loop model with the real headers and the best_combo picture model
(16 seeds): arriving at the edge and staying there, the C/N where half the
windows still show a picture improves by 1.0-2.1 dB in 10 of 15 cases
(deviation 0.69 / 1.0, 12 MHz and MATCHED, with and without fades) and lands
within 0.2 dB of an instant oracle; deviation 1.4 under fades and MATCHED 1.0
under fades gain 0..0.3 dB. A slow steady fly-out changes -0.2..+0.2 dB, which
is why a walk-away test can hide it. Simulation only; board check: `kp_steps`
and `lp_switches` in `T` should follow the C/N at the edge.

## Sampling phase on the carrier, WIDE rows (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. Host model (research README
section 19): a mid-transition MODEM_DIAG read costs 0.3-0.6 dB of range
when one LSB lane mixes and 1.4-5 dB when several bits do. The boot check
ran on receiver noise when the VTX was off and only reported in-flight
drift. The firmware now chooses the cleanest of the three positions on the
carrier once per boot, probes it every 10 s against half the old limit and
repairs a confirmed excursion with a short scan (no per-boot cap; false
alarms back off to 16 min). The WIDE profile gets its own 16 MHz kp/LP rows
(sharper picture; no range gain exists for wide VTXs in this demodulator),
MATCHED uses the 10 MHz rows only for a filter measured <= 11 MHz, and
FIELD_DIAG logs kp and LP per second. Host tests and a local ESP-IDF 6.0.2
build pass; board check pending.

## Board walk: the ladder never switched (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. Board check of this PR
(`5.0.0-pr.211`, kp_auto=2, A1 5865 MHz, DEFAULT 12 MHz profile), VTX
walked through the house to spots where the picture was certainly gone.
Data: `research/data/walk_2026-10-13/` (`walk_serial.log`: FIELD_DIAG 1/s
plus `T` every 5 s; VTX on ~57-171 s; `off_*.bin`: IQSNAP with the VTX off).
Findings only, no fix yet.

- **Nothing was applied.** `prog_switches=0`, `kp_switches=0`, `kp_level=0`,
  `program=U85Q` the whole run. The internal ladder moved (`kp_steps` 1 -> 8,
  `lp_switches` 0 -> 6) but `kp_switch_confirm_toward()` never confirmed.
- **The gain never left G59-60 for a weak carrier.** Table maximum is 83.
  With the VTX on: 76 of ~114 s at G59, none above 60, `sync_valid_pct=0`
  in all but 17 s; AFC never locked, deviation never measured.
- **With no carrier the IQ is a constant DC point, not noise.** VTX off,
  G59, four snapshots (`research/dc_coherence_check.py`): DC
  (-1.5, +2.9..3.3) cells, spread 0.06-0.08 (I) / 0.4-0.5 (Q) cells, 3 I
  and 2 Q codes in 8190 bytes. Receiver noise is below one LSB at G59.
- **The DC point reads as a strong coherent carrier.** dg_measure counts a
  phase step <= 45 degrees on power >= 8; a fixed point has no phase step,
  so coherence 99-100 %. `cn_estimate_x10` turns that into 14-20 dB
  (FIELD_DIAG `cn_est_x10=200` with the VTX off, before and after the walk).
  With the window DC removed the same captures read coherence 0 (noise
  floor). `carrier()` in gain.c (coherence >= 55, P50 >= 5) is also
  satisfied by the DC point, consistent with the gain parking at G59.
- **So the ladder steers on the DC, not on the VTX.** With the VTX weak the
  C/N estimate swung 0-12 dB second to second with `sync_valid_pct=0`; the
  wanted program crossed the 8.5 dB LP threshold (1.5 dB hysteresis) back
  and forth, and the 2 s directional confirmation restarted every time.
- **During the fade itself the ladder was blind.** 57-77 s: 50-120 gain
  writes per second; `kp_ladder_window_usable()` rejects any window with a
  write in the previous 100 ms, so the smoothed C/N was not even updated.

Where the DC comes from and why the gain stops at G59
(`range_diag_vtx_off.txt`, `T` DCO line; gain.c, calibration.c):

- **G59-60 are DCO FLOOR rows, so their DC is never corrected.** The DCO
  table covers the top RF stage, G54-83 (`top_rf_stage_from=54`): 14 rows
  OK, 16 FLOOR. `dco_quiet_search()` marks a row and every row below it
  FLOOR when the receiver noise there is below one quantizer step ("nothing
  to correct"), so FLOOR is G54-69. Below the noise step the DC is still
  there: (-1.5, +3) cells at G59, `dc_ok=1`, `dc_held=0`. The noise being
  below one step is exactly what makes the DC a clean constant.
- **That DC point sits inside the Direct Gain healthy band.** `healthy()`
  wants `carrier()` (coherence >= 55, P50 >= 5, origin < 650) and P50
  13..32; the DC point gives coherence 99-100, P50 ~15, origin 0. The gain
  climbs while the DC is below the band and holds as soon as the DC alone
  reaches it: G59-60, about 23 steps below the maximum. `no_carrier`
  (P50 <= 4, origin >= 650, coherence < 20) can never fire on it, so the
  "listen at the table maximum" path is never taken.
- **A weak VTX is therefore invisible.** At G59 even receiver noise is
  below one LSB, so a carrier near the sensitivity limit is too; noise
  only dithers the quantizer from ~G70 (first OK row). The range edge is
  set by where the DC point parks the gain, not by the demodulator.
- `RANGE_DIAG verdict=OK strong carrier with gain headroom` with the VTX
  off, for the same reason.

This is not PR 211 code: gain.c and the DCO FLOOR rule are unchanged by
it. PR 211's ladder only made it visible, because its C/N input is the same
coherence.

At the sync edge, after a reboot (`edge_serial.log`, `weak_*.bin`; VTX
placed where the picture is between desync and just video):

- The gain now sits at G70-76 (OK rows, `DCO held=1`) and receiver noise
  is visible (spread 1.8-2.5 cells, 11-13 codes per axis), DC still
  (+1.4..2.2, -0.8..-1.3) cells. Coherence 54-68 %, `cn_est_x10` mostly
  CN_UNKNOWN or -2..+3 dB, briefly 3-6 dB when sync returns.
  `dc_coherence_check.py` reads 3-17 points lower without the DC (50-68
  vs 33-65 %): the DC still lifts it, but no longer dominates.
- Still no switch in 40 s (`prog_switches=0`, `kp_switches=0`), internal
  `lp_switches` 4 -> 6. Here this is the curve, not a gate: the deviation
  is unknown (`deviation_x100=0`), so the ladder uses the wide class, whose
  LP window is 2.0-8.5 dB (`kp_lp_floor_x10` 1.5 dB + hysteresis/3) and
  whose kp row is 0.85 up to 2 dB. A noise-like window counts as 0 dB, so
  at the edge the wanted pair is U85Q kp 0.85, the pair already running.
- The deviation is never measured because `profile_observe()` needs
  `s_afc_locked`, and the AFC never locked in either boot, even close in
  (sync 52-62 %, `afc_corrections=0`). With no deviation the ladder can
  only ever use the wide class.

Open: whether the G59 park returns in this boot with the VTX off; why the
AFC never locks (burst gate `AFC2_BURST_MIN_X10`, CFO -476 kHz seen once);
an IQ capture of the weak carrier at G59 for comparison.

Hard check of the steering (`research/field_walk_analysis.py`,
`research/ladder_replay.py`: the C ladder and guard in Python, wide class,
each 1 s FIELD_DIAG C/N held for its 20 windows):

- **The C/N alone would have switched LP 15 times in the walk.** Best case
  (every window usable): first apply at 60.8 s, 15 internal LP changes, 15
  applies. Board: 6 internal changes (from 85 s), 0 applies. So a gate
  outside the C/N held the ladder.
- **One unusable window resets everything.** The guard wants 40 unbroken
  usable windows (2 s), the LP hold 20 (1 s). Random unusable windows in
  the replay: 1 % -> 13 applies, 5 % -> 3, 10 % -> 2, 20 % -> 0 (6 internal
  changes at 10 %, as on the board). `measurement_coverage_pct` while the
  C/N was in the LP band was 50 % (walk) and 49-62 % (edge); that counts
  the AFC envelope test and failed window copies too, so it bounds the
  ladder's share from below only. 57-77 s add the gain writes (50-120/s).
  Which gate fails the other windows is not visible in the log: `T` has no
  counter for unusable ladder windows or guard resets.
- **Even the best case flaps.** 15 program restarts in 114 s in the walk;
  at the edge LP is on for 2-4 s and off again, because a noise-like window
  counts as 0 dB, which is below the wide class's 1.5 dB LP floor.
- **The deviation decides most of the range and is never known.** The
  wide class (unknown deviation) keeps U85Q kp 0.85 below 2 dB and gains
  0.36 dB from the ladder in simulation; for deviation 1.0 the kp + LP
  ladder is worth 1.6 dB and for 0.69 3.35 dB (research README section
  16). The board never measures it because the AFC never locks.
- **Max gain is not the issue in boot 2.** At G70-78 receiver noise spans
  1.8-2.5 cells per axis, so the 4-bit quantizer is not what limits C/N
  there; more gain would mainly move the envelope out of the band. The
  G59 park of boot 1 is the gain problem (DC point, above).


### First fixes (same PR, 2026-10-13)

- **Fixed IQ point is no carrier** (`gain.h` `dg_stuck`, `gain.c`):
  `dg_measure` counts samples equal to the one before. VTX off at G59:
  476-873 per mille per 64 bytes; weak carrier in noise at G73-75: 15-301.
  At >= 400 per mille with P50 <= 32 the window is no carrier: Direct Gain
  listens at the table maximum (host test: G59 -> G83 on the board's DC
  point), and `receiver_obs_cn_x10()` returns CN_UNKNOWN for the ladder,
  FIELD_DIAG, the profile gate and the AFC strength test. This removes the
  G59 park and the 14-20 dB reading without a VTX; it does not correct the
  DC of the FLOOR rows (that needs a DCO search that works without noise
  dither, still open).
- **Unusable windows hold, they do not erase** (`kp_ladder.h`,
  `kp_switch_guard.h`, `control.c`): the pending kp/LP change and the switch
  candidate survive unusable windows and failed copies; a gap of more than
  500 ms without a usable window restarts them. Replay on the walk C/N
  (`ladder_replay.py ... p_bad new`): 10 % unusable 15 applies (old: 2),
  20-30 % 13 (old: 0).
- **Gain-write guard 5 ms** instead of 100 ms plus "no write during the
  window": the C/N is one observer probe taken right before it is read, and
  Direct Gain settles a write in 0.3-1 ms.
- **`T` LADDER line**: windows, unusable by reason (settle, gain write,
  context, copy failure), fixed-point windows, guard confirms and resets,
  current level, LP and the smoothed C/N.

Not changed yet: the wide-class curve with an unknown deviation (U85Q kp
0.85 below 2 dB, LP only 2-8.5 dB) and with it the LP flapping at the edge
(the replay best case still applies 15 program changes in 114 s); the AFC
never locking, so the deviation is never measured; the FLOOR-row DC itself.

### Walk 3 on 2d0088f, and a revert (2026-10-13)

`walk3_close_check.log`, `walk3_gain_loop.log`:

- **Close in (G25-36, 0-120 s):** no fixed-point windows (`stuck_iq=0`),
  but the C/N model still dipped below 8.5 dB in 19 of 115 s with a perfect
  picture, and LP was applied twice (46-51 s, 124-130 s). Next to the VTX
  the smoothed C/N sits at 9-13 dB, only just above the LP threshold: the
  coherence model reads too low on a strong real carrier (sim tables:
  deviation 1.0 PAL; a 45-degree step test also fails on fast FM swings).
- **Direct Gain looped** after the fixed-point change: with the VTX off
  ~1400 writes and hundreds of overloads a second, at the weak spots
  75-600/s. The fixed point sent the gain to G83; the first window after
  the write still carried the row's uncorrected DC and clipped, the
  overload dropped the gain into the FLOOR rows (18 this boot, G54-71, DC
  up to (-4.3, +3.8) cells, P50 51-87 with clipping) and round again.
  **Reverted for Direct Gain**: `dg_stuck` now only feeds the C/N model.
  The G59 park with the VTX off is back; the fix belongs in the DCO path
  (correct the FLOOR rows' DC), not in the gain controller.
- **Weak spot 147-195 s:** sync 0 at G60-67 (FLOOR rows) with coherence
  85-98 and C/N 4-12 dB: the DC point with a little carrier on it (span > 2)
  still inflates the coherence. Another reason to fix the DC itself.
- 12 program and 11 kp switches in 250 s; `copy_fail` 44 % of control
  windows (1812 of 4132).

### Second round of fixes (2026-10-13, not yet board-tested)

- **FLOOR-row DC** (`phy_dco_search_flat`, `calibration.c`, DCO_VERSION 7):
  rows whose noise is below one quantizer step get DCO codes by bisecting,
  per axis, the code where the fixed output code flips (DC within half a
  cell). Only rows without a flip in +-96 codes stay FLOOR. This targets the
  root of the G59-67 park ("black screen, gain stuck at 65"). Board check:
  boot log `DCO flat ... ok` lines, `T` DCO `floor_rows` near 0, gain
  reaching G75-83 with the VTX off.
- **LP follows the gain** (`kp_ladder.h` KP_LP_GAIN_ON/OFF): LP from G70,
  back below G65, with the fast entry and adaptive release above. kp within
  a program still follows the C/N curve.
- **AFC through missing windows** (`afc2_ctrl_miss`): failed copies,
  non-stationary windows and windows without a burst no longer clear the 16
  estimates; more than 20 in a row (1 s) do. Host test: two thirds missing
  keeps 14 of 16.
- **Control-window copy** holds the scheduler for the copy. Board check:
  `T` LADDER `copy_fail` near 0.
- `ladder_gate.py check` passes with the new timing (after 0.48-0.82 dB vs
  oracle 0.50-0.90).

Follow-up: AFC now also accepts syncs confirmed by the preceding line's
PAL/NTSC period, so a B/W camera can supply measurements without a colour
burst (`d199d02`). The short control window may contain only one sync;
such windows remain missing evidence. Host tests/build passed; board test
pending.

### Envelope C/N instead of phase coherence (2026-10-13, board test pending)

The phase-step coherence model read 9-13 dB next to the VTX and could select
weak-signal kp levels on a strong carrier. A fast FM swing also fails its
45-degree phase-step test, so the model depends on picture/deviation.

- `dg_measure` now computes the DC-removed envelope statistic
  `100 * mean(|z|^2)^2 / var(|z|^2)` on the existing observer probe.
  A fixed IQ point has no residual power and remains unknown. This extends
  the envelope statistic already used by calibration, outside the hardware
  sample path.
- `cn_estimate_env_x10` uses four filter-class tables. All runtime C/N
  consumers (kp, AFC correction, profile gates, diagnostic traces and
  per-second records) use the envelope estimate. Fixed-point windows are
  excluded from the per-second envelope average; a majority makes C/N
  unknown. `T` GAIN adds `env_x100`, `stuck` and `cn_x10`.
- Reproduction: `python research/cn_envelope.py`; full results in
  `research/data/cn_envelope.txt`. PAL, two seeds, deviations 0.69/1.0/1.4,
  256-byte probes matching `copy_probe_window`. Deviation 1.0 medians exactly
  reproduce the tables. Noise-only medians are 114-116; the unknown cutoff
  is 130. Deviations 0.69/1.4 differ by up to ~1.1 dB through 12 dB C/N
  (the earlier ~0.5 dB claim was too optimistic). Above ~15 dB, filter AM
  and quantisation limit accuracy; values saturate at 20 dB.
- Host checks cover analytic envelope moments, DC translations on both
  axes, fixed and rotating IQ, monotonicity over the full uint16 range,
  and the 6 dB AFC boundary for every filter class.
- Log audit: normal and narrow sometimes share the same setting at
  13.125 MHz, but the old class lookup matched narrow first and reported
  10 MHz. Normal now wins a shared setting; a distinct narrow setting
  measured wider than 11 MHz also uses the 12 MHz C/N model, matching the
  kp ladder's existing width boundary.
- `range_diag_vtx_off.txt` said `OK strong carrier with gain headroom`
  while `video_recent=0` and the last 60 s had no sync. The strong-estimate
  verdict now requires recent video evidence and is labelled
  `VIDEO_EVIDENCE`; it is not a claim about displayed picture quality.

Validation: all 13 C host executables pass (including 68 kp-ladder and 24
gain-fade checks), diagnostic-log tests pass, the full envelope sweep
reproduces the tables, and `ladder_gate.py check` passes. ESP-IDF v6.0.2
build passes without warnings.

### Gain-driven LP independent of IQ windows (2026-10-13)

Operator: from G70 the receiver should stay on LP. The gain thresholds were
already present, but failed copies exited the control loop before the policy
and gain-write guards held its confirmation. A decision based on gain still
depended on C/N window availability.

- Separate evidence for LP and kp: in AUTO kp+LP, G70 enters LP, G65-69
  retains it, and below G65 releases after the adaptive hold. The existing
  rescue confirmation/restart gap remains. Copy failures, unknown C/N and
  gain writes do not block the program policy; missing C/N holds kp.
- Retune settle, scan/calibration/menu ownership, unknown verified transport
  state and manual modes still block automatic switching. Pending program
  and kp evidence have independent stale timers. Failed copies remain
  missing AFC evidence and never read the stale sample buffer.
- Host regression follows G70 with no usable IQ through the actual switch
  confirmation: LP is confirmed within 1 s, without creating C/N or moving
  kp. It also covers G65-83 staying on LP, G64 release, ownership and manual
  mode. No hardware/range claim; board test pending.
- The envelope calculation uses integer raw moments in the existing sample
  loop and translates them to the DC centre once per probe. This removes
  the unfinished implementation's two extra passes and per-sample floating
  point. A direct centred oracle agrees within one ratio unit on 64
  asymmetric/correlated probes. Actual C5 observer timing remains to be checked.

These are simulation-derived decision estimates, not independently measured
RF C/N. After flashing, compare VTX-off and close-VTX logs, kp selection,
`afc_locked`, and `obs_us_max` to check the added observer cost on the C5.

## PR #212 receiver update (2026-10-18)

Direct Gain V6 adds same-context carrier memory to V5: a 10-ms fade hold,
eight-index climbs per 3 ms, and a one-write overload return to the remembered
healthy gain. Host models cover this behavior; measured range benefit remains
unproved. GAIN (`G`) saves AUTO (V6, default), MAX or fixed G78..G24.
FLYWHEEL (`U`) is saved and restores a saved ON or OFF. Default without a
saved setting: ON since 2026-10-18 (operator: the PR #218 witness fix held on
the board; was OFF).
Menu entry checks flywheel release before reserving 384-byte internal DMA
descriptor chunks: about 18/22 KiB for interlaced NTSC/PAL, with a 9/11-KiB
progressive fallback. Earlier board tests exposed heap fragmentation and
camera/flywheel drift; boot/menu and flywheel ON/OFF must be checked on the
cleaned build. [PR212](PR212.md) records the current scope and evidence limits.



## Bounded background work during sync loss (2026-10-18)

Extends C5VRX by Twotoz and the contributors' existing hardware dataplane
and native-pulse flywheel on main `275606a64508`; see the legacy
`pr-derived-findings.md` CPU-plane boundary and PR212's ownership rules.

The output alarm used to spin from its early admission time to every tip,
even without camera authority, while yielding to a returning camera, or
without the H/V evidence needed to repair that tip. Those early alarms now
advance the same clock and apply pending publications immediately. Allowed
repairs keep their timing checks, original end time and native suppression.
Already late alarms retain their miss/phase accounting. This avoids spending
the admission lead in an ISR when no DAC write is permitted.

RMT draining snapshots the producer head. Batches arriving during parsing
remain queued for a subsequent drain instead of extending the current PHY
lease. The existing overflow, source invalidation and rearm paths remain in
force. The gain timer also coalesces overdue callbacks: a notification asks
for the current completed IQ window; it cannot recover historical samples.

Evidence: production-source host fixtures check early return in all three
non-repair states, camera acquisition, wrong-standard recovery, noisy PLL
holdover, delayed interrupt/backlog handling, and an RMT callback publishing
a batch during parsing. These are software regressions, not measured CPU
occupancy, physical H/V lock or RF sensitivity. No demod LUT, filter, gain
policy, or acquired sample rate changes. Board checks still needed: compare
IDLE availability, gain response, RMT overflow/rearms, output late/miss
counters and goggle rolling on the same weak-signal setup, with flywheel
both OFF and ON. Absolute RF/demod sensitivity remains unchanged.


## Flywheel witness freshness and delayed observations (PR #218, 2026-10-18)

C5VRX-5 version 5.0.229 reported NTSC/AFC lock and zero transport faults,
but flywheel ON rolled the operator's picture. Two snapshots showed
`camera_present=0`, `h_trusted=1`, `v_trusted=1`; repairs rose by 53,960
and missed tips by 978 between host requests approximately six seconds apart.
Maximum IRQ delay was 32,600 ticks at 20 MHz (1.63 ms). Turning flywheel OFF
cleanly restored ordinary live output; the operator confirmed a stable picture.
Selected original lines and full-log hashes are in
[`flywheel_desync_2026-10-18.json`](../research/data/flywheel_desync_2026-10-18.json).
The strict model's `valid=0` alone is not proof of a bad grid: that diagnostic
model is deliberately separate from the camera-phase guard.

The reproducible software defect was a mismatch between camera presence and
repair authority. Instantaneous GPIO2 sync levels only suppressed clamps while
the 20-ms presence flag was fresh, although H/V authority survives five seconds.
An unreadable source also returned the same negative edge result as a missing
pulse. Production-source host fixtures reproduce a clamp over an asserted sync
level (or unavailable source) after presence expires with authority still set.

Live sync levels now always win, including during a partially emitted repair.
Unreadable source ownership and raw observer coverage older than 9 ms refuse
repairs before any clamp. The existing observer model already uses this 9-ms
age bound. Completed non-sync runs count as coverage: real noise with no camera
H can still use qualified holdover; observer blindness cannot. Coverage gaps
revoke old H/V authority and require fresh acquisition. Old, overflowing or
implausibly future runs cannot renew authority or steer the grid. Runs from
before a pending grid change was applied cannot adjust the replacement grid.
PLL baselines use the observed halfline, not the later ISR tip at delivery.
`SYNC_FLYWHEEL_WITNESS` exposes freshness, sample age and yield/gap/rejection
counters separately from the existing status format.

The actual-source runtime fixture covers fresh native priority, stale/unreadable
witnesses, reacquisition, PAL/NTSC switching, and completed observations in 6-ms
batches with three noise pulses per line. Its 10-second noisy scenarios retain
zero H reanchors and less than 9 ticks of late-run phase error after settling;
one-second camera holdover stays within the existing 40-tick drift limit.
These are software checks. They do not prove physical goggle lock or recovery
of the measured 1.63-ms interrupt stalls. The bounded background work from
PR #217 is included with its original authorship; pulse deadlines and native
priority remain strict. No demod LUT, RF gain policy or sample path changes.

Review follow-up (PR #218 comments): `SYNC_FLYWHEEL_LOAD` reports measured
`output_alarm` occupancy since the previous print: ISR calls, worst single ISR
(us) and per-mille CPU for the whole ISR, the early-alarm spin, the spin before
tips a native pulse then suppressed, and clamp holds. The ~19 % figure in the
review is a ceiling estimate; these counters replace it with a measurement per
scenario (strong camera, weak, none, flywheel OFF). Board numbers are pending;
hardware-timed OE gating is not attempted before them. Separately, gain-driven
LP (G70 on / G65 off) now applies only under Direct Gain AUTO: a manual MAX or
fixed G70+ forced U85Q-LP on a strong carrier; manual gain falls back to the
C/N rule (`kp_ladder_gain_evidence`, `test/test_kp_ladder.c`). Host tests only.

Before claiming the board issue fixed, compare this PR build OFF/ON on the same
camera: boot/menu/restart, native return, witness age/gaps, RMT rearm/overflow,
IRQ delay and missed tips, plus operator-visible rolling. The board remains
on the stable OFF setting while this software change is prepared.

## Deviation measured without the AFC lock (2026-10-18)

C5VRX by Twotoz and the C5VRX contributors; extends the PR #196 VTX
matching and the PR #211 ladder. The kp + LP ladder is worth 1.6 dB
(deviation 1.0) to 3.35 dB (0.69) in simulation only when the deviation is
known; unknown, it runs the wide class (~0.4 dB). The board never knew it.

- **The lock gate kept every window out.** `profile_observe()` returned
  before measuring while the AFC was unlocked. Walks of 2026-10-13: the
  PROFILE line reads `observed=0` in every log, also `walk3_close_check.log`
  close in (254 FIELD_TRACE windows with sync 100 %). Board `5.0.0-pr.218`
  (2026-10-18, 2.5 h uptime): `PROFILE_GATES unlocked=1499` against
  `observed=537`; `deviation_x100` for the session stayed -1.
- **The span does not need the lock.** A carrier offset moves sync tip and
  porch together. The real `afc2_measure()` on `research/sim.py` video
  (4092-byte windows, 12 and 10 MHz filter, PAL/NTSC, deviation
  0.69/1.0/1.4/1.7, C/N 8 and 12 dB, offset -1..+1 MHz): median span within
  19 kHz (1.4 %) of the centred carrier, inside the estimator's 8 % batch
  spread and 5 % agreement; no wide VTX reads narrow. Host test
  `test_afc_measure.c` checks the span over -1..+1 MHz in 250 kHz steps.
- **Change:** the measurement no longer waits for the lock, so the session
  deviation (the ladder's class) is set as soon as the other gates pass
  (confirmed sync, C/N >= 8 dB, healthy band, no clipping, held gain).
  Narrowing the filter still needs the lock (`profile_narrowing`,
  `profile_live_switch_wanted`; README section 3: a -0.5 MHz residual costs
  the narrow settings at the edge). A startup result that asks for a narrow
  profile while unlocked keeps DEFAULT (`narrow_awaits_afc_lock`); the live
  switch narrows after the lock and two agreeing results. Widening never
  waits. `T` PROFILE_GATES now reports `observed_unlocked`.

Why the AFC does not lock (board IQ 2026-10-18, cause still open):

- Lock needs 16 window estimates with a median absolute deviation <= 80 kHz.
  Close in (`walk3_close_check.log`, 254 FIELD_TRACE windows with sync
  100 %) the porch estimate has MAD 211 kHz; the 16-window gate passes 2 of
  238 times.
- It is not estimator noise. One control window holds 1-2 lines (4092
  samples; `afc_bias.py` models 8), yet the same model with 1-2 lines and
  `afc2_measure()` on sim.py video both stay at MAD <= 30 kHz at >= 10 dB.
- **The whole signal moves.** 14 IQ snapshots with the VTX close in on A1
  (`research/data/afc_2026-10-18/`, gain G27-42, AFC offset +500 kHz,
  `research/afc_snapshot_levels.c`): within one 205 us snapshot the levels
  agree within ~90 kHz, between snapshots the porch reads -903..-1472 kHz
  and the sync tip moves with it (-2225..-2739 kHz); the span stays
  1237-1345 kHz (deviation ~0.68). Sync and blanking shift together, so
  this is a frequency shift of the received signal, not a misplaced porch
  window.
- **Timing** (`field_trace_20s.log`, 50 ms FIELD_TRACE windows, 20 s): the
  porch sits near -1000 kHz, then every 0.4-0.8 s ramps down to
  -1300..-1650 kHz over 0.1-0.3 s and snaps back. Gain (G35-37), gain
  writes, kp and LP do not change with it.
- Two candidates fit and these data cannot separate them: the VTX carrier
  itself (supply/VCO pull) or an AC-coupled VTX modulator moving the
  blanking level with picture brightness; the C5 LO moving by ~85 ppm is
  less likely. Next test: the same 20 s with the camera lens covered (flat
  black picture). If the excursions stay, the carrier moves; if they go,
  picture content moves the blanking level.
- Consequence for this PR: a carrier that swings ~0.5 MHz is exactly the
  case in which a narrow filter hurts, so keeping narrowing behind the lock
  is right for this VTX. The deviation itself is unaffected (span constant).
  No AFC threshold is changed before the cause is known.

The same stream also showed one task-watchdog report on `5.0.0-pr.218`
(IDLE on CPU 0 starved, `gain_obs` running, MEPC 0x4202b688) in ~26 s of
log; not investigated here.

Same board, same status: the second-round fixes hold where they can be read
passively: `DCO floor_rows=0` (30 rows valid), `LADDER copy_fail=0` in
180,562 windows. Host tests and an ESP-IDF 6.0.2 build pass; the change is
not board-tested and no range gain is measured.

## Task watchdog: the gain observers took half the CPU (2026-10-18)

C5VRX by Twotoz and the C5VRX contributors; extends Direct Gain V6 and its
200 us observer/sentinel (`receiver.c`, `gain.c`).

- **Symptom.** `task_wdt: IDLE (CPU 0)` with `gain_obs` running, about once
  every few minutes on `5.0.0-pr.218` and on main after PR #220 (MEPC in
  `measure()`, `gain.c:106`). The calibration task (priority 1) starves
  with IDLE.
- **Measured** (local diagnostic build: FreeRTOS run-time counters per task
  and cycle counters per section, 10 s windows, not committed). `gain_obs`
  32-54 %, `gain_fast` 20 %, `control` 18-23 %, `esp_timer` 6.5 %; IDLE
  0.2-17 % (0.3 % in the window before a watchdog report). `measure()`
  costs 28-29 us for the sentinel's 64 bytes and 100-155 us for the
  observer's 256 bytes: ~125 instructions per byte (-O2 disassembly),
  ~1.3 MB/s in total.
- **Ruled out on the board:** flash cache (`measure()` in IRAM and the
  phase table in DRAM: unchanged, 29 us) and compiler level alone (-O2:
  29 -> 24 us).
- **Fix.** Every per-sample quantity depends on the byte or on the byte and
  its predecessor: power, flags and phase from tables, I/Q span from one-hot
  bits OR-ed per sample (no sign extension or min/max), the 45-degree test as
  one unsigned compare, and the envelope moments in their own loop that runs
  only for the 20 ms C/N probe. `test/test_gain_measure.c` compares every
  field with the former implementation (kept verbatim in the test) on 13,800
  windows: random bytes of sizes 1-8190, fixed points/rails, a rotating
  carrier at every phase step and the board IQ captures; a deliberate
  one-code change makes it fail.
- **Board** (diagnostic build with the table-driven measure): sentinel
  15-16 us, observer 49-60 us; IDLE 18-33 % in the worst of three windows
  (was 0.2-17 %); no watchdog report in that run.
- **Board, this version** (cold start via RESET, ELF `127d6b3aa`): normal
  startup and console, 0 task-watchdog reports in 5 minutes of continuous
  heartbeat (VTX not received in that run).
- **Not caused by this change: silent USB console after the flash route.**
  After `` ` `` -> ROM -> flash -> `write-mem 0x600b1034` + watchdog reset
  the USB serial console sometimes stays silent while the receiver keeps a
  picture. It first looked tied to this change (3 failures with it), but
  the unchanged main build failed the same route too (2026-10-19). After a
  replug the board came up in the ROM loader twice, so the board may stay
  powered through the video side; a RESET tap recovers it. Cause open.

## DC table from another filter: 3.7 cells of DC and the gain parked on it (2026-10-19)

C5VRX by Twotoz and the C5VRX contributors; extends the per-gain DC tables
(`dco_slots.h`, PR #196) and the FLOOR-row work of PR #211. Relates to
issue #222 track A (hidden loss in the gain allocation).

- **Board** (main after #221, A1, no VTX received, four IQ snapshots at
  G71): DC (-0.9, -3.6) cells, receiver noise 0.68 cells per axis. `T`:
  `DCO held=1`, `DCO_CONTEXT table_bw20=0 table_code=-1 table_skirt=0`
  against the live `active_bw20=1 active_code=16 active_skirt=32`,
  `filter_same=0`; `SPHASE dc_mcell` -1.1..-2.6 / -2.1..-3.7 cells in every
  status of 2026-10-18/19. The codes are held, but they were measured with
  another analog filter.
- **The gain parks on it.** p50 = 15 comes from the DC point (3.7^2 ~ 14),
  inside the Direct Gain healthy band (13..32): the gain stays at G71
  instead of the table maximum, the same mechanism as the 2026-10-13 G59
  park, now through the filter context.
- **Cost** (`research/dc_board_study.py`, `data/dc_board_study.json`;
  bit-exact U85Q, deviation 0.69, 12 MHz, quantiser scaled to a fixed
  receiver noise per axis instead of the Direct Gain radius used by
  `situations.py`, two seeds): range limit (picture >= 0.9 and SINAD >= 0)
  **14 dB C/N with the measured DC, 4-6 dB without it**; below 10 dB the
  SINAD stays at -1.6 dB (the demodulator follows the DC point). The
  2026-10-13 walk-edge state (DC 2.1 cells against 2.2 cells of noise at
  G74) costs 0-0.3 dB in the same model. Simulation, not measured range.
- **Cause.** `dco_matches()` compares frequency, gain table and IQ scale but
  deliberately not the filter ("an assumption, not measured equivalence").
  The board refutes the assumption.
- **Fix.** `dco_filter_same()` / `dco_filter_stale()` (`dco_slots.h`, host
  test `test/test_dco_slots.c`): at startup a table with rows measured under
  another filter is measured again in full, like a manual CALIBRATE of the
  DC rows, through the existing own-channel (VTX off) and quiet-channel
  (VTX on) paths; the table takes the live filter only after a complete
  pass (`done` or `floor`), so an interrupted pass is repeated at the next
  boot. Live profile switches still keep the table (unchanged, see the
  `dco_matches` comment); a WIDE profile with another filter therefore still
  runs on codes of the normal filter until a boot measures it.
- **Board check** (2026-10-19, ELF `6722a826d`, A1, no VTX; data in
  `research/data/dc_2026-10-19/`): the first boot logged `DCO startup
  filter_stale=1 table_bw20=0 table_code=-1 ... live_code=16 live_skirt=32`
  and measured all rows on the own channel (`searched=30 result=done`); `T`
  then showed `DCO_CONTEXT ... filter_same=1`. Six IQ snapshots: gain **G82**
  (was G71), receiver noise **2.15 cells** per axis (was 0.68), DC
  **1.5-2.4 cells** (was 3.7), DC/noise **0.7** (was 5.3). In the model above
  that state costs 0-0.3 dB instead of 8-10 dB. A residual Q offset of about
  -1.5 cells stays after the search; a DC search should end within half a
  cell, so this is open. No range measurement yet.

## Analog filter branches, per-filter DC, a live DC trim that runs (2026-10-19)

C5VRX by Twotoz and the C5VRX contributors; extends ESPARGOS esp-sdr's C5
filter control (GPL-3.0), PR #196's filter sweep and the DC work above.
Board: XIAO ESP32-C5, A1, no VTX; data in `research/data/filter_2026-10-19/`.

- **Two analog filter branches.** The PHY builds its analog tables at
  channel setup from the bandwidth mode then in force: set up in BW20 the
  RX0 code sets the width and the skirt does nothing, set up in BW40 the
  reverse (same boot: code 63/skirt 0 11.9 vs 19.7 MHz noise bandwidth,
  code 16/skirt 32 21.1 vs 13.0 MHz). `rf_set_channel` tuned before
  `phy_wifi_fbw_sel` and `rf_set_filter` never redid the setup, so the sweep
  measured settings in whatever branch was active: the board flew a stored
  "12.5 MHz" setting at 19.7 MHz noise bandwidth (live IQ spectrum; the
  firmware's own PSD method agrees). `channel_setup()` selects the mode
  first and `rf_set_filter` repeats the setup on a mode change;
  FILTER_VERSION 5 re-sweeps. After: stored 13.0 MHz, live 13.0-13.4 MHz.
  `FILTER_HW` reads the filter registers back (they always held what was
  set; the branch was the problem, not the registers).
- **DC per filter.** One table per (channel, filter key), index version 3
  with an upgrade that keeps measured tables; startup measures the NORMAL,
  NARROW and WIDE filters' tables (resumable, `DCO_READY`); narrowing in
  flight waits for the narrow table. Stale tables are re-measured row by row
  and stamped only when complete (review P1/P2).
- **DC moves at every channel setup.** Same DC words in both PBUS banks
  (`DCO_BANKS`), yet DC (+1.5, -0.1) -> (-0.5, -3.0) cells after 8 retunes:
  a stored table only fits the setup it was measured in. The live trim never
  ran (`DC_LIVE trials=0`): it needed a carrier circle, its sample missed a
  2 ms deadline under the PHY lease, gain writes and sampling-phase probes
  re-held the bare table codes, steps were one code within +-4, and two
  single windows had to agree within 150 mcells while the DC wanders about
  half a cell. All five fixed (noise-mean estimate, lease-free sample,
  trimmed holds, sized steps, 8-window mean, deadband 600 mcells).
- **Sensitivity model** (`research/sensitivity_model.py`): required C/N
  referred to the fixed 20.7 MHz chip channel, from no-carrier board IQ (live
  filter shape, noise, DC, digital floor) through the bit-exact U85Q chain;
  dBm only with an assumed NF. Deviation 0.69, 2 dB C/N steps:

  | board state | live NBW | DC | range limit |
  | --- | ---: | ---: | ---: |
  | before #223 (wrong DC table, G71) | 20.5 MHz | 3.6 cells | 14 dB |
  | DC table fixed (G82) | 20.7 MHz | 1.7 cells | 4 dB |
  | filter branch fixed (G83) | 13.4 MHz | 1.7 cells | 4 dB (2 without DC) |
  | after a channel round, no trim | 13.7 MHz | 3.1 cells | 6 dB |
  | after a channel round, live trim | 13.2 MHz | 1.2 cells | 4 dB |

  Simulation at measured board levels; no range walk yet. The digital path
  (quantisation, sampling phase, folding) costs 0.03-0.16 dB.
- **1 dB steps** (`data/sensitivity_2026-10-19_fine.json`): with the live
  trim the limit is **3 dB C/N** (DC 1.16 cells, of which the DC costs 1 dB;
  2 dB without it), against 14 dB before #223: 11 dB in this model.
- **Open:** a DC wander of about half a cell over seconds remains below the
  trim's reach (2 dB C/N steps cannot resolve its cost); carrier-on checks of
  the live profile switch onto the narrow table and of the trim at the edge.

## Boot-to-boot range: the DC of every channel setup (2026-10-19, PR #224)

Operator: "some boots or calibrations the range is great, some terrible".
`tools/boot_loop.py` reboots the board N times with the VTX off and records
IQ snapshots and `T` per boot; `research/sensitivity_model.py` turns every
snapshot into a range limit (required C/N, 20.7 MHz chip channel).

- **The variation is all DC.** Baseline (trial trim), six boots, 60 s after
  startup: DC 1.0-3.8 cells, range limit **2-8 dB C/N**; without the DC every
  boot is 2-3 dB (noise bandwidth 13.0-13.5 MHz, noise 1.9 cells, digital
  path 0.03-0.1 dB). Up to 6 dB, about twice the range, between boots of one
  calibration.
- **Not in any register.** PBUS bank 1 (vendor) and bank 2 (ours) read the
  same on every boot (279-280/226-227, 253/224) while the boot DC ranged
  0.7-3.7 (I) and -1.8 to -4.2 cells (Q): analog state of the channel setup,
  no deterministic correction. Within a boot it wanders 1-2 cells over
  seconds.
- **The trial trim could not follow.** 0-9 accepted steps per boot (0-3 with
  a median estimator for clipped clouds): single steps on 0.1-0.9 s samples of
  a low-priority task, verify-or-revert, a deadband.
- **Live DC servo** (`main/dc_servo.h`, test `test_dc_servo`): the mean of
  every settled observer probe per gain row (the AGC already sums I and Q),
  a computed step on both DAC codes per row from the learned slope. Clipping
  biases the mean towards zero but never across it, so the loop converges.
  Six boots: DC 0.3-0.8 cells after 60 s, range limit **2-4 dB** on every
  boot. Learned slope 170-550 mcells per code: one code is ~0.4 cells, the
  analog floor of the correction.
- **Channel round** (8 x next channel, back): 2-4 dB on four of six boots;
  two were 7-8 dB. One learned a negative slope from the new table's DC jump
  (now: steps are forgotten on any context change, a measured slope never
  flips sign on one measurement); in the other G83 sat at 0.3 cells while
  G82 held 3.9, with Direct Gain moving between rows faster than one row
  collected a mean (now: one probe sum per row, every row corrected). Board
  repeat of both fixes with a channel round and a hard calibration: pending.
- **Video quality** (`main/video_quality.h`, `T VIDEO_Q`): the spread of the
  demodulated frequency on the sync tips and the back porch, which are flat
  by definition, against the sync-to-blanking distance; and sync lines found
  per window. It sees what the no-carrier model cannot (sampling phase,
  profile, offset): the closed loops on it follow. The 2026-10-19 edge
  snapshots with 3-4 cells of DC find no line at all.
- **Live LUT re-centring is no fine DC actuator:** a kp/LUT restart costs
  5.5-7.5 ms of video (`kp_restart_us`) behind a 5 s guard.

## Blackbox reads, flicker noise and bursts (2026-10-19, PR #225)

- **First flight-log reads** (five sessions): `VIDEO_Q` read unknown in every
  second (20-window threshold = the AFC's maximum rate; now 5), and dropout
  traces filled ~75 % of the log with an unstable picture (now one per
  30 s). Without a VTX Direct Gain held G83 for 741 of 794 s, with bursts of
  255+ writes and overloads a second in ~50 s that coincided with 2-3 cells
  of DC; `T GAIN_SAT` now splits saturated windows by rail side, and a burst
  stores an IQ snapshot in the log. Six minutes, VTX off: 1034 one-sided,
  550 two-sided saturated windows, 0-1 overloads (the burst gate held).
- **Flicker (1/f) noise is before the gain.** Low-frequency excess (4-30 kHz
  against 1-3 MHz), VTX off, fixed gains: G83 7.1 dB (noise 1.9 cells), G78
  6.5 dB (1.1), G66 6.0 dB (0.5). A noise added after the gain stages would
  rise to +11..+18 dB at the lower gains; it stays at 6-7 dB, so it scales
  with the gain and no RF/baseband gain split can reduce it. Not a firmware
  fix; ~0.5-1 dB at the edge in the model.
- **Packet-like bursts.** One snapshot at fixed G72 held two strong ~55 us
  bursts ~70 us apart (DC +3.5/+4.5 cells, 16-38 % on the rails); TX is
  locked (`lock_rx_only`), so they are external. 51 snapshots on A1, A4 and
  A8 at G83 held none: rare. The blackbox overload snapshots will show their
  rate, length and channel.

## Direct Gain V7: a peak AGC (2026-10-19, PR #226)

- **Blackbox, VTX on, handheld (session 13):** close in at a C/N estimate of
  15-20 dB the V6 gain swung 10-30 steps within a second with 255+ writes and
  overloads; sync fell to 60-80 % in those seconds (94-95 % in calm ones).
  At the range edge the gain sat at G66-G81.
- **Front end:** the RF/LNA stage is 8 (the highest) over G54-G83, and the
  noise follows the baseband gain (-4.4..-4.9 dB per 5-6 steps) down to the
  quantisation floor (~0.3 cells at G60). The noise is set before the
  baseband: no range is hidden in another gain split.
- **Cause in V6:** the healthy band reached P50 32 (radius 5.7 cells, ~2.5 dB
  under the rails), so multipath clipped, the emergency drop overshot, the
  next window read weak and the climb (8 steps per 3 ms) folded again; and
  2 % of clipping without a rail-side check also lowered the gain.
- **V7 (gain.c):** the classic radio AGC on envelope peaks.
  - Attack only on folding: two-sided clipping in two probes, or 1 ms of
    saturation.
  - Release only when the highest P95 of a 16 ms memory leaves room under
    the target (P95 30). Steps come from a tuple gain model.
- **Model (test_gain_track: Rician/Rayleigh at 30 Hz, Gaussian noise peaks
  as on the board, each write 0.4 ms of garbage):**

  | metric | V6 | V7 |
  | --- | ---: | ---: |
  | close in, picture SNR >= 10 dB | 93.5 % | 94.1 % |
  | Rayleigh fading | 87.3 % | 87.0 % |
  | mid range | 94.5 % | 94.7 % |
  | usable edge (~3 dB C/N), above threshold | 38.3 % | 39.8 % |
  | writes per second, close in / edge | 122 / 90 | 46 / 29 |

  A 30 ms multipath null takes 3 writes instead of 10.
- **Conclusion:** the gain loop is not where range is lost (1.5 points at the
  edge); V7 is about stability close in. Board, VTX off: G83 with 0 writes
  in 2 minutes. Carrier test pending.
- **Tuning lessons:**
  - A level loop on a mean followed every fade both ways.
  - A fixed release level under the receiver-noise peaks (P95 15-19 at G83)
    held the gain a step below the maximum.
  - P95 alone as an attack trigger stepped down at the edge on noise.

## The whole chain against an absolute reference (2026-10-19, PR #226)

The question: the C5VRX loses range against an RX5808, so where between
antenna pin and goggle are dB lost? Every step was measured against the Wi-Fi
PHY's own calibrated per-packet RSSI and noise floor (research/nfprobe,
research/wifi_reference_snr.py).

| step | measurement | loss |
| --- | --- | --- |
| chip front end (Wi-Fi mode) | noise floor -94..-96 dBm per 20 MHz from 5180 to 5845 MHz; packets decoded to -97 dBm | none: NF ~6 dB |
| our receive state (forced G83, AGC off, dump, clocks, IQ and DAC pins) | C5VRX tuned to 5180 MHz; RSSI histogram (vendor AGC, 3 s) interleaved with IQ snapshots at G83 | none within ~1-2 dB |
| self-desense (each ingredient alone) | weak packets still decoded to -93..-96 dBm with IQ pins, all clocks, dump, DAC pins | none visible (traffic limits it to a few dB) |
| 4-bit IQ, U85Q, DAC | bit-exact model against an analog limiter-discriminator, same signals | none: the goggle holds ~1 dB lower, the picture is 4-7 dB cleaner |
| field edge captures | the picture went at -1..+4 dB C/N, where the model loses it | as modelled |

How the receive-state check works:
- The vendor AGC's RSSI histogram had its bulk at -94..-85 dBm (mode -91).
- In our IQ at G83 those packets form a cluster 6-7 dB over our noise
  (4.93 cells^2, NBW 13.6 MHz).
- The expected value from the PHY's -168 dBm/Hz is +6.3 dB.

**Result:** the chain from antenna pin to video reaches about -93 dBm (NF 6 dB,
~1.5 dB C/N in 20.7 MHz), RX5808 class or better. With this calibration,
4.93 cells^2 at G83 is -96.7 dBm, so the receiver can report the absolute
carrier power at its pin. Range lost against an RX5808 at equal distance is
signal that never reaches the pin: that is the next measurement (VTX at a
known distance and power against free space; FPV channel by channel).

## Received level at the chip pin (RX_LEVEL, 2026-10-19)

`main/rx_level.h`, host test `test/test_rx_level.c`. Shown in console `T`
(`RX_LEVEL carrier_dbm=...`) and on the menu's CHANNEL page.

- **What it reads:** the absolute carrier level at the chip pin in dBm.
- **How:** every observer probe's DC-removed mean power is referred to the
  pin with the Direct Gain model, then calibrated against the Wi-Fi PHY
  (4.76 cells^2 of thermal noise at G83 = -96.7 dBm).
- **Accuracy:** +-1 dB over G54-G83 (one RF stage, the checked model). Below
  G54 the RF stage steps are model values, flagged `exact=0`.
- **Noise:** learned per channel with no carrier at the table maximum. At
  5865 MHz (A1) it reads -95.0 dBm, 1.7 dB above the 5180 MHz calibration.
  A1 is at the top of the C5's documented range.

**Use:** a VTX at a known distance and power against free space gives the
dB lost before the chip. For example, 25 mW with 2 dBi each side over 10 m at
5.8 GHz is ~-50 dBm. To stay in the exact range (below about -67 dBm at G54),
test far enough away or at the lowest VTX power.

## KEEL-1: the gain on 3 dB home rows (2026-10-19)

The successor of Direct Gain V7.

**Cause.** Every gain row has its own DC, and the live servo learns a row
only while the gain stays on it. A gain that visits many rows briefly keeps
landing on stale DC. Blackbox session 13 at the edge:
- DC above 1 cell gave 15-25 % video, below 0.5 cell 35-40 %;
- a steady gain gave 75 % video, V6 hopping 28 %.

**Design.** The V7 peak AGC, restricted to home rows 3 dB apart from the
table maximum down. A release only goes to the next home row that fits under
the target, so there is no hunting between rows. The peak memory is 16 ms:
the rows do the steadying.

**Search harness.** `test/test_keel_track.c`:
- 12 scenarios: close / mid / edge, K 0-10, 5-80 Hz, a -12 dB obstacle;
- DC per row shrinks with settled dwell;
- each write costs 2 ms of picture;
- score = usable %, edge weighted twice.

| servo time constant | V6 | V7 | KEEL-1 |
| --- | ---: | ---: | ---: |
| 0.1 s | 52.5 | 56.1 | 57.9 |
| 0.3 s | 49.9 | 51.6 | 56.4 |
| 1 s | 44.0 | 41.6 | 52.6 |
| 3 s | 37.2 | 34.3 | 46.3 |

At the usable edge KEEL-1 settles on one row: 0-2.5 writes a second, 40 %
against V7's 38 % above threshold. With a slow servo, V7 did worse than V6:
it used fewer writes but over more rows.

**Open.**
- The real per-row servo time is not measured; the blackbox (DC, gain,
  writes and dBm per second) will say.
- Board, VTX off: G83 with the owner `keel1`. Carrier and flight test pending.

**CI.** A host-test line `cc ... && ./t` let a failed compile pass, because
`-e` ignores failures inside an `&&` list. Each compile is now its own line.

## First KEEL-1 board session and the real RF stage gains (2026-10-19)

Blackbox session 24: KEEL-1, VTX close, hand on the antenna, -25..-56 dBm
at the pin.

- **Close in the gain hunted:** G7-G61 within a second, 255 writes and
  overloads in about half the seconds, picture SNR 0-8 dB at C/N >= 20 dB.
- **After the carrier arrived or left at G79/G83:**
  - those rows held 2-3 cells of DC (IQ snapshots);
  - the picture was gone for 19 s.
  - Cause: observer windows with the carrier folding fed the DC servo. Fixed
    by keeping windows that reach both rails of an axis out of the servo
    (T: `DC_FEED rejected_folded`).
- **The model's RF stage steps are wrong:**
  - IQ snapshots at fixed gains (`research/gain_step_measure.py`), VTX at
    ~2 m: G42 -> G36 (RF stage 5 -> 4) is about -13.5 dB, the model says
    -3.3 dB.
  - An overload snapshot at G2 read ~10 dB above the model.
  - Close in KEEL-1 (and V6/V7) computed steps with the wrong size: the
    hunting. Above G54 (one RF stage) the model holds.
- **Next:** measure every RF stage with the VTX at ~0.3 m and ~8 m, give the
  gain core a measured per-row table, and repeat the session.

## KEEL-1 on the board: what the logs showed and fixed (2026-10-19/20)

Blackbox sessions 24-34, walking with a VTX. Each item was found in the
per-second log or the new evidence records, then fixed at its cause.

| finding | cause | fix |
| --- | --- | --- |
| 255 writes/s at the edge | the 64-sample sentinel forwarded noise rail touches as overloads | gross saturation only |
| wrong gains, '255' seconds | AFC steps and refused writes reset the whole core; counters wrapped | keep core state on the same receiver; resync refused writes |
| rows with 2-3 cells DC after carrier changes | folding windows fed the DC servo | windows reaching both rails are kept out |
| gain 7-10 dB low at the edge | overloads with clean snapshots; a 9 dB step and 1 s memory each | no saturation evidence across a write; 3 dB steps; 300 ms memory |
| demod on its narrowest tracker with LP, no AFC at the edge | the envelope C/N read 3-12 dB low (snapshots: 14-19 dB true) | C/N from power against the learned noise per filter |
| VTX MATCHED screen in flight | status screen at the startup profile switch | the live switch, no screen |
| folds ~41 ms after a release (P50 ~55) | releases too optimistic for slow (walking) fades or a table step larger than the model | learned headroom per row: +3 dB per folded release, decays 1 dB / 2 s |

**Evidence in the blackbox now:**
- every step down with its window (FLOG_E_ATTACK);
- per second: writes down and up, sentinel forwards, overloads within 5 ms
  of a write, the received level and its noise;
- in T: refused writes, rebinds, heat events.

**Still open (needs a VTX):**
- The pair sweep at ~10 m to measure the real steps of G40-G83 (the model
  holds over G29-G39 only).
- A walk with this build.

## DC on every channel: folding, unconverged rows and templates (2026-10-20, PR #228)

Board, VTX off, all 48 channels surveyed 4 s after each switch (`T`:
RX_LEVEL clipped_pm, DC_SERVO dc). 33 of 48 channels had the DC on a rail
(3-6 cells). Four causes, each fixed at its root:

- **Folding, not clipping.** The ultrafine lane keeps the sign bit and folds
  the magnitude bits (`legacy/c5vrx3/main/rf.c`: +288 reads as +32). With a
  few cells of DC the mean of the folded cloud stops growing and then
  shrinks: every channel with a template 4+ cells off read a flat 3.0-3.5
  cells whatever the code. The servo learned a slope of the wrong sign
  there and walked to its limit (Q +188 codes, DAC flat end). The sign bit
  never folds: the observer now counts sign bits per axis (2 operations a
  byte), and the servo (`dc_servo_decide_sign`) learns and steps
  proportionally only where the sign share is 10-90 % (|DC| < 1.28 sigma,
  nothing folds); outside it takes full steps in the sign's direction,
  checks the polarity on the share and flips it on an expanding window.
  Host model with folding (`test_dc_servo.c`, 16 checks): 9 cells pinned
  with a lost slope ends at 0.15 cells (was 10.9-28 cells). Board A1: from
  3.4 cells at the trim limit to < 0.5 cells, trims -17/-16.
- **Unconverged search rows stored as OK.** A startup search from the chip's
  codes with 4+ cells of DC saw no slope (folded) and ran to its clamp
  (G83 on 5805 MHz: codes 316 = base+96, residual 3.5 cells); the row was
  stored and became every other channel's template. Now a row left with
  more than 1.2 cells is missing, stored tables drop such rows on load, and
  missing rows are searched again from a converged neighbour's codes
  (`phy_dco_search_from`, rows ~1 dB apart): 5805 G78-G83 all repaired,
  residual <= 0.3 cells, G83 225/186. A missing row holds the nearest
  converged row's codes.
- **Template choice.** The codes move with the frequency (G83 225/186 at
  5805 MHz, 144/279 at 5925: ~0.7 codes a MHz) more than with the filter
  (15-40 codes): `dco_nearest_slot` now minimises an expected code distance
  (0.7/MHz, +30 for another filter, +20 for a table with template rows)
  instead of "same filter first". Eviction drops the table with the most
  template rows first.
- **Channel change.** Urgent DC wakes skipped the periodic services, so the
  new channel's table was never loaded while one-sided clipping kept waking
  the servo; the servo also loads it itself now.

Template rows trim +-192 codes (own rows +-96), and a template row the servo
holds within 0.3 cells for 8 estimates on receiver noise becomes the
channel's own row (`DCO_ADOPT`): channels calibrate themselves in use.

## Range audit, first pass: the 2026-10-19 walk (session 34, PR #228 data)

`tools/edge_audit.py` on blackbox session 34 (build keel1h, before the
PR #228 DC and gain fixes), every second binned by the envelope C/N, plus
the ten raw edge IQ snapshots (M2M4 on the 4-bit samples):

- **The dropout seconds had the DC on a rail.** The 25 seconds at C/N
  bins <= 2 dB (23 dropouts) sat at 2.6-4.2 cells of DC; the seconds at
  4-20 dB at 0.13-0.26. That is the fold runaway of the DC servo fixed in
  PR #228; the edge of that walk was set by it, not by the RF.
- **No excess noise in the chain.** The snapshots' noise equals the
  startup noise map of their row (G71 0.69 vs 0.687 cells^2, G77 2.09 vs
  2.14): the digital noise is the receiver's thermal noise.
- **The gain was far too low at some moments.** G59 with a carrier of 0.66
  cells radius, G46 with 0.99, G71 with no carrier left: the carrier sat
  within about one 4-bit step, 10-20 dB of sensitivity unused at that
  moment (after attacks, during fades).
- **Two C/N readings disagree by ~7 dB.** In clean-DC seconds the video
  share fell to 35-56 % at an envelope C/N of 4-8 dB, while the level
  meter put the carrier at -86..-78 dBm over a -97 dBm noise floor (11-19
  dB). One of the two is wrong at the lower rows; C/N-AUTO (PR #228)
  replaces the envelope estimate and is checked on the next walk.
- The video share is ~75-89 % even on a strong signal, so it is a relative
  measure only.

## Range audit: static edge, restart gap, self-noise (2026-10-20, PR #230)

- **Static edge (VTX fixed, A1):** the picture fails at C/N -3..+3 dB on the
  20.7 MHz scale, carrier -90..-93 dBm over a -96.7 dBm noise floor (12.9
  MHz): about where the bit-exact simulation puts the U85Q+LP threshold
  (picture 0.03-0.25 at 0 dB, 0.95 at 2 dB). The 8-10 dB read on the walk
  was the old C/N scale plus walking fades. Rating: ~-92 dBm static,
  ~-88..-90 dBm walking (meter calibrated against Wi-Fi RSSI, +-1-2 dB).
- **Self-noise:** console N alternates the receiver noise at G83 with the
  video output (DAC, PARLIO TX) on and off: +0.00 / -0.04 dB over 8 pairs.
  The board does not raise its own floor; the floor is the chip's (noise
  figure ~6 dB from kTB in 12.9 MHz).
- **Restart gap:** T TRANSPORT_STAGES_US. Of ~5.5 ms without picture: LUT
  2.7-2.9 ms (2.2-2.5 with the delta load), program load 0.8, TX start 0.8.
  The IDF program load writes the program's own LUT in 32-bit mode first;
  a target LUT written into the program image would save the second pass.
- **Live LUT writes:** console H. The CPU readback returns nothing valid
  while the BitScrambler runs (1023-1024 of 1024 mismatch with the same
  values written), so a live kp switch cannot be verified; the halted
  restart stays.
- **Gain writes on a still signal:** -47 dBm, VTX and receiver still: 0
  writes, 0 overloads, sync 90 %. The walk's 10-29 writes a second close
  in came from walking fades.

## Antenna-to-pin budget and the gain table below G72 (2026-10-20, PR #231)

VTX 100 mW, whoop antenna (linear) at 2 m indoors, receiver Lumenier AXII 2
(RHCP) on the XIAO ESP32-C5 (U.FL straight to LNA_IN, no antenna switch).
Free space: +20 dBm + ~1 dBi - 3 dB (linear to circular) + ~1.6 dBi - 53.8
dB = about -34 dBm at the pin.

- The pair sweep on the carrier (console Q) measured **~1.0 dB a row** over
  G19-G31 (11.6 dB for 12 rows), as on noise at G72-G83 and the earlier
  G29-G39 sweep. The RF-stage/BB/fine model has steps of -3.3 and +2.7 dB
  there and is 4.2 dB short at G24.
- Corrected for it, the carrier read -38.7 -> about -34.5 dBm: **the
  antenna-to-pin path delivers what free space predicts (within ~1 dB)**.
  An earlier -46.5 dBm at "2 m" (G37, model error ~0.7 dB) was the
  placement indoors: 8 dB between two placements is ordinary multipath.
- Below the noise-measured rows the gain table now steps 1.0 dB a row
  (gain.c dg_apply_measured_gain), for the dBm meter and the KEEL-1 home
  rows.


## Single program-image LUT load (2026-10-20, PR #231)

The restart now prepares the selected LUT in an IDF V1 program image before
quiescing the pipeline. Its LUT storage replaces the previous work LUT; the
header/instruction prefix adds 300 bytes. The SDK writes the target once;
full instruction and LUT readback still gate publication. A mismatch enters
the existing bounded rollback, whose image restores the verified instructions
and transformed LUT (including output scaling and flywheel flag format).

Board COM10, VTX off, build from 270553e plus this change: four manual LP
switches took 3437, 3614, 3776 and 3238 us. Two kp switches took 4712 and
5327 us including target preparation and scheduling. Recorded LUT verify
stages were about 1195-1304 us versus 2463 us in the pre-change status;
these are small bench samples, not a controlled RF or visible-picture A/B.
All six switches left instruction/LUT verification, restart failures,
TX empty, RX overflow, DMA fault and BS overload counters at zero.
Gain returned to AUTO/G83; original kp_auto=2 restored after manual tests.
The counter kp_writes now includes all 1024 SDK table writes per successful
program load, rather than only subsequent corrective writes.

Validation: host C suite, runtime diagnostic suite (including actual
format/publication/rollback branches), research CI checks, ESP-IDF v6.0.2
firmware build and linked IRAM check passed. Read-only verification tests
inject corruption at the first and last entries, checking no hidden repair.
The format harness checks the selected image prefix on forward and rollback
paths for every supported kp/output/program combination. Physical restart
failure injection and an RF/goggle walk remain unmeasured.
