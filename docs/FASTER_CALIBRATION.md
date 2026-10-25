# Faster, verified startup calibration

C5VRX by Twotoz and the C5VRX contributors. Extends the startup-only
calibration from PR #196, neighbouring-row DC repair from #228, and the
per-filter DC context fixes. No change to the IQ/video dataplane, stored
calibration meaning, live calibration policy or profile estimator thresholds.

## Changes

- DC: start the descending pass from an own, converged, unbanned neighbour
  in the same channel/filter context. Search with 16 windows, then independently
  measure the selected codes with all 64 windows. A centred start skips the two
  response probes. A fast residual above 0.3 cell vector magnitude, or a failed
  measurement, retries the full search. A failed/bad neighbour search retries
  the chip's starting codes. Existing 1.2-cell per-axis row rejection remains.
- Every matrix search, including the full fallback and carrier path, now
  reapplies the chosen codes and measures their actual final residual before
  restoring all owned PBUS words. No residual from another trial is stored.
- Low-gain flat DC: start with a local +/-8-code bracket, expand through the
  original +/-96 range if needed, and still bisect to adjacent codes on both
  axes in both cross-coupling rounds. A failed neighbouring seed retries the
  chip's codes. Original 8-window observations and final checks remain.
- Filter: visit all 130 physical settings; make no monotonicity assumption
  about the undocumented controls. Use 24 windows to screen only settings
  clearly outside all three class contenders (2.5 MHz width and 25% NBW
  guards). Extend contenders in place to all 96 windows. A screen/full
  discrepancy outside those guards restarts the entire grid without screening,
  including previously rejected settings. Recheck the baseline and independently
  remeasure each distinct selected filter with 96 windows. A failed winner
  recheck aborts storage and restores the old filter; exact channel/offset
  restore and hardware/NVS checks remain mandatory.
- Sampling phase: try three positions with the original 48-window precision.
  Only a complete scan with contrast and two agreeing observations at the
  selected position can finish early. Missing, ambiguous or drifting evidence
  falls back to the original nine-position scan. No live phase actuation added.
- Noise/gain map: reuse the noise statistics from accepted quiet DC verification
  (64 windows instead of another 24) only for the exact frequency/filter, within
  30 seconds, after the normal quiet/kurtosis gates. Missing/stale/foreign/noisy
  entries are measured normally. Run the normal-filter noise map directly after
  its DC pass, before visiting other profile filters. Cache is volatile, cleared
  for each startup/manual run, and occupies 720 bytes in RTC_FAST on C5.
- VTX deviation: shorten only the absent-VTX arrival grace from 20 to 2 seconds.
  Keep the 15-second measurement budget, three batches of 32 estimates, spread,
  agreement and fresh C/N gates. A later VTX is handled by the existing passive
  live matcher and its normal profile-switch safeguards. This changes the grace
  period; it does not make a deviation result require less evidence.

Board-proven 50 ms DC settling and two-tick collector yields are preserved.
The filter collector retains its existing one-tick yield. The full
8-minute startup budget remains a ceiling, not a promised duration. A difficult
board can deliberately take the exhaustive fallback and be slower.

## Evidence and board comparison

Host runtime tests execute the production search functions with simulated
hardware boundaries: coupled DC response, centred seeds, failed/noisy final
verification, local and full flat brackets, rollback, filter screening and
promotion, failed copies, carriers, phase ambiguity/scatter, exact noise-cache
context and expiry, and VTX arrival timing. The spectrum test compares 24 and
96 independent synthetic noise windows over 100 filter shapes; it checks that
true improvements at all three width boundaries survive screening. These
checks do not prove the guards for every physical chip or interference source.

Compare main and the PR on the same board, channel, temperature and VTX state:
repeat manual CALIBRATE with VTX off, cold boots with existing tables, and
VTX-on boots. Record `STARTUP done ms`, filter widths/NBW, per-row residuals,
missing/template rows and `DCO_READY`. Inspect any `screen=fallback_full`,
`fast_search=fallback_full` or winner-recheck failure. Confirm colour/detail,
line lock, no watchdog, and matched profiles on narrow and wide VTXs.
A controlled hardware comparison is still required for actual speedup,
filter-choice equivalence, residual distributions and RF/video quality.

## Timing model (scheduled waits only, not measured wall time)

At the configured 1 kHz tick, an old normal DC callback waits 50 + 64*2 =
178 ms. A short callback waits 50 + 16*2 = 82 ms; its independent final
verification still waits 178 ms. With K search observations the old search
therefore waits 178*K and the new one 82*K + 178 ms. For K=3/4/7 this is
534/712/1246 ms versus 424/506/752 ms (1.26/1.41/1.66x). A centred neighbour
needs just one short observation plus verification: 260 ms versus the old
minimum of three long observations, 534 ms (2.05x). Shared per-row overhead
reduces those ratios; avoiding iterations or repair passes increases them.
Flat DC uses about 42 observations for four full brackets versus about 26
when all four local brackets succeed, at the same 26 ms scheduled delay per
observation: about 1.62x for the search portion.

The old filter grid plus two baseline checks has 132*(20+96) = 15312 ms of
scheduled waits. Screening costs 20+24 = 44 ms per setting; promoting to 96
adds 72 ms. With P of 130 settings promoted and three independent winner
checks, the new waits total 6300 + 72*P ms: 8.64 s for 25% promoted (1.77x),
10.98 s for 50% (1.39x), 13.32 s for 75% (1.15x). No fallback is included;
with many contenders or a fallback, the filter sweep can be slower.

A successful quick phase search uses 3 initial + 1..3 selection + 1
confirmation observations versus 9 + 1..12 in the old scan. Failed quick
paths add work before falling back. Cache hits avoid a 20 ms gain delay
plus 24 one-tick observations; no-VTX grace saves exactly 18 seconds when
neither run sees a VTX. CPU computation, scheduling, status rendering,
failed probe copies, row count, filter winners and repairs are excluded.
These formulas cannot establish a total-calibration multiplier without
hardware logs and the actual path counts.
