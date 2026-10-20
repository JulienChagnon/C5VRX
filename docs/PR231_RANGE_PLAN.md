# PR #231: range investigation and implementation plan

C5VRX by Twotoz and the C5VRX contributors.

Operator direction, 2026-10-20: prioritize range and substantially improve
video quality. Target usable video at **-100 dBm at the antenna input or
weaker**. Becoming competitive with/better than Rapidfire is the ambition,
not a measured result. Use the extensive existing IQ, blackbox and sweep
data first. Commit completed steps regularly. Do not equate longer sync
hold with usable picture, a corrected meter with sensitivity gain, or
simulation with a physical RF result.

## Work order

1. **Flywheel, ladder and LP loss audit.** Correlate recorded restart events
   with dropout onsets and recoveries, preserving firmware/session context.
   Inspect every restart trigger and flywheel source-validity/repair gate.
   Distinguish restart cost, wrong repair, CPU contention and genuine RF
   loss. A coincident event is not by itself causation.
2. **Low-gain acquisition.** Replay the actual session 55 IQ through gain
   measurement/control. A constant origin code was not a faithful replay.
   Inspect DC correction, sample validity, PHY ownership and rejected
   writes before choosing a fix. Do not restore the earlier unconditional
   fixed-IQ-to-max rule: it caused a documented overload/DC oscillation.
3. **Detector and filtering.** Compare bit-exact U85Q/kp/LP against candidate
   estimators on identical IQ. Score sync, luma error and retained detail
   separately; include narrow/wide deviation and physical ADC slicing.
   Use recorded IQ/noise/filter data where available. Establish state,
   throughput and DAC costs before proposing a production replacement.
4. **Implement justified changes separately.** Regression tests, appropriate
   VTX-off board checks, small commits on this PR. Preserve last verified
   transport rollback and live source ownership. Do not turn a control
   improvement into an unsupported dB claim.
5. **Maintain a loss budget.** Separate observed losses, simulated effects,
   hypotheses and unresolved RF/calibration uncertainty. Use existing data
   before requesting new measurements. Absolute -100 dBm sensitivity and a
   competitor comparison ultimately need a common input and picture criterion.

## Completed checkpoint

- `eb08b90`: target LUT loaded once in the program image, with full readback
  and verified rollback. Four VTX-off LP switches: 3.24-3.78 ms; two kp
  operations: 4.71/5.33 ms including preparation/scheduling. No transport or
  verification errors. Host/runtime/research checks and IDF build passed.
  See FINDINGS.md for evidence limits. This reduces interruption; it does
  not change the steady-state RF sensitivity.
- Board G24 -> AUTO returned to G83 on the current firmware. Session 55
  now reproduces a gain-policy park from sparse outliers in an origin cloud;
  a targeted correction passes recorded-IQ regression tests. Physical
  weak-VTX recovery and the cause of the code outliers remain open.
- Initial physical-ADC simulation: a float PLL improves line retention
  but does not uniformly improve luma SINAD over U85Q-LP. This is a
  candidate study, not an information-theoretic bound.

- Restart/dropout audit saved: 56 sessions, 368 switch events, 54 dropouts;
  27 dropouts have a preceding switch within 2 s. Temporal association only;
  no matched flywheel on/off result. See FINDINGS.md and the audit JSON.

## Corrections to earlier conclusions

- The recorded sensitivity model explicitly **assumes** NF (default 5 dB);
  it does not measure receiver noise figure. Do not call ~6 dB a proven
  hardware value.
- Indoor agreement between a corrected meter and free-space prediction
  does not establish an independent 0 dB antenna/front-end loss bound.
- A short PLL comparison cannot establish that the detector is exhausted.
- The current evidence does not demonstrate -100 dBm, or superiority to
  Rapidfire, and does not rule out other range losses.
