# PR #254: retained PHY research findings

Knowledge-only extraction on 2026-10-25 from [PR #254](https://github.com/Twotoz/C5VRX/pull/254),
reviewed at `6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85`. No lab firmware, console commands,
tools, CI changes, generated inventories or measurement archives are imported.
Main remains the production authority; descriptions of branch experiments
below do not imply that those experiments are available in main.

C5VRX by Twotoz and the C5VRX contributors.
[Canonical source](https://github.com/Twotoz/C5VRX);
[official website and Discord invite](https://twotoz.github.io/C5VRX/).
This extends the earlier C5VRX-3/4 PHY work; the original 11p observation
is credited to SushiDude (@Ready4Sushi).

## Scope and interpretation

The reports describe one USB-connected ESP32-C5 rev1 board, VTX off,
at 5805 and 5865 MHz, using IDF 6.0.2 and pinned libphy.a SHA-256
`dbf33c418c8d408d4005c849d12a1432deea82e2e5e57de3c8ddf914d104fffb`.
The path was fixed ultrafine signed Q4/I4, PARLIO RX40, raw32K,
U80P64-R1, DAC20 at TX40 and digital filter mode0.
The bench antenna/termination and external RF environment were not
independently established. These are reviewed reports, not measurements
rerun during this extraction.

Within-boot candidates used STOCK–TEST–TEST–STOCK at fixed gain/context,
300 or 1200 ms settling and separately contiguous 8190-byte IQ snapshots.
Four snapshots per stage were used initially, sixteen in larger follow-ups.
Writes/readbacks and saved-state restoration were verified.
Positive noise deltas mean higher DC-centered complex IQ variance.

**Lower captured variance is not proof of better noise figure, input-referred
C/N, video SINAD or range.** A transfer/gain change can reduce wanted signal
as well as noise. No calibrated weak carrier, attenuation threshold or
75-ohm-loaded CVBS quality was measured. Do not add selected small negative
deltas from separate tests as if they were independent sensitivity gains.

## Useful measured results

| Area | Retained observation | Consequence |
| --- | --- | --- |
| 11p helper (1,0) | Narrower effective noise bandwidth in all five original paired runs. 5865 MHz variance deltas were -0.291/-0.392/-0.223 dB; 5805 MHz -0.339/+0.105 dB. | Most promising noise-bandwidth result; the +0.105 dB run had an impulsive capture and 0.599 dB test-block disagreement. Retain flags and all samples. |
| NF-auto patterns | Patterns0..7 gave small inconsistent changes; patterns8..F failed entry because 0x600a7c50[0] did not latch. | No reproducible benefit; a rejected entry is not a measured operating mode or proof the bit is universally read-only. |
| Channel A/B | Approximately -0.09..+0.06 dB with changing signs in the first comparisons. | No universal improvement established. |
| RX IQ enables | Clearing all three 0x600a0438[31:29] bits gave +0.742/+0.796 dB at 5865/5805 MHz with the stronger headroom screen. | Keep existing calibrated correction; this is not a new gain over stock. Desired FM phase/image/color behavior was not measured. |
| TX IQ enables / RX coefficients | Isolated TX enables and zeroed RX coefficient fields had much smaller, inconsistent effects. | The original combined RX/TX disable result cannot all be attributed to RX calibration. Zero coefficients are not a demonstrated identity transform. |
| Digital filter | Modes4/8/12 increased noise by +0.241/+0.402/+0.733 dB at 5865 and +0.310/+0.038/+0.403 dB at 5805 relative to mode0. | Mode0 is already part of stock, not an extra gain supplied by this PR. |
| ADC selector | At 5865, selector0 initially gave -0.142 dB, then -0.079/-0.040 dB; unchanged selector1 controls also moved +0.055/-0.060/-0.082 dB. | Initial clue did not clearly separate from controls; preserve vendor selection. |
| Packet ADC/CCA, FFT, RIFS/VHT/11b-LR | No reproducible additional noise benefit. FFT_FORCE_OFF and several policy profiles already matched the baseline. | No-op deltas measure variation, not an intervention. |
| LLTF00 | Early 5805-only reductions failed the independent order-controlled test: pooled -0.006 dB at 5805 and -0.002 dB at 5865. | No confirmed extra gain; preserve the negative confirmation. |
| Joint controls | On the integrated 11p baseline, sense/RX11b combinations gave -0.010/+0.001 dB at 5805 and +0.001/-0.037 dB at 5865. | No robust extra 0.4 dB or additive improvement. Historical NO11P names mean no additional 11p write, not necessarily an 11p-off baseline. |
| Output self-noise | Output OFF minus ON gave +0.210 dB flagged at 5805 and -0.064 dB at 5865. | No reproducible operating-receiver improvement established. |
| Full RF calibration | Partial–full–partial boot comparison at G78: -0.135 dB at 5805 and -0.083 dB at 5865; G72 gave -0.050/+0.093 dB near quantization limits. | Same index after calibration need not mean identical physical gain. Original partial policy restored; no universal extra sensitivity gain. |
| Adjacent LO-cap codes | At stock code12/5805 and code10/5865, neighboring codes produced small changes comparable to shams and opposite-sign repeats. | No basis for overriding the vendor formula. |
| HEMU/PWDET | Both tested fields already matched their vendor targets on both channels. | These were no-op controls, including the apparent -0.050 dB HEMU result. |
| ULP reference trial | Read-only witnesses and a bounded experiment were implemented; the reviewed report contains no measured result rows. | Pending, not evidence of a supply or sensitivity benefit. |

The separate operator-selected integration of 11p is already documented
in [FINDINGS.md](FINDINGS.md) under "11p receive baseline".
This extraction neither changes that integration nor establishes its RF acceptance.

## Gain and DC: preserve the correction to the earlier diagnosis

The old forced-gain sweep reported G83 signed-rail occupancy of 4.676% at
5805 and 6.471% at 5865 MHz. Those probes paused the receiver and bypassed
the normal post-gain DC hook, leaving vendor DC rather than the corrected
live state. They must not be cited as production DC performance.

With the ordinary DC servo running, 32 fixed-MAX captures per frequency
gave mean signed rails of 0.0275% at 5805 and 0.0244% at 5865,
with mean centering within 0.07 cell per axis. Same-gain calibrated DC
searches could improve an off-center starting pair but did not establish
repeatable extra noise reduction: complete G83 deltas were approximately
-0.005..+0.009 dB. Signed rails are a headroom proxy, not a direct count of
folding errors in the sign-preserving ultrafine slice.

Read-only physical tuples showed RF stage8/code487 at G73/G74; the highest
RF stage begins at G54 and the table maximum is G83. There is no evidence
here of an accidentally low RF stage compensated solely by BB gain.
Higher output noise at G83 also reflects higher gain, not automatically
worse input-referred SNR. These data do not justify lowering the
no-carrier maximum to G80, nor prove G83 optimal for every carrier.

## Recovered PHY details worth keeping

These are findings for the exact pinned archive, not stable public APIs.

- NF-auto owns 0x600a7018[23], [28], 0x600a7c44[0] and
  0x600a7c50[0]: sixteen raw bit patterns, not sixteen documented modes.
- `phy_11p_set(1,0)` changes coupled analog and MMIO state.
  Exact rollback requires both saved phy_param flags, eight analog bytes
  and four recovered MMIO masks; `phy_11p_set(0,0)` is not exact rollback.
- `phy_iq_corr_enable()` enables RX bank 0x600a0438[31:29] and TX bank
  0x600a0c0c[14:13]. RX coefficients occupy 0x600a0438[28:16];
  TX coefficients occupy 0x600a0c0c[12:0].
- `phy_rxiq_scale_set()` uses selector phy_param[650] and 0x600a043c.
  The measured selector/register were zero; no calibrated dB scale or
  useful MODEM_DIAG override was established.
- `phy_adc_rate_set(selector)` couples analog block0x66/host0/reg4[2]
  to !selector and 0x600a0448[1:0] to duplicated selector.
  The observed vendor selector was0 at5805 and1 at5865.
  RX40 metadata does not independently measure the RF ADC clock.
- Verified vendor-used spur slot0 at0x600a7c14 had enable bit13 clear.
  A set bit in adjacent0x600a7c18 does not prove a valid active slot1;
  nonzero-slot bounds/coefficient semantics remain unverified.
- `phy_bias_reg_set` initializes block0x6a/host1 reg0/reg1 to0x9f/0x7f,
  but operating witnesses were0xcf/0x7f. The whole frequency setter writes
  high nibble12 above4000 MHz; frequency calibration also owns that nibble.
  Resetting it to initialization is not an established LNA-noise tweak.
- LO-cap block0x63/host1/reg21[4:0] follows the recovered 5GHz formula
  10 + trunc((5880-f)/36), clamped10..31: code10 at5865,12 at5805.
  Code magnitude is not calibrated capacitance, RF gain or noise figure.
- `noise_check_loop`, `phy_get_adc_rand`, `phy_pwdet_always_en` and
  `phy_pwdet_onetime_en` are single-return stubs in this archive.
  `phy_set_adc_rand` instead couples clocks, PBUS, power and DC calibration;
  its off branch is not saved-state rollback or a safe dither toggle.
- `phy_get_loop_snr` changes clocks/PBUS/gain, calibrates DC and starts an
  internal TX tone. It is not an antenna-input C/N measurement.
  Native-AGC helpers with broad/unmasked writes are not bounded extra
  gain knobs for the forced-gain receiver.
- LLTF masks own0x600a790c[13:12]; RIFS owns0x600a70f4[0];
  VHT owns0x600a7ce4[5] beside 11p fields; 11b-LR couples
  0x600a8060[11:10] and0x600a807c[11]. Preserve adjacent state.
- ULP block0x61/host0 reference reg4=0x58, active reg6=0x57,
  reg5=0x47 and caches phy_param[652:653]=0 were witnessed.
  The code-only helper writes reg6; initialization/tracking also owns
  enable/cache state. One code has no established voltage or sensitivity scale.
- Effective configuration disabled PM, software coexistence and automatic
  PLL tracking. A coexistence component macro is not evidence of active
  arbitration. Automatic PHY maintenance can power down RX/TX, recalibrate
  DC/IQ and regenerate gain tables outside the application PHY lease;
  enabling it is not an isolated PLL-noise switch.
- Additional names such as current-level, loop-cap, RX-EVM and ULP helpers
  require consumer/side-effect/restore analysis before use; a suggestive
  symbol or linked definition is not evidence of runtime activity or benefit.

## Measurement lessons and next acceptance gate

Use identical physical gain and a matching held calibrated DC row, verify
actual frequency after startup/channel restore, and preserve full entry and
outer gain/DC restoration. Early rail screens admitted later distortion;
the stronger screen required zero observed rails and abs(mean)+6*sigma <=6
cells per axis across four qualification captures, with m2 >0.67 cells².
This is a diagnostic screen, not a universal RF optimum.

USB exports lost bytes in several profiles. Reject incomplete or corrupt
captures; never synthesize missing samples or score an incomplete profile.
Bounded ACK retries must resend the same timestamp/token/raw bytes rather
than substitute a later noise window. Keep all integrity-valid outliers,
quality flags and failed/no-measurement attempts.

Noise bandwidth was complex Welch PSD (1024-sample Hann,50% overlap),
integrated and normalized by mean PSD at +/-0.5..2MHz. That normalization
is not a calibrated RF passband. Raw transport integrity and successful
rollback do not imply clean noise or sensitivity acceptance.

Unchanged-state controls themselves varied: NO_FLAGS absolute-delta p95
was0.099 dB for16 captures/control,0.044 for32 or64, and0.026 for128.
These mixed-context descriptive values are not confidence bounds for a
particular intervention. They explain why cherry-picked hundredths of a dB
need repeated shams/order controls.

The reports retain1411 initial noise captures,7871 extended-audit captures,
64 live-MAX witnesses and192 calibration-policy captures. Raw IQ/logs/PSD
were local ignored artifacts; their hashes and derived summaries are in the
PR archive. Raw data availability must not be implied by linking a summary.

Before claiming improved reception, compare a fixed calibrated weak RF
input with an attenuator: carrier response and noise together, input-referred
C/N, equivalent usable attenuation, P64 video SINAD, H/V lock, chroma/detail
and loaded CVBS on PAL and NTSC, across frequencies and VTX deviations.

## Frozen sources

- [Original NF/11p measurements](https://github.com/Twotoz/C5VRX/blob/6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85/docs/NF_PHY_BENCH_2026-10-24.md)
- [Extended audit, corrections and controls](https://github.com/Twotoz/C5VRX/blob/6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85/docs/PHY_AUDIT_BENCH_2026-10-24.md)
- [Calibration boot comparison](https://github.com/Twotoz/C5VRX/blob/6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85/docs/PHY_CALIBRATION_BENCH_2026-10-24.md)
- [Static helper/configuration audit](https://github.com/Twotoz/C5VRX/blob/6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85/docs/PHY_FEATURE_AUDIT_2026-10-24.md)
- [Historical protocol and reversible-state requirements](https://github.com/Twotoz/C5VRX/blob/6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85/docs/NF_AUTO_LAB.md)

Follow those exact-revision sources for detailed tables, masks, flags,
firmware identities and archived summary links. The old bench instructions
refer to the experimental branch, not console functionality added to main.
