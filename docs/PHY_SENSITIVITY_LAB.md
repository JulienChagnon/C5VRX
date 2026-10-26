# VTX-on PHY sensitivity lab (console `S`)

C5VRX by Twotoz and the C5VRX contributors.
[Canonical source](https://github.com/Twotoz/C5VRX);
[official website and Discord invite](https://twotoz.github.io/C5VRX/).

This extends the PR #254 VTX-off PHY noise lab
([findings](PR254_PHY_FINDINGS.md); frozen lab code `main/phy_noise_lab.c`,
`tools/phy_noise_lab.py` and `docs/PHY_FEATURE_AUDIT_2026-10-24.md` at
`6274b2d0d51b44fbf7b9bef1cc8fb71bb2470e85`) and the pinned PHY lab in
`legacy/c5vrx4/main/phy_rx_lab.c` (#150-155). Every register footprint used
here is PR #254's recovered write mask of the pinned IDF 6.0.2 `libphy.a`
(SHA-256
`dbf33c418c8d408d4005c849d12a1432deea82e2e5e57de3c8ddf914d104fffb`); none is new reverse engineering. The 11p
observation is credited to SushiDude (@Ready4Sushi).

**Status: firmware, host tool and host tests only. No board run yet, so no
candidate is BETTER or WORSE today.** This document is the candidate
inventory, the measurement design and the test plan.

## 1. Why another lab

PR #254 measured captured noise variance with the VTX **off**. Its own
conclusion: a lower noise variance can come from a stage that also lowers
the wanted carrier, so none of those deltas is a sensitivity result, and
the next step needs a carrier, "carrier response and noise together".

This lab runs the same kind of reversible PHY settings with a **weak VTX
carrier present** and scores, per capture, what reaches the demodulator:

| Metric | What it is | Why it answers the question |
| --- | --- | --- |
| `cn_db` | M2M4 carrier-to-noise from the second and fourth moments of the IQ (constant-envelope FM in Gaussian noise: S = sqrt(2 M2² − M4), N = M2 − S) | A setting that scales carrier and noise together leaves it unchanged; a real sensitivity change moves it |
| `video_snr_db` | Demodulated video SNR of the existing AFC decoder (`afc.h` `afc2_snr_x10`): sync-to-blanking distance over the spread of the instantaneous frequency on the sync tips and back porch | What the goggles get after FM detection, independent of picture content |
| `click_pm` | Adjacent-sample phase steps above ¾π, per mille (the largest FM step is 1.41 rad) | FM threshold clicks: the sparkles and lost syncs at the range edge |
| `video_cov` (tool) | Share of captures in which the decoder found syncs | A candidate that loses sync cannot hide behind better C/N |
| `origin_pm`, `rail_pm` | Samples in the four origin cells / on a signed-nibble rail | Diagnostics: weak signal / folding risk |

Host model (`test/test_sense_metrics.c`, synthetic NTSC-like FM, 4-bit
quantised, 16 captures per point):

| True C/N (dB) | M2M4 `cn_db` | `click_pm` | `video_snr_db` |
| --- | --- | --- | --- |
| 3 | 2.3 | 32 | −2.9 (11/16 captures with syncs) |
| 6 | 4.8 | 6.4 | −0.2 |
| 9 | 7.1 | 0.5 | 2.8 |
| 12 | 8.8 | 0.02 | 5.7 |
| 15 | 9.9 | 0 | 7.8 |
| clean | 10.6 | 0 | 12.3 |

**The 4-bit IQ saturates the scores near 10-11 dB.** Comparisons therefore
need a weak carrier, about 2-8 dB `cn_db`, where all three metrics still
move. Absolute values are biased (quantisation of a deterministic carrier,
analog-filter AM); only STOCK-to-TEST differences at the same input count.
The simulation is host evidence, not a measurement of this receiver.

## 2. Candidate inventory

All candidates are masked writes saved before and restored after, or the
existing gain/output APIs. Results columns are PR #254 (VTX off, noise
variance only).

| Suite | Candidates | Origin (pinned libphy) | PR #254 VTX-off result | Why test it with a carrier |
| --- | --- | --- | --- | --- |
| `1` NF-auto | 16 raw patterns of 0x600A7018[23], [28], 0x600A7C44[0], 0x600A7C50[0] | `phy_noise_floor_auto_set` | Patterns 0..7 small and inconsistent; 8..F did not latch | A packet noise-floor estimator may still gate or scale continuous IQ |
| `2` IQ correction | RX enables 0x600A0438[29], [30], [31], all; TX enables 0x600A0C0C[13], [14], both; RX coefficient fields A/B raw zero; all five enables | `phy_iq_corr_enable`, `phy_rxiq_set_reg`, `phy_txiq_set_reg` | RX enables off +0.74/+0.80 dB more noise; others small | An IQ transform changes image rejection and carrier amplitude, which a noise-only test cannot see; FM is phase-sensitive |
| `3` Digital filter | Modes 4, 8, 12 against production 0 at 0x600A0430[21:18] | `phy_rx_filter_mode` | Mode 0 lowest noise (+0.04..+0.73 dB for the others) | A narrower digital response may also cut deviation; only a carrier shows the trade |
| `4` Packet/RX policy | Channel helpers A/B (both arguments), PKDADC enable, BB CCA, MAC CCA on/off, FFT force off / scale 0, RX11B opt (both), RX sense (both), LLTF masks 00/10/01/11, RIFS 0/1, VHT 0/1, 11b-LR 0/1, HEMU RU26, PWDET clear | `phy_lltf_mask_en`, `phy_wifi_rifs_mode_en`, `phy_vht_support`, `phy_rx11blr_cfg`, `phy_rx_11b_opt`, `phy_rx_sense_set`, CCA/FFT/HEMU/PWDET helpers | No reproducible noise benefit; HEMU/PWDET already at vendor targets | The packet engine may react to a carrier (AGC-like gating, detection resets) where it did not react to noise |
| `5` Clocks | TX clock enables 0x600A0890[17:16] off/on; Wi-Fi enable 0x600A9C18[1] 0/1 | `phy_set_txclk_en`, `phy_wifi_enable_set` | In the PR #254 lab (its suite 5); no result retained in its findings | Stopping an unused TX clock could lower self-interference next to the antenna input |
| `6` Analog | ADC selector 0/1 (analog 0x66/0/4[2] and 0x600A0448[1:0]); LO-cap code ±1 (0x63/1/21[4:0], vendor range 10..31); ULP active code = the chip's own reference (0x61/0/6, only within one code); spur slot 0 off when active | `phy_adc_rate_set`, `phy_get_dcap_degen`, `phy_ulp_ext_code_set` | Selector clue did not separate from controls; LO-cap neighbours comparable to shams; ULP not measured | LO drive and converter clocking change conversion gain and phase noise, so carrier and noise both move |
| `7` Analog filter | RX0 capacitor code (0x67/1 regs 6/7 low six bits) −2, −1, +1, +2 around the live code; regs 8..13 untouched | ESPARGOS filter register map used by `phy_filter_set_code` | Calibration already picks the lowest noise bandwidth per class | With a real VTX the filter trades noise bandwidth against deviation clipping: the most direct sensitivity knob |
| `8` Gain rows | TEST at gain −6, −3, +3, +6 from the run gain (rows with DC correction only) | Existing gain table and `receiver_set_gain` | Higher gain raises output noise, not input-referred | Tests whether Direct Gain's row choice costs C/N at the edge (RF vs BB stage) |
| `9` DAC output off | Video output stopped vs running | `transport_stop_tx` | Console `N`: +0.21 / −0.06 dB, not reproduced | Self-interference from the 40 MHz DAC next to the receiver, now against a carrier |

Every suite starts and ends with a **sham** (STOCK twice, no write) and has
one after every six candidates. Candidates already equal to stock run as
no-ops (`noop=1`) and are reported, not scored.

### Not in the lab, and why

| Function / area | Reason |
| --- | --- |
| 11p off | The vendor's pre-11p MMIO values were never witnessed; `phy_11p_set(0,0)` is not rollback. The 11p analog part is covered by suite 7 |
| Native AGC (`phy_agc_max_gain_set`, `phy_wifi_agc_sat_gain`) | Unmasked full-word writes; native AGC caused line noise on a continuous carrier (`legacy/c5vrx3/docs/native-agc-v2.md`) |
| `phy_get_loop_snr`, `phy_set_adc_rand` | Change clocks/PBUS, start an internal TX tone or run DC calibration; not reversible toggles |
| Full RF calibration, PLL/temperature tracking | Boot-level maintenance that regenerates gain tables, DC and IQ; needs its own boot comparison |
| `phy_bias_reg_set`, `phy_current_level_set`, `phy_rx_loop_cap_set`, `phy_rxevm_en` | Owned by frequency calibration, no live consumer traced, or a reset routine without exact rollback (PR #254 audit) |
| Spur slots 1..3 | Slot semantics and coefficients unverified |

## 3. How a run protects the receiver

- Pinned PHY only (`C5VRX_PHY_PINNED`); any other libphy refuses.
- Refuses during the startup calibration, menu, scan, another bench test,
  or without confirmed video sync within the last 3 s (**VTX must be on**).
- One PHY transaction for the whole run and the gain observer paused: no
  AFC, gain, DC servo, filter or tuning write can land between STOCK and
  TEST.
- Fixed gain: the highest row at or below the live gain with a calibrated
  DC row and no rail pile-up (`rail_pm` ≤ 2). Every gain write in the lab re-holds
  that row's calibrated DC pair explicitly (the post-gain hook is silent
  while the receiver is paused). The DC pair, frequency,
  offset, physical RF/BB/fine tuple, filter bytes and the profile's own
  bits are checked before and after every capture.
- A TEST write that does not read back is `REJECTED_ENTRY`; a TEST setting
  that stalls the IQ is `BREAKS_IQ`. Both are restored and the run goes on.
  Any other context change ends the run after restoring.
- Restore is verified; a failed restore prints `SENSLAB_FATAL` and reboots
  before the receiver resumes. Nothing is written to NVS.
- The video output keeps running (except in suite 9), so you see each
  setting live in the goggles.

## 4. Test plan

### Equipment and setup

- The C5VRX board on USB (power the goggles separately or from the bay as
  usual), a PC with Python 3 and `pyserial`.
- A VTX at fixed low power (25 mW), on a fixed, known channel, with a
  static camera image (a test card or a still scene; avoid moving content).
- Fixed geometry: board and VTX on a table, nobody moving in between
  (people and doors change the path by several dB). Antennas fixed.
- **Weak signal**: increase distance, turn the VTX antenna away, or (best)
  use a fixed attenuator, until the picture shows light snow but holds
  sync. The run's `baseline_cn_db` should be about **2-8 dB**;
  `SENSLAB_WARN signal_strong` means the scores are saturated.
- The firmware must have finished its startup calibration (`STARTUP done`).

### Procedure

1. Flash the build, boot with the VTX on, wait for `STARTUP done` and a
   stable picture. Check `T`: correct channel, `afc_locked=1`.
2. **Method check**: run the shams only. Note the sham spread printed by
   the tool; with 16 captures per stage it should be a few tenths of a dB.
   ```sh
   python tools/phy_sense_lab.py record COM10 runs/5805_sham1 --suite 0
   ```
3. **Full sweep**: about 5-10 minutes. Do not touch anything.
   ```sh
   python tools/phy_sense_lab.py record COM10 runs/5805_all1 --suite a
   ```
4. Repeat step 3 at least twice more (fresh directory each time), then on a
   second channel (5865 MHz). Optionally a third at a different C/N (a bit
   closer or further).
5. Combine:
   ```sh
   python tools/phy_sense_lab.py analyse runs/*/console.log --json runs/summary.json
   ```
6. Single suites (`--suite 2`, `7`, ...) can be repeated to confirm a
   candidate without the full sweep.

Without the tool: press `S`, then the suite key within 5 s, in any serial
terminal, and save the log.

### Reading the result

Per run, each candidate gets one verdict:

| Verdict | Meaning |
| --- | --- |
| `BETTER` / `WORSE` | At least one metric moved beyond max(3 SE, 2 × sham RMS, floor) and none moved the other way. Floors: 0.10 dB `cn_db`, 0.20 dB `video_snr_db`, 0.05 ‰ clicks, 0.10 video coverage |
| `MIXED` | Metrics disagree (e.g. more C/N but more clicks) |
| `NO_EFFECT` | Within the comparison's own spread |
| `UNSTABLE` | STOCK before/after or the two TEST blocks differ by more than the threshold (fade, movement): repeat |
| `REJECTED_ENTRY` / `BREAKS_IQ` / `INCOMPLETE` | The write did not latch / the setting stalled the IQ / the run stopped |
| `NOOP` | The candidate was already the stock state |

Across runs, `REPRODUCED_BETTER` needs `BETTER` in at least two runs and
never `WORSE` or `MIXED`. **One run is a screening result.** Do not pick
the best of many candidates from one run: with about 80 candidates a few
will cross a 3-sigma line by chance.

### Acceptance ladder

1. Screening (this lab): `REPRODUCED_BETTER` on two channels, at least
   three runs each, sham spread small, no `UNSTABLE` in those runs.
2. Threshold test: with a step attenuator, find the attenuation where the
   picture loses lock, stock vs candidate, ABBA, several times. A real gain
   moves this point.
3. Video check: PAL and NTSC, colour, detail, loaded 75 Ω CVBS, and repeated
   cold boots with the candidate applied at startup.
4. Field: only then a separate production PR, with its own restore, and
   the evidence recorded in `docs/FINDINGS.md`. A `WORSE` result is kept
   as a finding too.

## 5. Console records

```
SENSLAB_BEGIN fw=… suite=a freq=5805 offset_khz=… gain=74 entry_gain=76 packed=… dco=…/… held=1
  filter_code=… skirt=… dfilt=0 p11=1 captures_per_stage=16 dwell_ms=300
  baseline_cn_db=4.80 baseline_click_pm=6.100 baseline_video_snr_db=-0.30 video_caps=16/16 …
SENSLAB_PROFILE id=203 name=IQ_RX_0438_ALL_OFF kind=patch patches=1 noop=0 sequence=STOCK_TEST_TEST_STOCK
SENSLAB_PATCH id=203 i=0 mmio=600a0438 mask=e0000000 stock=e0000000 test=00000000
SENSLAB_CAP id=203 step=0 stage=STOCK rep=0 gain=74 cn_db=… click_pm=… origin_pm=… rail_pm=…
  video_lines=… video_snr_db=… m2=… kurt=… dc=…/… fnv1a=… t_ms=…
SENSLAB_APPLY id=203 verified=1
SENSLAB_RESTORE id=203 verified=1
SENSLAB_RESULT id=203 name=… status=PASS d_cn_db=… se_cn_db=… d_click_pm=… d_video_snr_db=… drift_cn_db=… …
SENSLAB_END status=COMPLETE profiles=… rejected=… breaks_iq=… skipped=… restored=1
```

The firmware's `SENSLAB_RESULT` is a quick view; the verdict comes from the
tool, which also uses the shams. Raw IQ is not exported; each capture's
FNV-1a hash identifies it, and console `I` still exports single snapshots.

## 6. Evidence limits

- One weak-signal level per run: a result is a screening result at that
  level, not an input-referred sensitivity, noise figure or range claim.
- M2M4 assumes a constant-envelope carrier. Filter AM (suite 7) and
  multipath also read as noise; there the video SNR and clicks carry more
  weight.
- The shams measure the method's spread in that run; a slow fade within a
  profile shows as `UNSTABLE`, not as a candidate effect.
- Register names describe the recovered writes, not documented semantics.
- Host tests: `test/test_sense_metrics.c` (metric behaviour on synthetic FM)
  and `test/test_sense_lab_tool.py` (verdicts, combining runs, record format
  against the firmware source).

## 7. Board results (2026-10-26)

First board runs at 5865 MHz, recorded in `docs/FINDINGS.md` ("VTX-on PHY
lab, AFC window and digital mode 12") with the logs in
`research/data/vtx_on_lab_2026-10-26/`.

- **Stay away from the bench during a run.** A person within ~1-2 m moved
  the carrier 4-11 dB (sham spread 2.7-2.9 dB C/N); with nobody near it
  held +-0.1 dB and the sham spread fell to 0.12-0.21 dB. Start the tool,
  leave, and come back after the `SENSLAB_END` time.
- `no_vtx_video` refusals came from the AFC decoder finding too few syncs
  in one descriptor (fixed in the same change); retry the suite.
- Mode 12 reproduced BETTER; no other candidate repeated. The sweep is
  one level on one channel.

## Console compatibility with live phase repair

Console `S` followed by a suite key selects this lab. The standalone RX
sampling-phase slip introduced in PR #272 is now `!`; it does not start a
lab or wait for a suite key. `T` still reports the phase state. Older bench
scripts that sent `S` for a single slip must send `!` on this firmware.
