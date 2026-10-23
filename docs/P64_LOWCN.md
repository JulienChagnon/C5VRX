# Low-C/N P64 experiments, 2026-10-23

C5VRX by Twotoz and the C5VRX contributors. This extends PR #250 and the
U80P64-R1 correction at `dbc7cb3`; it uses the existing physical-ADC and
goggle models. It is simulation research, not measured RF sensitivity,
goggle acceptance or a firmware R2 release.

## Search and selection

`python research/p64_lowcn_search.py --seconds 900` ran for 900.50 seconds
and proposed 1,683 candidates. Of these, 486 failed the quick clean-tone
gate and 388 failed the good-signal gate. Remaining proposals can include
duplicate tables; see the complete compressed trace and checkpoint under
`research/data/p64_lowcn_search/`.

The search used seeds 731/732 for weak input and 733 for strong input.
Weak training points were -10, -6, -3, -1 and 0 dB PAL, plus -2 dB NTSC,
at nominal deviation. Four 20 dB PAL/NTSC cases covered deviation 0.7/1.4.
Fitness was mean weak-signal SINAD gain plus three times mean lock-fraction
gain, relative to R1 on exactly the same inputs. Each training weak case
allowed at most 0.25 dB SINAD loss or 5 percentage points of sync loss;
each strong case allowed 0.3 dB or 2 points. These sparse gates did not
guarantee generalization.

E1 was the highest-scoring eligible training candidate. It allocated 24
coarse and 40 fine states, instead of R1's 14/50 split. After it failed
validation, E2 was frozen as the best training candidate retaining the
original 14/50 split, then tested with a completely new seed set.
Neither candidate was installed in the firmware.

## Independent validation

Each candidate was checked against R1 and U85Q on identical inputs:
72 cases across PAL/NTSC, deviation 0.7/1.0/1.4 and C/N
-10, -8, -6, -4, -3, -2, -1, 0, 2, 4, 8 and 20 dB. E1 used seeds
9917/9918 and E2 used 30001/30002. Additional gain/CFO and noise-only
cases used the next two seeds. C/N is referenced to 20 MHz before the
12 MHz RX filter; ADC gain is fixed at 3 for the main sweep. Optional
demod LP and automatic kp are absent; the scoring chain includes its
normal modeled 5 MHz video filter.

| Candidate | Mean SINAD change below 0 dB | Mean sync change below 0 dB | Worst sync change at/above 0 dB | Accepted |
| --- | ---: | ---: | ---: | --- |
| P64-LowCN-E1 | +0.011 dB | +4.24 pp | -3.82 pp | No |
| P64-LowCN-E2 | -0.100 dB | +5.22 pp | -35.04 pp | No |

These are paired changes against R1 within each experiment; the two
experiments used different noise seeds. Means can hide individual weak
points. E1, for example, reduced PAL nominal-deviation lock at -1 dB from
89.7% to 52.8%, despite improving at -3 dB. Neither candidate recovered
line sync at -10 dB. Neither showed lock on the noise-only check.

Both passed 588 clean tones, twelve sync-to-porch transitions, the full
2,348-byte loader image and independent BitScrambler emulation. Thus
correct transfer and program loading alone do not demonstrate better
weak-signal reception. The validation gate also required no individual
0..20 dB case to lose over 0.3 dB SINAD or 3 sync percentage points.
Both failed that gate, so **U80P64-R1 remains the P64 firmware model**.

## Reproduction and artifacts

The JSON files contain frozen parameters, exact LUT hashes, per-case scores,
selection seeds and runtime. The `.bsasm` files embed the complete LUT and
reuse the existing eight-instruction P64 program: Q4/I4 RX40, two lookups
per pair, 64 persistent states, DAC20 duplicated at TX40, no CPU sample loop.

With the pinned ESP-IDF v6.0.2 assembler available in `IDF_PATH`:

```sh
python research/p64_lowcn_validate.py
python research/p64_lowcn_validate.py \
  --checkpoint research/data/p64_lowcn_search/original_grid_checkpoint.json \
  --out research/data/p64_lowcn_original_grid \
  --seed-base 30001 --model-id P64-LowCN-E2
```

Re-running the search overwrites its default checkpoint/trace; use `--out`
for a separate experiment. A next search should include multiple deviations
and the 0 dB transition in training, then reserve another fresh validation
set. Do not improve the reported validation scores by tuning on these seeds.
