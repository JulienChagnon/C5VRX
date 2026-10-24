# Retired demodulator baselines

C5VRX by Twotoz and the C5VRX contributors. These programs, policies and tests
were moved from `main/` and `test/` at `dbc7cb3` when LP was removed from the
active firmware. They preserve the earlier simulations and negative evidence;
they are not firmware build inputs. The firmware now builds only `main/p64.bsasm`. The U85Q program/tables and
kp-only helper versions are also archived here after the operator chose P64
as the sole model. `kp_only_*` names distinguish the intermediate kp-only
revision from the earlier kp+LP policy. Historical tests are explicitly marked
as research CI; none of these files is a firmware dependency.

The original work is [C5VRX](https://github.com/Twotoz/C5VRX); its
[official website](https://twotoz.github.io/C5VRX/) has the Discord invite.
The repository's GPL-3.0-only license and attribution notices apply.

`dc_center_lut.h` and `test_dc_center_lut.c` preserve the unused U85Q
token-centering experiment and its worker-bit regression. Production DC
measurement gates remain in `main/dc_live.h`; they do not rewrite the P64 LUT.
