"""C5VRX by Twotoz/contributors: reproduce the PR #250 P64 and P64-R1 tables.

Adapted from research/p64_optimizer.py at 9e4a616; optimization and benchmark
scores are simulation evidence. 7.2 degrees is the video-state grid spacing.
P64-R1 uses all 16 phase tokens, permits sync-to-porch escape, and emits the
committed phase advance instead of a clamp on an uncommitted prediction.
"""
import json
import math
import re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _nibble(word, positions):
    value = sum(((word >> position) & 1) << bit for bit, position in enumerate(positions))
    return (value - 16 if value > 7 else value) + 0.5


def build_lut64_from_params(params):
    """Builds a 1024-entry 16-bit LUT for the 64-state BitScrambler datapath:
    - 64 states x 16 tokens = 1024 words
    - Bundle 0 token: bits 12..15
    - State: bits 6..11
    - DAC code: bits 0..5
    """
    n_sync = int(params['n_sync'])
    n_vid = 64 - n_sync
    T = 16 # 16 tokens total
    TA = int(params.get('token_sectors', 12))
    if TA not in (12, 16) or (TA == 16 and params['r_uncertainty'] > 0):
        raise ValueError('16 phase tokens leave no separate erasure tokens')
    bits = (4, 5, 6, 7, 0, 1, 2, 3, 15, 11)
    lo_step = 2 * math.pi * (-6.0e6) * 50e-9
    hi_step = 2 * math.pi * (+9.0e6) * 50e-9

    rad_a = np.zeros(1024)
    ang_a = np.zeros(1024)
    ang_b = np.zeros(1024)
    for a in range(1024):
        word = 0
        for j in range(10):
            word |= ((a >> j) & 1) << int(bits[j])
        ia = _nibble(word, (4, 5, 6, 7))
        qa = _nibble(word, (0, 1, 2, 3))
        rad_a[a] = math.hypot(ia, qa)
        ang_a[a] = math.atan2(qa, ia)
        ib_neg = bool((word >> 15) & 1)
        qb_neg = bool((word >> 11) & 1)
        if not ib_neg and not qb_neg: ab = 0.25 * math.pi
        elif ib_neg and not qb_neg: ab = 0.75 * math.pi
        elif ib_neg and qb_neg: ab = -0.75 * math.pi
        else: ab = -0.25 * math.pi
        ang_b[a] = ab

    r_unc = params['r_uncertainty']
    w_unc = params['w_uncertainty']
    tok = np.zeros(1024, dtype=int)
    for a in range(1024):
        r = rad_a[a]
        if r < r_unc:
            quad = int(math.floor((ang_b[a] + math.pi) / (0.5 * math.pi))) % 4
            tok[a] = TA + quad
        else:
            tok[a] = int(math.floor((ang_a[a] + math.pi) * TA / (2 * math.pi))) % TA

    obs_angles = np.zeros(T)
    obs_weights = np.ones(T)
    for t in range(TA):
        obs_angles[t] = (t + 0.5) * 2 * math.pi / TA - math.pi
    for q in range(T - TA):
        obs_angles[TA + q] = (q + 0.5) * 0.5 * math.pi - math.pi
        obs_weights[TA + q] = w_unc

    kp_sync = params['kp_sync']
    kp_vid = params['kp_vid']
    v_sync = params['v_sync_prior']
    v_vid = params['v_video_prior']
    err_pos = params['err_pos_bound']
    err_neg = params['err_neg_bound']
    err_sc = params['err_scale']
    w_suppress = params['w_suppress_thresh']
    quant_off = params['code_quant_offset']
    boost = params['video_gain_boost']
    min_vid_c = int(params['min_video_code'])
    max_sync_c = int(params['max_sync_code'])
    sync_w_th = params['sync_weight_thresh']
    vid_w_th = params['vid_weight_thresh']
    cur_exit_th = params['exit_sync_th']
    cur_enter_th = params['enter_sync_th']

    ph_sync = (np.arange(n_sync) + 0.5) * 2 * math.pi / n_sync - math.pi
    ph_vid = (np.arange(n_vid) + 0.5) * 2 * math.pi / n_vid - math.pi
    lut = np.zeros(1024, dtype=np.int64)

    for s in range(64):
        is_sync = s < n_sync
        curr_phase = ph_sync[s] if is_sync else ph_vid[s - n_sync]
        for t in range(T):
            o = obs_angles[t]
            w = obs_weights[t]
            err = (o - curr_phase + math.pi) % (2 * math.pi) - math.pi
            if is_sync:
                gain = kp_sync * w
                if (err > err_pos or err < err_neg) and w < w_suppress:
                    err_eff = min(0.06 * math.pi, max(-0.55 * math.pi, err * err_sc))
                else:
                    err_eff = err
                d_theta = gain * err_eff + (1.0 - gain) * v_sync
                next_regime = 1 if (d_theta > cur_exit_th and w > sync_w_th) else 0
            else:
                gain_vid = min(0.98, kp_vid * boost) if w >= 0.90 else kp_vid * w
                d_theta = gain_vid * err + (1.0 - gain_vid) * v_vid
                next_regime = 0 if (d_theta < cur_enter_th and w > vid_w_th) else 1

            d_theta = max(lo_step, min(hi_step, d_theta))
            next_phase = (curr_phase + d_theta + math.pi) % (2 * math.pi) - math.pi
            raw_code = int(np.rint(np.clip((d_theta - lo_step) / (hi_step - lo_step) * 63 + quant_off, 0, 63)))

            if next_regime == 0:
                p_idx = int(math.floor((next_phase + math.pi) * n_sync / (2 * math.pi))) % n_sync
                nxt = p_idx
                code = min(raw_code, max_sync_c)
            else:
                p_idx = int(math.floor((next_phase + math.pi) * n_vid / (2 * math.pi))) % n_vid
                nxt = n_sync + p_idx
                code = max(raw_code, min_vid_c)

            # Table address = (s << 4) | t
            if params.get('phase_consistent_output', False):
                # The recurrent state contains the QUANTIZED phase. Its
                # actual advance, not the uncommitted prediction, is the
                # recovered frequency. Do not force a constant sync code.
                committed = ph_sync[nxt] if nxt < n_sync else ph_vid[nxt - n_sync]
                advance = (committed - curr_phase + math.pi) % (2 * math.pi) - math.pi
                code = int(np.rint(np.clip((advance - lo_step) / (hi_step - lo_step) * 63, 0, 63)))
            lut[(s << 4) | t] = code | (nxt << 6)
    # Bundle 0 token in bits 12..15
    lut |= tok.astype(np.int64) << 12
    return lut.astype(np.uint16)


def champion_table():
    champion = json.loads((ROOT / 'research/data/p64_champion.json').read_text())
    return build_lut64_from_params(champion['best_candidate'])


def repaired_table():
    repair = json.loads((ROOT / 'research/data/p64_repair.json').read_text())
    return build_lut64_from_params(repair['params'])


def checked_in_table():
    text = (ROOT / 'main/p64_tables.h').read_text()
    values = text.split('p64_table[1024] = {', 1)[1].split('}', 1)[0]
    return np.array([int(value, 16) for value in re.findall(r'0x[0-9a-fA-F]+', values)], dtype=np.uint16)


if __name__ == '__main__':
    assert np.array_equal(repaired_table(), checked_in_table()), 'P64 header differs from repair parameters'
    print('PASS: P64-R1 header reproduces all 1024 words from repair parameters')
