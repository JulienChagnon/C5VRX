"""P5-TRACK fast evaluator (C5VRX by Twotoz and the C5VRX contributors).

Numba port of the sim.py scoring chain (goggle sync/H-PLL model, calibration,
luma SINAD) plus extra noise metrics, and a cached scenario bank. The port is
checked against sim.goggle / sim.calibrate by `python p5t_eval.py check`.

Synthetic engineering numbers (sim.py), not goggle acceptance or range.
"""
import os
import sys
import time
from pathlib import Path

import numpy as np
from numba import njit
from scipy import signal as sg

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sim  # noqa: E402

FS = sim.FS
PEAK_LP = sg.butter(1, 2e5, fs=FS, output='sos')
BANK_DIR = HERE / 'data' / 'p5t_bank'
BURN = 3000


# ---------------------------------------------------------------- demods
@njit(cache=True)
def _address(word, bits):
    a = 0
    for j in range(10):
        a |= ((word >> bits[j]) & 1) << j
    return a


@njit(cache=True)
def fst_codes(raw, lut):
    """Plain shared-word LUT16 program (u85q.bsasm layout), == sim._pair_codes."""
    bits = np.array([4, 5, 6, 7, 0, 1, 2, 3, 15, 11], np.int64)
    out = np.zeros(len(raw), np.uint8)
    state = 0
    for k in range(len(raw) // 2 - 1):
        word = np.int64(raw[2 * k]) | (np.int64(raw[2 * k + 1]) << 8)
        token = np.int64(lut[_address(word, bits)]) >> 11
        value = np.int64(lut[(state << 5) + token])
        state = (value >> 6) & 31
        out[2 * k + 2] = value & 63
        out[2 * k + 3] = value & 63
    return out


@njit(cache=True)
def lp_codes(raw, lut):
    """u85q_lp.bsasm program (counter-A IIR, pole 1/2), == u85q_lp.lp_codes."""
    bits = np.array([4, 5, 6, 7, 0, 1, 2, 3, 15, 11], np.int64)
    out = np.zeros(len(raw), np.uint8)
    state = 0
    x = 0
    a = 0
    for k in range(len(raw) // 2 - 4):
        word = np.int64(raw[2 * k]) | (np.int64(raw[2 * k + 1]) << 8)
        y = (a >> 10) & 63
        out[2 * k] = y
        out[2 * k + 1] = y
        address = _address(word, bits)
        a = ((y >> 1) << 10) | address
        token = np.int64(lut[address]) >> 11
        hi = ((a >> 8) + (((x >> 1) << 2) | (state >> 3))) & 255
        a = (hi << 8) | (a & 255)
        value = np.int64(lut[(state << 5) + token])
        x = value & 63
        state = (value >> 6) & 31
    return out


@njit(cache=True)
def phase_ref_codes(raw, nbins, lo_rad, hi_rad):
    """Host reference of an open-loop PhaseN endpoint demod (Phase5 = 32
    bins, Phase8 = 256 bins): sample-A phase quantized to nbins, 50-ns wrapped
    delta, linear onto DAC codes 0..63 over [lo_rad, hi_rad]."""
    out = np.zeros(len(raw), np.uint8)
    prev = 0
    two_pi = 2 * np.pi
    for k in range(len(raw) // 2):
        b = np.int64(raw[2 * k])
        i = ((b >> 4) ^ 8) - 8 + 0.5
        q = ((b & 15) ^ 8) - 8 + 0.5
        p = int(np.floor((np.arctan2(q, i) % two_pi) / two_pi * nbins + 0.5)) % nbins
        d = (p - prev) % nbins
        if d >= nbins // 2:
            d -= nbins
        prev = p
        rad = d * two_pi / nbins
        c = int(np.floor((rad - lo_rad) / (hi_rad - lo_rad) * 63 + 0.5))
        c = 0 if c < 0 else (63 if c > 63 else c)
        out[2 * k] = c
        out[2 * k + 1] = c
    return out


# ---------------------------------------------------------------- scoring
@njit(cache=True)
def _fit(y, truth, region, lag, step, burn):
    n = len(y)
    sa = st = saa = sat = stt = 0.0
    cnt = 0
    k = burn
    while k < n - burn:
        if region[k] != 3:
            a = y[k + lag]
            t = truth[k]
            sa += a
            st += t
            saa += a * a
            sat += a * t
            stt += t * t
            cnt += 1
        k += step
    den = cnt * saa - sa * sa
    if den <= 0 or cnt == 0:
        return 1e30, 0.0, 0.0
    g = (cnt * sat - sa * st) / den
    off = (st - g * sa) / cnt
    mse = (stt - 2 * g * sat - 2 * off * st + g * g * saa + 2 * g * off * sa + off * off * cnt) / cnt
    return mse, g, off


@njit(cache=True)
def calibrate_nb(y, truth, region, burn):
    best, blag = 1e300, 0
    for lag in range(-40, 41):
        mse, _, _ = _fit(y, truth, region, lag, 16, burn)
        if mse < best:
            best, blag = mse, lag
    _, g, off = _fit(y, truth, region, blag, 1, burn)
    return blag, g, off


@njit(cache=True)
def apply_cal_nb(y, lag, g, off):
    n = len(y)
    out = np.empty(n)
    if lag >= 0:
        for k in range(n - lag):
            out[k] = y[k + lag]
        for k in range(n - lag, n):
            out[k] = y[n - 1]
    else:
        L = -lag
        for k in range(L, n):
            out[k] = y[k - L]
        for k in range(L):
            out[k] = y[0]
    return g * out + off


@njit(cache=True)
def _bsearch(a, v, right):
    lo, hi = 0, len(a)
    while lo < hi:
        mid = (lo + hi) // 2
        if (a[mid] <= v) if right else (a[mid] < v):
            lo = mid + 1
        else:
            hi = mid
    return lo


@njit(cache=True)
def goggle_nb(ys, lp, peak, period, hs0, window_us=1.5, gain=.3, min_width_us=1.5):
    n = len(ys)
    W = int(min_width_us * 40)
    thr = np.empty(n)
    if not peak:
        thr[:] = -20.
    else:
        tipv, blk, rel = -40., 0., 1 / (2e-3 * 40e6)
        for k0 in range(0, n, 40):
            m = lp[k0]
            for k in range(k0, min(k0 + 40, n)):
                if lp[k] < m:
                    m = lp[k]
            tipv = m if m < tipv else tipv + (blk - 40. - tipv) * 40 * rel
            for k in range(k0, min(k0 + 40, n)):
                thr[k] = (tipv + blk) / 2
    cs = np.zeros(n + 1, np.int64)
    for k in range(n):
        cs[k + 1] = cs[k] + (1 if ys[k] < thr[k] else 0)
    cands = np.empty(n // W + 2)
    nc = 0
    last = -10 ** 9
    for f in range(1, n):
        if ys[f] < thr[f] and not (ys[f - 1] < thr[f - 1]):
            if f + W >= n or f - last < W:
                continue
            if cs[f + W] - cs[f] >= .8 * W:
                cands[nc] = f
                nc += 1
                last = f
    cands = cands[:nc]
    win = window_us * 40
    locked = False
    hits = 0
    hist = np.zeros(32, np.int64)
    hlen = 0
    hpos = 0
    hsum = 0
    tp = cands[0] if nc else 0.
    maxl = len(cands) + int(n / period) + 128
    ls = np.empty(maxl)
    lk = np.zeros(maxl, np.bool_)
    nl = 0
    while tp < n - period:
        lo = _bsearch(cands, tp - win, False)
        hi = _bsearch(cands, tp + win, True)
        if hi > lo:
            bi = lo
            bd = abs(cands[lo] - tp)
            for j in range(lo + 1, hi):
                if abs(cands[j] - tp) < bd:
                    bd = abs(cands[j] - tp)
                    bi = j
            c = cands[bi]
            tp += gain * (c - tp) if locked else (c - tp)
            hits += 1
            v = 1
        else:
            v = 0
            hits = 0
            if not locked:
                nxt = _bsearch(cands, tp + win, True)
                if nxt < nc and cands[nxt] < tp + period - win:
                    tp = cands[nxt] - period
        if hlen == 32:
            hsum -= hist[hpos]
        else:
            hlen += 1
        hist[hpos] = v
        hsum += v
        hpos = (hpos + 1) % 32
        if not locked and hits >= 8:
            locked = True
        elif locked and hlen == 32 and hsum < 16:
            locked = False
        ls[nl] = tp
        lk[nl] = locked
        nl += 1
        tp += period
    if hs0 < 0 or nl == 0:
        return 0.
    ls = ls[:nl]
    lk = lk[:nl]
    dd = (ls - hs0 % period + period / 2) % period - period / 2
    if lk.any():
        sr = 0.
        si = 0.
        for k in range(nl):
            if lk[k]:
                sr += np.cos(2 * np.pi * dd[k] / period)
                si += np.sin(2 * np.pi * dd[k] / period)
        ang = np.arctan2(si, sr) * period / (2 * np.pi)
        dd = (dd - ang + period / 2) % period - period / 2
    if nl <= 40:
        return 0.
    ok = 0
    for k in range(40, nl):
        if lk[k] and abs(dd[k]) < 20:
            ok += 1
    return ok / (nl - 40)


def hs0_of(truth):
    below_t = truth < -20
    d = np.diff(np.r_[False, below_t, False].astype(np.int8))
    s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    hs = s[((e - s) > 140) & ((e - s) < 320)]
    return float(hs[0]) if len(hs) else -1.0


@njit(cache=True)
def _err_stats(yi, truth, region, burn, period):
    """Luma error power, sparkle rate (|err| > 25 IRE) and streak index
    (variance of per-line mean error in picture)."""
    n = len(yi)
    se = 0.
    st = 0.
    st2 = 0.
    cnt = 0
    spark = 0
    nlines = int(n / period) + 2
    lsum = np.zeros(nlines)
    lcnt = np.zeros(nlines)
    for k in range(burn, n - burn):
        if region[k] == 1:
            e = yi[k] - truth[k]
            se += e * e
            st += truth[k]
            st2 += truth[k] * truth[k]
            cnt += 1
            if abs(e) > 25.:
                spark += 1
            li = int(k / period)
            lsum[li] += e
            lcnt[li] += 1
    if cnt == 0:
        return -99., 1., 99.
    var_t = st2 / cnt - (st / cnt) ** 2
    sinad = 10 * np.log10(var_t / max(se / cnt, 1e-12))
    m = 0.
    m2 = 0.
    nl = 0
    for li in range(nlines):
        if lcnt[li] > 100:
            v = lsum[li] / lcnt[li]
            m += v
            m2 += v * v
            nl += 1
    streak = np.sqrt(max(m2 / max(nl, 1) - (m / max(nl, 1)) ** 2, 0.))
    return sinad, spark / cnt, streak


def score_codes(case, codes, clean_codes, ref_gain=None):
    """Returns dict(lock, sinad, spark, streak, gain). `gain` is the clean
    calibration gain (IRE per volt) used for the level-compliance check."""
    y = sg.sosfilt(sim.GOGGLE, sim.DAC_VOLTS[codes])
    yc = sg.sosfilt(sim.GOGGLE, sim.DAC_VOLTS[clean_codes])
    truth, region = case['truth'], case['region']
    lag, g, off = calibrate_nb(yc, truth, region, BURN)
    yi = apply_cal_nb(y, lag, g, off)
    period = 2560. if case['standard'] == 'PAL' else 40e6 / 15734.264
    ys = sg.sosfilt(sim.SYNC_LP, yi)
    lp = sg.sosfilt(PEAK_LP, ys)
    hs0 = case['hs0']
    lock = min(goggle_nb(ys, lp, False, period, hs0), goggle_nb(ys, lp, True, period, hs0))
    sinad, spark, streak = _err_stats(yi, truth, region, BURN, period)
    return dict(lock=float(lock), sinad=float(sinad), spark=float(spark), streak=float(streak), gain=float(g))


# ---------------------------------------------------------------- bank
def case_key(seed, cnr, std, dev, adc_gain, offset_hz, filter_hz):
    return f'{seed}_{cnr:+.1f}_{std}_{dev:.2f}_{adc_gain:.2f}_{offset_hz / 1e3:+.0f}_{(filter_hz or 0) / 1e6:.0f}'


def make_bank_case(seed, cnr, std, dev, adc_gain=2.0, offset_hz=0.0, filter_hz=12e6):
    BANK_DIR.mkdir(parents=True, exist_ok=True)
    key = case_key(seed, cnr, std, dev, adc_gain, offset_hz, filter_hz)
    path = BANK_DIR / f'{key}.npz'
    if path.exists():
        z = np.load(path)
        c = {k: z[k] for k in ('raw', 'clean', 'truth', 'region')}
    else:
        c = sim.make_case(seed, cnr, std, dev, filter_hz=filter_hz, offset_hz=offset_hz, fields=1,
                          physical_adc=True, adc_gain=adc_gain)
        c = {k: c[k] for k in ('raw', 'clean', 'truth', 'region')}
        np.savez(path, **c)
    c['truth'] = c['truth'].astype(np.float64)
    c['standard'] = std
    c['cnr'] = cnr
    c['deviation'] = dev
    c['key'] = key
    c['hs0'] = hs0_of(c['truth'])
    return c


def check(n_cases=4):
    import u85q_lp
    lut = sim.u85q_lut()
    worst = 0.
    for k, (cnr, std) in enumerate(((-1, 'PAL'), (0, 'NTSC'), (3, 'PAL'), (8, 'NTSC'))[:n_cases]):
        c = sim.make_case(500 + k, cnr, std, 1.0, filter_hz=12e6, fields=1, physical_adc=True, adc_gain=2.0)
        a = fst_codes(c['raw'], lut)
        assert np.array_equal(a, sim.u85q_codes(c['raw'])), 'fst_codes != sim'
        b = lp_codes(c['raw'], lut)
        assert np.array_equal(b, u85q_lp.lp_codes(c['raw'], lut)), 'lp_codes != u85q_lp'
        t0 = time.time()
        ref = sim.measure(c, a, fst_codes(c['clean'], lut))
        t1 = time.time()
        cc = dict(c, hs0=hs0_of(c['truth']))
        got = score_codes(cc, a, fst_codes(c['clean'], lut))
        t2 = time.time()
        print(f"C/N {cnr:+} {std}: ref lock {ref['picture']:.4f} sinad {ref['sinad']:.3f} ({t1 - t0:.2f}s) | "
              f"nb lock {got['lock']:.4f} sinad {got['sinad']:.3f} spark {got['spark']:.4f} ({t2 - t1:.3f}s)")
        worst = max(worst, abs(ref['picture'] - got['lock']), abs(ref['sinad'] - got['sinad']))
    print('max abs diff', worst)
    return worst < 1e-6


if __name__ == '__main__':
    sys.exit(0 if check() else 1)
