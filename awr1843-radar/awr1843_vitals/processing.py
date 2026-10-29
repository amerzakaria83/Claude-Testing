"""Vital-sign extraction from the AWR1843 out-of-box demo data.

Principle
---------
The demo's AZIMUTH_STATIC_HEATMAP TLV contains, for every range bin and every
azimuth virtual antenna, the complex zero-Doppler value of the range FFT.  A
person's chest sits in one (range, angle) cell; the *phase* of that cell
changes by ``4 * pi * d / wavelength`` when the chest moves by ``d``.  With a
3.9 mm wavelength, a 0.1 mm heartbeat movement is a ~18 degree phase change,
which is easily measurable.  Sampling that phase every frame gives a chest
displacement signal from which breathing (0.1-0.6 Hz) and heartbeat
(0.8-2.5 Hz) are separated with band-pass filters and spectral analysis.

Processing chain (``VitalSignsProcessor.update``):

1. Beamform each frame over the 8 azimuth antennas (FFT -> angle bins).
2. Find the person: the (range, angle) cell inside the search window with the
   largest *temporal variance* (a breathing chest moves; walls and furniture
   do not).  Lock on it with hysteresis.
3. Remove the static reflection that shares the cell (circle fit of the
   complex samples) and unwrap the phase -> displacement in mm.
4. Detect large body movements; estimates use only the still segment since.
5. Breathing: band-pass 0.1-0.6 Hz, FFT peak (rate), peak-to-trough (depth),
   breath-by-breath peaks and apnea detection.
6. Heartbeat: band-pass 0.8-2.5 Hz on the chest *velocity*, suppress the
   harmonics of the breathing rate, harmonic-sum FFT peak, median tracking.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage, signal

from awr1843.config import SPEED_OF_LIGHT, RadarParams
from awr1843.tlv import Frame


# --------------------------------------------------------------------- types
@dataclass
class VitalsSettings:
    range_min_m: float = 0.3
    range_max_m: float = 1.5
    angle_max_deg: float = 60.0
    num_angle_bins: int = 32
    history_s: float = 30.0          # signal kept for display / breathing
    breath_window_s: float = 25.0
    heart_window_s: float = 12.0
    min_breath_s: float = 10.0       # still time needed before reporting
    min_heart_s: float = 8.0
    breath_band_hz: tuple[float, float] = (0.1, 0.6)    # 6 - 36 per minute
    heart_band_hz: tuple[float, float] = (0.8, 2.5)     # 48 - 150 per minute
    motion_threshold_mm: float = 8.0  # displacement change within 0.5 s
    presence_ratio: float = 6.0       # target variance vs. background
    apnea_s: float = 10.0
    dc_removal: bool = True


@dataclass
class VitalsResult:
    time_s: float = 0.0
    status: str = "Waiting for data"
    present: bool = False
    motion: bool = False
    still_for_s: float = 0.0
    warmup_left_s: float = 0.0
    distance_m: float | None = None
    angle_deg: float | None = None
    signal_quality: float = 0.0          # presence ratio (higher is better)
    breath_bpm: float | None = None
    breath_confidence: float = 0.0
    breath_depth_mm: float | None = None
    breath_interval_s: float | None = None
    apnea: bool = False
    apnea_s: float = 0.0
    heart_bpm: float | None = None
    heart_confidence: float = 0.0
    # waveforms for display (time axis relative to "now", seconds <= 0)
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    displacement_mm: np.ndarray = field(default_factory=lambda: np.zeros(0))
    breath_wave: np.ndarray = field(default_factory=lambda: np.zeros(0))
    heart_wave: np.ndarray = field(default_factory=lambda: np.zeros(0))
    breath_peaks_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    motion_mask: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    breath_spectrum: tuple[np.ndarray, np.ndarray] | None = None   # (bpm, power)
    heart_spectrum: tuple[np.ndarray, np.ndarray] | None = None
    range_time: np.ndarray | None = None      # [time, range] dB at locked angle
    range_axis_m: np.ndarray | None = None
    doppler: np.ndarray | None = None         # [velocity, time] dB
    doppler_axis_mm_s: np.ndarray | None = None
    doppler_t: np.ndarray | None = None
    range_angle: np.ndarray | None = None     # [range, angle] variance map (search window)


# ------------------------------------------------------------------ helpers
def fit_circle(z: np.ndarray) -> complex | None:
    """Algebraic (Kasa) circle fit of complex samples; returns the centre."""
    x, y = z.real.astype(float), z.imag.astype(float)
    if len(z) < 10:
        return None
    a = np.column_stack([x, y, np.ones_like(x)])
    b = -(x * x + y * y)
    try:
        (p, q, r), *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = -p / 2, -q / 2
    rad2 = cx * cx + cy * cy - r
    if rad2 <= 0:
        return None
    spread = np.sqrt(np.var(x) + np.var(y))
    # A nearly straight trace (tiny arc) gives an unstable, huge circle.
    if np.sqrt(rad2) > 30 * max(spread, 1e-9):
        return None
    return complex(cx, cy)


def bandpass(x: np.ndarray, fs: float, band: tuple[float, float], order: int = 4) -> np.ndarray:
    lo, hi = band
    hi = min(hi, 0.45 * fs)
    sos = signal.butter(order, [lo, hi], btype="bandpass", fs=fs, output="sos")
    padlen = min(len(x) - 1, 3 * (2 * len(sos) + 1) * 4)
    return signal.sosfiltfilt(sos, x, padlen=padlen)


def spectral_peak(x: np.ndarray, fs: float, band: tuple[float, float],
                  suppress_f0: float | None = None, harmonic_sum: bool = False,
                  nfft: int = 8192):
    """Dominant frequency in ``band``.  Returns (freq_hz, confidence, f, P)."""
    x = np.asarray(x, float)
    x = x - x.mean()
    win = np.hanning(len(x))
    spec = np.abs(np.fft.rfft(x * win, nfft)) ** 2
    f = np.fft.rfftfreq(nfft, 1 / fs)
    if suppress_f0:
        for k in range(2, 12):
            spec[np.abs(f - k * suppress_f0) < 0.06] *= 0.05
    score = spec.copy()
    if harmonic_sum:  # favour the fundamental over its 2nd harmonic
        score += 0.5 * np.interp(2 * f, f, spec, right=0)
    inband = (f >= band[0]) & (f <= band[1])
    if not inband.any() or spec[inband].sum() <= 0:
        return None, 0.0, f[inband], spec[inband]
    idx = np.flatnonzero(inband)
    k = idx[np.argmax(score[idx])]
    fk = f[k]
    if 0 < k < len(spec) - 1:  # parabolic interpolation of the peak
        a, b, c = np.log(spec[k - 1:k + 2] + 1e-30)
        den = a - 2 * b + c
        if den < 0:
            fk = f[k] + 0.5 * (a - c) / den * (f[1] - f[0])
    df = 0.05 if band[1] < 1 else 0.08
    peak = spec[inband & (np.abs(f - fk) <= df)].sum()
    conf = float(peak / spec[inband].sum())
    return float(fk), conf, f[inband], spec[inband]


# ---------------------------------------------------------------- processor
class VitalSignsProcessor:
    def __init__(self, params: RadarParams, settings: VitalsSettings | None = None):
        self.params = params
        self.settings = settings or VitalsSettings()
        self.fs = params.frame_rate_hz
        self.wavelength_m = SPEED_OF_LIGHT / (params.start_freq_ghz * 1e9)
        self.reset()

    # ------------------------------------------------------------ input
    def reset(self) -> None:
        n = int(round(self.settings.history_s * self.fs)) + 1
        self._frames: deque[np.ndarray] = deque(maxlen=n)
        self._fnums: deque[int] = deque(maxlen=n)
        self._last_fnum: int | None = None
        self._lock: tuple[int, int] | None = None
        self._lock_challenger: tuple[tuple[int, int], int] | None = None
        self._heart_hist: deque[float] = deque(maxlen=7)
        self._heart_pending: deque[float] = deque(maxlen=6)
        self._depth_hist: deque[float] = deque(maxlen=30)
        self.missing_heatmap = False
        self.frames_seen = 0

    def add_frame(self, frame: Frame) -> None:
        az = frame.azimuth_static
        if az is None:
            self.missing_heatmap = True
            return
        self.missing_heatmap = False
        self.frames_seen += 1
        fn = frame.frame_number
        if self._last_fnum is not None:
            gap = fn - self._last_fnum
            if gap <= 0 or gap > self.fs * 2 or (self._frames and az.shape != self._frames[-1].shape):
                self.reset()               # sensor restarted / replay looped
            elif gap > 1:                  # fill dropped frames by interpolation
                prev = self._frames[-1]
                for k in range(1, gap):
                    w = k / gap
                    self._frames.append(((1 - w) * prev + w * az).astype(np.complex64))
                    self._fnums.append(self._last_fnum + k)
        self._frames.append(az.astype(np.complex64))
        self._fnums.append(fn)
        self._last_fnum = fn

    # ---------------------------------------------------------- analysis
    @property
    def range_resolution(self) -> float:
        return self.params.range_resolution_m

    def _angles(self) -> np.ndarray:
        n = self.settings.num_angle_bins
        k = np.arange(n) - n // 2
        return np.degrees(np.arcsin(np.clip(2 * k / n, -1, 1)))

    def _beamform(self, cube: np.ndarray) -> np.ndarray:
        """[..., antenna] -> [..., angle] (fftshifted, angle axis = _angles())."""
        spec = np.fft.fft(cube, n=self.settings.num_angle_bins, axis=-1)
        return np.fft.fftshift(spec, axes=-1)

    def _locate(self, cube: np.ndarray):
        """Pick the (range bin, angle bin) of the moving chest."""
        s = self.settings
        n_r = cube.shape[1]
        r0 = max(1, int(np.floor(s.range_min_m / self.range_resolution)))
        r1 = min(n_r - 1, int(np.ceil(s.range_max_m / self.range_resolution)))
        if r1 <= r0:
            return None, 0.0, None
        recent = cube[-int(max(3, 5 * self.fs)):, r0:r1 + 1]
        bf = self._beamform(recent)                         # [t, r, angle]
        var = np.var(bf, axis=0).real                       # temporal variance
        ang_ok = np.abs(self._angles()) <= s.angle_max_deg
        var[:, ~ang_ok] = 0
        flat = np.sort(var[:, ang_ok].ravel())
        background = np.median(flat[: max(1, int(len(flat) * 0.8))]) + 1e-12
        r_i, a_i = np.unravel_index(np.argmax(var), var.shape)
        best = (r0 + r_i, a_i)
        ratio = float(var[r_i, a_i] / background)

        # hysteresis: only move the lock if a cell wins clearly, repeatedly
        if self._lock is not None and r0 <= self._lock[0] <= r1:
            cur = var[self._lock[0] - r0, self._lock[1]]
            near = abs(best[0] - self._lock[0]) <= 1 and abs(best[1] - self._lock[1]) <= 1
            if near or var[r_i, a_i] < 2.0 * cur:
                self._lock_challenger = None
                return self._lock, float(cur / background), var
            if self._lock_challenger and self._lock_challenger[0] == best:
                count = self._lock_challenger[1] + 1
            else:
                count = 1
            self._lock_challenger = (best, count)
            if count < 3:
                return self._lock, float(cur / background), var
        if self._lock is not None and best != self._lock:
            self._heart_hist.clear()       # a different reflector: start tracking afresh
        self._lock = best
        self._lock_challenger = None
        return best, ratio, var

    def _track_heart(self, bpm: float) -> float | None:
        """Median tracker with octave-error correction and jump confirmation."""
        hist = list(self._heart_hist)
        med = float(np.median(hist[-5:])) if len(hist) >= 3 else None
        if med is not None and abs(bpm - med) >= 15:
            if abs(bpm / 2 - med) < 8:          # locked on the 2nd harmonic
                bpm = bpm / 2
            elif abs(bpm * 2 - med) < 12:       # locked on a sub-harmonic
                bpm = bpm * 2
            else:                               # real change? must persist
                self._heart_pending.append(bpm)
                pend = list(self._heart_pending)
                if len(pend) >= 4 and np.ptp(pend[-4:]) < 10:
                    self._heart_hist.clear()
                    self._heart_hist.extend(pend[-4:])
                    self._heart_pending.clear()
                    return float(np.median(pend[-4:]))
                return med
        self._heart_pending.clear()
        self._heart_hist.append(bpm)
        return float(np.median(list(self._heart_hist)[-5:]))

    def update(self) -> VitalsResult:
        s = self.settings
        res = VitalsResult()
        if self.missing_heatmap and not self._frames:
            res.status = ("No range-azimuth heatmap in the data - use a vitals .cfg "
                          "(guiMonitor rangeAzimuthHeatMap = 1, clutterRemoval off)")
            return res
        n = len(self._frames)
        fs = self.fs
        if n < max(10, int(3 * fs)):
            res.status = "Collecting data..."
            res.warmup_left_s = s.min_breath_s
            return res

        cube = np.stack(self._frames)                       # [t, range, antenna]
        res.time_s = self._fnums[-1] / fs
        lock, ratio, var = self._locate(cube)
        res.range_angle = var
        res.signal_quality = ratio
        if lock is None:
            res.status = "Range window outside of the radar's range"
            return res
        r_bin, a_bin = lock
        res.distance_m = r_bin * self.range_resolution
        res.angle_deg = float(self._angles()[a_bin])
        res.present = ratio >= s.presence_ratio

        # ---------------------------------------------- chest signal
        steer = np.exp(-2j * np.pi * (a_bin - s.num_angle_bins // 2)
                       * np.arange(cube.shape[2]) / s.num_angle_bins)
        z = cube[:, r_bin, :] @ steer                       # beamformed slow-time signal
        t = (np.arange(n) - (n - 1)) / fs                   # seconds, 0 = now
        res.t = t

        # range-time map at the locked angle (display)
        rmax = min(cube.shape[1], int((s.range_max_m + 0.5) / self.range_resolution) + 1)
        rt = np.abs(cube[:, :rmax, :] @ steer)
        res.range_time = 20 * np.log10(rt + 1)
        res.range_axis_m = np.arange(rmax) * self.range_resolution

        centre = fit_circle(z) if s.dc_removal else None
        zc = z - (centre if centre is not None else z.mean())
        phase = np.unwrap(np.angle(zc))
        disp = phase * self.wavelength_m / (4 * np.pi) * 1000.0     # mm
        disp = disp - np.median(disp)
        res.displacement_mm = disp

        # micro-Doppler spectrogram of the chest cell
        nper = int(min(len(zc), max(16, 2 ** np.ceil(np.log2(2.5 * fs)))))
        f_sp, t_sp, sxx = signal.stft(zc, fs=fs, nperseg=nper, noverlap=nper - max(1, nper // 8),
                                      return_onesided=False, boundary=None, padded=False)
        sxx = np.fft.fftshift(np.abs(sxx) ** 2, axes=0)
        f_sp = np.fft.fftshift(f_sp)
        res.doppler = 10 * np.log10(sxx + 1e-9)
        res.doppler_axis_mm_s = f_sp * self.wavelength_m / 2 * 1000
        res.doppler_t = t_sp - t_sp[-1] if len(t_sp) else t_sp

        # ---------------------------------------------- motion
        # (a) slow large movements show up as big displacement changes;
        # (b) fast movements alias the phase (> lambda/4 per frame), which shows
        #     up as frequent near-pi phase steps; breathing at 15-20 fps stays well
        #     below that (<= ~1.3 rad per frame).
        lag = max(1, int(round(0.5 * fs)))
        jump = np.zeros(n)
        jump[lag:] = np.abs(disp[lag:] - disp[:-lag])
        motion = jump > s.motion_threshold_mm
        # At low frame rates (8 fps) fast deep breathing can alias as well; the
        # phase is then unreliable, so treating it as motion is the right call.
        step = np.abs(np.angle(zc[1:] * np.conj(zc[:-1])))
        wild = np.concatenate([[0.0], (step > 0.75 * np.pi).astype(float)])
        win = max(2, int(round(fs)))
        motion |= np.convolve(wild, np.ones(win), "same") >= 3
        # widen motion events by 1 s on both sides
        k = int(fs)
        motion = np.convolve(motion.astype(float), np.ones(2 * k + 1), "same") > 0
        res.motion_mask = motion
        res.motion = bool(motion[-int(max(1, fs)):].any())
        still = np.flatnonzero(motion)
        start = still[-1] + 1 if len(still) else 0
        res.still_for_s = (n - start) / fs

        if not res.present:
            res.status = "No person detected in the search window"
            return res
        if res.motion:
            res.status = "Motion detected - please sit still"
            return res
        if res.still_for_s < s.min_breath_s:
            res.warmup_left_s = s.min_breath_s - res.still_for_s
            res.status = f"Measuring... {res.warmup_left_s:.0f} s"

        # ---------------------------------------------- breathing
        seg = disp[start:]
        tseg = t[start:]
        if len(seg) > 3 * fs:
            bw = bandpass(seg, fs, s.breath_band_hz)
            res.breath_wave = np.full(n, np.nan)
            res.breath_wave[start:] = bw
            # Breathing pauses: trailing stretch where the local peak-to-peak
            # (5 s window) is far below this person's typical breath depth.
            typical = float(np.median(self._depth_hist)) if self._depth_hist else None
            rate_start = 0
            if typical:
                k = int(2.5 * fs)
                env = (ndimage.maximum_filter1d(bw, 2 * k + 1)
                       - ndimage.minimum_filter1d(bw, 2 * k + 1))
                low = env < max(0.3, 0.25 * typical)
                edges = np.flatnonzero(np.diff(np.concatenate([[0], low.astype(int), [0]])))
                for a, b in zip(edges[::2], edges[1::2]):
                    if b == len(low):
                        res.apnea_s = (b - a) / fs          # ongoing pause
                    elif b - a >= 5 * fs:
                        rate_start = b                      # rate restarts after a pause
                res.apnea = res.apnea_s >= s.apnea_s
            if res.still_for_s >= s.min_breath_s and res.apnea_s < 6:
                win = int(min(len(bw) - rate_start, s.breath_window_s * fs))
                if win >= 8 * fs:
                    f_b, conf_b, f_ax, p_ax = spectral_peak(bw[-win:], fs, s.breath_band_hz)
                    res.breath_spectrum = (f_ax * 60, p_ax / (p_ax.max() + 1e-30))
                    if f_b:
                        res.breath_bpm = f_b * 60
                        res.breath_confidence = conf_b
                # breath-by-breath: peaks (exhale/inhale extremes) and depth
                ref = bw[-int(min(len(bw), s.breath_window_s * fs)):]
                dist = max(1, int(fs * 60 / 40))
                prom = 0.25 * np.std(ref)
                pk, _ = signal.find_peaks(bw, distance=dist, prominence=prom)
                tr, _ = signal.find_peaks(-bw, distance=dist, prominence=prom)
                res.breath_peaks_t = tseg[pk]
                if len(pk) >= 2:
                    res.breath_interval_s = float(np.diff(tseg[pk])[-1])
                if len(pk) and len(tr):
                    depths = [bw[p] - bw[tr[tr < p][-1]] for p in pk[-4:] if (tr < p).any()]
                    if depths:
                        res.breath_depth_mm = float(np.median(depths))
                        self._depth_hist.append(res.breath_depth_mm)

        # ---------------------------------------------- heartbeat
        if len(seg) > 3 * fs:
            hw = bandpass(seg, fs, s.heart_band_hz)
            res.heart_wave = np.full(n, np.nan)
            res.heart_wave[start:] = hw
            if res.still_for_s >= s.min_heart_s:
                win = int(min(len(seg), s.heart_window_s * fs))
                vel = np.gradient(seg[-win:]) * fs            # chest velocity: boosts heart vs breath
                vel = bandpass(vel, fs, s.heart_band_hz)
                f0 = res.breath_bpm / 60 if res.breath_bpm else None
                f_h, conf_h, f_ax, p_ax = spectral_peak(vel, fs, s.heart_band_hz,
                                                        suppress_f0=f0, harmonic_sum=True)
                res.heart_spectrum = (f_ax * 60, p_ax / (p_ax.max() + 1e-30))
                if f_h:
                    res.heart_bpm = self._track_heart(f_h * 60)
                    res.heart_confidence = conf_h

        if res.apnea:
            res.status = f"No breathing detected for {res.apnea_s:.0f} s"
        elif res.apnea_s >= 6:
            res.status = f"Breathing paused ({res.apnea_s:.0f} s)"
        elif res.still_for_s >= s.min_breath_s:
            res.status = "Measuring"
        return res
