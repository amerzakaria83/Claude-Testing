"""Simulated board with a person sitting in front of it (known ground truth).

The chest moves by breathing (~3 mm amplitude, 12-18 /min) plus heartbeat
(~0.15 mm, 66-78 /min) plus slight body sway.  With ``events=True`` a
150 s cycle also contains a body movement (55-59 s) and a breath hold
(110-124 s) so motion and apnea detection can be seen working.
"""

from __future__ import annotations

import math

import numpy as np

from awr1843.config import DATA_PORT_BYTES_PER_SEC, SPEED_OF_LIGHT
from awr1843.device import RadarDevice
from awr1843.simulator import SimulatedRadar
from awr1843.tlv import LOG2_Q9_TO_DB, encode_frame


class VitalsSimulatedRadar(SimulatedRadar):
    def __init__(self, distance_m: float = 0.8, angle_deg: float = 10.0,
                 events: bool = True, seed: int | None = None, realtime: bool = True,
                 breath_bpm: tuple[float, float] = (12, 18),
                 heart_bpm: tuple[float, float] = (66, 78), person: bool = True):
        super().__init__(seed=seed, realtime=realtime)
        self.person = person
        self.distance_m = distance_m
        self.angle_deg = angle_deg
        self.events = events
        self.breath_range = breath_bpm
        self.heart_range = heart_bpm
        self._breath_phase = 0.0
        self._heart_phase = 0.0
        self._sway = 0.0
        self._chair_phase = self._rng.uniform(0, 2 * np.pi)
        self.truth: dict = {}

    # ------------------------------------------------------- ground truth
    def breath_bpm_at(self, t: float) -> float:
        lo, hi = self.breath_range
        return lo + (hi - lo) * 0.5 * (1 + math.sin(2 * math.pi * t / 150))

    def heart_bpm_at(self, t: float) -> float:
        lo, hi = self.heart_range
        return lo + (hi - lo) * 0.5 * (1 + math.sin(2 * math.pi * t / 90 + 1.0))

    def event_at(self, t: float) -> str:
        if not self.events:
            return ""
        c = t % 150
        if 55 <= c < 59:
            return "motion"
        if 110 <= c < 124:
            return "apnea"
        return ""

    # ------------------------------------------------------------ frames
    def make_frame(self, p) -> bytes:
        rng = self._rng
        t = self._sim_time
        dt = p.frame_period_ms / 1000
        lam = SPEED_OF_LIGHT / (p.start_freq_ghz * 1e9)
        n_rb, n_ant = p.num_range_bins, p.num_virtual_antennas_azimuth
        res = p.range_resolution_m
        ant = np.arange(n_ant)

        event = self.event_at(t)
        fb, fh = self.breath_bpm_at(t) / 60, self.heart_bpm_at(t) / 60
        fh_inst = fh * (1 + 0.03 * rng.standard_normal())       # heart rate variability
        self._breath_phase += 2 * math.pi * fb * dt
        self._heart_phase += 2 * math.pi * fh_inst * dt
        b, h = self._breath_phase, self._heart_phase
        breath = 3.0e-3 * math.sin(b + 0.35 * math.sin(b))       # faster inhale than exhale
        if event == "apnea":
            breath *= 0.02
        heart = 0.15e-3 * (math.sin(h) + 0.5 * math.sin(2 * h + 0.6)
                           + 0.2 * math.sin(3 * h + 1.3))
        self._sway = 0.995 * self._sway + 2e-5 * rng.standard_normal()
        disp = breath + heart + self._sway
        r = self.distance_m
        if event == "motion":
            c = (t % 150) - 55
            disp += 0.04 * math.sin(2 * math.pi * 0.8 * c) * math.sin(math.pi * c / 4)
        self.truth = {"time_s": t, "breath_bpm": None if event == "apnea" else fb * 60,
                      "heart_bpm": fh * 60, "event": event,
                      "displacement_mm": disp * 1000, "distance_m": r}

        az = (rng.normal(0, 25, (n_rb, n_ant)) + 1j * rng.normal(0, 25, (n_rb, n_ant)))

        def add(rng_m, ang_deg, amp, phase):
            steer = np.exp(1j * math.pi * ant * math.sin(math.radians(ang_deg)))
            pos = rng_m / res
            for k in range(int(pos) - 2, int(pos) + 3):
                if 0 <= k < n_rb:
                    az[k] += amp * np.sinc(k - pos) * np.exp(1j * phase) * steer

        if self.person:
            add(r + disp, self.angle_deg, 3500 / r, 4 * math.pi * (r + disp) / lam)  # chest
        add(r, self.angle_deg, 900 / r, self._chair_phase)        # chair / static body parts
        add(2.6, -20, 5000 / 2.6, 1.0)                           # wall
        add(0.5, 35, 1500 / 0.5, 2.0)                            # desk edge
        az = np.clip(az, -30000 - 30000j, 30000 + 30000j)

        g = p.gui
        args = {}
        if g.range_azimuth_heatmap:
            args["azimuth_static"] = az
        if g.log_mag_range:
            mag_db = 20 * np.log10(np.abs(az).sum(axis=1) + 1) + 20
            args["range_profile"] = np.clip(mag_db / LOG2_Q9_TO_DB, 0, 65535)
        if g.noise_profile:
            args["noise_profile"] = np.full(n_rb, 60 / LOG2_Q9_TO_DB)
        if g.detected_objects and self.person:
            x = r * math.sin(math.radians(self.angle_deg))
            y = r * math.cos(math.radians(self.angle_deg))
            v = 2 * math.pi * fb * 3e-3 * math.cos(b)
            args["xyzv"] = np.array([[x, y, 0.1, v]], np.float32)
            if g.detected_objects == 1:
                args["snr_noise_db"] = np.array([[25.0, 55.0]])
        n_bytes = len(encode_frame(self._frame_number, **args))
        if g.stats:
            tx_us = n_bytes / DATA_PORT_BYTES_PER_SEC * 1e6
            proc_us = 900
            margin = max(0, p.frame_period_ms * 1000 - p.active_frame_time_ms * 1000
                         - proc_us - tx_us)
            args["stats"] = (proc_us, int(tx_us), int(margin), 15,
                             int(22 + rng.normal(0, 2)), int(12 + rng.normal(0, 2)))
            base = 41 + 5 * (1 - math.exp(-t / 120))
            args["temperature"] = (0, int(t * 1000), *[int(base + o) for o in
                                                       (0, 0, 1, 1, 3, 3, 3, 1, 2, 2)])
        self._frame_number += 1
        return encode_frame(self._frame_number, **args)


def simulated_vitals_device(**kwargs) -> RadarDevice:
    sim = VitalsSimulatedRadar(**kwargs)
    dev = RadarDevice(sim.cli_port, sim.data_port, "SIM-CLI", "SIM-DATA")
    dev.simulator = sim
    return dev
