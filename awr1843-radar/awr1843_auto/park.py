"""Park Assist: obstacle distance per sector with a parking-sensor style beeper.

The radar is mounted on a bumper looking outwards (front or rear).  Sectors
are lateral strips relative to the car's width; the distance shown is the
nearest *persistent* detection in each strip (seen in >= 2 of the last 3
frames), which rejects single-frame noise.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from awr1843.config import RadarParams
from awr1843.tlv import Frame

SECTORS = ("left", "centre", "right")


@dataclass
class ParkSettings:
    car_width_m: float = 1.8
    side_margin_m: float = 1.0        # how far beyond the car's width to look
    max_range_m: float = 4.0
    stop_m: float = 0.3
    red_m: float = 0.6
    amber_m: float = 1.2
    sound: bool = True


@dataclass
class ParkResult:
    time_s: float = 0.0
    distances: dict[str, float] = field(default_factory=lambda: {s: math.inf for s in SECTORS})
    nearest: float = math.inf
    nearest_sector: str = ""
    closing_speed: float = 0.0        # m/s, positive = getting closer
    zone: str = "clear"               # clear / green / amber / red / stop
    beep_interval_s: float | None = None   # None = silent, 0 = continuous tone
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


class ParkAssist:
    def __init__(self, params: RadarParams, settings: ParkSettings | None = None):
        self.params = params
        self.s = settings or ParkSettings()
        self._hist: deque[dict[str, float]] = deque(maxlen=3)
        self._closing = 0.0

    def zone_for(self, d: float) -> str:
        s = self.s
        if d <= s.stop_m:
            return "stop"
        if d <= s.red_m:
            return "red"
        if d <= s.amber_m:
            return "amber"
        if d <= s.max_range_m:
            return "green"
        return "clear"

    def process(self, frame: Frame) -> ParkResult:
        s = self.s
        p = frame.points
        res = ParkResult(time_s=frame.frame_number * self.params.frame_period_ms / 1000)
        if len(p):
            x, y = p["x"].astype(float), p["y"].astype(float)
            v = p["velocity"].astype(float)
            keep = (y > 0.05) & (y < s.max_range_m) & (np.abs(x) < s.car_width_m / 2 + s.side_margin_m)
            x, y, v = x[keep], y[keep], v[keep]
        else:
            x = y = v = np.zeros(0)
        res.points = np.column_stack([x, y]) if len(x) else np.zeros((0, 2))
        half = s.car_width_m / 2
        strips = {"left": x < -half / 3, "centre": np.abs(x) <= half / 3, "right": x > half / 3}
        now = {k: float(y[m].min()) if m.any() else math.inf for k, m in strips.items()}
        self._hist.append(now)
        for k in SECTORS:
            vals = sorted(h[k] for h in self._hist)
            # second smallest of the last 3 frames: must be seen at least twice
            res.distances[k] = vals[1] if len(vals) >= 2 else vals[0]
        k = min(SECTORS, key=lambda q: res.distances[q])
        res.nearest, res.nearest_sector = res.distances[k], k
        if len(y) and math.isfinite(res.nearest):
            near = y < res.nearest + 0.3
            if near.any():
                self._closing = 0.7 * self._closing + 0.3 * float(-np.median(v[near]))
        res.closing_speed = self._closing
        res.zone = self.zone_for(res.nearest)
        d = res.nearest
        if d <= s.stop_m:
            res.beep_interval_s = 0.0
        elif d <= s.amber_m + 0.3:
            res.beep_interval_s = 0.08 + 0.9 * (d - s.stop_m) / (s.amber_m + 0.3 - s.stop_m)
        return res
