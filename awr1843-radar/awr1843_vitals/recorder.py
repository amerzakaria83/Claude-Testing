"""CSV log of the vital-sign estimates (one row per update)."""

from __future__ import annotations

import csv
import time
from pathlib import Path

from .processing import VitalsResult

COLUMNS = ["host_time", "radar_time_s", "status", "present", "motion", "distance_m",
           "angle_deg", "signal_quality", "breath_bpm", "breath_confidence",
           "breath_depth_mm", "breath_pause_s", "apnea", "heart_bpm", "heart_confidence"]


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return f"{v:.3f}"
    return v


class VitalsLog:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "w", newline="")
        self._w = csv.writer(self._fh)
        self._w.writerow(COLUMNS)
        self.rows = 0

    def write(self, r: VitalsResult) -> None:
        self._w.writerow([_fmt(v) for v in (
            time.time(), r.time_s, r.status, r.present, r.motion, r.distance_m, r.angle_deg,
            r.signal_quality, r.breath_bpm, r.breath_confidence, r.breath_depth_mm,
            r.apnea_s, r.apnea, r.heart_bpm, r.heart_confidence)])
        self.rows += 1
        if self.rows % 20 == 0:
            self._fh.flush()

    def close(self) -> None:
        self._fh.close()
